/* pru1_lateral/main.c — PRU1 lateral step generator + home sensor  (Klipper-inspired).
 *
 * Architecture:
 *   The Linux host pre-computes step moves as (interval, count, add) triples
 *   and pushes them into the lateral move ring in PRU Shared RAM.
 *   This PRU consumes the ring using the 200 MHz IEP counter (owned + init by PRU0).
 *   No software ramp computation here.
 *
 * Additional responsibilities vs PRU0:
 *   - Sample dual-contact home sensor (NO + NC) with IEP-timer debounce.
 *   - CMD_HOME_START: enter homing mode; stop + reset position when HOME_HIT.
 *   - Track signed position in steps (for overrun protection + telemetry).
 *
 * GPIO (pinmux mode 6 = PRU output, R30; mode 6 input = R31):
 *   R30[0]  P8_45   LATERAL_STEP
 *   R30[1]  P8_46   LATERAL_DIR
 *   R30[2]  P8_43   LATERAL_ENABLE   (active-low: 0 = driver ON)
 *   R31[3]  P8_44   HOME_NO          (normally-open;   LOW when at home)
 *   R31[4]  P8_41   HOME_NC          (normally-closed; HIGH when at home)
 *
 * Sensor logic:
 *   Away from home : NO=HIGH (open),  NC=LOW  (closed)  → normal travel
 *   At home        : NO=LOW  (closed), NC=HIGH (open)   → HOME_HIT
 *   Fault          : NO=HIGH, NC=HIGH                   → cable fault
 *
 * PRU1 does NOT call IEP_INIT(). PRU0 owns the IEP counter; PRU1 reads IEP_NOW().
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_stepper.h"
#include "../include/pru_tmcuart.h"

/* ── PRU register aliases (shared) ──────────────────────────────────────────*/
#include "../include/pru_regs.h"

/* ── GPIO bit masks ──────────────────────────────────────────────────────────*/
#define LAT_STEP_BIT   (1u << 0)
#define LAT_DIR_BIT    (1u << 1)
#define LAT_EN_BIT     (1u << 2)   /* active-low */
#define HOME_NO_BIT    (1u << 3)   /* R31: LOW when at home */
#define HOME_NC_BIT    (1u << 4)   /* R31: HIGH when at home */

/* ── Throttle constants (shared) ─────────────────────────────────────────── */
#include "../include/pru_throttle.h"

/* IEP-based debounce window: 100 000 cycles = 500 µs */
#define DEBOUNCE_CYCLES    100000u

/* Software position limits (steps from home, lateral only) */
#define POS_LIMIT_MIN      (-10000)
#define POS_LIMIT_MAX      (614400)  /* ~200 mm at 3072 steps/mm */

/* ── Shared RAM pointers ─────────────────────────────────────────────────────*/
static volatile pru_cmd_t        *cmd_ring   =
    (volatile pru_cmd_t *)       (PRU_SRAM_PHYS_BASE + IPC_CMD_RING_OFFSET);
static volatile uint32_t         *cmd_whead  =
    (volatile uint32_t *)        (PRU_SRAM_PHYS_BASE + IPC_CMD_WHEAD_OFFSET);
static volatile uint32_t         *cmd_rhead  =
    (volatile uint32_t *)        (PRU_SRAM_PHYS_BASE + IPC_CMD_RHEAD_OFFSET);
static volatile pru_axis_telem_t *telem      =
    (volatile pru_axis_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_LATERAL_TELEM_OFFSET);
static volatile pru_sync_t       *pru_sync   =
    (volatile pru_sync_t *)      (PRU_SRAM_PHYS_BASE + IPC_SYNC_OFFSET);

static volatile pru_move_t  *lat_move_ring  =
    (volatile pru_move_t *)  (PRU_SRAM_PHYS_BASE + IPC_LAT_MOVE_RING_OFFSET);
static volatile uint32_t    *lat_move_whead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_LAT_MOVE_WHEAD_OFFSET);
static volatile uint32_t    *lat_move_rhead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_LAT_MOVE_RHEAD_OFFSET);

/* ── Module state ────────────────────────────────────────────────────────────*/
static stepper_t   lateral     = {0};
static move_ring_t g_mr;                 /* initialised in main() */
static uint8_t     homing_mode = 0u;     /* set by CMD_HOME_START */

/* -------------------------------------------------------------------------
 * Minimal PRU-side TMC UART implementation (adapted from src_k/tmcuart.c)
 * - Lightweight mailbox in shared RAM (REQ / RESP rings) defined in
 *   `pru_ipc.h` (IPC_TMCUART_*). Host pushes requests, PRU consumes and
 *   writes responses. This implementation currently assumes separate
 *   TX/RX pins (no single-wire direction switching).
 */

/* Default PRU pins used for TMC UART (adjust in board/device trees as needed) */
#define TMC_TX_BIT   (1u << 5)  /* R30 bit for TX */
#define TMC_RX_BIT   (1u << 6)  /* R31 bit for RX */

enum {
    TU_LINE_HIGH = 1<<0, TU_ACTIVE = 1<<1, TU_READ_SYNC = 1<<2,
    TU_REPORT = 1<<3, TU_PULLUP = 1<<4, TU_SINGLE_WIRE = 1<<5
};

/* Ring pointers into PRU shared RAM */
static volatile pru_tmcuart_msg_t *tmc_req_ring =
    (volatile pru_tmcuart_msg_t *)(PRU_SRAM_PHYS_BASE + IPC_TMCUART_REQ_RING_OFFSET);
static volatile uint32_t *tmc_req_whead =
    (volatile uint32_t *)(PRU_SRAM_PHYS_BASE + IPC_TMCUART_REQ_WHEAD_OFFSET);
static volatile uint32_t *tmc_req_rhead =
    (volatile uint32_t *)(PRU_SRAM_PHYS_BASE + IPC_TMCUART_REQ_RHEAD_OFFSET);

static volatile pru_tmcuart_msg_t *tmc_resp_ring =
    (volatile pru_tmcuart_msg_t *)(PRU_SRAM_PHYS_BASE + IPC_TMCUART_RESP_RING_OFFSET);
static volatile uint32_t *tmc_resp_whead =
    (volatile uint32_t *)(PRU_SRAM_PHYS_BASE + IPC_TMCUART_RESP_WHEAD_OFFSET);
static volatile uint32_t *tmc_resp_rhead =
    (volatile uint32_t *)(PRU_SRAM_PHYS_BASE + IPC_TMCUART_RESP_RHEAD_OFFSET);

/* Local PRU state for the tmcuart state machine */
static uint32_t tmc_req_local_rhead = 0u;
static uint8_t  tmc_flags = TU_LINE_HIGH;
static uint8_t  tmc_pos = 0u, tmc_read_count = 0u, tmc_write_count = 0u;
static uint32_t tmc_cfg_bit_time = (PRU_CLOCK_HZ / 115200u); /* default ~115200 baud */
static uint32_t tmc_bit_time = 0u;
static uint8_t  tmc_data[10];
static uint32_t tmc_timer_waketime = 0u;
static uint8_t  tmc_mode = 0u;
static uint8_t  tmc_last_chip = 0u, tmc_last_reg = 0u;

#define TMC_MODE_IDLE        0u
#define TMC_MODE_READ_EVENT  1u
#define TMC_MODE_READ_SYNC   2u
#define TMC_MODE_SEND_EVENT  3u
#define TMC_MODE_SEND_FINISH 4u

/* Reset TX line to idle (logic HIGH) */
static inline void tmcuart_reset_line(void) {
    /* Restore TX to idle. For single-wire boards prefer tri-state hook. */
    if (tmc_flags & TU_SINGLE_WIRE) {
        TMC_SET_TX_TRISTATE();
    } else {
        TMC_SET_TX_OUTPUT(1);
    }
    tmc_flags = (tmc_flags & (TU_PULLUP | TU_SINGLE_WIRE)) | TU_LINE_HIGH;
    tmc_mode = TMC_MODE_IDLE;
}

/* Finalize a transaction: restore line and mark report pending */
static inline void tmcuart_finalize(void) {
    tmcuart_reset_line();
    tmc_flags |= TU_REPORT;
}

/* Event: read one serial bit into tmc_data */
static inline void tmcuart_read_event(void) {
    uint8_t v = (__R31 & TMC_RX_BIT) ? 1u : 0u;
    uint8_t pos = tmc_pos, mask = 1u << (pos & 0x07), data = tmc_data[pos >> 3];
    if (v) data |= mask; else data &= ~mask;
    tmc_data[pos >> 3] = data;
    pos++;
    if (pos >= tmc_read_count) {
        tmcuart_finalize();
        return;
    }
    tmc_pos = pos;
    tmc_timer_waketime += tmc_bit_time;
}

/* Event: finish of write phase; possibly prepare for read */
static inline void tmcuart_send_finish_event(void) {
    if (!tmc_read_count) {
        tmcuart_finalize();
        return;
    }
    /* prepare for rx (leave TX idle, sample RX) */
    if (tmc_flags & TU_SINGLE_WIRE) {
        /* If requested, put RX pin into input mode and enable pull-up */
        TMC_SET_RX_INPUT_PULLUP((tmc_flags & TU_PULLUP) ? 1 : 0);
        /* Make sure TX is released */
        TMC_SET_TX_TRISTATE();
    } else {
        /* Ensure TX idles high for multi-wire */
        TMC_SET_TX_OUTPUT(1);
    }
    tmc_pos = 0u;
    tmc_mode = TMC_MODE_READ_SYNC;
    tmc_timer_waketime += (tmc_bit_time * 4u);
}

/* Event: send bits (toggle TX pin as needed) */
static inline void tmcuart_send_event(void) {
    __R30 ^= TMC_TX_BIT; /* toggle */
    tmc_flags ^= TU_LINE_HIGH;
    uint8_t line_state = (tmc_flags & TU_LINE_HIGH) ? 1u : 0u;
    uint32_t bit_time = tmc_bit_time;
    uint32_t next = bit_time;
    uint8_t pos = tmc_pos;
    for (;;) {
        pos++;
        if (pos >= tmc_write_count) {
            tmc_mode = TMC_MODE_SEND_FINISH;
            tmc_timer_waketime += next;
            return;
        }
        uint8_t data = tmc_data[pos >> 3];
        uint8_t bit = (data >> (pos & 0x07)) & 0x01;
        if (bit != line_state) break;
        next += bit_time;
    }
    tmc_pos = pos;
    tmc_timer_waketime += next;
}

/* Helper: divide by 4 rounding to nearest integer (signed) */
static inline int32_t div_round_closest_4(int32_t x) {
    if (x >= 0) return (x + 2) / 4;
    return (x - 2) / 4;
}

/* Event: send sync nibble with enhanced baud detection (measure offsets)
 * Mirrors logic from src_k/tmcuart.c: toggles 4 sync bits, measures
 * scheduling offsets and computes adjusted bit_time for subsequent send. */
static inline void tmcuart_send_sync_event(void) {
    __R30 ^= TMC_TX_BIT; /* toggle */
    uint32_t cur = IEP_NOW();
    tmc_flags ^= TU_LINE_HIGH;

    /* Advance pos for this sent bit */
    tmc_pos++;
    if (tmc_pos == 1u) {
        uint32_t offset = cur - tmc_timer_waketime;
        tmc_bit_time = offset;
    } else if (tmc_pos >= 5u) {
        uint32_t offset = cur - tmc_timer_waketime;
        uint32_t start_offset = tmc_bit_time;
        int32_t diff = (int32_t)offset - (int32_t)start_offset;
        int32_t adj = div_round_closest_4(diff);
        int32_t new_bt = (int32_t)tmc_cfg_bit_time + adj;
        if (new_bt < 1) new_bt = 1;
        tmc_bit_time = (uint32_t)new_bt;
        tmc_mode = TMC_MODE_SEND_EVENT;
        tmc_timer_waketime += (uint32_t)diff + tmc_bit_time;
        return;
    }
    /* Default inter-sync spacing uses configured nominal bit time */
    tmc_timer_waketime += tmc_cfg_bit_time;
}

/* Event: sync wait for start of incoming bits (simple detection) */
static inline void tmcuart_read_sync_event(void) {
    uint8_t v = (__R31 & TMC_RX_BIT) ? 1u : 0u;
    if (v) {
        tmc_flags |= TU_READ_SYNC;
    } else if (tmc_flags & TU_READ_SYNC) {
        /* Now synchronized - begin reading */
        tmc_pos = 0u;
        tmc_mode = TMC_MODE_READ_EVENT;
        tmcuart_read_event();
        return;
    }
    if (++tmc_pos >= 64u) {
        /* Timeout */
        tmc_read_count = 0u;
        tmcuart_finalize();
        return;
    }
    tmc_timer_waketime += tmc_bit_time;
}

/* Poll the tmcuart state machine and flush responses to shared RAM */
static inline void tmcuart_poll(void) {
    uint32_t now = IEP_NOW();
    if (tmc_mode != TMC_MODE_IDLE && !timer_before(now, tmc_timer_waketime)) {
        switch (tmc_mode) {
        case TMC_MODE_SEND_SYNC:  tmcuart_send_sync_event(); break;
        case TMC_MODE_SEND_EVENT: tmcuart_send_event(); break;
        case TMC_MODE_SEND_FINISH: tmcuart_send_finish_event(); break;
        case TMC_MODE_READ_SYNC: tmcuart_read_sync_event(); break;
        case TMC_MODE_READ_EVENT: tmcuart_read_event(); break;
        default: break;
        }
    }

    /* If response ready, copy into RESP ring for host to pick up */
    if (tmc_flags & TU_REPORT) {
        uint32_t wh = *tmc_resp_whead % IPC_TMCUART_RESP_SLOTS;
        volatile pru_tmcuart_msg_t *dst = &tmc_resp_ring[wh];
        uint8_t out_len = (uint8_t)(tmc_read_count / 8u);
        dst->len = out_len;
        dst->type = 2u;
        dst->chip = tmc_last_chip;
        dst->reg = tmc_last_reg;
        uint8_t i;
        for (i = 0u; i < out_len && i < 8u; ++i) dst->data[i] = tmc_data[i];
        *tmc_resp_whead = (*tmc_resp_whead + 1u) % IPC_TMCUART_RESP_SLOTS;
        tmc_flags &= ~TU_REPORT;
    }
}


/* ── Home sensor state ───────────────────────────────────────────────────────*/
typedef enum { HOME_AWAY = 0, HOME_HIT = 1, HOME_FAULT = 2 } home_state_t;

static home_state_t home_stable  = HOME_AWAY;
static home_state_t home_pending = HOME_AWAY;
static uint32_t     debounce_t0  = 0u;      /* IEP time when pending state began */
static uint8_t      home_valid   = 0u;      /* 1 once first debounce is settled */
static uint8_t      home_hit_latch = 0u;    /* one-shot: HOME_HIT just detected */

static inline home_state_t read_home_raw(void) {
    uint8_t no = (__R31 & HOME_NO_BIT) ? 1u : 0u;
    uint8_t nc = (__R31 & HOME_NC_BIT) ? 1u : 0u;
    if (!no && nc)  return HOME_HIT;
    if (no  && nc)  return HOME_FAULT;   /* both open → disconnected */
    if (!no && !nc) return HOME_FAULT;   /* both closed → short */
    return HOME_AWAY;                    /* no=HIGH, nc=LOW → normal */
}

/* IEP-based debounce: state must be stable for DEBOUNCE_CYCLES (500 µs). */
static void sample_home(void) {
    home_state_t raw = read_home_raw();
    uint32_t now = IEP_NOW();

    if (raw != home_pending) {
        home_pending = raw;
        debounce_t0  = now;
    } else if (!timer_before(now, debounce_t0 + DEBOUNCE_CYCLES)) {
        /* Stable for DEBOUNCE_CYCLES */
        if (raw != home_stable || !home_valid) {
            home_stable = raw;
            home_valid  = 1u;
            if (raw == HOME_HIT)
                home_hit_latch = 1u;
        }
    }
}

/* ── process_command ─────────────────────────────────────────────────────────
 * Returns 1 on CMD_EMERGENCY_STOP.
 */
static int process_command(const volatile pru_cmd_t *c) {
    if (c->axis != AXIS_LATERAL && c->axis != AXIS_ALL) return 0;

    switch (c->cmd) {
    case CMD_ENABLE:
        if (c->value_a)
            __R30 &= ~LAT_EN_BIT;
        else
            __R30 |=  LAT_EN_BIT;
        break;

    case CMD_HOME_START:
        homing_mode = 1u;
        home_stable = HOME_AWAY;
        home_valid  = 0u;
        break;

    case CMD_QUEUE_FLUSH:
        stepper_force_stop(&lateral);
        __R30 &= ~LAT_STEP_BIT;
        stepper_flush_ring(&g_mr);
        homing_mode = 0u;
        break;

    case CMD_RESET_POSITION:
        lateral.step_count    = 0u;
        lateral.position      = 0;
        telem->step_count     = 0u;
        telem->position_steps = 0;
        break;

    case CMD_EMERGENCY_STOP:
        stepper_force_stop(&lateral);
        __R30 &= ~LAT_STEP_BIT;
        __R30 |=  LAT_EN_BIT;
        pru_sync->lateral_interval = 0u;
        homing_mode = 0u;
        return 1;

    default:
        break;
    }
    return 0;
}

/* ── publish_telemetry ───────────────────────────────────────────────────────*/
static void publish_telemetry(void) {
    telem->seq              = telem->seq + 1u;
    telem->step_count       = lateral.step_count;
    telem->current_interval = lateral.interval;
    telem->moves_pending    = stepper_ring_count(&g_mr);
    telem->position_steps   = lateral.position;

    uint16_t st  = 0u;
    uint16_t flt = telem->faults;   /* preserve existing fault bits */

    if (lateral.running)             st |= STATE_RUNNING;
    else                             st |= STATE_IDLE;
    if (homing_mode)                 st |= STATE_HOMING;
    if (home_stable == HOME_HIT)     st |= STATE_AT_HOME;
    if (home_stable == HOME_FAULT) { st |= STATE_FAULT;  flt |= FAULT_HOME_SENSOR; }
    if (lateral.underrun)            flt |= FAULT_MOVE_UNDERRUN;
    if (!(__R30 & LAT_EN_BIT))       st |= STATE_ENABLED;
    telem->state  = st;
    telem->faults = flt;

    pru_sync->lateral_interval = lateral.interval;
}

/* ── main ────────────────────────────────────────────────────────────────────*/
int main(void) {
    __R30 &= ~LAT_STEP_BIT;
    __R30 &= ~LAT_DIR_BIT;
    __R30 |=  LAT_EN_BIT;

    /* PRU1 does NOT call IEP_INIT(). PRU0 already started the counter. */

    g_mr.ring  = lat_move_ring;
    g_mr.whead = lat_move_whead;
    g_mr.rhead = lat_move_rhead;
    g_mr.slots = IPC_LAT_MOVE_SLOTS;

    *lat_move_whead = 0u;
    *lat_move_rhead = 0u;
    *cmd_rhead      = 0u;

    uint32_t cmd_local_rhead = 0u;
    uint32_t cmd_counter     = 0u;
    uint32_t telem_counter   = 0u;

    while (1) {

        /* ── Home sensor sampling (every iteration) ─────────────────────────*/
        sample_home();

        /* When homing and HOME_HIT detected: stop + reset position */
        if (home_hit_latch && homing_mode) {
            stepper_force_stop(&lateral);
            stepper_flush_ring(&g_mr);
            __R30 &= ~LAT_STEP_BIT;
            lateral.step_count  = 0u;
            lateral.position    = 0;
            telem->step_count   = 0u;
            telem->position_steps = 0;
            homing_mode = 0u;
        }
        home_hit_latch = 0u;   /* consume */

        /* Poll PRU-side tmcuart state machine (non-blocking) */
        tmcuart_poll();

        /* ── Step generation (hot path) ─────────────────────────────────────*/
        if (lateral.running) {
            if (!timer_before(IEP_NOW(), lateral.next_edge_time)) {
                if (lateral.dir_pending) {
                    if (lateral.direction) __R30 |=  LAT_DIR_BIT;
                    else                   __R30 &= ~LAT_DIR_BIT;
                    lateral.dir_pending = 0u;
                }
                uint8_t pin = stepper_edge(&lateral, &g_mr);
                if (pin) __R30 |=  LAT_STEP_BIT;
                else     __R30 &= ~LAT_STEP_BIT;

                /* Software position overrun check on each rising edge */
                if (pin) {
                    if (lateral.position < POS_LIMIT_MIN ||
                        lateral.position > POS_LIMIT_MAX) {
                        stepper_force_stop(&lateral);
                        stepper_flush_ring(&g_mr);
                        __R30 &= ~LAT_STEP_BIT;
                        telem->faults |= FAULT_OVERRUN;
                    }
                }
            }
        } else {
            if (stepper_ring_count(&g_mr) > 0u) {
                stepper_start_first(&lateral, &g_mr);
                if (lateral.dir_pending) {
                    if (lateral.direction) __R30 |=  LAT_DIR_BIT;
                    else                   __R30 &= ~LAT_DIR_BIT;
                    lateral.dir_pending = 0u;
                }
            }
        }

        /* ── Command ring (throttled) ───────────────────────────────────────*/
        if (++cmd_counter >= CMD_CHECK_STRIDE) {
            cmd_counter = 0u;
            uint32_t wh = *cmd_whead % IPC_CMD_RING_SLOTS;
            while (cmd_local_rhead != wh) {
                if (process_command(&cmd_ring[cmd_local_rhead])) goto emergency;
                cmd_local_rhead = (cmd_local_rhead + 1u) % IPC_CMD_RING_SLOTS;
                *cmd_rhead = cmd_local_rhead;
            }
            /* Also check TMC UART request ring (throttled here with commands) */
            {
                uint32_t twh = *tmc_req_whead % IPC_TMCUART_REQ_SLOTS;
                while (tmc_req_local_rhead != twh) {
                    volatile pru_tmcuart_msg_t *src = &tmc_req_ring[tmc_req_local_rhead];
                    uint8_t len = src->len;
                    uint8_t type = src->type;
                    uint8_t i;
                    /* Busy? consume and drop (host should retry) */
                    if (tmc_flags & TU_ACTIVE) {
                        tmc_req_local_rhead = (tmc_req_local_rhead + 1u) % IPC_TMCUART_REQ_SLOTS;
                        *tmc_req_rhead = tmc_req_local_rhead;
                        continue;
                    }
                        /* Copy request */
                        tmc_last_chip = src->chip;
                        tmc_last_reg  = src->reg;
                        tmc_write_count = (uint8_t)(len * 8u);
                        tmc_read_count  = (type == 0x01) ? (uint8_t)(len * 8u) : 0u;
                        for (i = 0u; i < len && i < 8u; ++i) tmc_data[i] = src->data[i];

                        /* Arm transaction: map mailbox flags into PRU internal flags */
                        uint8_t req_flags = src->flags;
                        tmc_pos = 0u;
                        /* Preserve LINE_HIGH state, set ACTIVE and map pullup/single-wire */
                        tmc_flags = (tmc_flags & TU_LINE_HIGH) | TU_ACTIVE;
                        if (req_flags & TMC_FLAG_PULLUP)  tmc_flags |= TU_PULLUP;
                        if (req_flags & TMC_FLAG_SINGLE_WIRE) tmc_flags |= TU_SINGLE_WIRE;
                        /* Choose sync mode when first byte indicates sync nibble (0x2a masked) */
                        if (len >= 1 && (tmc_data[0] & 0x3Fu) == 0x2Au) {
                            tmc_bit_time = 0u; /* will be measured by sync event */
                            tmc_timer_waketime = IEP_NOW() + (200u * (PRU_CLOCK_HZ / 1000000u));
                            tmc_pos = 0u; /* sync event will increment before use */
                            tmc_mode = TMC_MODE_SEND_SYNC;
                        } else {
                            tmc_bit_time = tmc_cfg_bit_time;
                            tmc_timer_waketime = IEP_NOW() + (200u * (PRU_CLOCK_HZ / 1000000u));
                            tmc_mode = TMC_MODE_SEND_EVENT;
                        }

                        /* Mark consumed */
                        tmc_req_local_rhead = (tmc_req_local_rhead + 1u) % IPC_TMCUART_REQ_SLOTS;
                        *tmc_req_rhead = tmc_req_local_rhead;
                }
            }
        }

        /* ── Telemetry (throttled) ──────────────────────────────────────────*/
        if (++telem_counter >= TELEM_STRIDE) {
            telem_counter = 0u;
            publish_telemetry();
        }
    }

emergency:
    stepper_force_stop(&lateral);
    __R30 &= ~LAT_STEP_BIT;
    __R30 |=  LAT_EN_BIT;
    homing_mode = 0u;
    publish_telemetry();
    __asm volatile ("HALT");
    return 0;
}

