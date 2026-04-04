/* pru1_dual_stepper/main.c — PRU1 dual-axis step generator (Klipper-inspired).
 *
 * Architecture:
 *   Single PRU1 drives both SPINDLE and LATERAL stepper axes simultaneously.
 *   The Linux host pre-computes step moves as (interval, count, add) triples
 *   and pushes them into per-axis move rings in PRU Shared RAM.
 *   PRU1 owns and initialises the IEP timer (200 MHz free-running counter).
 *   PRU0 is reserved for auxiliary tasks (sensors, UART, I/O) and must NOT
 *   call IEP_INIT().
 *
 * GPIO (pinmux mode 6 = PRU1 R30 output, P8 LCD_DATAx pins):
 *   R30[1]  P8_46   SPINDLE_STEP
 *   R30[3]  P8_44   SPINDLE_DIR
 *   R30[5]  P8_42   SPINDLE_ENABLE   (active-low: 0 = driver ON)
 *   R30[0]  P8_45   LATERAL_STEP
 *   R30[2]  P8_43   LATERAL_DIR
 *   R30[4]  P8_41   LATERAL_ENABLE   (active-low: 0 = driver ON)
 *
 * Pin ownership by R30 bit:
 *   P8_45 = LCD_DATA0 → pr1_pru1_pru_r30_0   (pinctrl offset 0xA0, mode 6)
 *   P8_46 = LCD_DATA1 → pr1_pru1_pru_r30_1   (pinctrl offset 0xA4, mode 6)
 *   P8_43 = LCD_DATA2 → pr1_pru1_pru_r30_2   (pinctrl offset 0xA8, mode 6)
 *   P8_44 = LCD_DATA3 → pr1_pru1_pru_r30_3   (pinctrl offset 0xAC, mode 6)
 *   P8_41 = LCD_DATA4 → pr1_pru1_pru_r30_4   (pinctrl offset 0xB0, mode 6)
 *   P8_42 = LCD_DATA5 → pr1_pru1_pru_r30_5   (pinctrl offset 0xB4, mode 6)
 *
 * IPC command ring:
 *   Commands with axis == AXIS_SPINDLE apply to the spindle axis.
 *   Commands with axis == AXIS_LATERAL apply to the lateral axis.
 *   Commands with axis == AXIS_ALL     apply to both axes.
 *   NOTE: do not load both PRU0 (old spindle fw) and this firmware
 *         simultaneously — they would both advance cmd_rhead.
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_stepper.h"
#include "../include/pru_regs.h"

/* ── GPIO bit masks ──────────────────────────────────────────────────────────*/
#define SP_STEP_BIT    (1u << 1)   /* P8_46 — Spindle STEP  */
#define SP_DIR_BIT     (1u << 3)   /* P8_44 — Spindle DIR   */
#define SP_EN_BIT      (1u << 5)   /* P8_42 — Spindle EN    (active-low) */

#define LAT_STEP_BIT   (1u << 0)   /* P8_45 — Lateral STEP  */
#define LAT_DIR_BIT    (1u << 2)   /* P8_43 — Lateral DIR   */
#define LAT_EN_BIT     (1u << 4)   /* P8_41 — Lateral EN    (active-low) */

/* ── Throttle constants ──────────────────────────────────────────────────────*/
#include "../include/pru_throttle.h"

/* ── Shared RAM pointers ─────────────────────────────────────────────────────*/
static volatile pru_cmd_t        *cmd_ring   =
    (volatile pru_cmd_t *)       (PRU_SRAM_PHYS_BASE + IPC_CMD_RING_OFFSET);
static volatile uint32_t         *cmd_whead  =
    (volatile uint32_t *)        (PRU_SRAM_PHYS_BASE + IPC_CMD_WHEAD_OFFSET);
static volatile uint32_t         *cmd_rhead  =
    (volatile uint32_t *)        (PRU_SRAM_PHYS_BASE + IPC_CMD_RHEAD_OFFSET);

static volatile pru_axis_telem_t *sp_telem   =
    (volatile pru_axis_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_SPINDLE_TELEM_OFFSET);
static volatile pru_axis_telem_t *lat_telem  =
    (volatile pru_axis_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_LATERAL_TELEM_OFFSET);
static volatile pru_sync_t       *pru_sync   =
    (volatile pru_sync_t *)      (PRU_SRAM_PHYS_BASE + IPC_SYNC_OFFSET);

/* Spindle move ring */
static volatile pru_move_t  *sp_move_ring  =
    (volatile pru_move_t *)  (PRU_SRAM_PHYS_BASE + IPC_SP_MOVE_RING_OFFSET);
static volatile uint32_t    *sp_move_whead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_SP_MOVE_WHEAD_OFFSET);
static volatile uint32_t    *sp_move_rhead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_SP_MOVE_RHEAD_OFFSET);

/* Lateral move ring */
static volatile pru_move_t  *lat_move_ring  =
    (volatile pru_move_t *)  (PRU_SRAM_PHYS_BASE + IPC_LAT_MOVE_RING_OFFSET);
static volatile uint32_t    *lat_move_whead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_LAT_MOVE_WHEAD_OFFSET);
static volatile uint32_t    *lat_move_rhead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_LAT_MOVE_RHEAD_OFFSET);

/* ── Module state ────────────────────────────────────────────────────────────*/
static stepper_t   spindle = {0};
static stepper_t   lateral = {0};
static move_ring_t g_mr_sp;    /* spindle move ring context (initialised in main) */
static move_ring_t g_mr_lat;   /* lateral move ring context (initialised in main) */

/* ── apply_dir: set DIR pin for one axis ─────────────────────────────────────*/
static inline void apply_dir_sp(const stepper_t *s) {
    if (s->direction) __R30 |= SP_DIR_BIT; else __R30 &= ~SP_DIR_BIT;
}
static inline void apply_dir_lat(const stepper_t *s) {
    if (s->direction) __R30 |= LAT_DIR_BIT; else __R30 &= ~LAT_DIR_BIT;
}

/* ── spindle_stop: immediate stop, clear step pin ────────────────────────────*/
static void spindle_stop(void) {
    stepper_force_stop(&spindle);
    __R30 &= ~SP_STEP_BIT;
    stepper_flush_ring(&g_mr_sp);
}

/* ── lateral_stop: immediate stop, clear step pin ────────────────────────────*/
static void lateral_stop(void) {
    stepper_force_stop(&lateral);
    __R30 &= ~LAT_STEP_BIT;
    stepper_flush_ring(&g_mr_lat);
}

/* ── process_command: handle one command, return 1 on EMERGENCY_STOP ─────────*/
static int process_command(const volatile pru_cmd_t *c) {
    const uint8_t ax = c->axis;

    switch (c->cmd) {

    case CMD_ENABLE:
        if (ax == AXIS_SPINDLE || ax == AXIS_ALL) {
            if (c->value_a) __R30 &= ~SP_EN_BIT; else __R30 |= SP_EN_BIT;
        }
        if (ax == AXIS_LATERAL || ax == AXIS_ALL) {
            if (c->value_a) __R30 &= ~LAT_EN_BIT; else __R30 |= LAT_EN_BIT;
        }
        break;

    case CMD_QUEUE_FLUSH:
        if (ax == AXIS_SPINDLE || ax == AXIS_ALL) spindle_stop();
        if (ax == AXIS_LATERAL || ax == AXIS_ALL) lateral_stop();
        break;

    case CMD_RESET_POSITION:
        if (ax == AXIS_SPINDLE || ax == AXIS_ALL) {
            spindle.step_count   = 0u;
            sp_telem->step_count = 0u;
        }
        if (ax == AXIS_LATERAL || ax == AXIS_ALL) {
            lateral.step_count    = 0u;
            lat_telem->step_count = 0u;
            lateral.position      = 0;
            lat_telem->position_steps = 0;
        }
        break;

    case CMD_EMERGENCY_STOP:
        /* Stop both axes regardless of axis field */
        spindle_stop();
        lateral_stop();
        __R30 |= SP_EN_BIT;    /* disable spindle driver */
        __R30 |= LAT_EN_BIT;   /* disable lateral driver */
        pru_sync->spindle_interval = 0u;
        pru_sync->lateral_interval = 0u;
        return 1;              /* caller jumps to emergency label */

    case CMD_HOME_START:
        /* Home sensing not yet implemented in dual-stepper firmware.
         * Reserve for future: lateral stop + position reset on trigger.
         * Pins for home sensor must be on R31 bits not used by R30 steppers.
         */
        break;

    default:
        break;
    }
    return 0;
}

/* ── publish_telemetry: update shared RAM telemetry structs ──────────────────*/
static void publish_telemetry(void) {
    /* ── Spindle ── */
    uint16_t st_sp = 0u;
    if (spindle.running) st_sp |= STATE_RUNNING; else st_sp |= STATE_IDLE;
    if (!(__R30 & SP_EN_BIT)) st_sp |= STATE_ENABLED;

    uint16_t flt_sp = sp_telem->faults;
    if (spindle.underrun) flt_sp |= FAULT_MOVE_UNDERRUN;

    sp_telem->seq              = sp_telem->seq + 1u;
    sp_telem->step_count       = spindle.step_count;
    sp_telem->current_interval = spindle.interval;
    sp_telem->moves_pending    = stepper_ring_count(&g_mr_sp);
    sp_telem->position_steps   = 0;    /* spindle has no position tracking */
    sp_telem->state            = st_sp;
    sp_telem->faults           = flt_sp;

    pru_sync->spindle_interval = spindle.interval;

    /* ── Lateral ── */
    uint16_t st_lat = 0u;
    if (lateral.running) st_lat |= STATE_RUNNING; else st_lat |= STATE_IDLE;
    if (!(__R30 & LAT_EN_BIT)) st_lat |= STATE_ENABLED;

    uint16_t flt_lat = lat_telem->faults;
    if (lateral.underrun) flt_lat |= FAULT_MOVE_UNDERRUN;

    lat_telem->seq              = lat_telem->seq + 1u;
    lat_telem->step_count       = lateral.step_count;
    lat_telem->current_interval = lateral.interval;
    lat_telem->moves_pending    = stepper_ring_count(&g_mr_lat);
    lat_telem->position_steps   = lateral.position;
    lat_telem->state            = st_lat;
    lat_telem->faults           = flt_lat;

    pru_sync->lateral_interval  = lateral.interval;
}

/* ── main ────────────────────────────────────────────────────────────────────*/
int main(void) {
    /* Safe default outputs: all STEP=low, all ENABLE=high (drivers disabled) */
    __R30 = SP_EN_BIT | LAT_EN_BIT;

    /* PRU1 owns the IEP timer in the dual-stepper topology.
     * PRU0 (aux) must NOT call IEP_INIT(). */
    IEP_INIT();

    /* Initialise move ring contexts */
    g_mr_sp.ring  = sp_move_ring;
    g_mr_sp.whead = sp_move_whead;
    g_mr_sp.rhead = sp_move_rhead;
    g_mr_sp.slots = IPC_SP_MOVE_SLOTS;

    g_mr_lat.ring  = lat_move_ring;
    g_mr_lat.whead = lat_move_whead;
    g_mr_lat.rhead = lat_move_rhead;
    g_mr_lat.slots = IPC_LAT_MOVE_SLOTS;

    /* Reset ring heads */
    *sp_move_whead  = 0u;
    *sp_move_rhead  = 0u;
    *lat_move_whead = 0u;
    *lat_move_rhead = 0u;
    *cmd_rhead      = 0u;

    uint32_t cmd_local_rhead = 0u;
    uint32_t cmd_counter     = 0u;
    uint32_t telem_counter   = 0u;

    while (1) {
        /* ── Spindle step engine (hot path) ─────────────────────────────── */
        if (spindle.running) {
            if (!timer_before(IEP_NOW(), spindle.next_edge_time)) {
                if (spindle.dir_pending) {
                    apply_dir_sp(&spindle);
                    spindle.dir_pending = 0u;
                }
                uint8_t pin = stepper_edge(&spindle, &g_mr_sp);
                if (pin) __R30 |= SP_STEP_BIT; else __R30 &= ~SP_STEP_BIT;
            }
        } else {
            if (stepper_ring_count(&g_mr_sp) > 0u) {
                stepper_start_first(&spindle, &g_mr_sp);
                if (spindle.dir_pending) {
                    apply_dir_sp(&spindle);
                    spindle.dir_pending = 0u;
                }
            }
        }

        /* ── Lateral step engine (hot path) ─────────────────────────────── */
        if (lateral.running) {
            if (!timer_before(IEP_NOW(), lateral.next_edge_time)) {
                if (lateral.dir_pending) {
                    apply_dir_lat(&lateral);
                    lateral.dir_pending = 0u;
                }
                uint8_t pin = stepper_edge(&lateral, &g_mr_lat);
                if (pin) __R30 |= LAT_STEP_BIT; else __R30 &= ~LAT_STEP_BIT;
            }
        } else {
            if (stepper_ring_count(&g_mr_lat) > 0u) {
                stepper_start_first(&lateral, &g_mr_lat);
                if (lateral.dir_pending) {
                    apply_dir_lat(&lateral);
                    lateral.dir_pending = 0u;
                }
            }
        }

        /* ── Command ring (throttled) ────────────────────────────────────── */
        if (++cmd_counter >= CMD_CHECK_STRIDE) {
            cmd_counter = 0u;
            uint32_t wh = *cmd_whead % IPC_CMD_RING_SLOTS;
            while (cmd_local_rhead != wh) {
                if (process_command(&cmd_ring[cmd_local_rhead])) goto emergency;
                cmd_local_rhead = (cmd_local_rhead + 1u) % IPC_CMD_RING_SLOTS;
                *cmd_rhead = cmd_local_rhead;
            }
        }

        /* ── Telemetry (throttled) ───────────────────────────────────────── */
        if (++telem_counter >= TELEM_STRIDE) {
            telem_counter = 0u;
            publish_telemetry();
        }
    }

emergency:
    /* EMERGENCY_STOP: halt both axes immediately, disable drivers, halt PRU */
    spindle_stop();
    lateral_stop();
    __R30 |= SP_EN_BIT;
    __R30 |= LAT_EN_BIT;
    publish_telemetry();
    __asm volatile ("HALT");
    return 0;
}
