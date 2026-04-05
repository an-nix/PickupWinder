/* pru0_motor_control/main.c — PRU0 dual-axis motor control (authoritative topology).
 *
 * Responsibilities:
 *   - PRU0 owns and initialises IEP timer (200 MHz).
 *   - PRU0 consumes both move rings (AXIS_SPINDLE + AXIS_LATERAL).
 *   - PRU0 drives STEP/DIR/EN for both motors with IEP-absolute timing.
 *   - PRU0 samples endstops on R31 and exposes state/fault via telemetry.
 *   - PRU1 is orchestration/supervision only and must not drive motor pins.
 *
 * Canonical PRU0 mapping:
 *   Motor A (spindle semantics in IPC):
 *     R30[1] -> P9_29 STEP_A   (future TMC UART TX #1 pin)
 *     R30[5] -> P9_27 DIR_A
 *     R30[7] -> P9_25 EN_A     (active-low)
 *
 *   Motor B (lateral semantics in IPC):
 *     R30[4] -> P9_42 STEP_B
 *     R30[0] -> P9_31 DIR_B    (future TMC UART TX #2 pin)
 *     R30[3] -> P9_28 EN_B     (active-low)
 *
 *   Endstops:
 *     R31[15] -> P8_15 ENDSTOP_1
 *     R31[14] -> P8_16 ENDSTOP_2
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_stepper.h"

#include "../include/pru_regs.h"

/* Motor A (spindle IPC axis) */
#define SP_STEP_BIT        (1u << 1)
#define SP_DIR_BIT         (1u << 5)
#define SP_EN_BIT          (1u << 7)   /* active-low */

/* Motor B (lateral IPC axis) */
#define LAT_STEP_BIT       (1u << 4)
#define LAT_DIR_BIT        (1u << 0)
#define LAT_EN_BIT         (1u << 3)   /* active-low */

/* Endstops (inputs on R31) */
#define ENDSTOP1_BIT       (1u << 15)
#define ENDSTOP2_BIT       (1u << 14)

/* Throttle constants (shared) */
#include "../include/pru_throttle.h"

/* Shared RAM pointers */
static volatile pru_cmd_t        *cmd_ring   =
    (volatile pru_cmd_t *)       (PRU_SRAM_PHYS_BASE + IPC_CMD_RING_OFFSET);
static volatile uint32_t         *cmd_whead  =
    (volatile uint32_t *)        (PRU_SRAM_PHYS_BASE + IPC_CMD_WHEAD_OFFSET);
static volatile uint32_t         *cmd_rhead  =
    (volatile uint32_t *)        (PRU_SRAM_PHYS_BASE + IPC_CMD_RHEAD_OFFSET);
static volatile pru_axis_telem_t *sp_telem   =
    (volatile pru_axis_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_SPINDLE_TELEM_OFFSET);
static volatile pru_sync_t       *pru_sync   =
    (volatile pru_sync_t *)      (PRU_SRAM_PHYS_BASE + IPC_SYNC_OFFSET);

static volatile pru_move_t  *sp_move_ring  =
    (volatile pru_move_t *)  (PRU_SRAM_PHYS_BASE + IPC_SP_MOVE_RING_OFFSET);
static volatile uint32_t    *sp_move_whead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_SP_MOVE_WHEAD_OFFSET);
static volatile uint32_t    *sp_move_rhead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_SP_MOVE_RHEAD_OFFSET);

static volatile pru_axis_telem_t *lat_telem  =
    (volatile pru_axis_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_LATERAL_TELEM_OFFSET);

static volatile pru_move_t  *lat_move_ring  =
    (volatile pru_move_t *)  (PRU_SRAM_PHYS_BASE + IPC_LAT_MOVE_RING_OFFSET);
static volatile uint32_t    *lat_move_whead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_LAT_MOVE_WHEAD_OFFSET);
static volatile uint32_t    *lat_move_rhead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_LAT_MOVE_RHEAD_OFFSET);

/* Module state */
static stepper_t  spindle = {0};
static stepper_t  lateral = {0};
static move_ring_t g_mr_sp;
static move_ring_t g_mr_lat;
static uint8_t homing_mode = 0u;

static void init_shared_state(void) {
    *cmd_whead      = 0u;
    *cmd_rhead      = 0u;
    *sp_move_whead  = 0u;
    *sp_move_rhead  = 0u;
    *lat_move_whead = 0u;
    *lat_move_rhead = 0u;

    *sp_telem = (pru_axis_telem_t){0};
    *lat_telem = (pru_axis_telem_t){0};
    *pru_sync = (pru_sync_t){0};
}

static inline void apply_dir_sp(const stepper_t *s) {
    if (s->direction) __R30 |= SP_DIR_BIT; else __R30 &= ~SP_DIR_BIT;
}

static inline void apply_dir_lat(const stepper_t *s) {
    if (s->direction) __R30 |= LAT_DIR_BIT; else __R30 &= ~LAT_DIR_BIT;
}

static inline uint8_t endstop1_hit(void) {
    return (__R31 & ENDSTOP1_BIT) ? 0u : 1u;
}

static inline uint8_t endstop2_hit(void) {
    return (__R31 & ENDSTOP2_BIT) ? 0u : 1u;
}

static void spindle_stop(void) {
    stepper_force_stop(&spindle);
    __R30 &= ~SP_STEP_BIT;
    stepper_flush_ring(&g_mr_sp);
}

static void lateral_stop(void) {
    stepper_force_stop(&lateral);
    __R30 &= ~LAT_STEP_BIT;
    stepper_flush_ring(&g_mr_lat);
}

/* process_command: return 1 if emergency stop was requested */
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
        homing_mode = 0u;
        break;

    case CMD_RESET_POSITION:
        if (ax == AXIS_SPINDLE || ax == AXIS_ALL) {
            spindle.step_count   = 0u;
            sp_telem->step_count = 0u;
        }
        if (ax == AXIS_LATERAL || ax == AXIS_ALL) {
            lateral.step_count     = 0u;
            lateral.position       = 0;
            lat_telem->step_count  = 0u;
            lat_telem->position_steps = 0;
        }
        homing_mode = 0u;
        break;

    case CMD_HOME_START:
        if (ax == AXIS_LATERAL || ax == AXIS_ALL) {
            homing_mode = 1u;
        }
        break;

    case CMD_EMERGENCY_STOP:
        spindle_stop();
        lateral_stop();
        __R30 |= SP_EN_BIT;
        __R30 |= LAT_EN_BIT;
        pru_sync->spindle_interval = 0u;
        pru_sync->lateral_interval = 0u;
        homing_mode = 0u;
        return 1;

    default:
        break;
    }
    return 0;
}

/* publish telemetry */
static void publish_telemetry(void) {
    uint8_t es1 = endstop1_hit();
    uint8_t es2 = endstop2_hit();

    uint16_t sp_state = 0u;
    if (spindle.running) sp_state |= STATE_RUNNING; else sp_state |= STATE_IDLE;
    if (!(__R30 & SP_EN_BIT)) sp_state |= STATE_ENABLED;

    uint16_t sp_faults = 0u;
    if (spindle.underrun) sp_faults |= FAULT_MOVE_UNDERRUN;

    sp_telem->seq              = sp_telem->seq + 1u;
    sp_telem->step_count       = spindle.step_count;
    sp_telem->current_interval = spindle.interval;
    sp_telem->moves_pending    = stepper_ring_count(&g_mr_sp);
    sp_telem->position_steps   = 0;
    sp_telem->state            = sp_state;
    sp_telem->faults           = sp_faults;
    pru_sync->spindle_interval = spindle.interval;

    uint16_t lat_state = 0u;
    if (lateral.running) lat_state |= STATE_RUNNING; else lat_state |= STATE_IDLE;
    if (!(__R30 & LAT_EN_BIT)) lat_state |= STATE_ENABLED;
    if (homing_mode) lat_state |= STATE_HOMING;
    if (es1 || es2) lat_state |= STATE_AT_HOME;

    uint16_t lat_faults = 0u;
    if (lateral.underrun) lat_faults |= FAULT_MOVE_UNDERRUN;

    lat_telem->seq              = lat_telem->seq + 1u;
    lat_telem->step_count       = lateral.step_count;
    lat_telem->current_interval = lateral.interval;
    lat_telem->moves_pending    = stepper_ring_count(&g_mr_lat);
    lat_telem->position_steps   = lateral.position;
    lat_telem->state            = lat_state;
    lat_telem->faults           = lat_faults;
    pru_sync->lateral_interval  = lateral.interval;
}

/* main */
int main(void) {
    /* Safe outputs: STEP low, EN high (drivers disabled). */
    __R30 &= ~(SP_STEP_BIT | SP_DIR_BIT | LAT_STEP_BIT | LAT_DIR_BIT);
    __R30 |=  (SP_EN_BIT | LAT_EN_BIT);

    /* PRU0 owns IEP */
    IEP_INIT();

    g_mr_sp.ring  = sp_move_ring;
    g_mr_sp.whead = sp_move_whead;
    g_mr_sp.rhead = sp_move_rhead;
    g_mr_sp.slots = IPC_SP_MOVE_SLOTS;

    g_mr_lat.ring  = lat_move_ring;
    g_mr_lat.whead = lat_move_whead;
    g_mr_lat.rhead = lat_move_rhead;
    g_mr_lat.slots = IPC_LAT_MOVE_SLOTS;

    init_shared_state();

    uint32_t cmd_local_rhead = 0u;
    uint32_t cmd_counter     = 0u;
    uint32_t telem_counter   = 0u;

    while (1) {
        if (homing_mode && (endstop1_hit() || endstop2_hit())) {
            lateral_stop();
            lateral.step_count      = 0u;
            lateral.position        = 0;
            lat_telem->step_count   = 0u;
            lat_telem->position_steps = 0;
            homing_mode = 0u;
        }

        /* Spindle (motor A) hot path */
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

        /* Lateral (motor B) hot path */
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

        /* Command ring (throttled) */
        if (++cmd_counter >= CMD_CHECK_STRIDE) {
            cmd_counter = 0u;
            uint32_t wh = *cmd_whead % IPC_CMD_RING_SLOTS;
            while (cmd_local_rhead != wh) {
                if (process_command(&cmd_ring[cmd_local_rhead])) goto emergency;
                cmd_local_rhead = (cmd_local_rhead + 1u) % IPC_CMD_RING_SLOTS;
                *cmd_rhead = cmd_local_rhead;
            }
        }

        /* Telemetry (throttled) */
        if (++telem_counter >= TELEM_STRIDE) {
            telem_counter = 0u;
            publish_telemetry();
        }
    }

emergency:
    spindle_stop();
    lateral_stop();
    __R30 |= SP_EN_BIT;
    __R30 |= LAT_EN_BIT;
    homing_mode = 0u;
    publish_telemetry();
    __asm volatile ("HALT");
    return 0;
}
