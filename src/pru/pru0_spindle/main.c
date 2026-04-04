/* pru0_spindle/main.c — PRU0 spindle step generator (Klipper-inspired).
 *
 * ── Topology note ────────────────────────────────────────────────────────────
 * ORIGINAL topology (one axis per PRU):
 *   PRU0 = spindle step gen  — owns IEP timer — calls IEP_INIT()
 *   PRU1 = lateral step gen  — reads IEP_NOW() only
 *
 * DUAL-STEPPER topology (both axes on PRU1, default for P8_41-P8_46):
 *   PRU1 = pru1_dual_stepper/main.c — owns IEP — calls IEP_INIT()
 *   PRU0 = aux tasks (sensors, UART, …) — must NOT call IEP_INIT()
 *
 * This file is the ORIGINAL single-axis spindle firmware kept for reference.
 * It is NOT loaded by default when using pru1_dual_stepper.
 * Build with:  make pru0_spindle   (or: make all if FW_NAME_0 still points here)
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * Responsibilities:
 *  - Own the IEP hardware counter (call IEP_INIT()).
 *  - Consume move triples from the spindle move ring and emit STEP edges
 *    with absolute IEP timing using the stepper engine in pru_stepper.h.
 *  - Publish telemetry and respond to lifecycle commands via command ring.
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_stepper.h"

#include "../include/pru_regs.h"

/* GPIO bit masks */
#define SPINDLE_STEP_BIT   (1u << 0)
#define SPINDLE_DIR_BIT    (1u << 1)
#define SPINDLE_EN_BIT     (1u << 2)   /* active-low */

/* Throttle constants (shared) */
#include "../include/pru_throttle.h"

/* Shared RAM pointers */
static volatile pru_cmd_t        *cmd_ring   =
    (volatile pru_cmd_t *)       (PRU_SRAM_PHYS_BASE + IPC_CMD_RING_OFFSET);
static volatile uint32_t         *cmd_whead  =
    (volatile uint32_t *)        (PRU_SRAM_PHYS_BASE + IPC_CMD_WHEAD_OFFSET);
static volatile uint32_t         *cmd_rhead  =
    (volatile uint32_t *)        (PRU_SRAM_PHYS_BASE + IPC_CMD_RHEAD_OFFSET);
static volatile pru_axis_telem_t *telem      =
    (volatile pru_axis_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_SPINDLE_TELEM_OFFSET);
static volatile pru_sync_t       *pru_sync   =
    (volatile pru_sync_t *)      (PRU_SRAM_PHYS_BASE + IPC_SYNC_OFFSET);

static volatile pru_move_t  *sp_move_ring  =
    (volatile pru_move_t *)  (PRU_SRAM_PHYS_BASE + IPC_SP_MOVE_RING_OFFSET);
static volatile uint32_t    *sp_move_whead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_SP_MOVE_WHEAD_OFFSET);
static volatile uint32_t    *sp_move_rhead =
    (volatile uint32_t *)    (PRU_SRAM_PHYS_BASE + IPC_SP_MOVE_RHEAD_OFFSET);

/* Module state */
static stepper_t  spindle = {0};
static move_ring_t g_mr;    /* initialised in main() */

/* process_command: return 1 if emergency stop was requested */
static int process_command(const volatile pru_cmd_t *c) {
    if (c->axis != AXIS_SPINDLE && c->axis != AXIS_ALL) return 0;

    switch (c->cmd) {
    case CMD_ENABLE:
        if (c->value_a) __R30 &= ~SPINDLE_EN_BIT; else __R30 |= SPINDLE_EN_BIT;
        break;

    case CMD_QUEUE_FLUSH:
        stepper_force_stop(&spindle);
        __R30 &= ~SPINDLE_STEP_BIT;
        stepper_flush_ring(&g_mr);
        break;

    case CMD_RESET_POSITION:
        spindle.step_count = 0u;
        telem->step_count  = 0u;
        break;

    case CMD_EMERGENCY_STOP:
        stepper_force_stop(&spindle);
        __R30 &= ~SPINDLE_STEP_BIT;
        __R30 |= SPINDLE_EN_BIT;     /* disable driver */
        pru_sync->spindle_interval = 0u;
        return 1;

    default:
        break;
    }
    return 0;
}

/* publish telemetry */
static void publish_telemetry(void) {
    telem->seq              = telem->seq + 1u;
    telem->step_count       = spindle.step_count;
    telem->current_interval = spindle.interval;
    telem->moves_pending    = stepper_ring_count(&g_mr);
    telem->position_steps   = 0; /* spindle has no position tracking */

    uint16_t st = 0u;
    if (spindle.running) st |= STATE_RUNNING; else st |= STATE_IDLE;
    if (!(__R30 & SPINDLE_EN_BIT)) st |= STATE_ENABLED;
    telem->state  = st;

    uint16_t flt = telem->faults;
    if (spindle.underrun) flt |= FAULT_MOVE_UNDERRUN;
    telem->faults = flt;

    pru_sync->spindle_interval = spindle.interval;
}

/* main */
int main(void) {
    /* Safe outputs */
    __R30 &= ~SPINDLE_STEP_BIT;
    __R30 &= ~SPINDLE_DIR_BIT;
    __R30 |=  SPINDLE_EN_BIT;  /* driver disabled */

    /* PRU0 owns IEP */
    IEP_INIT();

    /* init move ring context */
    g_mr.ring  = sp_move_ring;
    g_mr.whead = sp_move_whead;
    g_mr.rhead = sp_move_rhead;
    g_mr.slots = IPC_SP_MOVE_SLOTS;

    *sp_move_whead = 0u;
    *sp_move_rhead = 0u;
    *cmd_rhead     = 0u;

    uint32_t cmd_local_rhead = 0u;
    uint32_t cmd_counter     = 0u;
    uint32_t telem_counter   = 0u;

    while (1) {
        /* Step generation (hot path) */
        if (spindle.running) {
            if (!timer_before(IEP_NOW(), spindle.next_edge_time)) {
                if (spindle.dir_pending) {
                    if (spindle.direction) __R30 |= SPINDLE_DIR_BIT;
                    else                   __R30 &= ~SPINDLE_DIR_BIT;
                    spindle.dir_pending = 0u;
                }
                uint8_t pin = stepper_edge(&spindle, &g_mr);
                if (pin) __R30 |= SPINDLE_STEP_BIT; else __R30 &= ~SPINDLE_STEP_BIT;
            }
        } else {
            if (stepper_ring_count(&g_mr) > 0u) {
                stepper_start_first(&spindle, &g_mr);
                if (spindle.dir_pending) {
                    if (spindle.direction) __R30 |= SPINDLE_DIR_BIT;
                    else                   __R30 &= ~SPINDLE_DIR_BIT;
                    spindle.dir_pending = 0u;
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
    stepper_force_stop(&spindle);
    __R30 &= ~SPINDLE_STEP_BIT;
    __R30 |=  SPINDLE_EN_BIT;
    publish_telemetry();
    __asm volatile ("HALT");
    return 0;
}
