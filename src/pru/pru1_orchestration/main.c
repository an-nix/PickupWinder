/* pru1_orchestration/main.c — PRU1: orchestrator and sole host interface.
 *
 * Architecture layer 2/4.
 *
 * PRU1 is the brain. It is the only PRU that communicates with the host
 * (daemon). It reads commands from host_cmd_t, manages the homing state
 * machine, writes motor_params_t for PRU0 to consume, reads motor_telem_t
 * feedback from PRU0, and publishes pru_status_t for the daemon.
 *
 * Responsibilities:
 *   1. Poll host_cmd_t for new commands from the daemon.
 *   2. On HOST_CMD_SET_SPEED: update motor_params intervals/directions.
 *   3. On HOST_CMD_ENABLE: enable/disable motor drivers via motor_params.
 *   4. On HOST_CMD_ESTOP: stop everything, disable drivers.
 *   5. On HOST_CMD_HOME_START: enter homing state machine.
 *   6. On HOST_CMD_ACK_EVENT: clear pending event.
 *   7. On HOST_CMD_RESET_POS: request PRU0 to reset counters.
 *   8. Monitor endstop_mask from motor_telem for homing completion.
 *   9. Synchronize lateral/spindle speeds (future).
 *  10. Publish pru_status_t every STATUS_STRIDE iterations.
 *
 * Rules:
 *   - PRU1 must NEVER call IEP_INIT() — PRU0 owns the IEP.
 *   - PRU1 must NOT write motor pins (R30 STEP/DIR/EN).
 *   - PRU1 writes motor_params_t; PRU0 reads it.
 *   - PRU1 reads motor_telem_t; PRU0 writes it.
 */

#include <stdint.h>
#include "../include/pru_ipc.h"

/* ── Shared RAM pointers ─────────────────────────────────────────────────── */
static volatile host_cmd_t     *host_cmd =
    (volatile host_cmd_t *)    (PRU_SRAM_PHYS_BASE + IPC_HOST_CMD_OFFSET);

static volatile motor_params_t *params =
    (volatile motor_params_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_PARAMS_OFFSET);

static volatile motor_telem_t  *telem =
    (volatile motor_telem_t *) (PRU_SRAM_PHYS_BASE + IPC_MOTOR_TELEM_OFFSET);

static volatile pru_status_t   *status =
    (volatile pru_status_t *)  (PRU_SRAM_PHYS_BASE + IPC_PRU_STATUS_OFFSET);

/* ── Cadence ─────────────────────────────────────────────────────────────── */
#define CMD_CHECK_STRIDE    1000u      /* check host cmd every ~1000 loops   */
#define STATUS_STRIDE       500000u    /* publish status every ~2.5 ms       */

/* ── Homing state machine ────────────────────────────────────────────────── */
typedef enum {
    HOMING_IDLE     = 0,
    HOMING_APPROACH = 1,   /* lateral moving toward endstop                  */
    HOMING_HIT      = 2,   /* endstop triggered, resetting position          */
} homing_state_t;

static homing_state_t g_homing = HOMING_IDLE;

/* Saved lateral speed/direction before homing overrides them. */
static uint32_t g_saved_lat_interval = 0u;
static uint8_t  g_saved_lat_dir      = 0u;

/* Default homing approach speed: ~2 mm/s with 3072 steps/mm → 6144 Hz
 * interval = 200 MHz / (2 * 6144) ≈ 16276 IEP cycles */
#define HOMING_INTERVAL  16276u
#define HOMING_DIR       1u   /* direction toward home sensor (adjustable)  */

/* ── Helper: acknowledge host command ────────────────────────────────────── */
static inline void ack_cmd(uint8_t opcode) {
    host_cmd->cmd_ack = opcode;
    host_cmd->cmd     = HOST_CMD_NOP;
}

/* ── Helper: emergency stop ──────────────────────────────────────────────── */
static void do_estop(void) {
    params->sp_run     = 0u;
    params->lat_run    = 0u;
    params->sp_enable  = 0u;
    params->lat_enable = 0u;
    params->sp_interval  = 0u;
    params->lat_interval = 0u;
    g_homing = HOMING_IDLE;
}

/* ── Process one host command ────────────────────────────────────────────── */
static void process_host_cmd(void) {
    uint8_t cmd = host_cmd->cmd;
    if (cmd == HOST_CMD_NOP) return;

    switch (cmd) {

    case HOST_CMD_SET_SPEED:
        /* Update target speeds. PRU0 picks them up immediately. */
        params->sp_interval  = host_cmd->sp_interval_target;
        params->lat_interval = host_cmd->lat_interval_target;
        params->sp_dir       = host_cmd->sp_dir;
        params->lat_dir      = host_cmd->lat_dir;
        /* Auto-start if intervals are non-zero and drivers enabled. */
        if (host_cmd->sp_interval_target > 0u && params->sp_enable)
            params->sp_run = 1u;
        if (host_cmd->lat_interval_target > 0u && params->lat_enable)
            params->lat_run = 1u;
        /* Stop if interval is zero. */
        if (host_cmd->sp_interval_target == 0u)
            params->sp_run = 0u;
        if (host_cmd->lat_interval_target == 0u)
            params->lat_run = 0u;
        ack_cmd(cmd);
        break;

    case HOST_CMD_ENABLE: {
        uint8_t en = (host_cmd->value_a != 0u) ? 1u : 0u;
        uint8_t ax = host_cmd->axis;
        if (ax == AXIS_SPINDLE || ax == AXIS_ALL) {
            params->sp_enable = en;
            if (!en) params->sp_run = 0u;
        }
        if (ax == AXIS_LATERAL || ax == AXIS_ALL) {
            params->lat_enable = en;
            if (!en) params->lat_run = 0u;
        }
        ack_cmd(cmd);
        break;
    }

    case HOST_CMD_ESTOP:
        do_estop();
        ack_cmd(cmd);
        break;

    case HOST_CMD_HOME_START:
        if (g_homing == HOMING_IDLE) {
            /* Save current lateral params, override with homing approach. */
            g_saved_lat_interval  = params->lat_interval;
            g_saved_lat_dir       = params->lat_dir;
            params->lat_interval  = HOMING_INTERVAL;
            params->lat_dir       = HOMING_DIR;
            params->lat_enable    = 1u;
            params->lat_run       = 1u;
            /* Stop spindle during homing. */
            params->sp_run        = 0u;
            g_homing = HOMING_APPROACH;
        }
        ack_cmd(cmd);
        break;

    case HOST_CMD_ACK_EVENT:
        status->event_pending = 0u;
        status->event_type    = EVENT_NONE;
        ack_cmd(cmd);
        break;

    case HOST_CMD_RESET_POS:
        /* PRU1 cannot directly reset PRU0 counters (they are in PRU0's
         * local variables). We signal via motor_telem: set step_count and
         * position to 0. PRU0 reads telem.seq==0 as a reset request.
         *
         * Simpler approach: motor_telem is PRU0-written, so we use a flag
         * in motor_params. For now, we zero the telem fields directly since
         * both PRUs share the same RAM and PRU0 will overwrite on next
         * telem publish. The effect is the counts restart from zero.       */
        telem->sp_step_count  = 0u;
        telem->lat_step_count = 0u;
        telem->lat_position   = 0;
        telem->sp_faults      = 0u;
        telem->lat_faults     = 0u;
        ack_cmd(cmd);
        break;

    default:
        ack_cmd(cmd);
        break;
    }
}

/* ── Homing state machine ────────────────────────────────────────────────── */
static void homing_tick(void) {
    if (g_homing == HOMING_IDLE) return;

    uint8_t es = telem->endstop_mask;

    switch (g_homing) {

    case HOMING_APPROACH:
        if (es & (ENDSTOP1_MASK | ENDSTOP2_MASK)) {
            /* Endstop hit: PRU0 already stopped the lateral motor (safety).
             * Reset lateral position to zero. */
            params->lat_run = 0u;
            telem->lat_step_count = 0u;
            telem->lat_position   = 0;
            telem->lat_faults     = 0u;  /* clear FAULT_ENDSTOP_HIT */
            g_homing = HOMING_HIT;
        }
        break;

    case HOMING_HIT:
        /* Restore saved lateral parameters. */
        params->lat_interval = g_saved_lat_interval;
        params->lat_dir      = g_saved_lat_dir;
        /* Notify host. */
        status->event_type    = EVENT_HOME_COMPLETE;
        status->event_pending = 1u;
        g_homing = HOMING_IDLE;
        break;

    default:
        g_homing = HOMING_IDLE;
        break;
    }
}

/* ── Publish aggregated status ───────────────────────────────────────────── */
static void publish_status(void) {
    /* Copy telemetry snapshot. */
    status->sp_step_count     = telem->sp_step_count;
    status->lat_step_count    = telem->lat_step_count;
    status->lat_position      = telem->lat_position;
    status->sp_interval_actual  = telem->sp_interval_actual;
    status->lat_interval_actual = telem->lat_interval_actual;
    status->endstop_mask      = telem->endstop_mask;
    status->sp_faults         = telem->sp_faults;
    status->lat_faults        = telem->lat_faults;

    /* PRU1 state. */
    uint8_t st = PRU1_STATE_IDLE;
    if (g_homing != HOMING_IDLE)          st |= PRU1_STATE_HOMING;
    if (params->sp_run || params->lat_run) st |= PRU1_STATE_RUNNING;
    if (telem->endstop_mask)               st |= PRU1_STATE_AT_HOME;
    if (telem->sp_faults || telem->lat_faults) st |= PRU1_STATE_FAULT;
    status->pru1_state = st;

    /* Endstop event (outside homing): notify host of safety stop. */
    if (g_homing == HOMING_IDLE &&
        (telem->lat_faults & FAULT_ENDSTOP_HIT) &&
        !status->event_pending) {
        status->event_type    = EVENT_ENDSTOP_HIT;
        status->event_pending = 1u;
    }

    status->seq++;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Main loop
 * ════════════════════════════════════════════════════════════════════════════ */
int main(void) {
    /* PRU1 never touches IEP or motor pins. */

    /* Zero shared memory areas owned by PRU1. */
    *params = (motor_params_t){0};
    *status = (pru_status_t){0};
    host_cmd->cmd     = HOST_CMD_NOP;
    host_cmd->cmd_ack = HOST_CMD_NOP;

    uint32_t loop_cnt = 0u;

    while (1) {
        /* ── 1. Check host commands (throttled) ─────────────────────────── */
        if (loop_cnt % CMD_CHECK_STRIDE == 0u) {
            process_host_cmd();
        }

        /* ── 2. Homing state machine ────────────────────────────────────── */
        if (loop_cnt % CMD_CHECK_STRIDE == 0u) {
            homing_tick();
        }

        /* ── 3. Publish status (throttled) ──────────────────────────────── */
        if (loop_cnt % STATUS_STRIDE == 0u) {
            publish_status();
        }

        loop_cnt++;
    }

    return 0;
}
