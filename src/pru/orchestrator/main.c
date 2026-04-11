/* orchestrator/main.c — PRU orchestrator firmware (runs on PRU0 at runtime).
 *
 * Architecture layer 2/4.
 *
 * PRU0 is the orchestrator. It is the only PRU that communicates with the
 * host (daemon). It reads commands from host_cmd_t, manages the homing state
 * machine and software limits, writes motor_params_t for PRU1 to consume,
 * reads motor_telem_t feedback from PRU1, and publishes pru_status_t.
 *
 * Responsibilities:
 *   1. Poll host_cmd_t for new commands from the daemon.
 *   2. On HOST_CMD_SET_SPEED: update motor_params intervals/directions.
 *   3. On HOST_CMD_ENABLE: enable/disable motor drivers via motor_params.
 *   4. On HOST_CMD_ESTOP: stop everything, disable drivers.
 *   5. On HOST_CMD_HOME_START: enter homing state machine.
 *   6. On HOST_CMD_ACK_EVENT: clear pending event and limit locks.
 *   7. On HOST_CMD_RESET_POS: zero telem step counts and position.
 *   8. On HOST_CMD_SET_LIMITS: configure axis software limits.
 *   9. On HOST_CMD_MOVE_TO: arm trapezoidal profile in PRU1 (motor_params).
 *  10. Read R31 endstop inputs (P9_28/P9_30), halt lateral on assertion.
 *  11. Check software limits; latch axis if position out of range.
 *  12. Check telem->lat_move_done → publish EVENT_MOVE_COMPLETE.
 *  13. Coordinate spindle speed with lateral ramps (Q6 sp_lat_coord ratio).
 *  14. Publish pru_status_t every STATUS_STRIDE iterations.
 *
 * Rules:
 *   - PRU0 must NEVER call IEP_INIT() — PRU1 (motor) owns the IEP.
 *   - PRU0 must NOT write motor pins (R30 STEP/DIR/EN).
 *   - PRU0 writes motor_params_t; PRU1 reads it.
 *   - PRU0 reads motor_telem_t; PRU1 writes it.
 *   - PRU0 reads R31 for endstop inputs (P9_28=R31[6], P9_30=R31[2]).
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_regs.h"  /* register volatile __R31 */
#include "../include/pru_rsc_table.h"  /* required by remoteproc */

/* ── Shared RAM pointers ─────────────────────────────────────────────────── */
static volatile host_cmd_t     *host_cmd =
    (volatile host_cmd_t *)    (PRU_SRAM_PHYS_BASE + IPC_HOST_CMD_OFFSET);

static volatile motor_params_t *params =
    (volatile motor_params_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_PARAMS_OFFSET);

static volatile motor_telem_t  *telem =
    (volatile motor_telem_t *) (PRU_SRAM_PHYS_BASE + IPC_MOTOR_TELEM_OFFSET);

static volatile pru_status_t   *status =
    (volatile pru_status_t *)  (PRU_SRAM_PHYS_BASE + IPC_PRU_STATUS_OFFSET);

static volatile move_ctrl_t    *move_ctrl =
    (volatile move_ctrl_t *)   (PRU_SRAM_PHYS_BASE + IPC_MOVE_CTRL_OFFSET);

static volatile step_block_t   *move_blocks =
    (volatile step_block_t *)  (PRU_SRAM_PHYS_BASE + IPC_STEP_BLOCKS_OFFSET);

/* ── Cadence ─────────────────────────────────────────────────────────────── */
#define CMD_CHECK_STRIDE    1000u      /* check host cmd every ~1000 loops   */
#define STATUS_STRIDE       500000u    /* publish status every ~2.5 ms       */

/* ── Endstop inputs (PRU0 R31) ───────────────────────────────────────────── */
/* P9_28 = conf_mcasp0_ahclkr  offset 0x19C  MODE6 = pr1_pru0_pru_r31_3     */
/* P9_30 = conf_mcasp0_fsr     offset 0x198  MODE6 = pr1_pru0_pru_r31_2     */
/* Both confirmed via pinctrl debugfs: pin 103 / pin 102, value 0x36=MODE6   */
#define ES1_BIT  (1u << 3)   /* R31[3]  P9_28  ENDSTOP_1  (active-HIGH)     */
#define ES2_BIT  (1u << 2)   /* R31[2]  P9_30  ENDSTOP_2  (active-HIGH)     */

static uint8_t g_endstop_mask      = 0u;  /* live endstop state (R31)        */
static uint8_t g_prev_endstop_mask = 0u;  /* previous state for edge detect  */

/* Software limits (per-axis). Defaults: disabled. */
static int32_t  g_limit_min[2]     = {0, 0};
static int32_t  g_limit_max[2]     = {0, 0};
static uint8_t  g_limits_enabled[2]= {0, 0};
static uint8_t  g_limit_locked[2]  = {0, 0};

/* Flag: a move_to profile is in progress on the lateral axis. While set,   *
 * SET_SPEED commands are ignored for the lateral axis.                      */
static uint8_t  g_lat_in_move      = 0u;

/* Spindle-lateral coordination (spindle speed tracks lateral ramps).       *
 * g_sp_lat_coord: Q6 ratio = (sp_iv * 64) / lat_cruise_iv, set by MOVE_TO. *
 *   Non-zero while coordination is active. Cleared by set_speed or e_stop. *
 * g_sp_requested_iv: last spindle speed set by host (restored on stop).    */
static uint32_t g_sp_lat_coord     = 0u;
static uint32_t g_sp_requested_iv  = 0u;

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

/* ── Endstop tick: read R31, safety-stop lateral on assertion ───────────── */
static void endstop_tick(void) {
    uint32_t r31 = __R31;
    uint8_t es = 0u;
    if (r31 & ES1_BIT) es |= ENDSTOP1_MASK;
    if (r31 & ES2_BIT) es |= ENDSTOP2_MASK;
    g_endstop_mask = es;

    /* Rising edge: new endstop assertion */
    if (es && !g_prev_endstop_mask) {
        /* Unconditional lateral safety stop */
        params->lat_run = 0u;
        params->lat_move_active = 0u;  /* abort any armed move               */
        g_lat_in_move   = 0u;
        /* Restore spindle from coordination to last requested speed.        */
        if (g_sp_lat_coord != 0u && g_sp_requested_iv > 0u) {
            params->sp_interval = g_sp_requested_iv;
        }
        g_sp_lat_coord = 0u;
        status->lat_faults |= FAULT_ENDSTOP_HIT;

        /* Notify host if not in homing (homing_tick handles the homing case) */
        if (g_homing == HOMING_IDLE && !status->event_pending) {
            status->event_type    = EVENT_ENDSTOP_HIT;
            status->event_pending = 1u;
        }
    }
    g_prev_endstop_mask = es;
}

/* ── Software limits check: latch and notify on overrun ──────────────── */
static void limits_check(void) {
    /* Only check lateral axis for now (AXIS_LATERAL). */
    if (!g_limits_enabled[AXIS_LATERAL] || g_limit_locked[AXIS_LATERAL]) return;
    int32_t pos = telem->lat_position;
    if (pos < g_limit_min[AXIS_LATERAL] || pos > g_limit_max[AXIS_LATERAL]) {
        /* Latch: stop lateral motor and disable until host acknowledges. */
        params->lat_run    = 0u;
        params->lat_enable = 0u;
        status->lat_faults |= FAULT_OVERRUN;
        if (!status->event_pending) {
            status->event_type    = EVENT_LIMIT_HIT;
            status->event_pending = 1u;
        }
        g_limit_locked[AXIS_LATERAL] = 1u;
        g_lat_in_move  = 0u;
        /* Restore spindle from coordination to last requested speed.        */
        if (g_sp_lat_coord != 0u && g_sp_requested_iv > 0u) {
            params->sp_interval = g_sp_requested_iv;
        }
        g_sp_lat_coord = 0u;
    }
}

/* ── Helper: emergency stop ──────────────────────────────────────────────── */
static void do_estop(void) {
    params->sp_run     = 0u;
    params->lat_run    = 0u;
    params->sp_enable  = 0u;
    params->lat_enable = 0u;
    params->sp_interval  = 0u;
    params->lat_interval = 0u;
    params->lat_move_active = 0u;  /* abort any armed move                 */
    g_homing       = HOMING_IDLE;
    g_lat_in_move  = 0u;
    g_sp_lat_coord = 0u;  /* disable spindle coordination on estop         */
}

/* ── Process one host command ────────────────────────────────────────────── */
static void process_host_cmd(void) {
    uint8_t cmd = host_cmd->cmd;
    if (cmd == HOST_CMD_NOP) return;

    switch (cmd) {

    case HOST_CMD_SET_SPEED:
        /* Track requested spindle speed; disable active coordination.       */
        g_sp_requested_iv  = host_cmd->sp_interval_target;
        g_sp_lat_coord     = 0u;  /* set_speed re-takes direct spindle ctrl  */
        /* Update spindle unconditionally. Update lateral only when no
         * move_to profile is active (g_lat_in_move guards the profile). */
        params->sp_interval = g_sp_requested_iv;
        params->sp_dir      = host_cmd->sp_dir;
        if (g_sp_requested_iv > 0u && params->sp_enable)
            params->sp_run = 1u;
        if (g_sp_requested_iv == 0u)
            params->sp_run = 0u;

        if (!g_lat_in_move) {
            params->lat_interval = host_cmd->lat_interval_target;
            params->lat_dir      = host_cmd->lat_dir;
            if (host_cmd->lat_interval_target > 0u && params->lat_enable)
                params->lat_run = 1u;
            if (host_cmd->lat_interval_target == 0u)
                params->lat_run = 0u;
        }
        ack_cmd(cmd);
        break;

    case HOST_CMD_ENABLE: {
        uint8_t en = (host_cmd->value_a != 0u) ? 1u : 0u;
        uint8_t ax = host_cmd->axis;
        if (ax == AXIS_SPINDLE || ax == AXIS_ALL) {
            /* Allow spindle enable regardless of lateral locks. */
            params->sp_enable = en;
            if (!en) params->sp_run = 0u;
        }
        if (ax == AXIS_LATERAL || ax == AXIS_ALL) {
            /* Do not enable lateral if a software limit lock or move is active. */
            if (en && (g_limit_locked[AXIS_LATERAL] || g_lat_in_move)) {
                /* Ignore enable request while locked or move_to in progress. */
            } else {
                params->lat_enable = en;
                if (!en) params->lat_run = 0u;
            }
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
        /* Clear pending event and release any software lock. */
        status->event_pending = 0u;
        status->event_type    = EVENT_NONE;
        /* Clear all limit locks on ack so host can re-enable safely. */
        g_limit_locked[AXIS_SPINDLE] = 0u;
        g_limit_locked[AXIS_LATERAL] = 0u;
        status->lat_faults = 0u;  /* clear latched fault that caused the event */
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

    case HOST_CMD_SET_LIMITS: {
        uint8_t ax = host_cmd->axis;
        /* Store limits and enable the software limit for that axis. */
        if (ax == AXIS_SPINDLE || ax == AXIS_LATERAL) {
            g_limit_min[ax]      = host_cmd->limit_min;
            g_limit_max[ax]      = host_cmd->limit_max;
            g_limits_enabled[ax] = 1u;
            /* Clearing any previous lock when new limits are set. */
            g_limit_locked[ax]   = 0u;
        }
        ack_cmd(cmd);
        break;
    }

    case HOST_CMD_MOVE_TO:
        /* Only lateral axis is supported for move_to. */
        if (!g_limit_locked[AXIS_LATERAL] && move_ctrl->block_ready && move_ctrl->block_count > 0u) {
            int32_t delta = host_cmd->move_target - telem->lat_position;
            uint8_t new_dir = (delta < 0) ? 1u : 0u;

            params->lat_target_pos  = host_cmd->move_target;
            params->lat_start_iv    = move_blocks[0].interval;
            params->lat_cruise_iv   = host_cmd->move_cruise_iv;
            params->lat_delta_iv    = 0u;
            params->lat_dir         = new_dir;
            /* Store Q6 coordination ratio (0 = coordination disabled).      */
            g_sp_lat_coord          = host_cmd->move_sp_lat_coord;
            /* Set spindle to proportional starting speed immediately.       *
             * lat_run==1: hot retarget — track current interval (≈cruise). *
             * lat_run==0: cold start — both start from their slow speed.   */
            if (g_sp_lat_coord != 0u && params->sp_enable) {
                uint32_t lat_ref = params->lat_run ? params->lat_interval
                                                   : host_cmd->move_start_iv;
                uint32_t sp_init = (g_sp_lat_coord * lat_ref) >> 6u;
                if (sp_init < SP_IV_MIN) sp_init = SP_IV_MIN;
                if (sp_init > SP_IV_MAX) sp_init = SP_IV_MAX;
                params->sp_interval = sp_init;
                params->sp_run      = 1u;
            }
            params->lat_enable      = 1u;
            params->lat_run         = 1u;
            /* Arm: PRU1 reads start_flag and self-clears it. */
            move_ctrl->done_flag    = 0u;
            move_ctrl->start_flag   = 1u;
            g_lat_in_move           = 1u;
        }
        ack_cmd(cmd);
        break;

    default:
        ack_cmd(cmd);
        break;
    }
}

/* ── Spindle-lateral speed coordination ──────────────────────────────────── *
 * Adjusts spindle interval proportionally to lateral interval so that       *
 * turns/mm stays constant during lateral ramps, stops, and reversals.       *
 * Stays active after move_complete (covers the reversal-gap period too).    *
 * Formula (Q6, no division in PRU): sp_adj = (sp_lat_coord * lat_iv) >> 6  *
 * where sp_lat_coord = (sp_cruise_iv * 64) / lat_cruise_iv (daemon-side).   */
static void coord_tick(void) {
    if (g_sp_lat_coord == 0u) return;
    uint32_t lat_iv = params->lat_interval;
    if (lat_iv == 0u) return;  /* lateral parked; leave spindle alone       */
    uint32_t sp_adj  = (g_sp_lat_coord * lat_iv) >> 6u;
    if (sp_adj < SP_IV_MIN) sp_adj = SP_IV_MIN;
    if (sp_adj > SP_IV_MAX) sp_adj = SP_IV_MAX;
    params->sp_interval = sp_adj;
    if (params->sp_enable && sp_adj > 0u) params->sp_run = 1u;
}

/* ── Detect lateral move completion from PRU1 ────────────────────────── */
static void move_complete_check(void) {
    if (!telem->lat_move_done) return;
    if (status->event_pending) return;  /* hold until previous event is acked */
    telem->lat_move_done  = 0u;
    g_lat_in_move         = 0u;         /* release lateral SET_SPEED lock     */
    /* g_sp_lat_coord intentionally kept: spindle stays slow during reversal */
    status->event_type    = EVENT_MOVE_COMPLETE;
    status->event_pending = 1u;
}

/* ── Homing state machine ────────────────────────────────────────────────── */
static void homing_tick(void) {
    if (g_homing == HOMING_IDLE) return;

    uint8_t es = g_endstop_mask;  /* set by endstop_tick() */

    switch (g_homing) {

    case HOMING_APPROACH:
        if (es & (ENDSTOP1_MASK | ENDSTOP2_MASK)) {
            /* Endstop hit: endstop_tick() already stopped the lateral motor.
             * Reset lateral position to zero. */
            params->lat_run = 0u;
            telem->lat_step_count = 0u;
            telem->lat_position   = 0;
            status->lat_faults    = 0u;  /* clear FAULT_ENDSTOP_HIT */
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
    status->endstop_mask        = g_endstop_mask;    /* from R31 directly    */
    status->sp_faults           = telem->sp_faults;
    /* status->lat_faults managed by endstop_tick(); do not overwrite here  */

    /* PRU1 state. */
    uint8_t st = PRU1_STATE_IDLE;
    if (g_homing != HOMING_IDLE)                       st |= PRU1_STATE_HOMING;
    if (params->sp_run || params->lat_run)             st |= PRU1_STATE_RUNNING;
    if (g_endstop_mask)                                st |= PRU1_STATE_AT_HOME;
    if (telem->sp_faults || status->lat_faults)        st |= PRU1_STATE_FAULT;
    status->pru1_state = st;
    /* Endstop events are raised in endstop_tick(); no duplicate check here */

    status->seq++;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Main loop
 * ════════════════════════════════════════════════════════════════════════════ */
int main(void) {
    /* PRU1 never touches IEP or motor pins. */

    /* Zero all shared memory areas that PRU0 owns or initialises.
     * CRITICAL: host_cmd must be zeroed to HOST_CMD_NOP before any command
     * processing begins.  Leftover non-zero values from a previous boot or
     * from the daemon writing before the firmware restarted would otherwise
     * be interpreted as a spurious command (e.g. cmd=0x44 = HALT the PRU). */
    *host_cmd  = (host_cmd_t){0};      /* zeroes cmd, cmd_ack, all fields     */
    *params    = (motor_params_t){0};
    *status    = (pru_status_t){0};
    *move_ctrl = (move_ctrl_t){0};
    /* Explicitly set sentinel values after zero. */
    host_cmd->cmd     = HOST_CMD_NOP;
    host_cmd->cmd_ack = HOST_CMD_NOP;

    uint32_t loop_cnt = 0u;

    while (1) {
        /* ── 1. Check host commands (throttled) ─────────────────────────── */
        if (loop_cnt % CMD_CHECK_STRIDE == 0u) {
            process_host_cmd();
        }

        /* ── 2. Read endstops + safety stop (throttled) ─────────────────── */
        if (loop_cnt % CMD_CHECK_STRIDE == 0u) {
            endstop_tick();
        }

        /* ── 3. Homing state machine ────────────────────────────────────── */
        if (loop_cnt % CMD_CHECK_STRIDE == 0u) {
            homing_tick();
        }

        /* ── 3b. Software limits check (throttled) ─────────────────────── */
        if (loop_cnt % CMD_CHECK_STRIDE == 0u) {
            limits_check();
        }
        /* ── 3c. Spindle-lateral coordination (throttled) ────────────────── */
        if (loop_cnt % CMD_CHECK_STRIDE == 0u) {
            coord_tick();
        }
        /* ── 3d. Move completion check (throttled) ─────────────────────── */
        if (loop_cnt % CMD_CHECK_STRIDE == 0u) {
            move_complete_check();
        }
        /* ── 4. Publish status (throttled) ──────────────────────────────── */
        if (loop_cnt % STATUS_STRIDE == 0u) {
            publish_status();
        }

        loop_cnt++;
    }

    return 0;
}
