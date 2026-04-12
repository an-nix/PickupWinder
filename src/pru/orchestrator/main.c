/* orchestrator/main.c — PRU0 orchestrator firmware.
 *
 * Architecture layer 2/4.  Loaded as am335x-pru0-fw → remoteproc1 → PRU0
 * (4a334000.pru).
 *
 * PRU0 is the brain.  It is the only PRU that communicates with the host
 * (daemon).  Responsibilities:
 *   1. Poll host_cmd_t for new commands from the daemon.
 *   2. Manage ramp segments for BOTH axes (set_speed with ramp + move_to).
 *   3. Homing state machine (lateral endstop approach).
 *   4. Software limits check (lateral position bounds).
 *   5. Spindle-lateral speed coordination (Q6 ratio).
 *   6. Publish pru_status_t for daemon to broadcast.
 *
 * Ramp model (unified for set_speed and move_to):
 *   The daemon always pre-computes ramp_seg_t[MAX_RAMP_SEGS] in shared RAM
 *   and sets seg_count in the command.  If seg_count==0 the interval is
 *   applied immediately (e.g. estop, already-at-speed).  If seg_count>0
 *   PRU0 feeds segments to PRU1 one-by-one via the ramp_arm handshake.
 *   There is NO separate RAMP_TO opcode — ramping IS set_speed.
 *
 * Rules:
 *   - PRU0 must NEVER call IEP_INIT() — PRU1 owns the IEP.
 *   - PRU0 must NOT write motor pins (R30 STEP/DIR/EN).
 *   - PRU0 writes motor_params_t; PRU1 reads it.
 *   - PRU0 reads motor_telem_t; PRU1 writes it.
 *   - PRU0 reads R31 for endstop inputs (P9_28=R31[3], P9_30=R31[2]).
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_regs.h"       /* register volatile __R31 */
#include "../include/pru_rsc_table.h"  /* required by remoteproc  */

/* ── Shared RAM pointers ─────────────────────────────────────────────────── */
static volatile host_cmd_t     *host_cmd =
    (volatile host_cmd_t *)    (PRU_SRAM_PHYS_BASE + IPC_HOST_CMD_OFFSET);

static volatile motor_params_t *params =
    (volatile motor_params_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_PARAMS_OFFSET);

static volatile motor_telem_t  *telem =
    (volatile motor_telem_t *) (PRU_SRAM_PHYS_BASE + IPC_MOTOR_TELEM_OFFSET);

static volatile pru_status_t   *status =
    (volatile pru_status_t *)  (PRU_SRAM_PHYS_BASE + IPC_PRU_STATUS_OFFSET);

/* Per-axis ramp segment arrays (ARM fills, PRU0 reads one-by-one). */
static volatile ramp_seg_t     *g_ramp_segs[2];

/* ── Cadence ─────────────────────────────────────────────────────────────── */
/* Power-of-2 strides → compiler uses bitwise AND instead of software
 * division (__pruabi_remu).  PRU has no hardware divider; the modulo
 * operator on non-power-of-2 values costs hundreds of cycles per call.
 *
 *   CMD_CHECK_STRIDE  = 1024  → check host cmd every ~5 µs
 *   STATUS_STRIDE     = 524288 → publish status every ~2.6 ms           */
#define CMD_CHECK_STRIDE    1024u
#define STATUS_STRIDE       524288u

/* ── Endstop inputs (PRU0 R31) ───────────────────────────────────────────── */
#define ES1_BIT  (1u << 3)   /* R31[3]  P9_28  ENDSTOP_1  (active-HIGH)     */
#define ES2_BIT  (1u << 2)   /* R31[2]  P9_30  ENDSTOP_2  (active-HIGH)     */

static uint8_t g_endstop_mask      = 0u;
static uint8_t g_prev_endstop_mask = 0u;

/* ── Software limits (per-axis) ──────────────────────────────────────────── */
static int32_t  g_limit_min[2]      = {0, 0};
static int32_t  g_limit_max[2]      = {0, 0};
static uint8_t  g_limits_enabled[2]  = {0, 0};
static uint8_t  g_limit_locked[2]    = {0, 0};

/* ── Per-axis ramp state ─────────────────────────────────────────────────── */
#define RAMP_OP_NONE      0u
#define RAMP_OP_SET_SPEED 1u   /* speed ramp → EVENT_SPEED_REACHED          */
#define RAMP_OP_MOVE_TO   2u   /* position move → EVENT_MOVE_COMPLETE       */

static uint8_t  g_ramp_active[2]    = {0, 0};
static uint8_t  g_ramp_op[2]        = {RAMP_OP_NONE, RAMP_OP_NONE};
static uint32_t g_ramp_cruise_iv[2] = {0, 0};

/* Move-to state (lateral axis). */
static uint8_t  g_lat_in_move       = 0u;

/* Spindle-lateral coordination.
 * g_sp_lat_coord: Q6 ratio = (sp_iv × 64) / lat_cruise_iv.
 *   Non-zero while coordination is active.  Cleared by set_speed or estop.
 * g_sp_requested_iv: last spindle speed set by host (restored on stop).    */
static uint32_t g_sp_lat_coord      = 0u;
static uint32_t g_sp_requested_iv   = 0u;

/* ── Homing state machine ────────────────────────────────────────────────── */
typedef enum {
    HOMING_IDLE     = 0,
    HOMING_APPROACH = 1,
    HOMING_HIT      = 2,
} homing_state_t;

static homing_state_t g_homing = HOMING_IDLE;
static uint32_t g_saved_lat_interval = 0u;
static uint8_t  g_saved_lat_dir      = 0u;

#define HOMING_INTERVAL  16276u   /* ~2 mm/s at 3072 steps/mm               */
#define HOMING_DIR       1u       /* direction toward home sensor            */

/* ── Helper: acknowledge host command ────────────────────────────────────── */
static inline void ack_cmd(uint8_t opcode) {
    host_cmd->cmd_ack = opcode;
    host_cmd->cmd     = HOST_CMD_NOP;
}

/* ── Helper: cancel ramp on an axis ──────────────────────────────────────── */
static void cancel_ramp(uint8_t ax) {
    g_ramp_active[ax]             = 0u;
    g_ramp_op[ax]                 = RAMP_OP_NONE;
    params->motor[ax].ramp_arm    = 0u;
    params->motor[ax].ramp_count  = 0u;
}

/* ── Helper: arm ramp on an axis ───────────────────────────────────────────── *
 * Writes seg[0] fields (interval, add, count) from shared RAM into
 * motor_ctl_t, then sets ramp_arm = n_segs (total segment count).
 *
 * PRU1 reads ramp_arm as the total count and self-advances through all
 * segments by reading g_ramp_segs directly.  PRU0 is not involved in any
 * per-segment handshake — it only waits for the final seg_done=1.          */
static void arm_ramp(uint8_t ax, uint8_t n_segs) {
    volatile ramp_seg_t *seg0 = &g_ramp_segs[ax][0];
    params->motor[ax].interval   = seg0->start_iv;
    params->motor[ax].ramp_add   = seg0->add;
    params->motor[ax].ramp_count = seg0->count;
    params->motor[ax].ramp_arm   = n_segs;   /* triggers PRU1; value = total segs */
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
        params->motor[MOTOR_1].run = 0u;
        cancel_ramp(MOTOR_1);
        g_lat_in_move = 0u;

        /* Restore spindle from coordination to last requested speed. */
        if (g_sp_lat_coord != 0u && g_sp_requested_iv > 0u)
            params->motor[MOTOR_0].interval = g_sp_requested_iv;
        g_sp_lat_coord = 0u;

        status->motor[MOTOR_1].faults |= FAULT_ENDSTOP_HIT;

        if (g_homing == HOMING_IDLE && !status->event_pending) {
            status->event_type    = EVENT_ENDSTOP_HIT;
            status->event_pending = 1u;
        }
    }
    g_prev_endstop_mask = es;
}

/* ── Software limits check ───────────────────────────────────────────────── */
static void limits_check(void) {
    if (!g_limits_enabled[AXIS_LATERAL] || g_limit_locked[AXIS_LATERAL]) return;
    int32_t pos = telem->motor[MOTOR_1].position;
    if (pos < g_limit_min[AXIS_LATERAL] || pos > g_limit_max[AXIS_LATERAL]) {
        params->motor[MOTOR_1].run    = 0u;
        params->motor[MOTOR_1].enable = 0u;
        cancel_ramp(MOTOR_1);
        status->motor[MOTOR_1].faults |= FAULT_OVERRUN;
        if (!status->event_pending) {
            status->event_type    = EVENT_LIMIT_HIT;
            status->event_pending = 1u;
        }
        g_limit_locked[AXIS_LATERAL] = 1u;
        g_lat_in_move = 0u;
        if (g_sp_lat_coord != 0u && g_sp_requested_iv > 0u)
            params->motor[MOTOR_0].interval = g_sp_requested_iv;
        g_sp_lat_coord = 0u;
    }
}

/* ── Emergency stop ──────────────────────────────────────────────────────── */
static void do_estop(void) {
    params->motor[MOTOR_0].run    = 0u;
    params->motor[MOTOR_1].run    = 0u;
    params->motor[MOTOR_0].enable = 0u;
    params->motor[MOTOR_1].enable = 0u;
    params->motor[MOTOR_0].interval = 0u;
    params->motor[MOTOR_1].interval = 0u;
    cancel_ramp(MOTOR_0);
    cancel_ramp(MOTOR_1);
    g_homing       = HOMING_IDLE;
    g_lat_in_move  = 0u;
    g_sp_lat_coord = 0u;
}

/* ── Process one host command ────────────────────────────────────────────── */
static void process_host_cmd(void) {
    uint8_t cmd = host_cmd->cmd;
    if (cmd == HOST_CMD_NOP) return;

    switch (cmd) {

    case HOST_CMD_SET_SPEED: {
        uint8_t  ax     = host_cmd->axis;   /* AXIS_SPINDLE, AXIS_ALL, etc. */
        uint8_t  n_segs = host_cmd->seg_count;

        /* Speed command always disables spindle-lateral coordination. */
        g_sp_lat_coord    = 0u;

        /* ── Spindle axis ─────────────────────────────────────────── */
        if (ax == AXIS_SPINDLE || ax == AXIS_ALL) {
            uint32_t sp_iv  = host_cmd->motor[MOTOR_0].interval_target;
            uint8_t  sp_dir = host_cmd->motor[MOTOR_0].dir;

            g_sp_requested_iv = sp_iv;
            cancel_ramp(MOTOR_0);
            params->motor[MOTOR_0].dir = sp_dir;

            if (n_segs > 0u) {
                /* Ramp: segments pre-loaded in shared RAM by daemon.
                 * If the motor is already running, snap the first segment's
                 * start_iv to the actual current interval so there is no
                 * speed jump when cancel_ramp + arm_ramp happens.
                 *
                 * We only patch start_iv and set add=0 (coast at cur_iv for
                 * seg[0].count steps).  The next segment boundary force-loads
                 * seg[1].start_iv.  This avoids a division on the PRU (no
                 * hardware divider — calling __pruabi_divi corrupted the
                 * return address via the unmapped stack before the linker
                 * memory fix).                                               */
                uint32_t cur_iv = params->motor[MOTOR_0].interval;
                if (cur_iv >= SP_IV_MIN && cur_iv <= SP_IV_MAX
                    && params->motor[MOTOR_0].run) {
                    volatile ramp_seg_t *seg0 = &g_ramp_segs[MOTOR_0][0];
                    seg0->start_iv = cur_iv;
                    seg0->add      = 0;
                }
                params->motor[MOTOR_0].enable = 1u;
                params->motor[MOTOR_0].run    = 1u;
                g_ramp_cruise_iv[MOTOR_0] = host_cmd->cruise_iv;
                g_ramp_op[MOTOR_0]        = RAMP_OP_SET_SPEED;
                g_ramp_active[MOTOR_0]    = 1u;
                telem->motor[MOTOR_0].seg_done = 0u;
                arm_ramp(MOTOR_0, n_segs);
            } else {
                /* Direct speed change (already at target, or stopped).
                 * Always re-enable: the daemon may send set_speed right after
                 * an estop, before the enable command is processed by PRU0. */
                params->motor[MOTOR_0].interval = sp_iv;
                if (sp_iv > 0u) {
                    params->motor[MOTOR_0].enable = 1u;
                    params->motor[MOTOR_0].run    = 1u;
                } else {
                    params->motor[MOTOR_0].run = 0u;
                }
            }
        }

        /* ── Lateral axis (only when no move_to is active) ───────── */
        if ((ax == AXIS_LATERAL || ax == AXIS_ALL) && !g_lat_in_move) {
            uint32_t lat_iv  = host_cmd->motor[MOTOR_1].interval_target;
            uint8_t  lat_dir = host_cmd->motor[MOTOR_1].dir;
            cancel_ramp(MOTOR_1);
            params->motor[MOTOR_1].interval = lat_iv;
            params->motor[MOTOR_1].dir      = lat_dir;
            /* Always re-enable lateral axis on explicit set_speed command. */
            if (lat_iv > 0u) {
                params->motor[MOTOR_1].enable = 1u;
                params->motor[MOTOR_1].run    = 1u;
            } else {
                params->motor[MOTOR_1].run = 0u;
            }
        }
        ack_cmd(cmd);
        break;
    }

    case HOST_CMD_ENABLE: {
        uint8_t en = (host_cmd->value_a != 0u) ? 1u : 0u;
        uint8_t ax = host_cmd->axis;
        if (ax == AXIS_SPINDLE || ax == AXIS_ALL) {
            params->motor[MOTOR_0].enable = en;
            if (!en) params->motor[MOTOR_0].run = 0u;
        }
        if (ax == AXIS_LATERAL || ax == AXIS_ALL) {
            if (en && (g_limit_locked[AXIS_LATERAL] || g_lat_in_move)) {
                /* Ignore enable while locked or move in progress. */
            } else {
                params->motor[MOTOR_1].enable = en;
                if (!en) params->motor[MOTOR_1].run = 0u;
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
            g_saved_lat_interval = params->motor[MOTOR_1].interval;
            g_saved_lat_dir      = params->motor[MOTOR_1].dir;
            params->motor[MOTOR_1].interval = HOMING_INTERVAL;
            params->motor[MOTOR_1].dir      = HOMING_DIR;
            params->motor[MOTOR_1].enable   = 1u;
            params->motor[MOTOR_1].run      = 1u;
            params->motor[MOTOR_0].run      = 0u;  /* stop spindle */
            g_homing = HOMING_APPROACH;
        }
        ack_cmd(cmd);
        break;

    case HOST_CMD_ACK_EVENT:
        status->event_pending = 0u;
        status->event_type    = EVENT_NONE;
        g_limit_locked[AXIS_SPINDLE] = 0u;
        g_limit_locked[AXIS_LATERAL] = 0u;
        status->motor[MOTOR_0].faults = 0u;
        status->motor[MOTOR_1].faults = 0u;
        ack_cmd(cmd);
        break;

    case HOST_CMD_RESET_POS:
        telem->motor[MOTOR_0].step_count = 0u;
        telem->motor[MOTOR_1].step_count = 0u;
        telem->motor[MOTOR_1].position   = 0;
        telem->motor[MOTOR_0].faults     = 0u;
        telem->motor[MOTOR_1].faults     = 0u;
        ack_cmd(cmd);
        break;

    case HOST_CMD_SET_LIMITS: {
        uint8_t ax = host_cmd->axis;
        if (ax == AXIS_SPINDLE || ax == AXIS_LATERAL) {
            g_limit_min[ax]      = host_cmd->limit_min;
            g_limit_max[ax]      = host_cmd->limit_max;
            g_limits_enabled[ax] = 1u;
            g_limit_locked[ax]   = 0u;
        }
        ack_cmd(cmd);
        break;
    }

    case HOST_CMD_MOVE_TO: {
        /* Lateral move using compressed ramp segments from ARM planner. */
        if (g_limit_locked[AXIS_LATERAL]) { ack_cmd(cmd); break; }
        uint8_t n = host_cmd->seg_count;
        if (n == 0u) { ack_cmd(cmd); break; }

        int32_t delta   = host_cmd->move_target - telem->motor[MOTOR_1].position;
        uint8_t new_dir = (delta < 0) ? 1u : 0u;

        params->motor[MOTOR_1].dir    = new_dir;
        params->motor[MOTOR_1].enable = 1u;
        params->motor[MOTOR_1].run    = 1u;

        /* Store Q6 spindle-lateral coordination ratio. */
        g_sp_lat_coord = host_cmd->move_sp_lat_coord;
        g_ramp_cruise_iv[MOTOR_1] = host_cmd->cruise_iv;

        /* Set spindle to proportional starting speed immediately. */
        if (g_sp_lat_coord != 0u && params->motor[MOTOR_0].enable) {
            uint32_t lat_ref = params->motor[MOTOR_1].run
                             ? telem->motor[MOTOR_1].interval_actual
                             : g_ramp_segs[MOTOR_1][0].start_iv;
            uint32_t sp_init = (g_sp_lat_coord * lat_ref) >> 6u;
            if (sp_init < SP_IV_MIN) sp_init = SP_IV_MIN;
            if (sp_init > SP_IV_MAX) sp_init = SP_IV_MAX;
            params->motor[MOTOR_0].interval = sp_init;
            params->motor[MOTOR_0].run      = 1u;
        }

        /* Start segment execution on MOTOR_1. */
        g_ramp_op[MOTOR_1]        = RAMP_OP_MOVE_TO;
        g_ramp_active[MOTOR_1]    = 1u;
        g_lat_in_move             = 1u;
        telem->motor[MOTOR_1].seg_done = 0u;
        arm_ramp(MOTOR_1, n);

        ack_cmd(cmd);
        break;
    }

    default:
        ack_cmd(cmd);
        break;
    }
}

/* ── Ramp completion tick ─────────────────────────────────────────────────── *
 * PRU1 now self-advances through all ramp segments without any PRU0 per-
 * segment handshake.  PRU1 sets seg_done=1 only once ALL segments complete.
 * PRU0's only job is to detect that final flag and fire the event.          */
static void ramp_tick(void) {
    uint8_t ax;
    for (ax = 0u; ax < 2u; ax++) {
        if (!g_ramp_active[ax]) continue;
        if (!telem->motor[ax].seg_done) continue;

        /* All segments done. */
        telem->motor[ax].seg_done = 0u;
        g_ramp_active[ax] = 0u;
        uint8_t op = g_ramp_op[ax];
        g_ramp_op[ax] = RAMP_OP_NONE;

        if (op == RAMP_OP_MOVE_TO) {
            params->motor[ax].run = 0u;
            if (ax == MOTOR_1) g_lat_in_move = 0u;
            /* g_sp_lat_coord kept: spindle stays proportional during reversal gap. */
            if (!status->event_pending) {
                status->event_type    = EVENT_MOVE_COMPLETE;
                status->event_pending = 1u;
            }
        } else if (op == RAMP_OP_SET_SPEED) {
            params->motor[ax].interval = g_ramp_cruise_iv[ax];
            if (!status->event_pending) {
                status->event_type    = EVENT_SPEED_REACHED;
                status->event_pending = 1u;
            }
        }
    }
}

/* ── Spindle-lateral speed coordination ──────────────────────────────────── */
static void coord_tick(void) {
    if (g_sp_lat_coord == 0u) return;
    uint32_t lat_iv = telem->motor[MOTOR_1].interval_actual;
    if (lat_iv == 0u) return;
    uint32_t sp_adj = (g_sp_lat_coord * lat_iv) >> 6u;
    if (sp_adj < SP_IV_MIN) sp_adj = SP_IV_MIN;
    if (sp_adj > SP_IV_MAX) sp_adj = SP_IV_MAX;
    params->motor[MOTOR_0].interval = sp_adj;
    if (params->motor[MOTOR_0].enable && sp_adj > 0u)
        params->motor[MOTOR_0].run = 1u;
}

/* ── Homing state machine ────────────────────────────────────────────────── */
static void homing_tick(void) {
    if (g_homing == HOMING_IDLE) return;
    uint8_t es = g_endstop_mask;

    switch (g_homing) {
    case HOMING_APPROACH:
        if (es & (ENDSTOP1_MASK | ENDSTOP2_MASK)) {
            params->motor[MOTOR_1].run = 0u;
            telem->motor[MOTOR_1].step_count = 0u;
            telem->motor[MOTOR_1].position   = 0;
            status->motor[MOTOR_1].faults    = 0u;
            g_homing = HOMING_HIT;
        }
        break;
    case HOMING_HIT:
        params->motor[MOTOR_1].interval = g_saved_lat_interval;
        params->motor[MOTOR_1].dir      = g_saved_lat_dir;
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
    /* Copy telemetry fields selectively. */
    status->motor[MOTOR_0].step_count      = telem->motor[MOTOR_0].step_count;
    status->motor[MOTOR_0].position        = telem->motor[MOTOR_0].position;
    status->motor[MOTOR_0].interval_actual = telem->motor[MOTOR_0].interval_actual;
    status->motor[MOTOR_0].state           = telem->motor[MOTOR_0].state;
    status->motor[MOTOR_0].faults          = telem->motor[MOTOR_0].faults;

    status->motor[MOTOR_1].step_count      = telem->motor[MOTOR_1].step_count;
    status->motor[MOTOR_1].position        = telem->motor[MOTOR_1].position;
    status->motor[MOTOR_1].interval_actual = telem->motor[MOTOR_1].interval_actual;
    status->motor[MOTOR_1].state           = telem->motor[MOTOR_1].state;
    /* motor[MOTOR_1].faults managed by endstop_tick/limits_check — preserve */

    status->endstop_mask = g_endstop_mask;

    /* PRU1 state word. */
    uint8_t st = PRU1_STATE_IDLE;
    if (g_homing != HOMING_IDLE)
        st |= PRU1_STATE_HOMING;
    if (params->motor[MOTOR_0].run || params->motor[MOTOR_1].run)
        st |= PRU1_STATE_RUNNING;
    if (g_endstop_mask)
        st |= PRU1_STATE_AT_HOME;
    if (telem->motor[MOTOR_0].faults || status->motor[MOTOR_1].faults)
        st |= PRU1_STATE_FAULT;
    status->pru1_state = st;

    status->seq++;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Main loop
 * ════════════════════════════════════════════════════════════════════════════ */
int main(void) {
    /* PRU0 never touches IEP or motor pins. */

    /* Zero all shared memory areas that PRU0 owns or initialises.
     * CRITICAL: host_cmd must be zeroed to HOST_CMD_NOP to prevent
     * spurious commands from leftover shared RAM content. */
    *host_cmd = (host_cmd_t){0};
    *params   = (motor_params_t){0};
    *status   = (pru_status_t){0};
    host_cmd->cmd     = HOST_CMD_NOP;
    host_cmd->cmd_ack = HOST_CMD_NOP;

    /* Initialise per-axis ramp segment pointers. */
    g_ramp_segs[MOTOR_0] =
        (volatile ramp_seg_t *)(PRU_SRAM_PHYS_BASE + IPC_RAMP_SEGS_0_OFFSET);
    g_ramp_segs[MOTOR_1] =
        (volatile ramp_seg_t *)(PRU_SRAM_PHYS_BASE + IPC_RAMP_SEGS_1_OFFSET);

    uint32_t cmd_cnt    = 0u;
    uint32_t status_cnt = 0u;

    while (1) {
        if (++cmd_cnt >= CMD_CHECK_STRIDE) {
            cmd_cnt = 0u;
            process_host_cmd();
            endstop_tick();
            homing_tick();
            limits_check();
            ramp_tick();
            coord_tick();
        }

        if (++status_cnt >= STATUS_STRIDE) {
            status_cnt = 0u;
            publish_status();
        }
    }

    return 0;
}
