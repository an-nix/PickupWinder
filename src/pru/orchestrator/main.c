/* orchestrator/main.c — PRU0 orchestrator firmware.
 *
 * Architecture layer 2/4.  Loaded as am335x-pru0-fw → remoteproc1 → PRU0
 * (4a334000.pru).
 *
 * PRU0 is the brain.  It is the only PRU that communicates with the host
 * (daemon).  Responsibilities:
 *   1. Poll host_cmd_t for new commands from the daemon.
 *   2. Manage ramp segments for BOTH axes (set_speed with ramp + move_to).
 *      Segment advance moved here from PRU1: PRU0 watches accel_count in
 *      telem and feeds segments one-by-one to PRU1 via ramp_arm.
 *   3. Endstop raw pin forwarding (NO homing logic — moved to host daemon).
 *      PRU0 reads R31 pins, writes raw values to pru_status_t, and sends
 *      EVENT_ENDSTOP_HIT / EVENT_ENDSTOP_CLEAR on debounced state change.
 *      PRU0 does NOT stop motors or interpret endstop state.
 *   4. Software limits check (lateral position bounds).
 *   5. Spindle-lateral speed coordination (Q6 ratio).
 *   6. Publish pru_status_t for daemon to broadcast.
 *
 * Removed from PRU0 (moved to host daemon):
 *   - Homing state machine (HOMING_IDLE/APPROACH/HIT) — moved to host daemon
 *   - Direct motor stop on endstop — moved to host daemon
 *   - HOME_START handler body (endstop hit → reset pos) — moved to host daemon
 *   - EVENT_HOME_COMPLETE raised by PRU0 — host infers from ENDSTOP_HIT
 *
 * Ramp model (unified for set_speed and move_to):
 *   The daemon always pre-computes ramp_seg_t[MAX_RAMP_SEGS] in shared RAM
 *   and sets seg_count in the command.  If seg_count==0 the interval is
 *   applied immediately.  If seg_count>0 PRU0 feeds segments to PRU1
 *   one-by-one via the ramp_arm handshake, watching telem->accel_count
 *   to detect when each segment completes.
 *
 * Rules:
 *   - PRU0 must NEVER call IEP_INIT() — PRU1 owns the IEP.
 *   - PRU0 must NOT write motor pins (R30 STEP/DIR/EN).
 *   - PRU0 writes motor_params_t; PRU1 reads it.
 *   - PRU0 reads motor_telem_t; PRU1 writes it every loop.
 *   - PRU0 reads R31 for endstop inputs (P9_28=R31[3], P9_30=R31[2]).
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_regs.h"       /* register volatile __R31 */
#include "../include/pru_stepper.h"    /* IEP_NOW() — read-only, PRU1 owns IEP */
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

/* Per-axis ramp segment arrays (ARM fills, PRU0 reads one-by-one).
 * Moved here from PRU1: PRU0 now owns all segment advance logic.           */
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

/* ── Endstop inputs (PRU0 R31) ───────────────────────────────────────────── *
 * Two-pin NO/NC sensor.  Both HIGH = inactive (or sensor absent).
 * ES1 = NO contact (P9_28 = R31[3], active-HIGH when triggered).
 * ES2 = NC contact (P9_30 = R31[2], active-HIGH when inactive, LOW when hit).
 *
 * PRU0 forwards raw pin values to pru_status_t every loop.
 * All interpretation (homing complete, fault, abort) is on the host daemon.
 *                                                                           */
#define ES_NO_BIT  (1u << 3)   /* R31[3]  P9_28  NO contact  (active-HIGH)  */
#define ES_NC_BIT  (1u << 2)   /* R31[2]  P9_30  NC contact  (active-HIGH)  */

/* Raw pin state (updated by endstop_tick every CMD_CHECK_STRIDE).          */
static uint8_t g_es_pin_no  = 1u;   /* default inactive (pull-up)           */
static uint8_t g_es_pin_nc  = 1u;   /* default inactive (pull-up)           */

/* Aggregated mask for backwards compatibility: bit0=ES1,bit1=ES2.
 * ES1 asserted when NO goes HIGH; ES2 asserted when NC goes LOW.           */
static uint8_t g_endstop_mask = 0u;

/* Debounce state — moved from PRU homing FSM to raw forwarding.            */
#define DEBOUNCE_CYCLES  20000u   /* ~100 µs at 200 MHz                     */
static uint8_t  g_es_prev_no       = 1u;
static uint8_t  g_es_prev_nc       = 1u;
static uint8_t  g_es_debounce_pend = 0u;
static uint32_t g_es_debounce_t0   = 0u;

/* ── Software limits (per-axis) ──────────────────────────────────────────── */
static int32_t  g_limit_min[2]      = {0, 0};
static int32_t  g_limit_max[2]      = {0, 0};
static uint8_t  g_limits_enabled[2]  = {0, 0};
static uint8_t  g_limit_locked[2]    = {0, 0};

/* ── Per-axis ramp state (moved from PRU1) ───────────────────────────────── */
#define RAMP_OP_NONE      0u
#define RAMP_OP_SET_SPEED 1u   /* speed ramp → EVENT_SPEED_REACHED          */
#define RAMP_OP_MOVE_TO   2u   /* position move → EVENT_MOVE_COMPLETE       */

static uint8_t  g_ramp_active[2]    = {0, 0};
static uint8_t  g_ramp_op[2]        = {RAMP_OP_NONE, RAMP_OP_NONE};
static uint32_t g_ramp_cruise_iv[2] = {0, 0};

/* Per-axis segment tracking (was g_ramping/g_seg_idx/g_seg_total in PRU1). */
static uint8_t  g_seg_ramping[2]    = {0, 0};
static uint8_t  g_seg_idx[2]        = {0, 0};
static uint8_t  g_seg_total[2]      = {0, 0};

/* Previous accel_count per axis — edge detection for segment completion.   */
static uint32_t g_prev_accel[2]     = {0u, 0u};

/* Move-to state (lateral axis). */
static uint8_t  g_lat_in_move       = 0u;

/* Spindle-lateral coordination.
 * g_sp_lat_coord: Q6 ratio = (sp_iv × 64) / lat_cruise_iv.
 *   Non-zero while coordination is active.  Cleared by set_speed or estop.
 * g_sp_requested_iv: last spindle speed set by host (restored on stop).    */
static uint32_t g_sp_lat_coord      = 0u;
static uint32_t g_sp_requested_iv   = 0u;

/* ── Helper: acknowledge host command ────────────────────────────────────── */
static inline void ack_cmd(uint8_t opcode) {
    host_cmd->cmd_ack = opcode;
    host_cmd->cmd     = HOST_CMD_NOP;
}

/* ── Helper: cancel ramp on an axis ──────────────────────────────────────── */
static void cancel_ramp(uint8_t ax) {
    g_ramp_active[ax]             = 0u;
    g_ramp_op[ax]                 = RAMP_OP_NONE;
    g_seg_ramping[ax]             = 0u;
    params->motor[ax].ramp_arm    = 0u;
    params->motor[ax].ramp_count  = 0u;
}

/* ── Helper: arm first segment of a ramp on an axis ─────────────────────── *
 * Writes seg[0] fields from shared RAM into motor_ctl_t and sets ramp_arm=1.
 * PRU1 arms pulse_gen for this ONE segment and clears ramp_arm=0.
 * PRU0 watches telem->motor[ax].accel_count to detect when it reaches 0
 * (segment complete), then calls arm_next_seg() for the next segment.      */
static void arm_ramp(uint8_t ax, uint8_t n_segs) {
    volatile ramp_seg_t *seg0 = &g_ramp_segs[ax][0];
    params->motor[ax].interval   = seg0->start_iv;
    params->motor[ax].ramp_add   = seg0->add;
    params->motor[ax].ramp_count = seg0->count;
    params->motor[ax].ramp_arm   = 1u;          /* ONE segment per arm_ramp */
    g_seg_idx[ax]     = 0u;
    g_seg_total[ax]   = n_segs;
    g_seg_ramping[ax] = 1u;
    g_prev_accel[ax]  = seg0->count;            /* initialise edge detector */
}

/* ── Endstop tick: read R31, forward raw state, debounce, send event ─────── *
 * PRU0 forwards raw NO/NC pin values to pru_status_t EVERY tick.
 * On debounced state change → raises EVENT_ENDSTOP_HIT or _CLEAR.
 *
 * PRU0 does NOT:
 *   - Stop any motor on endstop.   (moved to host daemon)
 *   - Interpret NO/NC polarity.    (moved to host daemon)
 *   - Run a homing state machine.  (moved to host daemon)
 */
static void endstop_tick(void) {
    uint32_t r31   = __R31;
    uint8_t  no    = (r31 & ES_NO_BIT) ? 1u : 0u;  /* raw NO pin value     */
    uint8_t  nc    = (r31 & ES_NC_BIT) ? 1u : 0u;  /* raw NC pin value     */

    /* Always forward raw pin state to status so daemon can poll it.        */
    g_es_pin_no = no;
    g_es_pin_nc = nc;

    /* Aggregate mask for legacy endstop_mask field:
     *   ES1 asserted when NO is HIGH (contact closed).
     *   ES2 asserted when NC is LOW  (contact opened = triggered).
     * This is kept for the software limits check which uses g_endstop_mask. */
    g_endstop_mask = 0u;
    if (no)  g_endstop_mask |= ENDSTOP1_MASK;
    if (!nc) g_endstop_mask |= ENDSTOP2_MASK;

    /* ── Debounce + event ───────────────────────────────────────────────── */
    if (no != g_es_prev_no || nc != g_es_prev_nc) {
        /* New transient — start debounce window if not already pending.    */
        if (!g_es_debounce_pend) {
            g_es_debounce_t0   = IEP_NOW();  /* IEP_NOW() used read-only    */
            g_es_debounce_pend = 1u;
        }
    }

    if (g_es_debounce_pend) {
        uint32_t elapsed = IEP_NOW() - g_es_debounce_t0;
        if (elapsed >= DEBOUNCE_CYCLES) {
            /* Confirm state still differs from last stable values.         */
            if (no != g_es_prev_no || nc != g_es_prev_nc) {
                g_es_prev_no = no;
                g_es_prev_nc = nc;

                /* Choose event type based on whether sensor is triggered.  *
                 * Both pins HIGH = inactive or sensor absent.              */
                if (!status->event_pending) {
                    uint8_t triggered = (no == 1u || nc == 0u) ? 1u : 0u;
                    status->event_type    = triggered
                                           ? EVENT_ENDSTOP_HIT
                                           : EVENT_ENDSTOP_CLEAR;
                    status->event_pending = 1u;
                }
            }
            g_es_debounce_pend = 0u;
        }
    }
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
    /* Homing state removed — moved to host daemon */
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
                 * Snap seg[0].start_iv to current interval if running
                 * to avoid a speed jump at ramp start.                     */
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
                arm_ramp(MOTOR_0, n_segs);
            } else {
                /* Direct speed change (already at target, or stopped). */
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
        /* Homing logic moved to host daemon.
         * PRU0 now only starts lateral movement at the specified interval
         * and direction (provided by the daemon in motor[MOTOR_1] fields).
         * The daemon monitors EVENT_ENDSTOP_HIT to determine home complete.
         * All error handling (timeout, absent sensor) is on the daemon.    */
        {
            uint32_t lat_iv  = host_cmd->motor[MOTOR_1].interval_target;
            uint8_t  lat_dir = host_cmd->motor[MOTOR_1].dir;
            if (lat_iv > 0u) {
                cancel_ramp(MOTOR_1);
                params->motor[MOTOR_1].interval = lat_iv;
                params->motor[MOTOR_1].dir      = lat_dir;
                params->motor[MOTOR_1].enable   = 1u;
                params->motor[MOTOR_1].run      = 1u;
                params->motor[MOTOR_0].run      = 0u;  /* stop spindle */
            }
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
        g_ramp_op[MOTOR_1]     = RAMP_OP_MOVE_TO;
        g_ramp_active[MOTOR_1] = 1u;
        g_lat_in_move          = 1u;
        arm_ramp(MOTOR_1, n);

        ack_cmd(cmd);
        break;
    }

    default:
        ack_cmd(cmd);
        break;
    }
}

/* ── Ramp segment advance tick (moved from PRU1) ─────────────────────────── *
 * PRU1 is now a constant-time pulse generator: it arms ONE segment per      *
 * ramp_arm write and clears ramp_arm=0 when done.  PRU0 detects completion  *
 * by watching telem->motor[ax].accel_count reach 0 (written every PRU1 loop)*
 * and feeds the next segment immediately.                                   */
static void ramp_tick(void) {
    uint8_t ax;
    for (ax = 0u; ax < 2u; ax++) {
        if (!g_ramp_active[ax]) continue;
        if (!g_seg_ramping[ax]) continue;
        if (params->motor[ax].ramp_arm) continue; /* PRU1 not yet done with arm */

        uint32_t ac = telem->motor[ax].accel_count;

        /* Edge: accel_count just reached 0 (was non-zero last tick).       */
        if (ac == 0u && g_prev_accel[ax] != 0u) {
            g_seg_idx[ax]++;

            if (g_seg_idx[ax] < g_seg_total[ax]) {
                /* More segments: load next one into PRU1 via ramp_arm.     */
                volatile ramp_seg_t *s = &g_ramp_segs[ax][g_seg_idx[ax]];
                params->motor[ax].interval   = s->start_iv;
                params->motor[ax].ramp_add   = s->add;
                params->motor[ax].ramp_count = s->count;
                params->motor[ax].ramp_arm   = 1u;   /* trigger PRU1       */
                g_prev_accel[ax] = s->count;
            } else {
                /* All segments done — write cruise interval and fire event. */
                g_seg_ramping[ax]  = 0u;
                g_ramp_active[ax]  = 0u;
                uint8_t op = g_ramp_op[ax];
                g_ramp_op[ax] = RAMP_OP_NONE;

                if (op == RAMP_OP_MOVE_TO) {
                    params->motor[ax].run = 0u;
                    if (ax == MOTOR_1) g_lat_in_move = 0u;
                    /* Keep g_sp_lat_coord: spindle stays proportional during
                     * the reversal gap waiting for the next move_to.       */
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

        g_prev_accel[ax] = ac;
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

/* ── Publish aggregated status ───────────────────────────────────────────── *
 * Uses status_motor_t (24B, no accel_count) for the status array.           */
static void publish_status(void) {
    /* Copy telemetry fields to status_motor_t[2] (subset of motor_t).     */
    status->motor[MOTOR_0].step_count      = telem->motor[MOTOR_0].step_count;
    status->motor[MOTOR_0].position        = telem->motor[MOTOR_0].position;
    status->motor[MOTOR_0].interval_actual = telem->motor[MOTOR_0].interval_actual;
    status->motor[MOTOR_0].state           = telem->motor[MOTOR_0].state;
    status->motor[MOTOR_0].faults          = telem->motor[MOTOR_0].faults;

    status->motor[MOTOR_1].step_count      = telem->motor[MOTOR_1].step_count;
    status->motor[MOTOR_1].position        = telem->motor[MOTOR_1].position;
    status->motor[MOTOR_1].interval_actual = telem->motor[MOTOR_1].interval_actual;
    status->motor[MOTOR_1].state           = telem->motor[MOTOR_1].state;
    /* motor[MOTOR_1].faults managed by limits_check — preserve */

    /* Raw endstop pin values forwarded from endstop_tick().
     * Daemon reads these to drive homing/fault logic.                      */
    status->endstop_mask    = g_endstop_mask;
    status->endstop_pin_no  = g_es_pin_no;
    status->endstop_pin_nc  = g_es_pin_nc;

    /* PRU1 state word. */
    uint8_t st = PRU1_STATE_IDLE;
    /* Homing flag removed — homing is now host-side.
     * PRU1_STATE_HOMING bit unused; kept for compat but not set.           */
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

    /* Initialise per-axis ramp segment pointers (shared RAM, ARM-filled).  */
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
            /* homing_tick() removed — homing FSM moved to host daemon      */
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
