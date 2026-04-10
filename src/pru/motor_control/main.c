/* motor_control/main.c — motor firmware (runs on PRU1 at runtime).
 *
 * Architecture layer 1/4.  This firmware runs on PRU1 at runtime
 * (loaded as am335x-pru0-fw by the PRU loader).
 *
 * PRU0 (motor firmware) is a dumb pulse generator. It knows nothing about
 * homing, recipes, synchronization, commands, or endstops. Its sole
 * responsibilities:
 *
 *   1. Read motor_params_t (written by PRU1/orchestrator) every iteration.
 *   2. Generate STEP/DIR pulses using the IEP hardware counter.
 *   3. Publish motor_telem_t for PRU1 (and host) to read.
 *
 * PRU0 owns and initializes the IEP timer. PRU1 may read IEP but never
 * resets it.
 *
 * Pin mapping on PRU1 R30 (active-low enable) — GPMC pins in MODE6:
 *   Motor A (spindle): STEP=R30[1] (P8_45), DIR=R30[5] (P8_43), EN=R30[7] (P8_41)
 *   Motor B (lateral): STEP=R30[2] (P8_46), DIR=R30[0] (P8_44), EN=R30[3] (P8_42)
 *
 * Endstops are read by PRU0 (orchestrator) via its R31 (P9_28/P9_30).
 * This firmware does NOT read R31 or perform any endstop logic.
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_stepper.h"
#include "../include/pru_regs.h"

/* ── Pin bitmasks ────────────────────────────────────────────────────────── */
#define SP_STEP_BIT    (1u << 1)   /* P8_45 R30[1] */
#define SP_DIR_BIT     (1u << 5)   /* P8_43 R30[5] */
#define SP_EN_BIT      (1u << 7)   /* P8_41 R30[7] active-low */

#define LAT_STEP_BIT   (1u << 2)   /* P8_46 R30[2] */
#define LAT_DIR_BIT    (1u << 0)   /* P8_44 R30[0] */
#define LAT_EN_BIT     (1u << 3)   /* P8_42 R30[3] active-low */

/* ── Telemetry publish cadence ───────────────────────────────────────────── */
#define TELEM_STRIDE   500000u     /* ~2.5 ms at 200 MHz (400 Hz) */

/* ── Trapezoidal move profile phases ────────────────────────────────────── */
#define MOVE_IDLE    0u
#define MOVE_ACCEL   1u
#define MOVE_CRUISE  2u
#define MOVE_DECEL   3u

/* ── Shared RAM pointers ─────────────────────────────────────────────────── */
static volatile motor_params_t *params =
    (volatile motor_params_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_PARAMS_OFFSET);

static volatile motor_telem_t *telem =
    (volatile motor_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_TELEM_OFFSET);

/* ── Pulse generators ────────────────────────────────────────────────────── */
static pulse_gen_t spindle = {0};
static pulse_gen_t lateral = {0};

/* ── Trapezoidal move profile state (lateral axis) ──────────────────────── */
static uint8_t  g_lat_phase        = MOVE_IDLE;
static int32_t  g_lat_target       = 0;
static uint32_t g_lat_start_iv     = 0u;
static uint32_t g_lat_cruise_iv    = 0u;
static uint32_t g_lat_delta_iv     = 0u;
static uint32_t g_lat_accel_steps  = 0u; /* steps taken during accel phase  */
static uint8_t  g_prev_lat_pin     = 0u; /* for rising-edge detection        */

/* ── Integer abs (no stdlib) ───────────────────────────────────────────── */
static inline uint32_t u32abs(int32_t v) {
    return (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
}

/* ── Arm a new move_to profile ───────────────────────────────────────────── *
 * Called once when params->lat_move_active transitions to 1.               *
 * Reads profile params from motor_params_t, derives direction from sign of  *
 * (target - current_position), enables & runs lateral motor.               */
static void lat_move_arm(void) {
    int32_t delta = params->lat_target_pos - lateral.position;
    if (delta == 0) {
        telem->lat_move_done    = 1u;
        params->lat_move_active = 0u;
        return;
    }

    uint8_t new_dir = (delta < 0) ? 1u : 0u;

    /* ── Hot retarget: motor already moving in the same direction. ─────── *
     * Just update destination + profile constants. Keep current interval   *
     * to avoid deceleration between consecutive small moves.               */
    if (g_lat_phase != MOVE_IDLE && new_dir == params->lat_dir) {
        telem->lat_move_done    = 0u;
        g_lat_target            = params->lat_target_pos;
        g_lat_start_iv          = params->lat_start_iv;
        g_lat_cruise_iv         = params->lat_cruise_iv;
        g_lat_delta_iv          = params->lat_delta_iv;
        /* If currently decelerating toward old target: snap back to cruise *
         * so remaining-distance decel trigger fires for the new target.    */
        if (g_lat_phase == MOVE_DECEL) {
            params->lat_interval = g_lat_cruise_iv;
            g_lat_phase          = MOVE_CRUISE;
        }
        params->lat_move_active = 0u;
        return;
    }

    /* ── Cold start or direction reversal: full re-arm from start speed. ── */
    telem->lat_move_done    = 0u;
    g_lat_phase             = MOVE_IDLE;
    g_lat_target            = params->lat_target_pos;
    g_lat_start_iv          = params->lat_start_iv;
    g_lat_cruise_iv         = params->lat_cruise_iv;
    g_lat_delta_iv          = params->lat_delta_iv;
    g_lat_accel_steps       = 0u;

    params->lat_dir         = new_dir;
    params->lat_interval    = g_lat_start_iv;
    params->lat_enable      = 1u;
    params->lat_run         = 1u;

    g_lat_phase             = MOVE_ACCEL;
    params->lat_move_active = 0u;  /* self-clear the arm flag               */
}

/* ── Update profile on each lateral rising edge ─────────────────────────── *
 * No division, no float. Pure addition/subtraction + comparisons.          *
 * Called only when g_lat_phase != MOVE_IDLE and a rising step edge fires.  */
static void lat_move_on_step(void) {
    uint32_t remaining = u32abs(g_lat_target - lateral.position);

    switch (g_lat_phase) {

    case MOVE_ACCEL:
        g_lat_accel_steps++;
        /* Decrease interval toward cruise speed. */
        if (g_lat_delta_iv > 0u &&
            params->lat_interval > g_lat_cruise_iv + g_lat_delta_iv) {
            params->lat_interval -= g_lat_delta_iv;
        } else {
            params->lat_interval = g_lat_cruise_iv;
        }
        /* Transition: remaining <= accel_steps → skip cruise, go to decel. */
        if (remaining <= g_lat_accel_steps) {
            g_lat_phase = MOVE_DECEL;
        } else if (params->lat_interval <= g_lat_cruise_iv) {
            params->lat_interval = g_lat_cruise_iv;
            g_lat_phase = MOVE_CRUISE;
        }
        break;

    case MOVE_CRUISE:
        /* Enter decel when distance left equals ramp distance. */
        if (remaining <= g_lat_accel_steps) {
            g_lat_phase = MOVE_DECEL;
        }
        break;

    case MOVE_DECEL:
        /* Increase interval back toward start speed. */
        if (g_lat_delta_iv > 0u) {
            params->lat_interval += g_lat_delta_iv;
            if (params->lat_interval >= g_lat_start_iv) {
                params->lat_interval = g_lat_start_iv;
            }
        }
        /* Stop when target reached. */
        if (remaining == 0u) {
            g_lat_phase      = MOVE_IDLE;
            params->lat_run  = 0u;
            telem->lat_move_done = 1u;
        }
        break;

    default:
        g_lat_phase = MOVE_IDLE;
        break;
    }
}

/* ── Apply direction GPIO ────────────────────────────────────────────────── */
static inline void apply_dir_sp(uint8_t dir) {
    if (dir) __R30 |= SP_DIR_BIT; else __R30 &= ~SP_DIR_BIT;
}

static inline void apply_dir_lat(uint8_t dir) {
    if (dir) __R30 |= LAT_DIR_BIT; else __R30 &= ~LAT_DIR_BIT;
}

/* ── Apply enable GPIO (active-low) ─────────────────────────────────────── */
static inline void apply_enable_sp(uint8_t en) {
    if (en) __R30 &= ~SP_EN_BIT; else __R30 |= SP_EN_BIT;
}

static inline void apply_enable_lat(uint8_t en) {
    if (en) __R30 &= ~LAT_EN_BIT; else __R30 |= LAT_EN_BIT;
}

/* ── Publish telemetry ───────────────────────────────────────────────────── */
static void publish_telem(void) {
    telem->sp_step_count     = spindle.step_count;
    telem->lat_step_count    = lateral.step_count;
    telem->lat_position      = lateral.position;
    telem->sp_interval_actual  = spindle.interval;
    telem->lat_interval_actual = lateral.interval;

    /* Spindle state */
    uint8_t sp_st = MOTOR_STATE_IDLE;
    if (spindle.running) sp_st = MOTOR_STATE_RUNNING;
    if (!(__R30 & SP_EN_BIT)) sp_st |= MOTOR_STATE_ENABLED;
    telem->sp_state = sp_st;

    /* Lateral state */
    uint8_t lat_st = MOTOR_STATE_IDLE;
    if (lateral.running) lat_st = MOTOR_STATE_RUNNING;
    if (!(__R30 & LAT_EN_BIT)) lat_st |= MOTOR_STATE_ENABLED;
    telem->lat_state = lat_st;

    /* endstop_mask is 0: endstops are owned by PRU0 (orchestrator) */
    telem->endstop_mask = 0u;
    telem->seq++;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Main loop
 * ════════════════════════════════════════════════════════════════════════════ */
int main(void) {
    /* Safe state: STEP/DIR low, drivers disabled (EN high = active-low). */
    __R30 &= ~(SP_STEP_BIT | SP_DIR_BIT | LAT_STEP_BIT | LAT_DIR_BIT);
    __R30 |=  (SP_EN_BIT | LAT_EN_BIT);

    /* PRU0 owns the IEP timer. */
    IEP_INIT();

    /* Zero shared memory areas owned by PRU0. */
    *telem = (motor_telem_t){0};

    uint32_t loop_cnt = 0u;

    while (1) {
        uint32_t now = IEP_NOW();

        /* ── 0. Arm move profile if requested ──────────────────────────── */
        if (params->lat_move_active) {
            lat_move_arm();
        }

        /* ── 1. Read motor parameters from PRU1 ─────────────────────────── */
        uint32_t sp_iv   = params->sp_interval;
        uint32_t lat_iv  = params->lat_interval;
        uint8_t  sp_dir  = params->sp_dir;
        uint8_t  lat_dir = params->lat_dir;
        uint8_t  sp_en   = params->sp_enable;
        uint8_t  lat_en  = params->lat_enable;
        uint8_t  sp_run  = params->sp_run;
        uint8_t  lat_run = params->lat_run;

        /* ── 2. Apply enable outputs ────────────────────────────────────── */
        apply_enable_sp(sp_en);
        apply_enable_lat(lat_en);

        /* ── 3. Apply direction outputs ─────────────────────────────────── */
        apply_dir_sp(sp_dir);
        apply_dir_lat(lat_dir);


        /* ── 4. Spindle pulse generation ────────────────────────────────── */
        uint8_t sp_pin = pulse_update(&spindle, sp_iv, sp_dir, sp_run, now);
        if (sp_pin) __R30 |= SP_STEP_BIT; else __R30 &= ~SP_STEP_BIT;

        /* ── 5. Lateral pulse generation ────────────────────────────────── */
        uint8_t lat_pin = pulse_update(&lateral, lat_iv, lat_dir, lat_run, now);
        if (lat_pin) __R30 |= LAT_STEP_BIT; else __R30 &= ~LAT_STEP_BIT;

        /* ── 5b. Trapezoidal profile step update ────────────────────────── */
        if (lat_pin && !g_prev_lat_pin && g_lat_phase != MOVE_IDLE) {
            lat_move_on_step();
        }
        g_prev_lat_pin = lat_pin;

        /* ── 6. Publish telemetry (throttled) ───────────────────────────── */
        if (++loop_cnt >= TELEM_STRIDE) {
            loop_cnt = 0u;
            publish_telem();
        }
    }

    return 0;
}
