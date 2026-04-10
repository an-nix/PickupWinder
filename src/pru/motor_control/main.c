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

/* ── Shared RAM pointers ─────────────────────────────────────────────────── */
static volatile motor_params_t *params =
    (volatile motor_params_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_PARAMS_OFFSET);

static volatile motor_telem_t *telem =
    (volatile motor_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_TELEM_OFFSET);

/* ── Pulse generators ────────────────────────────────────────────────────── */
static pulse_gen_t spindle = {0};
static pulse_gen_t lateral = {0};

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

        /* ── 6. Publish telemetry (throttled) ───────────────────────────── */
        if (++loop_cnt >= TELEM_STRIDE) {
            loop_cnt = 0u;
            publish_telem();
        }
    }

    return 0;
}
