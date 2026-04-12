/* motor_control/main.c — PRU1 motor firmware (constant-time pulse generator).
 *
 * Architecture layer 1/4.  Loaded as am335x-pru1-fw → remoteproc2 → PRU1
 * (4a338000.pru).
 *
 * PRU1 is a PURE CONSTANT-TIME pulse generator.  Every main-loop iteration
 * takes exactly LOOP_CYCLES_TARGET IEP cycles (busy-wait padded).
 *
 * Responsibilities:
 *   1. Read motor_ctl_t from motor_params_t (written by PRU0).
 *   2. Detect ramp_arm=1 → arm pulse_set_ramp() for ONE segment, clear
 *      ramp_arm=0.  Segment advance is entirely PRU0's job.
 *   3. Generate STEP/DIR/EN pulses via IEP hardware counter.
 *   4. Write all telemetry fields (incl. accel_count) inline every iteration.
 *      No TELEM_STRIDE, no gating — telemetry is always current.
 *   5. Busy-wait to LOOP_CYCLES_TARGET so loop duration is deterministic.
 *
 * Removed from PRU1 (moved to PRU0):
 *   - g_ramp_segs[], g_seg_idx[], g_seg_total[], g_ramping[] — moved to PRU0
 *   - Segment auto-advance logic (blocks 4b and 5) — moved to PRU0
 *   - publish_telem(), TELEM_STRIDE, loop_cnt — replaced by inline writes
 *   - seg_done write logic — PRU0 now detects completion via accel_count==0
 *
 * PRU1 knows NOTHING about:
 *   - Segment arrays (ramp_seg_t[]) — managed by PRU0
 *   - Host commands, homing, coordination — all PRU0's job
 *   - Which operation is in progress (set_speed ramp vs move_to)
 *
 * PRU1 owns and initialises the IEP timer.  PRU0 reads IEP but never resets.
 *
 * Motor indexing (from pru_ipc.h):
 *   MOTOR_0 = spindle (Motor B, even P8 pins)
 *   MOTOR_1 = lateral (Motor A, odd  P8 pins)
 *
 * Pin mapping on PRU1 R30 — MODE5 (pr1_pru1_pru_r30_N):
 *   Motor A — lateral (odd P8 pins):
 *     STEP_A = R30[0]  P8_45     DIR_A  = R30[2]  P8_43
 *     EN_A   = R30[4]  P8_41  (active-low)
 *   Motor B — spindle (even P8 pins):
 *     STEP_B = R30[1]  P8_46     DIR_B  = R30[3]  P8_44
 *     EN_B   = R30[5]  P8_42  (active-low)
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_stepper.h"
#include "../include/pru_regs.h"
#include "../include/pru_rsc_table.h"  /* required by remoteproc */

/* ── Pin bitmasks (PRU1 R30) ─────────────────────────────────────────────── */
#define LAT_STEP_BIT   (1u << 0)   /* P8_45 R30[0]  MOTOR_1 lateral step    */
#define SP_STEP_BIT    (1u << 1)   /* P8_46 R30[1]  MOTOR_0 spindle step    */
#define LAT_DIR_BIT    (1u << 2)   /* P8_43 R30[2]  MOTOR_1 lateral dir     */
#define SP_DIR_BIT     (1u << 3)   /* P8_44 R30[3]  MOTOR_0 spindle dir     */
#define LAT_EN_BIT     (1u << 4)   /* P8_41 R30[4]  MOTOR_1 lateral en (↓)  */
#define SP_EN_BIT      (1u << 5)   /* P8_42 R30[5]  MOTOR_0 spindle en (↓)  */

/* ── Loop timing — constant iteration target ─────────────────────────────── *
 * Busy-wait at end of each loop ensures every iteration takes exactly        *
 * LOOP_CYCLES_TARGET IEP cycles → zero rhythmic timing perturbation.        *
 *   LOOP_CYCLES_TARGET = 650                                                 *
 *   Max STEP frequency  = 200 MHz / (2 × 650) ≈ 153 kHz                    *
 *   ≈ 1440 RPM at 32× microstepping — covers the full operating range.     */
#define LOOP_CYCLES_TARGET  650u

/* ── Shared RAM pointers ─────────────────────────────────────────────────── */
static volatile motor_params_t *params =
    (volatile motor_params_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_PARAMS_OFFSET);

static volatile motor_telem_t *telem =
    (volatile motor_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_TELEM_OFFSET);

/* ── Pulse generators ────────────────────────────────────────────────────── */
static pulse_gen_t spindle = {0};
static pulse_gen_t lateral = {0};

/* ── Shadow registers for EN (prevents spurious GPIO writes on EN) ───────── *
 * EN is active-low and must only change when the enable state changes.
 * Shadow values initialised to 0xFF (force-write on first loop).           */
static uint8_t g_sp_en_shadow  = 0xFFu;
static uint8_t g_lat_en_shadow = 0xFFu;

/* ── Apply enable GPIO (active-low) ─────────────────────────────────────── */
static inline void apply_enable_sp(uint8_t en) {
    if (en) __R30 &= ~SP_EN_BIT; else __R30 |= SP_EN_BIT;
}
static inline void apply_enable_lat(uint8_t en) {
    if (en) __R30 &= ~LAT_EN_BIT; else __R30 |= LAT_EN_BIT;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Main loop
 * ════════════════════════════════════════════════════════════════════════════ */
int main(void) {
    /* Safe state: STEP/DIR low, both drivers disabled (EN active-low → HIGH) */
    __R30 = 0u;
    __R30 |= (SP_EN_BIT | LAT_EN_BIT);

    /* PRU1 owns the IEP timer. */
    IEP_INIT();

    /* Zero shared memory area owned by PRU1. */
    *telem = (motor_telem_t){0};

    while (1) {
        /* ── a. Capture loop start time for busy-wait pad ───────────────── */
        uint32_t t_start = IEP_NOW();

        /* ── b. Read motor parameters from PRU0 ────────────────────────── */
        uint32_t sp_iv   = params->motor[MOTOR_0].interval;
        uint32_t lat_iv  = params->motor[MOTOR_1].interval;
        uint8_t  sp_dir  = params->motor[MOTOR_0].dir;
        uint8_t  lat_dir = params->motor[MOTOR_1].dir;
        uint8_t  sp_en   = params->motor[MOTOR_0].enable;
        uint8_t  lat_en  = params->motor[MOTOR_1].enable;
        uint8_t  sp_run  = params->motor[MOTOR_0].run;
        uint8_t  lat_run = params->motor[MOTOR_1].run;

        /* ── c. Ramp arm detection (ONE segment, PRU0 owns advance) ─────── *
         * PRU0 writes {interval, ramp_add, ramp_count, ramp_arm=1} for each *
         * segment in sequence.  PRU1 arms the pulse generator and clears    *
         * ramp_arm=0.  PRU0 detects segment completion via accel_count==0   *
         * in telem (no seg_done flag needed).                               *
         * Segment arrays (ramp_seg_t[]) are owned by PRU0 — moved to PRU0. */
        if (params->motor[MOTOR_0].ramp_arm) {
            pulse_set_ramp(&spindle,
                           params->motor[MOTOR_0].interval,
                           params->motor[MOTOR_0].ramp_add,
                           params->motor[MOTOR_0].ramp_count);
            params->motor[MOTOR_0].ramp_arm = 0u;  /* acknowledge to PRU0 */
        }
        if (params->motor[MOTOR_1].ramp_arm) {
            pulse_set_ramp(&lateral,
                           params->motor[MOTOR_1].interval,
                           params->motor[MOTOR_1].ramp_add,
                           params->motor[MOTOR_1].ramp_count);
            params->motor[MOTOR_1].ramp_arm = 0u;  /* acknowledge to PRU0 */
        }

        /* ── EN: only on change (shadow register prevents GPIO glitches) ── */
        if (sp_en != g_sp_en_shadow) {
            apply_enable_sp(sp_en);
            g_sp_en_shadow = sp_en;
        }
        if (lat_en != g_lat_en_shadow) {
            apply_enable_lat(lat_en);
            g_lat_en_shadow = lat_en;
        }

        /* ── d. Pulse generation ────────────────────────────────────────── */
        uint32_t now = IEP_NOW();
        uint8_t sp_pin  = pulse_update(&spindle, sp_iv,  sp_dir,  sp_run,  now);
        uint8_t lat_pin = pulse_update(&lateral, lat_iv, lat_dir, lat_run, now);

        /* ── e. Single atomic __R30 write for STEP + DIR ────────────────── *
         * DIR and STEP written in one instruction — no 1-cycle DIR/STEP     *
         * glitch possible.  EN bits preserved by the masked clear.         */
        uint32_t r30 = __R30;
        r30 &= ~(SP_STEP_BIT | LAT_STEP_BIT | SP_DIR_BIT | LAT_DIR_BIT);
        r30 |= ((uint32_t)sp_pin  << 1u);   /* SP_STEP_BIT  = R30[1] */
        r30 |= ((uint32_t)lat_pin << 0u);   /* LAT_STEP_BIT = R30[0] */
        r30 |= ((uint32_t)sp_dir  << 3u);   /* SP_DIR_BIT   = R30[3] */
        r30 |= ((uint32_t)lat_dir << 2u);   /* LAT_DIR_BIT  = R30[2] */
        __R30 = r30;

        /* ── f. Inline telemetry write — every iteration, no throttle ───── *
         * All telem fields written unconditionally.  accel_count is the key *
         * field: PRU0 polls it to detect segment completion without any     *
         * seg_done handshake.  publish_telem() and TELEM_STRIDE removed.   */

        /* Spindle (MOTOR_0) */
        telem->motor[MOTOR_0].step_count      = spindle.step_count;
        telem->motor[MOTOR_0].position        = spindle.position;
        telem->motor[MOTOR_0].interval_actual = spindle.interval;
        telem->motor[MOTOR_0].accel_count     = spindle.accel_count; /* PRU0 watches */
        {
            uint8_t sp_st = MOTOR_STATE_IDLE;
            if (spindle.running)        sp_st  = MOTOR_STATE_RUNNING;
            if (!(__R30 & SP_EN_BIT))   sp_st |= MOTOR_STATE_ENABLED;
            telem->motor[MOTOR_0].state = sp_st;
        }

        /* Lateral (MOTOR_1) */
        telem->motor[MOTOR_1].step_count      = lateral.step_count;
        telem->motor[MOTOR_1].position        = lateral.position;
        telem->motor[MOTOR_1].interval_actual = lateral.interval;
        telem->motor[MOTOR_1].accel_count     = lateral.accel_count; /* PRU0 watches */
        {
            uint8_t lat_st = MOTOR_STATE_IDLE;
            if (lateral.running)        lat_st  = MOTOR_STATE_RUNNING;
            if (!(__R30 & LAT_EN_BIT))  lat_st |= MOTOR_STATE_ENABLED;
            telem->motor[MOTOR_1].state = lat_st;
        }

        /* Endstop mask: read by PRU0; we write 0 (endstops owned by PRU0). */
        telem->endstop_mask = 0u;

        /* ── g. Sequence counter (every iteration) ───────────────────────── */
        telem->seq++;

        /* ── h. Busy-wait pad — constant loop duration ───────────────────── *
         * Spin until LOOP_CYCLES_TARGET cycles have elapsed since t_start.  *
         * Guarantees every iteration is exactly LOOP_CYCLES_TARGET cycles   *
         * long → no rhythmic timing perturbation on STEP output.           */
        while ((uint32_t)(IEP_NOW() - t_start) < LOOP_CYCLES_TARGET) {}
    }

    return 0;
}

