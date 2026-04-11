/* pru_stepper.h — IEP timer access + continuous pulse generation for PRU.
 *
 * Provides:
 *   - IEP hardware counter macros (200 MHz, 5 ns/cycle).
 *   - timer_before() for 32-bit rollover-safe comparison.
 *   - pulse_gen_t: IEP-timed pulse generation with Klipper-style
 *     constant-acceleration support (interval += add each step).
 *
 * Acceleration model (inspired by Klipper stepper.c):
 *   The host pre-computes three values per ramp segment:
 *     interval — IEP half-period for the first step of the segment
 *     count    — number of steps in this segment
 *     add      — signed delta added to interval after each step
 *
 *   On each rising-edge step inside the PRU:
 *     interval += add
 *     count--
 *   When count reaches 0, the segment is complete.
 *
 *   This is a first-order Bresenham approximation of the ideal constant-
 *   acceleration curve  v(s) = sqrt(2·a·s), but broken into piecewise-linear
 *   segments.  A single segment over a wide speed range is NOT sufficient:
 *   v = clock/iv is hyperbolic, so a constant add accelerates ~150× faster
 *   at high speed than at low speed.  Use compute_accel_ramp() with
 *   PRU_N_ACCEL_SEG sub-segments; see architecture doc §10 for derivation.
 *
 *   Advantages over other approaches:
 *     - ZERO division in the PRU hot loop (just one add per step).
 *     - No sqrt, no log2 tables, no Q-format frequency tracking.
 *     - Host computes add = (end_iv - start_iv) / count  (one division,
 *       done in the daemon or Python at command time, not per step).
 *     - Symmetric accel/decel: same |add|, opposite sign.
 *
 * Design principle: PRU0 is a dumb pulse generator. It reads interval/dir/run
 * from motor_params_t (written by PRU1) and generates edges. Speed changes
 * take effect immediately. The ramp_seg_t overlay is optional — when add==0
 * and count==0 the engine behaves identically to the previous constant-speed
 * mode.
 *
 * Include only from PRU firmware. No dynamic memory, no float, no division.
 */

#ifndef PRU_STEPPER_H
#define PRU_STEPPER_H

#include <stdint.h>
#include "pru_ipc.h"  /* ramp_seg_t, MAX_RAMP_SEGS */

/* ── IEP register access ────────────────────────────────────────────────────
 * AM335x IEP base: PRU constant table entry 26 (CREGISTER=26) → local 0x0002E000.
 * Both PRU0 and PRU1 share the same hardware counter.
 *
 * Two implementations are provided:
 *
 *   __TI_COMPILER_VERSION__ (clpru): use CT_IEP.TMR_CNT from <pru_iep.h>.
 *     The TI constant-table access emits a true 32-bit LBBO automatically.
 *
 *   pru-elf-gcc: standard "lbbo %0, %1, 0, 4" inline asm.  Pass the IEP
 *     counter address as a register operand so gcc allocates freely.
 *     DO NOT hand-encode ".word 0xF103FCFC": that decodes as
 *     "lbbo r28.b3, r28, 3, 16" (single-byte, wrong offset) — the correct
 *     opcode for "lbbo r28, r28, 0, 4" is 0xF1003C9C, but letting gcc
 *     assign the register avoids this entirely.
 */

#ifdef __TI_COMPILER_VERSION__
/* ── clpru path: constant table CT17 / struct CT_IEP ─────────────────────── */
#  include <pru_iep.h>
#  define IEP_NOW()    ((uint32_t)CT_IEP.TMR_CNT)
/* Call once on boot from the IEP-owner PRU only. */
#  define IEP_INIT()   do { CT_IEP.TMR_GLB_CFG = 0x11u; CT_IEP.TMR_CNT = 0u; } while(0)

#else
/* ── pru-elf-gcc fallback: volatile pointer to IEP counter ─────────────── *
 * Read the IEP counter directly via volatile pointer.  The compiler will  *
 * generate a 32-bit LBBO (or similar) when it sees the volatile access.  */
#  define PRU_IEP_BASE       0x0002E000u
#  define PRU_IEP_GLB_CFG    (*(volatile uint32_t *)(PRU_IEP_BASE + 0x00u))
#  define PRU_IEP_TMR_CNT    (*(volatile uint32_t *)(PRU_IEP_BASE + 0x0Cu))

#  define IEP_NOW()          PRU_IEP_TMR_CNT
/* Call once on boot from the IEP-owner PRU only. */
#  define IEP_INIT()         do { PRU_IEP_GLB_CFG = 0x11u; PRU_IEP_TMR_CNT = 0u; } while(0)

#endif /* __TI_COMPILER_VERSION__ */

/* ── Timing constants ────────────────────────────────────────────────────── */
#define MIN_INTERVAL_CYC   625u   /* 160 kHz max → 3.125 µs minimum         */
#define MAX_INTERVAL_CYC   187500u /* ~534 Hz min                            */
#define DIR_SETUP_CYC       40u   /* 200 ns A4988 dir-setup = 40 IEP cycles */

/* ── 32-bit rollover-safe timer comparison ───────────────────────────────── */
static inline int timer_before(uint32_t t1, uint32_t t2) {
    return (int32_t)(t1 - t2) < 0;
}

/* ── Continuous pulse generator state ────────────────────────────────────── *
 * With Klipper-style acceleration:
 *   accel_add  : signed delta added to interval each rising edge (0 = coast)
 *   accel_count: number of accel steps remaining (0 = constant speed)       */
typedef struct {
    uint32_t next_edge_time;    /* absolute IEP value of next STEP edge      */
    uint32_t interval;          /* current IEP interval between edges        */
    uint32_t step_count;        /* physical steps (rising edges) since reset */
    int32_t  position;          /* signed step position (lateral only)       */
    int32_t  accel_add;         /* Klipper: interval delta per step          */
    uint32_t accel_count;       /* Klipper: remaining steps in this segment  */
    uint8_t  direction;         /* current direction (0=fwd, 1=rev)          */
    uint8_t  step_pin_state;    /* current STEP pin logic level (0 or 1)    */
    uint8_t  running;           /* 1 → actively generating pulses            */
    uint8_t  _pad;
} pulse_gen_t;

/* ── pulse_update ────────────────────────────────────────────────────────────
 * Called every PRU main-loop iteration for one axis.
 *
 * - If new_interval is 0 or run==0 → stop generating pulses.
 * - On rising edge: interval += accel_add, accel_count-- (Klipper ramp).
 * - When accel_count reaches 0, add is cleared → constant speed.
 *
 * Returns the new STEP pin state (0 or 1). Caller writes to __R30.
 */
static inline uint8_t pulse_update(pulse_gen_t *pg,
                                   uint32_t new_interval,
                                   uint8_t  new_dir,
                                   uint8_t  run,
                                   uint32_t now)
{
    /* ── Stop condition ──────────────────────────────────────────────────── */
    if (!run || new_interval == 0u) {
        if (pg->running) {
            pg->running        = 0u;
            pg->step_pin_state = 0u;
        }
        return 0u;
    }

    /* ── Clamp interval ──────────────────────────────────────────────────── */
    if (new_interval < MIN_INTERVAL_CYC)
        new_interval = MIN_INTERVAL_CYC;

    /* ── Start from idle ─────────────────────────────────────────────────── */
    if (!pg->running) {
        pg->interval       = new_interval;
        pg->direction      = new_dir;
        pg->next_edge_time = now + new_interval;
        pg->running        = 1u;
        pg->step_pin_state = 0u;
        return 0u;  /* first edge will fire on next pass */
    }

    /* ── Adopt new interval if no ramp active (instant speed change) ─────  */
    if (pg->accel_count == 0u)
        pg->interval = new_interval;
    pg->direction = new_dir;

    /* ── Check if time to toggle ─────────────────────────────────────────── */
    if (!timer_before(now, pg->next_edge_time)) {
        pg->step_pin_state ^= 1u;

        if (pg->step_pin_state) {
            /* Rising edge = physical step */
            pg->step_count++;
            if (!pg->direction) pg->position++;
            else                pg->position--;

            /* ── Klipper ramp: interval += add ──────────────────────────── */
            if (pg->accel_count > 0u) {
                int32_t iv = (int32_t)pg->interval + pg->accel_add;
                /* Clamp to valid range */
                if (iv < (int32_t)MIN_INTERVAL_CYC) iv = (int32_t)MIN_INTERVAL_CYC;
                if (iv > (int32_t)MAX_INTERVAL_CYC) iv = (int32_t)MAX_INTERVAL_CYC;
                pg->interval = (uint32_t)iv;
                pg->accel_count--;
            }
        }

        pg->next_edge_time += pg->interval;
    }

    return pg->step_pin_state;
}

/* ── pulse_set_ramp — arm a Klipper-style ramp segment ───────────────────── *
 * Called once to start an accel or decel segment.
 *   start_iv : interval for the first step of the segment
 *   add      : signed delta per step (negative = accelerating, positive = decelerating)
 *   count    : number of steps in this segment (0 disables ramp)
 *
 * The caller must also set pg->interval = start_iv before the first step
 * if the motor is currently idle. If already running, the ramp takes effect
 * on the next rising edge.                                                  */
static inline void pulse_set_ramp(pulse_gen_t *pg,
                                  uint32_t start_iv,
                                  int32_t  add,
                                  uint32_t count)
{
    pg->interval    = start_iv;
    pg->accel_add   = add;
    pg->accel_count = count;
}

/* ── pulse_stop — immediate halt ─────────────────────────────────────────── */
static inline void pulse_stop(pulse_gen_t *pg) {
    pg->running        = 0u;
    pg->step_pin_state = 0u;
    pg->interval       = 0u;
    pg->accel_add      = 0;
    pg->accel_count    = 0u;
}

/* ── pulse_reset_counters ────────────────────────────────────────────────── */
static inline void pulse_reset_counters(pulse_gen_t *pg) {
    pg->step_count = 0u;
    pg->position   = 0;
}

/* ── Klipper multi-segment ramp ─────────────────────────────────────────── *
 *
 * WHY MULTI-SEGMENT IS REQUIRED:
 * A single {interval, add, count} over a wide speed range produces non-
 * constant acceleration.  v = 100 MHz / iv is hyperbolic:
 *   At iv=93000 (10 RPM):  add=-14 → Δv ≈ 0.015% per step
 *   At iv=625  (1500 RPM): add=-14 → Δv ≈ 2.24%  per step  (~150× faster!)
 * → single segment: motor crawls at low speed, stalls at high speed.
 *
 * SOLUTION (identical to Klipper queue_step + stepper_load_next):
 * PRU_N_ACCEL_SEG sub-segments, each covering an EQUAL INTERVAL sub-range
 * (Δiv uniform, not Δv uniform).  This ensures every segment has a non-zero
 * |gap|/count ratio regardless of speed, so add is always non-zero.
 *
 * Why Δiv-uniform beats Δv-uniform:
 *   With Δv-uniform, high-speed segments have huge count (many steps at fast
 *   speed) but tiny |gap| (iv changes little at high speed) → add truncates
 *   to 0 → flat speed plateau for thousands of steps → visible staircase.
 *   With Δiv-uniform, every segment has the same |gap|=Δiv, and count is
 *   proportional to the kinematic steps in that iv band → add is always
 *   a reasonable fraction of Δiv → smooth across the full speed range.
 *
 * Interval force-loaded at each boundary (= stepper_load_next) prevents
 * error accumulation.
 *
 * Shared between test_spindle and production orchestrator firmware.         */

#define PRU_N_ACCEL_SEG  MAX_RAMP_SEGS  /* alias for backward compatibility  */

/* accel_seg_t is now ramp_seg_t from pru_ipc.h (identical layout).           */
typedef ramp_seg_t accel_seg_t;

/* compute_accel_ramp — fill segs[PRU_N_ACCEL_SEG] for a constant-accel ramp
 * from start_iv to end_iv.  Precondition: start_iv > end_iv (accelerating).
 * All divisions done here — ZERO division in the step loop.
 *
 * Used by the daemon (ARM) and potentially by test firmware.
 * The PRU1 motor firmware does not use this function (it uses the
 * sp_ramp_seg_t[]/sp_ramp_ctrl_t mechanism via IPC).
 * The static qualifier is kept for header-only inclusion; the __attribute__
 * suppresses the -Wunused-function warning in translation units that include
 * this header without calling compute_accel_ramp.                           */
static void __attribute__((unused))
compute_accel_ramp(ramp_seg_t *segs,
                   uint32_t start_iv, uint32_t end_iv,
                   uint32_t accel_s2)
{
    uint32_t div_iv, i;

    /* Integer division: last segment absorbs the remainder by using end_iv  */
    div_iv = (start_iv - end_iv) / MAX_RAMP_SEGS;
    if (div_iv < 1u) div_iv = 1u;

    for (i = 0u; i < MAX_RAMP_SEGS; i++) {
        uint32_t ivs, ive, vs, ve, cnt;
        int32_t  gap;

        ivs = start_iv - div_iv * i;
        ive = (i == MAX_RAMP_SEGS - 1u) ? end_iv : (ivs - div_iv);

        /* Clamp */
        if (ive < 1u) ive = 1u;
        if (ivs < ive) ivs = ive;

        /* Velocities for the kinematic step count */
        vs = 100000000u / ivs;
        ve = 100000000u / ive;

        /* s = (ve²-vs²)/(2a) = (vs+ve)·(ve-vs)/(2a) — uint64 for overflow */
        cnt = (uint32_t)(((uint64_t)(vs + ve) * (ve - vs))
                         / (2u * (uint64_t)accel_s2));
        if (cnt < 1u) cnt = 1u;

        gap = (int32_t)ive - (int32_t)ivs;   /* always negative (accel)     */

        segs[i].start_iv = ivs;
        segs[i].count    = cnt;
        segs[i].add      = gap / (int32_t)cnt;
        /* add may still truncate to 0 if div_iv < cnt (very slow segment
         * with tiny Δiv).  Force ±1 is then safe here because div_iv is
         * small — premature clamping cannot cause a backward jump since
         * the NEXT segment's start_iv is only div_iv away.                  */
        if (segs[i].add == 0 && gap != 0)
            segs[i].add = (gap > 0) ? 1 : -1;
    }
}

#endif /* PRU_STEPPER_H */
