/* test_spindle/main.c — Multi-speed spindle sweep with Klipper-style ramp.
 *
 * Direct IEP polling — no pulse_gen_t, no abstraction layer.
 * Uses the same "interval += add" Klipper acceleration model but drives
 * STEP/DIR/EN directly with two busy-wait loops per step:
 *
 *   for each step:
 *     wait iv cycles → STEP HIGH
 *     wait iv cycles → STEP LOW
 *     iv += add       (Klipper: one integer add per step, host-computed add)
 *
 * This approach is equivalent to test_gpio's IEP usage and is provably correct.
 *
 * Sequence (7 levels: 25 → 1500 RPM):
 *   For each level:
 *     FWD: accel ramp (~1s) | cruise 10s | decel ramp (~1s) | stop 0.5s
 *     REV: same
 *
 * Pin mapping (groupe B — spindle câblé sur P8_42/44/46):
 *   STEP = R30[1]  P8_46
 *   DIR  = R30[3]  P8_44
 *   EN   = R30[5]  P8_42  (active-low)
 *
 * Deploy:
 *   sudo cp am335x-pru0-fw-test-spindle /lib/firmware/am335x-pru1-fw
 *   echo stop  | sudo tee /sys/class/remoteproc/remoteproc2/state
 *   sleep 1
 *   echo start | sudo tee /sys/class/remoteproc/remoteproc2/state
 */

#include <stdint.h>
#include "../include/pru_stepper.h"   /* IEP_INIT, IEP_NOW, MIN/MAX_INTERVAL_CYC */
#include "../include/pru_regs.h"      /* __R30 */
#include "../include/pru_rsc_table.h"  /* required by remoteproc */

/* ── Pin bitmasks (groupe B: spindle on P8_42/44/46) ─────────────────────── */
#define SP_STEP_BIT  (1u << 1)   /* P8_46  PRU1 R30[1] */
#define SP_DIR_BIT   (1u << 3)   /* P8_44  PRU1 R30[3] */
#define SP_EN_BIT    (1u << 5)   /* P8_42  PRU1 R30[5]  active-low */

/* ── Speed conversions ───────────────────────────────────────────────────── *
 * Half-period in IEP cycles:  iv = 100_000_000 / step_hz
 * Step rate = 100 MHz / iv   (period = 2 * iv at 200 MHz)                   */
#define RPM_TO_HZ(rpm)  ((rpm) * 6400u / 60u)
#define HZ_TO_IV(hz)    (100000000u / (hz))
#define RPM_TO_IV(rpm)  HZ_TO_IV(RPM_TO_HZ(rpm))

/* ── Ramp start (10 RPM — low enough to never stall) ────────────────────── */
#define START_RPM   10u
#define START_HZ    RPM_TO_HZ(START_RPM)   /* 1066 Hz */
#define START_IV    HZ_TO_IV(START_HZ)     /* ~93809 cycles */

/* ── Phase durations ─────────────────────────────────────────────────────── */
#define CRUISE_CYC  (2u * 200000000u)   /* 2 s cruise per direction */
#define PAUSE_CYC   (100000000u)         /* 0.5 s pause between directions */

/* ── Max acceleration ────────────────────────────────────────────────────── *
 * Expressed in steps/s² — ramp time = Δv / ACCEL.                          *
 *   25 RPM  (Δv =  1 600 steps/s) → ~  80 ms                               *
 *   800 RPM (Δv = 84 266 steps/s) → ~  4.2 s                               *
 *   1500 RPM(Δv = 158 933 steps/s) → ~  8.0 s                              *
 * Increase to accelerate faster; decrease if motor stalls.                  */
#define ACCEL_STEPS_S2  20000u   /* steps/s² — ~188 RPM/s for 6400 steps/rev  */

/* accel_seg_t and compute_accel_ramp() are in pru_stepper.h.
 * See doc/beaglebone_architecture.md §10 for the derivation.
 * PRU_N_ACCEL_SEG × 12 bytes = 192 bytes in PRU DMEM (8 KB available).     */
static accel_seg_t ramp_segs[PRU_N_ACCEL_SEG];

/* ── Speed levels ────────────────────────────────────────────────────────── */
static const uint32_t speed_rpm[] = { 25, 50, 100, 200, 400, 800, 1500 };
#define N_LEVELS  (sizeof(speed_rpm) / sizeof(speed_rpm[0]))

/* ── wait_iv ─────────────────────────────────────────────────────────────── *
 * Busy-wait exactly `iv` IEP cycles using rollover-safe subtraction.        */
static void wait_iv(uint32_t iv) {
    uint32_t t0 = IEP_NOW();
    while ((uint32_t)(IEP_NOW() - t0) < iv) { }
}

/* ── one_step ────────────────────────────────────────────────────────────── *
 * Generate one STEP pulse: HIGH for iv cycles, then LOW for iv cycles.
 * Total pulse period = 2 * iv IEP cycles.                                   */
static void one_step(uint32_t iv) {
    wait_iv(iv);
    __R30 |=  SP_STEP_BIT;
    wait_iv(iv);
    __R30 &= ~SP_STEP_BIT;
}

/* ── run_ramp_forward ────────────────────────────────────────────────────── *
 * Execute ramp_segs[] in forward order (accel: interval decreasing).
 * Matches Klipper's stepper_event loop + stepper_load_next:
 *   - Load segment's start_iv at boundary (force-correct accumulated error)
 *   - Inner loop: interval += add, count--                                   */
static void run_ramp_forward(void)
{
    uint32_t seg, s, iv;
    int32_t  iv32;

    for (seg = 0u; seg < PRU_N_ACCEL_SEG; seg++) {
        iv = ramp_segs[seg].start_iv;          /* force-load (stepper_load_next) */
        for (s = 0u; s < ramp_segs[seg].count; s++) {
            one_step(iv);
            iv32 = (int32_t)iv + ramp_segs[seg].add;
            if (iv32 < (int32_t)MIN_INTERVAL_CYC) iv32 = (int32_t)MIN_INTERVAL_CYC;
            if (iv32 > (int32_t)MAX_INTERVAL_CYC) iv32 = (int32_t)MAX_INTERVAL_CYC;
            iv = (uint32_t)iv32;
        }
    }
}

/* ── run_ramp_reverse ────────────────────────────────────────────────────── *
 * Execute ramp_segs[] in reverse order with negated add (decel).
 * Segment N-1 starts at cruise_iv, segment 0 ends near START_IV.
 *
 * For decel segment at index idx:
 *   start_iv = next accel segment's start_iv  (or cruise_iv for last)
 *   add      = -accel_add  (interval increases instead of decreasing)        */
static void run_ramp_reverse(uint32_t cruise_iv)
{
    uint32_t ri, idx, s, iv;
    int32_t  iv32;

    for (ri = 0u; ri < PRU_N_ACCEL_SEG; ri++) {
        idx = PRU_N_ACCEL_SEG - 1u - ri;
        /* Decel starts where accel ended: next segment's start_iv, or cruise */
        iv = (idx < PRU_N_ACCEL_SEG - 1u) ? ramp_segs[idx + 1u].start_iv : cruise_iv;
        for (s = 0u; s < ramp_segs[idx].count; s++) {
            one_step(iv);
            iv32 = (int32_t)iv - ramp_segs[idx].add;      /* negate accel add */
            if (iv32 < (int32_t)MIN_INTERVAL_CYC) iv32 = (int32_t)MIN_INTERVAL_CYC;
            if (iv32 > (int32_t)MAX_INTERVAL_CYC) iv32 = (int32_t)MAX_INTERVAL_CYC;
            iv = (uint32_t)iv32;
        }
    }
}

/* ── run_direction ───────────────────────────────────────────────────────── *
 * Full trapezoidal profile in one direction:
 *   1. Compute PRU_N_ACCEL_SEG acceleration segments (= Klipper queue_step)
 *   2. Execute segments forward (accel: START_IV → cruise_iv)
 *   3. Cruise at constant cruise_iv for CRUISE_CYC
 *   4. Execute segments backward with negated add (decel: cruise_iv → START_IV)
 *   5. Pause                                                                 */
static void run_direction(uint8_t dir, uint32_t cruise_iv)
{
    uint32_t t0;

    /* Set direction */
    if (dir == 0u) __R30 &= ~SP_DIR_BIT;
    else           __R30 |=  SP_DIR_BIT;

    /* Compute accel segments (divisions here, NOT in the step loop) */
    compute_accel_ramp(ramp_segs, START_IV, cruise_iv, ACCEL_STEPS_S2);

    /* ── Accel: START_IV → cruise_iv (N_SEG segments forward) ─────────── */
    run_ramp_forward();

    /* ── Cruise: cruise_iv for CRUISE_CYC ─────────────────────────────── */
    t0 = IEP_NOW();
    while ((uint32_t)(IEP_NOW() - t0) < CRUISE_CYC) {
        one_step(cruise_iv);
    }

    /* ── Decel: cruise_iv → START_IV (N_SEG segments reversed) ────────── */
    run_ramp_reverse(cruise_iv);

    /* ── Stop + pause ──────────────────────────────────────────────────── */
    __R30 &= ~SP_STEP_BIT;
    wait_iv(PAUSE_CYC);
}

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Safe initial state: all pins low, EN high (disabled) */
    __R30 &= ~SP_STEP_BIT;
    __R30 &= ~SP_DIR_BIT;
    __R30 |=  SP_EN_BIT;

    IEP_INIT();

    /* Enable driver (active-low) */
    __R30 &= ~SP_EN_BIT;

    /* Sweep all speed levels, FWD then REV at each */
    {
        uint32_t lvl;
        for (lvl = 0u; lvl < N_LEVELS; lvl++) {
            uint32_t cruise_iv = RPM_TO_IV(speed_rpm[lvl]);
            run_direction(0u, cruise_iv);
            run_direction(1u, cruise_iv);
        }
    }

    /* Sweep complete — motor holds (EN still low) */
    __R30 &= ~SP_STEP_BIT;
    while (1) { }

    return 0;
}
