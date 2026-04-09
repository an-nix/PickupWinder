/* test_spindle/main.c — Standalone PRU0 spindle test.
 *
 * Purpose: validate PRU pin control + stepper driver hardware independently
 *          from the daemon, IPC, and PRU1.
 *
 * This firmware replaces am335x-pru0-fw temporarily.
 * Load it via:
 *   sudo cp am335x-pru0-fw-test /lib/firmware/am335x-pru0-fw
 *   sudo sh -c 'echo stop  > /sys/class/remoteproc/remoteproc1/state'
 *   sleep 1
 *   sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state'
 *
 * What it does:
 *   1. Enables spindle driver (EN_A LOW — active-low).
 *   2. Sets DIR_A (forward).
 *   3. Generates STEP pulses at 25 RPM using the IEP hardware counter.
 *   4. Blinks the STEP pin forever — no IPC, no daemon, no PRU1.
 *
 * Speed:
 *   25 RPM × 6400 steps/rev (200-step × 32 µstep) = 2667 Hz
 *   IEP half-period = 200 000 000 / (2 × 2667) = 37 481 cycles ≈ 37 500
 *
 * Pin mapping (PRU0 R30, active-low EN):
 *   STEP_A = R30[1]  P9_29
 *   DIR_A  = R30[5]  P9_27
 *   EN_A   = R30[7]  P9_25  (0 = enabled, 1 = disabled)
 *
 * To restore normal operation after test:
 *   sudo cp am335x-pru0-fw-orig /lib/firmware/am335x-pru0-fw
 *   sudo sh -c 'echo stop  > /sys/class/remoteproc/remoteproc1/state'
 *   sleep 1
 *   sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state'
 */

#include <stdint.h>
#include "../include/pru_stepper.h"   /* IEP_INIT, IEP_NOW, pulse_gen_t, pulse_update */
#include "../include/pru_regs.h"      /* __R30 */

/* ── Pin bitmasks ─────────────────────────────────────────────────────────── */
#define SP_STEP_BIT  (1u << 1)   /* P9_29 */
#define SP_DIR_BIT   (1u << 5)   /* P9_27 */
#define SP_EN_BIT    (1u << 7)   /* P9_25 — active-low */

/* ── Speed: 25 RPM ─────────────────────────────────────────────────────────
 * 25 rpm × 6400 steps/rev = 2666.67 Hz
 * IEP half-period = 200 000 000 / (2 × 2666.67) = 37 500 cycles            */
#define TEST_INTERVAL_CYC  37500u

/* Direction: 0 = forward, 1 = reverse.  Change if motor runs backwards.    */
#define TEST_DIR  0u

int main(void) {
    /* ── 1. Safe initial state: STEP low, DIR low, EN high (disabled) ────── */
    __R30 &= ~SP_STEP_BIT;
    __R30 &= ~SP_DIR_BIT;
    __R30 |=  SP_EN_BIT;   /* disabled (active-low) */

    /* ── 2. Init IEP timer ───────────────────────────────────────────────── */
    IEP_INIT();

    /* ── 3. Set direction ────────────────────────────────────────────────── */
#if TEST_DIR
    __R30 |=  SP_DIR_BIT;
#else
    __R30 &= ~SP_DIR_BIT;
#endif

    /* ── 4. Enable driver (EN_A LOW = active-low) ────────────────────────── */
    __R30 &= ~SP_EN_BIT;

    /* ── 5. Pulse generator state ────────────────────────────────────────── */
    pulse_gen_t spindle = {0};

    /* ── 6. Main loop: generate pulses at TEST_INTERVAL_CYC ──────────────── */
    while (1) {
        uint32_t now = IEP_NOW();

        uint8_t pin = pulse_update(&spindle,
                                   TEST_INTERVAL_CYC,
                                   TEST_DIR,
                                   1u,    /* run = always */
                                   now);

        if (pin)
            __R30 |=  SP_STEP_BIT;
        else
            __R30 &= ~SP_STEP_BIT;
    }

    return 0;
}
