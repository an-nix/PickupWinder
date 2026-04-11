/* test_gpio/main.c — PRU0 GPIO blink test.
 *
 * Toggles ALL stepper output pins simultaneously every 1 second.
 * Connect a 330 Ω + red LED (anode→pin, cathode→GND) on each pin to
 * verify the BBB is actually driving the pad.
 *
 * Pins tested (PRU R30):
 *
 *   R30[7]  P8_41  EN_A    (spindle enable — active-low in production)
 *   R30[5]  P8_43  DIR_A   (spindle direction)
 *   R30[1]  P8_45  STEP_A  (spindle step)
 *
 *   R30[3]  P8_42  EN_B    (lateral enable — active-low in production)
 *   R30[0]  P8_44  DIR_B   (lateral direction)
 *   R30[2]  P8_46  STEP_B  (lateral step)
 *
 * Expected: all 6 LEDs blink in sync at 0.5 Hz (1 s ON, 1 s OFF).
 * If a LED stays dark → that pad is not driven (pinmux not loaded,
 * overlay missing, or wiring fault on that specific pin).
 *
 * LED wiring:
 *   BBB pin ──[ 330 Ω ]──|▶|── GND (P9_1 or P9_2)
 *   Use red LEDs (Vf ≈ 2.0 V).  Current ≈ 3.9 mA — safe for BBB.
 *
 * To load (PRU1 / P8 pins):
 *   # copy the test image as the PRU1 firmware (remoteproc2)
 *   sudo cp am335x-pru0-fw-test-gpio /lib/firmware/am335x-pru1-fw
 *   sudo sh -c 'echo stop  > /sys/class/remoteproc/remoteproc2/state'
 *   sleep 1
 *   sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state'
 *
 * Note: The R30 bit positions are the same as the previous mapping; this
 * test targets the PRU that drives the P8_41..P8_46 group (PRU1 in our DTS).
 *
 * Build: make test-gpio
 */

#include <stdint.h>
#include "../include/pru_stepper.h"   /* IEP_INIT, IEP_NOW              */
#include "../include/pru_regs.h"      /* __R30                          */

/* ── All output pins — PRU1 R30 bits (confirmed from pinctrl debugfs dump) ── */
#define PIN_STEP_A   (1u << 0)   /* P8_45 PRU1 R30[0] */
#define PIN_STEP_B   (1u << 1)   /* P8_46 PRU1 R30[1] */
#define PIN_DIR_A    (1u << 2)   /* P8_43 PRU1 R30[2] */
#define PIN_DIR_B    (1u << 3)   /* P8_44 PRU1 R30[3] */
#define PIN_EN_A     (1u << 4)   /* P8_41 PRU1 R30[4] active-low */
#define PIN_EN_B     (1u << 5)   /* P8_42 PRU1 R30[5] active-low */

#define ALL_PINS  (PIN_STEP_A | PIN_DIR_A | PIN_EN_A | \
                   PIN_STEP_B | PIN_DIR_B | PIN_EN_B)

/* ── 1-second delay at 200 MHz IEP ──────────────────────────────────────── *
 * 200 000 000 cycles = 1 s.  Split into two 100 M halves to avoid the      *
 * unlikely 32-bit wrap case on the very first iteration.                   */
#define HALF_SECOND_CYC   100000000u   /* 0.5 s */

int main(void) {
    /* Safe start: all pins LOW */
    __R30 &= ~ALL_PINS;

    IEP_INIT();

    uint32_t last = IEP_NOW();
    uint8_t  phase = 0u;

    while (1) {
        /* Wait for 0.5 s using rollover-safe subtraction */
        while ((uint32_t)(IEP_NOW() - last) < HALF_SECOND_CYC) { /* spin */ }
        last += HALF_SECOND_CYC;

        /* Toggle all pins every half-period → 0.5 Hz blink (1 s ON, 1 s OFF) */
        phase ^= 1u;
        if (phase)
            __R30 |=  ALL_PINS;
        else
            __R30 &= ~ALL_PINS;
    }

    return 0;
}
