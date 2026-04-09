/* test_static/main.c — PRU0 ultra-minimal static output test.
 *
 * Sets ALL stepper pins HIGH immediately, then loops doing nothing.
 * No IEP timer, no delay, no pulse_gen.
 *
 * If pins stay LOW after loading → R30 writes don't reach the pads
 *   (pinmux issue, wrong PRU loaded, or wrong remoteproc).
 * If pins go HIGH → R30 works; the blink test's IEP timer was blocking.
 *
 * Expected: P9_25, P9_27, P9_28, P9_29, P9_30, P9_31 all go to 3.3V
 *           immediately after PRU0 start.
 *
 * Load:
 *   sudo rm /lib/firmware/am335x-pru0-fw
 *   sudo cp am335x-pru0-fw-test-static /lib/firmware/am335x-pru0-fw
 *   sudo sh -c 'echo stop  > /sys/class/remoteproc/remoteproc1/state'
 *   sleep 1
 *   sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state'
 */

#include <stdint.h>
#include "../include/pru_regs.h"   /* __R30 */

#define PIN_STEP_A   (1u << 1)   /* P9_29 */
#define PIN_DIR_A    (1u << 5)   /* P9_27 */
#define PIN_EN_A     (1u << 7)   /* P9_25 */
#define PIN_STEP_B   (1u << 2)   /* P9_30 */
#define PIN_DIR_B    (1u << 0)   /* P9_31 */
#define PIN_EN_B     (1u << 3)   /* P9_28 */

#define ALL_PINS  (PIN_STEP_A | PIN_DIR_A | PIN_EN_A | \
                   PIN_STEP_B | PIN_DIR_B | PIN_EN_B)

int main(void) {
    /* Set all 6 stepper pins HIGH immediately — no timer, no delay. */
    __R30 |= ALL_PINS;

    /* Loop forever doing nothing (pins stay HIGH). */
    while (1) { /* spin */ }

    return 0;
}
