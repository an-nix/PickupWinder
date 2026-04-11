/* test_iep_simple/main.c — test that IEP_NOW() returns incrementing values */

#include <stdint.h>
#include "../include/pru_stepper.h"
#include "../include/pru_regs.h"

int main(void) {
    __R30 = 0u;                                /* all pins low first */
    __R30 |= (1u << 4) | (1u << 5);           /* EN bits high (disabled) */

    IEP_INIT();

    /* Toggle spindle STEP pin every time IEP counter increases by 10M cycles */
    uint32_t threshold = 10000000u;
    uint32_t next_edge = threshold;

    for (int i = 0; i < 100; i++) {
        uint32_t now = IEP_NOW();

        if (now >= next_edge) {
            __R30 ^= (1u << 1);  /* toggle spindle STEP */
            next_edge += threshold;
        }
    }

    return 0;
}
