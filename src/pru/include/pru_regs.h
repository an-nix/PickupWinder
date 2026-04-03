#ifndef PRU_REGS_H
#define PRU_REGS_H

#include <stdint.h>

/* PRU R30/R31 aliases — shared between PRU0 and PRU1 */
volatile uint32_t __R30 __asm__("r30");
volatile uint32_t __R31 __asm__("r31");

#endif /* PRU_REGS_H */
