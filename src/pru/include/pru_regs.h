#ifndef PRU_REGS_H
#define PRU_REGS_H

#include <stdint.h>

/* PRU R30/R31 hardware register aliases.
 *
 * Two compiler variants:
 *
 * pru-elf-gcc (open-source):
 *   CRITICAL: 'register' + '__asm__("r30")' are BOTH required.
 *   Without 'register', __asm__("r30") on a global creates a RAM variable
 *   named r30 — writes never reach the physical pins.
 *
 * clpru (TI):
 *   Uses 'volatile register unsigned int __R30' WITHOUT __asm__.
 *   clpru recognises R30/R31 by name natively; __asm__ is not supported.
 *
 * R30 = direct output  → motor firmware STEP/DIR/EN (P8 header, PRU1)
 * R31 = direct input   → orchestrator endstop pins  (P9_28/P9_30, PRU0)
 */
#ifdef __TI_COMPILER_VERSION__
volatile register unsigned int __R30;
volatile register unsigned int __R31;
#else
register volatile uint32_t __R30 __asm__("r30");
register volatile uint32_t __R31 __asm__("r31");
#endif

#endif /* PRU_REGS_H */

