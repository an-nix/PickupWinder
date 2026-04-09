#ifndef PRU_REGS_H
#define PRU_REGS_H

#include <stdint.h>

/* PRU R30/R31 hardware register aliases.
 *
 * CRITICAL: 'register' is REQUIRED.
 *
 * Without 'register', __asm__("r30") on a global variable only assigns an
 * assembly symbol name to a memory location — writes to __R30 go to RAM
 * and never reach the physical output pins.
 *
 * With 'register', GCC maps __R30 directly to hardware register r30.
 * Writes to __R30 are emitted as "or r30, r30, IMM" instructions which
 * drive the physical GPIO pins (R30 = PRU direct output register).
 *
 * R30 = PRU0 direct output  → STEP/DIR/EN pins
 * R31 = PRU0 direct input   → endstop pins + interrupt bits
 */
register volatile uint32_t __R30 __asm__("r30");
register volatile uint32_t __R31 __asm__("r31");

#endif /* PRU_REGS_H */
