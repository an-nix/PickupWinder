/* pru1_orchestration/main.c — PRU1 orchestration/supervision firmware.
 *
 * New authoritative architecture:
 *   - PRU0 = motor control + real-time GPIO (STEP/DIR/EN + endstops).
 *   - PRU1 = orchestration/supervision + host-link side tasks.
 *
 * This PRU1 image intentionally contains NO motor-control logic:
 *   - no STEP/DIR/EN pin writes,
 *   - no move ring consumption,
 *   - no IEP ownership/reset.
 *
 * Command ring and motor telemetry ownership stay on PRU0 to avoid
 * concurrent cmd_rhead advancement and ring corruption.
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_regs.h"

static volatile pru_sync_t *pru_sync =
    (volatile pru_sync_t *)(PRU_SRAM_PHYS_BASE + IPC_SYNC_OFFSET);

/* R31 bits expected for the physical pins when configured for PRU1.
 * Use the same bit indexes as documented for the board pins (15,14).
 * PRU1 will sample its __R31 pins and publish a compact mask into
 * pru_sync->endstop_mask where bit0 = ENDSTOP_1, bit1 = ENDSTOP_2.
 */
#define PRU1_R31_ES1_BIT  (1u << 15)
#define PRU1_R31_ES2_BIT  (1u << 14)

/* Lightweight supervisor heartbeat (private to PRU1 runtime). */
static volatile uint32_t supervisor_heartbeat = 0u;

int main(void) {
    /* PRU1 must never call IEP_INIT(); PRU0 owns the timer. */
    while (1) {
        /* Keep local activity for debug/probe tooling and touch shared sync. */
        supervisor_heartbeat++;
        (void)pru_sync->spindle_interval;
        (void)pru_sync->lateral_interval;
        /* Sample endstops and publish mask for PRU0 to consume. */
        {
            uint32_t mask = 0u;
            uint8_t es1 = (__R31 & PRU1_R31_ES1_BIT) ? 0u : 1u;
            uint8_t es2 = (__R31 & PRU1_R31_ES2_BIT) ? 0u : 1u;
            if (es1) mask |= (1u << 0);
            if (es2) mask |= (1u << 1);
            pru_sync->endstop_mask = mask;
        }
        __asm volatile ("NOP");
    }
    return 0;
}
