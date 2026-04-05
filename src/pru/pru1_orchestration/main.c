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

/* Lightweight supervisor heartbeat (private to PRU1 runtime). */
static volatile uint32_t supervisor_heartbeat = 0u;

int main(void) {
    /* PRU1 must never call IEP_INIT(); PRU0 owns the timer. */
    while (1) {
        /* Keep local activity for debug/probe tooling and touch shared sync. */
        supervisor_heartbeat++;
        (void)pru_sync->spindle_interval;
        (void)pru_sync->lateral_interval;
        __asm volatile ("NOP");
    }
    return 0;
}
