/* pru0_spindle/main.c — PRU0 spindle step generator.
 *
 * Responsibilities:
 *   - Generate STEP/DIR pulses for the spindle stepper motor.
 *   - Execute acceleration/deceleration ramp (via pru_ramp.h).
 *   - Apply spindle-lateral speed compensation from pru_sync.lateral_hz.
 *   - Process commands from the IPC ring buffer.
 *   - Publish spindle telemetry to shared RAM.
 *
 * GPIO mapping (configure via device-tree overlay, pinmux mode 6 = PRU output):
 *   R30[0]  P9_31   SPINDLE_STEP
 *   R30[1]  P9_29   SPINDLE_DIR
 *   R30[2]  P9_30   SPINDLE_ENABLE  (active low: 0 = driver on)
 *
 * Timing budget:
 *   PRU clock   : 200 MHz (5 ns/cycle)
 *   Max step    : 160 kHz  → half-period = 625 cycles
 *   Loop period : LOOP_CYCLES (must be ≤ 625 for 160 kHz support)
 *   Ramp tick   : RAMP_TICK_CYCLES = 200 000 (1 ms)
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_ramp.h"

/* ── PRU register aliases (compatible with pru-gcc) ──────────────────────────*/
volatile uint32_t __R30 __asm__("r30");  /* Output register (GPIO) */
volatile uint32_t __R31 __asm__("r31");  /* Input register  (GPIO) */

/* ── GPIO bit masks (R30) ────────────────────────────────────────────────────*/
#define SPINDLE_STEP_BIT   (1u << 0)
#define SPINDLE_DIR_BIT    (1u << 1)
#define SPINDLE_EN_BIT     (1u << 2)  /* active low */

/* ── Loop timing ─────────────────────────────────────────────────────────────
 * Each main() iteration takes exactly LOOP_CYCLES PRU cycles.
 * The __delay_cycles() call at the top of the loop pads the remainder.
 * LOOP_OVERHEAD must be measured/calibrated with the actual compiled binary.
 */
#define LOOP_CYCLES        500u    /* 2.5 µs per iteration → max 200 kHz edge rate */
#define LOOP_OVERHEAD      18u     /* estimated instruction overhead per iteration */
#define LOOP_DELAY         (LOOP_CYCLES - LOOP_OVERHEAD)

/* ── Telemetry publish interval (in loop iterations) ─────────────────────────*/
#define TELEM_INTERVAL     2000u   /* publish every 2000 × 2.5 µs = 5 ms */

/* ── Shared RAM pointers ─────────────────────────────────────────────────────*/
static volatile pru_cmd_t       *cmd_ring  = (volatile pru_cmd_t *)
    (PRU_SRAM_PHYS_BASE + IPC_CMD_RING_OFFSET);   /* PRU local: 0x00010000 */
static volatile uint32_t        *cmd_whead = (volatile uint32_t *)
    (PRU_SRAM_PHYS_BASE + IPC_CMD_WHEAD_OFFSET);
static volatile uint32_t        *cmd_rhead = (volatile uint32_t *)
    (PRU_SRAM_PHYS_BASE + IPC_CMD_RHEAD_OFFSET);
static volatile pru_axis_telem_t *telem    = (volatile pru_axis_telem_t *)
    (PRU_SRAM_PHYS_BASE + IPC_SPINDLE_TELEM_OFFSET);
static volatile pru_sync_t       *pru_sync = (volatile pru_sync_t *)
    (PRU_SRAM_PHYS_BASE + IPC_SYNC_OFFSET);

/* ── NOTE: on the PRU itself the shared RAM is at local 0x00010000.
 *    Replace the base above with 0x00010000 when compiling with pru-gcc/-clpru.
 *    The #define PRU_SRAM_PHYS_BASE is the host physical address used by Linux.
 *    For PRU firmware, access via local address; Linux accesses via /dev/mem.
 *    See IpcChannel.cpp for the Linux-side mapping.
 */

/* ── Private state ───────────────────────────────────────────────────────────*/
static axis_state_t spindle;
static uint32_t     telem_counter = 0u;

/* ── apply_compensation ──────────────────────────────────────────────────────
 * Adjust spindle target_hz to maintain constant turns-per-mm ratio.
 * Called every ramp tick (1 ms) from process_ramp_compensation().
 *
 * Formula: spindle_hz = nominal_spindle * (lateral_actual / lateral_nominal)
 * Using only 32-bit integer multiply and 32-bit divide (safe, done once/ms).
 */
static void apply_compensation(uint32_t nominal_spindle_hz) {
    uint32_t lat_hz   = pru_sync->lateral_hz;
    uint32_t lat_nom  = pru_sync->nominal_lat_hz;
    uint32_t flags    = pru_sync->control_flags;

    if (!(flags & 1u)) return;         /* compensation disabled */
    if (lat_hz < 100u || lat_nom == 0u) return;

    /* 32-bit: safe as long as nominal_spindle_hz < 4 000 000 000 / 65535 */
    uint32_t comp = (uint32_t)(((uint64_t)nominal_spindle_hz * lat_hz) / lat_nom);
    comp = clamp32(comp, SPEED_HZ_MIN_PRU, 160000u);
    spindle.target_hz = comp;
    pru_sync->spindle_hz = comp;
}

/* ── process_command ─────────────────────────────────────────────────────────
 * Handle one command from the ring buffer. Returns 1 if stop/emergency.
 */
static uint8_t process_command(const volatile pru_cmd_t *cmd,
                                uint32_t *nominal_spindle_hz) {
    if (cmd->axis != AXIS_SPINDLE && cmd->axis != AXIS_ALL) return 0u;

    switch (cmd->cmd) {
        case CMD_SPINDLE_SET_HZ:
            *nominal_spindle_hz = cmd->value_a;
            spindle.target_hz   = cmd->value_a;
            pru_sync->spindle_hz = cmd->value_a;
            break;

        case CMD_SPINDLE_SET_ACCEL:
            spindle.accel_hz_per_tick = cmd->value_a;
            break;

        case CMD_SPINDLE_START:
            ramp_start(&spindle, *nominal_spindle_hz,
                       (uint8_t)(cmd->value_a == 0u ? 1u : 0u),
                       spindle.accel_hz_per_tick);
            if (spindle.direction)
                __R30 |=  SPINDLE_DIR_BIT;
            else
                __R30 &= ~SPINDLE_DIR_BIT;
            break;

        case CMD_SPINDLE_STOP:
            ramp_stop(&spindle);
            break;

        case CMD_SPINDLE_FORCE:
            ramp_force_stop(&spindle);
            __R30 &= ~SPINDLE_STEP_BIT;
            break;

        case CMD_SPINDLE_ENABLE:
            if (cmd->value_a)
                __R30 &= ~SPINDLE_EN_BIT;  /* active low: clear = enable */
            else
                __R30 |=  SPINDLE_EN_BIT;
            break;

        case CMD_SET_NOMINAL_LAT:
            pru_sync->nominal_lat_hz = cmd->value_a;
            break;

        case CMD_EMERGENCY_STOP:
            ramp_force_stop(&spindle);
            __R30 &= ~SPINDLE_STEP_BIT;   /* immediate: STEP low */
            __R30 |=  SPINDLE_EN_BIT;      /* disable driver */
            pru_sync->spindle_hz = 0u;
            return 1u;

        case CMD_RESET_POSITION:
            telem->step_count = 0u;
            break;

        default:
            break;
    }
    return 0u;
}

/* ── publish_telemetry ───────────────────────────────────────────────────────*/
static void publish_telemetry(void) {
    telem->seq         = telem->seq + 1u;
    telem->current_hz  = spindle.current_hz;
    telem->target_hz   = spindle.target_hz;
    /* step_count incremented inline in the main loop */

    uint16_t st = 0u;
    if (!spindle.running)           st |= STATE_IDLE;
    else if (spindle.current_hz < spindle.target_hz) st |= STATE_ACCEL | STATE_RUNNING;
    else if (spindle.current_hz > spindle.target_hz) st |= STATE_DECEL | STATE_RUNNING;
    else                            st |= STATE_RUNNING;
    if (!(__R30 & SPINDLE_EN_BIT))  st |= STATE_ENABLED;
    telem->state  = st;
    telem->faults = 0u;
}

/* ── main ────────────────────────────────────────────────────────────────────*/
int main(void) {
    /* Ensure outputs start safe: STEP low, DIR forward, driver disabled */
    __R30 &= ~SPINDLE_STEP_BIT;
    __R30 &= ~SPINDLE_DIR_BIT;
    __R30 |=  SPINDLE_EN_BIT;    /* active low: high = disabled */

    /* Init ramp state */
    spindle.accel_hz_per_tick = 1000u; /* 1 kHz/ms default */
    pru_sync->spindle_hz      = 0u;
    pru_sync->control_flags   = 1u;    /* compensation enabled by default */

    uint32_t nominal_spindle_hz = 0u;
    uint32_t cmd_local_rhead    = 0u;

    while (1) {
        __delay_cycles(LOOP_DELAY);

        /* ── 1. Drain command ring buffer ───────────────────────────────────*/
        uint32_t wh = *cmd_whead % IPC_CMD_RING_SLOTS;
        while (cmd_local_rhead != wh) {
            uint8_t stop = process_command(&cmd_ring[cmd_local_rhead],
                                           &nominal_spindle_hz);
            cmd_local_rhead = (cmd_local_rhead + 1u) % IPC_CMD_RING_SLOTS;
            *cmd_rhead = cmd_local_rhead;
            if (stop) goto emergency;
        }

        /* ── 2. Ramp tick + step generation ─────────────────────────────────*/
        ramp_tick(&spindle, LOOP_CYCLES);

        if (spindle.step_due) {
            __R30 ^= SPINDLE_STEP_BIT;        /* toggle STEP pin */
            if (__R30 & SPINDLE_STEP_BIT)     /* rising edge = full step */
                telem->step_count++;
            spindle.step_due = 0u;
        }

        /* ── 3. Compensation (once per ramp tick, piggyback on step_counter 0) */
        if (spindle.ramp_counter < (uint32_t)LOOP_CYCLES)  /* just ticked */
            apply_compensation(nominal_spindle_hz);

        /* ── 4. Telemetry ───────────────────────────────────────────────────*/
        if (++telem_counter >= TELEM_INTERVAL) {
            telem_counter = 0u;
            publish_telemetry();
        }
    }

emergency:
    publish_telemetry();
    /* Halt PRU (remoteproc will restart on next load) */
    __asm volatile ("HALT");
    return 0;
}
