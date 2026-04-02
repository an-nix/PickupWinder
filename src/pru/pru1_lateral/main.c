/* pru1_lateral/main.c — PRU1 lateral step generator + home sensor.
 *
 * Responsibilities:
 *   - Generate STEP/DIR pulses for the lateral carriage stepper.
 *   - Execute acceleration/deceleration ramp (via pru_ramp.h).
 *   - Sample the dual-contact home sensor (NO+NC) with hardware debounce.
 *   - Detect reversal bounds and update STATE_* flags in lateral_telem.
 *   - Write lateral_hz to pru_sync so PRU0 can apply compensation.
 *   - Process commands from the IPC ring buffer.
 *   - Publish lateral telemetry to shared RAM.
 *
 * GPIO mapping (configure via device-tree overlay, pinmux mode 6 = PRU output,
 *              mode 6 = PRU input for R31 pins):
 *   R30[0]  P8_45   LATERAL_STEP
 *   R30[1]  P8_46   LATERAL_DIR
 *   R30[2]  P8_43   LATERAL_ENABLE   (active low)
 *   R31[3]  P8_44   HOME_NO          (normally-open; LOW when at home)
 *   R31[4]  P8_41   HOME_NC          (normally-closed; HIGH when at home)
 *
 * Sensor logic (INPUT_PULLUP convention, sensor pulls to GND):
 *   Away from home  : NO=HIGH (open), NC=LOW  (closed)
 *   At home         : NO=LOW  (closed), NC=HIGH (open)
 *   Fault           : NO=HIGH, NC=HIGH (both open → cable fault)
 *   Fault           : NO=LOW,  NC=LOW  (both closed → wiring short)
 *
 * Debounce: 3 consecutive stable reads at DEBOUNCE_INTERVAL_CYCLES apart.
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_ramp.h"

/* ── PRU register aliases ────────────────────────────────────────────────────*/
volatile register uint32_t __R30;
volatile register uint32_t __R31;

/* ── GPIO bit masks ──────────────────────────────────────────────────────────*/
#define LAT_STEP_BIT   (1u << 0)
#define LAT_DIR_BIT    (1u << 1)
#define LAT_EN_BIT     (1u << 2)   /* active low */
#define HOME_NO_BIT    (1u << 3)   /* R31: LOW when at home */
#define HOME_NC_BIT    (1u << 4)   /* R31: HIGH when at home */

/* ── Loop timing ─────────────────────────────────────────────────────────────
 * Lateral max speed is much lower (~4800 Hz) so we can afford a longer loop
 * period and dedicate more cycles to sensor sampling.
 * Loop period: 2000 cycles = 10 µs  → max step rate 50 kHz (well above needed).
 */
#define LOOP_CYCLES        2000u
#define LOOP_OVERHEAD      22u
#define LOOP_DELAY         (LOOP_CYCLES - LOOP_OVERHEAD)

/* Sensor debounce: 3 stable reads × 10 µs = 30 µs total */
#define DEBOUNCE_COUNT     3u

/* Software step position limits (steps from home). Linux sets these via commands.*/
#define POS_LIMIT_MIN     (-10000)  /* overshoot protection below home */
#define POS_LIMIT_MAX_DEF (307200)  /* ~100 mm at 3072 steps/mm */

/* Telemetry interval */
#define TELEM_INTERVAL     500u     /* 500 × 10 µs = 5 ms */

/* ── Shared RAM pointers ─────────────────────────────────────────────────────
 * NOTE: In PRU firmware use local address 0x00010000.
 *       PRU_SRAM_PHYS_BASE is the Linux host address for IpcChannel.cpp.
 *       Replace base with 0x00010000u when compiling for the PRU target.
 */
static volatile pru_cmd_t       *cmd_ring  = (volatile pru_cmd_t *)
    (PRU_SRAM_PHYS_BASE + IPC_CMD_RING_OFFSET);
static volatile uint32_t        *cmd_whead = (volatile uint32_t *)
    (PRU_SRAM_PHYS_BASE + IPC_CMD_WHEAD_OFFSET);
static volatile uint32_t        *cmd_rhead = (volatile uint32_t *)
    (PRU_SRAM_PHYS_BASE + IPC_CMD_RHEAD_OFFSET);
static volatile pru_axis_telem_t *telem    = (volatile pru_axis_telem_t *)
    (PRU_SRAM_PHYS_BASE + IPC_LATERAL_TELEM_OFFSET);
static volatile pru_sync_t       *pru_sync = (volatile pru_sync_t *)
    (PRU_SRAM_PHYS_BASE + IPC_SYNC_OFFSET);

/* ── Private state ───────────────────────────────────────────────────────────*/
static axis_state_t lateral;
static int32_t      position_steps = 0;
static int32_t      pos_limit_max  = POS_LIMIT_MAX_DEF;
static uint32_t     telem_counter  = 0u;

/* ── Home sensor state ───────────────────────────────────────────────────────*/
typedef enum {
    HOME_AWAY  = 0,
    HOME_HIT   = 1,
    HOME_FAULT = 2,
} home_state_t;

static home_state_t home_state      = HOME_AWAY;
static home_state_t home_state_prev = HOME_AWAY;
static uint8_t      debounce_count  = 0u;
static uint8_t      home_just_hit   = 0u;  /* single-shot latch */

/* ── read_home_raw ───────────────────────────────────────────────────────────
 * Read instantaneous sensor state from R31.
 */
static inline home_state_t read_home_raw(void) {
    uint8_t no = (__R31 & HOME_NO_BIT) ? 1u : 0u;
    uint8_t nc = (__R31 & HOME_NC_BIT) ? 1u : 0u;

    if (!no && nc)  return HOME_HIT;    /* NO=LOW, NC=HIGH */
    if (!no && !nc) return HOME_FAULT;  /* both closed → short */
    if (no  && nc)  return HOME_FAULT;  /* both open  → disconnected */
    return HOME_AWAY;                   /* no=HIGH, nc=LOW → normal */
}

/* ── sample_home ─────────────────────────────────────────────────────────────
 * Call once per loop iteration. Updates home_state with 3-sample debounce.
 */
static void sample_home(void) {
    home_state_t raw = read_home_raw();
    if (raw == home_state_prev) {
        if (debounce_count < DEBOUNCE_COUNT)
            debounce_count++;
    } else {
        home_state_prev = raw;
        debounce_count  = 0u;
    }

    if (debounce_count >= DEBOUNCE_COUNT) {
        if (raw != home_state) {
            home_state = raw;
            if (home_state == HOME_HIT)
                home_just_hit = 1u;   /* pulse for command handler to consume */
        }
    }
}

/* ── process_command ─────────────────────────────────────────────────────────*/
static uint8_t process_command(const volatile pru_cmd_t *cmd) {
    if (cmd->axis != AXIS_LATERAL && cmd->axis != AXIS_ALL) return 0u;

    switch (cmd->cmd) {
        case CMD_LATERAL_SET_HZ:
            lateral.target_hz = cmd->value_a;
            pru_sync->lateral_hz = cmd->value_a;
            break;

        case CMD_LATERAL_SET_ACCEL:
            lateral.accel_hz_per_tick = cmd->value_a;
            break;

        case CMD_LATERAL_START:
            ramp_start(&lateral, lateral.target_hz,
                       (uint8_t)(cmd->value_a == 0u ? 1u : 0u),
                       lateral.accel_hz_per_tick);
            if (lateral.direction)
                __R30 |=  LAT_DIR_BIT;
            else
                __R30 &= ~LAT_DIR_BIT;
            break;

        case CMD_LATERAL_STOP:
            ramp_stop(&lateral);
            break;

        case CMD_LATERAL_FORCE:
            ramp_force_stop(&lateral);
            __R30 &= ~LAT_STEP_BIT;
            break;

        case CMD_LATERAL_ENABLE:
            if (cmd->value_a)
                __R30 &= ~LAT_EN_BIT;   /* active low */
            else
                __R30 |=  LAT_EN_BIT;
            break;

        case CMD_RESET_POSITION:
            position_steps   = 0;
            telem->step_count = 0u;
            telem->position_steps = 0;
            break;

        case CMD_EMERGENCY_STOP:
            ramp_force_stop(&lateral);
            __R30 &= ~LAT_STEP_BIT;
            __R30 |=  LAT_EN_BIT;        /* disable driver */
            pru_sync->lateral_hz = 0u;
            return 1u;

        default:
            break;
    }
    return 0u;
}

/* ── update_position ─────────────────────────────────────────────────────────
 * Increment/decrement signed position counter on each step edge.
 * Enforce software limits and raise FAULT_OVERRUN if breached.
 */
static void update_position(void) {
    if (!lateral.direction)
        position_steps++;
    else
        position_steps--;

    /* Software limit check */
    if (position_steps < POS_LIMIT_MIN || position_steps > pos_limit_max) {
        ramp_force_stop(&lateral);
        __R30 &= ~LAT_STEP_BIT;
        telem->faults |= FAULT_OVERRUN;
    }

    telem->position_steps = position_steps;
    telem->step_count++;
}

/* ── publish_telemetry ───────────────────────────────────────────────────────*/
static void publish_telemetry(void) {
    telem->seq        = telem->seq + 1u;
    telem->current_hz = lateral.current_hz;
    telem->target_hz  = lateral.target_hz;
    pru_sync->lateral_hz = lateral.current_hz;

    uint16_t st = 0u;
    if (!lateral.running)                  st |= STATE_IDLE;
    else if (lateral.current_hz < lateral.target_hz) st |= STATE_ACCEL | STATE_RUNNING;
    else if (lateral.current_hz > lateral.target_hz) st |= STATE_DECEL | STATE_RUNNING;
    else                                   st |= STATE_RUNNING;
    if (home_state == HOME_HIT)            st |= STATE_AT_HOME;
    if (home_state == HOME_FAULT)          st |= STATE_FAULT;
    if (!(__R30 & LAT_EN_BIT))             st |= STATE_ENABLED;
    telem->state = st;
}

/* ── main ────────────────────────────────────────────────────────────────────*/
int main(void) {
    /* Safe output state: STEP low, DIR forward, driver disabled */
    __R30 &= ~LAT_STEP_BIT;
    __R30 &= ~LAT_DIR_BIT;
    __R30 |=  LAT_EN_BIT;    /* active low: high = disabled */

    lateral.accel_hz_per_tick = 48u;  /* 48 Hz/ms → 4800 Hz in 100 ms */
    pru_sync->lateral_hz = 0u;

    uint32_t cmd_local_rhead = 0u;

    while (1) {
        __delay_cycles(LOOP_DELAY);

        /* ── 1. Sample home sensor ──────────────────────────────────────────*/
        sample_home();

        /* Stop if home detected while homing (direction backward toward home) */
        if (home_just_hit && lateral.running && lateral.direction == 1u) {
            ramp_force_stop(&lateral);
            __R30 &= ~LAT_STEP_BIT;
            position_steps = 0;
            telem->step_count = 0u;
            telem->state |= STATE_AT_HOME;
            home_just_hit = 0u;
        } else {
            home_just_hit = 0u;
        }

        if (home_state == HOME_FAULT) {
            telem->faults |= FAULT_HOME_SENSOR;
        }

        /* ── 2. Drain command ring buffer ───────────────────────────────────*/
        uint32_t wh = *cmd_whead % IPC_CMD_RING_SLOTS;
        while (cmd_local_rhead != wh) {
            uint8_t stop = process_command(&cmd_ring[cmd_local_rhead]);
            cmd_local_rhead = (cmd_local_rhead + 1u) % IPC_CMD_RING_SLOTS;
            *cmd_rhead = cmd_local_rhead;
            if (stop) goto emergency;
        }

        /* ── 3. Ramp tick + step generation ─────────────────────────────────*/
        ramp_tick(&lateral, LOOP_CYCLES);

        if (lateral.step_due) {
            __R30 ^= LAT_STEP_BIT;
            if (__R30 & LAT_STEP_BIT)   /* rising edge */
                update_position();
            lateral.step_due = 0u;
            pru_sync->lateral_hz = lateral.current_hz;
        }

        /* ── 4. Telemetry ───────────────────────────────────────────────────*/
        if (++telem_counter >= TELEM_INTERVAL) {
            telem_counter = 0u;
            publish_telemetry();
        }
    }

emergency:
    publish_telemetry();
    __asm volatile ("HALT");
    return 0;
}
