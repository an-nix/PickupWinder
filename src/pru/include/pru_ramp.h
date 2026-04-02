/* pru_ramp.h — Fixed-point acceleration/deceleration ramp engine for PRU.
 *
 * RULES:
 *   - Pure C, no floats, no division in the step-generation hot path.
 *   - All inline functions; zero call overhead when compiled with -O2.
 *   - Speeds stored in Hz (uint32_t). Positions in steps (int32_t).
 *   - One ramp tick = RAMP_TICK_CYCLES PRU clock cycles (default 1 ms @ 200 MHz).
 *
 * Usage pattern (inside a PRU step loop):
 *
 *   axis_state_t s = {0};
 *   s.accel_hz_per_tick = 1000;          // 1000 Hz per ms = 1 kHz/ms
 *   s.target_hz         = 42000;
 *
 *   while (running) {
 *       __delay_cycles(LOOP_OVERHEAD_CORRECTION);
 *       ramp_tick(&s);
 *       if (s.step_due) {
 *           TOGGLE_STEP();
 *           s.step_due = 0;
 *       }
 *   }
 */

#ifndef PRU_RAMP_H
#define PRU_RAMP_H

#include <stdint.h>

/* ── PRU clock constants ─────────────────────────────────────────────────────*/
#define PRU_CLOCK_HZ          200000000u  /* 200 MHz */

/* Ramp tick period in PRU cycles.
 * 200 000 cycles = 1 ms. Adjust for tighter or coarser ramps.
 */
#define RAMP_TICK_CYCLES      200000u     /* 1 ms */

/* Minimum period for step pulse high or low half in PRU cycles.
 * 625 cycles = 3.125 µs half-period = 160 kHz max step rate.
 */
#define MIN_HALF_PERIOD_CYCLES 625u

/* Minimum meaningful speed. Below this threshold the axis is considered stopped.*/
#define SPEED_HZ_MIN_PRU      100u        /* 100 Hz */

/* ── Per-axis state ──────────────────────────────────────────────────────────*/
typedef struct {
    /* Configuration (written by command handler, read by ramp engine) */
    uint32_t target_hz;          /* commanded target speed (Hz) */
    uint32_t accel_hz_per_tick;  /* acceleration step per ramp tick (Hz/tick) */

    /* Runtime state (owned by ramp engine) */
    uint32_t current_hz;         /* current speed — changes toward target_hz */
    uint32_t half_period_cycles; /* half-period = PRU_CLOCK_HZ / (2 × current_hz) */
    uint32_t ramp_counter;       /* cycles elapsed since last ramp tick */
    uint32_t step_counter;       /* cycles elapsed since last step edge */

    /* Output signals */
    uint8_t  step_due;           /* 1 = caller must toggle STEP pin this iteration */
    uint8_t  direction;          /* 0 = forward, 1 = backward */
    uint8_t  running;            /* 1 = step generation active */
    uint8_t  _pad;
} axis_state_t;

/* ── hz_to_halfperiod ────────────────────────────────────────────────────────
 * Convert step frequency (Hz) to half-period in PRU cycles.
 * Returns 0 when hz == 0 to indicate "stopped".
 * Uses only 32-bit arithmetic safe on PRU.
 */
static inline uint32_t hz_to_halfperiod(uint32_t hz) {
    if (hz < SPEED_HZ_MIN_PRU) return 0u;
    uint32_t full = PRU_CLOCK_HZ / hz;
    return (full < 2u) ? 1u : (full >> 1u);
}

/* ── clamp32 ─────────────────────────────────────────────────────────────────*/
static inline uint32_t clamp32(uint32_t v, uint32_t lo, uint32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ── ramp_tick ───────────────────────────────────────────────────────────────
 * Must be called once per PRU loop iteration.
 * `elapsed_cycles` is the number of PRU cycles consumed by the current iteration
 * (measured or fixed via __delay_cycles).
 *
 * After call:
 *   - s->step_due == 1   → caller must toggle the STEP GPIO, then clear step_due.
 *   - s->current_hz      → updated speed.
 *   - s->direction       → current direction.
 */
static inline void ramp_tick(axis_state_t *s, uint32_t elapsed_cycles) {
    if (!s->running) {
        s->step_due = 0;
        return;
    }

    /* ── Ramp update (every RAMP_TICK_CYCLES) ──────────────────────────────*/
    s->ramp_counter += elapsed_cycles;
    if (s->ramp_counter >= RAMP_TICK_CYCLES) {
        s->ramp_counter -= RAMP_TICK_CYCLES;

        if (s->current_hz < s->target_hz) {
            s->current_hz += s->accel_hz_per_tick;
            if (s->current_hz > s->target_hz)
                s->current_hz = s->target_hz;
        } else if (s->current_hz > s->target_hz) {
            if (s->current_hz <= s->accel_hz_per_tick)
                s->current_hz = 0u;
            else
                s->current_hz -= s->accel_hz_per_tick;
            if (s->current_hz < s->target_hz)
                s->current_hz = s->target_hz;
        }

        /* Recompute half-period each ramp tick (avoid division in the step path) */
        s->half_period_cycles = hz_to_halfperiod(s->current_hz);

        /* Auto-stop when speed reaches 0 and target is 0 */
        if (s->current_hz == 0u && s->target_hz == 0u) {
            s->running   = 0u;
            s->step_due  = 0u;
            return;
        }
    }

    /* ── Step generation ───────────────────────────────────────────────────*/
    s->step_due = 0u;
    if (s->half_period_cycles == 0u) return;

    s->step_counter += elapsed_cycles;
    if (s->step_counter >= s->half_period_cycles) {
        s->step_counter -= s->half_period_cycles;
        s->step_due = 1u;
    }
}

/* ── ramp_start ──────────────────────────────────────────────────────────────
 * Begin step generation: set direction and kick off from current speed
 * (or from 0 if stopped).
 */
static inline void ramp_start(axis_state_t *s, uint32_t target_hz,
                               uint8_t direction, uint32_t accel_hz_per_tick) {
    s->target_hz        = target_hz;
    s->direction        = direction;
    s->accel_hz_per_tick= accel_hz_per_tick;
    s->running          = 1u;
    s->step_counter     = 0u;
    s->ramp_counter     = 0u;
    /* current_hz kept as-is to allow smooth speed changes without jolts */
    s->half_period_cycles = hz_to_halfperiod(s->current_hz);
}

/* ── ramp_stop ───────────────────────────────────────────────────────────────
 * Request controlled deceleration to 0.
 */
static inline void ramp_stop(axis_state_t *s) {
    s->target_hz = 0u;
}

/* ── ramp_force_stop ─────────────────────────────────────────────────────────
 * Immediate stop (no ramp). Use for emergency stop only.
 */
static inline void ramp_force_stop(axis_state_t *s) {
    s->target_hz          = 0u;
    s->current_hz         = 0u;
    s->half_period_cycles = 0u;
    s->step_counter       = 0u;
    s->ramp_counter       = 0u;
    s->running            = 0u;
    s->step_due           = 0u;
}

#endif /* PRU_RAMP_H */
