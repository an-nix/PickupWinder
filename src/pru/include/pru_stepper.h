/* pru_stepper.h — IEP timer access + continuous pulse generation for PRU0.
 *
 * Provides:
 *   - IEP hardware counter macros (200 MHz, 5 ns/cycle).
 *   - timer_before() for 32-bit rollover-safe comparison.
 *   - pulse_gen_t: minimal state for continuous IEP-timed pulse generation.
 *     PRU0 calls pulse_update() each loop iteration; it toggles the STEP pin
 *     when the IEP counter reaches the next edge time.
 *
 * Design principle: PRU0 is a dumb pulse generator. It reads interval/dir/run
 * from motor_params_t (written by PRU1) and generates edges. No ramps, no
 * move ring, no segment chaining. Speed changes take effect immediately.
 *
 * Include only from PRU firmware. No dynamic memory, no float, no division.
 */

#ifndef PRU_STEPPER_H
#define PRU_STEPPER_H

#include <stdint.h>

/* ── IEP register access ────────────────────────────────────────────────────
 * AM335x IEP base: PRU constant table entry 17 → local 0x0002E000.
 * Both PRU0 and PRU1 share the same hardware counter.                      */
#define PRU_IEP_BASE          0x0002E000u
#define PRU_IEP_GLB_CFG       (*(volatile uint32_t *)(PRU_IEP_BASE + 0x00u))
#define PRU_IEP_TMR_CNT       (*(volatile uint32_t *)(PRU_IEP_BASE + 0x0Cu))

#define IEP_NOW()             PRU_IEP_TMR_CNT

/* Call once on boot from PRU0 only. PRU1 reads but never resets the counter. */
#define IEP_INIT()  do { PRU_IEP_GLB_CFG = 0x11u; PRU_IEP_TMR_CNT = 0u; } while(0)

/* ── Timing constants ────────────────────────────────────────────────────── */
#define MIN_INTERVAL_CYC   625u   /* 160 kHz max → 3.125 µs minimum         */
#define DIR_SETUP_CYC       40u   /* 200 ns A4988 dir-setup = 40 IEP cycles */

/* ── 32-bit rollover-safe timer comparison ───────────────────────────────── */
static inline int timer_before(uint32_t t1, uint32_t t2) {
    return (int32_t)(t1 - t2) < 0;
}

/* ── Continuous pulse generator state ────────────────────────────────────── */
typedef struct {
    uint32_t next_edge_time;    /* absolute IEP value of next STEP edge      */
    uint32_t interval;          /* current IEP interval between edges        */
    uint32_t step_count;        /* physical steps (rising edges) since reset */
    int32_t  position;          /* signed step position (lateral only)       */
    uint8_t  direction;         /* current direction (0=fwd, 1=rev)          */
    uint8_t  step_pin_state;    /* current STEP pin logic level (0 or 1)    */
    uint8_t  running;           /* 1 → actively generating pulses            */
    uint8_t  _pad;
} pulse_gen_t;

/* ── pulse_update ────────────────────────────────────────────────────────────
 * Called every PRU0 main-loop iteration for one axis.
 *
 * - If new_interval is 0 or run==0 → stop generating pulses.
 * - If interval changed → adopt new interval immediately.
 * - If IEP has reached next_edge_time → toggle STEP, count rising edges.
 *
 * Returns the new STEP pin state (0 or 1). Caller writes to __R30.
 *
 * Parameters:
 *   pg           pulse generator state
 *   new_interval desired IEP interval (from motor_params); 0 = stop
 *   new_dir      desired direction (from motor_params)
 *   run          1 = should be generating pulses, 0 = idle
 *   now          current IEP_NOW() value
 */
static inline uint8_t pulse_update(pulse_gen_t *pg,
                                   uint32_t new_interval,
                                   uint8_t  new_dir,
                                   uint8_t  run,
                                   uint32_t now)
{
    /* ── Stop condition ──────────────────────────────────────────────────── */
    if (!run || new_interval == 0u) {
        if (pg->running) {
            pg->running       = 0u;
            pg->step_pin_state = 0u;
        }
        return 0u;
    }

    /* ── Clamp interval ──────────────────────────────────────────────────── */
    if (new_interval < MIN_INTERVAL_CYC)
        new_interval = MIN_INTERVAL_CYC;

    /* ── Start from idle ─────────────────────────────────────────────────── */
    if (!pg->running) {
        pg->interval       = new_interval;
        pg->direction      = new_dir;
        pg->next_edge_time = now + new_interval;
        pg->running        = 1u;
        pg->step_pin_state = 0u;
        return 0u;  /* first edge will fire on next pass */
    }

    /* ── Adopt new interval (instant speed change) ───────────────────────── */
    pg->interval  = new_interval;
    pg->direction = new_dir;

    /* ── Check if time to toggle ─────────────────────────────────────────── */
    if (!timer_before(now, pg->next_edge_time)) {
        pg->step_pin_state ^= 1u;

        if (pg->step_pin_state) {
            /* Rising edge = physical step */
            pg->step_count++;
            if (!pg->direction) pg->position++;
            else                pg->position--;
        }

        pg->next_edge_time += pg->interval;
    }

    return pg->step_pin_state;
}

/* ── pulse_stop — immediate halt ─────────────────────────────────────────── */
static inline void pulse_stop(pulse_gen_t *pg) {
    pg->running        = 0u;
    pg->step_pin_state = 0u;
    pg->interval       = 0u;
}

/* ── pulse_reset_counters ────────────────────────────────────────────────── */
static inline void pulse_reset_counters(pulse_gen_t *pg) {
    pg->step_count = 0u;
    pg->position   = 0;
}

#endif /* PRU_STEPPER_H */
