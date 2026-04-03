/* pru_stepper.h — IEP timer + Klipper-style step-move engine for PRU.
 *
 * Mirrors Klipper's stepper_event_edge() architecture:
 *   - Uses the AM335x IEP hardware counter (200 MHz) as absolute clock.
 *   - Host pre-computes (interval, count, add): no ramp logic on-PRU.
 *   - stepper_edge() is called when iep_now() >= next_edge_time.
 *   - Moves chain seamlessly: next_edge_time advances from the last edge time.
 *
 * Include only from PRU firmware. No dynamic memory, no float, no division.
 *
 * IEP access: direct volatile pointer to local address 0x0002E000.
 * Both PRU0 and PRU1 can access IEP via this local address (AM335x TRM).
 * PRU0 calls iep_init(); PRU1 only reads IEP_NOW().
 */

#ifndef PRU_STEPPER_H
#define PRU_STEPPER_H

#include <stdint.h>
#include "pru_ipc.h"

/* ── IEP register access (no TI PSSP dependency) ────────────────────────────
 * AM335x IEP base: PRU constant table entry 17 → local 0x0002E000.
 * Both PRU0 and PRU1 share the same hardware counter.
 */
#define PRU_IEP_BASE          0x0002E000u
#define PRU_IEP_GLB_CFG       (*(volatile uint32_t *)(PRU_IEP_BASE + 0x00u))
#define PRU_IEP_TMR_CNT       (*(volatile uint32_t *)(PRU_IEP_BASE + 0x0Cu))

#define IEP_NOW()             PRU_IEP_TMR_CNT

/* Call once on boot from PRU0 only. PRU1 reads but never resets the counter. */
#define IEP_INIT()  do { PRU_IEP_GLB_CFG = 0x11u; PRU_IEP_TMR_CNT = 0u; } while(0)

/* ── Timing constants ────────────────────────────────────────────────────────*/
#define MIN_INTERVAL_CYC   625u   /* 160 kHz max → 625 ticks min interval   */
#define DIR_SETUP_CYC       40u   /* 200 ns A4988 dir-setup = 40 IEP cycles */

/* 32-bit rollover-safe "is t1 before t2?" */
static inline int timer_before(uint32_t t1, uint32_t t2) {
    return (int32_t)(t1 - t2) < 0;
}

/* ── Per-axis stepper state ──────────────────────────────────────────────────*/
typedef struct {
    uint32_t  next_edge_time;   /* absolute IEP counter value of next edge   */
    uint32_t  interval;         /* IEP ticks between edges in current move   */
    int32_t   add;              /* per-edge signed change to interval        */
    uint32_t  count;            /* edges remaining in current move           */
    uint32_t  step_count;       /* physical steps (rising edges) since reset */
    int32_t   position;         /* signed step position (lateral only)       */
    uint8_t   direction;        /* current commanded direction (0/1)         */
    uint8_t   dir_pending;      /* 1 → main loop must update DIR GPIO        */
    uint8_t   running;          /* 1 → currently executing a move            */
    uint8_t   step_pin_state;   /* current STEP pin logic level (0 or 1)     */
    uint8_t   underrun;         /* 1 → ring drained while running (fault)    */
} stepper_t;

/* ── Move-ring context ───────────────────────────────────────────────────────*/
typedef struct {
    volatile pru_move_t  *ring;
    volatile uint32_t    *whead;
    volatile uint32_t    *rhead;
    uint32_t              slots;
} move_ring_t;

/* ── stepper_ring_count ──────────────────────────────────────────────────────*/
static inline uint32_t stepper_ring_count(const move_ring_t *mr) {
    uint32_t wh = *mr->whead % mr->slots;
    uint32_t rh = *mr->rhead % mr->slots;
    return (wh + mr->slots - rh) % mr->slots;
}

/* ── stepper_try_load_next ───────────────────────────────────────────────────
 * Pop one move from ring and chain it seamlessly to the previous edge.
 * next_edge_time is advanced by the new move's interval.
 * Returns 1 on success, 0 if ring empty.
 */
static inline int stepper_try_load_next(stepper_t *s, const move_ring_t *mr) {
    uint32_t rh = *mr->rhead % mr->slots;
    uint32_t wh = *mr->whead % mr->slots;
    if (rh == wh) return 0;

    volatile const pru_move_t *m = &mr->ring[rh];

    uint32_t new_iv = m->interval;
    if (new_iv < MIN_INTERVAL_CYC) new_iv = MIN_INTERVAL_CYC;

    /* Direction change: signal main loop to update DIR GPIO + add setup time */
    if (m->direction != s->direction) {
        s->direction   = m->direction;
        s->dir_pending = 1u;
        s->next_edge_time += DIR_SETUP_CYC;
    }

    s->interval       = new_iv;
    s->add            = m->add;
    s->count          = m->count;
    s->next_edge_time += new_iv;   /* seamless chain from last edge time */
    s->running        = 1u;

    /* Pre-apply add for first inter-edge gap (matches Klipper stepper_load_next). */
    int32_t niv = (int32_t)s->interval + s->add;
    if (niv < (int32_t)MIN_INTERVAL_CYC) niv = (int32_t)MIN_INTERVAL_CYC;
    s->interval = (uint32_t)niv;

    *mr->rhead = (rh + 1u) % mr->slots;
    return 1;
}

/* ── stepper_start_first ─────────────────────────────────────────────────────
 * Load the first move using current IEP time as base (fresh start).
 * Returns 1 on success, 0 if ring empty.
 */
static inline int stepper_start_first(stepper_t *s, const move_ring_t *mr) {
    uint32_t rh = *mr->rhead % mr->slots;
    uint32_t wh = *mr->whead % mr->slots;
    if (rh == wh) return 0;

    volatile const pru_move_t *m = &mr->ring[rh];

    uint32_t new_iv = m->interval;
    if (new_iv < MIN_INTERVAL_CYC) new_iv = MIN_INTERVAL_CYC;

    if (m->direction != s->direction) {
        s->direction   = m->direction;
        s->dir_pending = 1u;
    }

    s->interval       = new_iv;
    s->add            = m->add;
    s->count          = m->count;
    s->next_edge_time = IEP_NOW() + new_iv;
    s->running        = 1u;

    /* Pre-apply add for first inter-edge gap (matches Klipper stepper_load_next). */
    int32_t niv = (int32_t)s->interval + s->add;
    if (niv < (int32_t)MIN_INTERVAL_CYC) niv = (int32_t)MIN_INTERVAL_CYC;
    s->interval = (uint32_t)niv;

    *mr->rhead = (rh + 1u) % mr->slots;
    return 1;
}

/* ── stepper_edge ────────────────────────────────────────────────────────────
 * Called when IEP_NOW() >= next_edge_time.
 * Mirrors Klipper stepper_event_edge():
 *   1. toggle STEP pin
 *   2. count rising edges (physical steps)
 *   3. advance next_edge_time by CURRENT interval, THEN apply add
 *   4. when move exhausted: chain next or stop
 *
 * Returns: new STEP pin state (0 or 1). Caller writes this to __R30.
 *          Sets s->underrun = 1 if ring drained mid-move (caller should set FAULT).
 */
static inline uint8_t stepper_edge(stepper_t *s, const move_ring_t *mr) {
    s->step_pin_state ^= 1u;

    if (s->step_pin_state) {
        /* Rising edge = physical step */
        s->step_count++;
        if (!s->direction) s->position++;
        else               s->position--;
    }

    s->count--;
    if (s->count == 0u) {
        if (!stepper_try_load_next(s, mr)) {
            s->running  = 0u;
            s->underrun = 1u;   /* ring drained — host didn't keep up */
        }
    } else {
        /* Klipper order: advance by CURRENT interval, then update interval */
        s->next_edge_time += s->interval;
        int32_t niv = (int32_t)s->interval + s->add;
        if (niv < (int32_t)MIN_INTERVAL_CYC) niv = (int32_t)MIN_INTERVAL_CYC;
        s->interval = (uint32_t)niv;
    }

    return s->step_pin_state;
}

/* ── stepper_force_stop  (emergency stop or flush) ───────────────────────────*/
static inline void stepper_force_stop(stepper_t *s) {
    s->running        = 0u;
    s->count          = 0u;
    s->step_pin_state = 0u;
    s->next_edge_time = 0u;
    s->underrun       = 0u;   /* intentional stop — not an underrun */
}

/* ── stepper_flush_ring  discard all pending moves ───────────────────────────*/
static inline void stepper_flush_ring(const move_ring_t *mr) {
    *mr->rhead = *mr->whead % mr->slots;
}

/* ── hz_to_interval  (host helper, not for hot path) ────────────────────────
 * interval = PRU_CLOCK_HZ / (2 × step_hz)
 * (two pin edges per physical step → half the step period)
 */
static inline uint32_t hz_to_interval(uint32_t step_hz) {
    if (step_hz == 0u) return 0u;
    uint32_t iv = PRU_CLOCK_HZ / (2u * step_hz);
    return (iv < MIN_INTERVAL_CYC) ? MIN_INTERVAL_CYC : iv;
}

#endif /* PRU_STEPPER_H */
