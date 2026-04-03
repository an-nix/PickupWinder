/* MoveQueue.cpp — Host-side step move planner implementation. */

#include "MoveQueue.h"
#include <cmath>    /* std::round, std::sqrt */
#include <cstring>

/* ── hzToInterval ────────────────────────────────────────────────────────────*/
uint32_t MoveQueue::hzToInterval(uint32_t step_hz) {
    if (step_hz == 0u) return 0u;
    uint32_t iv = CLOCK_HZ / (2u * step_hz);
    return (iv < MIN_INTERVAL) ? MIN_INTERVAL : iv;
}

/* ── intervalToHz ────────────────────────────────────────────────────────────*/
uint32_t MoveQueue::intervalToHz(uint32_t interval) {
    if (interval == 0u) return 0u;
    return CLOCK_HZ / (2u * interval);
}

/* ── pushAccelSegment ────────────────────────────────────────────────────────
 * Computes an (interval, count, add) single-entry linear ramp from start_hz
 * to end_hz.  If the segment fits within CHUNK_EDGES it is one ring entry;
 * if not we split it into multiple constant-Hz approximation segments.
 *
 * Math:
 *   N_steps  = (end_hz² – start_hz²) / (2 × accel_hz_per_s)
 *   interval_0 = CLOCK / (2 × start_hz)
 *   interval_1 = CLOCK / (2 × end_hz)
 *   add_per_edge = (interval_1 – interval_0) / (2 × N_steps)
 */
uint32_t MoveQueue::pushAccelSegment(IpcChannel &ch, uint8_t axis,
                                     uint32_t start_hz, uint32_t end_hz,
                                     bool direction,
                                     uint32_t accel_hz_per_s) {
    if (start_hz == 0u || end_hz == 0u || accel_hz_per_s == 0u) return 0u;
    if (start_hz == end_hz) return 0u;

    /* N steps for the ramp (kinematic) */
    double v0 = (double)start_hz;
    double v1 = (double)end_hz;
    double a  = (double)accel_hz_per_s;
    double n_steps = (v1 * v1 - v0 * v0) / (2.0 * a);
    if (n_steps < 0.0) n_steps = -n_steps;  /* decel case */
    if (n_steps < 1.0) return 0u;

    uint32_t n = (uint32_t)n_steps;
    uint32_t n_edges = 2u * n;

    double   iv0  = (double)CLOCK_HZ / (2.0 * v0);
    double   iv1  = (double)CLOCK_HZ / (2.0 * v1);
    int32_t  add  = (int32_t)std::round((iv1 - iv0) / (double)n_edges);
    uint32_t iv_start = (uint32_t)(iv0 < (double)MIN_INTERVAL ? MIN_INTERVAL : iv0);

    /* Split into chunks if needed */
    uint32_t pushed = 0u;
    while (n_edges > 0u) {
        uint32_t chunk = (n_edges > CHUNK_EDGES) ? CHUNK_EDGES : n_edges;
        if (!ch.sendMove(axis, iv_start, chunk, add, direction ? 1u : 0u))
            break;
        /* Advance interval for next chunk */
        double advanced = (double)add * (double)chunk;
        iv0 += advanced;
        if (iv0 < MIN_INTERVAL) iv0 = MIN_INTERVAL;
        iv_start = (uint32_t)iv0;
        n_edges -= chunk;
        pushed  += chunk;
    }
    return pushed;
}

/* ── pushConstantEdges ───────────────────────────────────────────────────────*/
bool MoveQueue::pushConstantEdges(IpcChannel &ch, uint8_t axis,
                                  uint32_t step_hz, bool direction,
                                  uint32_t n_edges) {
    if (step_hz == 0u || n_edges == 0u) return false;
    uint32_t interval = hzToInterval(step_hz);
    while (n_edges > 0u) {
        uint32_t chunk = (n_edges > CHUNK_EDGES) ? CHUNK_EDGES : n_edges;
        if (!ch.sendMove(axis, interval, chunk, 0, direction ? 1u : 0u))
            return false;
        n_edges -= chunk;
    }
    return true;
}

/* ── pushConstantMs ──────────────────────────────────────────────────────────*/
bool MoveQueue::pushConstantMs(IpcChannel &ch, uint8_t axis,
                                uint32_t step_hz, bool direction,
                                uint32_t duration_ms) {
    if (step_hz == 0u || duration_ms == 0u) return false;
    /* edges = 2 × steps = 2 × Hz × duration_s */
    uint64_t n_edges = 2ull * (uint64_t)step_hz * (uint64_t)duration_ms / 1000ull;
    if (n_edges == 0u) return true;   /* nothing to push */
    return pushConstantEdges(ch, axis, step_hz, direction, (uint32_t)n_edges);
}

/* ── pushDecelToStop ─────────────────────────────────────────────────────────*/
bool MoveQueue::pushDecelToStop(IpcChannel &ch, uint8_t axis,
                                uint32_t start_hz, bool direction,
                                uint32_t accel_hz_per_s) {
    if (start_hz == 0u) return true;
    /* Decel to 1 Hz (effectively stopped; we can't use 0 for ramp math) */
    uint32_t pushed = pushAccelSegment(ch, axis, start_hz, 1u,
                                       direction, accel_hz_per_s);
    return pushed > 0u;
}
