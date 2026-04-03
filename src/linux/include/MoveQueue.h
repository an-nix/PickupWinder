/* MoveQueue.h — Host-side Klipper-style step move planner.
 *
 * Converts velocity parameters (Hz) into (interval, count, add) triples
 * and pushes them into the PRU move ring via IpcChannel.
 *
 * The PRU executes moves using the 200 MHz IEP counter.
 *   interval = PRU_CLOCK_HZ / (2 × step_hz)   [ticks between pin-edge toggles]
 *   count    = 2 × n_physical_steps            [total toggle events]
 *   add      = Δinterval per edge              [linear ramp; 0 = constant speed]
 *
 * Ramp math (constant acceleration a over N steps from v0 to v1):
 *   N          = (v1² – v0²) / (2 × a)         [steps]
 *   interval_0 = PRU_CLOCK_HZ / (2 × v0)
 *   interval_1 = PRU_CLOCK_HZ / (2 × v1)
 *   add        = (interval_1 – interval_0) / (2N)  [per edge, float→int32_t]
 *
 * All float arithmetic is host-side (Linux Cortex-A8); PRU sees integers only.
 * No heap allocation in any method.
 */

#pragma once
#include <cstdint>
#include "IpcChannel.h"

class MoveQueue {
public:
    /* Mirror of PRU_CLOCK_HZ for host-side interval computation. */
    static constexpr uint32_t CLOCK_HZ      = 200000000u;
    static constexpr uint32_t MIN_INTERVAL  = 625u;      /* 160 kHz max */
    static constexpr uint32_t LOW_WATERMARK = 8u;        /* refill trigger */
    static constexpr uint32_t CHUNK_EDGES   = 20000u;    /* max edges per move entry */

    /* Convert step frequency → IEP interval (ticks between edges). */
    static uint32_t hzToInterval(uint32_t step_hz);

    /* Convert IEP interval → approximate step frequency (Hz). */
    static uint32_t intervalToHz(uint32_t interval);

    /* ────────────────────────────────────────────────────────────────────────
     * pushAccelSegment
     *   Linear acceleration from start_hz to end_hz.
     *   accel_hz_per_s: acceleration magnitude in Hz/s (e.g. 100000 = 100 kHz/s).
     *   Returns number of edges pushed (0 if ring full or no steps needed).
     */
    static uint32_t pushAccelSegment(IpcChannel &ch, uint8_t axis,
                                     uint32_t start_hz, uint32_t end_hz,
                                     bool direction,
                                     uint32_t accel_hz_per_s);

    /* ────────────────────────────────────────────────────────────────────────
     * pushConstantEdges
     *   Constant speed exactly n_edges toggle events.
     *   Returns false if ring full.
     */
    static bool pushConstantEdges(IpcChannel &ch, uint8_t axis,
                                  uint32_t step_hz, bool direction,
                                  uint32_t n_edges);

    /* ────────────────────────────────────────────────────────────────────────
     * pushConstantMs
     *   Constant speed for duration_ms milliseconds.
     *   Internally splits into CHUNK_EDGES-sized pieces to stay < ring capacity.
     *   Returns false on ring full.
     */
    static bool pushConstantMs(IpcChannel &ch, uint8_t axis,
                                uint32_t step_hz, bool direction,
                                uint32_t duration_ms);

    /* ────────────────────────────────────────────────────────────────────────
     * pushDecelToStop
     *   Linear deceleration from start_hz to 0.
     *   accel_hz_per_s: same magnitude as used for acceleration.
     *   Returns false on ring full or if already at stop.
     */
    static bool pushDecelToStop(IpcChannel &ch, uint8_t axis,
                                uint32_t start_hz, bool direction,
                                uint32_t accel_hz_per_s);

    /* ── Utilities ──────────────────────────────────────────────────────────*/

    /** Free slots available in the axis move ring. */
    static uint32_t freeSlots(const IpcChannel &ch, uint8_t axis) {
        return ch.moveQueueFreeSlots(axis);
    }

    /** True if ring is below LOW_WATERMARK and needs more moves. */
    static bool needsRefill(const IpcChannel &ch, uint8_t axis) {
        return ch.moveQueueFreeSlots(axis) >
               (axis == AXIS_SPINDLE ? IPC_SP_MOVE_SLOTS : IPC_LAT_MOVE_SLOTS) -
               LOW_WATERMARK;
    }
};
