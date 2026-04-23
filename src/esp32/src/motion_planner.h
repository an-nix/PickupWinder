/**
 * @file motion_planner.h
 * @brief GRBL/Klipper-inspired motion planning layer.
 *
 * ── Architecture role ──────────────────────────────────────────────────────
 *
 *   SPI ingestion (Core 0)              Planner (Core 0)           Executor (Core 1)
 *   ────────────────────                ───────────────            ─────────────────
 *   spiTask → handleFrame()   ──►   s_multi_axis_queue   ──►   plannerTask()
 *                                        (existing)              │
 *                                                                ▼
 *                                                          segment_queue_
 *                                                           (NEW bounded)
 *                                                                │
 *                                                                ▼
 *                                                        executorTask (Core 1)
 *                                                        state machine
 *                                                                │
 *                                                                ▼
 *                                                          RMT ring buffer
 *
 * The planner consumes multi_axis_block_t (bulk blocks from SPI) and
 * decomposes them into individual planned_segment_t entries with Klipper-style
 * monotonic timestamps.  The executor consumes these one at a time through a
 * bounded state machine.
 *
 * ── Backpressure ───────────────────────────────────────────────────────────
 *
 *   segment_queue_ is bounded to SEGMENT_QUEUE_DEPTH.  If the executor is
 *   slow, the planner blocks (with timeout) providing natural backpressure
 *   all the way back to the SPI ingestion queue.
 *
 * ── Flush path ─────────────────────────────────────────────────────────────
 *
 *   On flush, the planner drains both cmd_queue_ and segment_queue_, then
 *   pushes a flush sentinel (is_flush=true) so the executor can reset its
 *   state atomically.
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_err.h>
#include "step_types.h"

// ---------------------------------------------------------------------------
// Planned segment — immutable output of planner, input to executor
// ---------------------------------------------------------------------------

/**
 * @brief Per-axis motion within a planned segment.
 */
typedef struct {
    uint16_t step_count;
    bool     direction;
} planned_axis_motion_t;

/**
 * @brief One fully-planned segment ready for execution.
 *
 * Produced by the planner task, consumed by the executor task.
 * Immutable after enqueue — no synchronisation needed beyond the queue.
 */
typedef struct {
    uint16_t              motion_sequence;     ///< Host-assigned sequence ID
    uint16_t              duration_us;         ///< Wall-clock duration
    int64_t               scheduled_time_us;   ///< Monotonic execution timestamp (Klipper-style)
    uint8_t               axis_count;
    uint8_t               axis_ids[MULTI_AXIS_MAX_AXES];
    planned_axis_motion_t axes[MULTI_AXIS_MAX_AXES];
    bool                  is_flush;            ///< True = flush sentinel, not a real segment
    uint16_t              flush_sequence;      ///< Valid only when is_flush == true
} planned_segment_t;

// ---------------------------------------------------------------------------
// Executor state machine
// ---------------------------------------------------------------------------

/**
 * @brief Executor FSM states.
 *
 * The state machine ensures bounded CPU usage per iteration and eliminates
 * the nested drain loops that caused watchdog resets.
 *
 *   IDLE ──► FETCH ──► DRAIN ──► RUN ──► (back to IDLE)
 *              │                           ▲
 *              ▼                           │
 *            FLUSH ────────────────────────┘
 *              │
 *              ▼
 *           RECOVERY ──────────────────────┘
 */
enum class ExecState : uint8_t {
    IDLE,       ///< Waiting for segments (blocking queue receive)
    FETCH,      ///< Pulling segment(s) from planner queue (non-blocking batch)
    DRAIN,      ///< Writing steps to RMT ring buffer
    RUN,        ///< KickStart RMT, fire deferred notifications
    FLUSH,      ///< Processing flush sentinel — reset pipeline
    RECOVERY,   ///< Recovering from RMT underrun / error
};

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/** Segment queue: planner → executor.  ~512 ms lookahead at 4 ms/segment. */
static constexpr uint32_t SEGMENT_QUEUE_DEPTH = 128;

/** Maximum segments the executor fetches per FETCH iteration.
 *  Set to 1 to force frequent yields and allow planner to refill. */
static constexpr uint32_t EXEC_BATCH_LIMIT = 16;

/** Time budget per executor iteration in microseconds (watchdog safe). */
static constexpr int64_t  EXEC_TIME_BUDGET_US = 8000;

/** Planner tuning: time budget and per-iteration limit (watchdog-safe).
 *  At 1500 RPM the executor can drain the planned segment queue faster than
 *  the old Core-0 planner budget refills it, especially while the SPI task is
 *  servicing frequent status polls. Give the planner a wider per-iteration
 *  budget so it can drain queued multi-axis blocks into SEGMENT_QUEUE_DEPTH
 *  before the executor reaches the end of its lookahead.
 */
static constexpr int64_t  PLANNER_TIME_BUDGET_US = 8000; // µs per planner loop
static constexpr uint32_t PLANNER_MAX_SEGMENTS_PER_ITER = 128; // up to one full seg queue per iteration

// ---------------------------------------------------------------------------
// MotionPlanner class
// ---------------------------------------------------------------------------

class MotionPlanner {
public:
    MotionPlanner();

    /**
     * @brief Initialise the segment output queue and launch the planner task.
     *
     * @param cmd_queue   Existing s_multi_axis_queue (SPI → planner input).
     * @param flush_queue Existing s_flush_queue (SPI → planner input).
     * @return ESP_OK on success.
     */
    esp_err_t init(QueueHandle_t cmd_queue, QueueHandle_t flush_queue);

    /** @brief Output queue handle for the executor to consume. */
    QueueHandle_t segmentQueue() const { return segment_queue_; }

    /** @brief Number of free slots in the segment output queue. */
    uint32_t segmentQueueFree() const;

    // ── Statistics ──────────────────────────────────────────────────────────
    uint32_t segmentsPlanned() const { return segments_planned_; }
    uint32_t segmentsDropped() const { return segments_dropped_; }
    uint16_t lastPlannedMotionSequence() const { return last_planned_motion_seq_; }
    void resetStats();

private:
    QueueHandle_t cmd_queue_     {nullptr};  ///< Input: s_multi_axis_queue
    QueueHandle_t flush_queue_   {nullptr};  ///< Input: s_flush_queue
    QueueHandle_t segment_queue_ {nullptr};  ///< Output: planned_segment_t

    int64_t  timeline_us_       {0};         ///< Monotonic scheduling timeline
    uint32_t segments_planned_  {0};
    uint32_t segments_dropped_  {0};
    uint16_t last_planned_motion_seq_ {0xFFFFu};

    // ── Incremental planner state (to avoid burst processing) ───────────
    multi_axis_block_t pending_block_ {};   ///< Currently-being-expanded block
    uint16_t            pending_segment_idx_ {0};
    bool                has_pending_block_   {false};

    // Non-blocking flush sentinel retry state
    bool     flush_pending_ {false};
    uint16_t pending_flush_sequence_ {0};

    /**
     * @brief Expand one multi_axis_block_t into planned_segment_t entries.
     *
     * Each segment in the block becomes one planned_segment_t with a
     * Klipper-style monotonic timestamp derived from cumulative duration.
     * Enqueues to segment_queue_ with bounded backpressure wait.
     */
    void planBlock(const multi_axis_block_t& block);

    /**
     * @brief Handle a flush request: drain queues, push flush sentinel.
     */
    void handleFlush(const flush_request_t& req);

    /**
     * @brief Planner task body.
     *
     * Pinned to Core 0, priority 8 (below SPI task at 10, above idle).
     * Runs opportunistically alongside the SPI task on Core 0.
     */
    static void plannerTask(void* arg);
};
