/**
 * @file motion_planner.cpp
 * @brief GRBL/Klipper-inspired motion planning layer — implementation.
 *
 * The planner task sits between SPI ingestion and the execution layer:
 *
 *   s_multi_axis_queue ──► plannerTask() ──► segment_queue_ ──► executor
 *
 * Responsibilities:
 *   • Decompose multi_axis_block_t (bulk) into individual planned_segment_t
 *   • Assign Klipper-style monotonic timestamps (scheduled_time_us)
 *   • Handle flush requests: drain both input and output queues
 *   • Provide bounded backpressure to the SPI ingestion layer
 *
 * Non-responsibilities (executor owns these):
 *   • No hardware access (no RMT, no GPIO, no ring buffer)
 *   • No endstop checking (real-time, must be in executor)
 *   • No StepperDriver interaction
 *
 * This separation means the planner is fully preemptible and testable
 * without hardware dependencies.
 */

#include "motion_planner.h"

#include <string.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>

static const char* TAG = "planner";

// ---------------------------------------------------------------------------
// Task parameters
// ---------------------------------------------------------------------------

static constexpr uint32_t    PLANNER_STACK = 4096;
static constexpr UBaseType_t PLANNER_PRIO  = 8;   // Below SPI (10), above idle
static constexpr BaseType_t  PLANNER_CORE  = 0;   // Same core as SPI task

/** Max blocks drained from cmd_queue_ during a single flush operation. */
static constexpr uint32_t MAX_FLUSH_DRAIN = 16;

/** Backpressure timeout: how long the planner waits for segment_queue_ space
 *  before dropping a segment.  10 ms is ~2.5 segments at 4 ms/segment. */
static constexpr TickType_t BACKPRESSURE_TIMEOUT = pdMS_TO_TICKS(10);

/** Planner poll interval when no command blocks are available. */
static constexpr TickType_t CMD_POLL_TIMEOUT = pdMS_TO_TICKS(5);

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MotionPlanner::MotionPlanner() {}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t MotionPlanner::init(QueueHandle_t cmd_queue, QueueHandle_t flush_queue)
{
    cmd_queue_   = cmd_queue;
    flush_queue_ = flush_queue;

    segment_queue_ = xQueueCreate(SEGMENT_QUEUE_DEPTH, sizeof(planned_segment_t));
    ESP_RETURN_ON_FALSE(segment_queue_ != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create segment queue");

    BaseType_t rc = xTaskCreatePinnedToCore(
        &MotionPlanner::plannerTask,
        "planner",
        PLANNER_STACK,
        this,
        PLANNER_PRIO,
        nullptr,
        PLANNER_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create planner task");

    ESP_LOGI(TAG, "planner ready: seg_queue_depth=%lu  core=%d  pri=%d",
             (unsigned long)SEGMENT_QUEUE_DEPTH,
             (int)PLANNER_CORE, (int)PLANNER_PRIO);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// segmentQueueFree()
// ---------------------------------------------------------------------------

uint32_t MotionPlanner::segmentQueueFree() const
{
    if (segment_queue_ == nullptr) return 0;
    return static_cast<uint32_t>(uxQueueSpacesAvailable(segment_queue_));
}

// ---------------------------------------------------------------------------
// planBlock()
// ---------------------------------------------------------------------------

void MotionPlanner::planBlock(const multi_axis_block_t& block)
{
    // Legacy path kept for compatibility: copy into pending buffer so the
    // real work happens incrementally inside plannerTask.  This avoids
    // burst CPU usage and guarantees non-blocking behavior.
    if (!has_pending_block_) {
        pending_block_ = block;
        pending_segment_idx_ = 0;
        has_pending_block_ = true;
        // Ensure timeline is clamped to now if it drifted into the past.
        const int64_t now_us = esp_timer_get_time();
        if (timeline_us_ < now_us) timeline_us_ = now_us;
    } else {
        // Already processing a block — drop this incoming block (should be
        // rare because the SPI layer back-pressures). Count as dropped.
        ++segments_dropped_;
        ESP_LOGW(TAG, "incoming block dropped: planner busy (dropped total=%lu)",
                 (unsigned long)segments_dropped_);
    }
}

// ---------------------------------------------------------------------------
// handleFlush()
// ---------------------------------------------------------------------------

void MotionPlanner::handleFlush(const flush_request_t& req)
{
    // Non-blocking flush: mark pending state and drop any currently stored
    // pending_block_.  We will attempt to push a flush sentinel to the
    // output queue without blocking; if that fails we remember the flush
    // and retry on the next planner loop iteration.
    has_pending_block_ = false; // drop current pending block
    timeline_us_ = esp_timer_get_time();
    planned_segment_t flush_seg {};
    flush_seg.is_flush = true;
    flush_seg.flush_sequence = req.flush_sequence;
    if (xQueueSend(segment_queue_, &flush_seg, 0) == pdTRUE) {
        flush_pending_ = false;
        ESP_LOGI(TAG, "flush sentinel posted seq=%u", req.flush_sequence);
    } else {
        // Queue full — remember to retry later.
        flush_pending_ = true;
        pending_flush_sequence_ = req.flush_sequence;
        ESP_LOGW(TAG, "flush sentinel queued later seq=%u", req.flush_sequence);
    }
}

// ---------------------------------------------------------------------------
// plannerTask()
// ---------------------------------------------------------------------------

void MotionPlanner::plannerTask(void* arg)
{
    auto* self = static_cast<MotionPlanner*>(arg);
    multi_axis_block_t block;
    flush_request_t flush_req;

    ESP_LOGI(TAG, "planner task running on core %d", xPortGetCoreID());

    for (;;) {
        const int64_t loop_start = esp_timer_get_time();

        // 1) Handle any flush requests immediately (non-blocking).
        if (xQueueReceive(self->flush_queue_, &flush_req, 0) == pdTRUE) {
            self->handleFlush(flush_req);
        }

        // 2) If a previous flush sentinel failed to post, retry non-blocking.
        if (self->flush_pending_) {
            planned_segment_t flush_seg {};
            flush_seg.is_flush = true;
            flush_seg.flush_sequence = self->pending_flush_sequence_;
            if (xQueueSend(self->segment_queue_, &flush_seg, 0) == pdTRUE) {
                self->flush_pending_ = false;
                ESP_LOGI(TAG, "flush sentinel posted retry seq=%u",
                         (unsigned)self->pending_flush_sequence_);
            }
        }

        // 3) Block on cmd_queue_ until data arrives or CMD_POLL_TIMEOUT (5 ms).
        //    Previously used timeout=0 (non-blocking) + vTaskDelay(1) which
        //    caused a 10 ms sleep at 100 Hz tick rate, starving the motor.
        if (!self->has_pending_block_) {
            if (xQueueReceive(self->cmd_queue_, &block, CMD_POLL_TIMEOUT) == pdTRUE) {
                // Store for incremental expansion.
                self->pending_block_ = block;
                self->pending_segment_idx_ = 0;
                self->has_pending_block_ = true;
                // Clamp timeline if idle.
                const int64_t now_us = esp_timer_get_time();
                if (self->timeline_us_ < now_us) self->timeline_us_ = now_us;
            }
        }

        // 4) Process up to PLANNER_MAX_SEGMENTS_PER_ITER segments from the
        //    pending_block_ within the time budget. All queue sends are
        //    non-blocking (timeout=0); on failure we drop the segment and
        //    advance so the planner never stalls.
        uint32_t processed = 0;
        while (self->has_pending_block_ &&
               processed < PLANNER_MAX_SEGMENTS_PER_ITER &&
               (esp_timer_get_time() - loop_start) < PLANNER_TIME_BUDGET_US) {

            const uint16_t idx = self->pending_segment_idx_;
            if (idx >= self->pending_block_.segment_count) {
                // Finished this block.
                self->has_pending_block_ = false;
                break;
            }

            const multi_axis_segment_t& src = self->pending_block_.segments[idx];

            planned_segment_t seg {};
            seg.motion_sequence = src.motion_sequence;
            seg.duration_us     = src.duration_us;
            seg.scheduled_time_us = self->timeline_us_;
            seg.axis_count      = self->pending_block_.axis_count;
            seg.is_flush        = false;

            const uint8_t n = (self->pending_block_.axis_count < MULTI_AXIS_MAX_AXES)
                              ? self->pending_block_.axis_count : MULTI_AXIS_MAX_AXES;
            for (uint8_t a = 0; a < n; ++a) {
                seg.axis_ids[a]        = self->pending_block_.axis_ids[a];
                seg.axes[a].step_count = src.step_counts[a];
                seg.axes[a].direction  = ((src.direction_mask >> a) & 1u) != 0;
            }

            // Advance timeline and enqueue atomically: only advance if the
            // segment was successfully enqueued so the timeline stays in sync.
            // On full queue, break out of the inner loop — the executor will
            // drain a slot, and we will retry this segment on the next
            // planner iteration (natural backpressure, no data loss).
            if (xQueueSend(self->segment_queue_, &seg, 0) == pdTRUE) {
                self->timeline_us_ += static_cast<int64_t>(src.duration_us);
                ++self->segments_planned_;
                ++self->pending_segment_idx_;
                ++processed;
            } else {
                // Queue full — yield and retry this segment next iteration.
                break;
            }
        }

        // 5) Yield so higher-priority tasks (SPI task pri=24, executor pri=20)
        //    get CPU immediately after any batch of work.
        //    If nothing was processed AND xQueueReceive returned immediately
        //    (no pending block and empty cmd_queue), do a minimal 1-tick sleep
        //    (= 1ms at CONFIG_FREERTOS_HZ=1000) to avoid a busy-loop that
        //    creates DMA timing jitter on Core 0 and causes 0x0150 bad magic.
        if (processed > 0) {
            taskYIELD();
        } else if (!self->has_pending_block_) {
            // No data arriving — sleep 1 tick rather than spin.
            // At 1000Hz this is 1ms; short enough to not starve the motor.
            vTaskDelay(1);
        }
    }
}
