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
static constexpr UBaseType_t PLANNER_PRIO  = 23;  // Just below SPI task (24) on Core 0
static constexpr BaseType_t  PLANNER_CORE  = 0;   // Same core as SPI task

/** Current depth of the SPI→planner multi-axis command queue. */
static constexpr uint32_t CMD_QUEUE_DEPTH = 64;

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

void MotionPlanner::resetStats()
{
    segments_planned_ = 0;
    segments_dropped_ = 0;
    last_planned_motion_seq_ = 0xFFFFu;
    last_flush_processed_seq_  = 0xFFFFu;
    last_flush_sequence_valid_ = false;
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
    // Ignore stale flushes: a flush whose sequence is not newer than the last
    // processed one must not purge segments that were already enqueued for
    // a more-recent motion (e.g. backoff after endstop).
    if (req.source == FLUSH_SOURCE_HOST
        && last_flush_sequence_valid_ &&
        sequence_is_stale_or_equal_u16(req.flush_sequence, last_flush_processed_seq_)) {
        ESP_LOGW(TAG, "ignoring stale host flush seq=%u (last=%u)",
                 static_cast<unsigned>(req.flush_sequence),
                 static_cast<unsigned>(last_flush_processed_seq_));
        return;
    }
    if (req.source == FLUSH_SOURCE_HOST) {
        last_flush_processed_seq_   = req.flush_sequence;
        last_flush_sequence_valid_  = true;
    }

    // Non-blocking flush: mark pending state and drop any currently stored
    // pending_block_.  We will attempt to push a flush sentinel to the
    // output queue without blocking; if that fails we remember the flush
    // and retry on the next planner loop iteration.
    has_pending_block_ = false; // drop current pending block
    pending_segment_idx_ = 0;
    timeline_us_ = esp_timer_get_time();
    // flush_sequence is a transport/control sequence, not a motion_sequence.
    // Reset the motion watermark so the next planned segment batch is accepted
    // regardless of its motion_sequence value.
    last_planned_motion_seq_ = 0xFFFFu;

    multi_axis_block_t dropped_block {};
    planned_segment_t dropped_seg {};
    uint32_t dropped_cmd = 0;
    uint32_t dropped_planned = 0;

    while (dropped_cmd < CMD_QUEUE_DEPTH &&
           xQueueReceive(cmd_queue_, &dropped_block, 0) == pdTRUE) {
        ++dropped_cmd;
    }
    while (dropped_planned < SEGMENT_QUEUE_DEPTH &&
           xQueueReceive(segment_queue_, &dropped_seg, 0) == pdTRUE) {
        ++dropped_planned;
    }

    planned_segment_t flush_seg {};
    flush_seg.is_flush = true;
    flush_seg.flush_sequence = req.flush_sequence;
    if (xQueueSend(segment_queue_, &flush_seg, 0) == pdTRUE) {
        flush_pending_ = false;
        ESP_LOGI(TAG, "flush sentinel posted seq=%u source=%u dropped_cmd=%lu dropped_seg=%lu",
                 req.flush_sequence,
                 static_cast<unsigned>(req.source),
                 static_cast<unsigned long>(dropped_cmd),
                 static_cast<unsigned long>(dropped_planned));
    } else {
        // Queue full — remember to retry later.
        flush_pending_ = true;
        pending_flush_sequence_ = req.flush_sequence;
        ESP_LOGW(TAG, "flush sentinel queued later seq=%u source=%u",
                 req.flush_sequence,
                 static_cast<unsigned>(req.source));
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

        // 1) Drain all pending flush requests.
        //    Keep the newest host flush and the newest internal flush in their
        //    own sequence domains; never compare host and internal sequences
        //    against each other. If an internal flush is present, it takes
        //    precedence for this iteration because it reflects executor-side
        //    recovery at the current motion boundary.
        {
            bool has_host_flush = false;
            bool has_internal_flush = false;
            flush_request_t latest_host_flush{};
            flush_request_t latest_internal_flush{};
            while (xQueueReceive(self->flush_queue_, &flush_req, 0) == pdTRUE) {
                if (flush_req.source == FLUSH_SOURCE_INTERNAL) {
                    if (!has_internal_flush ||
                        sequence_is_newer_u16(flush_req.flush_sequence,
                                             latest_internal_flush.flush_sequence)) {
                        latest_internal_flush = flush_req;
                        has_internal_flush = true;
                    }
                } else {
                    if (!has_host_flush ||
                        sequence_is_newer_u16(flush_req.flush_sequence,
                                             latest_host_flush.flush_sequence)) {
                        latest_host_flush = flush_req;
                        has_host_flush = true;
                    }
                }
            }
            if (has_internal_flush) {
                self->handleFlush(latest_internal_flush);
            } else if (has_host_flush) {
                self->handleFlush(latest_host_flush);
            }
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

            if (sequence_is_stale_or_equal_u16(
                    src.motion_sequence,
                    self->last_planned_motion_seq_)) {
                ESP_LOGW(TAG, "drop stale/out-of-order seg seq=%u last=%u",
                         static_cast<unsigned>(src.motion_sequence),
                         static_cast<unsigned>(self->last_planned_motion_seq_));
                ++self->segments_dropped_;
                ++self->pending_segment_idx_;
                ++processed;
                continue;
            }

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
                self->last_planned_motion_seq_ = src.motion_sequence;
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
        //    (= 1ms at CONFIG_FREERTOS_HZ=1000) to avoid a busy-loop that can
        //    starve Core 0 and aggravate SPI timing sensitivity.
        if (processed > 0) {
            taskYIELD();
        } else if (!self->has_pending_block_) {
            // No data arriving — sleep 1 tick rather than spin.
            // At 1000Hz this is 1ms; short enough to not starve the motor.
            vTaskDelay(1);
        }
    }
}
