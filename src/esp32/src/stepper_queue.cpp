/**
 * @file stepper_queue.cpp
 * @brief Per-motor FreeRTOS queue + executor task implementation.
 */

#include "stepper_queue.h"

#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>

static const char* TAG = "stepper_queue";

// Compile-time invariant: the auto-start fill threshold must be large enough
// that the second encode_steps ping-pong callback never immediately underruns.
static_assert(STEP_STREAM_START_FILL >= 2 * PART_SIZE,
              "STEP_STREAM_START_FILL must be >= 2 * PART_SIZE to prevent "
              "immediate ISR underrun on second encoder callback");

// Task parameters
static constexpr uint32_t EXECUTOR_STACK_WORDS = 4096;
static constexpr UBaseType_t EXECUTOR_PRIORITY  = 24;
static constexpr BaseType_t  EXECUTOR_CORE      = 1; // Pro CPU (real-time)

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

StepperQueue::StepperQueue(StepperDriver& driver, uint8_t motor_id)
    : driver_(driver)
    , motor_id_(motor_id)
{}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::init()
{
    // Each queue item is one `motion_block_t`, which can carry either a legacy
    // `step_block_t` or a compressed `segment_block_t`.
    queue_ = xQueueCreate(STEPPER_QUEUE_DEPTH, sizeof(motion_block_t));
    ESP_RETURN_ON_FALSE(queue_ != nullptr, ESP_ERR_NO_MEM, TAG,
                        "motor%u: failed to create step queue", motor_id_);

    // ── Executor task ───────────────────────────────────────────────────────
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "stepper_%u", motor_id_);

    BaseType_t rc = xTaskCreatePinnedToCore(
        &StepperQueue::executorTask,
        task_name,
        EXECUTOR_STACK_WORDS,
        this,
        EXECUTOR_PRIORITY,
        &task_,
        EXECUTOR_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "motor%u: failed to create executor task", motor_id_);

    ESP_LOGI(TAG, "motor%u: queue depth=%d  task priority=%d  core=%d",
             motor_id_, STEPPER_QUEUE_DEPTH,
             (int)EXECUTOR_PRIORITY, (int)EXECUTOR_CORE);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// enqueueMotionBlock() / enqueueStepBlock() / enqueueSegmentBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::enqueueMotionBlock(const motion_block_t& block,
                                           uint32_t timeout_ms)
{
    const TickType_t ticks = (timeout_ms == portMAX_DELAY)
                             ? portMAX_DELAY
                             : pdMS_TO_TICKS(timeout_ms);

    if (xQueueSend(queue_, &block, ticks) != pdTRUE) {
        ESP_LOGW(TAG, "motor%u: queue full — motion block dropped",
                 motor_id_);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t StepperQueue::enqueueStepBlock(const step_block_t& block,
                                         uint32_t timeout_ms)
{
    motion_block_t motion {};
    motion.kind = MOTION_BLOCK_KIND_STEP;
    motion.payload.step = block;
    return enqueueMotionBlock(motion, timeout_ms);
}

esp_err_t StepperQueue::enqueueSegmentBlock(const segment_block_t& block,
                                            uint32_t timeout_ms)
{
    motion_block_t motion {};
    motion.kind = MOTION_BLOCK_KIND_SEGMENT;
    motion.payload.segment = block;
    return enqueueMotionBlock(motion, timeout_ms);
}

// ---------------------------------------------------------------------------
// available()
// ---------------------------------------------------------------------------

uint32_t StepperQueue::available() const
{
    return static_cast<uint32_t>(uxQueueSpacesAvailable(queue_));
}

// ---------------------------------------------------------------------------
// executeConstantRateBlock() / kickStart()
// ---------------------------------------------------------------------------
//
// executeConstantRateBlock() deliberately does NOT call maybeStartDriver().
// The start decision belongs to the caller: multiAxisExecutorTask drain loop
// in comm_interface.cpp calls kickStart() ONCE per block batch, after ALL
// available blocks have been written to the ring.  This guarantees the ring
// is pre-filled with multiple segments of look-ahead before RMT starts,
// preventing the per-segment underruns that occur at low speed when each
// segment contributes only 2–5 steps.

esp_err_t StepperQueue::executeConstantRateBlock(bool direction,
                                                  uint16_t step_count,
                                                  uint32_t duration_us)
{
    if (step_count == 0) {
        return ESP_OK;
    }

    // Compute uniform interval: RMT clock is 80 MHz → 80 ticks/µs.
    uint32_t interval_ticks = (duration_us * RMT_TICKS_PER_US) / step_count;
    if (interval_ticks < RMT_STEP_MIN_TICKS) {
        interval_ticks = RMT_STEP_MIN_TICKS;
    }
    if (interval_ticks > RMT_STEP_MAX_TICKS) {
        interval_ticks = RMT_STEP_MAX_TICKS;
    }

    uint32_t remaining = step_count;
    while (remaining > 0) {
        step_block_t expanded {};
        expanded.count = (remaining > STEP_BLOCK_SIZE) ? STEP_BLOCK_SIZE : remaining;
        for (uint32_t i = 0; i < expanded.count; ++i) {
            expanded.steps[i].interval_ticks = interval_ticks;
            expanded.steps[i].direction       = direction;
        }
        esp_err_t err = pushExpandedBlock(driver_, expanded);
        if (err != ESP_OK) {
            return err;
        }
        remaining -= expanded.count;
    }
    return ESP_OK;
}

esp_err_t StepperQueue::kickStart()
{
    esp_err_t err = maybeStartDriver(driver_, true);
    if (err != ESP_OK) {
        // B6: log explicite — diagnostique les échecs RMT silencieux
        ESP_LOGW(TAG, "motor%u: kickStart failed: %s",
                 motor_id_, esp_err_to_name(err));
    }
    return err;
}

void StepperQueue::gracefulStop()
{
    driver_.gracefulStop();
}

// ---------------------------------------------------------------------------
// maybeStartDriver() / pushExpandedBlock() / executeSegmentBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::maybeStartDriver(StepperDriver& driver, bool force_start)
{
    const uint32_t buffered_steps = STEP_RING_SIZE - driver.ringFreeSlots();
    const bool ring_has_data = buffered_steps > 0;
    if (!ring_has_data) {
        return ESP_OK;
    }

    // Guard force_start: never begin streaming with fewer than PART_SIZE steps.
    // The RMT ping-pong encoder's first callback requests PART_SIZE symbols; if
    // fewer steps are available it immediately underruns and halts the motor.
    // At slow speeds (2-9 steps/segment during acceleration) this was causing a
    // stutter on every segment.  STEP_STREAM_START_FILL and the ring-full case
    // are unaffected — those scenarios already imply >= PART_SIZE steps buffered.
    const bool is_restart = !driver.isStreaming()
                            && buffered_steps > 0
                            && driver.getUnderrunCount() > 0;
    const bool should_start = (force_start && buffered_steps >= PART_SIZE)
        || (buffered_steps >= STEP_STREAM_START_FILL)
        || (driver.ringFreeSlots() == 0)
        || (is_restart && buffered_steps >= STEP_STREAM_RESTART_FILL);
    if (!should_start) {
        return ESP_OK;
    }

    if (driver.isStreaming() && !driver.isStopped()) {
        return ESP_OK;
    }

    if (driver.isStreaming()) {
        driver.stopStream();
    }
    return driver.startStream();
}

esp_err_t StepperQueue::pushExpandedBlock(StepperDriver& driver, const step_block_t& block)
{
    // If RMT is not running AND ring is completely full, we must start
    // streaming to make room.  This is a safety valve only — normally the
    // executor calls kickStart() after draining a batch.
    if (!driver.isStreaming() && driver.ringFreeSlots() == 0) {
        esp_err_t err = maybeStartDriver(driver, true);
        if (err != ESP_OK) {
            return err;
        }
    }

    // Pass the current task handle so ISR ring-space notifications wake
    // whichever task is currently blocked on this ring (multiAxisExecutorTask
    // or per-axis executorTask).
    return driver.pushBlock(block, xTaskGetCurrentTaskHandle());

    // NOTE: maybeStartDriver() is NOT called here.
    // The executor task calls kickStart() once after draining all available
    // segments into the ring.  With coast-mode the RMT never stops between
    // segments, so no restart is needed during normal streaming.
}

esp_err_t StepperQueue::executeSegmentBlock(StepperDriver& driver, const segment_block_t& block)
{
    for (uint32_t seg_index = 0; seg_index < block.count; ++seg_index) {
        const motion_segment_t& seg = block.segments[seg_index];
        if (seg.step_count == 0) {
            continue;
        }

        uint32_t remaining = seg.step_count;
        int32_t current_ticks = seg.start_ticks;
        while (remaining > 0) {
            step_block_t expanded {};
            expanded.count = (remaining > STEP_BLOCK_SIZE) ? STEP_BLOCK_SIZE : remaining;
            for (uint32_t i = 0; i < expanded.count; ++i) {
                uint32_t clamped_ticks = static_cast<uint32_t>(current_ticks);
                if (clamped_ticks < RMT_STEP_MIN_TICKS) {
                    clamped_ticks = RMT_STEP_MIN_TICKS;
                }
                if (clamped_ticks > RMT_STEP_MAX_TICKS) {
                    clamped_ticks = RMT_STEP_MAX_TICKS;
                }
                expanded.steps[i].interval_ticks = clamped_ticks;
                expanded.steps[i].direction = (seg.direction != 0);
                current_ticks += seg.add_ticks;
            }

            esp_err_t err = pushExpandedBlock(driver, expanded);
            if (err != ESP_OK) {
                return err;
            }
            remaining -= expanded.count;
        }
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// executorTask()  — Core 1, priority 24
// ---------------------------------------------------------------------------
void StepperQueue::executorTask(void* arg)
{
    StepperQueue* self = static_cast<StepperQueue*>(arg);
    StepperDriver& driver = self->driver_;
    motion_block_t block;

    ESP_LOGI(TAG, "motor%u: executor task started", self->motor_id_);

    constexpr int WORK_BUDGET = 8;

    for (;;) {
        if (xQueueReceive(self->queue_, &block, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int work_done = 0;

        do {
            const int64_t loop_start_us = esp_timer_get_time();

            if (self->isMultiExecActive()) {
                ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1));
                continue;
            }

            esp_err_t err = ESP_OK;

            if (block.kind == MOTION_BLOCK_KIND_SEGMENT) {
                err = executeSegmentBlock(driver, block.payload.segment);
            } else {
                err = pushExpandedBlock(driver, block.payload.step);
            }

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "motor%u: motion execute error: %s",
                         self->motor_id_, esp_err_to_name(err));
            }

            work_done++;

            if (work_done >= WORK_BUDGET) {
                work_done = 0;
                const uint32_t ring_free = driver.ringFreeSlots();
                if (ring_free > (STEP_RING_SIZE / 2U)) {
                    taskYIELD();
                } else if ((esp_timer_get_time() - loop_start_us) > 5000) {
                    taskYIELD();
                }
            }

        } while (xQueueReceive(self->queue_, &block, 0) == pdTRUE);

        esp_err_t err = maybeStartDriver(driver, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "motor%u: startStream error: %s",
                     self->motor_id_, esp_err_to_name(err));
        }
    }
}