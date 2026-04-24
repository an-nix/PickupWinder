/**
 * @file stepper_queue.cpp
 * @brief Per-motor helper around StepperDriver for multi-axis execution.
 */

#include "stepper_queue.h"

#include <esp_log.h>
#include <esp_check.h>
#include <freertos/task.h>

static const char* TAG = "stepper_queue";

static_assert(STEP_STREAM_START_FILL >= 2 * PART_SIZE,
              "STEP_STREAM_START_FILL must be >= 2 * PART_SIZE to prevent "
              "immediate ISR underrun on second encoder callback");

StepperQueue::StepperQueue(StepperDriver& driver, uint8_t motor_id)
    : driver_(driver)
    , motor_id_(motor_id)
{}

esp_err_t StepperQueue::init()
{
    ESP_LOGI(TAG, "motor%u: StepperQueue ready (legacy per-axis executor disabled)",
             motor_id_);
    return ESP_OK;
}

esp_err_t StepperQueue::enqueueMotionBlock(const motion_block_t& block,
                                           uint32_t timeout_ms)
{
    (void)block;
    (void)timeout_ms;
    ESP_LOGW(TAG,
             "motor%u: legacy per-axis motion is deprecated; use MULTI_AXIS_SEGMENT_BLOCK",
             motor_id_);
    return ESP_ERR_NOT_SUPPORTED;
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

uint32_t StepperQueue::available() const
{
    return static_cast<uint32_t>(STEPPER_QUEUE_DEPTH);
}

esp_err_t StepperQueue::executeConstantRateBlock(bool direction,
                                                  uint16_t step_count,
                                                  uint32_t duration_us)
{
    if (step_count == 0) {
        return ESP_OK;
    }

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
        ESP_LOGW(TAG, "motor%u: kickStart failed: %s",
                 motor_id_, esp_err_to_name(err));
    }
    return err;
}

void StepperQueue::gracefulStop()
{
    driver_.gracefulStop();
}

esp_err_t StepperQueue::maybeStartDriver(StepperDriver& driver, bool force_start)
{
    const uint32_t buffered_steps = STEP_RING_SIZE - driver.ringFreeSlots();
    const bool ring_has_data = buffered_steps > 0;
    if (!ring_has_data) {
        return ESP_OK;
    }

    const bool is_restart = !driver.isStreaming()
                            && buffered_steps > 0
                            && driver.getUnderrunCount() > 0;
    const uint32_t force_start_threshold = is_restart
        ? STEP_STREAM_RESTART_FILL
        : STEP_STREAM_START_FILL;
    const bool should_start = (force_start && buffered_steps >= force_start_threshold)
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
    if (!driver.isStreaming() && driver.ringFreeSlots() == 0) {
        esp_err_t err = maybeStartDriver(driver, true);
        if (err != ESP_OK) {
            return err;
        }
    }

    return driver.pushBlock(block, xTaskGetCurrentTaskHandle());
}
