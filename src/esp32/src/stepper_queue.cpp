/**
 * @file stepper_queue.cpp
 * @brief Per-motor helper around StepperDriver for multi-axis execution.
 */

#include "stepper_queue.h"

#include <esp_log.h>
#include <esp_check.h>
#include <freertos/task.h>

static const char* TAG = "stepper_queue";

// Compile-time invariant: the auto-start fill threshold must be large enough
// that the second encode_steps ping-pong callback never immediately underruns.
static_assert(STEP_STREAM_START_FILL >= 2 * PART_SIZE,
              "STEP_STREAM_START_FILL must be >= 2 * PART_SIZE to prevent "
              "immediate ISR underrun on second encoder callback");

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
    ESP_LOGI(TAG, "motor%u: StepperQueue ready (legacy per-axis executor disabled)",
             motor_id_);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// enqueueMotionBlock() / enqueueStepBlock() / enqueueSegmentBlock()
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// available()
// ---------------------------------------------------------------------------

uint32_t StepperQueue::available() const
{
    return static_cast<uint32_t>(STEPPER_QUEUE_DEPTH);
}

// ---------------------------------------------------------------------------
// executeConstantRateBlock() / kickStart()
// ---------------------------------------------------------------------------
//
// executeConstantRateBlock() deliberately does NOT call maybeStartDriver().
// The start decision belongs to the caller: MultiAxisExecutor drain loop
// calls kickStart() ONCE per block batch, after ALL
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
// maybeStartDriver() / pushExpandedBlock()
// ---------------------------------------------------------------------------

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
    // If RMT is not running AND ring is completely full, we must start
    // streaming to make room.  This is a safety valve only — normally the
    // executor calls kickStart() after draining a batch.
    if (!driver.isStreaming() && driver.ringFreeSlots() == 0) {
        esp_err_t err = maybeStartDriver(driver, true);
        if (err != ESP_OK) {
            return err;
        }
    }

    // Pass the current task handle so ISR ring-space notifications wake the
    // global multi-axis executor when it is blocked on this ring.
    return driver.pushBlock(block, xTaskGetCurrentTaskHandle());

    // NOTE: maybeStartDriver() is NOT called here.
    // The multi-axis executor calls kickStart() once after draining all available
    // segments into the ring.  With coast-mode the RMT never stops between
    // segments, so no restart is needed during normal streaming.
}