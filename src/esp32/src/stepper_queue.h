/**
 * @file stepper_queue.h
 * @brief Per-motor helper around StepperDriver for multi-axis execution.
 *
 * Production motion now goes through the global multi-axis executor. `StepperQueue`
 * remains the per-axis boundary around `StepperDriver` for:
 *
 * - driver enable/disable/stop control
 * - constant-rate segment expansion into `step_block_t`
 * - coordinated RMT start for synchronised multi-axis motion
 *
 * Legacy per-axis `STEP_BLOCK` / `SEGMENT_BLOCK` execution is no longer active.
 */

#pragma once

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include "step_types.h"
#include "stepper_driver.h"

class StepperQueue {
public:
    /**
     * @brief Construct a StepperQueue bound to a StepperDriver.
     *
     * @param driver    Initialised StepperDriver instance for this motor.
     * @param motor_id  Logical motor index (0-based, for logging).
     */
    StepperQueue(StepperDriver& driver, uint8_t motor_id);

    /**
        * @brief Initialise the per-axis helper state.
     *
     * Must be called after StepperDriver::init().
     */
    esp_err_t init();

    /**
        * @brief Legacy compatibility entry point for deprecated per-axis motion.
     *
        * Production firmware no longer runs the historical per-axis executor task;
        * callers should use `MULTI_AXIS_SEGMENT_BLOCK` instead.
     */
    esp_err_t enqueueMotionBlock(const motion_block_t& block,
                                 uint32_t timeout_ms = portMAX_DELAY);

    esp_err_t enqueueStepBlock(const step_block_t& block,
                               uint32_t timeout_ms = portMAX_DELAY);

    esp_err_t enqueueSegmentBlock(const segment_block_t& block,
                                  uint32_t timeout_ms = portMAX_DELAY);

    /** @brief Legacy compatibility wrapper for old explicit-step producers. */
    esp_err_t enqueueBlock(const step_block_t& block,
                           uint32_t timeout_ms = portMAX_DELAY) {
        return enqueueStepBlock(block, timeout_ms);
    }

    /**
     * @brief Emit a constant-rate step burst for use by the multi-axis executor.
     *
     * Computes a uniform step interval from @p duration_us / @p step_count,
     * clamps it to the RMT hardware limits, then calls pushExpandedBlock()
     * to fill the RMT ring.
     *
     * This method is intended to be called from the global multi-axis executor
     * task (Core 1) when the per-axis queue is empty and not competing for the
     * driver.  It must NOT be called concurrently with the per-axis executor
     * task for the same motor.
     *
     * @param direction   true = forward, false = reverse.
     * @param step_count  Number of steps to emit.
     * @param duration_us Segment wall-clock duration in microseconds.
     * @return ESP_OK on success, error code on RMT/ring error.
     */
    esp_err_t executeConstantRateBlock(bool direction,
                                       uint16_t step_count,
                                       uint32_t duration_us);

    /**
     * @brief Force-start the RMT stream if ring has data and is not running.
     *
     * Call after distributing steps across all axes in a multi-axis segment
     * to ensure all drivers begin streaming simultaneously.
     */
    esp_err_t kickStart();

    /**
     * @brief Signal the motor to stop after the current ring contents drain.
     *
     * Does NOT flush the ring buffer (contrast with emergencyStop via driver).
     * The motor decelerates naturally to zero as pre-queued steps are consumed.
     */
    void gracefulStop();

    /**
     * @brief Number of free slots remaining in the block queue.
     *
     * Use for flow control: signal the host when this drops below
     * FLOW_CONTROL_THRESHOLD.
     */
    uint32_t available() const;

    /** @brief Return the motor id (0 or 1). */
    uint8_t motorId() const { return motor_id_; }

    /** @brief Access the bound driver (for status / enable / estop handling). */
    StepperDriver& driver() { return driver_; }

    /** @brief Const access to the bound driver. */
    const StepperDriver& driver() const { return driver_; }

    /** @brief Mark this axis as being actively driven by the multi-axis executor. */
    void setMultiExecActive(bool active) {
        multi_exec_active_.store(active, std::memory_order_release);
    }

    /** @brief True when the multi-axis executor currently owns this driver. */
    bool isMultiExecActive() const {
        return multi_exec_active_.load(std::memory_order_acquire);
    }

private:
    StepperDriver& driver_;
    uint8_t        motor_id_;
    std::atomic<bool> multi_exec_active_ {false};

    static esp_err_t maybeStartDriver(StepperDriver& driver, bool force_start);
    static esp_err_t pushExpandedBlock(StepperDriver& driver, const step_block_t& block);
};
