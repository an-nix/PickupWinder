/**
 * @file stepper_queue.h
 * @brief Per-motor FreeRTOS queue + executor task for motion blocks.
 *
 * The host now sends compressed motion segments rather than only explicit
 * per-step blocks. `StepperQueue` remains the boundary between transport and
 * execution:
 *
 * - Core 0 / SPI task enqueues `motion_block_t`
 * - Core 1 / executor task expands segments into `step_block_t`
 * - `StepperDriver` streams the concrete steps via RMT
 */

#pragma once

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
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
     * @brief Create the FreeRTOS queue and launch the executor task.
     *
     * Must be called after StepperDriver::init().
     */
    esp_err_t init();

    /**
     * @brief Enqueue a step block for execution.
     *
     * Called by the communication layer (producer side).  Blocks for up to
     * @p timeout_ms milliseconds if the queue is full.
     *
     * @param block       Block of pre-timed step commands.
     * @param timeout_ms  Maximum wait time in ms (0 = non-blocking).
     * @return ESP_OK on success, ESP_ERR_TIMEOUT if the queue was full.
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

    QueueHandle_t  queue_  {nullptr};
    TaskHandle_t   task_   {nullptr};

    static esp_err_t maybeStartDriver(StepperDriver& driver, bool force_start);
    static esp_err_t pushExpandedBlock(StepperDriver& driver, const step_block_t& block);
    static esp_err_t executeSegmentBlock(StepperDriver& driver, const segment_block_t& block);

    /**
     * @brief Executor task body.
     *
    * Pinned to Core 1, priority 24. Dequeues `motion_block_t`, expands any
    * compressed segments into `step_block_t`, and keeps the software ring as
    * full as possible before starting / restarting the RMT stream.
     */
    static void executorTask(void* arg);
};
