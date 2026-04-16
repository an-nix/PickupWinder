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

private:
    StepperDriver& driver_;
    uint8_t        motor_id_;

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
