/**
 * @file stepper_queue.h
 * @brief Per-motor FreeRTOS queue + executor task for step_block_t.
 *
 * StepperQueue decouples the communication layer (producer) from the RMT
 * driver (consumer).  The executor task runs on Core 1 at priority 24 to
 * guarantee real-time execution of motor commands without interference from
 * Core 0 (Wi-Fi, UART, web server, etc.).
 *
 * ── Flow control ───────────────────────────────────────────────────────────
 *   available() returns the number of free slots remaining in the FreeRTOS
 *   queue.  The communication layer should send an ACK / backpressure signal
 *   to the host when available() < FLOW_CONTROL_THRESHOLD (= 4).
 *
 * ── Underrun handling ──────────────────────────────────────────────────────
 *   The executor blocks on xQueueReceive() when the block queue is empty and
 *   blocks on an ISR task notification when the driver's software ring is
 *   full. There is no CPU-side spin loop. If the producer still cannot keep
 *   up, the encoder emits one LOW-level pause chunk and ends the transaction,
 *   matching the FastAccelStepper IDF5 behavior.
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
    esp_err_t enqueueBlock(const step_block_t& block,
                           uint32_t timeout_ms = portMAX_DELAY);

    /**
     * @brief Number of free slots remaining in the block queue.
     *
     * Use for flow control: signal the host when this drops below
     * FLOW_CONTROL_THRESHOLD.
     */
    uint32_t available() const;

    /** @brief Return the motor id (0 or 1). */
    uint8_t motorId() const { return motor_id_; }

private:
    StepperDriver& driver_;
    uint8_t        motor_id_;

    QueueHandle_t  queue_  {nullptr};
    TaskHandle_t   task_   {nullptr};

    /**
     * @brief Executor task body.
     *
    * Pinned to Core 1, priority 24. Dequeues step_block_t and calls
    * driver_.pushBlock(). Drains the whole block queue when woken so the
    * software ring stays as full as possible.
     */
    static void executorTask(void* arg);
};
