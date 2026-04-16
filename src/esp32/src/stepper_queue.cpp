/**
 * @file stepper_queue.cpp
 * @brief Per-motor FreeRTOS queue + executor task implementation.
 */

#include "stepper_queue.h"

#include <esp_log.h>
#include <esp_check.h>

static const char* TAG = "stepper_queue";

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
    // ── FreeRTOS queue ──────────────────────────────────────────────────────
    //
    // Each item is a full step_block_t copy.  sizeof(step_block_t) =
    //   64 × (4 + 1) bytes + 4 bytes count  ≈  324 bytes.
    // STEPPER_QUEUE_DEPTH = 16 → queue RAM ≈ 5 kB per motor.
    //
    queue_ = xQueueCreate(STEPPER_QUEUE_DEPTH, sizeof(step_block_t));
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
// enqueueBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::enqueueBlock(const step_block_t& block,
                                     uint32_t timeout_ms)
{
    const TickType_t ticks = (timeout_ms == portMAX_DELAY)
                             ? portMAX_DELAY
                             : pdMS_TO_TICKS(timeout_ms);

    if (xQueueSend(queue_, &block, ticks) != pdTRUE) {
        ESP_LOGW(TAG, "motor%u: queue full — block dropped (seq overflow)",
                 motor_id_);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// available()
// ---------------------------------------------------------------------------

uint32_t StepperQueue::available() const
{
    return static_cast<uint32_t>(uxQueueSpacesAvailable(queue_));
}

// ---------------------------------------------------------------------------
// executorTask()  — Core 1, priority 24
// ---------------------------------------------------------------------------

void StepperQueue::executorTask(void* arg)
{
    StepperQueue* self = static_cast<StepperQueue*>(arg);
    StepperDriver& driver = self->driver_;
    step_block_t block;

    ESP_LOGI(TAG, "motor%u: executor task started", self->motor_id_);

    for (;;) {
        // Block until at least one block is available.
        if (xQueueReceive(self->queue_, &block, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // Drain loop: push ALL queued blocks into the ring buffer in one go.
        //
        // Pushing a single block at a time leaves only 64 entries in the ring.
        // At high speed, that is less than a FreeRTOS tick of look-ahead.
        // Draining the full queue fills the ring with up to STEP_RING_SIZE
        // entries, which maximizes host-side look-ahead.
        //
        // pushBlock() now blocks on an ISR task notification when the ring is
        // full. This mirrors FastAccelStepper's "task blocks / ISR streams"
        // model and avoids starving the CPU1 idle task.
        do {
            esp_err_t err = driver.pushBlock(block);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "motor%u: pushBlock error: %s",
                         self->motor_id_, esp_err_to_name(err));
            }
        } while (xQueueReceive(self->queue_, &block, 0) == pdTRUE);
    }
}
