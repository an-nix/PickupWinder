/**
 * @file stepper_queue.h
 * @brief Per-motor helper around StepperDriver for multi-axis execution.
 */

#pragma once

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include "step_types.h"
#include "stepper_driver.h"

class StepperQueue {
public:
    StepperQueue(StepperDriver& driver, uint8_t motor_id);
    esp_err_t init();
    esp_err_t enqueueMotionBlock(const motion_block_t& block,
                                 uint32_t timeout_ms = portMAX_DELAY);
    esp_err_t enqueueStepBlock(const step_block_t& block,
                               uint32_t timeout_ms = portMAX_DELAY);
    esp_err_t enqueueSegmentBlock(const segment_block_t& block,
                                  uint32_t timeout_ms = portMAX_DELAY);

    esp_err_t enqueueBlock(const step_block_t& block,
                           uint32_t timeout_ms = portMAX_DELAY) {
        return enqueueStepBlock(block, timeout_ms);
    }

    esp_err_t executeConstantRateBlock(bool direction,
                                       uint16_t step_count,
                                       uint32_t duration_us);

    esp_err_t kickStart();
    void gracefulStop();
    uint32_t available() const;
    uint8_t motorId() const { return motor_id_; }
    StepperDriver& driver() { return driver_; }
    const StepperDriver& driver() const { return driver_; }

    void setMultiExecActive(bool active) {
        multi_exec_active_.store(active, std::memory_order_release);
    }

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
