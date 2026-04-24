/**
 * @file comm_runtime.h
 * @brief Shared runtime context for communication components.
 */

#pragma once

#include <atomic>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "messages.h"
#include "../motion/motion_planner.h"
#include "../motion/stepper_queue.h"

/**
 * @brief Mutable runtime state shared by dispatcher, status builder and executor.
 *
 * This class centralizes communication-side state and utility methods to keep
 * component boundaries explicit and avoid tight coupling between classes.
 */
class CommRuntime {
public:
    /**
     * @brief Build a runtime context from the installed axis queues and planner.
     */
    CommRuntime(StepperQueue* queues[], uint8_t n_motors, MotionPlanner& planner)
        : n_motors_(n_motors < SPI_MAX_AXES ? n_motors : SPI_MAX_AXES)
        , planner_(planner)
    {
        for (uint8_t i = 0; i < SPI_MAX_AXES; ++i) {
            queues_[i] = (i < n_motors_) ? queues[i] : nullptr;
        }
    }

    /** @brief Get the queue for one axis, or null if not available. */
    StepperQueue* queueForAxis(uint8_t axis_id) const
    {
        if (axis_id >= n_motors_) {
            return nullptr;
        }
        return queues_[axis_id];
    }

    /** @brief Check whether an axis is installed and addressable. */
    bool hasAxis(uint8_t axis_id) const
    {
        return queueForAxis(axis_id) != nullptr;
    }

    /** @brief Return the number of configured motors (clamped to protocol max). */
    uint8_t motorCount() const
    {
        return n_motors_;
    }

    /** @brief Get planner instance used for block expansion and scheduling. */
    MotionPlanner& planner()
    {
        return planner_;
    }

    /** @brief Const planner accessor. */
    const MotionPlanner& planner() const
    {
        return planner_;
    }

    /** @brief Get queue receiving decoded multi-axis blocks. */
    QueueHandle_t multiAxisQueue() const
    {
        return multi_axis_queue_;
    }

    /** @brief Set queue receiving decoded multi-axis blocks. */
    void setMultiAxisQueue(QueueHandle_t queue)
    {
        multi_axis_queue_ = queue;
    }

    /** @brief Get queue receiving flush requests. */
    QueueHandle_t flushQueue() const
    {
        return flush_queue_;
    }

    /** @brief Set queue receiving flush requests. */
    void setFlushQueue(QueueHandle_t queue)
    {
        flush_queue_ = queue;
    }

    /** @brief Last acknowledged RX sequence for status sideband. */
    uint16_t lastRxSequence() const
    {
        return last_rx_sequence_;
    }

    /** @brief Last acknowledged RX message type for status sideband. */
    uint8_t lastRxType() const
    {
        return last_rx_type_;
    }

    /** @brief Last request result published in status sideband. */
    uint8_t lastResult() const
    {
        return last_result_;
    }

    /**
     * @brief Publish the acknowledgement triplet exposed by status frames.
     */
    void publishAck(const SpiMessageHeader& header, uint8_t result)
    {
        last_rx_sequence_ = header.sequence;
        last_rx_type_ = header.msg_type;
        last_result_ = result;
    }

    /** @brief Last accepted multi-axis block sequence, wrap-aware domain. */
    uint16_t lastAcceptedBlockSeq() const
    {
        return last_accepted_block_seq_;
    }

    /** @brief Update last accepted multi-axis block sequence. */
    void setLastAcceptedBlockSeq(uint16_t block_seq)
    {
        last_accepted_block_seq_ = block_seq;
    }

    /** @brief Highest executed motion sequence reported by executor. */
    uint16_t lastExecutedSequence() const
    {
        return last_executed_sequence_.load(std::memory_order_acquire);
    }

    /**
     * @brief Report a segment execution completion with wrap-aware ordering.
     */
    void notifySegmentExecuted(uint16_t motion_seq)
    {
        uint16_t current = last_executed_sequence_.load(std::memory_order_relaxed);
        if (sequence_is_newer_u16(motion_seq, current)) {
            last_executed_sequence_.store(motion_seq, std::memory_order_release);
        }
    }

    /** @brief Read lateral endstop state exposed by axis 1 driver. */
    uint8_t readLateralEndstopState() const
    {
        StepperQueue* lateral_queue = queueForAxis(1);
        if (lateral_queue == nullptr) {
            return static_cast<uint8_t>(LateralEndstopState::ABSENT);
        }
        return lateral_queue->driver().reportedEndstopState();
    }

    /**
     * @brief Validate lateral movement direction against active endstop policy.
     */
    bool isLateralMovementAllowed(uint8_t axis_id, bool direction) const
    {
        if (axis_id != 1) {
            return true;
        }

        StepperQueue* axis_queue = queueForAxis(axis_id);
        if (axis_queue == nullptr) {
            return false;
        }
        return axis_queue->driver().isEndstopMoveAllowed(direction);
    }

private:
    StepperQueue* queues_[SPI_MAX_AXES] {};
    uint8_t n_motors_;
    MotionPlanner& planner_;

    QueueHandle_t multi_axis_queue_ {nullptr};
    QueueHandle_t flush_queue_ {nullptr};

    uint16_t last_rx_sequence_ {0};
    uint8_t last_rx_type_ {static_cast<uint8_t>(SpiMessageType::NOP)};
    uint8_t last_result_ {static_cast<uint8_t>(SpiMessageResult::OK)};
    uint16_t last_accepted_block_seq_ {0xFFFFu};
    std::atomic<uint16_t> last_executed_sequence_ {0xFFFFu};
};
