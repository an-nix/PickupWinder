/**
 * @file comm_interface.h
 * @brief SPI slave transport for host → ESP32 motion messages.
 *
 * The communication layer is intentionally separated from the stepper runtime:
 *
 * - `messages.h` defines all wire-level structures and result codes
 * - `CommInterface` owns the SPI slave task and DMA buffers
 * - handler methods translate validated messages into queue/driver operations
 *
 * This structure keeps the code maintainable when adding new commands:
 * define a new payload in `messages.h`, then add one handler here.
 */

#pragma once

#include <atomic>
#include <memory>
#include <esp_err.h>

#include "comm_types.h"
#include "messages.h"
#include "motion_planner.h"
#include "stepper_queue.h"

class CommRequestDispatcher;
class CommStatusBuilder;
class SpiSlaveTransport;
class MultiAxisExecutor;

class CommInterface {
public:
    // Construct the SPI communication interface, wiring up per-axis stepper queues.
    CommInterface(StepperQueue* queues[], uint8_t n_motors);
    ~CommInterface();

    // Initialize SPI slave and start both the SPI task (Core 0) and the executor (Core 1).
    esp_err_t init(const SpiBusPins& pins);

private:
    friend class CommRequestDispatcher;
    friend class CommStatusBuilder;
    friend class SpiSlaveTransport;
    friend class MultiAxisExecutor;

    // Per-axis StepperQueue pointers. Entries may be null for unused axes.
    StepperQueue*   queues_[SPI_MAX_AXES];
    uint8_t         n_motors_;             // Number of valid axes configured
    SpiBusPins      pins_ {};              // Copied pin configuration
    MotionPlanner   planner_;              // Planner: converts blocks -> planned segments

    QueueHandle_t   multi_axis_queue_ {nullptr};
    QueueHandle_t   flush_queue_ {nullptr};

    // --- SPI protocol runtime state (for status + dedupe) ---
    uint16_t        last_rx_sequence_ {0}; // Sequence of last received request
    uint8_t         last_rx_type_ {static_cast<uint8_t>(SpiMessageType::NOP)}; // Msg type of last received
    uint8_t         last_result_ {static_cast<uint8_t>(SpiMessageResult::OK)};  // Result code last produced

    // Last accepted multi-axis block sequence (block_seq) for deduplication.
    uint16_t        last_accepted_block_seq_ {0xFFFFu};

    // Most recently fully executed motion_sequence published by executor (Core 1).
    // Atomic so Core 0 can read it without locks.
    std::atomic<uint16_t>   last_executed_sequence_ {0xFFFFu};

    std::unique_ptr<CommRequestDispatcher> request_dispatcher_;
    std::unique_ptr<CommStatusBuilder> status_builder_;
    std::unique_ptr<SpiSlaveTransport> spi_transport_;
    std::unique_ptr<MultiAxisExecutor> multi_axis_executor_;

    // Lookup helpers used by handlers/status building.
    StepperQueue* queueForAxis(uint8_t axis_id) const;
    bool hasAxis(uint8_t axis_id) const;

    esp_err_t configureHomePins() const;
    esp_err_t createRuntimeQueues();
    esp_err_t registerEndstopIsr() const;

    // Return the debounced driver-owned lateral endstop state.
    uint8_t readLateralEndstopState() const;

    // Return true if axis movement is permitted by the lateral homing guard.
    bool isLateralMovementAllowed(uint8_t axis_id, bool direction) const;

    void notifySegmentExecuted(uint16_t motion_seq);
};
