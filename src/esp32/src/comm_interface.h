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
#include <driver/gpio.h>
#include <esp_err.h>

#include "messages.h"
#include "stepper_queue.h"
#include "motion_planner.h"

// Pinout bundle for SPI + lateral endstop wiring
struct SpiBusPins {
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
    gpio_num_t ready;
    gpio_num_t home_pin_no;
    gpio_num_t home_pin_nc;
};

class CommInterface {
public:
    // Construct the SPI communication interface, wiring up per-axis stepper queues.
    CommInterface(StepperQueue* queues[], uint8_t n_motors);

    // Initialize SPI slave and start both the SPI task (Core 0) and the executor (Core 1).
    esp_err_t init(const SpiBusPins& pins);

private:
    // Per-axis StepperQueue pointers. Entries may be null for unused axes.
    StepperQueue*   queues_[SPI_MAX_AXES];
    uint8_t         n_motors_;             // Number of valid axes configured
    SpiBusPins      pins_ {};              // Copied pin configuration
    MotionPlanner   planner_;              // Planner: converts blocks -> planned segments

    // --- SPI protocol runtime state (for status + dedupe) ---
    uint16_t        last_rx_sequence_ {0}; // Sequence of last received request
    uint8_t         last_rx_type_ {static_cast<uint8_t>(SpiMessageType::NOP)}; // Msg type of last received
    uint8_t         last_result_ {static_cast<uint8_t>(SpiMessageResult::OK)};  // Result code last produced

    struct ProcessedRequestSignature {
        uint16_t sequence {0xFFFFu};
        uint16_t payload_length {0};
        uint16_t crc {0};
        uint8_t  msg_type {static_cast<uint8_t>(SpiMessageType::NOP)};
        uint8_t  result {static_cast<uint8_t>(SpiMessageResult::OK)};
        bool     valid {false};
    };

    static constexpr uint8_t RECENT_REQUEST_CACHE_DEPTH = 4;
    ProcessedRequestSignature recent_request_cache_[RECENT_REQUEST_CACHE_DEPTH] {};
    uint8_t         recent_request_cache_write_index_ {0};

    // Last accepted multi-axis block sequence (block_seq) for deduplication.
    uint16_t        last_accepted_block_seq_ {0xFFFFu};

    // Most recently fully executed motion_sequence published by executor (Core 1).
    // Atomic so Core 0 can read it without locks.
    std::atomic<uint16_t>   last_executed_sequence_ {0xFFFFu};

    // Build the status payload for the outgoing status frame (called on Core 0).
    void buildStatusFrame(uint8_t* out_frame) const;

    // Return the debounced driver-owned lateral endstop state.
    uint8_t readLateralEndstopState() const;

    // Return true if axis movement is permitted by the lateral homing guard.
    bool isLateralMovementAllowed(uint8_t axis_id, bool direction) const;

    // Validate and dispatch an incoming request frame payload (called from spiTask).
    esp_err_t handleFrame(const SpiMessageHeader& header, const uint8_t* payload);

    // Message handlers (one per supported SpiMessageType).
    esp_err_t handleEnableAxis(const EnableAxisPayload& payload);      // Enable/disable an axis
    esp_err_t handleEmergencyStop(const EmergencyStopPayload& payload);// Emergency stop (global or per-axis)
    esp_err_t handleStopAxis(const EmergencyStopPayload& payload);     // Graceful stop (no ring flush)
    esp_err_t handleDisableAll();                                     // Disable all axes
    esp_err_t handleResetStats();                                     // Reset counters/diagnostics
    esp_err_t handleStepBlock(const StepBlockPayload& payload);        // Legacy per-axis step block
    esp_err_t handleSegmentBlock(const SegmentBlockPayload& payload);  // Legacy per-axis segment block
    esp_err_t handleMultiAxisSegmentBlock(const uint8_t* payload, uint16_t payload_length); // Main multi-axis message
    esp_err_t handleFlush(const FlushPayload& payload);                // Post a flush request
    void notifySegmentExecuted(uint16_t motion_seq);                   // Called by executor when a segment completes
    esp_err_t handleEnableEndstop(const EnableEndstopPayload& payload);// Arm/disarm lateral endstop

    // Tasks: SPI slave loop runs on Core 0; multi-axis executor runs on Core 1.
    static void spiTask(void* arg);
    static void multiAxisExecutorTask(void* arg);
};
