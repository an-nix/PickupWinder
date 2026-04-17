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

#include <driver/gpio.h>
#include <esp_err.h>

#include "messages.h"
#include "stepper_queue.h"

struct SpiBusPins {
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
    gpio_num_t home_pin_no;
    gpio_num_t home_pin_nc;
};

class CommInterface {
public:
    /**
     * @brief Construct the SPI communication interface.
     *
     * @param queues   Stepper queues indexed by axis id.
     * @param n_motors Number of valid queues.
     */
    CommInterface(StepperQueue* queues[], uint8_t n_motors);

    /**
     * @brief Initialize the ESP32 SPI slave and start the communication task.
     */
    esp_err_t init(const SpiBusPins& pins);

private:
    StepperQueue*   queues_[SPI_MAX_AXES];
    uint8_t         n_motors_;
    SpiBusPins      pins_ {};

    uint16_t        last_rx_sequence_ {0};
    uint8_t         last_rx_type_ {static_cast<uint8_t>(SpiMessageType::NOP)};
    uint8_t         last_result_ {static_cast<uint8_t>(SpiMessageResult::OK)};

    /**
     * @brief Motion sequence of the most recently fully-executed multi-axis
     *        segment.  Updated by the executor task (Core 1) and read by the
     *        SPI task (Core 0); access is protected by the portMUX spinlock
     *        below.  Initialised to 0xFFFF so the host's first segment always
     *        compares as "not yet executed".
     */
    volatile uint16_t   last_executed_sequence_ {0xFFFFu};

    /** Spinlock protecting last_executed_sequence_ across cores. */
    portMUX_TYPE        exec_seq_mux_ {portMUX_INITIALIZER_UNLOCKED};

    /** Build the status payload for the next SPI response frame. */
    void buildStatusFrame(uint8_t* out_frame) const;

    /** Read the current lateral endstop state from the configured pins. */
    uint8_t readLateralEndstopState() const;

    /** Return true if the given axis may move given the lateral endstop state. */
    bool isLateralMovementAllowed(uint8_t axis_id) const;

    /** Parse and execute one validated request frame. */
    esp_err_t handleFrame(const SpiMessageHeader& header, const uint8_t* payload);

    esp_err_t handleEnableAxis(const EnableAxisPayload& payload);
    esp_err_t handleEmergencyStop(const EmergencyStopPayload& payload);
    esp_err_t handleStopAxis(const EmergencyStopPayload& payload);
    esp_err_t handleDisableAll();
    esp_err_t handleResetStats();
    esp_err_t handleStepBlock(const StepBlockPayload& payload);
    esp_err_t handleSegmentBlock(const SegmentBlockPayload& payload);

    /**
     * @brief Handle a MULTI_AXIS_SEGMENT_BLOCK (0x13) frame.
     *
     * Decodes the variable-length multi-axis segment payload and dispatches
     * one multi_axis_segment_block_t to the global multi-axis queue.
     */
    esp_err_t handleMultiAxisSegmentBlock(const uint8_t* payload, uint16_t payload_length);

    /**
     * @brief Handle a FLUSH (0x12) frame.
     *
     * Instructs the executor to discard all queued segments whose
     * motion_sequence > flush_sequence, allowing the host to inject a
     * new trajectory without draining the current buffer first.
     */
    esp_err_t handleFlush(const FlushPayload& payload);

    /**
     * @brief Update last_executed_sequence_ under the spinlock.
     *
     * Must be called by the executor task whenever it completes a
     * multi-axis segment.
     *
     * @param motion_seq  The motion_sequence of the just-completed segment.
     */
    void notifySegmentExecuted(uint16_t motion_seq);

    /** Core 0 SPI slave task. */
    static void spiTask(void* arg);

    /**
     * @brief Core 1 multi-axis segment executor task.
     *
     * Consumes multi_axis_block_t objects from the global multi-axis queue,
     * distributes constant-rate step bursts to each per-axis StepperQueue,
     * and calls notifySegmentExecuted() after each segment completes.  Also
     * drains the global flush queue between blocks to support host trajectory
     * cancellation without draining the entire axis queue first.
     *
     * Pinned to Core 1 at priority 24 (same as per-axis executor tasks).
     * Only one multi-axis executor task is ever launched.
     */
    static void multiAxisExecutorTask(void* arg);
};
