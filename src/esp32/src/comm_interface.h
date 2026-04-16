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

    /** Build the status payload for the next SPI response frame. */
    void buildStatusFrame(uint8_t* out_frame) const;

    /** Parse and execute one validated request frame. */
    esp_err_t handleFrame(const SpiMessageHeader& header, const uint8_t* payload);

    esp_err_t handleEnableAxis(const EnableAxisPayload& payload);
    esp_err_t handleEmergencyStop(const EmergencyStopPayload& payload);
    esp_err_t handleStopAxis(const EmergencyStopPayload& payload);
    esp_err_t handleDisableAll();
    esp_err_t handleResetStats();
    esp_err_t handleStepBlock(const StepBlockPayload& payload);
    esp_err_t handleSegmentBlock(const SegmentBlockPayload& payload);

    /** Core 0 SPI slave task. */
    static void spiTask(void* arg);
};
