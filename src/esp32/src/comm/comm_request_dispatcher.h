/**
 * @file comm_request_dispatcher.h
 * @brief Dispatch validated SPI requests to motion/control handlers.
 */

#pragma once

#include <esp_err.h>

#include "messages.h"

class CommRuntime;

/**
 * @brief Stateful SPI request dispatcher with deduplication cache.
 *
 * Input messages are assumed to be transport-validated (magic/version/CRC). This
 * class performs payload-level validation, executes the requested action, maps
 * `esp_err_t` to protocol result codes and publishes ACK metadata.
 */
class CommRequestDispatcher {
public:
    /** @brief Create dispatcher bound to shared runtime context. */
    explicit CommRequestDispatcher(CommRuntime& runtime);

    /**
     * @brief Process a validated request frame.
     * @param header Parsed SPI header.
     * @param payload Pointer to payload bytes.
     * @return `SpiMessageResult` value encoded as `uint8_t`.
     */
    uint8_t processValidatedRequest(const SpiMessageHeader& header,
                                    const uint8_t* payload);

private:
    struct ProcessedRequestSignature {
        uint16_t sequence {0xFFFFu};
        uint16_t payload_length {0};
        uint16_t crc {0};
        uint8_t  msg_type {static_cast<uint8_t>(SpiMessageType::NOP)};
        uint8_t  result {static_cast<uint8_t>(SpiMessageResult::OK)};
        bool     valid {false};
    };

    static constexpr uint8_t RECENT_REQUEST_CACHE_DEPTH = 4;

    CommRuntime& runtime_;
    ProcessedRequestSignature recent_request_cache_[RECENT_REQUEST_CACHE_DEPTH] {};
    uint8_t recent_request_cache_write_index_ {0};

    static bool shouldPublishAckForMessageType(uint8_t msg_type);
    const ProcessedRequestSignature* findCachedRequest(const SpiMessageHeader& header) const;
    void cacheProcessedRequest(const SpiMessageHeader& header, uint8_t result);
    void publishAck(const SpiMessageHeader& header, uint8_t result);
    static uint8_t mapRequestResult(esp_err_t err);

    esp_err_t handleFrame(const SpiMessageHeader& header, const uint8_t* payload);
    esp_err_t handleEnableAxis(const EnableAxisPayload& payload);
    esp_err_t handleEmergencyStop(const EmergencyStopPayload& payload);
    esp_err_t handleStopAxis(const EmergencyStopPayload& payload);
    esp_err_t handleDisableAll();
    esp_err_t handleResetStats();
    esp_err_t handleEnableEndstop(const EnableEndstopPayload& payload);
    esp_err_t handleStepBlock(const StepBlockPayload& payload);
    esp_err_t handleSegmentBlock(const SegmentBlockPayload& payload);
    esp_err_t handleMultiAxisSegmentBlock(const uint8_t* payload, uint16_t payload_length);
    esp_err_t handleFlush(const FlushPayload& payload);
};
