/**
 * @file comm_status_builder.h
 * @brief Build protocol-compliant status frames.
 */

#pragma once

#include <stdint.h>

#include "messages.h"

class CommRuntime;

/**
 * @brief Composes SPI `STATUS` frames from runtime and planner state.
 */
class CommStatusBuilder {
public:
    /** @brief Create a status builder bound to runtime context. */
    explicit CommStatusBuilder(const CommRuntime& runtime);

    /**
     * @brief Build one full SPI frame containing a `StatusPayload`.
     * @param out_frame Output DMA buffer, size must be `SPI_FRAME_SIZE`.
     */
    void buildStatusFrame(uint8_t* out_frame) const;

private:
    const CommRuntime& runtime_;

    void populateAxisStatus(StatusPayload& payload) const;
    void populateProtocolStatus(StatusPayload& payload) const;
    void populatePlannerStatus(StatusPayload& payload) const;
    void populateEndstopStatus(StatusPayload& payload) const;
};
