/**
 * @file messages.h
 * @brief Fixed-size SPI request/response messages shared by host and ESP32.
 *
 * The protocol is intentionally small and explicit:
 *
 * - one fixed-size SPI transfer = one wire frame
 * - every frame starts with a typed header
 * - payloads are message-specific packed structs
 * - CRC16 protects header+payload
 * - all integers are little-endian
 *
 * This keeps the transport easy to extend: new commands only need a new
 * payload struct plus one new handler in CommInterface.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "step_types.h"

// ---------------------------------------------------------------------------
// Wire constants
// ---------------------------------------------------------------------------

static constexpr uint16_t SPI_MSG_MAGIC          = 0x5057; // 'P''W'
static constexpr uint8_t  SPI_MSG_VERSION        = 1;
static constexpr size_t   SPI_FRAME_SIZE         = 512;
static constexpr size_t   SPI_MAX_PAYLOAD_SIZE   = SPI_FRAME_SIZE - 12;
static constexpr uint8_t  SPI_MAX_AXES           = 2;

// ---------------------------------------------------------------------------
// Message types
// ---------------------------------------------------------------------------

enum class SpiMessageType : uint8_t {
    NOP          = 0x00,
    ENABLE_AXIS  = 0x01,
    ESTOP        = 0x02,
    STOP_AXIS    = 0x03,
    DISABLE_ALL  = 0x04,
    RESET_STATS  = 0x05,
    GET_STATUS   = 0x06,
    STEP_BLOCK   = 0x10,
    SEGMENT_BLOCK = 0x11,
    PING         = 0x7F,

    STATUS       = 0x80,
};

enum class SpiMessageResult : uint8_t {
    OK             = 0x00,
    BAD_MAGIC      = 0x01,
    BAD_VERSION    = 0x02,
    BAD_LENGTH     = 0x03,
    BAD_CRC        = 0x04,
    UNKNOWN_TYPE   = 0x05,
    BAD_AXIS       = 0x06,
    QUEUE_FULL     = 0x07,
    INTERNAL_ERROR = 0x08,
};

namespace SpiStepFlags {
    static constexpr uint8_t NONE        = 0x00;
    static constexpr uint8_t DIR_REVERSE = 0x01;
}

// ---------------------------------------------------------------------------
// Frame header
// ---------------------------------------------------------------------------

struct __attribute__((packed)) SpiMessageHeader {
    uint16_t magic;
    uint8_t  version;
    uint8_t  msg_type;
    uint16_t sequence;
    uint16_t payload_length;
    uint16_t flags;
    uint16_t crc16;
};

static_assert(sizeof(SpiMessageHeader) == 12, "SpiMessageHeader must be 12 bytes");

// ---------------------------------------------------------------------------
// Request payloads
// ---------------------------------------------------------------------------

struct __attribute__((packed)) EnableAxisPayload {
    uint8_t axis_id;
    uint8_t enable;
    uint8_t reserved[2];
};

struct __attribute__((packed)) EmergencyStopPayload {
    uint8_t axis_id;     // 0xFF = all axes
    uint8_t reserved[3];
};

struct __attribute__((packed)) StepEntryMessage {
    uint32_t interval_ticks;
    uint8_t  flags;
};

static_assert(sizeof(StepEntryMessage) == 5, "StepEntryMessage must be 5 bytes");

struct __attribute__((packed)) StepBlockPayload {
    uint8_t          axis_id;
    uint8_t          block_seq;
    uint16_t         step_count;
    StepEntryMessage entries[STEP_BLOCK_SIZE];
};

static_assert(sizeof(StepBlockPayload) == 324, "StepBlockPayload must be 324 bytes");

struct __attribute__((packed)) MotionSegmentMessage {
    uint16_t step_count;
    uint16_t start_ticks;
    int16_t  add_ticks;
    uint8_t  flags;
    uint8_t  reserved;
};

static_assert(sizeof(MotionSegmentMessage) == 8, "MotionSegmentMessage must be 8 bytes");

struct __attribute__((packed)) SegmentBlockPayload {
    uint8_t              axis_id;
    uint8_t              block_seq;
    uint16_t             segment_count;
    MotionSegmentMessage segments[SEGMENT_BLOCK_SIZE];
};

static_assert(sizeof(SegmentBlockPayload) == 484, "SegmentBlockPayload must be 484 bytes");

// ---------------------------------------------------------------------------
// Response payloads
// ---------------------------------------------------------------------------

struct __attribute__((packed)) StatusPayload {
    uint32_t uptime_ms;
    uint16_t queue_free_slots[SPI_MAX_AXES];
    uint16_t ring_free_slots[SPI_MAX_AXES];
    uint32_t underrun_count[SPI_MAX_AXES];
    uint16_t last_rx_sequence;
    uint8_t  last_rx_type;
    uint8_t  last_result;
    uint8_t  protocol_version;
    uint8_t  enabled_mask;
    uint8_t  running_mask;
    uint8_t  reserved[5];
};

static_assert(sizeof(StatusPayload) == 32, "StatusPayload must be 32 bytes");

// ---------------------------------------------------------------------------
// CRC16-CCITT-FALSE
// ---------------------------------------------------------------------------

inline uint16_t spi_crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

inline void spi_message_zero_frame(uint8_t* frame) {
    memset(frame, 0, SPI_FRAME_SIZE);
}

inline void spi_message_finalize(uint8_t* frame) {
    auto* header = reinterpret_cast<SpiMessageHeader*>(frame);
    header->crc16 = 0;
    header->crc16 = spi_crc16_ccitt(frame, sizeof(SpiMessageHeader) + header->payload_length);
}

inline void spi_message_init_header(SpiMessageHeader& header,
                                    SpiMessageType type,
                                    uint16_t sequence,
                                    uint16_t payload_length,
                                    uint16_t flags = 0) {
    header.magic = SPI_MSG_MAGIC;
    header.version = SPI_MSG_VERSION;
    header.msg_type = static_cast<uint8_t>(type);
    header.sequence = sequence;
    header.payload_length = payload_length;
    header.flags = flags;
    header.crc16 = 0;
}

inline bool spi_message_header_is_sane(const SpiMessageHeader& header) {
    if (header.magic != SPI_MSG_MAGIC) {
        return false;
    }
    if (header.version != SPI_MSG_VERSION) {
        return false;
    }
    if (header.payload_length > SPI_MAX_PAYLOAD_SIZE) {
        return false;
    }
    return true;
}

inline bool spi_message_validate(const uint8_t* frame, SpiMessageHeader& out_header) {
    memcpy(&out_header, frame, sizeof(SpiMessageHeader));
    if (!spi_message_header_is_sane(out_header)) {
        return false;
    }
    const uint16_t received_crc = out_header.crc16;
    SpiMessageHeader temp = out_header;
    temp.crc16 = 0;

    uint8_t tmp[SPI_FRAME_SIZE];
    memcpy(tmp, frame, sizeof(SpiMessageHeader) + out_header.payload_length);
    memcpy(tmp, &temp, sizeof(SpiMessageHeader));
    const uint16_t expected_crc = spi_crc16_ccitt(tmp, sizeof(SpiMessageHeader) + out_header.payload_length);
    return expected_crc == received_crc;
}
