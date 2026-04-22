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
static constexpr uint8_t  SPI_MSG_VERSION        = 3;
static constexpr size_t   SPI_FRAME_SIZE         = 512;
static constexpr size_t   SPI_MAX_PAYLOAD_SIZE   = SPI_FRAME_SIZE - 12;
static constexpr uint8_t  SPI_MAX_AXES           = 4;

// ---------------------------------------------------------------------------
// Message types
// ---------------------------------------------------------------------------

enum class SpiMessageType : uint8_t {
    NOP                      = 0x00,
    ENABLE_AXIS              = 0x01,
    ESTOP                    = 0x02,
    STOP_AXIS                = 0x03,
    DISABLE_ALL              = 0x04,
    RESET_STATS              = 0x05,
    GET_STATUS               = 0x06,
    STEP_BLOCK               = 0x10,
    SEGMENT_BLOCK            = 0x11,
    /** Discard all queued segments with motion_sequence > flush_sequence. */
    FLUSH                    = 0x12,
    /**
     * Synchronised multi-axis time-based segment block.
     * All axes share the same duration_us; steps may differ per axis.
     */
    MULTI_AXIS_SEGMENT_BLOCK = 0x13,
    /** Arm or disarm the hardware endstop on a given axis. */
    ENABLE_ENDSTOP           = 0x14,
    PING                     = 0x7F,

    STATUS                   = 0x80,
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
    ENDSTOP_BLOCKED = 0x09,
};

enum class LateralEndstopState : uint8_t {
    PRESENT_OPEN   = 0x00,
    PRESENT_CLOSED = 0x01,
    ABSENT         = 0xFF,
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

/**
 * @brief Payload for FLUSH (0x12).
 *
 * The ESP32 must discard every queued MULTI_AXIS_SEGMENT_BLOCK whose
 * motion_sequence > flush_sequence, completing the current executing segment
 * first.  After flush the executor resumes from the next segment the host
 * sends.
 */
struct __attribute__((packed)) FlushPayload {
    uint16_t flush_sequence; /**< Discard segments with motion_sequence > this  */
    uint8_t  reserved[2];
};

static_assert(sizeof(FlushPayload) == 4, "FlushPayload must be 4 bytes");

struct __attribute__((packed)) EnableEndstopPayload {
    uint8_t axis_id;  ///< axis to arm/disarm
    uint8_t arm;      ///< 1 = arm, 0 = disarm
    uint8_t reserved[2];
};

static_assert(sizeof(EnableEndstopPayload) == 4, "EnableEndstopPayload must be 4 bytes");

/**
 * @brief Per-axis step count entry inside a MultiAxisSegmentEntry.
 *
 * axis_count entries immediately follow the fixed MultiAxisSegmentEntry header
 * in the wire payload; they are not a separate struct.
 */

/**
 * @brief One synchronised multi-axis segment (variable-length wire record).
 *
 * Wire layout:
 *   uint16_t  motion_sequence   — monotonically increasing segment identifier
 *   uint16_t  duration_us       — wall-clock duration of this segment in µs
 *   uint16_t  direction_mask    — bit i = 1 → axis i runs in reverse
 *   uint16_t  step_counts[N]    — one per axis (N = axis_count in block header)
 *
 * Total per-segment: 6 + N*2 bytes.
 */

/**
 * @brief Header of a MULTI_AXIS_SEGMENT_BLOCK payload.
 *
 * Followed immediately by `segment_count` variable-length segment records.
 */
struct __attribute__((packed)) MultiAxisSegmentBlockHeader {
    uint16_t block_seq;      /**< Block rolling sequence for duplicate detect   */
    uint8_t  segment_count;  /**< Number of segment records that follow          */
    uint8_t  axis_count;     /**< Number of axes per segment record              */
    /**
     * axis_ids[axis_count] follows the header as a variable-length region.
     * Each element maps slot index → logical axis_id.
     */
};

static_assert(sizeof(MultiAxisSegmentBlockHeader) == 4,
              "MultiAxisSegmentBlockHeader must be 4 bytes");

// ---------------------------------------------------------------------------
// Response payloads
// ---------------------------------------------------------------------------

struct __attribute__((packed)) StatusPayload {
    uint32_t uptime_ms;
    uint16_t queue_free_slots[SPI_MAX_AXES];
    uint16_t ring_free_slots[SPI_MAX_AXES];
    uint32_t underrun_count[SPI_MAX_AXES];
    /**
     * Last non-telemetry request durably acknowledged by firmware.
     * `GET_STATUS`, `PING`, `NOP`, and transient parse failures do not
     * advance this published ACK tuple.
     */
    uint16_t last_rx_sequence;
    uint8_t  last_rx_type;
    uint8_t  last_result;
    uint8_t  protocol_version;
    uint8_t  enabled_mask;
    uint8_t  running_mask;
    uint8_t lateral_endstop_state;
    /** Bit N = 1 means axis N endstop is armed (will stop motion on trigger). */
    uint8_t endstop_armed_mask;
    /** Bit N = 1 means axis N hit its endstop since the last arm. */
    uint8_t endstop_hit_mask;
    /**
    * Motion sequence of the most recently fully-executed multi-axis segment.
     * The host uses this to compute how much future motion is still buffered
     * on the MCU.  Initialised to 0xFFFF ("nothing executed yet") so that the
     * host's starting condition (segment.sequence > last_executed_sequence)
     * is always true before the first segment completes.
    *
    * Segments rejected by endstop gating or drained during recovery must not
    * advance this field.
     *
     * This field is updated by the executor task (Core 1) under a spinlock
     * and read by the SPI task (Core 0) — both must access it atomically.
     */
    uint16_t last_executed_sequence;
    /** Free slots in the SPI→planner multi-axis block queue (0–64). */
    uint8_t  multi_axis_queue_free;
    /** Free slots in the planner→executor segment queue (0–128).
     *  The host uses this to gate how many blocks it sends ahead. */
    uint8_t  planner_queue_free;
    /** Most recent motion_sequence successfully planned into segment_queue_. */
    uint16_t last_planned_sequence;
    /** Low 16 bits of planner dropped-segment counter for host diagnostics. */
    uint16_t segments_dropped;
};

static_assert(sizeof(StatusPayload) == 54, "StatusPayload must be 54 bytes");
static_assert(sizeof(StatusPayload) <= SPI_MAX_PAYLOAD_SIZE,
              "StatusPayload exceeds SPI_MAX_PAYLOAD_SIZE");

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
