/* protocol.h — SPI binary protocol: command & status frames.
 *
 * Fixed-size frames for RPi (master) ↔ ESP32 (slave) communication.
 * All multi-byte fields are little-endian (native on both ARM & Xtensa).
 *
 * Command frame (RPi → ESP32):  8 bytes
 * Status  frame (ESP32 → RPi): 44 bytes
 *
 * Wire format:
 *   Every SPI transfer is STATUS_FRAME_SIZE (44) bytes long.
 *   The RPi writes CMD_FRAME_SIZE (8) bytes of command followed by
 *   STATUS_FRAME_SIZE - CMD_FRAME_SIZE (36) bytes of padding (zeroes).
 *   The ESP32 clocks out a 44-byte StatusFrame simultaneously.
 *
 * CRC:
 *   CRC-8/MAXIM (Dallas/iButton) over bytes 0..6 of each CmdFrame.
 *   Polynomial 0x31, init 0x00.  Frames with mismatched CRC are dropped.
 *
 * Endianness:
 *   All multi-byte integers are little-endian.  Both ARM (Raspberry Pi)
 *   and Xtensa LX6 (ESP32) are natively little-endian, so no byte-swap.
 */

#pragma once

#include <cstdint>
#include <cstddef>

// ── Frame sizes ─────────────────────────────────────────────────────────────
static constexpr size_t CMD_FRAME_SIZE    = 8;
static constexpr size_t STATUS_FRAME_SIZE = 44;

// ── Command opcodes ─────────────────────────────────────────────────────────
enum class CmdOpcode : uint8_t {
    NOP          = 0x00,
    SET_SPEED    = 0x01,   // data = target Hz (uint32_t)
    MOVE_ABS     = 0x02,   // data = absolute position (int32_t, steps)
    MOVE_REL     = 0x03,   // data = relative distance  (int32_t, steps)
    STOP         = 0x04,   // controlled deceleration stop (axis-specific)
    ESTOP        = 0x05,   // immediate all-axis emergency stop
    ENABLE       = 0x06,   // data[0] = 1 enable / 0 disable
    HOME         = 0x07,   // start homing sequence for axis
    SET_ACCEL    = 0x08,   // data = accel in steps/s² (uint32_t)
    GET_STATUS   = 0x09,   // request full status frame on next transfer
    SET_MODE     = 0x0A,   // data[0] = mode (0=free, 1=winding)
    RESET_POS    = 0x0B,   // reset position counters
    SET_LIMITS   = 0x0C,   // data = limit value (int32_t), flags[0] = min/max
    ACK_EVENT    = 0x0D,   // acknowledge pending event
    SET_TENSION  = 0x0E,   // data = tension setpoint in 0.1g units (uint16_t packed in uint32_t)
    TARE_HX711   = 0x0F,   // axis = sensor index (0 or 1), tare (zero) the load cell
    // Host-uploaded ramp segments
    UPLOAD_RAMP_START = 0x10, // data = segment count (uint32)
    UPLOAD_RAMP_SEG   = 0x11, // data = next 32-bit word for current segment (start_iv, add, count in sequence)
    UPLOAD_RAMP_COMMIT= 0x12, // data = target position (int32) to apply uploaded segments
    UPLOAD_RAMP_ABORT = 0x13, // abort current upload
};

// ── Axis IDs ────────────────────────────────────────────────────────────────
enum class AxisId : uint8_t {
    BOBBIN    = 0,   // Axis 0 — bobbin rotation (spindle)
    LATERAL   = 1,   // Axis 1 — lateral carriage
    TENSIONER = 2,   // Axis 2 — wire tensioner
    ALL       = 0xFF // broadcast to all axes
};

// ── Command flags (bitfield) ────────────────────────────────────────────────
namespace CmdFlags {
    static constexpr uint8_t NONE        = 0x00;
    static constexpr uint8_t DIR_REVERSE = 0x01;  // direction: 0=forward, 1=reverse
    static constexpr uint8_t LIMIT_MAX   = 0x02;  // for SET_LIMITS: 0=min, 1=max
    static constexpr uint8_t SYNC_AXES   = 0x04;  // synchronize axis with master
    static constexpr uint8_t RAMP_ENABLE = 0x08;  // use acceleration ramp
}

// ── Status flags (bitfield) ─────────────────────────────────────────────────
namespace StatusFlags {
    static constexpr uint8_t ENABLED       = 0x01;
    static constexpr uint8_t MOVING        = 0x02;
    static constexpr uint8_t HOMING        = 0x04;
    static constexpr uint8_t FAULT         = 0x08;
    static constexpr uint8_t ENDSTOP_HIT   = 0x10;
    static constexpr uint8_t MOVE_COMPLETE = 0x20;
    static constexpr uint8_t SPEED_REACHED = 0x40;
    static constexpr uint8_t EVENT_PENDING = 0x80;
}

// ── Event types ─────────────────────────────────────────────────────────────
enum class EventType : uint8_t {
    NONE           = 0x00,
    ENDSTOP_HIT    = 0x01,
    HOME_COMPLETE  = 0x02,
    FAULT          = 0x03,
    LIMIT_HIT      = 0x04,
    MOVE_COMPLETE  = 0x05,
    SPEED_REACHED  = 0x06,
    ENDSTOP_CLEAR  = 0x07,
};

// ── Command frame (8 bytes, RPi → ESP32) ────────────────────────────────────
//
//   Byte  Field     Description
//   ─────────────────────────────────────
//   0     cmd       CmdOpcode
//   1     axis      AxisId
//   2..5  data      uint32_t / int32_t (little-endian)
//   6     flags     CmdFlags bitfield
//   7     crc8      CRC-8/MAXIM over bytes 0..6
//
struct __attribute__((packed)) CmdFrame {
    uint8_t  cmd;       // CmdOpcode
    uint8_t  axis;      // AxisId
    uint8_t  data[4];   // payload (little-endian u32/i32)
    uint8_t  flags;
    uint8_t  crc8;

    // ── Helpers ─────────────────────────────────────────────────────────────
    void set_data_u32(uint32_t v) {
        data[0] = (v      ) & 0xFF;
        data[1] = (v >>  8) & 0xFF;
        data[2] = (v >> 16) & 0xFF;
        data[3] = (v >> 24) & 0xFF;
    }

    void set_data_i32(int32_t v) {
        set_data_u32(static_cast<uint32_t>(v));
    }

    uint32_t get_data_u32() const {
        return static_cast<uint32_t>(data[0])
             | (static_cast<uint32_t>(data[1]) <<  8)
             | (static_cast<uint32_t>(data[2]) << 16)
             | (static_cast<uint32_t>(data[3]) << 24);
    }

    int32_t get_data_i32() const {
        return static_cast<int32_t>(get_data_u32());
    }
};

static_assert(sizeof(CmdFrame) == CMD_FRAME_SIZE, "CmdFrame must be 8 bytes");

// ── Per-axis status (8 bytes) ───────────────────────────────────────────────
struct __attribute__((packed)) AxisStatus {
    int32_t  position;     // current position in steps
    uint16_t current_hz;   // current speed in Hz (0 = stopped)
    uint8_t  flags;        // StatusFlags bitfield
    uint8_t  _pad;
};

static_assert(sizeof(AxisStatus) == 8, "AxisStatus must be 8 bytes");

// ── Status frame (44 bytes, ESP32 → RPi) ────────────────────────────────────
//
//   Byte   Field
//   ─────────────────────────────────────
//   0      global_flags   StatusFlags
//   1      event_type     EventType
//   2      event_axis     AxisId
//   3      endstop_mask   bit per endstop
//   4..7   uptime_ms      milliseconds since boot (uint32_t LE)
//   8..15  axis[0]        AxisStatus (bobbin)
//  16..23  axis[1]        AxisStatus (lateral)
//  24..31  axis[2]        AxisStatus (tensioner)
//  32..33  tension_raw[0] HX711 #0 reading in 0.1g units (int16_t LE)
//  34..35  tension_raw[1] HX711 #1 reading in 0.1g units (int16_t LE)
//  36..37  tension_setpoint Active setpoint in 0.1g units (int16_t LE)
//  38..39  pot_raw        Potentiometer ADC value 0-4095 (int16_t LE)
//  40..41  encoder_manual Manual encoder PCNT count (int16_t LE, wrapping)
//  42..43  reserved       (2 bytes, set to 0)
//
struct __attribute__((packed)) StatusFrame {
    uint8_t    global_flags;
    uint8_t    event_type;         // EventType
    uint8_t    event_axis;         // AxisId
    uint8_t    endstop_mask;
    uint32_t   uptime_ms;
    AxisStatus axis[3];
    int16_t    tension_raw[2];     // HX711 readings: [0]=tension, [1]=aux (0.1g)
    int16_t    tension_setpoint;   // Active setpoint (0.1g, echoed from SET_TENSION)
    int16_t    pot_raw;            // Potentiometer ADC value (0–4095)
    int16_t    encoder_manual;     // Manual encoder PCNT count (wrapping int16)
    uint8_t    reserved[2];
};

static_assert(sizeof(StatusFrame) == STATUS_FRAME_SIZE, "StatusFrame must be 44 bytes");

// ── CRC-8/MAXIM (Dallas/iButton) ───────────────────────────────────────────
// Polynomial 0x31, init 0x00, no reflect, no xor-out.
//
// Table-based for speed; ~256 bytes flash is negligible on ESP32.
//
namespace crc8 {

inline uint8_t compute(const uint8_t* data, size_t len) {
    // CRC-8/MAXIM lookup table (polynomial 0x31)
    static constexpr uint8_t table[256] = {
        0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
        0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
        0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
        0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
        0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
        0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
        0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
        0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
        0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
        0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
        0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
        0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
        0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
        0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
        0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
        0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
        0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
        0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
        0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
        0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
        0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
        0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
        0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
        0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
        0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
        0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
        0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
        0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
        0xB0, 0x81, 0xD2, 0xE3, 0x74, 0x45, 0x16, 0x27,
        0x09, 0x38, 0x6B, 0x5A, 0xCD, 0xFC, 0xAF, 0x9E,
        0xF3, 0xC2, 0x91, 0xA0, 0x37, 0x06, 0x55, 0x64,
        0x4A, 0x7B, 0x28, 0x19, 0x8E, 0xBF, 0xEC, 0xDD,
    };
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc = table[crc ^ data[i]];
    }
    return crc;
}

} // namespace crc8

// ── Protocol helpers ────────────────────────────────────────────────────────

/// Stamp CRC into a command frame (bytes 0..6 → byte 7).
inline void cmd_frame_stamp_crc(CmdFrame& f) {
    f.crc8 = crc8::compute(reinterpret_cast<const uint8_t*>(&f), CMD_FRAME_SIZE - 1);
}

/// Validate CRC of a received command frame.  Returns true if valid.
inline bool cmd_frame_check_crc(const CmdFrame& f) {
    uint8_t expected = crc8::compute(reinterpret_cast<const uint8_t*>(&f), CMD_FRAME_SIZE - 1);
    return expected == f.crc8;
}

/// Build a command frame.
inline CmdFrame make_cmd(CmdOpcode cmd, AxisId axis, uint32_t data, uint8_t flags = CmdFlags::NONE) {
    CmdFrame f{};
    f.cmd   = static_cast<uint8_t>(cmd);
    f.axis  = static_cast<uint8_t>(axis);
    f.flags = flags;
    f.set_data_u32(data);
    cmd_frame_stamp_crc(f);
    return f;
}
