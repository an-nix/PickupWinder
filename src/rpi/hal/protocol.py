"""protocol.py — SPI binary protocol: command & status frames.

Mirrors esp32/src/protocol.h exactly.  All multi-byte fields little-endian.

Command frame (RPi → ESP32):  8 bytes
Status  frame (ESP32 → RPi): 32 bytes
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

# ── Frame sizes ──────────────────────────────────────────────────────────────
CMD_FRAME_SIZE = 8
STATUS_FRAME_SIZE = 44
FRAME_SIZE = STATUS_FRAME_SIZE   # SPI transfer size (full-duplex uses larger)

# ── Command opcodes ──────────────────────────────────────────────────────────

class CmdOpcode(IntEnum):
    NOP          = 0x00
    SET_SPEED    = 0x01
    MOVE_ABS     = 0x02
    MOVE_REL     = 0x03
    STOP         = 0x04
    ESTOP        = 0x05
    ENABLE       = 0x06
    HOME         = 0x07
    SET_ACCEL    = 0x08
    GET_STATUS   = 0x09
    SET_MODE     = 0x0A
    RESET_POS    = 0x0B
    SET_LIMITS   = 0x0C
    ACK_EVENT    = 0x0D
    SET_TENSION  = 0x0E  # data = tension setpoint in 0.1g units
    TARE_HX711   = 0x0F  # axis = sensor index (0 or 1)
    # Host-uploaded ramp segments
    UPLOAD_RAMP_START = 0x10  # data = segment count (uint32)
    UPLOAD_RAMP_SEG   = 0x11  # data = next 32-bit word for current segment (start_iv, add, count in sequence)
    UPLOAD_RAMP_COMMIT= 0x12  # data = target position (int32) to apply uploaded segments
    UPLOAD_RAMP_ABORT = 0x13  # abort current upload


class AxisId(IntEnum):
    BOBBIN    = 0
    LATERAL   = 1
    TENSIONER = 2
    ALL       = 0xFF


# ── Command flags ────────────────────────────────────────────────────────────

class CmdFlags:
    NONE        = 0x00
    DIR_REVERSE = 0x01
    LIMIT_MAX   = 0x02
    SYNC_AXES   = 0x04
    RAMP_ENABLE = 0x08


# ── Status flags ─────────────────────────────────────────────────────────────

class StatusFlags:
    ENABLED       = 0x01
    MOVING        = 0x02
    HOMING        = 0x04
    FAULT         = 0x08
    ENDSTOP_HIT   = 0x10
    MOVE_COMPLETE = 0x20
    SPEED_REACHED = 0x40
    EVENT_PENDING = 0x80


# ── Event types ──────────────────────────────────────────────────────────────

class EventType(IntEnum):
    NONE           = 0x00
    ENDSTOP_HIT    = 0x01
    HOME_COMPLETE  = 0x02
    FAULT          = 0x03
    LIMIT_HIT      = 0x04
    MOVE_COMPLETE  = 0x05
    SPEED_REACHED  = 0x06
    ENDSTOP_CLEAR  = 0x07


# ── CRC-8/MAXIM ─────────────────────────────────────────────────────────────
# Polynomial 0x31, init 0x00.  Identical to the ESP32 table.

_CRC8_TABLE: list[int] = [
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
]


def crc8(data: bytes) -> int:
    """Compute CRC-8/MAXIM over data bytes."""
    crc = 0x00
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


# ── Command frame encoding ──────────────────────────────────────────────────

# Frame format: cmd(1) axis(1) data(4 LE) flags(1) crc8(1)
_CMD_STRUCT = struct.Struct("<BBiBB")  # Note: data as signed i32 for flexibility


def encode_cmd(
    cmd: CmdOpcode,
    axis: AxisId = AxisId.ALL,
    data: int = 0,
    flags: int = CmdFlags.NONE,
) -> bytes:
    """Encode a command frame (8 bytes) with CRC.

    Args:
        cmd: Command opcode.
        axis: Target axis.
        data: 32-bit payload (signed or unsigned — bitwise identical).
        flags: Command flags bitfield.

    Returns:
        8-byte command frame ready to send over SPI.
    """
    # Pack without CRC (7 bytes), then compute CRC over those
    payload = struct.pack("<BBiB", int(cmd), int(axis), data, flags)
    c = crc8(payload)
    return payload + bytes([c])


def encode_cmd_u32(
    cmd: CmdOpcode,
    axis: AxisId = AxisId.ALL,
    data: int = 0,
    flags: int = CmdFlags.NONE,
) -> bytes:
    """Encode a command frame with unsigned 32-bit data."""
    payload = struct.pack("<BBIB", int(cmd), int(axis), data & 0xFFFFFFFF, flags)
    c = crc8(payload)
    return payload + bytes([c])


# ── Status frame decoding ───────────────────────────────────────────────────

@dataclass
class AxisStatus:
    """Per-axis status from ESP32."""
    position: int      # steps (signed)
    current_hz: int    # step frequency
    flags: int         # StatusFlags bitfield

    @property
    def is_enabled(self) -> bool:
        return bool(self.flags & StatusFlags.ENABLED)

    @property
    def is_moving(self) -> bool:
        return bool(self.flags & StatusFlags.MOVING)

    @property
    def is_homing(self) -> bool:
        return bool(self.flags & StatusFlags.HOMING)

    @property
    def has_fault(self) -> bool:
        return bool(self.flags & StatusFlags.FAULT)

    @property
    def endstop_hit(self) -> bool:
        return bool(self.flags & StatusFlags.ENDSTOP_HIT)

    @property
    def move_complete(self) -> bool:
        return bool(self.flags & StatusFlags.MOVE_COMPLETE)

    @property
    def speed_reached(self) -> bool:
        return bool(self.flags & StatusFlags.SPEED_REACHED)


@dataclass
class MachineStatus:
    """Full machine status from ESP32 (44-byte StatusFrame decoded)."""
    global_flags: int
    event_type: EventType
    event_axis: int
    endstop_mask: int
    uptime_ms: int
    axes: list[AxisStatus]   # [bobbin, lateral, tensioner]
    tension_raw: list[int]   # HX711 readings [0]=tension, [1]=aux in 0.1g units
    tension_setpoint: int    # Setpoint echoed from last SET_TENSION command
    pot_raw: int             # Potentiometer ADC value (0–4095, GPIO 36)
    encoder_manual: int      # Manual encoder PCNT count (int16, wrapping)

    @property
    def has_event(self) -> bool:
        return self.event_type != EventType.NONE

    @property
    def event_pending(self) -> bool:
        """Alias for has_event (matches StatusFlags.EVENT_PENDING)."""
        return bool(self.global_flags & StatusFlags.EVENT_PENDING)

    @property
    def any_moving(self) -> bool:
        return bool(self.global_flags & StatusFlags.MOVING)

    @property
    def any_fault(self) -> bool:
        return bool(self.global_flags & StatusFlags.FAULT)


# Status frame struct: global_flags(1) event_type(1) event_axis(1)
# endstop_mask(1) uptime_ms(4) + 3 × AxisStatus(8) + tension(6) + pot(2) + enc(2) + reserved(2)
_STATUS_HEADER = struct.Struct("<BBBBI")  # gf(B) et(B) ea(B) em(B) uptime(I) = 8 bytes
_AXIS_STATUS = struct.Struct("<iHBB")    # position(i32) hz(u16) flags(u8) pad(u8) = 8 bytes
_SENSOR_EXT = struct.Struct("<hhhhh2x")  # t0(h) t1(h) setpoint(h) pot(h) enc(h) + 2 pad = 12 bytes


def decode_status(data: bytes) -> Optional[MachineStatus]:
    """Decode a 44-byte status frame from ESP32.

    Returns None if data is too short.
    """
    if len(data) < STATUS_FRAME_SIZE:
        return None

    gf, et, ea, em, uptime = _STATUS_HEADER.unpack_from(data, 0)

    axes: list[AxisStatus] = []
    for i in range(3):
        offset = 8 + i * 8
        pos, hz, flags, _ = _AXIS_STATUS.unpack_from(data, offset)
        axes.append(AxisStatus(position=pos, current_hz=hz, flags=flags))

    # Sensor extension (bytes 32–43)
    t0, t1, setpoint, pot, enc = _SENSOR_EXT.unpack_from(data, 32)

    return MachineStatus(
        global_flags=gf,
        event_type=EventType(et),
        event_axis=ea,
        endstop_mask=em,
        uptime_ms=uptime,
        axes=axes,
        tension_raw=[t0, t1],
        tension_setpoint=setpoint,
        pot_raw=pot,
        encoder_manual=enc,
    )


# ── NOP frame (for status-only reads) ───────────────────────────────────────

NOP_FRAME = encode_cmd(CmdOpcode.NOP)
assert len(NOP_FRAME) == CMD_FRAME_SIZE
