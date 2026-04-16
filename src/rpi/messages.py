from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Iterable, List

SPI_MSG_MAGIC = 0x5057
SPI_MSG_VERSION = 1
SPI_FRAME_SIZE = 512
SPI_MAX_AXES = 2
STEP_BLOCK_SIZE = 64
SEGMENT_BLOCK_SIZE = 60

_HEADER_STRUCT = struct.Struct("<HBBHHHH")
_ENABLE_STRUCT = struct.Struct("<BB2x")
_ESTOP_STRUCT = struct.Struct("<B3x")
_STEP_BLOCK_HEAD_STRUCT = struct.Struct("<BBH")
_STEP_ENTRY_STRUCT = struct.Struct("<IB")
_SEGMENT_BLOCK_HEAD_STRUCT = struct.Struct("<BBH")
_SEGMENT_ENTRY_STRUCT = struct.Struct("<HHhBB")
_STATUS_STRUCT = struct.Struct("<IHHHHIIHBBBBB5x")


class SpiMessageType(IntEnum):
    NOP = 0x00
    ENABLE_AXIS = 0x01
    ESTOP = 0x02
    STOP_AXIS = 0x03
    DISABLE_ALL = 0x04
    RESET_STATS = 0x05
    GET_STATUS = 0x06
    STEP_BLOCK = 0x10
    SEGMENT_BLOCK = 0x11
    PING = 0x7F
    STATUS = 0x80


class SpiMessageResult(IntEnum):
    OK = 0x00
    BAD_MAGIC = 0x01
    BAD_VERSION = 0x02
    BAD_LENGTH = 0x03
    BAD_CRC = 0x04
    UNKNOWN_TYPE = 0x05
    BAD_AXIS = 0x06
    QUEUE_FULL = 0x07
    INTERNAL_ERROR = 0x08


class SpiStepFlags(IntEnum):
    NONE = 0x00
    DIR_REVERSE = 0x01


@dataclass(slots=True)
class MessageHeader:
    magic: int
    version: int
    msg_type: int
    sequence: int
    payload_length: int
    flags: int
    crc16: int = 0

    def pack(self) -> bytes:
        return _HEADER_STRUCT.pack(
            self.magic,
            self.version,
            self.msg_type,
            self.sequence,
            self.payload_length,
            self.flags,
            self.crc16,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "MessageHeader":
        return cls(*_HEADER_STRUCT.unpack(data[: _HEADER_STRUCT.size]))


@dataclass(slots=True)
class EnableAxisPayload:
    axis_id: int
    enable: bool

    def pack(self) -> bytes:
        return _ENABLE_STRUCT.pack(self.axis_id, 1 if self.enable else 0)


@dataclass(slots=True)
class EmergencyStopPayload:
    axis_id: int = 0xFF

    def pack(self) -> bytes:
        return _ESTOP_STRUCT.pack(self.axis_id)


@dataclass(slots=True)
class StepEntry:
    interval_ticks: int
    direction_reverse: bool = False

    def pack(self) -> bytes:
        flags = int(SpiStepFlags.DIR_REVERSE) if self.direction_reverse else int(SpiStepFlags.NONE)
        return _STEP_ENTRY_STRUCT.pack(self.interval_ticks, flags)


@dataclass(slots=True)
class MotionSegment:
    step_count: int
    start_ticks: int
    add_ticks: int
    direction_reverse: bool = False

    def pack(self) -> bytes:
        flags = int(SpiStepFlags.DIR_REVERSE) if self.direction_reverse else int(SpiStepFlags.NONE)
        return _SEGMENT_ENTRY_STRUCT.pack(self.step_count, self.start_ticks, self.add_ticks, flags, 0)


@dataclass(slots=True)
class StepBlockPayload:
    axis_id: int
    block_seq: int
    entries: List[StepEntry]

    def pack(self) -> bytes:
        if len(self.entries) > STEP_BLOCK_SIZE:
            raise ValueError(f"step block too large: {len(self.entries)} > {STEP_BLOCK_SIZE}")
        payload = bytearray()
        payload += _STEP_BLOCK_HEAD_STRUCT.pack(self.axis_id, self.block_seq, len(self.entries))
        for entry in self.entries:
            payload += entry.pack()
        for _ in range(STEP_BLOCK_SIZE - len(self.entries)):
            payload += _STEP_ENTRY_STRUCT.pack(0, 0)
        return bytes(payload)


@dataclass(slots=True)
class SegmentBlockPayload:
    axis_id: int
    block_seq: int
    segments: List[MotionSegment]

    def pack(self) -> bytes:
        if len(self.segments) > SEGMENT_BLOCK_SIZE:
            raise ValueError(f"segment block too large: {len(self.segments)} > {SEGMENT_BLOCK_SIZE}")
        payload = bytearray()
        payload += _SEGMENT_BLOCK_HEAD_STRUCT.pack(self.axis_id, self.block_seq, len(self.segments))
        for segment in self.segments:
            payload += segment.pack()
        for _ in range(SEGMENT_BLOCK_SIZE - len(self.segments)):
            payload += _SEGMENT_ENTRY_STRUCT.pack(0, 0, 0, 0, 0)
        return bytes(payload)


@dataclass(slots=True)
class StatusPayload:
    uptime_ms: int
    queue_free_slots: tuple[int, int]
    ring_free_slots: tuple[int, int]
    underrun_count: tuple[int, int]
    last_rx_sequence: int
    last_rx_type: int
    last_result: int
    protocol_version: int
    enabled_mask: int
    running_mask: int

    @classmethod
    def unpack(cls, payload: bytes) -> "StatusPayload":
        values = _STATUS_STRUCT.unpack(payload[: _STATUS_STRUCT.size])
        return cls(
            uptime_ms=values[0],
            queue_free_slots=(values[1], values[2]),
            ring_free_slots=(values[3], values[4]),
            underrun_count=(values[5], values[6]),
            last_rx_sequence=values[7],
            last_rx_type=values[8],
            last_result=values[9],
            protocol_version=values[10],
            enabled_mask=values[11],
            running_mask=values[12],
        )


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: SpiMessageType, payload: bytes = b"", *, sequence: int = 0, flags: int = 0) -> bytes:
    if len(payload) > SPI_FRAME_SIZE - _HEADER_STRUCT.size:
        raise ValueError("payload too large for one SPI frame")
    header = MessageHeader(
        magic=SPI_MSG_MAGIC,
        version=SPI_MSG_VERSION,
        msg_type=int(msg_type),
        sequence=sequence & 0xFFFF,
        payload_length=len(payload),
        flags=flags & 0xFFFF,
        crc16=0,
    )
    frame = bytearray(SPI_FRAME_SIZE)
    frame[: _HEADER_STRUCT.size] = header.pack()
    frame[_HEADER_STRUCT.size : _HEADER_STRUCT.size + len(payload)] = payload
    header.crc16 = crc16_ccitt(frame[: _HEADER_STRUCT.size + len(payload)])
    frame[: _HEADER_STRUCT.size] = header.pack()
    return bytes(frame)


def parse_status_frame(frame: bytes) -> StatusPayload:
    if len(frame) != SPI_FRAME_SIZE:
        raise ValueError(f"invalid SPI frame size: {len(frame)}")
    header = MessageHeader.unpack(frame)
    if header.magic != SPI_MSG_MAGIC:
        raise ValueError(f"bad magic: 0x{header.magic:04X}")
    if header.version != SPI_MSG_VERSION:
        raise ValueError(f"bad version: {header.version}")
    if header.msg_type != int(SpiMessageType.STATUS):
        raise ValueError(f"unexpected response type: 0x{header.msg_type:02X}")
    if header.payload_length > SPI_FRAME_SIZE - _HEADER_STRUCT.size:
        raise ValueError("invalid payload length in response")

    raw = bytearray(frame[: _HEADER_STRUCT.size + header.payload_length])
    raw[10:12] = b"\x00\x00"
    expected_crc = crc16_ccitt(bytes(raw))
    if expected_crc != header.crc16:
        raise ValueError(f"bad response CRC: expected 0x{expected_crc:04X}, got 0x{header.crc16:04X}")

    payload = frame[_HEADER_STRUCT.size : _HEADER_STRUCT.size + header.payload_length]
    return StatusPayload.unpack(payload)


def make_nop(sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.NOP, b"", sequence=sequence)


def make_get_status(sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.GET_STATUS, b"", sequence=sequence)


def make_enable_axis(axis_id: int, enable: bool, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.ENABLE_AXIS, EnableAxisPayload(axis_id, enable).pack(), sequence=sequence)


def make_estop(axis_id: int = 0xFF, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.ESTOP, EmergencyStopPayload(axis_id).pack(), sequence=sequence)


def make_stop_axis(axis_id: int = 0xFF, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.STOP_AXIS, EmergencyStopPayload(axis_id).pack(), sequence=sequence)


def make_disable_all(sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.DISABLE_ALL, b"", sequence=sequence)


def make_reset_stats(sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.RESET_STATS, b"", sequence=sequence)


def make_step_block(payload: StepBlockPayload, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.STEP_BLOCK, payload.pack(), sequence=sequence)


def make_segment_block(payload: SegmentBlockPayload, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.SEGMENT_BLOCK, payload.pack(), sequence=sequence)
