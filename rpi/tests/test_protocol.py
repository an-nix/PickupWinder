"""test_protocol.py — Unit tests for the SPI binary protocol.

Tests CRC-8 computation, command encoding, status decoding,
and round-trip correctness.
"""

from __future__ import annotations

import struct
import pytest

from hal.protocol import (
    CmdOpcode,
    AxisId,
    CmdFlags,
    StatusFlags,
    EventType,
    crc8,
    encode_cmd,
    encode_cmd_u32,
    decode_status,
    AxisStatus,
    MachineStatus,
    FRAME_SIZE,
    CMD_FRAME_SIZE,
    STATUS_FRAME_SIZE,
    NOP_FRAME,
)


class TestCRC8:
    """CRC-8/MAXIM polynomial 0x31 tests."""

    def test_empty(self) -> None:
        assert crc8(b"") == 0x00

    def test_single_byte(self) -> None:
        # Known CRC-8/MAXIM value for single byte 0x01
        result = crc8(b"\x01")
        assert isinstance(result, int)
        assert 0 <= result <= 255

    def test_deterministic(self) -> None:
        data = b"\x01\x02\x03\x04\x05\x06\x07"
        assert crc8(data) == crc8(data)

    def test_different_data_different_crc(self) -> None:
        a = crc8(b"\x01\x02\x03")
        b = crc8(b"\x01\x02\x04")
        assert a != b

    def test_nop_frame_crc_valid(self) -> None:
        """NOP_FRAME should have valid CRC in last byte."""
        payload = NOP_FRAME[:7]
        expected_crc = crc8(payload)
        assert NOP_FRAME[7] == expected_crc


class TestEncodeCmd:
    """Command frame encoding tests."""

    def test_nop(self) -> None:
        frame = encode_cmd(CmdOpcode.NOP, AxisId.BOBBIN)
        assert len(frame) == CMD_FRAME_SIZE
        assert frame[0] == CmdOpcode.NOP
        assert frame[1] == AxisId.BOBBIN

    def test_enable_all(self) -> None:
        frame = encode_cmd(CmdOpcode.ENABLE, AxisId.ALL)
        assert frame[0] == CmdOpcode.ENABLE
        assert frame[1] == AxisId.ALL
        # Verify CRC
        assert frame[7] == crc8(frame[:7])

    def test_set_speed_with_data(self) -> None:
        hz = 6400
        frame = encode_cmd_u32(CmdOpcode.SET_SPEED, AxisId.BOBBIN, hz)
        assert frame[0] == CmdOpcode.SET_SPEED
        assert frame[1] == AxisId.BOBBIN
        assert struct.unpack_from("<I", frame, 2)[0] == hz

    def test_set_speed_with_flags(self) -> None:
        frame = encode_cmd_u32(
            CmdOpcode.SET_SPEED,
            AxisId.LATERAL,
            1000,
            CmdFlags.DIR_REVERSE,
        )
        assert frame[6] == CmdFlags.DIR_REVERSE

    def test_encode_u32_helper(self) -> None:
        frame = encode_cmd_u32(CmdOpcode.SET_SPEED, AxisId.BOBBIN, 12345)
        assert struct.unpack_from("<I", frame, 2)[0] == 12345

    def test_frame_size_always_8(self) -> None:
        """All command frames must be exactly CMD_FRAME_SIZE bytes."""
        frame = encode_cmd(CmdOpcode.ENABLE, AxisId.ALL)
        assert len(frame) == CMD_FRAME_SIZE

    def test_crc_last_byte(self) -> None:
        frame = encode_cmd_u32(CmdOpcode.SET_ACCEL, AxisId.BOBBIN, 100000)
        assert frame[7] == crc8(frame[:7])


class TestDecodeStatus:
    """StatusFrame decoding tests."""

    def _make_status_frame(
        self,
        global_flags: int = 0,
        event_type: int = 0,
        event_axis: int = 0,
        endstop_mask: int = 0,
        uptime_ms: int = 1000,
        axes: list[tuple[int, int, int]] | None = None,
        tension_raw: list[int] | None = None,
        tension_setpoint: int = 0,
        pot_raw: int = 0,
        encoder_manual: int = 0,
    ) -> bytes:
        """Build a raw 44-byte status frame for testing.

        axes is a list of (position, current_hz, flags) tuples.
        Matches protocol.h: header(8) + 3 × AxisStatus(8) + sensor_ext(12) = 44 bytes.
        """
        header = struct.pack(
            "<BBBBI",
            global_flags, event_type, event_axis, endstop_mask, uptime_ms,
        )
        axes_data = b""
        if axes is None:
            axes = [(0, 0, 0)] * 3
        for position, current_hz, flags in axes:
            axes_data += struct.pack("<iHBB", position, current_hz, flags, 0)
        if tension_raw is None:
            tension_raw = [0, 0]
        sensor_ext = struct.pack("<hhhhh2x",
                                  tension_raw[0], tension_raw[1], tension_setpoint,
                                  pot_raw, encoder_manual)
        frame = header + axes_data + sensor_ext
        assert len(frame) == STATUS_FRAME_SIZE
        return frame

    def test_decode_empty_status(self) -> None:
        frame = self._make_status_frame()
        status = decode_status(frame)
        assert status.global_flags == 0
        assert status.uptime_ms == 1000
        assert len(status.axes) == 3

    def test_decode_with_event(self) -> None:
        frame = self._make_status_frame(
            global_flags=StatusFlags.EVENT_PENDING,
            event_type=EventType.HOME_COMPLETE,
            event_axis=1,
        )
        status = decode_status(frame)
        assert status.event_pending
        assert status.event_type == EventType.HOME_COMPLETE
        assert status.event_axis == 1

    def test_decode_uptime(self) -> None:
        frame = self._make_status_frame(uptime_ms=42000)
        status = decode_status(frame)
        assert status.uptime_ms == 42000

    def test_decode_axis_position(self) -> None:
        frame = self._make_status_frame(
            axes=[
                (3072, 6400, StatusFlags.MOVING),   # Axis 0: position=3072, hz=6400
                (0, 0, 0),
                (0, 0, 0),
            ]
        )
        status = decode_status(frame)
        assert status.axes[0].position == 3072
        assert status.axes[0].current_hz == 6400
        assert status.axes[0].is_moving

    def test_decode_endstop_mask(self) -> None:
        frame = self._make_status_frame(endstop_mask=0x05)
        status = decode_status(frame)
        assert status.endstop_mask == 0x05

    def test_frame_size_is_44(self) -> None:
        assert STATUS_FRAME_SIZE == 44

    def test_frame_too_short_returns_none(self) -> None:
        result = decode_status(b"\x00" * 4)
        assert result is None

    def test_decode_tension_values(self) -> None:
        frame = self._make_status_frame(
            tension_raw=[150, 200],
            tension_setpoint=175,
        )
        status = decode_status(frame)
        assert status.tension_raw[0] == 150
        assert status.tension_raw[1] == 200
        assert status.tension_setpoint == 175

    def test_decode_pot_raw(self) -> None:
        frame = self._make_status_frame(pot_raw=2048)
        status = decode_status(frame)
        assert status.pot_raw == 2048

    def test_decode_pot_raw_max(self) -> None:
        frame = self._make_status_frame(pot_raw=4095)
        status = decode_status(frame)
        assert status.pot_raw == 4095

    def test_decode_pot_raw_zero(self) -> None:
        frame = self._make_status_frame(pot_raw=0)
        status = decode_status(frame)
        assert status.pot_raw == 0

    def test_decode_encoder_manual(self) -> None:
        frame = self._make_status_frame(encoder_manual=512)
        status = decode_status(frame)
        assert status.encoder_manual == 512

    def test_decode_encoder_manual_negative(self) -> None:
        """Encoder delta can be negative (counter-clockwise)."""
        frame = self._make_status_frame(encoder_manual=-300)
        status = decode_status(frame)
        assert status.encoder_manual == -300

    def test_decode_pot_and_encoder_independent(self) -> None:
        """pot_raw and encoder_manual are decoded independently."""
        frame = self._make_status_frame(pot_raw=1000, encoder_manual=-50)
        status = decode_status(frame)
        assert status.pot_raw == 1000
        assert status.encoder_manual == -50


class TestRoundTrip:
    """Ensure encode → mock → decode round-trips correctly."""

    def test_enable_roundtrip(self) -> None:
        from tests.mock_esp32 import MockSpiTransport
        mock = MockSpiTransport()
        mock.open()

        cmd = encode_cmd_u32(CmdOpcode.ENABLE, AxisId.ALL, 1)
        status = mock.send_command(cmd)
        assert status.global_flags & StatusFlags.ENABLED

    def test_set_speed_roundtrip(self) -> None:
        from tests.mock_esp32 import MockSpiTransport
        mock = MockSpiTransport()
        mock.open()

        cmd_en = encode_cmd_u32(CmdOpcode.ENABLE, AxisId.BOBBIN, 1)
        mock.send_command(cmd_en)

        cmd_spd = encode_cmd_u32(CmdOpcode.SET_SPEED, AxisId.BOBBIN, 6400)
        status = mock.send_command(cmd_spd)
        assert status.axes[0].current_hz == 6400

    def test_estop_roundtrip(self) -> None:
        from tests.mock_esp32 import MockSpiTransport
        mock = MockSpiTransport()
        mock.open()

        cmd_en = encode_cmd_u32(CmdOpcode.ENABLE, AxisId.ALL, 1)
        mock.send_command(cmd_en)

        cmd_spd = encode_cmd_u32(CmdOpcode.SET_SPEED, AxisId.BOBBIN, 5000)
        mock.send_command(cmd_spd)

        cmd_estop = encode_cmd(CmdOpcode.ESTOP)
        status = mock.send_command(cmd_estop)
        assert status.axes[0].current_hz == 0

    def test_pot_raw_roundtrip(self) -> None:
        """pot_raw set on mock is reflected in decoded status."""
        from tests.mock_esp32 import MockSpiTransport
        mock = MockSpiTransport()
        mock.open()
        mock.set_pot_reading(3500)

        cmd = encode_cmd(CmdOpcode.NOP)
        status = mock.send_command(cmd)
        assert status.pot_raw == 3500

    def test_encoder_manual_roundtrip(self) -> None:
        """encoder_manual set on mock is reflected in decoded status."""
        from tests.mock_esp32 import MockSpiTransport
        mock = MockSpiTransport()
        mock.open()
        mock.set_encoder_count(-128)

        cmd = encode_cmd(CmdOpcode.NOP)
        status = mock.send_command(cmd)
        assert status.encoder_manual == -128

    def test_sensor_fields_default_zero(self) -> None:
        """pot_raw and encoder_manual default to 0 on a fresh mock."""
        from tests.mock_esp32 import MockSpiTransport
        mock = MockSpiTransport()
        mock.open()

        cmd = encode_cmd(CmdOpcode.NOP)
        status = mock.send_command(cmd)
        assert status.pot_raw == 0
        assert status.encoder_manual == 0
