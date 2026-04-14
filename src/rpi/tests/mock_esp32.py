"""mock_esp32.py — SPI mock that simulates ESP32 responses.

Provides a drop-in replacement for SpiTransport that can be used
in tests and --dry-run mode without any hardware.

The mock maintains internal axis state and responds to commands
with realistic StatusFrames, including:
- Position tracking (increments based on speed)
- Speed changes (immediate)
- Enable/disable state
- Event generation (home_complete, move_complete, endstop_hit)

Interface contract: MockSpiTransport has the same public methods as
SpiTransport — open(), close(), transfer(), send_command(), get_status().
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Optional

from hal.protocol import (
    CmdOpcode,
    AxisId,
    CmdFlags,
    StatusFlags,
    EventType,
    crc8,
    decode_status,
    MachineStatus,
    FRAME_SIZE,
    STATUS_FRAME_SIZE,
    CMD_FRAME_SIZE,
)


class MockAxisState:
    """Internal state for one simulated axis."""

    def __init__(self, axis_id: int) -> None:
        self.axis_id = axis_id
        self.enabled = False
        self.position: int = 0
        self.current_hz: int = 0
        self.flags: int = 0          # StatusFlags bitfield per-axis
        self.target_position: Optional[int] = None
        self.direction: int = 0      # 0 = forward, 1 = reverse
        self.endstop_active = False


class MockSpiTransport:
    """Drop-in replacement for SpiTransport using simulated ESP32.

    Thread-safe: can be called from asyncio.to_thread() just like
    the real SpiTransport.

    Usage:
        transport = MockSpiTransport()
        transport.open()  # no-op but keeps interface consistent
        status = transport.send_command(encoded_cmd_bytes)
    """

    def __init__(self, speed_hz: int = 4_000_000, mode: int = 0) -> None:
        self._lock = threading.Lock()
        self._opened = False
        self._axes = [MockAxisState(i) for i in range(3)]
        self._global_flags: int = 0
        self._event_type: int = EventType.NONE
        self._event_axis: int = 0
        self._endstop_mask: int = 0
        self._start_time = time.monotonic()
        self._pending_events: list[tuple[int, int]] = []
        self._tension_raw: list[int] = [0, 0]   # 0.1g units
        self._tension_setpoint: int = 0
        self._pot_raw: int = 0          # potentiometer ADC value (0–4095)
        self._encoder_manual: int = 0   # manual encoder PCNT count (int16, wrapping)

    def open(self, bus: int = 0, device: int = 0) -> None:
        """Open mock transport (no-op, matches SpiTransport.open)."""
        self._opened = True
        self._start_time = time.monotonic()

    def close(self) -> None:
        self._opened = False

    @property
    def is_open(self) -> bool:
        return self._opened

    def __enter__(self) -> MockSpiTransport:
        self.open()
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def transfer(self, cmd: bytes) -> bytes:
        """Full-duplex SPI transfer simulation.

        Matches SpiTransport.transfer(): takes padded bytes, returns
        STATUS_FRAME_SIZE raw bytes.
        """
        with self._lock:
            # Parse first 8 bytes as CmdFrame
            if len(cmd) >= CMD_FRAME_SIZE:
                self._process_command(cmd[:CMD_FRAME_SIZE])
            return self._build_status()

    def send_command(self, cmd: bytes) -> MachineStatus:
        """Send command and decode status.

        Matches SpiTransport.send_command(cmd: bytes) -> MachineStatus.
        """
        # Pad to FRAME_SIZE like real transport
        tx = bytearray(FRAME_SIZE)
        tx[:len(cmd)] = cmd[:FRAME_SIZE]
        raw = self.transfer(bytes(tx))
        status = decode_status(raw)
        if status is None:
            raise RuntimeError("Failed to decode mock status frame")
        return status

    def get_status(self) -> MachineStatus:
        """Send NOP and read status. Matches SpiTransport.get_status()."""
        from hal.protocol import NOP_FRAME
        return self.send_command(NOP_FRAME)

    # ── Simulation helpers (test-only) ───────────────────────────────────────

    def inject_event(self, event_type: int, axis: int) -> None:
        """Inject a pending event (for test scenarios)."""
        with self._lock:
            self._pending_events.append((event_type, axis))

    def set_endstop(self, axis: int, active: bool) -> None:
        """Simulate endstop activation."""
        with self._lock:
            if 0 <= axis < 3:
                self._axes[axis].endstop_active = active
                if active:
                    self._endstop_mask |= (1 << axis)
                else:
                    self._endstop_mask &= ~(1 << axis)

    # ── Internal ─────────────────────────────────────────────────────────────

    def _process_command(self, frame: bytes) -> None:
        """Parse and execute an 8-byte CmdFrame."""
        if len(frame) < CMD_FRAME_SIZE:
            return

        cmd = frame[0]
        axis = frame[1]
        data = frame[2:6]
        flags = frame[6]
        rx_crc = frame[7]

        # Validate CRC
        expected_crc = crc8(frame[:7])
        if rx_crc != expected_crc:
            return  # silently ignore bad CRC (like real ESP32)

        if cmd == CmdOpcode.NOP:
            return

        elif cmd == CmdOpcode.ENABLE:
            val = struct.unpack_from("<I", data, 0)[0]
            enable = val != 0
            self._for_axes(axis, lambda ax: self._set_enabled(ax, enable))
            if any(ax.enabled for ax in self._axes):
                self._global_flags |= StatusFlags.ENABLED
            else:
                self._global_flags &= ~StatusFlags.ENABLED

        elif cmd == CmdOpcode.SET_SPEED:
            hz = struct.unpack_from("<I", data, 0)[0]
            reverse = bool(flags & CmdFlags.DIR_REVERSE)
            if 0 <= axis < 3:
                ax = self._axes[axis]
                ax.current_hz = hz
                ax.direction = 1 if reverse else 0
                if hz > 0:
                    ax.flags |= StatusFlags.MOVING
                else:
                    ax.flags &= ~StatusFlags.MOVING

        elif cmd == CmdOpcode.STOP:
            self._for_axes(axis, self._stop_axis)

        elif cmd == CmdOpcode.ESTOP:
            for ax in self._axes:
                self._stop_axis(ax)
            self._global_flags |= StatusFlags.ENDSTOP_HIT  # abuse as estop flag

        elif cmd == CmdOpcode.HOME:
            if 0 <= axis < 3:
                ax = self._axes[axis]
                ax.position = 0
                ax.flags &= ~StatusFlags.MOVING
                self._pending_events.append((EventType.HOME_COMPLETE, axis))

        elif cmd == CmdOpcode.MOVE_ABS:
            target = struct.unpack_from("<i", data, 0)[0]
            if 0 <= axis < 3:
                ax = self._axes[axis]
                ax.position = target
                ax.target_position = None
                ax.flags &= ~StatusFlags.MOVING
                self._pending_events.append((EventType.MOVE_COMPLETE, axis))

        elif cmd == CmdOpcode.MOVE_REL:
            delta = struct.unpack_from("<i", data, 0)[0]
            if 0 <= axis < 3:
                ax = self._axes[axis]
                ax.position += delta
                ax.flags &= ~StatusFlags.MOVING
                self._pending_events.append((EventType.MOVE_COMPLETE, axis))

        elif cmd == CmdOpcode.SET_ACCEL:
            pass  # Accept silently

        elif cmd == CmdOpcode.SET_MODE:
            mode = struct.unpack_from("<I", data, 0)[0]
            if mode & 0x01:
                self._global_flags |= StatusFlags.HOMING  # reuse as winding flag
            else:
                self._global_flags &= ~StatusFlags.HOMING

        elif cmd == CmdOpcode.SET_LIMITS:
            pass  # Accept silently

        elif cmd == CmdOpcode.RESET_POS:
            self._for_axes(axis, lambda ax: setattr(ax, 'position', 0))

        elif cmd == CmdOpcode.ACK_EVENT:
            self._event_type = EventType.NONE
            self._event_axis = 0
            self._global_flags &= ~StatusFlags.EVENT_PENDING

        elif cmd == CmdOpcode.SET_TENSION:
            val = struct.unpack_from("<H", data, 0)[0]
            self._tension_setpoint = val

        elif cmd == CmdOpcode.TARE_HX711:
            # Tare resets the reading to 0 for the named sensor
            if 0 <= axis < 2:
                self._tension_raw[axis] = 0

    def _set_enabled(self, ax: MockAxisState, enabled: bool) -> None:
        ax.enabled = enabled
        if enabled:
            ax.flags |= StatusFlags.ENABLED
        else:
            ax.flags &= ~StatusFlags.ENABLED

    def _stop_axis(self, ax: MockAxisState) -> None:
        ax.current_hz = 0
        ax.target_position = None
        ax.flags &= ~StatusFlags.MOVING

    def _for_axes(self, axis: int, func: object) -> None:
        """Apply a function to one or all axes."""
        if axis == AxisId.ALL:
            for ax in self._axes:
                func(ax)  # type: ignore[operator]
        elif 0 <= axis < 3:
            func(self._axes[axis])  # type: ignore[operator]

    def _build_status(self) -> bytes:
        """Build a 44-byte StatusFrame matching protocol.h layout.

        Header (8 bytes):
          global_flags(B) event_type(B) event_axis(B) endstop_mask(B) uptime_ms(I)
        Per axis (8 bytes × 3):
          position(i32) current_hz(u16) flags(u8) pad(u8)
        Sensor extension (12 bytes):
          tension_raw[0](h) tension_raw[1](h) tension_setpoint(h)
          pot_raw(h) encoder_manual(h) reserved(2x)
        """
        # Promote pending events
        if self._pending_events and self._event_type == EventType.NONE:
            ev_type, ev_axis = self._pending_events.pop(0)
            self._event_type = ev_type
            self._event_axis = ev_axis
            self._global_flags |= StatusFlags.EVENT_PENDING

        if self._event_type == EventType.NONE:
            self._global_flags &= ~StatusFlags.EVENT_PENDING

        # Update global MOVING flag
        if any(ax.flags & StatusFlags.MOVING for ax in self._axes):
            self._global_flags |= StatusFlags.MOVING
        else:
            self._global_flags &= ~StatusFlags.MOVING

        uptime = int((time.monotonic() - self._start_time) * 1000) & 0xFFFFFFFF

        # Header: BBBBI = 8 bytes
        header = struct.pack(
            "<BBBBI",
            self._global_flags & 0xFF,
            self._event_type & 0xFF,
            self._event_axis & 0xFF,
            self._endstop_mask & 0xFF,
            uptime,
        )

        # 3 × AxisStatus: iHBB = 8 bytes each = 24 bytes
        axes_data = b""
        for ax in self._axes:
            axes_data += struct.pack(
                "<iHBB",
                ax.position,
                ax.current_hz & 0xFFFF,
                ax.flags & 0xFF,
                0,  # padding
            )

        # Sensor extension: hhhhh 2x = 12 bytes (all signed int16)
        sensor_ext = struct.pack(
            "<hhhhh2x",
            self._tension_raw[0],
            self._tension_raw[1],
            self._tension_setpoint,
            self._pot_raw,
            self._encoder_manual,
        )

        frame = header + axes_data + sensor_ext
        assert len(frame) == STATUS_FRAME_SIZE, (
            f"Status frame is {len(frame)} bytes, expected {STATUS_FRAME_SIZE}"
        )
        return frame

    def set_tension_reading(self, sensor: int, value_dg: int) -> None:
        """Inject a tension reading for test scenarios (0.1g units)."""
        with self._lock:
            if 0 <= sensor < 2:
                self._tension_raw[sensor] = value_dg

    def set_pot_reading(self, value: int) -> None:
        """Inject a potentiometer ADC reading (0–4095)."""
        with self._lock:
            self._pot_raw = max(0, min(4095, value))

    def set_encoder_count(self, count: int) -> None:
        """Inject a manual encoder count (signed int16)."""
        with self._lock:
            # Clamp to int16 signed range to match int16_t in StatusFrame
            self._encoder_manual = max(-32768, min(32767, count))
