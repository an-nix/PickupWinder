"""spi_transport.py — Low-level SPI communication via spidev.

Thread-safe SPI master driver.  All ESP32 communication goes through
this single module.

MIGRATION: Replaces the Unix domain socket + JSON protocol used on BBB.
The BBB daemon mapped PRU shared RAM; here we use SPI full-duplex frames.

Usage:
    transport = SpiTransport()      # uses defaults
    transport.open()
    status = transport.transfer(cmd_bytes)
    transport.close()
"""

from __future__ import annotations

import threading
from typing import Optional

from .protocol import (
    CMD_FRAME_SIZE,
    STATUS_FRAME_SIZE,
    MachineStatus,
    NOP_FRAME,
    decode_status,
)

try:
    import spidev  # type: ignore[import-untyped]
    _HAS_SPIDEV = True
except ImportError:
    _HAS_SPIDEV = False


class SpiTransport:
    """Thread-safe SPI master for RPi ↔ ESP32 communication.

    Full-duplex transfers: sends CMD_FRAME (8 bytes) padded to
    STATUS_FRAME_SIZE (32 bytes), receives STATUS_FRAME simultaneously.

    Args:
        bus: SPI bus number (0 for /dev/spidev0.x).
        device: SPI chip-select device (0 or 1).
        speed_hz: SPI clock frequency.  ESP32 SPI slave supports up to
                  10 MHz reliably.  Default 4 MHz for safety.
        mode: SPI mode (0 = CPOL=0, CPHA=0 — must match ESP32).
    """

    def __init__(
        self,
        bus: int = 0,
        device: int = 0,
        speed_hz: int = 4_000_000,
        mode: int = 0,
    ) -> None:
        self._bus = bus
        self._device = device
        self._speed_hz = speed_hz
        self._mode = mode
        self._spi: Optional[spidev.SpiDev] = None  # type: ignore[name-defined]
        self._lock = threading.Lock()

    def open(self) -> None:
        """Open SPI device.  Raises RuntimeError if spidev not available."""
        if not _HAS_SPIDEV:
            raise RuntimeError(
                "spidev not installed. Run: pip install spidev"
            )
        spi = spidev.SpiDev()
        spi.open(self._bus, self._device)
        spi.max_speed_hz = self._speed_hz
        spi.mode = self._mode
        spi.bits_per_word = 8
        spi.no_cs = False
        self._spi = spi

    def close(self) -> None:
        """Close SPI device."""
        with self._lock:
            if self._spi is not None:
                self._spi.close()
                self._spi = None

    @property
    def is_open(self) -> bool:
        return self._spi is not None

    def transfer(self, cmd: bytes) -> bytes:
        """Full-duplex SPI transfer.

        Sends *cmd* (padded to STATUS_FRAME_SIZE) and returns the
        STATUS_FRAME_SIZE bytes received simultaneously.

        Thread-safe (locked).

        Args:
            cmd: Command frame bytes (CMD_FRAME_SIZE).

        Returns:
            Raw status frame bytes (STATUS_FRAME_SIZE).

        Raises:
            RuntimeError: If SPI device is not open.
        """
        if self._spi is None:
            raise RuntimeError("SPI device not open")

        # Pad command to match full-duplex frame size
        tx = bytearray(STATUS_FRAME_SIZE)
        tx[: len(cmd)] = cmd

        with self._lock:
            rx = self._spi.xfer2(list(tx))

        return bytes(rx)

    def send_command(self, cmd: bytes) -> MachineStatus:
        """Send a command and decode the status response.

        Convenience method: transfer + decode in one call.

        Args:
            cmd: Encoded command frame (from protocol.encode_cmd).

        Returns:
            Decoded MachineStatus.

        Raises:
            RuntimeError: If response cannot be decoded.
        """
        raw = self.transfer(cmd)
        status = decode_status(raw)
        if status is None:
            raise RuntimeError(
                f"Failed to decode status frame ({len(raw)} bytes)"
            )
        return status

    def get_status(self) -> MachineStatus:
        """Send a NOP command and return current status.

        This is the standard way to poll machine state without
        changing anything.
        """
        return self.send_command(NOP_FRAME)

    def __enter__(self) -> "SpiTransport":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
