from __future__ import annotations

import logging
import re
import threading
import time


from transport.messages import (
    SPI_FRAME_SIZE,
    SpiMessageType,
    SpiMessageResult,
    StatusPayload,
    MultiAxisSegmentBlockPayload,
    SegmentBlockPayload,
    StepBlockPayload,
    FlushPayload,
    EnableEndstopPayload,
    make_disable_all,
    make_enable_axis,
    make_estop,
    make_flush,
    make_enable_endstop,
    make_get_status,
    make_reset_stats,
    make_segment_block,
    make_step_block,
    make_stop_axis,
    make_multi_axis_segment_block,
    parse_status_frame,
)


logger = logging.getLogger(__name__)


class Esp32SpiTransport:
    """Thin wrapper around spidev using the PickupWinder fixed SPI frame format."""

    def __init__(
        self,
        bus: int | None = None,
        device: int | None = None,
        *,
        device_path: str | None = None,
        speed_hz: int = 1_000_000,
        mode: int = 0,
    ):
        try:
            import spidev  # type: ignore
        except ImportError as exc:  # pragma: no cover - depends on host machine
            raise RuntimeError("spidev module is required on the Raspberry Pi host") from exc

        self._spidev_module = spidev
        self._spi = spidev.SpiDev()

        # Persist open parameters so we can recover from a stuck/broken SPI fd.
        self._open_bus = bus
        self._open_device = device
        self._open_device_path = device_path
        self._speed_hz = speed_hz
        self._mode = mode
        self._device_path = None

        self._open_spi()
        self._sequence = 0
        self._consecutive_zero_frames = 0
        self._io_lock = threading.Lock()
        self._last_status: StatusPayload | None = None
        self._last_status_ts: float = 0.0
        self._last_stale_log_ts: float = 0.0
        self._diag_total_xfers: int = 0
        self._diag_bad_magic: int = 0
        self._diag_bad_crc: int = 0
        self._diag_zero_rx: int = 0
        self._diag_reopens: int = 0
        self._diag_last_log_ts: float = time.monotonic()

    def close(self) -> None:
        self._spi.close()

    def _open_spi(self) -> None:
        if self._open_device_path is not None:
            self._device_path = self._open_device_path
            if hasattr(self._spi, "open_path"):
                self._spi.open_path(self._open_device_path)
            else:
                parsed = re.fullmatch(r"/dev/spidev(\d+)\.(\d+)", self._open_device_path)
                if parsed is None:
                    raise ValueError(
                        "device_path must be in the form /dev/spidev<bus>.<device>"
                    )
                self._spi.open(int(parsed.group(1)), int(parsed.group(2)))
        elif self._open_bus is not None and self._open_device is not None:
            self._device_path = f"/dev/spidev{self._open_bus}.{self._open_device}"
            self._spi.open(self._open_bus, self._open_device)
        else:
            raise ValueError(
                "Must specify either bus/device or device_path for SPI transport"
            )

        self._spi.max_speed_hz = self._speed_hz
        self._spi.mode = self._mode
        # Force stable SPI settings explicitly (avoid driver defaults drift).
        self._spi.bits_per_word = 8
        if hasattr(self._spi, "lsbfirst"):
            self._spi.lsbfirst = False
        if hasattr(self._spi, "cshigh"):
            self._spi.cshigh = False
        if hasattr(self._spi, "threewire"):
            self._spi.threewire = False

    def _reopen_spi(self) -> None:
        try:
            self._spi.close()
        except Exception:
            pass
        self._spi = self._spidev_module.SpiDev()
        self._open_spi()
        self._diag_reopens += 1

    def _xfer(self, frame: bytes) -> bytes:
        """Perform one full-duplex SPI frame transfer with explicit params.

        Uses explicit speed/mode-compatible arguments on every call to avoid
        hidden defaults. Prefers xfer3 when available.
        """
        tx = list(frame)
        # delay_usecs=700: the ESP32 needs ~0.5 ms to process a received frame
        # (handleFrame + buildStatusFrame) and re-arm spi_slave_transmit().
        # 15 µs was too short — the DMA TX buffer was not armed when the Pi
        # sent the next frame, causing a 1-byte shift (bad magic: 0x0150).
        # 700 µs gives comfortable margin with negligible throughput impact
        # (700 µs vs 4096 µs transfer time = <15% overhead at 1 MHz).
        if hasattr(self._spi, "xfer3"):
            return bytes(self._spi.xfer3(tx, self._speed_hz, 700, 8))
        return bytes(self._spi.xfer2(tx, self._speed_hz, 700, 8))

    def _diag_maybe_log(self) -> None:
        now = time.monotonic()
        if now - self._diag_last_log_ts >= 1.0:
            if self._diag_bad_magic or self._diag_bad_crc or self._diag_zero_rx:
                logger.warning(
                    "spi diag host: xfers=%d bad_magic=%d bad_crc=%d zero_rx=%d reopens=%d",
                    self._diag_total_xfers,
                    self._diag_bad_magic,
                    self._diag_bad_crc,
                    self._diag_zero_rx,
                    self._diag_reopens,
                )
            self._diag_total_xfers = 0
            self._diag_bad_magic = 0
            self._diag_bad_crc = 0
            self._diag_zero_rx = 0
            self._diag_reopens = 0
            self._diag_last_log_ts = now

    def __enter__(self) -> "Esp32SpiTransport":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _next_sequence(self) -> int:
        seq = self._sequence & 0xFFFF
        self._sequence += 1
        return seq

    def transfer_frame(self, frame: bytes) -> StatusPayload:
        """Perform one SPI full-duplex transfer and parse the status response.

        Retries up to 4 times on transient bad-magic / bad-CRC frames.  When
        the SPI slave returns all-zeros it means the ESP32 had no transaction
        queued (SPI task was briefly between spi_slave_transmit() calls).  In
        that case the slave also discarded MOSI, so resending is safe.
        """
        if len(frame) != SPI_FRAME_SIZE:
            raise ValueError(f"frame must be exactly {SPI_FRAME_SIZE} bytes")
        last_exc: Exception | None = None
        for attempt in range(15):
            with self._io_lock:
                response = self._xfer(frame)
            self._diag_total_xfers += 1
            try:
                self._consecutive_zero_frames = 0
                status = parse_status_frame(response)
                self._last_status = status
                self._last_status_ts = time.monotonic()
                self._diag_maybe_log()
                return status
            except ValueError as exc:
                last_exc = exc
                if "bad magic" in str(exc) or "bad response CRC" in str(exc):
                    is_zero = response[:32] == b"\x00" * 32
                    if "bad magic" in str(exc):
                        self._diag_bad_magic += 1
                    else:
                        self._diag_bad_crc += 1
                    if is_zero:
                        self._diag_zero_rx += 1
                    # Zero-frame = ESP32 between spi_slave_transmit() calls
                    # (~50 µs gap).  Retry immediately — no sleep.
                    # Non-zero bad magic/CRC = real corruption — short sleep.
                    if attempt < 14:
                        if is_zero:
                            time.sleep(0.0005)
                        else:
                            time.sleep(0.001)
                        continue
                raise
        raise last_exc  # type: ignore[misc]

    def transfer_request(self, frame: bytes) -> tuple[int, StatusPayload]:
        sequence = int.from_bytes(frame[4:6], byteorder="little", signed=False)
        return sequence, self.transfer_frame(frame)

    def wait_for_request_result(
        self,
        sequence: int,
        *,
        poll_interval_s: float = 0.001,
        timeout_s: float = 1.5,
    ) -> StatusPayload:
        deadline = time.monotonic() + max(timeout_s, 0.05)
        last_exc: Exception | None = None
        while True:
            if time.monotonic() >= deadline:
                if last_exc is not None:
                    raise RuntimeError(
                        f"wait_for_request_result timeout for seq={sequence}: {last_exc!s}"
                    ) from last_exc
                raise RuntimeError(
                    f"wait_for_request_result timeout for seq={sequence}: no matching ack"
                )
            try:
                status = self.get_status(
                    timeout_s=min(0.25, max(0.02, deadline - time.monotonic())),
                    allow_stale=False,
                )
            except Exception as exc:
                last_exc = exc
                time.sleep(poll_interval_s)
                continue
            if status.last_rx_sequence == (sequence & 0xFFFF):
                return status
            time.sleep(poll_interval_s)

    def poll_status(self, *, timeout_s: float = 1.0, allow_stale: bool = True) -> StatusPayload:
        last_exc: Exception | None = None
        deadline = time.monotonic() + max(timeout_s, 0.05)
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            seq = self._next_sequence()
            frame = make_get_status(seq)
            try:
                # perform raw transfer so we can inspect the response on failure
                with self._io_lock:
                    response = self._xfer(frame)
                self._diag_total_xfers += 1
            except Exception as exc:
                last_exc = exc
                time.sleep(0.001)
                continue

            try:
                self._consecutive_zero_frames = 0
                status = parse_status_frame(response)
                self._last_status = status
                self._last_status_ts = time.monotonic()
                self._diag_maybe_log()
                return status
            except ValueError as exc:
                last_exc = exc
                # show a short hex preview to aid debugging (first 32 bytes)
                try:
                    preview = response[:32].hex()
                except Exception:
                    preview = "<unavailable>"
                tx_preview = frame[:16].hex()
                if attempt == 1 or attempt >= 10:
                    logger.warning(
                        "spi_transport: attempt %d parse error: %s; rx_preview=%s tx_preview=%s",
                        attempt,
                        str(exc),
                        preview,
                        tx_preview,
                    )
                if "bad magic" in str(exc) or "bad response CRC" in str(exc):
                    is_zero = response[:32] == b"\x00" * 32
                    if "bad magic" in str(exc):
                        self._diag_bad_magic += 1
                    else:
                        self._diag_bad_crc += 1
                    if is_zero:
                        self._diag_zero_rx += 1
                    # Zero-frame: retry immediately (ESP gap is ~50 µs).
                    # Non-zero corruption: short sleep.
                    time.sleep(0.001)
                    self._diag_maybe_log()
                    continue
                raise

        if allow_stale and self._last_status is not None:
            age_s = time.monotonic() - self._last_status_ts
            # Keep motion alive on transient telemetry outages.
            if age_s <= 2.0:
                now = time.monotonic()
                if now - self._last_stale_log_ts >= 0.5:
                    self._last_stale_log_ts = now
                    logger.warning(
                        "SPI status timeout after %d attempts on %s — using stale status age=%.3fs",
                        attempt,
                        self._device_path,
                        age_s,
                    )
                return self._last_status

        raise RuntimeError(
            f"SPI status poll timeout after {attempt} attempts on {self._device_path}: "
            f"{last_exc!s}"
        ) from last_exc

    def get_status(self, *, timeout_s: float = 1.0, allow_stale: bool = True) -> StatusPayload:
        return self.poll_status(timeout_s=timeout_s, allow_stale=allow_stale)

    def set_axis_enabled(self, axis_id: int, enable: bool) -> StatusPayload:
        return self.transfer_frame(make_enable_axis(axis_id, enable, self._next_sequence()))

    def set_axis_enabled_request(self, axis_id: int, enable: bool) -> tuple[int, StatusPayload]:
        return self.transfer_request(make_enable_axis(axis_id, enable, self._next_sequence()))

    def emergency_stop(self, axis_id: int = 0xFF) -> StatusPayload:
        return self.transfer_frame(make_estop(axis_id, self._next_sequence()))

    def stop_axis(self, axis_id: int = 0xFF) -> StatusPayload:
        return self.transfer_frame(make_stop_axis(axis_id, self._next_sequence()))

    def disable_all(self) -> StatusPayload:
        return self.transfer_frame(make_disable_all(self._next_sequence()))

    def reset_stats(self) -> StatusPayload:
        return self.transfer_frame(make_reset_stats(self._next_sequence()))

    def send_step_block(self, payload: StepBlockPayload) -> StatusPayload:
        return self.transfer_frame(make_step_block(payload, self._next_sequence()))

    def send_step_block_request(self, payload: StepBlockPayload) -> tuple[int, StatusPayload]:
        return self.transfer_request(make_step_block(payload, self._next_sequence()))

    def send_segment_block(self, payload: SegmentBlockPayload) -> StatusPayload:
        return self.transfer_frame(make_segment_block(payload, self._next_sequence()))

    def send_segment_block_request(self, payload: SegmentBlockPayload) -> tuple[int, StatusPayload]:
        return self.transfer_request(make_segment_block(payload, self._next_sequence()))

    def send_multi_axis_segment_block(self, payload: MultiAxisSegmentBlockPayload) -> StatusPayload:
        return self.transfer_frame(make_multi_axis_segment_block(payload, self._next_sequence()))

    def send_multi_axis_segment_block_request(self, payload: MultiAxisSegmentBlockPayload) -> tuple[int, StatusPayload]:
        return self.transfer_request(make_multi_axis_segment_block(payload, self._next_sequence()))

    def flush_until(self, sequence: int) -> StatusPayload:
        return self.transfer_frame(make_flush(FlushPayload(flush_sequence=sequence), self._next_sequence()))

    def arm_endstop(self, axis_id: int) -> StatusPayload:
        """Send ENABLE_ENDSTOP to arm the hardware endstop ISR on *axis_id*."""
        return self.transfer_frame(
            make_enable_endstop(EnableEndstopPayload(axis_id=axis_id, arm=True), self._next_sequence())
        )

    def disarm_endstop(self, axis_id: int) -> StatusPayload:
        """Send ENABLE_ENDSTOP to disarm the hardware endstop ISR on *axis_id*."""
        return self.transfer_frame(
            make_enable_endstop(EnableEndstopPayload(axis_id=axis_id, arm=False), self._next_sequence())
        )

    def enable_endstop_request(self, axis_id: int, arm: bool) -> tuple[int, StatusPayload]:
        """Send ENABLE_ENDSTOP and return (sequence, status) for deferred ACK polling.

        arm=True  → firmware arms the endstop ISR on axis_id
        arm=False → firmware disarms the endstop ISR on axis_id
        """
        return self.transfer_request(
            make_enable_endstop(EnableEndstopPayload(axis_id=axis_id, arm=arm), self._next_sequence())
        )

    def wait_for_queue_space(self, axis_id: int, *, minimum_free_blocks: int = 1, poll_interval_s: float = 0.001) -> StatusPayload:
        while True:
            status = self.poll_status()
            if status.queue_free_slots[axis_id] >= minimum_free_blocks:
                return status
            time.sleep(poll_interval_s)

    def send_step_block_with_backpressure(
        self,
        payload: StepBlockPayload,
        *,
        minimum_free_blocks: int = 1,
        poll_interval_s: float = 0.001,
    ) -> StatusPayload:
        self.wait_for_queue_space(payload.axis_id, minimum_free_blocks=minimum_free_blocks, poll_interval_s=poll_interval_s)

        sequence, _ = self.send_step_block_request(payload)
        status = self.wait_for_request_result(sequence, poll_interval_s=poll_interval_s)

        while status.last_result == int(SpiMessageResult.QUEUE_FULL):
            time.sleep(poll_interval_s)
            self.wait_for_queue_space(payload.axis_id, minimum_free_blocks=minimum_free_blocks, poll_interval_s=poll_interval_s)
            sequence, _ = self.send_step_block_request(payload)
            status = self.wait_for_request_result(sequence, poll_interval_s=poll_interval_s)

        if status.last_result != int(SpiMessageResult.OK):
            raise RuntimeError(
                f"step block request seq={sequence} failed with result=0x{status.last_result:02X} "
                f"type=0x{status.last_rx_type:02X}"
            )
        return status

    def send_multi_axis_segment_block_with_backpressure(
        self,
        payload: MultiAxisSegmentBlockPayload,
        *,
        minimum_free_blocks: int = 1,
        poll_interval_s: float = 0.001,
    ) -> StatusPayload:
        # Wait until MCU reports enough free queue slots for the first axis
        # in the block (single-axis homing uses axis_ids[0]).
        axis_id = payload.axis_ids[0]
        self.wait_for_queue_space(axis_id, minimum_free_blocks=minimum_free_blocks, poll_interval_s=poll_interval_s)

        sequence, _ = self.send_multi_axis_segment_block_request(payload)
        status = self.wait_for_request_result(sequence, poll_interval_s=poll_interval_s)

        while status.last_result == int(SpiMessageResult.QUEUE_FULL):
            time.sleep(poll_interval_s)
            self.wait_for_queue_space(axis_id, minimum_free_blocks=minimum_free_blocks, poll_interval_s=poll_interval_s)
            sequence, _ = self.send_multi_axis_segment_block_request(payload)
            status = self.wait_for_request_result(sequence, poll_interval_s=poll_interval_s)

        if status.last_result != int(SpiMessageResult.OK):
            raise RuntimeError(
                f"multi-axis segment block request seq={sequence} failed with result=0x{status.last_result:02X} "
                f"type=0x{status.last_rx_type:02X}"
            )
        return status
