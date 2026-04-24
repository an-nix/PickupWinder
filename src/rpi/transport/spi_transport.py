from __future__ import annotations

import logging
import re
import threading
import time
from typing import Iterable


from transport.messages import (
    SPI_FRAME_SIZE,
    MessageHeader,
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


class _ReadyPinMonitor:
    """Best-effort GPIO input reader for the ESP32 SPI READY sideband.

    Prefers libgpiod v2 when available and falls back to the older Chip/Line API.
    This keeps the transport usable across Raspberry Pi OS images that ship
    different python-gpiod versions.
    """

    def __init__(self, chip_path: str, line_offset: int):
        try:
            import gpiod  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "READY GPIO handshake requested but python gpiod module is not installed"
            ) from exc

        self._gpiod = gpiod
        self._chip_path = chip_path
        self._line_offset = line_offset
        self._reader = self._open_reader()

    def _open_reader(self):
        gpiod = self._gpiod
        line_offset = self._line_offset
        consumer = "pickupwinder-spi-ready"

        if hasattr(gpiod, "request_lines") and hasattr(gpiod, "LineSettings"):
            config = {
                line_offset: gpiod.LineSettings(direction=gpiod.line.Direction.INPUT)
            }
            request = gpiod.request_lines(
                self._chip_path,
                consumer=consumer,
                config=config,
            )

            def _read() -> int:
                value = request.get_value(line_offset)
                return int(getattr(value, "value", value))

            self._close = getattr(request, "release", lambda: None)
            return _read

        chip_name = self._chip_path.removeprefix("/dev/")
        chip = gpiod.Chip(chip_name)
        line = chip.get_line(line_offset)
        line.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_IN)

        def _read() -> int:
            return int(line.get_value())

        def _close() -> None:
            try:
                line.release()
            finally:
                chip.close()

        self._close = _close
        return _read

    def value(self) -> int:
        return self._reader()

    def close(self) -> None:
        self._close()


class Esp32SpiTransport:
    """Thin wrapper around spidev using the PickupWinder fixed SPI frame format."""

    def __init__(
        self,
        bus: int | None = None,
        device: int | None = None,
        *,
        device_path: str | None = None,
        speed_hz: int = 4_000_000,
        mode: int = 1,
        ready_gpio_chip: str | None = None,
        ready_gpio_line: int | None = None,
        ready_active_high: bool = True,
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
        self._ready_monitor: _ReadyPinMonitor | None = None
        self._ready_active_level = 1 if ready_active_high else 0
        self._ready_wait_timeout_s = 0.050
        self._ready_poll_sleep_s = 0.00002
        self._ready_timeout_streak: int = 0
        self._ready_timeout_disable_threshold: int = 3
        self._ready_handshake_disabled: bool = False
        # Real wall-clock gap enforced between completed SPI calls.
        # On Raspberry Pi spidev, delay_usecs is a controller-side transfer
        # delay and does not reliably create a slave-visible re-arm window
        # between two separate Python xfer calls. Without a ready GPIO, the
        # ESP32 slave needs a host-visible idle gap so spi_slave_transmit() can
        # rebuild and re-arm the next DMA descriptor before CS is asserted
        # again. Espressif explicitly recommends a ready/handshake GPIO; while
        # we still run without that wire, use a conservative software guard.
        # 800 µs costs little at 4 MHz because one motion frame can carry many
        # segments, but it materially reduces short frames and bad-magic polls.
        self._inter_transfer_guard_s: float = 0.0008
        self._last_xfer_end_ts: float = 0.0
        self._diag_total_xfers: int = 0
        self._diag_bad_magic: int = 0
        self._diag_bad_crc: int = 0
        self._diag_zero_rx: int = 0
        self._diag_echo_rx: int = 0
        self._diag_reopens: int = 0
        self._diag_lifetime_total_xfers: int = 0
        self._diag_lifetime_bad_magic: int = 0
        self._diag_lifetime_bad_crc: int = 0
        self._diag_lifetime_zero_rx: int = 0
        self._diag_lifetime_echo_rx: int = 0
        self._diag_lifetime_reopens: int = 0
        self._diag_ready_timeouts: int = 0
        self._diag_lifetime_ready_timeouts: int = 0
        self._diag_last_log_ts: float = time.monotonic()

        if ready_gpio_chip is not None and ready_gpio_line is not None:
            try:
                self._ready_monitor = _ReadyPinMonitor(ready_gpio_chip, ready_gpio_line)
                self._inter_transfer_guard_s = 0.0
                logger.info(
                    "SPI READY handshake enabled on %s line %d (active_%s)",
                    ready_gpio_chip,
                    ready_gpio_line,
                    "high" if ready_active_high else "low",
                )
            except Exception as exc:
                logger.warning(
                    "failed to initialize SPI READY handshake on %s line %d: %s; falling back to software guard",
                    ready_gpio_chip,
                    ready_gpio_line,
                    exc,
                )

    def close(self) -> None:
        if self._ready_monitor is not None:
            try:
                self._ready_monitor.close()
            except Exception:
                pass
        self._spi.close()

    @staticmethod
    def _axes_from_mask(mask: int, axis_ids: Iterable[int]) -> list[int]:
        return [axis_id for axis_id in axis_ids if mask & (1 << axis_id)]

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
        self._diag_lifetime_reopens += 1

    def _xfer(self, frame: bytes) -> bytes:
        """Perform one full-duplex SPI frame transfer with explicit params.

        Uses explicit speed/mode-compatible arguments on every call to avoid
        hidden defaults. Prefers xfer3 when available.
        """
        self._wait_until_ready()

        tx = list(frame)
        # Keep the spidev transfer itself simple and rely on the explicit host
        # inter-transfer guard above. delay_usecs does not reliably translate
        # into a slave-visible idle gap between separate xfer() calls.
        if hasattr(self._spi, "xfer3"):
            response = bytes(self._spi.xfer3(tx, self._speed_hz, 0, 8))
        else:
            response = bytes(self._spi.xfer2(tx, self._speed_hz, 0, 8))

        self._last_xfer_end_ts = time.monotonic()
        return response

    def _wait_until_ready(self) -> None:
        if self._ready_monitor is not None and not self._ready_handshake_disabled:
            deadline = time.monotonic() + self._ready_wait_timeout_s
            while time.monotonic() < deadline:
                if self._ready_monitor.value() == self._ready_active_level:
                    self._ready_timeout_streak = 0
                    return
                time.sleep(self._ready_poll_sleep_s)
            self._diag_ready_timeouts += 1
            self._diag_lifetime_ready_timeouts += 1
            self._ready_timeout_streak += 1
            if self._ready_timeout_streak >= self._ready_timeout_disable_threshold:
                self._ready_handshake_disabled = True
                logger.warning(
                    "SPI READY handshake timed out %d times on %s; disabling READY GPIO for this session and using software guard",
                    self._ready_timeout_streak,
                    self._device_path,
                )
            else:
                logger.warning(
                    "SPI READY handshake timeout on %s; falling back to software guard for this transfer",
                    self._device_path,
                )

        now = time.monotonic()
        if self._last_xfer_end_ts > 0.0:
            remaining_gap_s = self._inter_transfer_guard_s - (now - self._last_xfer_end_ts)
            if remaining_gap_s > 0.0:
                time.sleep(remaining_gap_s)

    def _diag_maybe_log(self) -> None:
        now = time.monotonic()
        if now - self._diag_last_log_ts >= 1.0:
            if self._diag_bad_magic or self._diag_bad_crc or self._diag_zero_rx or self._diag_echo_rx or self._diag_ready_timeouts:
                logger.warning(
                    "spi diag host: xfers=%d bad_magic=%d bad_crc=%d zero_rx=%d echo_rx=%d ready_timeouts=%d reopens=%d",
                    self._diag_total_xfers,
                    self._diag_bad_magic,
                    self._diag_bad_crc,
                    self._diag_zero_rx,
                    self._diag_echo_rx,
                    self._diag_ready_timeouts,
                    self._diag_reopens,
                )
            self._diag_total_xfers = 0
            self._diag_bad_magic = 0
            self._diag_bad_crc = 0
            self._diag_zero_rx = 0
            self._diag_echo_rx = 0
            self._diag_ready_timeouts = 0
            self._diag_reopens = 0
            self._diag_last_log_ts = now

    @staticmethod
    def _is_echoed_request_response(request_frame: bytes, response_frame: bytes) -> bool:
        """Return True when *response_frame* is just the host request echoed back.

        This shows up in the field as a parseable header with the request type
        (commonly GET_STATUS = 0x06) instead of STATUS, and with the same
        sequence/CRC as the frame we just transmitted. Treat it as transient
        link noise / slave-not-ready behavior and retry.
        """
        if len(request_frame) != SPI_FRAME_SIZE or len(response_frame) != SPI_FRAME_SIZE:
            return False
        try:
            request_header = MessageHeader.unpack(request_frame)
            response_header = MessageHeader.unpack(response_frame)
        except Exception:
            return False
        if response_header.msg_type == int(SpiMessageType.STATUS):
            return False
        return (
            response_header.magic == request_header.magic
            and response_header.version == request_header.version
            and response_header.msg_type == request_header.msg_type
            and response_header.sequence == request_header.sequence
            and response_header.payload_length == request_header.payload_length
            and response_header.flags == request_header.flags
            and response_header.crc16 == request_header.crc16
        )

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

        Retries up to 15 times on transient bad-magic / bad-CRC frames.  When
        the SPI slave returns all-zeros it means the ESP32 had no transaction
        queued yet when the master asserted CS. In
        that case the slave also discarded MOSI, so resending is safe.
        """
        if len(frame) != SPI_FRAME_SIZE:
            raise ValueError(f"frame must be exactly {SPI_FRAME_SIZE} bytes")
        last_exc: Exception | None = None
        for attempt in range(15):
            with self._io_lock:
                response = self._xfer(frame)
            self._diag_total_xfers += 1
            self._diag_lifetime_total_xfers += 1
            try:
                self._consecutive_zero_frames = 0
                status = parse_status_frame(response)
                self._last_status = status
                self._last_status_ts = time.monotonic()
                self._diag_maybe_log()
                return status
            except ValueError as exc:
                last_exc = exc
                if self._is_echoed_request_response(frame, response):
                    self._diag_echo_rx += 1
                    self._diag_lifetime_echo_rx += 1
                    if attempt < 14:
                        time.sleep(0.0005)
                        continue
                if "bad magic" in str(exc) or "bad response CRC" in str(exc):
                    is_zero = response[:32] == b"\x00" * 32
                    if "bad magic" in str(exc):
                        self._diag_bad_magic += 1
                        self._diag_lifetime_bad_magic += 1
                    else:
                        self._diag_bad_crc += 1
                        self._diag_lifetime_bad_crc += 1
                    if is_zero:
                        self._diag_zero_rx += 1
                        self._diag_lifetime_zero_rx += 1
                    # Zero-frame = ESP32 was not armed with a valid queued
                    # descriptor when the transfer started.
                    # Retry with a shorter backoff than for non-zero
                    # corruption so the host keeps pace without busy-spinning.
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
        hint_status: StatusPayload | None = None,
        poll_interval_s: float = 0.0005,
        timeout_s: float = 1.5,
    ) -> StatusPayload:
        """Wait until the ESP32 acknowledges *sequence* via last_rx_sequence.

        ``hint_status`` is the status payload returned by the preceding
        transfer_frame / transfer_request call.  With prequeue=2 the pipeline
        depth is two frames, so the hint rarely carries the matching ACK, but
        checking it is free and eliminates one poll on any early-delivery edge
        case (e.g. slow master, fast firmware processing).

        ``poll_interval_s`` defaults to 0.1 ms — tight enough to catch the
        ACK in the second poll without busy-spinning.  The old 1 ms default
        added unnecessary latency equal to the entire SPI task rebuild cycle.
        """
        transient_protocol_results = {
            int(SpiMessageResult.BAD_MAGIC),
            int(SpiMessageResult.BAD_VERSION),
            int(SpiMessageResult.BAD_LENGTH),
            int(SpiMessageResult.BAD_CRC),
        }
        target_seq = sequence & 0xFFFF
        # Free first check: the caller already holds a status response from the
        # preceding send.  If it carries the matching ACK we skip all polls.
        if hint_status is not None:
            if hint_status.last_rx_sequence == target_seq:
                if int(hint_status.last_result) not in transient_protocol_results:
                    return hint_status
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
            if status.last_rx_sequence == target_seq:
                if int(status.last_result) in transient_protocol_results:
                    last_exc = RuntimeError(
                        "transient SPI protocol error observed after matching ack "
                        f"for seq={sequence}: result=0x{int(status.last_result):02X}"
                    )
                    time.sleep(poll_interval_s)
                    continue
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
                self._diag_lifetime_total_xfers += 1
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
                if self._is_echoed_request_response(frame, response):
                    self._diag_echo_rx += 1
                    self._diag_lifetime_echo_rx += 1
                    time.sleep(0.0005)
                    self._diag_maybe_log()
                    continue
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
                        self._diag_lifetime_bad_magic += 1
                    else:
                        self._diag_bad_crc += 1
                        self._diag_lifetime_bad_crc += 1
                    if is_zero:
                        self._diag_zero_rx += 1
                        self._diag_lifetime_zero_rx += 1
                    # Retry corrupted polls with a short backoff. Zero-frames
                    # get the same treatment here because this path is
                    # telemetry-only and does not need the tighter request
                    # retry behavior used by transfer_frame().
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

    def transport_diagnostics(self) -> dict[str, int | float | None]:
        last_status_age_s: float | None = None
        if self._last_status is not None:
            last_status_age_s = round(time.monotonic() - self._last_status_ts, 3)
        return {
            "total_xfers": self._diag_lifetime_total_xfers,
            "bad_magic": self._diag_lifetime_bad_magic,
            "bad_crc": self._diag_lifetime_bad_crc,
            "zero_rx": self._diag_lifetime_zero_rx,
            "echo_rx": self._diag_lifetime_echo_rx,
            "ready_timeouts": self._diag_lifetime_ready_timeouts,
            "ready_handshake_disabled": int(self._ready_handshake_disabled),
            "reopens": self._diag_lifetime_reopens,
            "last_status_age_s": last_status_age_s,
        }

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

    def safe_shutdown(
        self,
        *,
        axis_ids: Iterable[int],
        keep_enabled_axes: Iterable[int] = (),
        timeout_s: float = 1.0,
        emergency_on_failure: bool = True,
    ) -> StatusPayload | None:
        resolved_axis_ids = tuple(dict.fromkeys(int(axis_id) for axis_id in axis_ids))
        keep_enabled = frozenset(int(axis_id) for axis_id in keep_enabled_axes)
        last_status: StatusPayload | None = None
        errors: list[str] = []

        def verify_status(label: str) -> None:
            nonlocal last_status
            try:
                last_status = self.get_status(timeout_s=timeout_s, allow_stale=False)
            except Exception as exc:
                errors.append(f"{label} status verification failed: {exc}")
                return

            unexpected_running = self._axes_from_mask(
                last_status.running_mask,
                resolved_axis_ids,
            )
            unexpected_enabled = [
                axis_id
                for axis_id in self._axes_from_mask(last_status.enabled_mask, resolved_axis_ids)
                if axis_id not in keep_enabled
            ]
            if unexpected_running:
                errors.append(
                    f"{label} running axes remain active: {unexpected_running}"
                )
            if unexpected_enabled:
                errors.append(
                    f"{label} enabled axes remain active: {unexpected_enabled}"
                )

        try:
            self.stop_axis()
        except Exception as exc:
            errors.append(f"stop_axis failed: {exc}")

        for axis_id in resolved_axis_ids:
            if axis_id in keep_enabled:
                continue
            try:
                self.set_axis_enabled(axis_id, False)
            except Exception as exc:
                errors.append(f"disable axis {axis_id} failed: {exc}")

        verify_status("safe_shutdown")

        if errors and emergency_on_failure:
            initial_errors = list(errors)
            errors.clear()
            try:
                self.emergency_stop()
            except Exception as exc:
                errors.append(f"emergency_stop failed: {exc}")
            if not keep_enabled:
                try:
                    self.disable_all()
                except Exception as exc:
                    errors.append(f"disable_all failed: {exc}")
            verify_status("post_emergency_stop")
            if errors:
                raise RuntimeError("; ".join(initial_errors + errors))
            logger.error(
                "Transport safe_shutdown escalated to emergency stop after: %s",
                "; ".join(initial_errors),
            )
            return last_status

        if errors:
            raise RuntimeError("; ".join(errors))

        return last_status

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
        transport_sequence, _ = self.transfer_request(
            make_flush(FlushPayload(flush_sequence=sequence), self._next_sequence())
        )
        return self.wait_for_request_result(transport_sequence)

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

        sequence, send_status = self.send_step_block_request(payload)
        status = self.wait_for_request_result(sequence, hint_status=send_status, poll_interval_s=poll_interval_s)

        while status.last_result == int(SpiMessageResult.QUEUE_FULL):
            time.sleep(poll_interval_s)
            self.wait_for_queue_space(payload.axis_id, minimum_free_blocks=minimum_free_blocks, poll_interval_s=poll_interval_s)
            sequence, send_status = self.send_step_block_request(payload)
            status = self.wait_for_request_result(sequence, hint_status=send_status, poll_interval_s=poll_interval_s)

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

        sequence, send_status = self.send_multi_axis_segment_block_request(payload)
        status = self.wait_for_request_result(sequence, hint_status=send_status, poll_interval_s=poll_interval_s)

        while status.last_result == int(SpiMessageResult.QUEUE_FULL):
            time.sleep(poll_interval_s)
            self.wait_for_queue_space(axis_id, minimum_free_blocks=minimum_free_blocks, poll_interval_s=poll_interval_s)
            sequence, send_status = self.send_multi_axis_segment_block_request(payload)
            status = self.wait_for_request_result(sequence, hint_status=send_status, poll_interval_s=poll_interval_s)

        if status.last_result != int(SpiMessageResult.OK):
            raise RuntimeError(
                f"multi-axis segment block request seq={sequence} failed with result=0x{status.last_result:02X} "
                f"type=0x{status.last_rx_type:02X}"
            )
        return status
