from __future__ import annotations

import time

try:  # pragma: no cover - import mode depends on how the script is started
    from .messages import (
        SPI_FRAME_SIZE,
        SpiMessageType,
        SpiMessageResult,
        StatusPayload,
        MultiAxisSegmentBlockPayload,
        SegmentBlockPayload,
        StepBlockPayload,
        FlushPayload,
        make_disable_all,
        make_enable_axis,
        make_estop,
        make_flush,
        make_get_status,
        make_reset_stats,
        make_segment_block,
        make_step_block,
        make_stop_axis,
        make_multi_axis_segment_block,
        parse_status_frame,
    )
except ImportError:  # pragma: no cover - direct script execution fallback
    from messages import (  # type: ignore
        SPI_FRAME_SIZE,
        SpiMessageType,
        SpiMessageResult,
        StatusPayload,
        MultiAxisSegmentBlockPayload,
        SegmentBlockPayload,
        StepBlockPayload,
        make_disable_all,
        make_enable_axis,
        make_estop,
        make_get_status,
        make_reset_stats,
        make_segment_block,
        make_step_block,
        make_stop_axis,
        make_multi_axis_segment_block,
        parse_status_frame,
    )


class Esp32SpiTransport:
    """Thin wrapper around spidev using the PickupWinder fixed SPI frame format."""

    def __init__(self, bus: int = 0, device: int = 0, *, speed_hz: int = 4_000_000, mode: int = 0):
        try:
            import spidev  # type: ignore
        except ImportError as exc:  # pragma: no cover - depends on host machine
            raise RuntimeError("spidev module is required on the Raspberry Pi host") from exc

        self._spidev_module = spidev
        self._spi = spidev.SpiDev()
        self._spi.open(bus, device)
        self._spi.max_speed_hz = speed_hz
        self._spi.mode = mode
        self._sequence = 0

    def close(self) -> None:
        self._spi.close()

    def __enter__(self) -> "Esp32SpiTransport":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _next_sequence(self) -> int:
        seq = self._sequence & 0xFFFF
        self._sequence += 1
        return seq

    def transfer_frame(self, frame: bytes) -> StatusPayload:
        if len(frame) != SPI_FRAME_SIZE:
            raise ValueError(f"frame must be exactly {SPI_FRAME_SIZE} bytes")
        response = bytes(self._spi.xfer2(list(frame)))
        return parse_status_frame(response)

    def transfer_request(self, frame: bytes) -> tuple[int, StatusPayload]:
        sequence = int.from_bytes(frame[4:6], byteorder="little", signed=False)
        return sequence, self.transfer_frame(frame)

    def wait_for_request_result(self, sequence: int, *, poll_interval_s: float = 0.001) -> StatusPayload:
        while True:
            status = self.get_status()
            if status.last_rx_sequence == (sequence & 0xFFFF):
                return status
            time.sleep(poll_interval_s)

    def poll_status(self) -> StatusPayload:
        last_exc: Exception | None = None
        for attempt in range(5):
            seq = self._next_sequence()
            frame = make_get_status(seq)
            try:
                # perform raw transfer so we can inspect the response on failure
                response = bytes(self._spi.xfer2(list(frame)))
            except Exception as exc:
                last_exc = exc
                time.sleep(0.01)
                continue

            try:
                return parse_status_frame(response)
            except ValueError as exc:
                last_exc = exc
                # show a short hex preview to aid debugging (first 32 bytes)
                try:
                    preview = response[:32].hex()
                except Exception:
                    preview = "<unavailable>"
                print(f"spi_transport: attempt {attempt+1}: parse error: {exc!s}; frame_preview={preview}")
                if "bad magic" in str(exc) or "bad response CRC" in str(exc):
                    time.sleep(0.01)
                    continue
                raise

        raise RuntimeError(
            "SPI status poll failed after 5 attempts: "
            f"{last_exc!s}"
        ) from last_exc

    def get_status(self) -> StatusPayload:
        return self.poll_status()

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
