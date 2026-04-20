from __future__ import annotations

from collections import deque
import threading
import time
from transport.messages import (
    SpiMessageType,
    SpiMessageResult,
    SPI_MSG_VERSION,
    StatusPayload,
)


class MockSpiTransport:
    """Fake SPI transport for host-side streaming tests.

    This transport does not access real SPI hardware. It implements the
    minimal status/request behavior required by the streamer and example
    scripts.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sequence = 0
        self._last_motion_executed_sequence = -1
        self._pending_motion_sequences: deque[int] = deque()
        self._enabled_axes: set[int] = set()
        self._status = self._make_status(self._last_motion_executed_sequence)
        self.sent_blocks: list[tuple[int, int]] = []
        self.sent_payloads: list[object] = []

    def close(self) -> None:
        return None

    def __enter__(self) -> "MockSpiTransport":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _next_sequence(self) -> int:
        with self._lock:
            seq = self._sequence & 0xFFFF
            self._sequence += 1
        return seq

    def _advance_executed(self) -> int:
        if self._pending_motion_sequences:
            self._last_motion_executed_sequence = self._pending_motion_sequences.popleft()
        return self._last_motion_executed_sequence

    def _make_status(self, last_rx_sequence: int) -> StatusPayload:
        last_executed_sequence = self._advance_executed()
        return StatusPayload(
            uptime_ms=int(time.time() * 1000) & 0xFFFFFFFF,
            queue_free_slots=(128, 128, 128, 128),
            ring_free_slots=(4096, 4096, 4096, 4096),
            underrun_count=(0, 0, 0, 0),
            last_rx_sequence=last_rx_sequence,
            last_rx_type=int(SpiMessageType.STATUS),
            last_result=int(SpiMessageResult.OK),
            protocol_version=SPI_MSG_VERSION,
            enabled_mask=0,
            running_mask=0,
            lateral_endstop_state=0,
            endstop_armed_mask=0,
            last_executed_sequence=last_executed_sequence,
            multi_axis_queue_free=64,
            planner_queue_free=128,
            last_planned_sequence=last_executed_sequence,
            segments_dropped=0,
        )

    def get_status(self, *, timeout_s: float = 1.0, allow_stale: bool = True) -> StatusPayload:
        self._status = self._make_status(self._status.last_rx_sequence)
        return self._status

    def poll_status(self, *, timeout_s: float = 1.0, allow_stale: bool = True) -> StatusPayload:
        return self.get_status(timeout_s=timeout_s, allow_stale=allow_stale)

    def transfer_frame(self, frame: bytes) -> StatusPayload:
        return self._status

    def transfer_request(self, frame: bytes) -> tuple[int, StatusPayload]:
        seq = int.from_bytes(frame[4:6], byteorder="little", signed=False)
        self._status = self._make_status(seq)
        return seq, self._status

    def wait_for_request_result(self, sequence: int, *, poll_interval_s: float = 0.001, timeout_s: float = 1.5) -> StatusPayload:
        return self._make_status(sequence)

    def set_axis_enabled_request(self, axis_id: int, enable: bool) -> tuple[int, StatusPayload]:
        seq = self._next_sequence()
        if enable:
            self._enabled_axes.add(axis_id)
        else:
            self._enabled_axes.discard(axis_id)
        status = self._make_status(seq)
        return seq, status

    def disable_all(self) -> StatusPayload:
        self._enabled_axes.clear()
        self._status = self._make_status(self._next_sequence())
        return self._status

    def send_multi_axis_segment_block_request(self, payload) -> tuple[int, StatusPayload]:
        seq = self._next_sequence()
        self.sent_blocks.append((seq, len(payload.segments)))
        self.sent_payloads.append(payload)
        if payload.segments:
            self._pending_motion_sequences.extend(
                int(segment.sequence) & 0xFFFF
                for segment in payload.segments
            )
        self._status = self._make_status(seq)
        return seq, self._status

    def send_multi_axis_segment_block(self, payload) -> StatusPayload:
        seq, status = self.send_multi_axis_segment_block_request(payload)
        return status

    def flush_until(self, sequence: int) -> StatusPayload:
        self._last_motion_executed_sequence = sequence & 0xFFFF
        self._pending_motion_sequences.clear()
        self._status = self._make_status(self._next_sequence())
        return self._status

    def set_axis_enabled(self, axis_id: int, enable: bool) -> StatusPayload:
        seq, status = self.set_axis_enabled_request(axis_id, enable)
        return status

    def wait_for_queue_space(self, axis_id: int, *, minimum_free_blocks: int = 1, poll_interval_s: float = 0.001) -> StatusPayload:
        return self._status
