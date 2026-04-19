from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Any, Iterator, List

from transport.messages import (
    MULTI_AXIS_SEGMENT_BLOCK_SIZE,
    MultiAxisSegment,
    MultiAxisSegmentBlockPayload,
    SpiMessageResult,
)
from motion.ramp import AxisMotionConfig, MultiAxisSegmentGenerator, RampConfig
from transport.spi_transport import Esp32SpiTransport


@dataclass(slots=True)
class StreamAxisConfig:
    axis_id: int
    ramp: RampConfig
    minimum_free_blocks: int = 1
    prefill_blocks: int | None = None
    low_watermark_blocks: int | None = None
    max_queued_blocks: int | None = None
    ring_send_threshold: int = 1


class MultiAxisRampStreamer:
    """Minimal deterministic SPI motion streamer.

    The streamer is the host-side source of truth for motion segments.
    It sends pre-computed multi-axis segment blocks over SPI, tracks in-flight
    motion, and uses MCU status feedback to keep the ESP32 queue and ring filled
    without overflowing them.

    Multiple segments are packed per SPI frame (up to MULTI_AXIS_SEGMENT_BLOCK_SIZE)
    to ensure the firmware drain loop has deep look-ahead before starting the RMT.
    """

    TARGET_BUFFER_TIME_S = 0.10
    MIN_BUFFER_TIME_S = 0.06
    MAX_BUFFER_TIME_S = 0.12
    MIN_SEGMENT_TIME_S = 0.002
    MAX_SEGMENT_TIME_S = 0.005
    POLL_SLEEP_S = 0.0005
    MAX_INFLIGHT_SEGMENTS = 24

    # ESP32 step ring capacity in firmware: one step consumes one ring entry.
    STEP_RING_CAPACITY = 4096
    RING_BUFFER_HEADROOM = 0.8

    def __init__(
        self,
        transport: Esp32SpiTransport,
        axis_streams: list[StreamAxisConfig],
        *,
        segment_duration_s: float = 0.004,
        target_buffer_time_s: float = TARGET_BUFFER_TIME_S,
        poll_interval_s: float = 0.001,
        print_every: int = 1,
        log_each_send: bool = False,
        send_log_path: str | None = None,
    ):
        self._transport = transport
        self._poll_interval_s = poll_interval_s
        self._print_every = max(print_every, 1)
        self._log_each_send = log_each_send
        self._send_log_path = send_log_path
        self._send_events: list[dict] = []
        self._stop_requested = False
        self._flush_sequence_requested: int | None = None
        self._endstop_triggered = False
        self._endstop_armed_axes: set[int] = set()

        self._axis_configs = [AxisMotionConfig(axis_id=s.axis_id, ramp=s.ramp) for s in axis_streams]
        self._axis_ids = [s.axis_id for s in axis_streams]
        self._segment_duration_s = max(self.MIN_SEGMENT_TIME_S, min(self.MAX_SEGMENT_TIME_S, segment_duration_s))
        self._target_buffer_time_s = self._compute_target_buffer_time(target_buffer_time_s)
        self._min_buffer_time_s = min(self.MIN_BUFFER_TIME_S, self._target_buffer_time_s * 0.5)

        self._generator = iter(MultiAxisSegmentGenerator(self._axis_configs, segment_duration_s=self._segment_duration_s))
        self._generator_finished = False
        self._inflight: deque[tuple[MultiAxisSegment, int]] = deque()
        self._buffered_time_s = 0.0
        self._last_sent_motion_seq = -1
        self._last_sent_transport_seq = -1

    # -- Helpers ---------------------------------------------------------------

    def _timestamp(self) -> str:
        now = time.time()
        seconds = int(now)
        milliseconds = int((now - seconds) * 1000)
        return time.strftime(f"%H:%M:%S.{milliseconds:03d}", time.localtime(now))

    def _compute_target_buffer_time(self, requested_time_s: float) -> float:
        requested_time_s = max(self.MIN_BUFFER_TIME_S, min(self.MAX_BUFFER_TIME_S, requested_time_s))
        if not self._axis_configs:
            return requested_time_s
        max_hz = max(config.ramp.target_hz for config in self._axis_configs)
        if max_hz <= 0.0:
            return requested_time_s
        safe_time_s = (self.STEP_RING_CAPACITY * self.RING_BUFFER_HEADROOM) / max_hz
        return max(self.MIN_BUFFER_TIME_S, min(requested_time_s, safe_time_s))

    def _enable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, status = self._transport.set_axis_enabled_request(axis_id, True)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"enable axis {axis_id} failed with result=0x{status.last_result:02X}")

    def _queue_full(self, status) -> bool:
        for axis_id in self._axis_ids:
            if axis_id < len(status.queue_free_slots) and status.queue_free_slots[axis_id] == 0:
                return True
            if hasattr(status, "ring_free_slots") and axis_id < len(status.ring_free_slots) and status.ring_free_slots[axis_id] == 0:
                return True
        return False

    def _remove_confirmed_segments(self, status) -> None:
        last_executed = int(getattr(status, "last_executed_sequence", -1))
        while self._inflight:
            segment, _transport_seq = self._inflight[0]
            if segment.sequence <= last_executed:
                self._buffered_time_s -= segment.duration_us / 1_000_000.0
                self._buffered_time_s = max(0.0, self._buffered_time_s)
                self._inflight.popleft()
            else:
                break

    def _check_endstop(self, status) -> bool:
        """Return True if an endstop was triggered on any armed axis.

        Reads endstop_armed_mask from the status frame. Sets
        _endstop_triggered and requests a stop + flush when triggered.
        """
        armed_mask = int(getattr(status, "endstop_armed_mask", 0))
        lateral_state = int(getattr(status, "lateral_endstop_state", 0xFF))
        # lateral_endstop_state values (from firmware LateralEndstopState):
        #   0x00 = PRESENT_OPEN, 0x01 = PRESENT_CLOSED, 0xFF = ABSENT
        PRESENT_CLOSED = 0x01
        if lateral_state == PRESENT_CLOSED and armed_mask != 0:
            if not self._endstop_triggered:
                self._endstop_triggered = True
                flush_seq = self._last_sent_motion_seq
                self.request_stop()
                self.request_flush(flush_seq)
            return True
        return False

    def _record_send_event(self, segment: MultiAxisSegment, transport_seq: int, status) -> None:
        event = {
            "timestamp": time.time(),
            "timestamp_str": self._timestamp(),
            "transport_sequence": transport_seq,
            "segment_sequence": segment.sequence,
            "duration_us": segment.duration_us,
            "axis_count": len(segment.steps),
            "total_steps": sum(segment.steps),
            "queue_free": list(status.queue_free_slots),
            "ring_free": list(status.ring_free_slots),
            "last_result": int(status.last_result),
            "enabled_mask": int(status.enabled_mask),
            "running_mask": int(status.running_mask),
        }
        self._send_events.append(event)
        if self._log_each_send:
            print(
                f"[{event['timestamp_str']}] send tx_seq={transport_seq} "
                f"motion_seq={segment.sequence} duration_us={segment.duration_us} "
                f"total_steps={event['total_steps']} result=0x{event['last_result']:02X}"
            )

    def _write_send_log(self) -> None:
        if self._send_log_path is None:
            return
        with open(self._send_log_path, "w", encoding="utf-8") as handle:
            json.dump(self._send_events, handle, indent=2)

    # -- Endstop control -------------------------------------------------------

    def arm_endstop(self, axis_id: int) -> None:
        """Send ENABLE_ENDSTOP arm command to firmware and track locally.

        Call before starting a move that should stop on endstop contact.
        """
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=True)
        self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
        self._endstop_armed_axes.add(axis_id)

    def disarm_endstop(self, axis_id: int) -> None:
        """Send ENABLE_ENDSTOP disarm command to firmware.

        Call before a clearance move that must pass through the endstop.
        """
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=False)
        self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
        self._endstop_armed_axes.discard(axis_id)

    @property
    def endstop_triggered(self) -> bool:
        return self._endstop_triggered

    # -- Stop / flush ----------------------------------------------------------

    def request_stop(self) -> None:
        self._stop_requested = True

    def has_stop_been_requested(self) -> bool:
        return self._stop_requested

    def request_flush(self, sequence: int) -> None:
        self._flush_sequence_requested = sequence

    def flush_until(self, sequence: int):
        status = self._transport.flush_until(sequence)
        self._inflight.clear()
        self._buffered_time_s = 0.0
        return status

    # -- Core streaming primitives ---------------------------------------------

    def _collect_and_send_batch(self, status) -> tuple[int, Any] | None:
        """Collect up to MULTI_AXIS_SEGMENT_BLOCK_SIZE segments and send one frame.

        Returns (segments_sent, last_status) on success, (0, status) on
        QUEUE_FULL, or None when nothing can be batched (buffer target
        reached, inflight limit reached, or generator already exhausted).

        Packing multiple segments per frame is critical for ring pre-fill:
        the firmware drain loop processes all queued frames before starting
        the RMT, so more segments per frame = deeper ring buffer at startup.
        """
        if self._generator_finished:
            return None

        batch: list[MultiAxisSegment] = []
        while (
            len(batch) < MULTI_AXIS_SEGMENT_BLOCK_SIZE
            and self._buffered_time_s < self._target_buffer_time_s
            and len(self._inflight) < self.MAX_INFLIGHT_SEGMENTS
            and not self._queue_full(status)
        ):
            try:
                segment = next(self._generator)
            except StopIteration:
                self._generator_finished = True
                break

            if segment.sequence <= self._last_sent_motion_seq:
                raise RuntimeError(
                    f"motion sequence not strictly increasing: "
                    f"got {segment.sequence}, last was {self._last_sent_motion_seq}"
                )
            batch.append(segment)

        if not batch:
            return None

        payload = MultiAxisSegmentBlockPayload(
            axis_ids=self._axis_ids,
            block_seq=batch[0].sequence,
            segments=batch,
        )
        transport_seq, send_status = self._transport.send_multi_axis_segment_block_request(payload)

        if send_status.last_result == int(SpiMessageResult.OK):
            for seg in batch:
                self._inflight.append((seg, transport_seq))
                self._buffered_time_s += seg.duration_us / 1_000_000.0
                self._last_sent_motion_seq = seg.sequence
                self._record_send_event(seg, transport_seq, send_status)
            self._last_sent_transport_seq = transport_seq
            return len(batch), send_status
        elif send_status.last_result == int(SpiMessageResult.QUEUE_FULL):
            return 0, send_status
        else:
            raise RuntimeError(
                f"segment batch starting motion_seq={batch[0].sequence} "
                f"failed with result=0x{send_status.last_result:02X}"
            )

    def _prefill(self, status) -> tuple[int, Any]:
        """Send MIN_PREFILL_BATCHES back-to-back without waiting between them.

        Ensures the firmware ring has deep look-ahead before the RMT starts.
        Returns (total_segments_sent, last_status).
        MIN_PREFILL_BATCHES = 3 fills ~3 x MULTI_AXIS_SEGMENT_BLOCK_SIZE segments.
        """
        MIN_PREFILL_BATCHES = 3
        total = 0
        last_status = status
        for _ in range(MIN_PREFILL_BATCHES):
            result = self._collect_and_send_batch(last_status)
            if result is None:
                break
            n, last_status = result
            total += n
            if n == 0:  # QUEUE_FULL -- stop prefilling
                break
        return total, last_status

    def _should_sleep(self) -> float:
        """Return sleep duration in seconds based on buffer fullness.

        Returns 0.0 if the buffer needs immediate refill.
        """
        if self._buffered_time_s >= self._target_buffer_time_s:
            return self._segment_duration_s
        if self._buffered_time_s >= self._min_buffer_time_s:
            return self._segment_duration_s / 2.0
        return 0.0

    # -- Main streaming loop ---------------------------------------------------

    def stream_all(self) -> int:
        self._generator_finished = False
        status = self._transport.get_status()
        self._enable_axes()
        status = self._transport.get_status()

        total_segments, status = self._prefill(status)

        while True:
            if self._stop_requested:
                if self._flush_sequence_requested is not None:
                    self.flush_until(self._flush_sequence_requested)
                break

            status = self._transport.get_status()
            self._remove_confirmed_segments(status)

            if self._check_endstop(status):
                break

            while not self._generator_finished:
                result = self._collect_and_send_batch(status)
                if result is None:
                    break
                n, status = result
                if n == 0:  # QUEUE_FULL
                    break
                total_segments += n
                if total_segments % self._print_every == 0:
                    print(
                        f"[{self._timestamp()}] segments={total_segments} "
                        f"buffered={self._buffered_time_s*1000:.1f}ms "
                        f"inflight={len(self._inflight)} "
                        f"queue_free={status.queue_free_slots} "
                        f"ring_free={status.ring_free_slots} "
                        f"underrun={status.underrun_count}"
                    )

            if self._flush_sequence_requested is not None:
                self.flush_until(self._flush_sequence_requested)
                self._flush_sequence_requested = None

            if self._generator_finished and not self._inflight:
                break

            sleep_s = self._should_sleep()
            if sleep_s > 0.0:
                time.sleep(sleep_s)

        self._write_send_log()
        return total_segments
