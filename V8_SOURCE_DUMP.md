# V8 Source Dump

### src/rpi/transport/streamer.py
```python
from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Any, Iterator

from transport.messages import (
    MULTI_AXIS_SEGMENT_BLOCK_SIZE,
    MultiAxisSegment,
    MultiAxisSegmentBlockPayload,
    SpiMessageResult,
    sequence_is_greater,
    sequence_is_less_equal,
)
from motion import AxisMotionConfig, MultiAxisSegmentGenerator, RampConfig
from transport.spi_transport import Esp32SpiTransport

logger = logging.getLogger(__name__)


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

    # Planner→executor segment queue depth on the ESP32 (matches SEGMENT_QUEUE_DEPTH in firmware).
    SEGMENT_QUEUE_DEPTH = 128
    # Legacy constant kept for reference (= EXEC_BATCH_LIMIT * 2).
    # The active gate is now required_lookahead() which is speed-dependent.
    PLANNER_QUEUE_SEND_THRESHOLD = 32

    # ESP32 step ring capacity in firmware: one step consumes one ring entry.
    STEP_RING_CAPACITY = 4096
    RING_BUFFER_HEADROOM = 0.8

    @staticmethod
    def required_lookahead(steps_per_segment: int) -> int:
        """Speed-dependent minimum segment lookahead depth in the ESP32 planner queue.

        At low speed each segment contains very few steps, so the ring drains
        faster relative to the inter-segment host→ESP32 pipeline latency (~3–5 ms).
        A deeper buffer prevents ring underruns and motor stutter.

        Thresholds match firmware EXEC_BATCH_LIMIT tiers:
          < 10  steps → 48 segments (low speed,  ~50 RPM)
          < 50  steps → 32 segments (mid speed)
          >= 50 steps → 16 segments (high speed, > ~200 RPM)
        """
        if steps_per_segment < 10:
            return 48
        elif steps_per_segment < 50:
            return 32
        else:
            return 16

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
        self._initialize_streamer_state(
            transport=transport,
            axis_configs=[AxisMotionConfig(axis_id=s.axis_id, ramp=s.ramp) for s in axis_streams],
            axis_ids=[s.axis_id for s in axis_streams],
            segment_duration_s=segment_duration_s,
            target_buffer_time_s=target_buffer_time_s,
            poll_interval_s=poll_interval_s,
            print_every=print_every,
            log_each_send=log_each_send,
            send_log_path=send_log_path,
            explicit_target_hz=None,
        )

    @classmethod
    def from_axis_ids(
        cls,
        transport: Esp32SpiTransport,
        axis_ids: list[int],
        *,
        target_hz: float,
        segment_duration_s: float = 0.004,
        poll_interval_s: float = 0.001,
        print_every: int = 1,
        target_buffer_time_s: float = 0.150,
    ) -> "MultiAxisRampStreamer":
        """Build a streamer from explicit axis IDs and a known target frequency.

        Use this constructor when the move generator is external (e.g. `WoundMove`)
        and no reliable `RampConfig` objects are available.

        Differences vs `__init__`:
          - `__init__`: derives `target_hz` from `RampConfig.target_hz`.
          - `from_axis_ids`: receives `target_hz` explicitly and avoids synthetic ramps.
        """
        if not axis_ids:
            raise ValueError("axis_ids must not be empty")
        if target_hz <= 0.0:
            raise ValueError("target_hz must be positive")

        streamer = cls.__new__(cls)
        streamer._initialize_streamer_state(
            transport=transport,
            axis_configs=[],
            axis_ids=axis_ids,
            segment_duration_s=segment_duration_s,
            target_buffer_time_s=target_buffer_time_s,
            poll_interval_s=poll_interval_s,
            print_every=print_every,
            log_each_send=False,
            send_log_path=None,
            explicit_target_hz=target_hz,
        )
        return streamer

    def _initialize_streamer_state(
        self,
        *,
        transport: Esp32SpiTransport,
        axis_configs: list[AxisMotionConfig],
        axis_ids: list[int],
        segment_duration_s: float,
        target_buffer_time_s: float,
        poll_interval_s: float,
        print_every: int,
        log_each_send: bool,
        send_log_path: str | None,
        explicit_target_hz: float | None,
    ) -> None:
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

        self._axis_configs = axis_configs
        self._axis_ids = list(axis_ids)
        self._segment_duration_s = max(self.MIN_SEGMENT_TIME_S, min(self.MAX_SEGMENT_TIME_S, segment_duration_s))
        if explicit_target_hz is None:
            max_hz = 0.0
            if self._axis_configs:
                max_hz = max(config.ramp.target_hz for config in self._axis_configs)
            self._target_buffer_time_s = self._safe_buffer_time_s(target_buffer_time_s, max_hz)
        else:
            self._target_buffer_time_s = self._safe_buffer_time_s(target_buffer_time_s, explicit_target_hz)
        self._min_buffer_time_s = min(self.MIN_BUFFER_TIME_S, self._target_buffer_time_s * 0.5)

        self._inflight: deque[tuple[MultiAxisSegment, int]] = deque()
        self._buffered_time_s = 0.0
        self._last_sent_motion_seq = -1
        self._last_sent_transport_seq = -1
        self._planner_under_pressure = False  # True while planner_queue_free < threshold
        self._buffered_segments = 0           # SEGMENT_QUEUE_DEPTH - planner_queue_free
        self._current_steps_per_segment = 0  # updated per-segment; drives required_lookahead()
        if explicit_target_hz is not None:
            self._current_steps_per_segment = max(1, int(round(explicit_target_hz * self._segment_duration_s)))
        self._prefilling = False              # suppresses pressure gate during initial prefill
        # Premature-completion detection
        self._last_confirmed_sequence: int = -1
        self._premature_notify_count: int = 0
        self._premature_notify_window_start: float = 0.0
        self._last_sequence_advance_time: float = time.time()
        self._last_sequence_advance_value: int = -1
        self._stall_timeout_s: float = 2.0  # stall if no progress for 2s

        self._sync_with_firmware_status()
        start_sequence = (
            (self._last_confirmed_sequence + 1) & 0xFFFF
            if self._last_confirmed_sequence >= 0
            else 0
        )
        if self._axis_configs:
            self._generator = iter(
                MultiAxisSegmentGenerator(
                    self._axis_configs,
                    segment_duration_s=self._segment_duration_s,
                    start_sequence=start_sequence,
                )
            )
        else:
            self._generator = iter(())
        self._generator_finished = False

    # -- Helpers ---------------------------------------------------------------

    @property
    def buffered_segments(self) -> int:
        """Number of segments currently buffered in the planner→executor queue.

        Computed from the last received planner_queue_free field:
            buffered = SEGMENT_QUEUE_DEPTH - planner_queue_free

        This mirrors Klipper's "move queue available" check: when buffered_segments
        approaches SEGMENT_QUEUE_DEPTH the host should stop requesting more motion.
        Value is 0 when no status has been received yet.
        """
        return self._buffered_segments

    def _planner_queue_free(self, status) -> int:
        """Return planner_queue_free from status, defaulting to full if absent."""
        return int(getattr(status, "planner_queue_free", self.SEGMENT_QUEUE_DEPTH))

    def _check_planner_pressure(self, status) -> bool:
        """Return True (blocked) when the ESP32 planner buffer already has enough lookahead.

        Uses Klipper's move-queue model: send if buffered < needed, not if free > threshold.
        During initial prefill (_prefilling=True) the gate is bypassed entirely so
        the host can fill up to the speed-appropriate prefill target without interference.

        Logs edge transitions:
          - 'planner pressure' when planner_queue_free drops below 16
          - 'planner recovered' when planner_queue_free recovers above 64
        """
        pqf = self._planner_queue_free(status)
        self._buffered_segments = self.SEGMENT_QUEUE_DEPTH - pqf

        if pqf < 16 and not self._planner_under_pressure:
            self._planner_under_pressure = True
            logger.debug("planner pressure: planner_queue_free=%s (< 16)", pqf)
        elif pqf > 64 and self._planner_under_pressure:
            self._planner_under_pressure = False
            logger.debug("planner recovered: planner_queue_free=%s (> 64)", pqf)

        # During prefill we bypass the pressure gate so the host can seed a deep buffer.
        if self._prefilling:
            return False

        # Klipper model: block if the buffer already holds the required lookahead depth.
        # This inverts the old "send if free slots >= threshold" gate: we now gate on
        # buffered depth rather than remaining free space, which is speed-aware.
        needed = self.required_lookahead(self._current_steps_per_segment)
        return self._buffered_segments >= needed

    def _max_segments_per_cycle(self) -> int:
        """Speed-dependent send cap per polling cycle.

        At low speed (few steps/segment) the defer ring on the ESP32 cannot
        overflow (each segment contributes <10 ring entries) so a higher cap
        is safe and necessary to keep the ring fed between underruns.
        At high speed a lower cap prevents burst-after-throttle overflow.

          < 10  steps/segment → 16 segments/cycle (low speed, 50 RPM)
          < 50  steps/segment →  8 segments/cycle (mid speed)
          >= 50 steps/segment →  4 segments/cycle (high speed, > 200 RPM)
        """
        if self._current_steps_per_segment < 10:
            return 16
        elif self._current_steps_per_segment < 50:
            return 8
        else:
            return 4

    def _max_inflight_segments(self) -> int:
        """Speed-dependent in-flight segment cap.

        At low speed each segment executes slowly so more can be in-flight
        simultaneously without risking the host advancing too far ahead
        of the motor's actual position.
        """
        if self._current_steps_per_segment < 10:
            return 96
        elif self._current_steps_per_segment < 50:
            return 48
        else:
            return 24

    def _timestamp(self) -> str:
        now = time.time()
        seconds = int(now)
        milliseconds = int((now - seconds) * 1000)
        return time.strftime(f"%H:%M:%S.{milliseconds:03d}", time.localtime(now))

    def _safe_buffer_time_s(self, requested_time_s: float, max_hz: float) -> float:
        requested_time_s = max(self.MIN_BUFFER_TIME_S, min(self.MAX_BUFFER_TIME_S, requested_time_s))
        if max_hz <= 0.0:
            return requested_time_s
        safe_time_s = (self.STEP_RING_CAPACITY * self.RING_BUFFER_HEADROOM) / max_hz
        return max(self.MIN_BUFFER_TIME_S, min(requested_time_s, safe_time_s))

    def set_generator(self, generator: Iterator[MultiAxisSegment]) -> None:
        """Override the segment generator for this streamer.

        Call before stream_all() when the segments are produced externally
        (e.g. by a WoundMove or RampMove).
        """
        self._generator = generator
        self._generator_finished = False

    def _sync_with_firmware_status(self) -> None:
        """Synchronize stream state with the ESP32's last executed sequence."""
        try:
            status = self._transport.get_status()
        except Exception:
            return

        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return

        self._last_confirmed_sequence = received_sequence
        self._last_sequence_advance_value = received_sequence
        self._last_sequence_advance_time = time.time()

    def _enable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, status = self._transport.set_axis_enabled_request(axis_id, True)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"enable axis {axis_id} failed with result=0x{status.last_result:02X}")

    def _disable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, status = self._transport.set_axis_enabled_request(axis_id, False)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"disable axis {axis_id} failed with result=0x{status.last_result:02X}")

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
            if sequence_is_less_equal(segment.sequence, last_executed):
                self._buffered_time_s -= segment.duration_us / 1_000_000.0
                self._buffered_time_s = max(0.0, self._buffered_time_s)
                self._inflight.popleft()
            else:
                break

    def _check_premature_completion(self, status) -> None:
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return

        # True premature: ESP32 reports completion of a seq we never sent.
        if self._last_sent_motion_seq >= 0 and sequence_is_greater(received_sequence, self._last_sent_motion_seq):
            now = time.time()
            if now - self._premature_notify_window_start > 1.0:
                self._premature_notify_count = 0
                self._premature_notify_window_start = now
            self._premature_notify_count += 1
            logger.warning(
                "premature completion: got seq=%s but last sent=%s — ESP32 reported completion before host sent this segment (count=%s)",
                received_sequence,
                self._last_sent_motion_seq,
                self._premature_notify_count,
            )

        # Advance confirmed pointer only when sequence strictly increases.
        if self._last_confirmed_sequence < 0 or sequence_is_greater(
            received_sequence, self._last_confirmed_sequence
        ):
            self._last_confirmed_sequence = received_sequence

    def _check_stall(self, status) -> bool:
        """Return True and request stop if last_executed_sequence has not
        advanced for _stall_timeout_s while segments are in flight.

        A stall means the RMT is dead and pushBlock() is likely deadlocked.
        Requesting a stop+flush allows the host to recover gracefully.
        """
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return False
        if not self._inflight:
            # No in-flight segments — not a stall, just idle.
            self._last_sequence_advance_time = time.time()
            return False

        if self._last_sequence_advance_value < 0 or sequence_is_greater(
            received_sequence, self._last_sequence_advance_value
        ):
            self._last_sequence_advance_value = received_sequence
            self._last_sequence_advance_time = time.time()
            return False

        elapsed = time.time() - self._last_sequence_advance_time
        if elapsed > self._stall_timeout_s:
            logger.warning(
                "motor stall detected: last_executed_sequence=%s unchanged for %.1fs with %s segments in flight — requesting stop and flush",
                received_sequence,
                elapsed,
                len(self._inflight),
            )
            self.request_stop()
            self.request_flush(self._last_sent_motion_seq)
            return True
        return False

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
            logger.debug(
                "[%s] send tx_seq=%s motion_seq=%s duration_us=%s total_steps=%s result=0x%02X",
                event["timestamp_str"],
                transport_seq,
                segment.sequence,
                segment.duration_us,
                event["total_steps"],
                event["last_result"],
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
            and len(self._inflight) < self._max_inflight_segments()
            and not self._queue_full(status)
            and not self._check_planner_pressure(status)
        ):
            try:
                segment = next(self._generator)
            except StopIteration:
                self._generator_finished = True
                break

            if self._last_sent_motion_seq >= 0 and not sequence_is_greater(
                segment.sequence, self._last_sent_motion_seq
            ):
                raise RuntimeError(
                    f"motion sequence not strictly increasing: "
                    f"got {segment.sequence}, last was {self._last_sent_motion_seq}"
                )
            batch.append(segment)
            # Keep speed estimate current so required_lookahead() uses fresh data.
            if segment.steps:
                self._current_steps_per_segment = sum(segment.steps)

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
        """Pre-send segments to seed the ESP32 planner queue before RMT starts.

        The prefill target is speed-dependent:
          - Low speed  (steps_per_segment < 10): 64 segments (half of SEGMENT_QUEUE_DEPTH)
            because the ring drains very fast at low RPM and needs a large head start.
          - Otherwise: required_lookahead(current_steps_per_segment) segments.

        The _prefilling flag is set for the duration of this call so that
        _check_planner_pressure() does not prematurely gate sends before the
        target depth has been reached.

        Returns (total_segments_sent, last_status).
        """
        is_low_speed = self._current_steps_per_segment < 10
        if is_low_speed:
            prefill_target = 64   # half of SEGMENT_QUEUE_DEPTH
        else:
            prefill_target = self.required_lookahead(self._current_steps_per_segment)

        total = 0
        last_status = status
        self._prefilling = True
        try:
            while total < prefill_target:
                result = self._collect_and_send_batch(last_status)
                if result is None:
                    break
                n, last_status = result
                total += n
                if n == 0:  # QUEUE_FULL — firmware can't accept more right now
                    break
        finally:
            self._prefilling = False
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
        axes_enabled = False

        try:
            self._enable_axes()
            axes_enabled = True
            status = self._transport.get_status()

            total_segments, status = self._prefill(status)

            while True:
                if self._stop_requested:
                    if self._flush_sequence_requested is not None:
                        self.flush_until(self._flush_sequence_requested)
                    break

                status = self._transport.get_status()
                self._remove_confirmed_segments(status)
                self._check_premature_completion(status)
                if self._check_stall(status):
                    break

                if self._check_endstop(status):
                    break

                # Rate-limit sends to MAX_SEGMENTS_PER_CYCLE per polling iteration to
                # prevent burst-after-throttle that overflows the ESP32 defer ring.
                cycle_segments_sent = 0
                while not self._generator_finished:
                    if cycle_segments_sent >= self._max_segments_per_cycle():
                        break
                    result = self._collect_and_send_batch(status)
                    if result is None:
                        break
                    n, status = result
                    if n == 0:  # QUEUE_FULL
                        break
                    cycle_segments_sent += n
                    total_segments += n
                    if total_segments % self._print_every == 0:
                        logger.debug(
                            "segments=%s buffered=%.1fms inflight=%s queue_free=%s ring_free=%s underrun=%s",
                            total_segments,
                            self._buffered_time_s * 1000.0,
                            len(self._inflight),
                            status.queue_free_slots,
                            status.ring_free_slots,
                            status.underrun_count,
                        )

                if self._flush_sequence_requested is not None:
                    self.flush_until(self._flush_sequence_requested)
                    self._flush_sequence_requested = None

                if self._generator_finished and not self._inflight:
                    break

                sleep_s = self._should_sleep()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        finally:
            if axes_enabled:
                try:
                    self._disable_axes()
                except Exception as exc:
                    logger.error("failed to disable axes: %s", exc)

        self._write_send_log()
        return total_segments
```

### src/rpi/transport/spi_transport.py
```python
from __future__ import annotations

import re
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


class Esp32SpiTransport:
    """Thin wrapper around spidev using the PickupWinder fixed SPI frame format."""

    def __init__(
        self,
        bus: int | None = None,
        device: int | None = None,
        *,
        device_path: str | None = None,
        speed_hz: int = 4_000_000,
        mode: int = 0,
    ):
        try:
            import spidev  # type: ignore
        except ImportError as exc:  # pragma: no cover - depends on host machine
            raise RuntimeError("spidev module is required on the Raspberry Pi host") from exc

        self._spidev_module = spidev
        self._spi = spidev.SpiDev()
        self._device_path = None

        if device_path is not None:
            self._device_path = device_path
            if hasattr(self._spi, "open_path"):
                self._spi.open_path(device_path)
            else:
                parsed = re.fullmatch(r"/dev/spidev(\d+)\.(\d+)", device_path)
                if parsed is None:
                    raise ValueError(
                        "device_path must be in the form /dev/spidev<bus>.<device>"
                    )
                self._spi.open(int(parsed.group(1)), int(parsed.group(2)))
        elif bus is not None and device is not None:
            self._device_path = f"/dev/spidev{bus}.{device}"
            self._spi.open(bus, device)
        else:
            raise ValueError(
                "Must specify either bus/device or device_path for SPI transport"
            )

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
            f"SPI status poll failed after 5 attempts on {self._device_path}: "
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
```

### src/rpi/motion/segment_generator.py
```python
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Iterator, Tuple

from transport.messages import MultiAxisSegment


@dataclass(slots=True)
class AxisStepProfile:
    axis_index: int
    step_at: Callable[[float], float]
    reverse_direction: bool = False
    total_duration: float = 0.0


class BaseSegmentGenerator(ABC):
    """Common iteration behavior for multi-axis segment generators."""

    def __init__(self, *, segment_duration_s: float = 0.004, start_sequence: int = 0) -> None:
        self.segment_duration_s = max(0.002, min(0.005, segment_duration_s))
        self._sequence = start_sequence & 0xFFFF
        self._time_cursor = 0.0
        self.overall_duration = 0.0

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        while self._time_cursor < self.overall_duration:
            next_cursor = min(self._time_cursor + self.segment_duration_s, self.overall_duration)
            duration_s = next_cursor - self._time_cursor
            duration_us = int(round(duration_s * 1_000_000))

            steps, directions = self._compute_segment(self._time_cursor, next_cursor)

            yield MultiAxisSegment(
                sequence=self._sequence,
                duration_us=duration_us,
                steps=steps,
                directions=directions,
            )
            self._sequence = (self._sequence + 1) & 0xFFFF
            self._time_cursor = next_cursor

    @abstractmethod
    def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
        """Return the next segment payload for the current time window."""
        ...


class StepProfileSegmentGenerator(BaseSegmentGenerator):
    def __init__(self, axis_profiles: list[AxisStepProfile], *, segment_duration_s: float = 0.004, start_sequence: int = 0) -> None:
        super().__init__(segment_duration_s=segment_duration_s, start_sequence=start_sequence)
        self.axis_profiles = axis_profiles
        self._axis_errors = [0.0 for _ in axis_profiles]
        self._current_steps = [profile.step_at(0.0) for profile in axis_profiles]
        self.overall_duration = max((profile.total_duration for profile in axis_profiles), default=0.0)

    def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
        max_axis = max((profile.axis_index for profile in self.axis_profiles), default=-1)
        steps = [0] * (max_axis + 1)
        directions = [0] * (max_axis + 1)

        for index, profile in enumerate(self.axis_profiles):
            target_steps = profile.step_at(time_end)
            delta_steps = target_steps - self._current_steps[index]
            count = int(round(self._axis_errors[index] + delta_steps))
            self._axis_errors[index] += delta_steps - float(count)
            self._current_steps[index] = target_steps

            is_negative = count < 0
            if is_negative:
                count = abs(count)
            directions[profile.axis_index] = 1 if is_negative ^ profile.reverse_direction else 0
            steps[profile.axis_index] = count

        return steps, directions
```

### src/rpi/motion/ramp_config.py
```python

from dataclasses import dataclass

from .trapezoidal_profile import TrapezoidalMotionProfile


def compute_ramp_times(
    target_rpm: float,
    duration_s: float,
    max_accel_steps_per_s2: float,
    max_decel_steps_per_s2: float,
    steps_per_rev: int,
    start_rpm: float = 0.0,
    min_ramp_s: float = 0.05,
    max_ramp_fraction: float = 0.25,
) -> tuple[float, float, float]:
    """Compute trapezoidal ramp times from machine acceleration limits.

    Implements:  ramp_s = clamp(physics_required, min_ramp_s, duration_s * max_ramp_fraction)

    The *physics_required* time is the minimum duration needed to reach
    ``target_rpm`` from ``start_rpm`` at the given acceleration limit.
    If the limit is zero (unconstrained), ``min_ramp_s`` is used directly.

    Args:
        target_rpm:             Target rotational speed in RPM.
        duration_s:             Total move duration in seconds.
        max_accel_steps_per_s2: Acceleration limit in steps/s².
        max_decel_steps_per_s2: Deceleration limit in steps/s².
        steps_per_rev:          Steps per motor revolution (full × microstep).
        start_rpm:              Initial speed in RPM (default 0).
        min_ramp_s:             Hard floor for accel / decel time (default 0.05 s).
        max_ramp_fraction:      Maximum fraction of ``duration_s`` allowed for
                                each ramp phase (default 0.25, i.e. 25 %).

    Returns:
        Tuple ``(accel_s, cruise_s, decel_s)`` that sum to at most ``duration_s``.
    """
    start_hz: float = start_rpm / 60.0 * float(steps_per_rev)
    target_hz: float = target_rpm / 60.0 * float(steps_per_rev)
    delta_hz: float = max(target_hz - start_hz, 0.0)

    if max_accel_steps_per_s2 > 0.0:
        physics_accel_s = delta_hz / max_accel_steps_per_s2
    else:
        physics_accel_s = min_ramp_s

    if max_decel_steps_per_s2 > 0.0:
        physics_decel_s = delta_hz / max_decel_steps_per_s2
    else:
        physics_decel_s = min_ramp_s

    cap = duration_s * max_ramp_fraction
    accel_s = max(min_ramp_s, min(physics_accel_s, cap))
    decel_s = max(min_ramp_s, min(physics_decel_s, cap))
    cruise_s = max(duration_s - accel_s - decel_s, 0.0)
    return accel_s, cruise_s, decel_s




@dataclass(slots=True)
class RampConfig:
    axis_id: int = 0
    steps_per_rev: int = 200 * 32
    start_rpm: float = 0.0
    target_rpm: float = 1000.0
    accel_s: float = 10.0
    cruise_s: float = 3.0
    decel_s: float = 10.0
    # Must match RMT_STEP_RESOLUTION_HZ in stepper_driver.h (80 MHz).
    resolution_hz: int = 80_000_000
    reverse_direction: bool = False
    phase_segments: int = 8
    segment_duration_s: float = 0.05

    @property
    def start_hz(self) -> float:
        return self.start_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def target_hz(self) -> float:
        return self.target_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def profile(self) -> TrapezoidalMotionProfile:
        return TrapezoidalMotionProfile(
            start_rpm=self.start_rpm,
            target_rpm=self.target_rpm,
            accel_s=self.accel_s,
            cruise_s=self.cruise_s,
            decel_s=self.decel_s,
        )

    @property
    def total_duration(self) -> float:
        return self.profile.total_duration

    def hz_at_time(self, t: float) -> float:
        return self.profile.rps_at(t) * float(self.steps_per_rev)

    def steps_at(self, t: float) -> float:
        return self.profile.steps_at(t, self.steps_per_rev)

    def step_delta(self, time_start: float, time_end: float) -> float:
        return self.profile.step_delta(time_start, time_end, self.steps_per_rev)

```

### src/rpi/motion/trapezoidal_profile.py
```python
from __future__ import annotations


class TrapezoidalMotionProfile:
    def __init__(
        self,
        start_rpm: float = 0.0,
        target_rpm: float = 1000.0,
        accel_s: float = 0.0,
        cruise_s: float = 0.0,
        decel_s: float = 0.0,
    ) -> None:
        if start_rpm < 0.0 or target_rpm < 0.0:
            raise ValueError("start_rpm and target_rpm must be non-negative")
        if accel_s < 0.0 or cruise_s < 0.0 or decel_s < 0.0:
            raise ValueError("accel_s, cruise_s, and decel_s must be >= 0")

        self.start_rpm = start_rpm
        self.target_rpm = target_rpm
        self.accel_s = accel_s
        self.cruise_s = cruise_s
        self.decel_s = decel_s

    @property
    def total_duration(self) -> float:
        return self.accel_s + self.cruise_s + self.decel_s

    def _clamp_time(self, t: float) -> float:
        return min(max(t, 0.0), self.total_duration)

    def rps_at(self, t: float) -> float:
        t = self._clamp_time(t)
        start_rps = self.start_rpm / 60.0
        target_rps = self.target_rpm / 60.0

        if t < self.accel_s:
            if self.accel_s <= 0.0:
                return target_rps
            rate = (target_rps - start_rps) / self.accel_s
            return start_rps + rate * t

        t -= self.accel_s
        if t < self.cruise_s:
            return target_rps

        t -= self.cruise_s
        if self.decel_s <= 0.0:
            return target_rps
        rate = (target_rps - start_rps) / self.decel_s
        return max(target_rps - rate * t, 0.0)

    def rpm_at(self, t: float) -> float:
        return self.rps_at(t) * 60.0

    def turns_at(self, t: float) -> float:
        t = self._clamp_time(t)
        start_rps = self.start_rpm / 60.0
        target_rps = self.target_rpm / 60.0

        if t < self.accel_s:
            if self.accel_s <= 0.0:
                return target_rps * t
            rate = (target_rps - start_rps) / self.accel_s
            return start_rps * t + 0.5 * rate * t * t

        turns = 0.0
        if self.accel_s > 0.0:
            rate = (target_rps - start_rps) / self.accel_s
            turns += start_rps * self.accel_s + 0.5 * rate * self.accel_s * self.accel_s
        else:
            turns += target_rps * self.accel_s

        t -= self.accel_s
        if t < self.cruise_s:
            return turns + target_rps * t

        turns += target_rps * self.cruise_s
        t -= self.cruise_s
        if self.decel_s <= 0.0:
            return max(turns + target_rps * t, 0.0)

        rate = (target_rps - start_rps) / self.decel_s
        result = turns + target_rps * t - 0.5 * rate * t * t
        return max(result, 0.0)

    def steps_at(self, t: float, steps_per_rev: int) -> float:
        return self.turns_at(t) * float(steps_per_rev)

    def step_delta(self, time_start: float, time_end: float, steps_per_rev: int) -> float:
        return self.steps_at(time_end, steps_per_rev) - self.steps_at(time_start, steps_per_rev)
```

### src/esp32/src/motion_planner.h
```cpp
/**
 * @file motion_planner.h
 * @brief GRBL/Klipper-inspired motion planning layer.
 *
 * ── Architecture role ──────────────────────────────────────────────────────
 *
 *   SPI ingestion (Core 0)              Planner (Core 0)           Executor (Core 1)
 *   ────────────────────                ───────────────            ─────────────────
 *   spiTask → handleFrame()   ──►   s_multi_axis_queue   ──►   plannerTask()
 *                                        (existing)              │
 *                                                                ▼
 *                                                          segment_queue_
 *                                                           (NEW bounded)
 *                                                                │
 *                                                                ▼
 *                                                        executorTask (Core 1)
 *                                                        state machine
 *                                                                │
 *                                                                ▼
 *                                                          RMT ring buffer
 *
 * The planner consumes multi_axis_block_t (bulk blocks from SPI) and
 * decomposes them into individual planned_segment_t entries with Klipper-style
 * monotonic timestamps.  The executor consumes these one at a time through a
 * bounded state machine.
 *
 * ── Backpressure ───────────────────────────────────────────────────────────
 *
 *   segment_queue_ is bounded to SEGMENT_QUEUE_DEPTH.  If the executor is
 *   slow, the planner blocks (with timeout) providing natural backpressure
 *   all the way back to the SPI ingestion queue.
 *
 * ── Flush path ─────────────────────────────────────────────────────────────
 *
 *   On flush, the planner drains both cmd_queue_ and segment_queue_, then
 *   pushes a flush sentinel (is_flush=true) so the executor can reset its
 *   state atomically.
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_err.h>
#include "step_types.h"

// ---------------------------------------------------------------------------
// Planned segment — immutable output of planner, input to executor
// ---------------------------------------------------------------------------

/**
 * @brief Per-axis motion within a planned segment.
 */
typedef struct {
    uint16_t step_count;
    bool     direction;
} planned_axis_motion_t;

/**
 * @brief One fully-planned segment ready for execution.
 *
 * Produced by the planner task, consumed by the executor task.
 * Immutable after enqueue — no synchronisation needed beyond the queue.
 */
typedef struct {
    uint16_t              motion_sequence;     ///< Host-assigned sequence ID
    uint16_t              duration_us;         ///< Wall-clock duration
    int64_t               scheduled_time_us;   ///< Monotonic execution timestamp (Klipper-style)
    uint8_t               axis_count;
    uint8_t               axis_ids[MULTI_AXIS_MAX_AXES];
    planned_axis_motion_t axes[MULTI_AXIS_MAX_AXES];
    bool                  is_flush;            ///< True = flush sentinel, not a real segment
    uint16_t              flush_sequence;      ///< Valid only when is_flush == true
} planned_segment_t;

// ---------------------------------------------------------------------------
// Executor state machine
// ---------------------------------------------------------------------------

/**
 * @brief Executor FSM states.
 *
 * The state machine ensures bounded CPU usage per iteration and eliminates
 * the nested drain loops that caused watchdog resets.
 *
 *   IDLE ──► FETCH ──► DRAIN ──► RUN ──► (back to IDLE)
 *              │                           ▲
 *              ▼                           │
 *            FLUSH ────────────────────────┘
 *              │
 *              ▼
 *           RECOVERY ──────────────────────┘
 */
enum class ExecState : uint8_t {
    IDLE,       ///< Waiting for segments (blocking queue receive)
    FETCH,      ///< Pulling segment(s) from planner queue (non-blocking batch)
    DRAIN,      ///< Writing steps to RMT ring buffer
    RUN,        ///< KickStart RMT, fire deferred notifications
    FLUSH,      ///< Processing flush sentinel — reset pipeline
    RECOVERY,   ///< Recovering from RMT underrun / error
};

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/** Segment queue: planner → executor.  ~512 ms lookahead at 4 ms/segment. */
static constexpr uint32_t SEGMENT_QUEUE_DEPTH = 128;

/** Minimum segments buffered before executor begins first RMT kickStart. */
static constexpr uint32_t SEGMENT_PREFILL_THRESHOLD = 16;

/** Maximum segments the executor fetches per FETCH iteration.
 *  Set to 1 to force frequent yields and allow planner to refill. */
static constexpr uint32_t EXEC_BATCH_LIMIT = 16;

/** Time budget per executor iteration in microseconds (watchdog safe). */
static constexpr int64_t  EXEC_TIME_BUDGET_US = 3000;

/** Planner tuning: time budget and per-iteration limit (watchdog-safe).
 *  200 µs budget allows processing 32+ segments per loop iteration at 5-10 µs/segment.
 *  Non-blocking xQueueSend ensures no watchdog blocking despite higher throughput.
 *  Higher batch size prevents executor starvation when planner runs infrequently.
 */
static constexpr int64_t  PLANNER_TIME_BUDGET_US = 2000; // µs per planner loop
static constexpr uint32_t PLANNER_MAX_SEGMENTS_PER_ITER = 60; // segments per loop to balance yield

// ---------------------------------------------------------------------------
// MotionPlanner class
// ---------------------------------------------------------------------------

class MotionPlanner {
public:
    MotionPlanner();

    /**
     * @brief Initialise the segment output queue and launch the planner task.
     *
     * @param cmd_queue   Existing s_multi_axis_queue (SPI → planner input).
     * @param flush_queue Existing s_flush_queue (SPI → planner input).
     * @return ESP_OK on success.
     */
    esp_err_t init(QueueHandle_t cmd_queue, QueueHandle_t flush_queue);

    /** @brief Output queue handle for the executor to consume. */
    QueueHandle_t segmentQueue() const { return segment_queue_; }

    /** @brief Number of free slots in the segment output queue. */
    uint32_t segmentQueueFree() const;

    // ── Statistics ──────────────────────────────────────────────────────────
    uint32_t segmentsPlanned() const { return segments_planned_; }
    uint32_t segmentsDropped() const { return segments_dropped_; }

private:
    QueueHandle_t cmd_queue_     {nullptr};  ///< Input: s_multi_axis_queue
    QueueHandle_t flush_queue_   {nullptr};  ///< Input: s_flush_queue
    QueueHandle_t segment_queue_ {nullptr};  ///< Output: planned_segment_t

    int64_t  timeline_us_       {0};         ///< Monotonic scheduling timeline
    uint32_t segments_planned_  {0};
    uint32_t segments_dropped_  {0};

    // ── Incremental planner state (to avoid burst processing) ───────────
    multi_axis_block_t pending_block_ {};   ///< Currently-being-expanded block
    uint16_t            pending_segment_idx_ {0};
    bool                has_pending_block_   {false};

    // Non-blocking flush sentinel retry state
    bool     flush_pending_ {false};
    uint16_t pending_flush_sequence_ {0};

    /**
     * @brief Expand one multi_axis_block_t into planned_segment_t entries.
     *
     * Each segment in the block becomes one planned_segment_t with a
     * Klipper-style monotonic timestamp derived from cumulative duration.
     * Enqueues to segment_queue_ with bounded backpressure wait.
     */
    void planBlock(const multi_axis_block_t& block);

    /**
     * @brief Handle a flush request: drain queues, push flush sentinel.
     */
    void handleFlush(const flush_request_t& req);

    /**
     * @brief Planner task body.
     *
     * Pinned to Core 0, priority 8 (below SPI task at 10, above idle).
     * Runs in SPI task's idle time between spi_slave_transmit() calls.
     */
    static void plannerTask(void* arg);
};
```

### src/esp32/src/motion_planner.cpp
```cpp
/**
 * @file motion_planner.cpp
 * @brief GRBL/Klipper-inspired motion planning layer — implementation.
 *
 * The planner task sits between SPI ingestion and the execution layer:
 *
 *   s_multi_axis_queue ──► plannerTask() ──► segment_queue_ ──► executor
 *
 * Responsibilities:
 *   • Decompose multi_axis_block_t (bulk) into individual planned_segment_t
 *   • Assign Klipper-style monotonic timestamps (scheduled_time_us)
 *   • Handle flush requests: drain both input and output queues
 *   • Provide bounded backpressure to the SPI ingestion layer
 *
 * Non-responsibilities (executor owns these):
 *   • No hardware access (no RMT, no GPIO, no ring buffer)
 *   • No endstop checking (real-time, must be in executor)
 *   • No StepperDriver interaction
 *
 * This separation means the planner is fully preemptible and testable
 * without hardware dependencies.
 */

#include "motion_planner.h"

#include <string.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>

static const char* TAG = "planner";

// ---------------------------------------------------------------------------
// Task parameters
// ---------------------------------------------------------------------------

static constexpr uint32_t    PLANNER_STACK = 4096;
static constexpr UBaseType_t PLANNER_PRIO  = 12;  // Above SPI (10), above idle
static constexpr BaseType_t  PLANNER_CORE  = 0;   // Same core as SPI task

/** Max blocks drained from cmd_queue_ during a single flush operation. */
static constexpr uint32_t MAX_FLUSH_DRAIN = 16;

/** Backpressure timeout: how long the planner waits for segment_queue_ space
 *  before dropping a segment.  10 ms is ~2.5 segments at 4 ms/segment. */
static constexpr TickType_t BACKPRESSURE_TIMEOUT = pdMS_TO_TICKS(10);

/** Planner poll interval when no command blocks are available. */
static constexpr TickType_t CMD_POLL_TIMEOUT = pdMS_TO_TICKS(5);

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MotionPlanner::MotionPlanner() {}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t MotionPlanner::init(QueueHandle_t cmd_queue, QueueHandle_t flush_queue)
{
    cmd_queue_   = cmd_queue;
    flush_queue_ = flush_queue;

    segment_queue_ = xQueueCreate(SEGMENT_QUEUE_DEPTH, sizeof(planned_segment_t));
    ESP_RETURN_ON_FALSE(segment_queue_ != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create segment queue");

    BaseType_t rc = xTaskCreatePinnedToCore(
        &MotionPlanner::plannerTask,
        "planner",
        PLANNER_STACK,
        this,
        PLANNER_PRIO,
        nullptr,
        PLANNER_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create planner task");

    ESP_LOGI(TAG, "planner ready: seg_queue_depth=%lu  core=%d  pri=%d",
             (unsigned long)SEGMENT_QUEUE_DEPTH,
             (int)PLANNER_CORE, (int)PLANNER_PRIO);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// segmentQueueFree()
// ---------------------------------------------------------------------------

uint32_t MotionPlanner::segmentQueueFree() const
{
    if (segment_queue_ == nullptr) return 0;
    return static_cast<uint32_t>(uxQueueSpacesAvailable(segment_queue_));
}

// ---------------------------------------------------------------------------
// planBlock()
// ---------------------------------------------------------------------------

void MotionPlanner::planBlock(const multi_axis_block_t& block)
{
    // Legacy path kept for compatibility: copy into pending buffer so the
    // real work happens incrementally inside plannerTask.  This avoids
    // burst CPU usage and guarantees non-blocking behavior.
    if (!has_pending_block_) {
        pending_block_ = block;
        pending_segment_idx_ = 0;
        has_pending_block_ = true;
        // Ensure timeline is clamped to now if it drifted into the past.
        const int64_t now_us = esp_timer_get_time();
        if (timeline_us_ < now_us) timeline_us_ = now_us;
    } else {
        // Already processing a block — drop this incoming block (should be
        // rare because the SPI layer back-pressures). Count as dropped.
        ++segments_dropped_;
        ESP_LOGW(TAG, "incoming block dropped: planner busy (dropped total=%lu)",
                 (unsigned long)segments_dropped_);
    }
}

// ---------------------------------------------------------------------------
// handleFlush()
// ---------------------------------------------------------------------------

void MotionPlanner::handleFlush(const flush_request_t& req)
{
    // Non-blocking flush: mark pending state and drop any currently stored
    // pending_block_.  We will attempt to push a flush sentinel to the
    // output queue without blocking; if that fails we remember the flush
    // and retry on the next planner loop iteration.
    has_pending_block_ = false; // drop current pending block
    timeline_us_ = esp_timer_get_time();
    planned_segment_t flush_seg {};
    flush_seg.is_flush = true;
    flush_seg.flush_sequence = req.flush_sequence;
    if (xQueueSend(segment_queue_, &flush_seg, 0) == pdTRUE) {
        flush_pending_ = false;
        ESP_LOGI(TAG, "flush sentinel posted seq=%u", req.flush_sequence);
    } else {
        // Queue full — remember to retry later.
        flush_pending_ = true;
        pending_flush_sequence_ = req.flush_sequence;
        ESP_LOGW(TAG, "flush sentinel queued later seq=%u", req.flush_sequence);
    }
}

// ---------------------------------------------------------------------------
// plannerTask()
// ---------------------------------------------------------------------------

void MotionPlanner::plannerTask(void* arg)
{
    auto* self = static_cast<MotionPlanner*>(arg);
    multi_axis_block_t block;
    flush_request_t flush_req;

    ESP_LOGI(TAG, "planner task running on core %d", xPortGetCoreID());

    for (;;) {
        const int64_t loop_start = esp_timer_get_time();

        // 1) Handle any flush requests immediately (non-blocking).
        if (xQueueReceive(self->flush_queue_, &flush_req, 0) == pdTRUE) {
            self->handleFlush(flush_req);
        }

        // 2) If a previous flush sentinel failed to post, retry non-blocking.
        if (self->flush_pending_) {
            planned_segment_t flush_seg {};
            flush_seg.is_flush = true;
            flush_seg.flush_sequence = self->pending_flush_sequence_;
            if (xQueueSend(self->segment_queue_, &flush_seg, 0) == pdTRUE) {
                self->flush_pending_ = false;
                ESP_LOGI(TAG, "flush sentinel posted retry seq=%u",
                         (unsigned)self->pending_flush_sequence_);
            }
        }

        // 3) If we don't have a pending block, try to pull one non-blocking.
        if (!self->has_pending_block_) {
            if (xQueueReceive(self->cmd_queue_, &block, 0) == pdTRUE) {
                // Store for incremental expansion.
                self->pending_block_ = block;
                self->pending_segment_idx_ = 0;
                self->has_pending_block_ = true;
                // Clamp timeline if idle.
                const int64_t now_us = esp_timer_get_time();
                if (self->timeline_us_ < now_us) self->timeline_us_ = now_us;
            }
        }

        // 4) Process up to PLANNER_MAX_SEGMENTS_PER_ITER segments from the
        //    pending_block_ within the time budget. All queue sends are
        //    non-blocking (timeout=0); on failure we drop the segment and
        //    advance so the planner never stalls.
        uint32_t processed = 0;
        while (self->has_pending_block_ &&
               processed < PLANNER_MAX_SEGMENTS_PER_ITER &&
               (esp_timer_get_time() - loop_start) < PLANNER_TIME_BUDGET_US) {

            const uint16_t idx = self->pending_segment_idx_;
            if (idx >= self->pending_block_.segment_count) {
                // Finished this block.
                self->has_pending_block_ = false;
                break;
            }

            const multi_axis_segment_t& src = self->pending_block_.segments[idx];

            planned_segment_t seg {};
            seg.motion_sequence = src.motion_sequence;
            seg.duration_us     = src.duration_us;
            seg.scheduled_time_us = self->timeline_us_;
            seg.axis_count      = self->pending_block_.axis_count;
            seg.is_flush        = false;

            const uint8_t n = (self->pending_block_.axis_count < MULTI_AXIS_MAX_AXES)
                              ? self->pending_block_.axis_count : MULTI_AXIS_MAX_AXES;
            for (uint8_t a = 0; a < n; ++a) {
                seg.axis_ids[a]        = self->pending_block_.axis_ids[a];
                seg.axes[a].step_count = src.step_counts[a];
                seg.axes[a].direction  = ((src.direction_mask >> a) & 1u) != 0;
            }

            // Advance timeline and enqueue atomically: only advance if the
            // segment was successfully enqueued so the timeline stays in sync.
            // On full queue, break out of the inner loop — the executor will
            // drain a slot, and we will retry this segment on the next
            // planner iteration (natural backpressure, no data loss).
            if (xQueueSend(self->segment_queue_, &seg, 0) == pdTRUE) {
                self->timeline_us_ += static_cast<int64_t>(src.duration_us);
                ++self->segments_planned_;
                ++self->pending_segment_idx_;
                ++processed;
            } else {
                // Queue full — yield and retry this segment next iteration.
                break;
            }
        }

        // 5) Yield behavior: if we processed nothing, sleep briefly to let
        //    IDLE0 and other low-priority tasks run and reset the watchdog.
        if (processed == 0) {
            vTaskDelay(1);
        } else {
            taskYIELD();
        }
    }
}
```

### src/esp32/src/stepper_driver.cpp
```cpp
/**
 * @file stepper_driver.cpp
 * @brief Physical-layer RMT stepper driver — streaming simple_encoder impl.
 *
 * See stepper_driver.h for architecture details.
 */

#include "stepper_driver.h"

#include <algorithm>
#include <cstring>
#include <esp_log.h>
#include <esp_check.h>
#include <hal/gpio_ll.h>

static const char* TAG = "stepper_driver";

// ---------------------------------------------------------------------------
// encode_steps() — simple_encoder callback, runs in ISR context (IRAM)
// ---------------------------------------------------------------------------
//
// RMT clock: 80 MHz (1 tick = 12.5 ns).
// Called by the RMT driver whenever it needs more symbols. Reads up to
// PART_SIZE=16 entries from the ring buffer and converts each to one
// rmt_symbol_word_t with a FastAccelStepper-style balanced pulse:
//   HIGH = ticks / 2         (rounded down)
//   LOW  = ticks − HIGH
// Both halves are clamped to >= RMT_STEP_PULSE_TICKS (8) = 100 ns,
// which meets A4988/DRV8825 STEP pulse width requirements.
//
// Direction changes:
//   If a ring entry has toggle_dir=1 and the previous chunk contained steps,
//   emit a pause chunk first (to meet driver IC setup time), then toggle
//   DIR on the next callback invocation.
//
// On starvation, the callback follows the same conservative policy as
// FastAccelStepper's ESP32 IDF5 backend: emit one LOW-level pause chunk,
// arm stop, and let the next callback finish the transaction.

extern "C" size_t IRAM_ATTR encode_steps(const void* /*data*/,
                                          size_t /*data_size*/,
                                          size_t /*symbols_written*/,
                                          size_t symbols_free,
                                          rmt_symbol_word_t* symbols,
                                          bool* done, void* arg)
{
    StepperDriver* drv = static_cast<StepperDriver*>(arg);
    *done = false;

    if (symbols_free < PART_SIZE) {
        return 0;  // Wait for more space
    }

    uint32_t rd = drv->ring_read_.load(std::memory_order_acquire);
    uint32_t wr = drv->ring_write_.load(std::memory_order_acquire);

    // Check for explicit stop request
    if (drv->rmt_stopped_.load(std::memory_order_relaxed)) {
        *done = true;
        return 0;
    }

    // Check for endstop trigger — stop immediately without emitting any
    // further step pulses. The ring is NOT reset here; emergencyStop() is
    // called from the executor task after it detects endstop_active_.
    if (drv->endstop_active_.load(std::memory_order_relaxed)) {
        drv->rmt_stopped_.store(true, std::memory_order_relaxed);
        *done = true;
        return 0;
    }

    // Ring empty — emit one LOW-level pause chunk, arm stop, and let the
    // next callback terminate the transmission.
    if (rd == wr) {
        drv->last_chunk_had_steps_ = false;
        drv->ring_underrun_count_.fetch_add(1, std::memory_order_relaxed);
        drv->rmt_stopped_.store(true, std::memory_order_relaxed);
        uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
        for (uint32_t i = 0; i < PART_SIZE; i++) {
            symbols[i].level0    = 0;
            symbols[i].duration0 = t;
            symbols[i].level1    = 0;
            symbols[i].duration1 = t;
        }
        return PART_SIZE;
    }

    // Data is available after underrun — clear the stop flag so we can continue
    // encoding. This handles the case where the ring was empty, we emitted a
    // pause, and now new data has arrived before on_trans_done_isr fires.
    drv->rmt_stopped_.store(false, std::memory_order_relaxed);

    // Peek at next entry — check for direction change
    ring_entry_t* entry = &drv->ring_[rd & STEP_RING_MASK];
    if (entry->toggle_dir) {
        if (drv->last_chunk_had_steps_) {
            // Previous chunk had steps — emit a fixed pause chunk so the next
            // callback can toggle DIR safely at the chunk boundary.
            drv->last_chunk_had_steps_ = false;
            uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
            for (uint32_t i = 0; i < PART_SIZE; i++) {
                symbols[i].duration0 = t;
                symbols[i].level0    = 0;
                symbols[i].duration1 = t;
                symbols[i].level1    = 0;
            }
            return PART_SIZE;
        }
        // Safe to toggle now (previous chunk was a pause or first chunk)
        gpio_ll_set_level(&GPIO, drv->dir_pin_,
                          gpio_ll_get_level(&GPIO, drv->dir_pin_) ^ 1);
        entry->toggle_dir = 0;
    }

    // Fill PART_SIZE symbols from ring buffer
    bool has_steps = false;
    for (uint32_t i = 0; i < PART_SIZE; i++) {
        if (rd != wr) {
            ring_entry_t* e = &drv->ring_[rd & STEP_RING_MASK];

            // Handle mid-chunk direction changes: stop filling, pad the
            // remainder with a fixed LOW-level pause chunk.
            if (e->toggle_dir && i > 0) {
                uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
                for (uint32_t j = i; j < PART_SIZE; j++) {
                    symbols[j].duration0 = t;
                    symbols[j].level0    = 0;
                    symbols[j].duration1 = t;
                    symbols[j].level1    = 0;
                }
                break;
            }

            uint16_t t = e->ticks;
            uint16_t high_ticks = t >> 1;
            uint16_t low_ticks = t - high_ticks;
            if (high_ticks < RMT_STEP_PULSE_TICKS) {
                high_ticks = RMT_STEP_PULSE_TICKS;
                low_ticks = t - high_ticks;
            }
            if (low_ticks < RMT_STEP_PULSE_TICKS) {
                low_ticks = RMT_STEP_PULSE_TICKS;
                high_ticks = t - low_ticks;
            }
            drv->last_ticks_ = t;
            symbols[i].level0    = 1;
            symbols[i].duration0 = high_ticks;
            symbols[i].level1    = 0;
            symbols[i].duration1 = low_ticks;

            rd++;
            has_steps = true;
        } else {
            // Ring exhausted mid-chunk — pad the remainder with a pause chunk,
            // arm stop, and let the next callback terminate the transaction.
            drv->ring_underrun_count_.fetch_add(1, std::memory_order_relaxed);
            drv->rmt_stopped_.store(true, std::memory_order_relaxed);
            uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
            for (uint32_t j = i; j < PART_SIZE; j++) {
                symbols[j].level0    = 0;
                symbols[j].duration0 = t;
                symbols[j].level1    = 0;
                symbols[j].duration1 = t;
            }
            break;
        }
    }

    drv->ring_read_.store(rd, std::memory_order_release);
    drv->last_chunk_had_steps_ = has_steps;

    TaskHandle_t prod = drv->producer_task_.load(std::memory_order_relaxed);
    TaskHandle_t exec = drv->executor_task_.load(std::memory_order_relaxed);
    if (prod != nullptr || exec != nullptr) {
        BaseType_t woken = pdFALSE;
        if (prod != nullptr) {
            vTaskNotifyGiveFromISR(prod, &woken);
        }
        if (exec != nullptr && exec != prod) {
            vTaskNotifyGiveFromISR(exec, &woken);
        }
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    return PART_SIZE;
}

// ---------------------------------------------------------------------------
// endstopIsrHandler()  — GPIO ISR, IRAM_ATTR
// ---------------------------------------------------------------------------
//
// Fires on any edge of either endstop contact (NO or NC).
// Validates the dual-contact NO/NC logic to guard against noise and cable breaks:
//   NO=0, NC=1 → endstop CLOSED (triggered) → set endstop_active_
//   NO=1, NC=0 → endstop OPEN  (released)   → clear endstop_active_
//   NO==NC      → ABSENT or cable break       → fail-safe: set endstop_active_
//
// arg = StepperDriver* (owns all needed state — no CommInterface dependency).

void IRAM_ATTR StepperDriver::endstopIsrHandler(void* arg)
{
    StepperDriver* drv = static_cast<StepperDriver*>(arg);

    if (!drv->isEndstopArmed()) {
        return;
    }

    const int no_lvl = gpio_get_level(drv->endstop_no_pin_);
    const int nc_lvl = gpio_get_level(drv->endstop_nc_pin_);

    const bool triggered = (no_lvl == 0 && nc_lvl == 1) || (no_lvl == nc_lvl);

    if (triggered) {
        drv->endstop_active_.store(true, std::memory_order_release);
        // Wake the executor task so it drains the pipeline immediately.
        BaseType_t woken = pdFALSE;
        TaskHandle_t exec = drv->executor_task_.load(std::memory_order_relaxed);
        if (exec != nullptr) {
            vTaskNotifyGiveFromISR(exec, &woken);
        }
        if (woken) portYIELD_FROM_ISR();
    } else {
        // Endstop released — clear flag.
        // Host must re-arm via SPI ENABLE_ENDSTOP before the next move.
        drv->endstop_active_.store(false, std::memory_order_release);
    }
}

// ---------------------------------------------------------------------------
// initEndstopIsr()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::initEndstopIsr(gpio_num_t no_pin, gpio_num_t nc_pin)
{
    if (no_pin == GPIO_NUM_NC || nc_pin == GPIO_NUM_NC) {
        ESP_LOGI(TAG, "motor%u: endstop pins not configured — ISR not installed",
                 motor_id_);
        return ESP_OK;
    }

    endstop_no_pin_ = no_pin;
    endstop_nc_pin_ = nc_pin;

    // gpio_install_isr_service returns ESP_ERR_INVALID_STATE if already called.
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "motor%u: gpio_install_isr_service failed: %s",
                 motor_id_, esp_err_to_name(err));
        return err;
    }

    const gpio_num_t pins[2] = { no_pin, nc_pin };
    for (gpio_num_t pin : pins) {
        ESP_RETURN_ON_ERROR(
            gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE),
            TAG, "gpio_set_intr_type failed for pin %d", (int)pin);
        ESP_RETURN_ON_ERROR(
            gpio_isr_handler_add(pin, &StepperDriver::endstopIsrHandler, this),
            TAG, "gpio_isr_handler_add failed for pin %d", (int)pin);
    }

    ESP_LOGI(TAG, "motor%u: endstop ISR installed NO=GPIO%d NC=GPIO%d",
             motor_id_, (int)no_pin, (int)nc_pin);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

StepperDriver::StepperDriver(gpio_num_t step_pin, gpio_num_t dir_pin,
                             gpio_num_t en_pin,   uint8_t    motor_id)
    : dir_pin_(dir_pin)
    , step_pin_(step_pin)
    , en_pin_(en_pin)
    , motor_id_(motor_id)
{
    memset(ring_, 0, sizeof(ring_));
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::init()
{
    // ── 1. DIR and EN GPIO ──────────────────────────────────────────────────
    gpio_config_t io_conf = {};
    io_conf.mode          = GPIO_MODE_OUTPUT;
    io_conf.intr_type     = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask  = (1ULL << dir_pin_) | (1ULL << en_pin_);
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: gpio_config DIR/EN failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    gpio_set_level(en_pin_,  1);
    gpio_set_level(dir_pin_, last_dir_ ? 1 : 0);

    // ── 2. RMT TX channel ───────────────────────────────────────────────────
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num           = step_pin_;
    tx_cfg.clk_src            = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz      = RMT_STEP_RESOLUTION_HZ;
    tx_cfg.mem_block_symbols  = RMT_MEM_SYMBOLS;
    tx_cfg.trans_queue_depth  = 4;
    tx_cfg.flags.invert_out   = false;
    tx_cfg.flags.with_dma     = false;

    err = rmt_new_tx_channel(&tx_cfg, &channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_new_tx_channel failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    // ── 3. Simple encoder with callback ─────────────────────────────────────
    rmt_simple_encoder_config_t enc_cfg = {};
    enc_cfg.callback       = encode_steps;
    enc_cfg.arg            = this;
    enc_cfg.min_chunk_size = PART_SIZE;

    err = rmt_new_simple_encoder(&enc_cfg, &encoder_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_new_simple_encoder failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    // ── 4. Transmit config ──────────────────────────────────────────────────
    tx_config_.loop_count              = 0;
    tx_config_.flags.eot_level         = 0;
    tx_config_.flags.queue_nonblocking = 1;

    // ── 5. on_trans_done callback ───────────────────────────────────────────
    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = &StepperDriver::on_trans_done_isr;
    err = rmt_tx_register_event_callbacks(channel_, &cbs, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_tx_register_event_callbacks failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    // ── 6. Enable the RMT channel ───────────────────────────────────────────
    err = rmt_enable(channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_enable failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "motor%u: init OK  step=GPIO%d  dir=GPIO%d  en=GPIO%d  "
                  "ring=%u  part=%u",
             motor_id_, (int)step_pin_, (int)dir_pin_, (int)en_pin_,
             STEP_RING_SIZE, PART_SIZE);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// enable / disable / emergencyStop
// ---------------------------------------------------------------------------

void StepperDriver::enable()
{
    gpio_set_level(en_pin_, 0);
    enabled_ = true;
}

void StepperDriver::disable()
{
    gpio_set_level(en_pin_, 1);
    enabled_ = false;
}

void StepperDriver::emergencyStop()
{
    rmt_disable(channel_);
    rmt_enable(channel_);

    ring_read_.store(0, std::memory_order_relaxed);
    ring_write_.store(0, std::memory_order_relaxed);
    rmt_running_.store(false, std::memory_order_relaxed);
    rmt_stopped_.store(true, std::memory_order_relaxed);
    last_chunk_had_steps_ = false;

    ESP_LOGW(TAG, "motor%u: emergency stop", motor_id_);
}

// ---------------------------------------------------------------------------
// stopStream()
// ---------------------------------------------------------------------------

void StepperDriver::stopStream()
{
    if (!rmt_running_.load(std::memory_order_relaxed)) return;

    // Signal the encoder callback to end the transmission
    rmt_stopped_.store(true, std::memory_order_release);

    // Wait for the RMT hardware to finish the current transaction
    rmt_tx_wait_all_done(channel_, pdMS_TO_TICKS(500));
    rmt_running_.store(false, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// startStream()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::startStream()
{
    rmt_running_.store(true, std::memory_order_release);
    rmt_stopped_.store(false, std::memory_order_release);
    last_chunk_had_steps_ = false;

    // `this` is in internal DRAM (static global) — passes esp_ptr_internal()
    // check. sizeof(*this) > 0 passes payload_bytes != 0. The callback ignores
    // both data and data_size entirely. Reset the encoder so each new
    // transaction restarts from symbol position 0.
    encoder_->reset(encoder_);
    esp_err_t err = rmt_transmit(channel_, encoder_, this, sizeof(*this), &tx_config_);
    if (err != ESP_OK) {
        rmt_running_.store(false, std::memory_order_relaxed);
        rmt_stopped_.store(true, std::memory_order_relaxed);
        ESP_LOGE(TAG, "motor%u: rmt_transmit failed: %s",
                 motor_id_, esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------------------------------
// gracefulStop()
// ---------------------------------------------------------------------------

void StepperDriver::gracefulStop()
{
    // Signal the encoder callback to stop after the current ring contents
    // have been consumed (no ring reset, unlike emergencyStop).
    rmt_stopped_.store(true, std::memory_order_release);
    // Do not call rmt_tx_wait_all_done here — the caller should not block.
    // The RMT transaction will end naturally after the pause chunk fires.
    rmt_running_.store(false, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// pushBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::pushBlock(const step_block_t& block, TaskHandle_t caller_task)
{
    if (ring_underrun_count_.load(std::memory_order_relaxed) > 0) {
        ESP_LOGW(TAG, "motor%u: ring underrun x%lu since last pushBlock",
                 motor_id_, (unsigned long)ring_underrun_count_.load(std::memory_order_relaxed));
        ring_underrun_count_.store(0, std::memory_order_relaxed);
    }

    if (block.count == 0) {
        return ESP_OK;
    }

    // Always update producer_task_ unconditionally so the ISR ring-space
    // notification always wakes the task that is actually blocked here,
    // not a stale handle from a previous call.
    producer_task_.store((caller_task != nullptr)
                     ? caller_task
                     : xTaskGetCurrentTaskHandle(),
                     std::memory_order_release);

    const uint32_t count = std::min<uint32_t>(block.count, STEP_BLOCK_SIZE);

    const bool new_dir = block.steps[0].direction;
    bool need_toggle = (new_dir != last_dir_);

    // If the ring is empty and the motor is idle, explicitly set the DIR pin
    // to the requested direction now. This avoids relying on the initial
    // `last_dir_` state and ensures reverse mode is applied on the first block.
    if (!rmt_running_.load(std::memory_order_relaxed) &&
        ring_read_.load(std::memory_order_relaxed) == ring_write_.load(std::memory_order_relaxed) &&
        need_toggle) {
        gpio_set_level(dir_pin_, new_dir ? 1 : 0);
        last_dir_ = new_dir;
        need_toggle = false;
    } else {
        last_dir_ = new_dir;
    }

    // If the endstop already fired before we even start writing, abort.
    if (endstop_active_.load(std::memory_order_acquire)) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint32_t i = 0; i < count; i++) {
        // Back-pressure: wait until the encoder ISR has consumed at least one
        // chunk and notified this producer task. This avoids a CPU1 spin loop
        // and keeps the task watchdog satisfied.
        while (ringFree() == 0) {
            // The endstop ISR also notifies this task.  If it fires while we
            // are blocked here, break out immediately instead of spinning.
            if (endstop_active_.load(std::memory_order_acquire)) {
                return ESP_ERR_INVALID_STATE;
            }
            // If RMT is not running and ring is full, the encoder callback
            // will never fire and ring_read_ will never advance.
            // Kick startStream() directly instead of waiting forever.
            if (!rmt_running_.load(std::memory_order_acquire)) {
                esp_err_t kick_err = startStream();
                if (kick_err != ESP_OK) {
                    ESP_LOGW(TAG, "motor%u: pushBlock kick startStream: %s",
                             motor_id_, esp_err_to_name(kick_err));
                }
            }
            ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(5));
            if (endstop_active_.load(std::memory_order_acquire)) {
                return ESP_ERR_INVALID_STATE;
            }
        }

        uint32_t ticks = block.steps[i].interval_ticks;

        ticks = std::max<uint32_t>(ticks, RMT_STEP_MIN_TICKS);
        ticks = std::min<uint32_t>(ticks, RMT_STEP_MAX_TICKS);

        uint32_t wr = ring_write_.load(std::memory_order_relaxed);
        ring_entry_t* e = &ring_[wr & STEP_RING_MASK];
        e->ticks      = static_cast<uint16_t>(ticks);
        e->toggle_dir = (i == 0 && need_toggle) ? 1 : 0;
        e->pad        = 0;

        ring_write_.store(wr + 1, std::memory_order_release);
    }

    // NOTE: startStream() is NOT called here.
    //
    // The executor task (stepper_queue.cpp) calls startStream() explicitly after
    // draining all available FreeRTOS queue blocks into the ring. This maximises
    // ring fill before the RMT starts, which is critical at high step rates where
    // a single 64-step block lasts less than one SPI round-trip.
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// on_trans_done ISR
// ---------------------------------------------------------------------------

bool IRAM_ATTR StepperDriver::on_trans_done_isr(
    rmt_channel_handle_t /*tx_chan*/,
    const rmt_tx_done_event_data_t* /*edata*/,
    void* user_ctx)
{
    StepperDriver* self = static_cast<StepperDriver*>(user_ctx);
    self->rmt_running_.store(false, std::memory_order_relaxed);
    BaseType_t woken = pdFALSE;
    TaskHandle_t prod = self->producer_task_.load(std::memory_order_relaxed);
    if (prod != nullptr) {
        vTaskNotifyGiveFromISR(prod, &woken);
    }
    TaskHandle_t exec = self->executor_task_.load(std::memory_order_relaxed);
    if (exec != nullptr && exec != prod) {
        vTaskNotifyGiveFromISR(exec, &woken);
    }
    return woken == pdTRUE;
}
```

### src/esp32/src/comm_interface.cpp
```cpp
/**
 * @file comm_interface.cpp
 * @brief SPI slave communication interface implementation.
 *
 * The ESP32 is the SPI slave. Every transfer is a fixed-size wire frame:
 *
 *   Host TX frame  ──► ESP32 parses and executes request
 *   Host RX frame ◄── ESP32 returns latest status payload
 *
 * Status is therefore naturally pipelined by one SPI transaction, which keeps
 * the slave task simple and deterministic.
 */

#include "comm_interface.h"

#include <string.h>
#include <driver/spi_slave.h>
#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "motion_planner.h"

static const char* TAG = "comm_iface";

static constexpr uint32_t  SPI_TASK_STACK  = 4096;
static constexpr UBaseType_t SPI_TASK_PRIO = 10;
static constexpr BaseType_t  SPI_TASK_CORE = 0;

static constexpr uint32_t    MULTI_EXEC_STACK  = 8192;
static constexpr UBaseType_t MULTI_EXEC_PRIO   = 20;
static constexpr BaseType_t  MULTI_EXEC_CORE   = 1;

/**
 * @brief Hard wall-clock budget for one drain-loop sub-slice before the
 *        executor unconditionally yields to the FreeRTOS scheduler.
 *
 * The yield is UNCONDITIONAL — it does NOT depend on ring-fill level.
 * RMT pulse timing is in hardware, so a 1 ms scheduler sleep never
 * introduces step jitter.
 *
 * NOTE: Superseded by EXEC_TIME_BUDGET_US in motion_planner.h for the
 * state-machine executor. Retained for the per-axis executorTask in
 * stepper_queue.cpp which still uses it indirectly.
 */
static constexpr int64_t  YIELD_INTERVAL_US      = 400;

/**
 * @brief Maximum queue entries drained in a single bounded flush loop.
 *
 * Used by handleFlush() in the SPI ingestion path only.
 * The planner and executor have their own bounded drain constants.
 */
static constexpr uint32_t MAX_FLUSH_DRAIN        = 8;

/**
 * @brief Global queue of multi-axis segment blocks fed by the SPI task and
 *        consumed by the multi-axis executor task.
 *
 * Depth is sized to hold ~600 ms of motion at 4 ms/segment.
 */
static constexpr uint32_t MULTI_AXIS_QUEUE_DEPTH = 64;
static QueueHandle_t s_multi_axis_queue  = nullptr;

/**
 * @brief Global queue for flush requests.  Depth 4 is more than enough since
 *        the host can only issue one flush at a time.
 */
static constexpr uint32_t FLUSH_QUEUE_DEPTH = 4;
static QueueHandle_t s_flush_queue = nullptr;

DMA_ATTR static uint8_t s_rx_frame[SPI_FRAME_SIZE];
DMA_ATTR static uint8_t s_tx_frame_a[SPI_FRAME_SIZE];
DMA_ATTR static uint8_t s_tx_frame_b[SPI_FRAME_SIZE];

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommInterface::CommInterface(StepperQueue* queues[], uint8_t n_motors)
    : n_motors_(n_motors < SPI_MAX_AXES ? n_motors : SPI_MAX_AXES)
{
    for (uint8_t i = 0; i < SPI_MAX_AXES; ++i) {
        queues_[i] = (i < n_motors_) ? queues[i] : nullptr;
    }
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t CommInterface::init(const SpiBusPins& pins)
{
    pins_ = pins;

    if (pins_.home_pin_no != GPIO_NUM_NC && pins_.home_pin_nc != GPIO_NUM_NC) {
        gpio_config_t home_cfg = {};
        home_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(pins_.home_pin_no))
                               | (1ULL << static_cast<uint32_t>(pins_.home_pin_nc));
        home_cfg.mode = GPIO_MODE_INPUT;
        home_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        home_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        home_cfg.intr_type = GPIO_INTR_DISABLE;
        ESP_RETURN_ON_ERROR(gpio_config(&home_cfg), TAG, "failed to configure home sensor pins");
    }

    // Create the global multi-axis segment queue.
    s_multi_axis_queue = xQueueCreate(MULTI_AXIS_QUEUE_DEPTH, sizeof(multi_axis_block_t));
    ESP_RETURN_ON_FALSE(s_multi_axis_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create multi-axis queue");

    // Create the global flush request queue.
    s_flush_queue = xQueueCreate(FLUSH_QUEUE_DEPTH, sizeof(flush_request_t));
    ESP_RETURN_ON_FALSE(s_flush_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create flush queue");

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = pins_.mosi;
    bus_cfg.miso_io_num = pins_.miso;
    bus_cfg.sclk_io_num = pins_.sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = SPI_FRAME_SIZE;

    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.mode = 0;
    slave_cfg.spics_io_num = pins_.cs;
    slave_cfg.queue_size = 1;
    slave_cfg.flags = 0;
    slave_cfg.post_setup_cb = nullptr;
    slave_cfg.post_trans_cb = nullptr;

    ESP_RETURN_ON_ERROR(
        spi_slave_initialize(SPI3_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO),
        TAG, "spi_slave_initialize failed");

    BaseType_t rc = xTaskCreatePinnedToCore(
        &CommInterface::spiTask,
        "comm_spi",
        SPI_TASK_STACK,
        this,
        SPI_TASK_PRIO,
        nullptr,
        SPI_TASK_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create comm_spi task");

    // ── Planner layer: decomposes multi-axis blocks into planned segments ───
    ESP_RETURN_ON_ERROR(planner_.init(s_multi_axis_queue, s_flush_queue),
                        TAG, "failed to init motion planner");

    rc = xTaskCreatePinnedToCore(
        &CommInterface::multiAxisExecutorTask,
        "multi_exec",
        MULTI_EXEC_STACK,
        this,
        MULTI_EXEC_PRIO,
        nullptr,
        MULTI_EXEC_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create multi_exec task");

    // Delegate endstop ISR registration to the lateral axis driver (axis 1).
    // StepperDriver owns endstop_active_ and executor_task_, so the ISR
    // can act without going through CommInterface.
    if (n_motors_ >= 2 && queues_[1] != nullptr) {
        ESP_RETURN_ON_ERROR(
            queues_[1]->driver().initEndstopIsr(pins_.home_pin_no, pins_.home_pin_nc),
            TAG, "initEndstopIsr failed");
    }

    ESP_LOGI(TAG, "SPI slave ready  MOSI=%d MISO=%d SCLK=%d CS=%d  frame=%uB",
             (int)pins_.mosi, (int)pins_.miso, (int)pins_.sclk, (int)pins_.cs,
             (unsigned)SPI_FRAME_SIZE);
    return ESP_OK;
}

void CommInterface::buildStatusFrame(uint8_t* out_frame) const
{
    spi_message_zero_frame(out_frame);

    auto* header = reinterpret_cast<SpiMessageHeader*>(out_frame);
    auto* payload = reinterpret_cast<StatusPayload*>(out_frame + sizeof(SpiMessageHeader));

    spi_message_init_header(*header,
                            SpiMessageType::STATUS,
                            last_rx_sequence_,
                            sizeof(StatusPayload),
                            0);

    payload->uptime_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            payload->queue_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->available());
            payload->ring_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->driver().ringFreeSlots());
            payload->underrun_count[axis] = queues_[axis]->driver().getUnderrunCount();
            if (queues_[axis]->driver().isStreaming()) {
                payload->running_mask |= static_cast<uint8_t>(1U << axis);
            }
            if (queues_[axis]->driver().isEnabled()) {
                payload->enabled_mask |= static_cast<uint8_t>(1U << axis);
            }
        } else {
            payload->queue_free_slots[axis] = 0;
            payload->ring_free_slots[axis]  = 0;
            payload->underrun_count[axis]   = 0;
        }
    }
    payload->last_rx_sequence = last_rx_sequence_;
    payload->last_rx_type     = last_rx_type_;
    payload->last_result      = last_result_;
    payload->protocol_version = SPI_MSG_VERSION;
    payload->lateral_endstop_state = readLateralEndstopState();

    payload->endstop_armed_mask = 0;
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            if (queues_[axis]->driver().isEndstopArmed()) {
                payload->endstop_armed_mask |= static_cast<uint8_t>(1U << axis);
            }
        }
    }

    // Atomic load — lock-free cross-core read (written by Core 1 executor).
    payload->last_executed_sequence = last_executed_sequence_.load(std::memory_order_acquire);

    // Planner lookahead pressure: how many slots are free in segment_queue_.
    const uint32_t pqf = planner_.segmentQueueFree();
    payload->planner_queue_free = static_cast<uint8_t>(pqf < 255u ? pqf : 255u);

    spi_message_finalize(out_frame);
}

esp_err_t CommInterface::handleEnableAxis(const EnableAxisPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (payload.enable) {
        queues_[payload.axis_id]->driver().enable();
    } else {
        queues_[payload.axis_id]->driver().disable();
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleEmergencyStop(const EmergencyStopPayload& payload)
{
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < n_motors_; ++axis) {
            if (queues_[axis] != nullptr) {
                queues_[axis]->driver().emergencyStop();
            }
        }
        return ESP_OK;
    }

    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    queues_[payload.axis_id]->driver().emergencyStop();
    return ESP_OK;
}

esp_err_t CommInterface::handleStopAxis(const EmergencyStopPayload& payload)
{
    // gracefulStop() marks the driver as stopped but does NOT flush the ring
    // buffer, so the motor decelerates naturally through any remaining queued
    // steps rather than cutting out instantly.
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < n_motors_; ++axis) {
            if (queues_[axis] != nullptr) {
                queues_[axis]->gracefulStop();
            }
        }
        return ESP_OK;
    }

    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    queues_[payload.axis_id]->gracefulStop();
    return ESP_OK;
}

esp_err_t CommInterface::handleDisableAll()
{
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().disable();
        }
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleResetStats()
{
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().resetUnderrunCount();
        }
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleEnableEndstop(const EnableEndstopPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    StepperDriver& drv = queues_[payload.axis_id]->driver();
    if (payload.arm) {
        drv.armEndstop();
        ESP_LOGI(TAG, "endstop armed on axis %u", payload.axis_id);
    } else {
        drv.disarmEndstop();
        ESP_LOGI(TAG, "endstop disarmed on axis %u", payload.axis_id);
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleStepBlock(const StepBlockPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isLateralMovementAllowed(payload.axis_id)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (payload.step_count > STEP_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    motion_block_t block {};
    block.kind = MOTION_BLOCK_KIND_STEP;
    block.payload.step.count = payload.step_count;
    for (uint32_t i = 0; i < block.payload.step.count; ++i) {
        block.payload.step.steps[i].interval_ticks = payload.entries[i].interval_ticks;
        block.payload.step.steps[i].direction = (payload.entries[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
    }

    return queues_[payload.axis_id]->enqueueMotionBlock(block, 0);
}

esp_err_t CommInterface::handleSegmentBlock(const SegmentBlockPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isLateralMovementAllowed(payload.axis_id)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (payload.segment_count > SEGMENT_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    motion_block_t block {};
    block.kind = MOTION_BLOCK_KIND_SEGMENT;
    block.payload.segment.count = payload.segment_count;
    for (uint32_t i = 0; i < block.payload.segment.count; ++i) {
        block.payload.segment.segments[i].step_count = payload.segments[i].step_count;
        block.payload.segment.segments[i].start_ticks = payload.segments[i].start_ticks;
        block.payload.segment.segments[i].add_ticks = payload.segments[i].add_ticks;
        block.payload.segment.segments[i].direction =
            (payload.segments[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
        block.payload.segment.segments[i].reserved = 0;
    }

    return queues_[payload.axis_id]->enqueueMotionBlock(block, 0);
}

esp_err_t CommInterface::handleMultiAxisSegmentBlock(const uint8_t* payload,
                                                     uint16_t payload_length)
{
    /*
     * Wire layout for MULTI_AXIS_SEGMENT_BLOCK payload:
     *
     *   MultiAxisSegmentBlockHeader   (4 bytes)
     *   uint8_t  axis_ids[axis_count] (axis_count bytes)
     *   For each segment:
     *     uint16_t motion_sequence    (2 bytes)
     *     uint16_t duration_us        (2 bytes)
     *     uint16_t direction_mask     (2 bytes)
     *     uint16_t step_counts[axis_count] (2 * axis_count bytes)
     *
     * Total minimum: 4 + axis_count + segment_count * (6 + 2*axis_count)
     */
    if (payload_length < sizeof(MultiAxisSegmentBlockHeader)) {
        return ESP_ERR_INVALID_SIZE;
    }

    MultiAxisSegmentBlockHeader hdr_val;
    memcpy(&hdr_val, payload, sizeof(hdr_val));
    const uint8_t axis_count     = hdr_val.axis_count;
    const uint8_t segment_count  = hdr_val.segment_count;

    if (axis_count == 0 || axis_count > MULTI_AXIS_MAX_AXES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (segment_count == 0 || segment_count > MULTI_AXIS_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Validate total payload length before reading any further.
    const size_t expected_length =
        sizeof(MultiAxisSegmentBlockHeader)
        + static_cast<size_t>(axis_count)
        + static_cast<size_t>(segment_count) * (6u + 2u * axis_count);
    if (payload_length < static_cast<uint16_t>(expected_length)) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Deserialise.
    multi_axis_block_t block {};
    block.axis_count     = axis_count;
    block.segment_count  = segment_count;

    const uint8_t* cursor = payload + sizeof(MultiAxisSegmentBlockHeader);

    // axis_ids
    for (uint8_t a = 0; a < axis_count; ++a) {
        block.axis_ids[a] = cursor[a];
    }
    cursor += axis_count;

    // segments
    for (uint8_t s = 0; s < segment_count; ++s) {
        uint16_t motion_seq, duration_us, dir_mask;
        memcpy(&motion_seq,  cursor,     2);
        memcpy(&duration_us, cursor + 2, 2);
        memcpy(&dir_mask,    cursor + 4, 2);
        cursor += 6;

        block.segments[s].motion_sequence = motion_seq;
        block.segments[s].duration_us     = duration_us;
        block.segments[s].direction_mask  = dir_mask;

        for (uint8_t a = 0; a < axis_count; ++a) {
            uint16_t steps;
            memcpy(&steps, cursor, 2);
            block.segments[s].step_counts[a] = steps;
            cursor += 2;
        }
    }

    // Non-blocking enqueue: return QUEUE_FULL immediately if full.
    if (xQueueSend(s_multi_axis_queue, &block, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT; // maps to QUEUE_FULL result code
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleFlush(const FlushPayload& flush_payload)
{
    /*
     * Post a flush_request_t to the flush queue.  The executor task
     * watches this queue and applies the flush before processing the next
     * segment.  Using a queue (instead of an atomic variable) ensures that
     * a flush posted just before new segments arrive is always processed in
     * the correct order.
     */
    flush_request_t req { .flush_sequence = flush_payload.flush_sequence };
    if (xQueueSend(s_flush_queue, &req, 0) != pdTRUE) {
        // Flush queue full — this should never happen in normal operation.
        ESP_LOGW(TAG, "flush queue full — flush_seq=%u dropped",
                 (unsigned)flush_payload.flush_sequence);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void CommInterface::notifySegmentExecuted(uint16_t motion_seq)
{
    /*
     * Called by the executor task (Core 1) after each multi-axis segment
     * completes.  Updates last_executed_sequence_ with an atomic store
     * so the SPI task (Core 0) can safely read it in buildStatusFrame().
     *
     * Only advances the sequence — never moves it backward.  This handles
     * the 16-bit wrap-around case correctly because we only call this in
     * strict execution order.
     */
    uint16_t current = last_executed_sequence_.load(std::memory_order_relaxed);
    if (static_cast<int16_t>(motion_seq - current) > 0) {
        last_executed_sequence_.store(motion_seq, std::memory_order_release);
    }
}

uint8_t CommInterface::readLateralEndstopState() const
{
    if (pins_.home_pin_no == GPIO_NUM_NC || pins_.home_pin_nc == GPIO_NUM_NC) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }

    const int no_state = gpio_get_level(pins_.home_pin_no);
    const int nc_state = gpio_get_level(pins_.home_pin_nc);

    if (no_state == nc_state) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }
    if (no_state == 0 && nc_state == 1) {
        return static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED);
    }
    return static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

bool CommInterface::isLateralMovementAllowed(uint8_t axis_id) const
{
    if (axis_id != 1) {
        return true;
    }
    return readLateralEndstopState() == static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

esp_err_t CommInterface::handleFrame(const SpiMessageHeader& header, const uint8_t* payload)
{
    switch (static_cast<SpiMessageType>(header.msg_type)) {
    case SpiMessageType::NOP:
    case SpiMessageType::GET_STATUS:
    case SpiMessageType::PING:
        return ESP_OK;

    case SpiMessageType::ENABLE_AXIS: {
        if (header.payload_length != sizeof(EnableAxisPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EnableAxisPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEnableAxis(p);
    }

    case SpiMessageType::ESTOP: {
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EmergencyStopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEmergencyStop(p);
    }

    case SpiMessageType::STOP_AXIS: {
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EmergencyStopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleStopAxis(p);
    }

    case SpiMessageType::DISABLE_ALL:
        if (header.payload_length != 0) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleDisableAll();

    case SpiMessageType::RESET_STATS:
        if (header.payload_length != 0) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleResetStats();

    case SpiMessageType::STEP_BLOCK: {
        if (header.payload_length != sizeof(StepBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        StepBlockPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleStepBlock(p);
    }

    case SpiMessageType::SEGMENT_BLOCK: {
        if (header.payload_length != sizeof(SegmentBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        SegmentBlockPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleSegmentBlock(p);
    }

    case SpiMessageType::MULTI_AXIS_SEGMENT_BLOCK:
        // Variable-length payload — pass raw buffer + length.
        return handleMultiAxisSegmentBlock(payload, header.payload_length);

    case SpiMessageType::FLUSH: {
        if (header.payload_length != sizeof(FlushPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        FlushPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleFlush(p);
    }

    case SpiMessageType::ENABLE_ENDSTOP: {
        if (header.payload_length != sizeof(EnableEndstopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EnableEndstopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEnableEndstop(p);
    }

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

void CommInterface::spiTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);
    ESP_LOGI(TAG, "SPI task started on core %d", xPortGetCoreID());

    // Double-buffer ping-pong: while DMA transmits tx_ping, we build the next
    // status frame into tx_pong.  This reduces pipeline lag by one full SPI
    // round-trip — the status sent in transaction N reflects state AFTER
    // transaction N-1 was handled, not state from before the previous transmit.
    uint8_t* tx_ping = s_tx_frame_a;
    uint8_t* tx_pong = s_tx_frame_b;

    // Pre-build the very first frame before entering the loop so the initial
    // transaction has valid (zero-but-structured) content.
    self->buildStatusFrame(tx_ping);

    for (;;) {
        // ── Transmit the previously-built status frame ────────────────────────
        spi_slave_transaction_t txn = {};
        txn.length = SPI_FRAME_SIZE * 8;
        txn.tx_buffer = tx_ping;
        txn.rx_buffer = s_rx_frame;

        esp_err_t err = spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY);
        // DMA is done with tx_ping — safe to reuse as the next write buffer.

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            // Rebuild into the same ping buffer and retry.
            self->buildStatusFrame(tx_ping);
            continue;
        }

        // ── Parse and handle incoming frame ───────────────────────────────────
        SpiMessageHeader header {};
        memcpy(&header, s_rx_frame, sizeof(SpiMessageHeader));

        if (header.magic != SPI_MSG_MAGIC) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_MAGIC);
        } else if (header.version != SPI_MSG_VERSION) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_VERSION);
        } else if (header.payload_length > SPI_MAX_PAYLOAD_SIZE) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
        } else if (!spi_message_validate(s_rx_frame, header)) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_CRC);
        } else {
            self->last_rx_sequence_ = header.sequence;
            self->last_rx_type_ = header.msg_type;

            const uint8_t* payload = s_rx_frame + sizeof(SpiMessageHeader);
            err = self->handleFrame(header, payload);
            if (err == ESP_OK) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::OK);
            } else if (err == ESP_ERR_TIMEOUT) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::QUEUE_FULL);
            } else if (err == ESP_ERR_INVALID_ARG) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_AXIS);
            } else if (err == ESP_ERR_INVALID_SIZE) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
            } else if (err == ESP_ERR_NOT_SUPPORTED) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::UNKNOWN_TYPE);
            } else if (err == ESP_ERR_INVALID_STATE) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::ENDSTOP_BLOCKED);
            } else {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::INTERNAL_ERROR);
                ESP_LOGW(TAG, "message 0x%02X failed: %s",
                         header.msg_type, esp_err_to_name(err));
            }
        }

        // ── Build next status frame into the now-idle buffer ──────────────────
        // We write into tx_pong (the buffer NOT currently wired to DMA).
        // Reflects state AFTER handling the frame we just received.
        self->buildStatusFrame(tx_pong);

        // Swap: tx_pong becomes the next transmit buffer.
        uint8_t* tmp = tx_ping;
        tx_ping = tx_pong;
        tx_pong = tmp;
    }
}

// ---------------------------------------------------------------------------
// multiAxisExecutorTask()  — Core 1, priority 20 — STATE MACHINE
// ---------------------------------------------------------------------------
//
// Refactored from a monolithic nested-loop drain pattern into a bounded
// state machine.  Each state transition does bounded work (≤ EXEC_TIME_BUDGET_US
// or ≤ EXEC_BATCH_LIMIT segments) then yields to the scheduler.
//
//   IDLE ──► FETCH ──► DRAIN ──► RUN ──► IDLE
//              │                          ▲
//              ├── (flush sentinel) ──► FLUSH ──┘
//              └── (error/underrun) ──► RECOVERY ──┘
//
// Watchdog safety: NO state executes for more than ~300 µs without exiting
// to the for(;;) top-level loop which naturally yields via xQueueReceive
// or explicit vTaskDelay(1).

void CommInterface::multiAxisExecutorTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);

    ESP_LOGI(TAG, "multi-axis executor (state machine) started on core %d",
             xPortGetCoreID());

    // Register this task for ISR ring-space wakeups on all drivers.
    {
        TaskHandle_t my_handle = xTaskGetCurrentTaskHandle();
        for (uint8_t a = 0; a < self->n_motors_; ++a) {
            if (self->queues_[a] != nullptr) {
                self->queues_[a]->driver().setExecutorTask(my_handle);
            }
        }
    }

    ESP_LOGI(TAG, "multi_exec stack high watermark at start: %u bytes free",
             (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));

    // ── Segment queue handle from the planner ─────────────────────────────
    QueueHandle_t seg_queue = self->planner_.segmentQueue();

    // ── Deferred notification ring ────────────────────────────────────────
    static constexpr int DEFER_DEPTH = 256;
    static int64_t  defer_fire_us[DEFER_DEPTH];
    static uint32_t defer_seqs[DEFER_DEPTH];
    int      defer_head = 0;
    int      defer_tail = 0;

    // ── Active axis tracking (persists across iterations for recovery) ────
    uint8_t active_axis_ids[MULTI_AXIS_MAX_AXES] = {};
    uint8_t active_axis_count = 0;

    // ── Batch buffer for FETCH state ──────────────────────────────────────
    planned_segment_t batch[EXEC_BATCH_LIMIT];
    uint32_t batch_count = 0;
    uint32_t batch_index = 0;

    // ── State machine ─────────────────────────────────────────────────────
    ExecState state = ExecState::IDLE;

    // Lambda: fire all due deferred notifications.
    auto fireDeferred = [&]() {
        const int64_t now = esp_timer_get_time();
        while (defer_head != defer_tail) {
            const int idx = defer_head & (DEFER_DEPTH - 1);
            if (now >= defer_fire_us[idx]) {
                self->notifySegmentExecuted(
                    static_cast<uint16_t>(defer_seqs[idx]));
                ++defer_head;
            } else {
                break;
            }
        }
    };

    // Lambda: kickStart all active axes.
    auto kickStartActiveAxes = [&]() {
        for (uint8_t a = 0; a < active_axis_count; ++a) {
            const uint8_t axis_id = active_axis_ids[a];
            if (axis_id < self->n_motors_ && self->queues_[axis_id] != nullptr) {
                self->queues_[axis_id]->kickStart();
            }
        }
    };

    // ── Main loop ─────────────────────────────────────────────────────────
    for (;;) {
        static uint32_t wm_iter = 0;
        if (++wm_iter % 2000 == 0) {
            ESP_LOGD(TAG, "multi_exec stack watermark: %u bytes free",
                     (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));
        }

        // Always fire due deferred notifications at top of loop.
        fireDeferred();

        switch (state) {

        // ══════════════════════════════════════════════════════════════════
        // IDLE: wait for segments from the planner (blocking with timeout)
        // ══════════════════════════════════════════════════════════════════
        case ExecState::IDLE: {
            // Compute wait timeout: wake early if a deferred notification
            // is about to fire.  Hard-cap at 1 ms so ISR ring-space
            // notifications (which wake ulTaskNotifyTake, not xQueueReceive)
            // don't cause >1 ms stalls.
            TickType_t wait_ticks;
            if (defer_head != defer_tail) {
                const int idx = defer_head & (DEFER_DEPTH - 1);
                const int64_t remaining_us =
                    defer_fire_us[idx] - esp_timer_get_time();
                if (remaining_us <= 500) {
                    wait_ticks = 0;
                } else {
                    wait_ticks = 1;
                }
            } else {
                wait_ticks = pdMS_TO_TICKS(1);
            }

            planned_segment_t seg;
            if (xQueueReceive(seg_queue, &seg, wait_ticks) == pdTRUE) {
                batch[0]    = seg;
                batch_count = 1;
                batch_index = 0;
                state = ExecState::FETCH;
            } else {
                // Timeout — kick-start any stalled axes (RMT underrun
                // while we were blocked on xQueueReceive).
                kickStartActiveAxes();
            }
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // FETCH: non-blocking batch fill up to EXEC_BATCH_LIMIT
        // ══════════════════════════════════════════════════════════════════
        case ExecState::FETCH: {
            // Fill remaining batch slots non-blocking.
            while (batch_count < EXEC_BATCH_LIMIT) {
                planned_segment_t seg;
                if (xQueueReceive(seg_queue, &seg, 0) != pdTRUE) break;
                batch[batch_count++] = seg;
            }
            batch_index = 0;
            state = ExecState::DRAIN;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // DRAIN: process batch segments — write steps to RMT ring
        // ══════════════════════════════════════════════════════════════════
        case ExecState::DRAIN: {
            const int64_t drain_start = esp_timer_get_time();

            while (batch_index < batch_count) {
                // ── Pre-check: yield if time budget will be exceeded ───────────
                // This prevents accumulating too much CPU time before yielding.
                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    kickStartActiveAxes();
                    taskYIELD();
                    // Restart from FETCH to get fresh batch and reset timer.
                    state = ExecState::FETCH;
                    goto exit_drain;
                }

                planned_segment_t& seg = batch[batch_index];

                // ── Flush sentinel ────────────────────────────────────────
                if (seg.is_flush) {
                    state = ExecState::FLUSH;
                    goto exit_drain;  // break out of DRAIN, handle in FLUSH
                }

                // ── Update active axis list ───────────────────────────────
                active_axis_count = seg.axis_count < MULTI_AXIS_MAX_AXES
                    ? seg.axis_count : MULTI_AXIS_MAX_AXES;
                for (uint8_t a = 0; a < active_axis_count; ++a) {
                    active_axis_ids[a] = seg.axis_ids[a];
                }

                // ── Endstop check (per-segment, real-time) ───────────────
                bool endstop_hit = false;
                for (uint8_t a = 0; a < seg.axis_count && !endstop_hit; ++a) {
                    const uint8_t eid = seg.axis_ids[a];
                    if (eid >= self->n_motors_ ||
                        self->queues_[eid] == nullptr) continue;
                    if (self->queues_[eid]->driver().isEndstopActive()) {
                        // Drain remaining batch, e-stop, notify host.
                        self->queues_[eid]->driver().emergencyStop();
                        self->notifySegmentExecuted(seg.motion_sequence);
                        ESP_LOGW(TAG, "endstop on axis %u at seq=%u",
                                 eid, seg.motion_sequence);
                        endstop_hit = true;
                    }
                }
                if (endstop_hit) {
                    state = ExecState::RECOVERY;
                    goto exit_drain;
                }

                // ── Lateral endstop gate (read once per segment) ──────────
                const uint8_t lateral_state = self->readLateralEndstopState();
                const bool lateral_blocked =
                    lateral_state !=
                    static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);

                // ── Write steps to ring buffer (no RMT start) ─────────────
                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id >= self->n_motors_ ||
                        self->queues_[axis_id] == nullptr) continue;
                    if (seg.axes[a].step_count == 0) continue;
                    if (axis_id == 1 && lateral_blocked) {
                        ESP_LOGD(TAG, "axis1 blocked, skip %u steps",
                                 seg.axes[a].step_count);
                        continue;
                    }

                    esp_err_t err =
                        self->queues_[axis_id]->executeConstantRateBlock(
                            seg.axes[a].direction,
                            seg.axes[a].step_count,
                            seg.duration_us);

                    if (err == ESP_ERR_INVALID_STATE) {
                        ESP_LOGW(TAG, "axis %u endstop mid-seg seq=%u",
                                 axis_id, seg.motion_sequence);
                        self->queues_[axis_id]->driver().emergencyStop();
                        self->notifySegmentExecuted(seg.motion_sequence);
                        state = ExecState::RECOVERY;
                        goto exit_drain;
                    } else if (err != ESP_OK) {
                        ESP_LOGW(TAG, "axis %u seg %u: %s",
                                 axis_id, seg.motion_sequence,
                                 esp_err_to_name(err));
                    }
                }

                // ── Schedule deferred notification ────────────────────────
                if ((defer_tail - defer_head) < DEFER_DEPTH) {
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = seg.scheduled_time_us
                                         + static_cast<int64_t>(seg.duration_us);
                    defer_seqs[idx]    = seg.motion_sequence;
                    ++defer_tail;
                } else {
                    // Ring full: evict the oldest (earliest scheduled) entry,
                    // notify it now (it is already overdue), then enqueue the
                    // current segment normally. This preserves ordering and
                    // avoids signalling completion before steps reach the ring.
                    const int evict_idx = defer_head & (DEFER_DEPTH - 1);
                    const uint32_t evicted_seq = defer_seqs[evict_idx];
                    self->notifySegmentExecuted(
                        static_cast<uint16_t>(evicted_seq));
                    ++defer_head;
                    // Enqueue current segment.
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = seg.scheduled_time_us
                                         + static_cast<int64_t>(seg.duration_us);
                    defer_seqs[idx]    = seg.motion_sequence;
                    ++defer_tail;
                    ESP_LOGW(TAG, "defer ring full: evicted seq=%u to make room for seq=%u",
                             (unsigned)evicted_seq,
                             (unsigned)seg.motion_sequence);
                }

                // Restart RMT immediately if it stopped mid-batch due to ring drain.
                // Do not wait for ExecState::RUN — the ring may fill with unconsumed
                // steps causing pushBlock() to deadlock on ulTaskNotifyTake.
                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id >= self->n_motors_ ||
                        self->queues_[axis_id] == nullptr) continue;
                    if (!self->queues_[axis_id]->driver().isStreaming()) {
                        self->queues_[axis_id]->kickStart();
                    }
                }

                ++batch_index;

                // ── Time budget check (watchdog safety) ───────────────────
                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    // Budget exhausted — transition to RUN to kickStart,
                    // then yield before processing remaining segments.
                    kickStartActiveAxes();
                    taskYIELD();
                    // Continue draining after yield (reset budget).
                    break;  // will re-enter DRAIN on next iteration
                }
            }

            // All segments in batch processed — transition to RUN.
            if (batch_index >= batch_count) {
                state = ExecState::RUN;
            }
            // else: budget break, stay in DRAIN for remaining segments.
            break;

        exit_drain:
            break;  // state already set by the goto target
        }

        // ══════════════════════════════════════════════════════════════════
        // RUN: kickStart RMT on all active axes, return to IDLE
        // ══════════════════════════════════════════════════════════════════
        case ExecState::RUN: {
            kickStartActiveAxes();
            fireDeferred();
            state = ExecState::IDLE;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // FLUSH: reset pipeline state, notify host
        // ══════════════════════════════════════════════════════════════════
        case ExecState::FLUSH: {
            // The flush sentinel is at batch[batch_index].
            const planned_segment_t& flush_seg = batch[batch_index];

            // Reset deferred notification ring.
            defer_head = defer_tail = 0;

            // Notify host with flush sequence.
            self->notifySegmentExecuted(flush_seg.flush_sequence);

            ESP_LOGI(TAG, "executor flush at seq=%u",
                     (unsigned)flush_seg.flush_sequence);

            // Clear batch and return to idle.
            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // RECOVERY: handle endstop / error, drain remaining, return to IDLE
        // ══════════════════════════════════════════════════════════════════
        case ExecState::RECOVERY: {
            // Drain any remaining segments in the planner's output queue
            // (bounded drain to avoid spending too long here).
            planned_segment_t discard;
            uint32_t drained = 0;
            while (drained < SEGMENT_QUEUE_DEPTH &&
                   xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
                ++drained;
            }

            // Reset deferred notifications.
            defer_head = defer_tail = 0;

            ESP_LOGW(TAG, "recovery: drained %lu remaining segments",
                     (unsigned long)drained);

            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }

        } // switch(state)

        // ── Watchdog safety: yield if idle, sleep if very idle ───────────────
        // If we fetched zero segments in FETCH, sleep to let IDLE1 run.
        // Otherwise, yield to respect other tasks without 10ms stalls.
        if (state == ExecState::IDLE && batch_count == 0) {
            vTaskDelay(1);  // Very idle — sleep and let watchdog reset
        } else {
            taskYIELD();    // Still have work — yield but stay ready
        }
    } // for(;;)
}
```

### src/esp32/src/main.cpp
```cpp
/**
 * @file main.cpp
 * @brief ESP32 Klipper-style stepper executor — entry point.
 *
 * Architecture overview
 * ─────────────────────
 *   Core 0  (APP CPU)
 *     • comm_spi task (pri 10) : SPI slave message parser → StepperQueue
 *
 *   Core 1  (PRO CPU)
 *     • stepper_0     (pri 24) : executor for motor A  (RMT channel 0)
 *     • stepper_1     (pri 24) : executor for motor B  (RMT channel 1)
 *
 *   RMT hardware
 *     • Channel 0 → STEP_A_GPIO  (2 MHz, 64-symbol block, trans_queue=1)
 *     • Channel 1 → STEP_B_GPIO  (2 MHz, 64-symbol block, trans_queue=1)
 *     • One balanced 50/50 HIGH/LOW RMT symbol per commanded step
 *
 * Pin assignments
 * ───────────────
 *   Motor A (Bobbin / axis 0)  : STEP=GPIO26  DIR=GPIO27  EN=GPIO14
 *   Motor B (Lateral / axis 1) : STEP=GPIO32  DIR=GPIO33  EN=GPIO25
 *
 *   SPI host link              : MOSI=GPIO23  MISO=GPIO19
 *                                SCLK=GPIO18  CS=GPIO5
 *
 * The Raspberry Pi demo lives in `src/rpi/` and streams fixed-size SPI
 * message frames to this firmware.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "step_types.h"
#include "stepper_driver.h"
#include "stepper_queue.h"
#include "comm_interface.h"

static const char* TAG = "main";

// ---------------------------------------------------------------------------
// Pin definitions — adjust to your board wiring
// ---------------------------------------------------------------------------

// Motor A — Bobbin axis
static constexpr gpio_num_t STEP_A = GPIO_NUM_26;
static constexpr gpio_num_t DIR_A  = GPIO_NUM_27;
static constexpr gpio_num_t EN_A   = GPIO_NUM_14;

// Motor B — Lateral axis
static constexpr gpio_num_t STEP_B = GPIO_NUM_32;
static constexpr gpio_num_t DIR_B  = GPIO_NUM_33;
static constexpr gpio_num_t EN_B   = GPIO_NUM_25;

// SPI host link
static constexpr gpio_num_t SPI_MOSI = GPIO_NUM_23;
static constexpr gpio_num_t SPI_MISO = GPIO_NUM_19;
static constexpr gpio_num_t SPI_SCLK = GPIO_NUM_18;
static constexpr gpio_num_t SPI_CS   = GPIO_NUM_5;

// Lateral home sensor (2-contact)
static constexpr gpio_num_t HOME_NO = GPIO_NUM_21;
static constexpr gpio_num_t HOME_NC = GPIO_NUM_22;

// ---------------------------------------------------------------------------
// Global instances — static storage, constructed once
// ---------------------------------------------------------------------------

static StepperDriver motor_a(STEP_A, DIR_A, EN_A, 0);
static StepperDriver motor_b(STEP_B, DIR_B, EN_B, 1);

static StepperQueue  queue_a(motor_a, 0);
static StepperQueue  queue_b(motor_b, 1);

static StepperQueue* queues[2] = {&queue_a, &queue_b};
static CommInterface comm(queues, 2);

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "PickupWinder — Klipper-style RMT stepper executor");
    ESP_LOGI(TAG, "RMT resolution : %lu Hz  (%lu ns/tick)",
             (unsigned long)RMT_STEP_RESOLUTION_HZ,
             (unsigned long)(1000000000UL / RMT_STEP_RESOLUTION_HZ));
    ESP_LOGI(TAG, "Max step rate  : ~166 kHz  (interval_min = %u ticks = %lu µs)",
             RMT_STEP_MIN_TICKS,
             (unsigned long)(RMT_STEP_MIN_TICKS * 1000000UL / RMT_STEP_RESOLUTION_HZ));
    ESP_LOGI(TAG, "Block size     : %d steps   Queue depth : %d blocks",
             STEP_BLOCK_SIZE, STEPPER_QUEUE_DEPTH);

    // ── 1. Initialise RMT drivers ───────────────────────────────────────────
    ESP_ERROR_CHECK(motor_a.init());
    ESP_ERROR_CHECK(motor_b.init());

    // ── 2. Enable motor drivers ─────────────────────────────────────────────
    //motor_a.enable();
    //motor_b.enable();

    // ── 3. Launch executor tasks (Core 1, priority 24) ──────────────────────
    ESP_ERROR_CHECK(queue_a.init());
    ESP_ERROR_CHECK(queue_b.init());

    // ── 4. Start SPI communication interface (Core 0, priority 10) ────────
    ESP_ERROR_CHECK(comm.init({SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_CS, HOME_NO, HOME_NC}));

    // app_main may return — FreeRTOS scheduler continues running the tasks.
    ESP_LOGI(TAG, "Scheduler running — app_main exiting.");
}
```

### src/rpi/core/config.py
```python
from dataclasses import dataclass
from typing import Optional

@dataclass
class AppConfiguration:
    """Configuration parameters for the PickupWinder host application."""


    rpc_socket_path: str = "/tmp/pickup_winder_rpc.sock"
    spi_device: str = "/dev/spidev0.0"
    spi_speed_hz: int = 1_000_000

    spindle_axis_id: int = 0
    spindle_steps_per_revolution: int = 200
    spindle_microstepping: int = 32
    spindle_invert_direction: bool = False
    spindle_max_speed_rpm: int = 1500
    # Unit: RPM/s (revolutions per minute gained per second).
    spindle_max_acceleration_rpm: Optional[float] = 10
    # Unit: RPM/s (revolutions per minute lost per second).
    spindle_max_deceleration_rpm: Optional[float] = None

    lateral_axis_id: int = 1
    lateral_steps_per_revolution: int = 200
    lateral_microstepping: int = 32
    lateral_invert_direction: bool = False
    lateral_max_rpm: int = 1000
    # Unit: mm/s² on traverse axis.
    lateral_max_acceleration_mm_per_s2: Optional[float] = None
    # Unit: mm/s² on traverse axis.
    lateral_max_deceleration_mm_per_s2: Optional[float] = None
    
    # Leadscrew/traverse pitch in mm per revolution for the lateral axis.
    # Used to compute steps/mm: steps_per_rev * microstepping / pitch_mm
    lateral_traverse_pitch_mm: float = 1.0
    # Optional explicit override for lateral steps-per-mm. If set, this
    # value takes precedence over the computed value.
    lateral_steps_per_mm_override: Optional[float] = None

    @property
    def lateral_steps_per_mm(self) -> float:
        """Return lateral axis steps per millimetre.

        Computed as: (steps_per_revolution * microstepping) / traverse_pitch_mm.
        If `lateral_steps_per_mm_override` is provided, it is returned instead.
        """
        if self.lateral_steps_per_mm_override is not None:
            return float(self.lateral_steps_per_mm_override)
        return (self.lateral_steps_per_revolution * self.lateral_microstepping) / float(self.lateral_traverse_pitch_mm)

    @property
    def spindle_max_acceleration_steps_per_s2(self) -> float:
        """Compute spindle acceleration in steps/s^2.

        Uses `spindle_max_acceleration_rpm` (RPM/s) if provided.
        Otherwise returns a safe default.
        """
        steps_per_rev = self.spindle_steps_per_revolution * self.spindle_microstepping
        if self.spindle_max_acceleration_rpm is not None:
            return (self.spindle_max_acceleration_rpm / 60.0) * steps_per_rev
        return 100_000.0

    @property
    def spindle_max_deceleration_steps_per_s2(self) -> float:
        """Compute spindle deceleration in steps/s^2.

        Uses `spindle_max_deceleration_rpm` (RPM/s) if provided. Otherwise falls back
        to the configured spindle acceleration limit.
        """
        steps_per_rev = self.spindle_steps_per_revolution * self.spindle_microstepping
        if self.spindle_max_deceleration_rpm is not None:
            return (self.spindle_max_deceleration_rpm / 60.0) * steps_per_rev
        return self.spindle_max_acceleration_steps_per_s2

    @property
    def lateral_max_acceleration_steps_per_s2(self) -> float:
        """Compute lateral acceleration in steps/s^2.

        Uses `lateral_max_acceleration_mm_per_s2` if provided.
        Otherwise returns a safe default.
        """
        if self.lateral_max_acceleration_mm_per_s2 is not None:
            return float(self.lateral_max_acceleration_mm_per_s2) * self.lateral_steps_per_mm
        return 100_000.0

    @property
    def lateral_max_deceleration_steps_per_s2(self) -> float:
        """Compute lateral deceleration in steps/s^2.

        Uses `lateral_max_deceleration_mm_per_s2` if provided.
        Otherwise falls back to the configured lateral acceleration limit.
        """
        if self.lateral_max_deceleration_mm_per_s2 is not None:
            return float(self.lateral_max_deceleration_mm_per_s2) * self.lateral_steps_per_mm
        return self.lateral_max_acceleration_steps_per_s2
```

### src/rpi/transport/streamer.py
```python
from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Any, Iterator

from transport.messages import (
    MULTI_AXIS_SEGMENT_BLOCK_SIZE,
    MultiAxisSegment,
    MultiAxisSegmentBlockPayload,
    SpiMessageResult,
    sequence_is_greater,
    sequence_is_less_equal,
)
from motion import AxisMotionConfig, MultiAxisSegmentGenerator, RampConfig
from transport.spi_transport import Esp32SpiTransport

logger = logging.getLogger(__name__)


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

    # Planner→executor segment queue depth on the ESP32 (matches SEGMENT_QUEUE_DEPTH in firmware).
    SEGMENT_QUEUE_DEPTH = 128
    # Legacy constant kept for reference (= EXEC_BATCH_LIMIT * 2).
    # The active gate is now required_lookahead() which is speed-dependent.
    PLANNER_QUEUE_SEND_THRESHOLD = 32

    # ESP32 step ring capacity in firmware: one step consumes one ring entry.
    STEP_RING_CAPACITY = 4096
    RING_BUFFER_HEADROOM = 0.8

    @staticmethod
    def required_lookahead(steps_per_segment: int) -> int:
        """Speed-dependent minimum segment lookahead depth in the ESP32 planner queue.

        At low speed each segment contains very few steps, so the ring drains
        faster relative to the inter-segment host→ESP32 pipeline latency (~3–5 ms).
        A deeper buffer prevents ring underruns and motor stutter.

        Thresholds match firmware EXEC_BATCH_LIMIT tiers:
          < 10  steps → 48 segments (low speed,  ~50 RPM)
          < 50  steps → 32 segments (mid speed)
          >= 50 steps → 16 segments (high speed, > ~200 RPM)
        """
        if steps_per_segment < 10:
            return 48
        elif steps_per_segment < 50:
            return 32
        else:
            return 16

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
        self._initialize_streamer_state(
            transport=transport,
            axis_configs=[AxisMotionConfig(axis_id=s.axis_id, ramp=s.ramp) for s in axis_streams],
            axis_ids=[s.axis_id for s in axis_streams],
            segment_duration_s=segment_duration_s,
            target_buffer_time_s=target_buffer_time_s,
            poll_interval_s=poll_interval_s,
            print_every=print_every,
            log_each_send=log_each_send,
            send_log_path=send_log_path,
            explicit_target_hz=None,
        )

    @classmethod
    def from_axis_ids(
        cls,
        transport: Esp32SpiTransport,
        axis_ids: list[int],
        *,
        target_hz: float,
        segment_duration_s: float = 0.004,
        poll_interval_s: float = 0.001,
        print_every: int = 1,
        target_buffer_time_s: float = 0.150,
    ) -> "MultiAxisRampStreamer":
        """Build a streamer from explicit axis IDs and a known target frequency.

        Use this constructor when the move generator is external (e.g. `WoundMove`)
        and no reliable `RampConfig` objects are available.

        Differences vs `__init__`:
          - `__init__`: derives `target_hz` from `RampConfig.target_hz`.
          - `from_axis_ids`: receives `target_hz` explicitly and avoids synthetic ramps.
        """
        if not axis_ids:
            raise ValueError("axis_ids must not be empty")
        if target_hz <= 0.0:
            raise ValueError("target_hz must be positive")

        streamer = cls.__new__(cls)
        streamer._initialize_streamer_state(
            transport=transport,
            axis_configs=[],
            axis_ids=axis_ids,
            segment_duration_s=segment_duration_s,
            target_buffer_time_s=target_buffer_time_s,
            poll_interval_s=poll_interval_s,
            print_every=print_every,
            log_each_send=False,
            send_log_path=None,
            explicit_target_hz=target_hz,
        )
        return streamer

    def _initialize_streamer_state(
        self,
        *,
        transport: Esp32SpiTransport,
        axis_configs: list[AxisMotionConfig],
        axis_ids: list[int],
        segment_duration_s: float,
        target_buffer_time_s: float,
        poll_interval_s: float,
        print_every: int,
        log_each_send: bool,
        send_log_path: str | None,
        explicit_target_hz: float | None,
    ) -> None:
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

        self._axis_configs = axis_configs
        self._axis_ids = list(axis_ids)
        self._segment_duration_s = max(self.MIN_SEGMENT_TIME_S, min(self.MAX_SEGMENT_TIME_S, segment_duration_s))
        if explicit_target_hz is None:
            max_hz = 0.0
            if self._axis_configs:
                max_hz = max(config.ramp.target_hz for config in self._axis_configs)
            self._target_buffer_time_s = self._safe_buffer_time_s(target_buffer_time_s, max_hz)
        else:
            self._target_buffer_time_s = self._safe_buffer_time_s(target_buffer_time_s, explicit_target_hz)
        self._min_buffer_time_s = min(self.MIN_BUFFER_TIME_S, self._target_buffer_time_s * 0.5)

        self._inflight: deque[tuple[MultiAxisSegment, int]] = deque()
        self._buffered_time_s = 0.0
        self._last_sent_motion_seq = -1
        self._last_sent_transport_seq = -1
        self._planner_under_pressure = False  # True while planner_queue_free < threshold
        self._buffered_segments = 0           # SEGMENT_QUEUE_DEPTH - planner_queue_free
        self._current_steps_per_segment = 0  # updated per-segment; drives required_lookahead()
        if explicit_target_hz is not None:
            self._current_steps_per_segment = max(1, int(round(explicit_target_hz * self._segment_duration_s)))
        self._prefilling = False              # suppresses pressure gate during initial prefill
        # Premature-completion detection
        self._last_confirmed_sequence: int = -1
        self._premature_notify_count: int = 0
        self._premature_notify_window_start: float = 0.0
        self._last_sequence_advance_time: float = time.time()
        self._last_sequence_advance_value: int = -1
        self._stall_timeout_s: float = 2.0  # stall if no progress for 2s

        self._sync_with_firmware_status()
        start_sequence = (
            (self._last_confirmed_sequence + 1) & 0xFFFF
            if self._last_confirmed_sequence >= 0
            else 0
        )
        if self._axis_configs:
            self._generator = iter(
                MultiAxisSegmentGenerator(
                    self._axis_configs,
                    segment_duration_s=self._segment_duration_s,
                    start_sequence=start_sequence,
                )
            )
        else:
            self._generator = iter(())
        self._generator_finished = False

    # -- Helpers ---------------------------------------------------------------

    @property
    def buffered_segments(self) -> int:
        """Number of segments currently buffered in the planner→executor queue.

        Computed from the last received planner_queue_free field:
            buffered = SEGMENT_QUEUE_DEPTH - planner_queue_free

        This mirrors Klipper's "move queue available" check: when buffered_segments
        approaches SEGMENT_QUEUE_DEPTH the host should stop requesting more motion.
        Value is 0 when no status has been received yet.
        """
        return self._buffered_segments

    def _planner_queue_free(self, status) -> int:
        """Return planner_queue_free from status, defaulting to full if absent."""
        return int(getattr(status, "planner_queue_free", self.SEGMENT_QUEUE_DEPTH))

    def _check_planner_pressure(self, status) -> bool:
        """Return True (blocked) when the ESP32 planner buffer already has enough lookahead.

        Uses Klipper's move-queue model: send if buffered < needed, not if free > threshold.
        During initial prefill (_prefilling=True) the gate is bypassed entirely so
        the host can fill up to the speed-appropriate prefill target without interference.

        Logs edge transitions:
          - 'planner pressure' when planner_queue_free drops below 16
          - 'planner recovered' when planner_queue_free recovers above 64
        """
        pqf = self._planner_queue_free(status)
        self._buffered_segments = self.SEGMENT_QUEUE_DEPTH - pqf

        if pqf < 16 and not self._planner_under_pressure:
            self._planner_under_pressure = True
            logger.debug("planner pressure: planner_queue_free=%s (< 16)", pqf)
        elif pqf > 64 and self._planner_under_pressure:
            self._planner_under_pressure = False
            logger.debug("planner recovered: planner_queue_free=%s (> 64)", pqf)

        # During prefill we bypass the pressure gate so the host can seed a deep buffer.
        if self._prefilling:
            return False

        # Klipper model: block if the buffer already holds the required lookahead depth.
        # This inverts the old "send if free slots >= threshold" gate: we now gate on
        # buffered depth rather than remaining free space, which is speed-aware.
        needed = self.required_lookahead(self._current_steps_per_segment)
        return self._buffered_segments >= needed

    def _max_segments_per_cycle(self) -> int:
        """Speed-dependent send cap per polling cycle.

        At low speed (few steps/segment) the defer ring on the ESP32 cannot
        overflow (each segment contributes <10 ring entries) so a higher cap
        is safe and necessary to keep the ring fed between underruns.
        At high speed a lower cap prevents burst-after-throttle overflow.

          < 10  steps/segment → 16 segments/cycle (low speed, 50 RPM)
          < 50  steps/segment →  8 segments/cycle (mid speed)
          >= 50 steps/segment →  4 segments/cycle (high speed, > 200 RPM)
        """
        if self._current_steps_per_segment < 10:
            return 16
        elif self._current_steps_per_segment < 50:
            return 8
        else:
            return 4

    def _max_inflight_segments(self) -> int:
        """Speed-dependent in-flight segment cap.

        At low speed each segment executes slowly so more can be in-flight
        simultaneously without risking the host advancing too far ahead
        of the motor's actual position.
        """
        if self._current_steps_per_segment < 10:
            return 96
        elif self._current_steps_per_segment < 50:
            return 48
        else:
            return 24

    def _timestamp(self) -> str:
        now = time.time()
        seconds = int(now)
        milliseconds = int((now - seconds) * 1000)
        return time.strftime(f"%H:%M:%S.{milliseconds:03d}", time.localtime(now))

    def _safe_buffer_time_s(self, requested_time_s: float, max_hz: float) -> float:
        requested_time_s = max(self.MIN_BUFFER_TIME_S, min(self.MAX_BUFFER_TIME_S, requested_time_s))
        if max_hz <= 0.0:
            return requested_time_s
        safe_time_s = (self.STEP_RING_CAPACITY * self.RING_BUFFER_HEADROOM) / max_hz
        return max(self.MIN_BUFFER_TIME_S, min(requested_time_s, safe_time_s))

    def set_generator(self, generator: Iterator[MultiAxisSegment]) -> None:
        """Override the segment generator for this streamer.

        Call before stream_all() when the segments are produced externally
        (e.g. by a WoundMove or RampMove).
        """
        self._generator = generator
        self._generator_finished = False

    def _sync_with_firmware_status(self) -> None:
        """Synchronize stream state with the ESP32's last executed sequence."""
        try:
            status = self._transport.get_status()
        except Exception:
            return

        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return

        self._last_confirmed_sequence = received_sequence
        self._last_sequence_advance_value = received_sequence
        self._last_sequence_advance_time = time.time()

    def _enable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, status = self._transport.set_axis_enabled_request(axis_id, True)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"enable axis {axis_id} failed with result=0x{status.last_result:02X}")

    def _disable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, status = self._transport.set_axis_enabled_request(axis_id, False)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"disable axis {axis_id} failed with result=0x{status.last_result:02X}")

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
            if sequence_is_less_equal(segment.sequence, last_executed):
                self._buffered_time_s -= segment.duration_us / 1_000_000.0
                self._buffered_time_s = max(0.0, self._buffered_time_s)
                self._inflight.popleft()
            else:
                break

    def _check_premature_completion(self, status) -> None:
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return

        # True premature: ESP32 reports completion of a seq we never sent.
        if self._last_sent_motion_seq >= 0 and sequence_is_greater(received_sequence, self._last_sent_motion_seq):
            now = time.time()
            if now - self._premature_notify_window_start > 1.0:
                self._premature_notify_count = 0
                self._premature_notify_window_start = now
            self._premature_notify_count += 1
            logger.warning(
                "premature completion: got seq=%s but last sent=%s — ESP32 reported completion before host sent this segment (count=%s)",
                received_sequence,
                self._last_sent_motion_seq,
                self._premature_notify_count,
            )

        # Advance confirmed pointer only when sequence strictly increases.
        if self._last_confirmed_sequence < 0 or sequence_is_greater(
            received_sequence, self._last_confirmed_sequence
        ):
            self._last_confirmed_sequence = received_sequence

    def _check_stall(self, status) -> bool:
        """Return True and request stop if last_executed_sequence has not
        advanced for _stall_timeout_s while segments are in flight.

        A stall means the RMT is dead and pushBlock() is likely deadlocked.
        Requesting a stop+flush allows the host to recover gracefully.
        """
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return False
        if not self._inflight:
            # No in-flight segments — not a stall, just idle.
            self._last_sequence_advance_time = time.time()
            return False

        if self._last_sequence_advance_value < 0 or sequence_is_greater(
            received_sequence, self._last_sequence_advance_value
        ):
            self._last_sequence_advance_value = received_sequence
            self._last_sequence_advance_time = time.time()
            return False

        elapsed = time.time() - self._last_sequence_advance_time
        if elapsed > self._stall_timeout_s:
            logger.warning(
                "motor stall detected: last_executed_sequence=%s unchanged for %.1fs with %s segments in flight — requesting stop and flush",
                received_sequence,
                elapsed,
                len(self._inflight),
            )
            self.request_stop()
            self.request_flush(self._last_sent_motion_seq)
            return True
        return False

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
            logger.debug(
                "[%s] send tx_seq=%s motion_seq=%s duration_us=%s total_steps=%s result=0x%02X",
                event["timestamp_str"],
                transport_seq,
                segment.sequence,
                segment.duration_us,
                event["total_steps"],
                event["last_result"],
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
            and len(self._inflight) < self._max_inflight_segments()
            and not self._queue_full(status)
            and not self._check_planner_pressure(status)
        ):
            try:
                segment = next(self._generator)
            except StopIteration:
                self._generator_finished = True
                break

            if self._last_sent_motion_seq >= 0 and not sequence_is_greater(
                segment.sequence, self._last_sent_motion_seq
            ):
                raise RuntimeError(
                    f"motion sequence not strictly increasing: "
                    f"got {segment.sequence}, last was {self._last_sent_motion_seq}"
                )
            batch.append(segment)
            # Keep speed estimate current so required_lookahead() uses fresh data.
            if segment.steps:
                self._current_steps_per_segment = sum(segment.steps)

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
        """Pre-send segments to seed the ESP32 planner queue before RMT starts.

        The prefill target is speed-dependent:
          - Low speed  (steps_per_segment < 10): 64 segments (half of SEGMENT_QUEUE_DEPTH)
            because the ring drains very fast at low RPM and needs a large head start.
          - Otherwise: required_lookahead(current_steps_per_segment) segments.

        The _prefilling flag is set for the duration of this call so that
        _check_planner_pressure() does not prematurely gate sends before the
        target depth has been reached.

        Returns (total_segments_sent, last_status).
        """
        is_low_speed = self._current_steps_per_segment < 10
        if is_low_speed:
            prefill_target = 64   # half of SEGMENT_QUEUE_DEPTH
        else:
            prefill_target = self.required_lookahead(self._current_steps_per_segment)

        total = 0
        last_status = status
        self._prefilling = True
        try:
            while total < prefill_target:
                result = self._collect_and_send_batch(last_status)
                if result is None:
                    break
                n, last_status = result
                total += n
                if n == 0:  # QUEUE_FULL — firmware can't accept more right now
                    break
        finally:
            self._prefilling = False
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
        axes_enabled = False

        try:
            self._enable_axes()
            axes_enabled = True
            status = self._transport.get_status()

            total_segments, status = self._prefill(status)

            while True:
                if self._stop_requested:
                    if self._flush_sequence_requested is not None:
                        self.flush_until(self._flush_sequence_requested)
                    break

                status = self._transport.get_status()
                self._remove_confirmed_segments(status)
                self._check_premature_completion(status)
                if self._check_stall(status):
                    break

                if self._check_endstop(status):
                    break

                # Rate-limit sends to MAX_SEGMENTS_PER_CYCLE per polling iteration to
                # prevent burst-after-throttle that overflows the ESP32 defer ring.
                cycle_segments_sent = 0
                while not self._generator_finished:
                    if cycle_segments_sent >= self._max_segments_per_cycle():
                        break
                    result = self._collect_and_send_batch(status)
                    if result is None:
                        break
                    n, status = result
                    if n == 0:  # QUEUE_FULL
                        break
                    cycle_segments_sent += n
                    total_segments += n
                    if total_segments % self._print_every == 0:
                        logger.debug(
                            "segments=%s buffered=%.1fms inflight=%s queue_free=%s ring_free=%s underrun=%s",
                            total_segments,
                            self._buffered_time_s * 1000.0,
                            len(self._inflight),
                            status.queue_free_slots,
                            status.ring_free_slots,
                            status.underrun_count,
                        )

                if self._flush_sequence_requested is not None:
                    self.flush_until(self._flush_sequence_requested)
                    self._flush_sequence_requested = None

                if self._generator_finished and not self._inflight:
                    break

                sleep_s = self._should_sleep()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        finally:
            if axes_enabled:
                try:
                    self._disable_axes()
                except Exception as exc:
                    logger.error("failed to disable axes: %s", exc)

        self._write_send_log()
        return total_segments
```

### src/rpi/transport/spi_transport.py
```python
from __future__ import annotations

import re
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


class Esp32SpiTransport:
    """Thin wrapper around spidev using the PickupWinder fixed SPI frame format."""

    def __init__(
        self,
        bus: int | None = None,
        device: int | None = None,
        *,
        device_path: str | None = None,
        speed_hz: int = 4_000_000,
        mode: int = 0,
    ):
        try:
            import spidev  # type: ignore
        except ImportError as exc:  # pragma: no cover - depends on host machine
            raise RuntimeError("spidev module is required on the Raspberry Pi host") from exc

        self._spidev_module = spidev
        self._spi = spidev.SpiDev()
        self._device_path = None

        if device_path is not None:
            self._device_path = device_path
            if hasattr(self._spi, "open_path"):
                self._spi.open_path(device_path)
            else:
                parsed = re.fullmatch(r"/dev/spidev(\d+)\.(\d+)", device_path)
                if parsed is None:
                    raise ValueError(
                        "device_path must be in the form /dev/spidev<bus>.<device>"
                    )
                self._spi.open(int(parsed.group(1)), int(parsed.group(2)))
        elif bus is not None and device is not None:
            self._device_path = f"/dev/spidev{bus}.{device}"
            self._spi.open(bus, device)
        else:
            raise ValueError(
                "Must specify either bus/device or device_path for SPI transport"
            )

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
            f"SPI status poll failed after 5 attempts on {self._device_path}: "
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
```

### src/rpi/motion/segment_generator.py
```python
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Iterator, Tuple

from transport.messages import MultiAxisSegment


@dataclass(slots=True)
class AxisStepProfile:
    axis_index: int
    step_at: Callable[[float], float]
    reverse_direction: bool = False
    total_duration: float = 0.0


class BaseSegmentGenerator(ABC):
    """Common iteration behavior for multi-axis segment generators."""

    def __init__(self, *, segment_duration_s: float = 0.004, start_sequence: int = 0) -> None:
        self.segment_duration_s = max(0.002, min(0.005, segment_duration_s))
        self._sequence = start_sequence & 0xFFFF
        self._time_cursor = 0.0
        self.overall_duration = 0.0

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        while self._time_cursor < self.overall_duration:
            next_cursor = min(self._time_cursor + self.segment_duration_s, self.overall_duration)
            duration_s = next_cursor - self._time_cursor
            duration_us = int(round(duration_s * 1_000_000))

            steps, directions = self._compute_segment(self._time_cursor, next_cursor)

            yield MultiAxisSegment(
                sequence=self._sequence,
                duration_us=duration_us,
                steps=steps,
                directions=directions,
            )
            self._sequence = (self._sequence + 1) & 0xFFFF
            self._time_cursor = next_cursor

    @abstractmethod
    def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
        """Return the next segment payload for the current time window."""
        ...


class StepProfileSegmentGenerator(BaseSegmentGenerator):
    def __init__(self, axis_profiles: list[AxisStepProfile], *, segment_duration_s: float = 0.004, start_sequence: int = 0) -> None:
        super().__init__(segment_duration_s=segment_duration_s, start_sequence=start_sequence)
        self.axis_profiles = axis_profiles
        self._axis_errors = [0.0 for _ in axis_profiles]
        self._current_steps = [profile.step_at(0.0) for profile in axis_profiles]
        self.overall_duration = max((profile.total_duration for profile in axis_profiles), default=0.0)

    def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
        max_axis = max((profile.axis_index for profile in self.axis_profiles), default=-1)
        steps = [0] * (max_axis + 1)
        directions = [0] * (max_axis + 1)

        for index, profile in enumerate(self.axis_profiles):
            target_steps = profile.step_at(time_end)
            delta_steps = target_steps - self._current_steps[index]
            count = int(round(self._axis_errors[index] + delta_steps))
            self._axis_errors[index] += delta_steps - float(count)
            self._current_steps[index] = target_steps

            is_negative = count < 0
            if is_negative:
                count = abs(count)
            directions[profile.axis_index] = 1 if is_negative ^ profile.reverse_direction else 0
            steps[profile.axis_index] = count

        return steps, directions
```

### src/rpi/motion/ramp_config.py
```python

from dataclasses import dataclass

from .trapezoidal_profile import TrapezoidalMotionProfile


def compute_ramp_times(
    target_rpm: float,
    duration_s: float,
    max_accel_steps_per_s2: float,
    max_decel_steps_per_s2: float,
    steps_per_rev: int,
    start_rpm: float = 0.0,
    min_ramp_s: float = 0.05,
    max_ramp_fraction: float = 0.25,
) -> tuple[float, float, float]:
    """Compute trapezoidal ramp times from machine acceleration limits.

    Implements:  ramp_s = clamp(physics_required, min_ramp_s, duration_s * max_ramp_fraction)

    The *physics_required* time is the minimum duration needed to reach
    ``target_rpm`` from ``start_rpm`` at the given acceleration limit.
    If the limit is zero (unconstrained), ``min_ramp_s`` is used directly.

    Args:
        target_rpm:             Target rotational speed in RPM.
        duration_s:             Total move duration in seconds.
        max_accel_steps_per_s2: Acceleration limit in steps/s².
        max_decel_steps_per_s2: Deceleration limit in steps/s².
        steps_per_rev:          Steps per motor revolution (full × microstep).
        start_rpm:              Initial speed in RPM (default 0).
        min_ramp_s:             Hard floor for accel / decel time (default 0.05 s).
        max_ramp_fraction:      Maximum fraction of ``duration_s`` allowed for
                                each ramp phase (default 0.25, i.e. 25 %).

    Returns:
        Tuple ``(accel_s, cruise_s, decel_s)`` that sum to at most ``duration_s``.
    """
    start_hz: float = start_rpm / 60.0 * float(steps_per_rev)
    target_hz: float = target_rpm / 60.0 * float(steps_per_rev)
    delta_hz: float = max(target_hz - start_hz, 0.0)

    if max_accel_steps_per_s2 > 0.0:
        physics_accel_s = delta_hz / max_accel_steps_per_s2
    else:
        physics_accel_s = min_ramp_s

    if max_decel_steps_per_s2 > 0.0:
        physics_decel_s = delta_hz / max_decel_steps_per_s2
    else:
        physics_decel_s = min_ramp_s

    cap = duration_s * max_ramp_fraction
    accel_s = max(min_ramp_s, min(physics_accel_s, cap))
    decel_s = max(min_ramp_s, min(physics_decel_s, cap))
    cruise_s = max(duration_s - accel_s - decel_s, 0.0)
    return accel_s, cruise_s, decel_s




@dataclass(slots=True)
class RampConfig:
    axis_id: int = 0
    steps_per_rev: int = 200 * 32
    start_rpm: float = 0.0
    target_rpm: float = 1000.0
    accel_s: float = 10.0
    cruise_s: float = 3.0
    decel_s: float = 10.0
    # Must match RMT_STEP_RESOLUTION_HZ in stepper_driver.h (80 MHz).
    resolution_hz: int = 80_000_000
    reverse_direction: bool = False
    phase_segments: int = 8
    segment_duration_s: float = 0.05

    @property
    def start_hz(self) -> float:
        return self.start_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def target_hz(self) -> float:
        return self.target_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def profile(self) -> TrapezoidalMotionProfile:
        return TrapezoidalMotionProfile(
            start_rpm=self.start_rpm,
            target_rpm=self.target_rpm,
            accel_s=self.accel_s,
            cruise_s=self.cruise_s,
            decel_s=self.decel_s,
        )

    @property
    def total_duration(self) -> float:
        return self.profile.total_duration

    def hz_at_time(self, t: float) -> float:
        return self.profile.rps_at(t) * float(self.steps_per_rev)

    def steps_at(self, t: float) -> float:
        return self.profile.steps_at(t, self.steps_per_rev)

    def step_delta(self, time_start: float, time_end: float) -> float:
        return self.profile.step_delta(time_start, time_end, self.steps_per_rev)

```

### src/rpi/motion/trapezoidal_profile.py
```python
from __future__ import annotations


class TrapezoidalMotionProfile:
    def __init__(
        self,
        start_rpm: float = 0.0,
        target_rpm: float = 1000.0,
        accel_s: float = 0.0,
        cruise_s: float = 0.0,
        decel_s: float = 0.0,
    ) -> None:
        if start_rpm < 0.0 or target_rpm < 0.0:
            raise ValueError("start_rpm and target_rpm must be non-negative")
        if accel_s < 0.0 or cruise_s < 0.0 or decel_s < 0.0:
            raise ValueError("accel_s, cruise_s, and decel_s must be >= 0")

        self.start_rpm = start_rpm
        self.target_rpm = target_rpm
        self.accel_s = accel_s
        self.cruise_s = cruise_s
        self.decel_s = decel_s

    @property
    def total_duration(self) -> float:
        return self.accel_s + self.cruise_s + self.decel_s

    def _clamp_time(self, t: float) -> float:
        return min(max(t, 0.0), self.total_duration)

    def rps_at(self, t: float) -> float:
        t = self._clamp_time(t)
        start_rps = self.start_rpm / 60.0
        target_rps = self.target_rpm / 60.0

        if t < self.accel_s:
            if self.accel_s <= 0.0:
                return target_rps
            rate = (target_rps - start_rps) / self.accel_s
            return start_rps + rate * t

        t -= self.accel_s
        if t < self.cruise_s:
            return target_rps

        t -= self.cruise_s
        if self.decel_s <= 0.0:
            return target_rps
        rate = (target_rps - start_rps) / self.decel_s
        return max(target_rps - rate * t, 0.0)

    def rpm_at(self, t: float) -> float:
        return self.rps_at(t) * 60.0

    def turns_at(self, t: float) -> float:
        t = self._clamp_time(t)
        start_rps = self.start_rpm / 60.0
        target_rps = self.target_rpm / 60.0

        if t < self.accel_s:
            if self.accel_s <= 0.0:
                return target_rps * t
            rate = (target_rps - start_rps) / self.accel_s
            return start_rps * t + 0.5 * rate * t * t

        turns = 0.0
        if self.accel_s > 0.0:
            rate = (target_rps - start_rps) / self.accel_s
            turns += start_rps * self.accel_s + 0.5 * rate * self.accel_s * self.accel_s
        else:
            turns += target_rps * self.accel_s

        t -= self.accel_s
        if t < self.cruise_s:
            return turns + target_rps * t

        turns += target_rps * self.cruise_s
        t -= self.cruise_s
        if self.decel_s <= 0.0:
            return max(turns + target_rps * t, 0.0)

        rate = (target_rps - start_rps) / self.decel_s
        result = turns + target_rps * t - 0.5 * rate * t * t
        return max(result, 0.0)

    def steps_at(self, t: float, steps_per_rev: int) -> float:
        return self.turns_at(t) * float(steps_per_rev)

    def step_delta(self, time_start: float, time_end: float, steps_per_rev: int) -> float:
        return self.steps_at(time_end, steps_per_rev) - self.steps_at(time_start, steps_per_rev)
```

### src/esp32/src/motion_planner.h
```cpp
/**
 * @file motion_planner.h
 * @brief GRBL/Klipper-inspired motion planning layer.
 *
 * ── Architecture role ──────────────────────────────────────────────────────
 *
 *   SPI ingestion (Core 0)              Planner (Core 0)           Executor (Core 1)
 *   ────────────────────                ───────────────            ─────────────────
 *   spiTask → handleFrame()   ──►   s_multi_axis_queue   ──►   plannerTask()
 *                                        (existing)              │
 *                                                                ▼
 *                                                          segment_queue_
 *                                                           (NEW bounded)
 *                                                                │
 *                                                                ▼
 *                                                        executorTask (Core 1)
 *                                                        state machine
 *                                                                │
 *                                                                ▼
 *                                                          RMT ring buffer
 *
 * The planner consumes multi_axis_block_t (bulk blocks from SPI) and
 * decomposes them into individual planned_segment_t entries with Klipper-style
 * monotonic timestamps.  The executor consumes these one at a time through a
 * bounded state machine.
 *
 * ── Backpressure ───────────────────────────────────────────────────────────
 *
 *   segment_queue_ is bounded to SEGMENT_QUEUE_DEPTH.  If the executor is
 *   slow, the planner blocks (with timeout) providing natural backpressure
 *   all the way back to the SPI ingestion queue.
 *
 * ── Flush path ─────────────────────────────────────────────────────────────
 *
 *   On flush, the planner drains both cmd_queue_ and segment_queue_, then
 *   pushes a flush sentinel (is_flush=true) so the executor can reset its
 *   state atomically.
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_err.h>
#include "step_types.h"

// ---------------------------------------------------------------------------
// Planned segment — immutable output of planner, input to executor
// ---------------------------------------------------------------------------

/**
 * @brief Per-axis motion within a planned segment.
 */
typedef struct {
    uint16_t step_count;
    bool     direction;
} planned_axis_motion_t;

/**
 * @brief One fully-planned segment ready for execution.
 *
 * Produced by the planner task, consumed by the executor task.
 * Immutable after enqueue — no synchronisation needed beyond the queue.
 */
typedef struct {
    uint16_t              motion_sequence;     ///< Host-assigned sequence ID
    uint16_t              duration_us;         ///< Wall-clock duration
    int64_t               scheduled_time_us;   ///< Monotonic execution timestamp (Klipper-style)
    uint8_t               axis_count;
    uint8_t               axis_ids[MULTI_AXIS_MAX_AXES];
    planned_axis_motion_t axes[MULTI_AXIS_MAX_AXES];
    bool                  is_flush;            ///< True = flush sentinel, not a real segment
    uint16_t              flush_sequence;      ///< Valid only when is_flush == true
} planned_segment_t;

// ---------------------------------------------------------------------------
// Executor state machine
// ---------------------------------------------------------------------------

/**
 * @brief Executor FSM states.
 *
 * The state machine ensures bounded CPU usage per iteration and eliminates
 * the nested drain loops that caused watchdog resets.
 *
 *   IDLE ──► FETCH ──► DRAIN ──► RUN ──► (back to IDLE)
 *              │                           ▲
 *              ▼                           │
 *            FLUSH ────────────────────────┘
 *              │
 *              ▼
 *           RECOVERY ──────────────────────┘
 */
enum class ExecState : uint8_t {
    IDLE,       ///< Waiting for segments (blocking queue receive)
    FETCH,      ///< Pulling segment(s) from planner queue (non-blocking batch)
    DRAIN,      ///< Writing steps to RMT ring buffer
    RUN,        ///< KickStart RMT, fire deferred notifications
    FLUSH,      ///< Processing flush sentinel — reset pipeline
    RECOVERY,   ///< Recovering from RMT underrun / error
};

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/** Segment queue: planner → executor.  ~512 ms lookahead at 4 ms/segment. */
static constexpr uint32_t SEGMENT_QUEUE_DEPTH = 128;

/** Minimum segments buffered before executor begins first RMT kickStart. */
static constexpr uint32_t SEGMENT_PREFILL_THRESHOLD = 16;

/** Maximum segments the executor fetches per FETCH iteration.
 *  Set to 1 to force frequent yields and allow planner to refill. */
static constexpr uint32_t EXEC_BATCH_LIMIT = 16;

/** Time budget per executor iteration in microseconds (watchdog safe). */
static constexpr int64_t  EXEC_TIME_BUDGET_US = 3000;

/** Planner tuning: time budget and per-iteration limit (watchdog-safe).
 *  200 µs budget allows processing 32+ segments per loop iteration at 5-10 µs/segment.
 *  Non-blocking xQueueSend ensures no watchdog blocking despite higher throughput.
 *  Higher batch size prevents executor starvation when planner runs infrequently.
 */
static constexpr int64_t  PLANNER_TIME_BUDGET_US = 2000; // µs per planner loop
static constexpr uint32_t PLANNER_MAX_SEGMENTS_PER_ITER = 60; // segments per loop to balance yield

// ---------------------------------------------------------------------------
// MotionPlanner class
// ---------------------------------------------------------------------------

class MotionPlanner {
public:
    MotionPlanner();

    /**
     * @brief Initialise the segment output queue and launch the planner task.
     *
     * @param cmd_queue   Existing s_multi_axis_queue (SPI → planner input).
     * @param flush_queue Existing s_flush_queue (SPI → planner input).
     * @return ESP_OK on success.
     */
    esp_err_t init(QueueHandle_t cmd_queue, QueueHandle_t flush_queue);

    /** @brief Output queue handle for the executor to consume. */
    QueueHandle_t segmentQueue() const { return segment_queue_; }

    /** @brief Number of free slots in the segment output queue. */
    uint32_t segmentQueueFree() const;

    // ── Statistics ──────────────────────────────────────────────────────────
    uint32_t segmentsPlanned() const { return segments_planned_; }
    uint32_t segmentsDropped() const { return segments_dropped_; }

private:
    QueueHandle_t cmd_queue_     {nullptr};  ///< Input: s_multi_axis_queue
    QueueHandle_t flush_queue_   {nullptr};  ///< Input: s_flush_queue
    QueueHandle_t segment_queue_ {nullptr};  ///< Output: planned_segment_t

    int64_t  timeline_us_       {0};         ///< Monotonic scheduling timeline
    uint32_t segments_planned_  {0};
    uint32_t segments_dropped_  {0};

    // ── Incremental planner state (to avoid burst processing) ───────────
    multi_axis_block_t pending_block_ {};   ///< Currently-being-expanded block
    uint16_t            pending_segment_idx_ {0};
    bool                has_pending_block_   {false};

    // Non-blocking flush sentinel retry state
    bool     flush_pending_ {false};
    uint16_t pending_flush_sequence_ {0};

    /**
     * @brief Expand one multi_axis_block_t into planned_segment_t entries.
     *
     * Each segment in the block becomes one planned_segment_t with a
     * Klipper-style monotonic timestamp derived from cumulative duration.
     * Enqueues to segment_queue_ with bounded backpressure wait.
     */
    void planBlock(const multi_axis_block_t& block);

    /**
     * @brief Handle a flush request: drain queues, push flush sentinel.
     */
    void handleFlush(const flush_request_t& req);

    /**
     * @brief Planner task body.
     *
     * Pinned to Core 0, priority 8 (below SPI task at 10, above idle).
     * Runs in SPI task's idle time between spi_slave_transmit() calls.
     */
    static void plannerTask(void* arg);
};
```

### src/esp32/src/motion_planner.cpp
```cpp
/**
 * @file motion_planner.cpp
 * @brief GRBL/Klipper-inspired motion planning layer — implementation.
 *
 * The planner task sits between SPI ingestion and the execution layer:
 *
 *   s_multi_axis_queue ──► plannerTask() ──► segment_queue_ ──► executor
 *
 * Responsibilities:
 *   • Decompose multi_axis_block_t (bulk) into individual planned_segment_t
 *   • Assign Klipper-style monotonic timestamps (scheduled_time_us)
 *   • Handle flush requests: drain both input and output queues
 *   • Provide bounded backpressure to the SPI ingestion layer
 *
 * Non-responsibilities (executor owns these):
 *   • No hardware access (no RMT, no GPIO, no ring buffer)
 *   • No endstop checking (real-time, must be in executor)
 *   • No StepperDriver interaction
 *
 * This separation means the planner is fully preemptible and testable
 * without hardware dependencies.
 */

#include "motion_planner.h"

#include <string.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>

static const char* TAG = "planner";

// ---------------------------------------------------------------------------
// Task parameters
// ---------------------------------------------------------------------------

static constexpr uint32_t    PLANNER_STACK = 4096;
static constexpr UBaseType_t PLANNER_PRIO  = 12;  // Above SPI (10), above idle
static constexpr BaseType_t  PLANNER_CORE  = 0;   // Same core as SPI task

/** Max blocks drained from cmd_queue_ during a single flush operation. */
static constexpr uint32_t MAX_FLUSH_DRAIN = 16;

/** Backpressure timeout: how long the planner waits for segment_queue_ space
 *  before dropping a segment.  10 ms is ~2.5 segments at 4 ms/segment. */
static constexpr TickType_t BACKPRESSURE_TIMEOUT = pdMS_TO_TICKS(10);

/** Planner poll interval when no command blocks are available. */
static constexpr TickType_t CMD_POLL_TIMEOUT = pdMS_TO_TICKS(5);

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MotionPlanner::MotionPlanner() {}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t MotionPlanner::init(QueueHandle_t cmd_queue, QueueHandle_t flush_queue)
{
    cmd_queue_   = cmd_queue;
    flush_queue_ = flush_queue;

    segment_queue_ = xQueueCreate(SEGMENT_QUEUE_DEPTH, sizeof(planned_segment_t));
    ESP_RETURN_ON_FALSE(segment_queue_ != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create segment queue");

    BaseType_t rc = xTaskCreatePinnedToCore(
        &MotionPlanner::plannerTask,
        "planner",
        PLANNER_STACK,
        this,
        PLANNER_PRIO,
        nullptr,
        PLANNER_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create planner task");

    ESP_LOGI(TAG, "planner ready: seg_queue_depth=%lu  core=%d  pri=%d",
             (unsigned long)SEGMENT_QUEUE_DEPTH,
             (int)PLANNER_CORE, (int)PLANNER_PRIO);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// segmentQueueFree()
// ---------------------------------------------------------------------------

uint32_t MotionPlanner::segmentQueueFree() const
{
    if (segment_queue_ == nullptr) return 0;
    return static_cast<uint32_t>(uxQueueSpacesAvailable(segment_queue_));
}

// ---------------------------------------------------------------------------
// planBlock()
// ---------------------------------------------------------------------------

void MotionPlanner::planBlock(const multi_axis_block_t& block)
{
    // Legacy path kept for compatibility: copy into pending buffer so the
    // real work happens incrementally inside plannerTask.  This avoids
    // burst CPU usage and guarantees non-blocking behavior.
    if (!has_pending_block_) {
        pending_block_ = block;
        pending_segment_idx_ = 0;
        has_pending_block_ = true;
        // Ensure timeline is clamped to now if it drifted into the past.
        const int64_t now_us = esp_timer_get_time();
        if (timeline_us_ < now_us) timeline_us_ = now_us;
    } else {
        // Already processing a block — drop this incoming block (should be
        // rare because the SPI layer back-pressures). Count as dropped.
        ++segments_dropped_;
        ESP_LOGW(TAG, "incoming block dropped: planner busy (dropped total=%lu)",
                 (unsigned long)segments_dropped_);
    }
}

// ---------------------------------------------------------------------------
// handleFlush()
// ---------------------------------------------------------------------------

void MotionPlanner::handleFlush(const flush_request_t& req)
{
    // Non-blocking flush: mark pending state and drop any currently stored
    // pending_block_.  We will attempt to push a flush sentinel to the
    // output queue without blocking; if that fails we remember the flush
    // and retry on the next planner loop iteration.
    has_pending_block_ = false; // drop current pending block
    timeline_us_ = esp_timer_get_time();
    planned_segment_t flush_seg {};
    flush_seg.is_flush = true;
    flush_seg.flush_sequence = req.flush_sequence;
    if (xQueueSend(segment_queue_, &flush_seg, 0) == pdTRUE) {
        flush_pending_ = false;
        ESP_LOGI(TAG, "flush sentinel posted seq=%u", req.flush_sequence);
    } else {
        // Queue full — remember to retry later.
        flush_pending_ = true;
        pending_flush_sequence_ = req.flush_sequence;
        ESP_LOGW(TAG, "flush sentinel queued later seq=%u", req.flush_sequence);
    }
}

// ---------------------------------------------------------------------------
// plannerTask()
// ---------------------------------------------------------------------------

void MotionPlanner::plannerTask(void* arg)
{
    auto* self = static_cast<MotionPlanner*>(arg);
    multi_axis_block_t block;
    flush_request_t flush_req;

    ESP_LOGI(TAG, "planner task running on core %d", xPortGetCoreID());

    for (;;) {
        const int64_t loop_start = esp_timer_get_time();

        // 1) Handle any flush requests immediately (non-blocking).
        if (xQueueReceive(self->flush_queue_, &flush_req, 0) == pdTRUE) {
            self->handleFlush(flush_req);
        }

        // 2) If a previous flush sentinel failed to post, retry non-blocking.
        if (self->flush_pending_) {
            planned_segment_t flush_seg {};
            flush_seg.is_flush = true;
            flush_seg.flush_sequence = self->pending_flush_sequence_;
            if (xQueueSend(self->segment_queue_, &flush_seg, 0) == pdTRUE) {
                self->flush_pending_ = false;
                ESP_LOGI(TAG, "flush sentinel posted retry seq=%u",
                         (unsigned)self->pending_flush_sequence_);
            }
        }

        // 3) If we don't have a pending block, try to pull one non-blocking.
        if (!self->has_pending_block_) {
            if (xQueueReceive(self->cmd_queue_, &block, 0) == pdTRUE) {
                // Store for incremental expansion.
                self->pending_block_ = block;
                self->pending_segment_idx_ = 0;
                self->has_pending_block_ = true;
                // Clamp timeline if idle.
                const int64_t now_us = esp_timer_get_time();
                if (self->timeline_us_ < now_us) self->timeline_us_ = now_us;
            }
        }

        // 4) Process up to PLANNER_MAX_SEGMENTS_PER_ITER segments from the
        //    pending_block_ within the time budget. All queue sends are
        //    non-blocking (timeout=0); on failure we drop the segment and
        //    advance so the planner never stalls.
        uint32_t processed = 0;
        while (self->has_pending_block_ &&
               processed < PLANNER_MAX_SEGMENTS_PER_ITER &&
               (esp_timer_get_time() - loop_start) < PLANNER_TIME_BUDGET_US) {

            const uint16_t idx = self->pending_segment_idx_;
            if (idx >= self->pending_block_.segment_count) {
                // Finished this block.
                self->has_pending_block_ = false;
                break;
            }

            const multi_axis_segment_t& src = self->pending_block_.segments[idx];

            planned_segment_t seg {};
            seg.motion_sequence = src.motion_sequence;
            seg.duration_us     = src.duration_us;
            seg.scheduled_time_us = self->timeline_us_;
            seg.axis_count      = self->pending_block_.axis_count;
            seg.is_flush        = false;

            const uint8_t n = (self->pending_block_.axis_count < MULTI_AXIS_MAX_AXES)
                              ? self->pending_block_.axis_count : MULTI_AXIS_MAX_AXES;
            for (uint8_t a = 0; a < n; ++a) {
                seg.axis_ids[a]        = self->pending_block_.axis_ids[a];
                seg.axes[a].step_count = src.step_counts[a];
                seg.axes[a].direction  = ((src.direction_mask >> a) & 1u) != 0;
            }

            // Advance timeline and enqueue atomically: only advance if the
            // segment was successfully enqueued so the timeline stays in sync.
            // On full queue, break out of the inner loop — the executor will
            // drain a slot, and we will retry this segment on the next
            // planner iteration (natural backpressure, no data loss).
            if (xQueueSend(self->segment_queue_, &seg, 0) == pdTRUE) {
                self->timeline_us_ += static_cast<int64_t>(src.duration_us);
                ++self->segments_planned_;
                ++self->pending_segment_idx_;
                ++processed;
            } else {
                // Queue full — yield and retry this segment next iteration.
                break;
            }
        }

        // 5) Yield behavior: if we processed nothing, sleep briefly to let
        //    IDLE0 and other low-priority tasks run and reset the watchdog.
        if (processed == 0) {
            vTaskDelay(1);
        } else {
            taskYIELD();
        }
    }
}
```

### src/esp32/src/stepper_driver.cpp
```cpp
/**
 * @file stepper_driver.cpp
 * @brief Physical-layer RMT stepper driver — streaming simple_encoder impl.
 *
 * See stepper_driver.h for architecture details.
 */

#include "stepper_driver.h"

#include <algorithm>
#include <cstring>
#include <esp_log.h>
#include <esp_check.h>
#include <hal/gpio_ll.h>

static const char* TAG = "stepper_driver";

// ---------------------------------------------------------------------------
// encode_steps() — simple_encoder callback, runs in ISR context (IRAM)
// ---------------------------------------------------------------------------
//
// RMT clock: 80 MHz (1 tick = 12.5 ns).
// Called by the RMT driver whenever it needs more symbols. Reads up to
// PART_SIZE=16 entries from the ring buffer and converts each to one
// rmt_symbol_word_t with a FastAccelStepper-style balanced pulse:
//   HIGH = ticks / 2         (rounded down)
//   LOW  = ticks − HIGH
// Both halves are clamped to >= RMT_STEP_PULSE_TICKS (8) = 100 ns,
// which meets A4988/DRV8825 STEP pulse width requirements.
//
// Direction changes:
//   If a ring entry has toggle_dir=1 and the previous chunk contained steps,
//   emit a pause chunk first (to meet driver IC setup time), then toggle
//   DIR on the next callback invocation.
//
// On starvation, the callback follows the same conservative policy as
// FastAccelStepper's ESP32 IDF5 backend: emit one LOW-level pause chunk,
// arm stop, and let the next callback finish the transaction.

extern "C" size_t IRAM_ATTR encode_steps(const void* /*data*/,
                                          size_t /*data_size*/,
                                          size_t /*symbols_written*/,
                                          size_t symbols_free,
                                          rmt_symbol_word_t* symbols,
                                          bool* done, void* arg)
{
    StepperDriver* drv = static_cast<StepperDriver*>(arg);
    *done = false;

    if (symbols_free < PART_SIZE) {
        return 0;  // Wait for more space
    }

    uint32_t rd = drv->ring_read_.load(std::memory_order_acquire);
    uint32_t wr = drv->ring_write_.load(std::memory_order_acquire);

    // Check for explicit stop request
    if (drv->rmt_stopped_.load(std::memory_order_relaxed)) {
        *done = true;
        return 0;
    }

    // Check for endstop trigger — stop immediately without emitting any
    // further step pulses. The ring is NOT reset here; emergencyStop() is
    // called from the executor task after it detects endstop_active_.
    if (drv->endstop_active_.load(std::memory_order_relaxed)) {
        drv->rmt_stopped_.store(true, std::memory_order_relaxed);
        *done = true;
        return 0;
    }

    // Ring empty — emit one LOW-level pause chunk, arm stop, and let the
    // next callback terminate the transmission.
    if (rd == wr) {
        drv->last_chunk_had_steps_ = false;
        drv->ring_underrun_count_.fetch_add(1, std::memory_order_relaxed);
        drv->rmt_stopped_.store(true, std::memory_order_relaxed);
        uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
        for (uint32_t i = 0; i < PART_SIZE; i++) {
            symbols[i].level0    = 0;
            symbols[i].duration0 = t;
            symbols[i].level1    = 0;
            symbols[i].duration1 = t;
        }
        return PART_SIZE;
    }

    // Data is available after underrun — clear the stop flag so we can continue
    // encoding. This handles the case where the ring was empty, we emitted a
    // pause, and now new data has arrived before on_trans_done_isr fires.
    drv->rmt_stopped_.store(false, std::memory_order_relaxed);

    // Peek at next entry — check for direction change
    ring_entry_t* entry = &drv->ring_[rd & STEP_RING_MASK];
    if (entry->toggle_dir) {
        if (drv->last_chunk_had_steps_) {
            // Previous chunk had steps — emit a fixed pause chunk so the next
            // callback can toggle DIR safely at the chunk boundary.
            drv->last_chunk_had_steps_ = false;
            uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
            for (uint32_t i = 0; i < PART_SIZE; i++) {
                symbols[i].duration0 = t;
                symbols[i].level0    = 0;
                symbols[i].duration1 = t;
                symbols[i].level1    = 0;
            }
            return PART_SIZE;
        }
        // Safe to toggle now (previous chunk was a pause or first chunk)
        gpio_ll_set_level(&GPIO, drv->dir_pin_,
                          gpio_ll_get_level(&GPIO, drv->dir_pin_) ^ 1);
        entry->toggle_dir = 0;
    }

    // Fill PART_SIZE symbols from ring buffer
    bool has_steps = false;
    for (uint32_t i = 0; i < PART_SIZE; i++) {
        if (rd != wr) {
            ring_entry_t* e = &drv->ring_[rd & STEP_RING_MASK];

            // Handle mid-chunk direction changes: stop filling, pad the
            // remainder with a fixed LOW-level pause chunk.
            if (e->toggle_dir && i > 0) {
                uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
                for (uint32_t j = i; j < PART_SIZE; j++) {
                    symbols[j].duration0 = t;
                    symbols[j].level0    = 0;
                    symbols[j].duration1 = t;
                    symbols[j].level1    = 0;
                }
                break;
            }

            uint16_t t = e->ticks;
            uint16_t high_ticks = t >> 1;
            uint16_t low_ticks = t - high_ticks;
            if (high_ticks < RMT_STEP_PULSE_TICKS) {
                high_ticks = RMT_STEP_PULSE_TICKS;
                low_ticks = t - high_ticks;
            }
            if (low_ticks < RMT_STEP_PULSE_TICKS) {
                low_ticks = RMT_STEP_PULSE_TICKS;
                high_ticks = t - low_ticks;
            }
            drv->last_ticks_ = t;
            symbols[i].level0    = 1;
            symbols[i].duration0 = high_ticks;
            symbols[i].level1    = 0;
            symbols[i].duration1 = low_ticks;

            rd++;
            has_steps = true;
        } else {
            // Ring exhausted mid-chunk — pad the remainder with a pause chunk,
            // arm stop, and let the next callback terminate the transaction.
            drv->ring_underrun_count_.fetch_add(1, std::memory_order_relaxed);
            drv->rmt_stopped_.store(true, std::memory_order_relaxed);
            uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
            for (uint32_t j = i; j < PART_SIZE; j++) {
                symbols[j].level0    = 0;
                symbols[j].duration0 = t;
                symbols[j].level1    = 0;
                symbols[j].duration1 = t;
            }
            break;
        }
    }

    drv->ring_read_.store(rd, std::memory_order_release);
    drv->last_chunk_had_steps_ = has_steps;

    TaskHandle_t prod = drv->producer_task_.load(std::memory_order_relaxed);
    TaskHandle_t exec = drv->executor_task_.load(std::memory_order_relaxed);
    if (prod != nullptr || exec != nullptr) {
        BaseType_t woken = pdFALSE;
        if (prod != nullptr) {
            vTaskNotifyGiveFromISR(prod, &woken);
        }
        if (exec != nullptr && exec != prod) {
            vTaskNotifyGiveFromISR(exec, &woken);
        }
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    return PART_SIZE;
}

// ---------------------------------------------------------------------------
// endstopIsrHandler()  — GPIO ISR, IRAM_ATTR
// ---------------------------------------------------------------------------
//
// Fires on any edge of either endstop contact (NO or NC).
// Validates the dual-contact NO/NC logic to guard against noise and cable breaks:
//   NO=0, NC=1 → endstop CLOSED (triggered) → set endstop_active_
//   NO=1, NC=0 → endstop OPEN  (released)   → clear endstop_active_
//   NO==NC      → ABSENT or cable break       → fail-safe: set endstop_active_
//
// arg = StepperDriver* (owns all needed state — no CommInterface dependency).

void IRAM_ATTR StepperDriver::endstopIsrHandler(void* arg)
{
    StepperDriver* drv = static_cast<StepperDriver*>(arg);

    if (!drv->isEndstopArmed()) {
        return;
    }

    const int no_lvl = gpio_get_level(drv->endstop_no_pin_);
    const int nc_lvl = gpio_get_level(drv->endstop_nc_pin_);

    const bool triggered = (no_lvl == 0 && nc_lvl == 1) || (no_lvl == nc_lvl);

    if (triggered) {
        drv->endstop_active_.store(true, std::memory_order_release);
        // Wake the executor task so it drains the pipeline immediately.
        BaseType_t woken = pdFALSE;
        TaskHandle_t exec = drv->executor_task_.load(std::memory_order_relaxed);
        if (exec != nullptr) {
            vTaskNotifyGiveFromISR(exec, &woken);
        }
        if (woken) portYIELD_FROM_ISR();
    } else {
        // Endstop released — clear flag.
        // Host must re-arm via SPI ENABLE_ENDSTOP before the next move.
        drv->endstop_active_.store(false, std::memory_order_release);
    }
}

// ---------------------------------------------------------------------------
// initEndstopIsr()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::initEndstopIsr(gpio_num_t no_pin, gpio_num_t nc_pin)
{
    if (no_pin == GPIO_NUM_NC || nc_pin == GPIO_NUM_NC) {
        ESP_LOGI(TAG, "motor%u: endstop pins not configured — ISR not installed",
                 motor_id_);
        return ESP_OK;
    }

    endstop_no_pin_ = no_pin;
    endstop_nc_pin_ = nc_pin;

    // gpio_install_isr_service returns ESP_ERR_INVALID_STATE if already called.
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "motor%u: gpio_install_isr_service failed: %s",
                 motor_id_, esp_err_to_name(err));
        return err;
    }

    const gpio_num_t pins[2] = { no_pin, nc_pin };
    for (gpio_num_t pin : pins) {
        ESP_RETURN_ON_ERROR(
            gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE),
            TAG, "gpio_set_intr_type failed for pin %d", (int)pin);
        ESP_RETURN_ON_ERROR(
            gpio_isr_handler_add(pin, &StepperDriver::endstopIsrHandler, this),
            TAG, "gpio_isr_handler_add failed for pin %d", (int)pin);
    }

    ESP_LOGI(TAG, "motor%u: endstop ISR installed NO=GPIO%d NC=GPIO%d",
             motor_id_, (int)no_pin, (int)nc_pin);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

StepperDriver::StepperDriver(gpio_num_t step_pin, gpio_num_t dir_pin,
                             gpio_num_t en_pin,   uint8_t    motor_id)
    : dir_pin_(dir_pin)
    , step_pin_(step_pin)
    , en_pin_(en_pin)
    , motor_id_(motor_id)
{
    memset(ring_, 0, sizeof(ring_));
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::init()
{
    // ── 1. DIR and EN GPIO ──────────────────────────────────────────────────
    gpio_config_t io_conf = {};
    io_conf.mode          = GPIO_MODE_OUTPUT;
    io_conf.intr_type     = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask  = (1ULL << dir_pin_) | (1ULL << en_pin_);
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: gpio_config DIR/EN failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    gpio_set_level(en_pin_,  1);
    gpio_set_level(dir_pin_, last_dir_ ? 1 : 0);

    // ── 2. RMT TX channel ───────────────────────────────────────────────────
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num           = step_pin_;
    tx_cfg.clk_src            = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz      = RMT_STEP_RESOLUTION_HZ;
    tx_cfg.mem_block_symbols  = RMT_MEM_SYMBOLS;
    tx_cfg.trans_queue_depth  = 4;
    tx_cfg.flags.invert_out   = false;
    tx_cfg.flags.with_dma     = false;

    err = rmt_new_tx_channel(&tx_cfg, &channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_new_tx_channel failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    // ── 3. Simple encoder with callback ─────────────────────────────────────
    rmt_simple_encoder_config_t enc_cfg = {};
    enc_cfg.callback       = encode_steps;
    enc_cfg.arg            = this;
    enc_cfg.min_chunk_size = PART_SIZE;

    err = rmt_new_simple_encoder(&enc_cfg, &encoder_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_new_simple_encoder failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    // ── 4. Transmit config ──────────────────────────────────────────────────
    tx_config_.loop_count              = 0;
    tx_config_.flags.eot_level         = 0;
    tx_config_.flags.queue_nonblocking = 1;

    // ── 5. on_trans_done callback ───────────────────────────────────────────
    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = &StepperDriver::on_trans_done_isr;
    err = rmt_tx_register_event_callbacks(channel_, &cbs, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_tx_register_event_callbacks failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    // ── 6. Enable the RMT channel ───────────────────────────────────────────
    err = rmt_enable(channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_enable failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "motor%u: init OK  step=GPIO%d  dir=GPIO%d  en=GPIO%d  "
                  "ring=%u  part=%u",
             motor_id_, (int)step_pin_, (int)dir_pin_, (int)en_pin_,
             STEP_RING_SIZE, PART_SIZE);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// enable / disable / emergencyStop
// ---------------------------------------------------------------------------

void StepperDriver::enable()
{
    gpio_set_level(en_pin_, 0);
    enabled_ = true;
}

void StepperDriver::disable()
{
    gpio_set_level(en_pin_, 1);
    enabled_ = false;
}

void StepperDriver::emergencyStop()
{
    rmt_disable(channel_);
    rmt_enable(channel_);

    ring_read_.store(0, std::memory_order_relaxed);
    ring_write_.store(0, std::memory_order_relaxed);
    rmt_running_.store(false, std::memory_order_relaxed);
    rmt_stopped_.store(true, std::memory_order_relaxed);
    last_chunk_had_steps_ = false;

    ESP_LOGW(TAG, "motor%u: emergency stop", motor_id_);
}

// ---------------------------------------------------------------------------
// stopStream()
// ---------------------------------------------------------------------------

void StepperDriver::stopStream()
{
    if (!rmt_running_.load(std::memory_order_relaxed)) return;

    // Signal the encoder callback to end the transmission
    rmt_stopped_.store(true, std::memory_order_release);

    // Wait for the RMT hardware to finish the current transaction
    rmt_tx_wait_all_done(channel_, pdMS_TO_TICKS(500));
    rmt_running_.store(false, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// startStream()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::startStream()
{
    rmt_running_.store(true, std::memory_order_release);
    rmt_stopped_.store(false, std::memory_order_release);
    last_chunk_had_steps_ = false;

    // `this` is in internal DRAM (static global) — passes esp_ptr_internal()
    // check. sizeof(*this) > 0 passes payload_bytes != 0. The callback ignores
    // both data and data_size entirely. Reset the encoder so each new
    // transaction restarts from symbol position 0.
    encoder_->reset(encoder_);
    esp_err_t err = rmt_transmit(channel_, encoder_, this, sizeof(*this), &tx_config_);
    if (err != ESP_OK) {
        rmt_running_.store(false, std::memory_order_relaxed);
        rmt_stopped_.store(true, std::memory_order_relaxed);
        ESP_LOGE(TAG, "motor%u: rmt_transmit failed: %s",
                 motor_id_, esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------------------------------
// gracefulStop()
// ---------------------------------------------------------------------------

void StepperDriver::gracefulStop()
{
    // Signal the encoder callback to stop after the current ring contents
    // have been consumed (no ring reset, unlike emergencyStop).
    rmt_stopped_.store(true, std::memory_order_release);
    // Do not call rmt_tx_wait_all_done here — the caller should not block.
    // The RMT transaction will end naturally after the pause chunk fires.
    rmt_running_.store(false, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// pushBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::pushBlock(const step_block_t& block, TaskHandle_t caller_task)
{
    if (ring_underrun_count_.load(std::memory_order_relaxed) > 0) {
        ESP_LOGW(TAG, "motor%u: ring underrun x%lu since last pushBlock",
                 motor_id_, (unsigned long)ring_underrun_count_.load(std::memory_order_relaxed));
        ring_underrun_count_.store(0, std::memory_order_relaxed);
    }

    if (block.count == 0) {
        return ESP_OK;
    }

    // Always update producer_task_ unconditionally so the ISR ring-space
    // notification always wakes the task that is actually blocked here,
    // not a stale handle from a previous call.
    producer_task_.store((caller_task != nullptr)
                     ? caller_task
                     : xTaskGetCurrentTaskHandle(),
                     std::memory_order_release);

    const uint32_t count = std::min<uint32_t>(block.count, STEP_BLOCK_SIZE);

    const bool new_dir = block.steps[0].direction;
    bool need_toggle = (new_dir != last_dir_);

    // If the ring is empty and the motor is idle, explicitly set the DIR pin
    // to the requested direction now. This avoids relying on the initial
    // `last_dir_` state and ensures reverse mode is applied on the first block.
    if (!rmt_running_.load(std::memory_order_relaxed) &&
        ring_read_.load(std::memory_order_relaxed) == ring_write_.load(std::memory_order_relaxed) &&
        need_toggle) {
        gpio_set_level(dir_pin_, new_dir ? 1 : 0);
        last_dir_ = new_dir;
        need_toggle = false;
    } else {
        last_dir_ = new_dir;
    }

    // If the endstop already fired before we even start writing, abort.
    if (endstop_active_.load(std::memory_order_acquire)) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint32_t i = 0; i < count; i++) {
        // Back-pressure: wait until the encoder ISR has consumed at least one
        // chunk and notified this producer task. This avoids a CPU1 spin loop
        // and keeps the task watchdog satisfied.
        while (ringFree() == 0) {
            // The endstop ISR also notifies this task.  If it fires while we
            // are blocked here, break out immediately instead of spinning.
            if (endstop_active_.load(std::memory_order_acquire)) {
                return ESP_ERR_INVALID_STATE;
            }
            // If RMT is not running and ring is full, the encoder callback
            // will never fire and ring_read_ will never advance.
            // Kick startStream() directly instead of waiting forever.
            if (!rmt_running_.load(std::memory_order_acquire)) {
                esp_err_t kick_err = startStream();
                if (kick_err != ESP_OK) {
                    ESP_LOGW(TAG, "motor%u: pushBlock kick startStream: %s",
                             motor_id_, esp_err_to_name(kick_err));
                }
            }
            ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(5));
            if (endstop_active_.load(std::memory_order_acquire)) {
                return ESP_ERR_INVALID_STATE;
            }
        }

        uint32_t ticks = block.steps[i].interval_ticks;

        ticks = std::max<uint32_t>(ticks, RMT_STEP_MIN_TICKS);
        ticks = std::min<uint32_t>(ticks, RMT_STEP_MAX_TICKS);

        uint32_t wr = ring_write_.load(std::memory_order_relaxed);
        ring_entry_t* e = &ring_[wr & STEP_RING_MASK];
        e->ticks      = static_cast<uint16_t>(ticks);
        e->toggle_dir = (i == 0 && need_toggle) ? 1 : 0;
        e->pad        = 0;

        ring_write_.store(wr + 1, std::memory_order_release);
    }

    // NOTE: startStream() is NOT called here.
    //
    // The executor task (stepper_queue.cpp) calls startStream() explicitly after
    // draining all available FreeRTOS queue blocks into the ring. This maximises
    // ring fill before the RMT starts, which is critical at high step rates where
    // a single 64-step block lasts less than one SPI round-trip.
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// on_trans_done ISR
// ---------------------------------------------------------------------------

bool IRAM_ATTR StepperDriver::on_trans_done_isr(
    rmt_channel_handle_t /*tx_chan*/,
    const rmt_tx_done_event_data_t* /*edata*/,
    void* user_ctx)
{
    StepperDriver* self = static_cast<StepperDriver*>(user_ctx);
    self->rmt_running_.store(false, std::memory_order_relaxed);
    BaseType_t woken = pdFALSE;
    TaskHandle_t prod = self->producer_task_.load(std::memory_order_relaxed);
    if (prod != nullptr) {
        vTaskNotifyGiveFromISR(prod, &woken);
    }
    TaskHandle_t exec = self->executor_task_.load(std::memory_order_relaxed);
    if (exec != nullptr && exec != prod) {
        vTaskNotifyGiveFromISR(exec, &woken);
    }
    return woken == pdTRUE;
}
```

### src/esp32/src/comm_interface.cpp
```cpp
/**
 * @file comm_interface.cpp
 * @brief SPI slave communication interface implementation.
 *
 * The ESP32 is the SPI slave. Every transfer is a fixed-size wire frame:
 *
 *   Host TX frame  ──► ESP32 parses and executes request
 *   Host RX frame ◄── ESP32 returns latest status payload
 *
 * Status is therefore naturally pipelined by one SPI transaction, which keeps
 * the slave task simple and deterministic.
 */

#include "comm_interface.h"

#include <string.h>
#include <driver/spi_slave.h>
#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "motion_planner.h"

static const char* TAG = "comm_iface";

static constexpr uint32_t  SPI_TASK_STACK  = 4096;
static constexpr UBaseType_t SPI_TASK_PRIO = 10;
static constexpr BaseType_t  SPI_TASK_CORE = 0;

static constexpr uint32_t    MULTI_EXEC_STACK  = 8192;
static constexpr UBaseType_t MULTI_EXEC_PRIO   = 20;
static constexpr BaseType_t  MULTI_EXEC_CORE   = 1;

/**
 * @brief Hard wall-clock budget for one drain-loop sub-slice before the
 *        executor unconditionally yields to the FreeRTOS scheduler.
 *
 * The yield is UNCONDITIONAL — it does NOT depend on ring-fill level.
 * RMT pulse timing is in hardware, so a 1 ms scheduler sleep never
 * introduces step jitter.
 *
 * NOTE: Superseded by EXEC_TIME_BUDGET_US in motion_planner.h for the
 * state-machine executor. Retained for the per-axis executorTask in
 * stepper_queue.cpp which still uses it indirectly.
 */
static constexpr int64_t  YIELD_INTERVAL_US      = 400;

/**
 * @brief Maximum queue entries drained in a single bounded flush loop.
 *
 * Used by handleFlush() in the SPI ingestion path only.
 * The planner and executor have their own bounded drain constants.
 */
static constexpr uint32_t MAX_FLUSH_DRAIN        = 8;

/**
 * @brief Global queue of multi-axis segment blocks fed by the SPI task and
 *        consumed by the multi-axis executor task.
 *
 * Depth is sized to hold ~600 ms of motion at 4 ms/segment.
 */
static constexpr uint32_t MULTI_AXIS_QUEUE_DEPTH = 64;
static QueueHandle_t s_multi_axis_queue  = nullptr;

/**
 * @brief Global queue for flush requests.  Depth 4 is more than enough since
 *        the host can only issue one flush at a time.
 */
static constexpr uint32_t FLUSH_QUEUE_DEPTH = 4;
static QueueHandle_t s_flush_queue = nullptr;

DMA_ATTR static uint8_t s_rx_frame[SPI_FRAME_SIZE];
DMA_ATTR static uint8_t s_tx_frame_a[SPI_FRAME_SIZE];
DMA_ATTR static uint8_t s_tx_frame_b[SPI_FRAME_SIZE];

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommInterface::CommInterface(StepperQueue* queues[], uint8_t n_motors)
    : n_motors_(n_motors < SPI_MAX_AXES ? n_motors : SPI_MAX_AXES)
{
    for (uint8_t i = 0; i < SPI_MAX_AXES; ++i) {
        queues_[i] = (i < n_motors_) ? queues[i] : nullptr;
    }
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t CommInterface::init(const SpiBusPins& pins)
{
    pins_ = pins;

    if (pins_.home_pin_no != GPIO_NUM_NC && pins_.home_pin_nc != GPIO_NUM_NC) {
        gpio_config_t home_cfg = {};
        home_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(pins_.home_pin_no))
                               | (1ULL << static_cast<uint32_t>(pins_.home_pin_nc));
        home_cfg.mode = GPIO_MODE_INPUT;
        home_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        home_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        home_cfg.intr_type = GPIO_INTR_DISABLE;
        ESP_RETURN_ON_ERROR(gpio_config(&home_cfg), TAG, "failed to configure home sensor pins");
    }

    // Create the global multi-axis segment queue.
    s_multi_axis_queue = xQueueCreate(MULTI_AXIS_QUEUE_DEPTH, sizeof(multi_axis_block_t));
    ESP_RETURN_ON_FALSE(s_multi_axis_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create multi-axis queue");

    // Create the global flush request queue.
    s_flush_queue = xQueueCreate(FLUSH_QUEUE_DEPTH, sizeof(flush_request_t));
    ESP_RETURN_ON_FALSE(s_flush_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create flush queue");

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = pins_.mosi;
    bus_cfg.miso_io_num = pins_.miso;
    bus_cfg.sclk_io_num = pins_.sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = SPI_FRAME_SIZE;

    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.mode = 0;
    slave_cfg.spics_io_num = pins_.cs;
    slave_cfg.queue_size = 1;
    slave_cfg.flags = 0;
    slave_cfg.post_setup_cb = nullptr;
    slave_cfg.post_trans_cb = nullptr;

    ESP_RETURN_ON_ERROR(
        spi_slave_initialize(SPI3_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO),
        TAG, "spi_slave_initialize failed");

    BaseType_t rc = xTaskCreatePinnedToCore(
        &CommInterface::spiTask,
        "comm_spi",
        SPI_TASK_STACK,
        this,
        SPI_TASK_PRIO,
        nullptr,
        SPI_TASK_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create comm_spi task");

    // ── Planner layer: decomposes multi-axis blocks into planned segments ───
    ESP_RETURN_ON_ERROR(planner_.init(s_multi_axis_queue, s_flush_queue),
                        TAG, "failed to init motion planner");

    rc = xTaskCreatePinnedToCore(
        &CommInterface::multiAxisExecutorTask,
        "multi_exec",
        MULTI_EXEC_STACK,
        this,
        MULTI_EXEC_PRIO,
        nullptr,
        MULTI_EXEC_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create multi_exec task");

    // Delegate endstop ISR registration to the lateral axis driver (axis 1).
    // StepperDriver owns endstop_active_ and executor_task_, so the ISR
    // can act without going through CommInterface.
    if (n_motors_ >= 2 && queues_[1] != nullptr) {
        ESP_RETURN_ON_ERROR(
            queues_[1]->driver().initEndstopIsr(pins_.home_pin_no, pins_.home_pin_nc),
            TAG, "initEndstopIsr failed");
    }

    ESP_LOGI(TAG, "SPI slave ready  MOSI=%d MISO=%d SCLK=%d CS=%d  frame=%uB",
             (int)pins_.mosi, (int)pins_.miso, (int)pins_.sclk, (int)pins_.cs,
             (unsigned)SPI_FRAME_SIZE);
    return ESP_OK;
}

void CommInterface::buildStatusFrame(uint8_t* out_frame) const
{
    spi_message_zero_frame(out_frame);

    auto* header = reinterpret_cast<SpiMessageHeader*>(out_frame);
    auto* payload = reinterpret_cast<StatusPayload*>(out_frame + sizeof(SpiMessageHeader));

    spi_message_init_header(*header,
                            SpiMessageType::STATUS,
                            last_rx_sequence_,
                            sizeof(StatusPayload),
                            0);

    payload->uptime_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            payload->queue_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->available());
            payload->ring_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->driver().ringFreeSlots());
            payload->underrun_count[axis] = queues_[axis]->driver().getUnderrunCount();
            if (queues_[axis]->driver().isStreaming()) {
                payload->running_mask |= static_cast<uint8_t>(1U << axis);
            }
            if (queues_[axis]->driver().isEnabled()) {
                payload->enabled_mask |= static_cast<uint8_t>(1U << axis);
            }
        } else {
            payload->queue_free_slots[axis] = 0;
            payload->ring_free_slots[axis]  = 0;
            payload->underrun_count[axis]   = 0;
        }
    }
    payload->last_rx_sequence = last_rx_sequence_;
    payload->last_rx_type     = last_rx_type_;
    payload->last_result      = last_result_;
    payload->protocol_version = SPI_MSG_VERSION;
    payload->lateral_endstop_state = readLateralEndstopState();

    payload->endstop_armed_mask = 0;
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            if (queues_[axis]->driver().isEndstopArmed()) {
                payload->endstop_armed_mask |= static_cast<uint8_t>(1U << axis);
            }
        }
    }

    // Atomic load — lock-free cross-core read (written by Core 1 executor).
    payload->last_executed_sequence = last_executed_sequence_.load(std::memory_order_acquire);

    // Planner lookahead pressure: how many slots are free in segment_queue_.
    const uint32_t pqf = planner_.segmentQueueFree();
    payload->planner_queue_free = static_cast<uint8_t>(pqf < 255u ? pqf : 255u);

    spi_message_finalize(out_frame);
}

esp_err_t CommInterface::handleEnableAxis(const EnableAxisPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (payload.enable) {
        queues_[payload.axis_id]->driver().enable();
    } else {
        queues_[payload.axis_id]->driver().disable();
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleEmergencyStop(const EmergencyStopPayload& payload)
{
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < n_motors_; ++axis) {
            if (queues_[axis] != nullptr) {
                queues_[axis]->driver().emergencyStop();
            }
        }
        return ESP_OK;
    }

    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    queues_[payload.axis_id]->driver().emergencyStop();
    return ESP_OK;
}

esp_err_t CommInterface::handleStopAxis(const EmergencyStopPayload& payload)
{
    // gracefulStop() marks the driver as stopped but does NOT flush the ring
    // buffer, so the motor decelerates naturally through any remaining queued
    // steps rather than cutting out instantly.
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < n_motors_; ++axis) {
            if (queues_[axis] != nullptr) {
                queues_[axis]->gracefulStop();
            }
        }
        return ESP_OK;
    }

    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    queues_[payload.axis_id]->gracefulStop();
    return ESP_OK;
}

esp_err_t CommInterface::handleDisableAll()
{
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().disable();
        }
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleResetStats()
{
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().resetUnderrunCount();
        }
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleEnableEndstop(const EnableEndstopPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    StepperDriver& drv = queues_[payload.axis_id]->driver();
    if (payload.arm) {
        drv.armEndstop();
        ESP_LOGI(TAG, "endstop armed on axis %u", payload.axis_id);
    } else {
        drv.disarmEndstop();
        ESP_LOGI(TAG, "endstop disarmed on axis %u", payload.axis_id);
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleStepBlock(const StepBlockPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isLateralMovementAllowed(payload.axis_id)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (payload.step_count > STEP_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    motion_block_t block {};
    block.kind = MOTION_BLOCK_KIND_STEP;
    block.payload.step.count = payload.step_count;
    for (uint32_t i = 0; i < block.payload.step.count; ++i) {
        block.payload.step.steps[i].interval_ticks = payload.entries[i].interval_ticks;
        block.payload.step.steps[i].direction = (payload.entries[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
    }

    return queues_[payload.axis_id]->enqueueMotionBlock(block, 0);
}

esp_err_t CommInterface::handleSegmentBlock(const SegmentBlockPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isLateralMovementAllowed(payload.axis_id)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (payload.segment_count > SEGMENT_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    motion_block_t block {};
    block.kind = MOTION_BLOCK_KIND_SEGMENT;
    block.payload.segment.count = payload.segment_count;
    for (uint32_t i = 0; i < block.payload.segment.count; ++i) {
        block.payload.segment.segments[i].step_count = payload.segments[i].step_count;
        block.payload.segment.segments[i].start_ticks = payload.segments[i].start_ticks;
        block.payload.segment.segments[i].add_ticks = payload.segments[i].add_ticks;
        block.payload.segment.segments[i].direction =
            (payload.segments[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
        block.payload.segment.segments[i].reserved = 0;
    }

    return queues_[payload.axis_id]->enqueueMotionBlock(block, 0);
}

esp_err_t CommInterface::handleMultiAxisSegmentBlock(const uint8_t* payload,
                                                     uint16_t payload_length)
{
    /*
     * Wire layout for MULTI_AXIS_SEGMENT_BLOCK payload:
     *
     *   MultiAxisSegmentBlockHeader   (4 bytes)
     *   uint8_t  axis_ids[axis_count] (axis_count bytes)
     *   For each segment:
     *     uint16_t motion_sequence    (2 bytes)
     *     uint16_t duration_us        (2 bytes)
     *     uint16_t direction_mask     (2 bytes)
     *     uint16_t step_counts[axis_count] (2 * axis_count bytes)
     *
     * Total minimum: 4 + axis_count + segment_count * (6 + 2*axis_count)
     */
    if (payload_length < sizeof(MultiAxisSegmentBlockHeader)) {
        return ESP_ERR_INVALID_SIZE;
    }

    MultiAxisSegmentBlockHeader hdr_val;
    memcpy(&hdr_val, payload, sizeof(hdr_val));
    const uint8_t axis_count     = hdr_val.axis_count;
    const uint8_t segment_count  = hdr_val.segment_count;

    if (axis_count == 0 || axis_count > MULTI_AXIS_MAX_AXES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (segment_count == 0 || segment_count > MULTI_AXIS_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Validate total payload length before reading any further.
    const size_t expected_length =
        sizeof(MultiAxisSegmentBlockHeader)
        + static_cast<size_t>(axis_count)
        + static_cast<size_t>(segment_count) * (6u + 2u * axis_count);
    if (payload_length < static_cast<uint16_t>(expected_length)) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Deserialise.
    multi_axis_block_t block {};
    block.axis_count     = axis_count;
    block.segment_count  = segment_count;

    const uint8_t* cursor = payload + sizeof(MultiAxisSegmentBlockHeader);

    // axis_ids
    for (uint8_t a = 0; a < axis_count; ++a) {
        block.axis_ids[a] = cursor[a];
    }
    cursor += axis_count;

    // segments
    for (uint8_t s = 0; s < segment_count; ++s) {
        uint16_t motion_seq, duration_us, dir_mask;
        memcpy(&motion_seq,  cursor,     2);
        memcpy(&duration_us, cursor + 2, 2);
        memcpy(&dir_mask,    cursor + 4, 2);
        cursor += 6;

        block.segments[s].motion_sequence = motion_seq;
        block.segments[s].duration_us     = duration_us;
        block.segments[s].direction_mask  = dir_mask;

        for (uint8_t a = 0; a < axis_count; ++a) {
            uint16_t steps;
            memcpy(&steps, cursor, 2);
            block.segments[s].step_counts[a] = steps;
            cursor += 2;
        }
    }

    // Non-blocking enqueue: return QUEUE_FULL immediately if full.
    if (xQueueSend(s_multi_axis_queue, &block, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT; // maps to QUEUE_FULL result code
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleFlush(const FlushPayload& flush_payload)
{
    /*
     * Post a flush_request_t to the flush queue.  The executor task
     * watches this queue and applies the flush before processing the next
     * segment.  Using a queue (instead of an atomic variable) ensures that
     * a flush posted just before new segments arrive is always processed in
     * the correct order.
     */
    flush_request_t req { .flush_sequence = flush_payload.flush_sequence };
    if (xQueueSend(s_flush_queue, &req, 0) != pdTRUE) {
        // Flush queue full — this should never happen in normal operation.
        ESP_LOGW(TAG, "flush queue full — flush_seq=%u dropped",
                 (unsigned)flush_payload.flush_sequence);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void CommInterface::notifySegmentExecuted(uint16_t motion_seq)
{
    /*
     * Called by the executor task (Core 1) after each multi-axis segment
     * completes.  Updates last_executed_sequence_ with an atomic store
     * so the SPI task (Core 0) can safely read it in buildStatusFrame().
     *
     * Only advances the sequence — never moves it backward.  This handles
     * the 16-bit wrap-around case correctly because we only call this in
     * strict execution order.
     */
    uint16_t current = last_executed_sequence_.load(std::memory_order_relaxed);
    if (static_cast<int16_t>(motion_seq - current) > 0) {
        last_executed_sequence_.store(motion_seq, std::memory_order_release);
    }
}

uint8_t CommInterface::readLateralEndstopState() const
{
    if (pins_.home_pin_no == GPIO_NUM_NC || pins_.home_pin_nc == GPIO_NUM_NC) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }

    const int no_state = gpio_get_level(pins_.home_pin_no);
    const int nc_state = gpio_get_level(pins_.home_pin_nc);

    if (no_state == nc_state) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }
    if (no_state == 0 && nc_state == 1) {
        return static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED);
    }
    return static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

bool CommInterface::isLateralMovementAllowed(uint8_t axis_id) const
{
    if (axis_id != 1) {
        return true;
    }
    return readLateralEndstopState() == static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

esp_err_t CommInterface::handleFrame(const SpiMessageHeader& header, const uint8_t* payload)
{
    switch (static_cast<SpiMessageType>(header.msg_type)) {
    case SpiMessageType::NOP:
    case SpiMessageType::GET_STATUS:
    case SpiMessageType::PING:
        return ESP_OK;

    case SpiMessageType::ENABLE_AXIS: {
        if (header.payload_length != sizeof(EnableAxisPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EnableAxisPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEnableAxis(p);
    }

    case SpiMessageType::ESTOP: {
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EmergencyStopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEmergencyStop(p);
    }

    case SpiMessageType::STOP_AXIS: {
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EmergencyStopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleStopAxis(p);
    }

    case SpiMessageType::DISABLE_ALL:
        if (header.payload_length != 0) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleDisableAll();

    case SpiMessageType::RESET_STATS:
        if (header.payload_length != 0) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleResetStats();

    case SpiMessageType::STEP_BLOCK: {
        if (header.payload_length != sizeof(StepBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        StepBlockPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleStepBlock(p);
    }

    case SpiMessageType::SEGMENT_BLOCK: {
        if (header.payload_length != sizeof(SegmentBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        SegmentBlockPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleSegmentBlock(p);
    }

    case SpiMessageType::MULTI_AXIS_SEGMENT_BLOCK:
        // Variable-length payload — pass raw buffer + length.
        return handleMultiAxisSegmentBlock(payload, header.payload_length);

    case SpiMessageType::FLUSH: {
        if (header.payload_length != sizeof(FlushPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        FlushPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleFlush(p);
    }

    case SpiMessageType::ENABLE_ENDSTOP: {
        if (header.payload_length != sizeof(EnableEndstopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EnableEndstopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEnableEndstop(p);
    }

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

void CommInterface::spiTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);
    ESP_LOGI(TAG, "SPI task started on core %d", xPortGetCoreID());

    // Double-buffer ping-pong: while DMA transmits tx_ping, we build the next
    // status frame into tx_pong.  This reduces pipeline lag by one full SPI
    // round-trip — the status sent in transaction N reflects state AFTER
    // transaction N-1 was handled, not state from before the previous transmit.
    uint8_t* tx_ping = s_tx_frame_a;
    uint8_t* tx_pong = s_tx_frame_b;

    // Pre-build the very first frame before entering the loop so the initial
    // transaction has valid (zero-but-structured) content.
    self->buildStatusFrame(tx_ping);

    for (;;) {
        // ── Transmit the previously-built status frame ────────────────────────
        spi_slave_transaction_t txn = {};
        txn.length = SPI_FRAME_SIZE * 8;
        txn.tx_buffer = tx_ping;
        txn.rx_buffer = s_rx_frame;

        esp_err_t err = spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY);
        // DMA is done with tx_ping — safe to reuse as the next write buffer.

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            // Rebuild into the same ping buffer and retry.
            self->buildStatusFrame(tx_ping);
            continue;
        }

        // ── Parse and handle incoming frame ───────────────────────────────────
        SpiMessageHeader header {};
        memcpy(&header, s_rx_frame, sizeof(SpiMessageHeader));

        if (header.magic != SPI_MSG_MAGIC) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_MAGIC);
        } else if (header.version != SPI_MSG_VERSION) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_VERSION);
        } else if (header.payload_length > SPI_MAX_PAYLOAD_SIZE) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
        } else if (!spi_message_validate(s_rx_frame, header)) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_CRC);
        } else {
            self->last_rx_sequence_ = header.sequence;
            self->last_rx_type_ = header.msg_type;

            const uint8_t* payload = s_rx_frame + sizeof(SpiMessageHeader);
            err = self->handleFrame(header, payload);
            if (err == ESP_OK) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::OK);
            } else if (err == ESP_ERR_TIMEOUT) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::QUEUE_FULL);
            } else if (err == ESP_ERR_INVALID_ARG) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_AXIS);
            } else if (err == ESP_ERR_INVALID_SIZE) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
            } else if (err == ESP_ERR_NOT_SUPPORTED) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::UNKNOWN_TYPE);
            } else if (err == ESP_ERR_INVALID_STATE) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::ENDSTOP_BLOCKED);
            } else {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::INTERNAL_ERROR);
                ESP_LOGW(TAG, "message 0x%02X failed: %s",
                         header.msg_type, esp_err_to_name(err));
            }
        }

        // ── Build next status frame into the now-idle buffer ──────────────────
        // We write into tx_pong (the buffer NOT currently wired to DMA).
        // Reflects state AFTER handling the frame we just received.
        self->buildStatusFrame(tx_pong);

        // Swap: tx_pong becomes the next transmit buffer.
        uint8_t* tmp = tx_ping;
        tx_ping = tx_pong;
        tx_pong = tmp;
    }
}

// ---------------------------------------------------------------------------
// multiAxisExecutorTask()  — Core 1, priority 20 — STATE MACHINE
// ---------------------------------------------------------------------------
//
// Refactored from a monolithic nested-loop drain pattern into a bounded
// state machine.  Each state transition does bounded work (≤ EXEC_TIME_BUDGET_US
// or ≤ EXEC_BATCH_LIMIT segments) then yields to the scheduler.
//
//   IDLE ──► FETCH ──► DRAIN ──► RUN ──► IDLE
//              │                          ▲
//              ├── (flush sentinel) ──► FLUSH ──┘
//              └── (error/underrun) ──► RECOVERY ──┘
//
// Watchdog safety: NO state executes for more than ~300 µs without exiting
// to the for(;;) top-level loop which naturally yields via xQueueReceive
// or explicit vTaskDelay(1).

void CommInterface::multiAxisExecutorTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);

    ESP_LOGI(TAG, "multi-axis executor (state machine) started on core %d",
             xPortGetCoreID());

    // Register this task for ISR ring-space wakeups on all drivers.
    {
        TaskHandle_t my_handle = xTaskGetCurrentTaskHandle();
        for (uint8_t a = 0; a < self->n_motors_; ++a) {
            if (self->queues_[a] != nullptr) {
                self->queues_[a]->driver().setExecutorTask(my_handle);
            }
        }
    }

    ESP_LOGI(TAG, "multi_exec stack high watermark at start: %u bytes free",
             (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));

    // ── Segment queue handle from the planner ─────────────────────────────
    QueueHandle_t seg_queue = self->planner_.segmentQueue();

    // ── Deferred notification ring ────────────────────────────────────────
    static constexpr int DEFER_DEPTH = 256;
    static int64_t  defer_fire_us[DEFER_DEPTH];
    static uint32_t defer_seqs[DEFER_DEPTH];
    int      defer_head = 0;
    int      defer_tail = 0;

    // ── Active axis tracking (persists across iterations for recovery) ────
    uint8_t active_axis_ids[MULTI_AXIS_MAX_AXES] = {};
    uint8_t active_axis_count = 0;

    // ── Batch buffer for FETCH state ──────────────────────────────────────
    planned_segment_t batch[EXEC_BATCH_LIMIT];
    uint32_t batch_count = 0;
    uint32_t batch_index = 0;

    // ── State machine ─────────────────────────────────────────────────────
    ExecState state = ExecState::IDLE;

    // Lambda: fire all due deferred notifications.
    auto fireDeferred = [&]() {
        const int64_t now = esp_timer_get_time();
        while (defer_head != defer_tail) {
            const int idx = defer_head & (DEFER_DEPTH - 1);
            if (now >= defer_fire_us[idx]) {
                self->notifySegmentExecuted(
                    static_cast<uint16_t>(defer_seqs[idx]));
                ++defer_head;
            } else {
                break;
            }
        }
    };

    // Lambda: kickStart all active axes.
    auto kickStartActiveAxes = [&]() {
        for (uint8_t a = 0; a < active_axis_count; ++a) {
            const uint8_t axis_id = active_axis_ids[a];
            if (axis_id < self->n_motors_ && self->queues_[axis_id] != nullptr) {
                self->queues_[axis_id]->kickStart();
            }
        }
    };

    // ── Main loop ─────────────────────────────────────────────────────────
    for (;;) {
        static uint32_t wm_iter = 0;
        if (++wm_iter % 2000 == 0) {
            ESP_LOGD(TAG, "multi_exec stack watermark: %u bytes free",
                     (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));
        }

        // Always fire due deferred notifications at top of loop.
        fireDeferred();

        switch (state) {

        // ══════════════════════════════════════════════════════════════════
        // IDLE: wait for segments from the planner (blocking with timeout)
        // ══════════════════════════════════════════════════════════════════
        case ExecState::IDLE: {
            // Compute wait timeout: wake early if a deferred notification
            // is about to fire.  Hard-cap at 1 ms so ISR ring-space
            // notifications (which wake ulTaskNotifyTake, not xQueueReceive)
            // don't cause >1 ms stalls.
            TickType_t wait_ticks;
            if (defer_head != defer_tail) {
                const int idx = defer_head & (DEFER_DEPTH - 1);
                const int64_t remaining_us =
                    defer_fire_us[idx] - esp_timer_get_time();
                if (remaining_us <= 500) {
                    wait_ticks = 0;
                } else {
                    wait_ticks = 1;
                }
            } else {
                wait_ticks = pdMS_TO_TICKS(1);
            }

            planned_segment_t seg;
            if (xQueueReceive(seg_queue, &seg, wait_ticks) == pdTRUE) {
                batch[0]    = seg;
                batch_count = 1;
                batch_index = 0;
                state = ExecState::FETCH;
            } else {
                // Timeout — kick-start any stalled axes (RMT underrun
                // while we were blocked on xQueueReceive).
                kickStartActiveAxes();
            }
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // FETCH: non-blocking batch fill up to EXEC_BATCH_LIMIT
        // ══════════════════════════════════════════════════════════════════
        case ExecState::FETCH: {
            // Fill remaining batch slots non-blocking.
            while (batch_count < EXEC_BATCH_LIMIT) {
                planned_segment_t seg;
                if (xQueueReceive(seg_queue, &seg, 0) != pdTRUE) break;
                batch[batch_count++] = seg;
            }
            batch_index = 0;
            state = ExecState::DRAIN;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // DRAIN: process batch segments — write steps to RMT ring
        // ══════════════════════════════════════════════════════════════════
        case ExecState::DRAIN: {
            const int64_t drain_start = esp_timer_get_time();

            while (batch_index < batch_count) {
                // ── Pre-check: yield if time budget will be exceeded ───────────
                // This prevents accumulating too much CPU time before yielding.
                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    kickStartActiveAxes();
                    taskYIELD();
                    // Restart from FETCH to get fresh batch and reset timer.
                    state = ExecState::FETCH;
                    goto exit_drain;
                }

                planned_segment_t& seg = batch[batch_index];

                // ── Flush sentinel ────────────────────────────────────────
                if (seg.is_flush) {
                    state = ExecState::FLUSH;
                    goto exit_drain;  // break out of DRAIN, handle in FLUSH
                }

                // ── Update active axis list ───────────────────────────────
                active_axis_count = seg.axis_count < MULTI_AXIS_MAX_AXES
                    ? seg.axis_count : MULTI_AXIS_MAX_AXES;
                for (uint8_t a = 0; a < active_axis_count; ++a) {
                    active_axis_ids[a] = seg.axis_ids[a];
                }

                // ── Endstop check (per-segment, real-time) ───────────────
                bool endstop_hit = false;
                for (uint8_t a = 0; a < seg.axis_count && !endstop_hit; ++a) {
                    const uint8_t eid = seg.axis_ids[a];
                    if (eid >= self->n_motors_ ||
                        self->queues_[eid] == nullptr) continue;
                    if (self->queues_[eid]->driver().isEndstopActive()) {
                        // Drain remaining batch, e-stop, notify host.
                        self->queues_[eid]->driver().emergencyStop();
                        self->notifySegmentExecuted(seg.motion_sequence);
                        ESP_LOGW(TAG, "endstop on axis %u at seq=%u",
                                 eid, seg.motion_sequence);
                        endstop_hit = true;
                    }
                }
                if (endstop_hit) {
                    state = ExecState::RECOVERY;
                    goto exit_drain;
                }

                // ── Lateral endstop gate (read once per segment) ──────────
                const uint8_t lateral_state = self->readLateralEndstopState();
                const bool lateral_blocked =
                    lateral_state !=
                    static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);

                // ── Write steps to ring buffer (no RMT start) ─────────────
                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id >= self->n_motors_ ||
                        self->queues_[axis_id] == nullptr) continue;
                    if (seg.axes[a].step_count == 0) continue;
                    if (axis_id == 1 && lateral_blocked) {
                        ESP_LOGD(TAG, "axis1 blocked, skip %u steps",
                                 seg.axes[a].step_count);
                        continue;
                    }

                    esp_err_t err =
                        self->queues_[axis_id]->executeConstantRateBlock(
                            seg.axes[a].direction,
                            seg.axes[a].step_count,
                            seg.duration_us);

                    if (err == ESP_ERR_INVALID_STATE) {
                        ESP_LOGW(TAG, "axis %u endstop mid-seg seq=%u",
                                 axis_id, seg.motion_sequence);
                        self->queues_[axis_id]->driver().emergencyStop();
                        self->notifySegmentExecuted(seg.motion_sequence);
                        state = ExecState::RECOVERY;
                        goto exit_drain;
                    } else if (err != ESP_OK) {
                        ESP_LOGW(TAG, "axis %u seg %u: %s",
                                 axis_id, seg.motion_sequence,
                                 esp_err_to_name(err));
                    }
                }

                // ── Schedule deferred notification ────────────────────────
                if ((defer_tail - defer_head) < DEFER_DEPTH) {
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = seg.scheduled_time_us
                                         + static_cast<int64_t>(seg.duration_us);
                    defer_seqs[idx]    = seg.motion_sequence;
                    ++defer_tail;
                } else {
                    // Ring full: evict the oldest (earliest scheduled) entry,
                    // notify it now (it is already overdue), then enqueue the
                    // current segment normally. This preserves ordering and
                    // avoids signalling completion before steps reach the ring.
                    const int evict_idx = defer_head & (DEFER_DEPTH - 1);
                    const uint32_t evicted_seq = defer_seqs[evict_idx];
                    self->notifySegmentExecuted(
                        static_cast<uint16_t>(evicted_seq));
                    ++defer_head;
                    // Enqueue current segment.
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = seg.scheduled_time_us
                                         + static_cast<int64_t>(seg.duration_us);
                    defer_seqs[idx]    = seg.motion_sequence;
                    ++defer_tail;
                    ESP_LOGW(TAG, "defer ring full: evicted seq=%u to make room for seq=%u",
                             (unsigned)evicted_seq,
                             (unsigned)seg.motion_sequence);
                }

                // Restart RMT immediately if it stopped mid-batch due to ring drain.
                // Do not wait for ExecState::RUN — the ring may fill with unconsumed
                // steps causing pushBlock() to deadlock on ulTaskNotifyTake.
                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id >= self->n_motors_ ||
                        self->queues_[axis_id] == nullptr) continue;
                    if (!self->queues_[axis_id]->driver().isStreaming()) {
                        self->queues_[axis_id]->kickStart();
                    }
                }

                ++batch_index;

                // ── Time budget check (watchdog safety) ───────────────────
                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    // Budget exhausted — transition to RUN to kickStart,
                    // then yield before processing remaining segments.
                    kickStartActiveAxes();
                    taskYIELD();
                    // Continue draining after yield (reset budget).
                    break;  // will re-enter DRAIN on next iteration
                }
            }

            // All segments in batch processed — transition to RUN.
            if (batch_index >= batch_count) {
                state = ExecState::RUN;
            }
            // else: budget break, stay in DRAIN for remaining segments.
            break;

        exit_drain:
            break;  // state already set by the goto target
        }

        // ══════════════════════════════════════════════════════════════════
        // RUN: kickStart RMT on all active axes, return to IDLE
        // ══════════════════════════════════════════════════════════════════
        case ExecState::RUN: {
            kickStartActiveAxes();
            fireDeferred();
            state = ExecState::IDLE;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // FLUSH: reset pipeline state, notify host
        // ══════════════════════════════════════════════════════════════════
        case ExecState::FLUSH: {
            // The flush sentinel is at batch[batch_index].
            const planned_segment_t& flush_seg = batch[batch_index];

            // Reset deferred notification ring.
            defer_head = defer_tail = 0;

            // Notify host with flush sequence.
            self->notifySegmentExecuted(flush_seg.flush_sequence);

            ESP_LOGI(TAG, "executor flush at seq=%u",
                     (unsigned)flush_seg.flush_sequence);

            // Clear batch and return to idle.
            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // RECOVERY: handle endstop / error, drain remaining, return to IDLE
        // ══════════════════════════════════════════════════════════════════
        case ExecState::RECOVERY: {
            // Drain any remaining segments in the planner's output queue
            // (bounded drain to avoid spending too long here).
            planned_segment_t discard;
            uint32_t drained = 0;
            while (drained < SEGMENT_QUEUE_DEPTH &&
                   xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
                ++drained;
            }

            // Reset deferred notifications.
            defer_head = defer_tail = 0;

            ESP_LOGW(TAG, "recovery: drained %lu remaining segments",
                     (unsigned long)drained);

            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }

        } // switch(state)

        // ── Watchdog safety: yield if idle, sleep if very idle ───────────────
        // If we fetched zero segments in FETCH, sleep to let IDLE1 run.
        // Otherwise, yield to respect other tasks without 10ms stalls.
        if (state == ExecState::IDLE && batch_count == 0) {
            vTaskDelay(1);  // Very idle — sleep and let watchdog reset
        } else {
            taskYIELD();    // Still have work — yield but stay ready
        }
    } // for(;;)
}
```

### src/esp32/src/main.cpp
```cpp
/**
 * @file main.cpp
 * @brief ESP32 Klipper-style stepper executor — entry point.
 *
 * Architecture overview
 * ─────────────────────
 *   Core 0  (APP CPU)
 *     • comm_spi task (pri 10) : SPI slave message parser → StepperQueue
 *
 *   Core 1  (PRO CPU)
 *     • stepper_0     (pri 24) : executor for motor A  (RMT channel 0)
 *     • stepper_1     (pri 24) : executor for motor B  (RMT channel 1)
 *
 *   RMT hardware
 *     • Channel 0 → STEP_A_GPIO  (2 MHz, 64-symbol block, trans_queue=1)
 *     • Channel 1 → STEP_B_GPIO  (2 MHz, 64-symbol block, trans_queue=1)
 *     • One balanced 50/50 HIGH/LOW RMT symbol per commanded step
 *
 * Pin assignments
 * ───────────────
 *   Motor A (Bobbin / axis 0)  : STEP=GPIO26  DIR=GPIO27  EN=GPIO14
 *   Motor B (Lateral / axis 1) : STEP=GPIO32  DIR=GPIO33  EN=GPIO25
 *
 *   SPI host link              : MOSI=GPIO23  MISO=GPIO19
 *                                SCLK=GPIO18  CS=GPIO5
 *
 * The Raspberry Pi demo lives in `src/rpi/` and streams fixed-size SPI
 * message frames to this firmware.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "step_types.h"
#include "stepper_driver.h"
#include "stepper_queue.h"
#include "comm_interface.h"

static const char* TAG = "main";

// ---------------------------------------------------------------------------
// Pin definitions — adjust to your board wiring
// ---------------------------------------------------------------------------

// Motor A — Bobbin axis
static constexpr gpio_num_t STEP_A = GPIO_NUM_26;
static constexpr gpio_num_t DIR_A  = GPIO_NUM_27;
static constexpr gpio_num_t EN_A   = GPIO_NUM_14;

// Motor B — Lateral axis
static constexpr gpio_num_t STEP_B = GPIO_NUM_32;
static constexpr gpio_num_t DIR_B  = GPIO_NUM_33;
static constexpr gpio_num_t EN_B   = GPIO_NUM_25;

// SPI host link
static constexpr gpio_num_t SPI_MOSI = GPIO_NUM_23;
static constexpr gpio_num_t SPI_MISO = GPIO_NUM_19;
static constexpr gpio_num_t SPI_SCLK = GPIO_NUM_18;
static constexpr gpio_num_t SPI_CS   = GPIO_NUM_5;

// Lateral home sensor (2-contact)
static constexpr gpio_num_t HOME_NO = GPIO_NUM_21;
static constexpr gpio_num_t HOME_NC = GPIO_NUM_22;

// ---------------------------------------------------------------------------
// Global instances — static storage, constructed once
// ---------------------------------------------------------------------------

static StepperDriver motor_a(STEP_A, DIR_A, EN_A, 0);
static StepperDriver motor_b(STEP_B, DIR_B, EN_B, 1);

static StepperQueue  queue_a(motor_a, 0);
static StepperQueue  queue_b(motor_b, 1);

static StepperQueue* queues[2] = {&queue_a, &queue_b};
static CommInterface comm(queues, 2);

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "PickupWinder — Klipper-style RMT stepper executor");
    ESP_LOGI(TAG, "RMT resolution : %lu Hz  (%lu ns/tick)",
             (unsigned long)RMT_STEP_RESOLUTION_HZ,
             (unsigned long)(1000000000UL / RMT_STEP_RESOLUTION_HZ));
    ESP_LOGI(TAG, "Max step rate  : ~166 kHz  (interval_min = %u ticks = %lu µs)",
             RMT_STEP_MIN_TICKS,
             (unsigned long)(RMT_STEP_MIN_TICKS * 1000000UL / RMT_STEP_RESOLUTION_HZ));
    ESP_LOGI(TAG, "Block size     : %d steps   Queue depth : %d blocks",
             STEP_BLOCK_SIZE, STEPPER_QUEUE_DEPTH);

    // ── 1. Initialise RMT drivers ───────────────────────────────────────────
    ESP_ERROR_CHECK(motor_a.init());
    ESP_ERROR_CHECK(motor_b.init());

    // ── 2. Enable motor drivers ─────────────────────────────────────────────
    //motor_a.enable();
    //motor_b.enable();

    // ── 3. Launch executor tasks (Core 1, priority 24) ──────────────────────
    ESP_ERROR_CHECK(queue_a.init());
    ESP_ERROR_CHECK(queue_b.init());

    // ── 4. Start SPI communication interface (Core 0, priority 10) ────────
    ESP_ERROR_CHECK(comm.init({SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_CS, HOME_NO, HOME_NC}));

    // app_main may return — FreeRTOS scheduler continues running the tasks.
    ESP_LOGI(TAG, "Scheduler running — app_main exiting.");
}
```

### src/rpi/core/config.py
```python
from dataclasses import dataclass
from typing import Optional

@dataclass
class AppConfiguration:
    """Configuration parameters for the PickupWinder host application."""


    rpc_socket_path: str = "/tmp/pickup_winder_rpc.sock"
    spi_device: str = "/dev/spidev0.0"
    spi_speed_hz: int = 1_000_000

    spindle_axis_id: int = 0
    spindle_steps_per_revolution: int = 200
    spindle_microstepping: int = 32
    spindle_invert_direction: bool = False
    spindle_max_speed_rpm: int = 1500
    # Unit: RPM/s (revolutions per minute gained per second).
    spindle_max_acceleration_rpm: Optional[float] = 10
    # Unit: RPM/s (revolutions per minute lost per second).
    spindle_max_deceleration_rpm: Optional[float] = None

    lateral_axis_id: int = 1
    lateral_steps_per_revolution: int = 200
    lateral_microstepping: int = 32
    lateral_invert_direction: bool = False
    lateral_max_rpm: int = 1000
    # Unit: mm/s² on traverse axis.
    lateral_max_acceleration_mm_per_s2: Optional[float] = None
    # Unit: mm/s² on traverse axis.
    lateral_max_deceleration_mm_per_s2: Optional[float] = None
    
    # Leadscrew/traverse pitch in mm per revolution for the lateral axis.
    # Used to compute steps/mm: steps_per_rev * microstepping / pitch_mm
    lateral_traverse_pitch_mm: float = 1.0
    # Optional explicit override for lateral steps-per-mm. If set, this
    # value takes precedence over the computed value.
    lateral_steps_per_mm_override: Optional[float] = None

    @property
    def lateral_steps_per_mm(self) -> float:
        """Return lateral axis steps per millimetre.

        Computed as: (steps_per_revolution * microstepping) / traverse_pitch_mm.
        If `lateral_steps_per_mm_override` is provided, it is returned instead.
        """
        if self.lateral_steps_per_mm_override is not None:
            return float(self.lateral_steps_per_mm_override)
        return (self.lateral_steps_per_revolution * self.lateral_microstepping) / float(self.lateral_traverse_pitch_mm)

    @property
    def spindle_max_acceleration_steps_per_s2(self) -> float:
        """Compute spindle acceleration in steps/s^2.

        Uses `spindle_max_acceleration_rpm` (RPM/s) if provided.
        Otherwise returns a safe default.
        """
        steps_per_rev = self.spindle_steps_per_revolution * self.spindle_microstepping
        if self.spindle_max_acceleration_rpm is not None:
            return (self.spindle_max_acceleration_rpm / 60.0) * steps_per_rev
        return 100_000.0

    @property
    def spindle_max_deceleration_steps_per_s2(self) -> float:
        """Compute spindle deceleration in steps/s^2.

        Uses `spindle_max_deceleration_rpm` (RPM/s) if provided. Otherwise falls back
        to the configured spindle acceleration limit.
        """
        steps_per_rev = self.spindle_steps_per_revolution * self.spindle_microstepping
        if self.spindle_max_deceleration_rpm is not None:
            return (self.spindle_max_deceleration_rpm / 60.0) * steps_per_rev
        return self.spindle_max_acceleration_steps_per_s2

    @property
    def lateral_max_acceleration_steps_per_s2(self) -> float:
        """Compute lateral acceleration in steps/s^2.

        Uses `lateral_max_acceleration_mm_per_s2` if provided.
        Otherwise returns a safe default.
        """
        if self.lateral_max_acceleration_mm_per_s2 is not None:
            return float(self.lateral_max_acceleration_mm_per_s2) * self.lateral_steps_per_mm
        return 100_000.0

    @property
    def lateral_max_deceleration_steps_per_s2(self) -> float:
        """Compute lateral deceleration in steps/s^2.

        Uses `lateral_max_deceleration_mm_per_s2` if provided.
        Otherwise falls back to the configured lateral acceleration limit.
        """
        if self.lateral_max_deceleration_mm_per_s2 is not None:
            return float(self.lateral_max_deceleration_mm_per_s2) * self.lateral_steps_per_mm
        return self.lateral_max_acceleration_steps_per_s2
```

