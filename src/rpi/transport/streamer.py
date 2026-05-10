from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Any, Iterator, TYPE_CHECKING
from motion.segment_producer import SegmentProducer

from transport.messages import (
    MULTI_AXIS_SEGMENT_BLOCK_SIZE,
    LATERAL_ENDSTOP_ABSENT,
    LATERAL_ENDSTOP_PRESENT_CLOSED,
    LATERAL_ENDSTOP_PRESENT_OPEN,
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
    PREFILL_MAX_BUFFER_TIME_S = 0.50
    MIN_BUFFER_TIME_S = 0.06
    MAX_BUFFER_TIME_S = 0.25
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

                Thresholds are intentionally conservative for the 10..49 steps/segment
                band because this is where the current 1500 RPM winding workload lands:

                    < 10  steps → 64 segments (low speed, long host/firmware latency ratio)
                    < 50  steps → 48 segments (current winding regime, needs more margin)
                    >= 50 steps → 32 segments (large segments already amortize comm latency)

        The source of truth is the firmware planner queue depth, not the host's
        buffered_time estimate. A deeper lookahead here reduces sensitivity to
        one or two transient SPI retries or short frames.
        """
        if steps_per_segment < 10:
            return 48
        elif steps_per_segment < 50:
            return 32
        else:
            return 32

    def _planner_buffer_deficit(self, status) -> int:
        """Return how many planned segments the firmware is short of target.

        Positive value => host should send more motion immediately.
        Zero => firmware already has the required lookahead depth.
        """
        self._check_planner_pressure(status)
        needed = self.required_lookahead(self._current_steps_per_segment)
        return max(0, needed - self._buffered_segments)

    def __init__(
        self,
        transport: Esp32SpiTransport,
        axis_streams: list[StreamAxisConfig],
        *,
        segment_duration_s: float = 0.004,
        target_buffer_time_s: float = TARGET_BUFFER_TIME_S,
        poll_interval_s: float = 0.001,
        stall_timeout_s: float = 5.0,
        print_every: int = 1,
        log_each_send: bool = False,
        send_log_path: str | None = None,
        keep_enabled_axes: set[int] | None = None,
        initial_segments_dropped: int = 0,
    ):
        self._initialize_streamer_state(
            transport=transport,
            axis_configs=[AxisMotionConfig(axis_id=s.axis_id, ramp=s.ramp) for s in axis_streams],
            axis_ids=[s.axis_id for s in axis_streams],
            segment_duration_s=segment_duration_s,
            target_buffer_time_s=target_buffer_time_s,
            poll_interval_s=poll_interval_s,
            stall_timeout_s=stall_timeout_s,
            print_every=print_every,
            log_each_send=log_each_send,
            send_log_path=send_log_path,
            explicit_target_hz=None,
            keep_enabled_axes=keep_enabled_axes,
            initial_segments_dropped=initial_segments_dropped,
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
        stall_timeout_s: float = 5.0,
        keep_enabled_axes: set[int] | None = None,
        initial_segments_dropped: int = 0,
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
            stall_timeout_s=stall_timeout_s,
            print_every=print_every,
            log_each_send=False,
            send_log_path=None,
            explicit_target_hz=target_hz,
            keep_enabled_axes=keep_enabled_axes,
            initial_segments_dropped=initial_segments_dropped,
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
        stall_timeout_s: float,
        print_every: int,
        log_each_send: bool,
        send_log_path: str | None,
        explicit_target_hz: float | None,
        keep_enabled_axes: set[int] | None,
        initial_segments_dropped: int,
    ) -> None:
        self._transport = transport
        self._poll_interval_s = poll_interval_s
        self._print_every = max(print_every, 1)
        self._log_each_send = log_each_send
        self._send_log_path = send_log_path
        self._send_events: list[dict] = []
        self._stop_requested = False
        self._flush_sequence_requested: int | None = None
        self._flush_floor_sequence: int = -1
        self._endstop_triggered = False
        self._endstop_armed_axes: set[int] = set()
        self._keep_enabled_axes = set(keep_enabled_axes or ())

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
        self._retry_batch: list[MultiAxisSegment] | None = None
        self._pending_segment: MultiAxisSegment | None = None
        self._last_sent_motion_seq = -1
        self._last_sent_transport_seq = -1
        self._planner_under_pressure = False  # True while planner_queue_free < threshold
        self._buffered_segments = 0           # SEGMENT_QUEUE_DEPTH - planner_queue_free
        self._current_steps_per_segment = 0  # updated per-segment; drives required_lookahead()
        self._initial_steps_per_segment = 0  # fixed startup estimate used by prefill
        if explicit_target_hz is not None:
            self._initial_steps_per_segment = max(1, int(round(explicit_target_hz * self._segment_duration_s)))
            self._current_steps_per_segment = self._initial_steps_per_segment
        elif self._axis_configs:
            max_hz = max(
                (config.ramp.target_hz for config in self._axis_configs),
                default=0.0,
            )
            if max_hz > 0.0:
                self._initial_steps_per_segment = max(
                    1,
                    int(round(max_hz * self._segment_duration_s)),
                )
                self._current_steps_per_segment = self._initial_steps_per_segment
        self._prefilling = False              # suppresses pressure gate during initial prefill
        # Premature-completion detection
        self._last_confirmed_sequence: int = -1
        self._last_confirmed_motion_seq: int = -1
        self._premature_notify_count: int = 0
        self._premature_notify_window_start: float = 0.0
        self._last_sequence_advance_time: float = time.time()
        self._last_sequence_advance_value: int = -1
        self._last_desync_log_ts: float = 0.0
        self._stall_timeout_s: float = max(1.0, float(stall_timeout_s))
        self._last_underrun_count: tuple[int, int, int, int] | None = None
        self._last_segments_dropped: int = max(0, int(initial_segments_dropped))
        self._last_logged_segments_dropped: int | None = self._last_segments_dropped
        self._homing_mode = False

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

                    < 10  steps/segment → 16 segments/cycle
                    < 50  steps/segment →  8 segments/cycle
                    >= 50 steps/segment →  8 segments/cycle
        """
        if self._current_steps_per_segment < 10:
                        return 16
        elif self._current_steps_per_segment < 50:
                        return 8
        else:
                        return 8

    def _max_inflight_segments(self) -> int:
        """Speed-dependent in-flight segment cap.

        Must be >= required_lookahead() so the planner pressure gate can
        actually be reached before the inflight cap blocks sending.
        At high speed required_lookahead=32, so cap must be > 32.
        """
        if self._current_steps_per_segment < 10:
            return 96
        elif self._current_steps_per_segment < 50:
            return 48
        else:
            return 128

    def _timestamp(self) -> str:
        now = time.time()
        seconds = int(now)
        milliseconds = int((now - seconds) * 1000)
        return time.strftime(f"%H:%M:%S.{milliseconds:03d}", time.localtime(now))

    def _safe_buffer_time_s(self, requested_time_s: float, max_hz: float) -> float:
        # Clamp to [MIN_BUFFER_TIME_S, MAX_BUFFER_TIME_S].
        # The ring-capacity cap was removed: at high step rates the ring holds
        # only ~25ms but the planner segment_queue holds 512ms, so capping by
        # ring capacity forced the pipeline to 60ms — too small to sustain
        # required_lookahead=32 against SPI failure bursts.
        return max(self.MIN_BUFFER_TIME_S, min(self.MAX_BUFFER_TIME_S, requested_time_s))

    def set_generator(self, generator: SegmentProducer | Iterator[MultiAxisSegment]) -> None:
        """Override the segment generator for this streamer.

        Call before stream_all() when the segments are produced externally
        (e.g. by a WoundMove or RampMove).
        """
        self._generator = generator
        self._generator_finished = False
        self._retry_batch = None

    def _sync_with_firmware_status(self) -> None:
        """Synchronize stream state with the ESP32's last executed sequence.

        This is the only place where the stall advance tracker is seeded from
        the firmware's current state.  Subsequent calls to
        _update_confirmed_motion_sequence() deliberately do NOT touch these
        fields so that the stall timer is not reset on every poll iteration.
        """
        try:
            status = self._transport.get_status()
        except Exception:
            return

        self._update_confirmed_motion_sequence(status)

        # Seed the stall-advance tracker from the firmware's current sequence.
        # _update_confirmed_motion_sequence no longer does this, so we must do
        # it explicitly here at creation time.
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence != 0xFFFF and received_sequence >= 0:
            self._last_sequence_advance_value = received_sequence
            self._last_sequence_advance_time = time.time()

    def _update_confirmed_motion_sequence(self, status) -> None:
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return

        self._last_confirmed_motion_seq = received_sequence
        self._last_confirmed_sequence = received_sequence
        # NOTE: _last_sequence_advance_value and _last_sequence_advance_time are
        # intentionally NOT updated here.  They are managed exclusively by
        # _check_stall() so that calling _remove_confirmed_segments() on every
        # poll cycle does not silently reset the stall timer when the firmware
        # is stuck (e.g. RMT failed to restart after emergency stop).

    def _wait_for_request_result(self, sequence: int, send_status: Any = None) -> Any:
        try:
            if send_status is not None:
                return self._transport.wait_for_request_result(
                    sequence,
                    hint_status=send_status,
                    poll_interval_s=self._poll_interval_s,
                )
            return self._transport.wait_for_request_result(
                sequence,
                poll_interval_s=self._poll_interval_s,
            )
        except TypeError as exc:
            if send_status is None or "hint_status" not in str(exc):
                raise
            return self._transport.wait_for_request_result(
                sequence,
                poll_interval_s=self._poll_interval_s,
            )

    def _enable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, send_status = self._transport.set_axis_enabled_request(axis_id, True)
            status = self._wait_for_request_result(sequence, send_status)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"enable axis {axis_id} failed with result=0x{status.last_result:02X}")

    def _disable_axes(self) -> None:
        for axis_id in self._axis_ids:
            if axis_id not in self._keep_enabled_axes:
                # For axes that will be disabled, send an explicit stop before
                # pulling EN high so the driver stops cleanly.
                # For keep_enabled_axes (homing phases), the move already
                # contains a built-in deceleration-to-zero profile; sending
                # stop_axis() here would start a firmware coast/brake sequence
                # on top of the ongoing decel, causing running_mask to stay
                # asserted far longer than the 1-second timeout that follows.
                self._transport.stop_axis(axis_id)

            stop_deadline = time.time() + 1.0
            while time.time() < stop_deadline:
                status = self._transport.get_status()
                if (int(status.running_mask) & (1 << axis_id)) == 0:
                    break
                time.sleep(self._poll_interval_s)

            if axis_id in self._keep_enabled_axes:
                continue

            sequence, send_status = self._transport.set_axis_enabled_request(axis_id, False)
            status = self._wait_for_request_result(sequence, send_status)
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
        self._update_confirmed_motion_sequence(status)
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
            self._last_confirmed_motion_seq = received_sequence

    def _check_stall(self, status) -> bool:
        """Return True and request stop if last_executed_sequence has not
        advanced for _stall_timeout_s while segments are in flight.

        A stall means the RMT is dead and pushBlock() is likely deadlocked.
        Requesting a stop+flush allows the host to recover gracefully.
        """
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return False

        # When no segment has been confirmed yet, only reset the stall timer if
        # there is nothing in-flight.  If segments ARE in-flight but none have been
        # confirmed, the timer keeps running so that a firmware RMT failure
        # (kickStart not triggered after emergency stop) is caught.
        if self._last_confirmed_sequence < 0:
            if not self._inflight:
                self._last_sequence_advance_time = time.time()
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

    def _log_runtime_diagnostics(self, status) -> None:
        raw_underrun = getattr(status, "underrun_count", (0, 0, 0, 0))
        underrun: tuple[int, int, int, int] = (
            int(raw_underrun[0]),
            int(raw_underrun[1]),
            int(raw_underrun[2]),
            int(raw_underrun[3]),
        )
        if self._last_underrun_count is None:
            self._last_underrun_count = underrun
        elif underrun != self._last_underrun_count:
            deltas = [curr - prev for curr, prev in zip(underrun, self._last_underrun_count)]
            if any(delta > 0 for delta in deltas):
                last_executed = int(getattr(status, "last_executed_sequence", -1))
                last_planned = int(getattr(status, "last_planned_sequence", -1))
                logger.warning(
                    "firmware underrun counter advanced: delta=%s total=%s queue_free=%s ring_free=%s multi_axis_free=%s planner_free=%s inflight=%s buffered=%.1fms last_executed=%s last_confirmed=%s last_sent=%s last_planned=%s",
                    deltas,
                    underrun,
                    getattr(status, "queue_free_slots", ()),
                    getattr(status, "ring_free_slots", ()),
                    getattr(status, "multi_axis_queue_free", -1),
                    getattr(status, "planner_queue_free", -1),
                    len(self._inflight),
                    self._buffered_time_s * 1000.0,
                    last_executed,
                    self._last_confirmed_sequence,
                    self._last_sent_motion_seq,
                    last_planned,
                )
            self._last_underrun_count = underrun

        segments_dropped = int(getattr(status, "segments_dropped", 0))
        if self._last_logged_segments_dropped is None:
            self._last_logged_segments_dropped = segments_dropped
        elif segments_dropped > self._last_logged_segments_dropped:
            delta = segments_dropped - self._last_logged_segments_dropped
            logger.warning(
                "planner dropped segments: delta=%s total=%s planner_free=%s inflight=%s last_executed=%s last_planned=%s",
                delta,
                segments_dropped,
                getattr(status, "planner_queue_free", -1),
                len(self._inflight),
                getattr(status, "last_executed_sequence", -1),
                getattr(status, "last_planned_sequence", -1),
            )
            self._last_logged_segments_dropped = segments_dropped
        elif segments_dropped < self._last_logged_segments_dropped:
            self._last_logged_segments_dropped = segments_dropped

        multi_axis_free = int(getattr(status, "multi_axis_queue_free", -1))
        planner_free = int(getattr(status, "planner_queue_free", -1))
        raw_ring_free = getattr(status, "ring_free_slots", (0, 0, 0, 0))
        ring_free: tuple[int, int, int, int] = (
            int(raw_ring_free[0]),
            int(raw_ring_free[1]),
            int(raw_ring_free[2]),
            int(raw_ring_free[3]),
        )
        last_executed = int(getattr(status, "last_executed_sequence", -1))
        tracked_ring_free = [
            ring_free[axis_id]
            for axis_id in self._axis_ids
            if 0 <= axis_id < len(ring_free)
        ]
        # Only fire when the ring is nearly empty (>= STEP_RING_CAPACITY - 256 free
        # slots = fewer than 256 steps = <2ms remaining at 1500 RPM).  The old
        # threshold of 2048 fired continuously at high speed because the ring is
        # always ~75% empty — a false positive that flooded the log.
        ring_nearly_empty = bool(
            tracked_ring_free
            and min(tracked_ring_free) >= self.STEP_RING_CAPACITY - 256
        )
        no_sequence_progress_s = time.time() - self._last_sequence_advance_time
        if (
            len(self._inflight) >= 32
            and self._buffered_time_s >= 0.200
            and multi_axis_free >= 60
            and planner_free >= 84
            and ring_nearly_empty
            and no_sequence_progress_s >= 0.100
        ):
            now = time.time()
            if now - self._last_desync_log_ts >= 0.5:
                self._last_desync_log_ts = now
                logger.warning(
                    "host/firmware buffer desync suspected: inflight=%s buffered=%.1fms multi_axis_free=%s planner_free=%s ring_free=%s last_executed=%s last_confirmed=%s last_sent=%s stalled_for=%.3fs",
                    len(self._inflight),
                    self._buffered_time_s * 1000.0,
                    multi_axis_free,
                    planner_free,
                    ring_free,
                    last_executed,
                    self._last_confirmed_sequence,
                    self._last_sent_motion_seq,
                    no_sequence_progress_s,
                )

    def _check_endstop(self, status) -> bool:
        """Return True if an endstop was triggered on any armed axis.

        Detection priority:
          1. ``endstop_hit_mask`` filtered on axes in ``_endstop_armed_axes``
             (canonical firmware signal, R2: only match bits for armed axes).
          2. Fallback: CLOSED lateral state + axis stopped, for armed axes.
          3. ``segments_dropped`` delta + any armed axis stopped (queue drop
             can indicate endstop; only fires when correlated with a stopped
             axis to avoid false positives from planner pressure alone).
        """
        armed_mask   = int(getattr(status, "endstop_armed_mask", 0))
        lateral_state = int(getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT))
        running_mask  = int(getattr(status, "running_mask", 0))
        hit_mask      = int(getattr(status, "endstop_hit_mask", 0))

        # R2: signal canonique filtré sur les axes réellement armés côté host.
        # Un bit résiduel sur un axe non armé (e.g. bug R1 non encore corrigé
        # sur firmware plus ancien) ne déclenche plus de faux endstop.
        for axis_id in self._endstop_armed_axes:
            if hit_mask & (1 << axis_id):
                self._mark_endstop_triggered()
                return True

        # Fallback : capteur fermé + axe armé arrêté
        for axis_id in self._endstop_armed_axes:
            axis_stopped = (running_mask & (1 << axis_id)) == 0
            if lateral_state == LATERAL_ENDSTOP_PRESENT_CLOSED and axis_stopped:
                self._mark_endstop_triggered()
                return True

        # segments_dropped uniquement hors homing et si un axe armé est aussi arrêté
        dropped_now = int(getattr(status, "segments_dropped", 0))
        if (
            not self._homing_mode
            and self._endstop_armed_axes
            and dropped_now > self._last_segments_dropped
        ):
            any_armed_stopped = any(
                (running_mask & (1 << a)) == 0
                for a in self._endstop_armed_axes
            )
            if any_armed_stopped:
                self._mark_endstop_triggered()
                return True
        # Toujours mettre à jour _last_segments_dropped, même hors déclenchement
        self._last_segments_dropped = dropped_now

        return False

    def note_endstop_armed(self, axis_id: int, arm: bool) -> None:
        if arm:
            self._endstop_armed_axes.add(axis_id)
        else:
            self._endstop_armed_axes.discard(axis_id)

    def set_homing_mode(self, enabled: bool) -> None:
        """Disable segments_dropped false-positive detection during homing."""
        self._homing_mode = bool(enabled)

    def _mark_endstop_triggered(self) -> None:
        if self._endstop_triggered:
            return
        self._endstop_triggered = True
        flush_seq = self._last_sent_motion_seq
        if flush_seq < 0:
            flush_seq = self._last_confirmed_motion_seq
        elif (
            self._last_confirmed_motion_seq >= 0
            and sequence_is_greater(self._last_confirmed_motion_seq, flush_seq)
        ):
            flush_seq = self._last_confirmed_motion_seq
        if flush_seq < 0:
            flush_seq = 0xFFFF
        self._flush_floor_sequence = int(flush_seq) & 0xFFFF
        self.request_stop()
        self.request_flush(flush_seq)

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
        sequence, send_status = self._transport.enable_endstop_request(axis_id, arm=True)
        self._wait_for_request_result(sequence, send_status)
        self._endstop_armed_axes.add(axis_id)

    def disarm_endstop(self, axis_id: int) -> None:
        """Send ENABLE_ENDSTOP disarm command to firmware.

        Call before a clearance move that must pass through the endstop.
        """
        sequence, send_status = self._transport.enable_endstop_request(axis_id, arm=False)
        self._wait_for_request_result(sequence, send_status)
        self._endstop_armed_axes.discard(axis_id)

    @property
    def endstop_triggered(self) -> bool:
        return self._endstop_triggered

    @property
    def last_sent_motion_seq(self) -> int:
        """Last motion sequence number sent to the firmware (or -1 if none)."""
        return self._last_sent_motion_seq

    @property
    def flush_floor_sequence(self) -> int:
        """Last flush floor requested by this streamer, or -1 if none."""
        return self._flush_floor_sequence

    # -- Stop / flush ----------------------------------------------------------

    def request_stop(self, *, keep_enabled_axes: set[int] | None = None) -> None:
        if keep_enabled_axes is not None:
            self._keep_enabled_axes = set(keep_enabled_axes)
        self._stop_requested = True

    def has_stop_been_requested(self) -> bool:
        return self._stop_requested

    def request_flush(self, sequence: int) -> None:
        self._flush_sequence_requested = sequence

    def flush_until(self, sequence: int) -> Any:
        status = self._transport.flush_until(sequence)
        self._inflight.clear()
        self._buffered_time_s = 0.0
        self._retry_batch = None
        return status

    def _flush_requested_stop(self) -> None:
        if self._flush_sequence_requested is None:
            return
        # Let the firmware finish publishing the stop/endstop status before
        # sending the explicit flush request on the same SPI link.
        time.sleep(self._poll_interval_s * 2.0)
        self.flush_until(self._flush_sequence_requested)
        self._flush_sequence_requested = None

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
        if self._generator_finished and self._retry_batch is None:
            return None

        effective_buffer_target_s = self._target_buffer_time_s
        if self._prefilling:
            effective_buffer_target_s = max(
                self._target_buffer_time_s,
                self.PREFILL_MAX_BUFFER_TIME_S,
            )

        planner_deficit = self._planner_buffer_deficit(status)
        ring_critical = self._ring_critically_low(status)

        if self._retry_batch is not None:
            batch = list(self._retry_batch)
            batch_duration_s = sum(seg.duration_us for seg in batch) / 1_000_000.0
        else:
            if planner_deficit <= 0 and not ring_critical:
                return None

            batch = []
            batch_duration_s = 0.0

            batch_target_segments = min(
                MULTI_AXIS_SEGMENT_BLOCK_SIZE,
                max(1, planner_deficit),
            )
            if ring_critical:
                batch_target_segments = max(batch_target_segments, 16)

            while (
                len(batch) < batch_target_segments
                and len(self._inflight) < self._max_inflight_segments()
                and not self._queue_full(status)
                and not self._check_planner_pressure(status)
            ):
                if self._pending_segment is not None:
                    segment = self._pending_segment
                    self._pending_segment = None
                else:
                    try:
                        segment = next(self._generator)
                    except StopIteration:
                        self._generator_finished = True
                        break

                segment_duration_s = segment.duration_us / 1_000_000.0
                if (
                    batch
                    and batch_duration_s + segment_duration_s > effective_buffer_target_s
                ):
                    self._pending_segment = segment
                    break

                reference_sequence = (
                    batch[-1].sequence
                    if batch
                    else self._last_sent_motion_seq
                )
                if reference_sequence >= 0 and not sequence_is_greater(
                    segment.sequence, reference_sequence
                ):
                    raise RuntimeError(
                        f"motion sequence not strictly increasing: "
                        f"got {segment.sequence}, last was {reference_sequence}"
                    )
                batch.append(segment)
                batch_duration_s += segment_duration_s
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
        ack_status = self._wait_for_request_result(transport_seq, send_status)
        self._update_confirmed_motion_sequence(ack_status)

        if ack_status.last_result == int(SpiMessageResult.OK):
            self._retry_batch = None
            for seg in batch:
                self._inflight.append((seg, transport_seq))
                self._buffered_time_s += seg.duration_us / 1_000_000.0
                self._last_sent_motion_seq = seg.sequence
                self._record_send_event(seg, transport_seq, ack_status)
            self._last_sent_transport_seq = transport_seq
            return len(batch), ack_status
        elif ack_status.last_result == int(SpiMessageResult.QUEUE_FULL):
            if self._retry_batch is None:
                self._retry_batch = list(batch)
            return 0, ack_status
        elif ack_status.last_result == int(SpiMessageResult.ENDSTOP_BLOCKED):
            logger.info(
                "endstop blocked batch motion_seq=%s transport_seq=%s",
                batch[0].sequence,
                transport_seq,
            )
            self._mark_endstop_triggered()
            return 0, ack_status
        else:
            raise RuntimeError(
                f"segment batch starting motion_seq={batch[0].sequence} "
                f"failed with result=0x{ack_status.last_result:02X}"
            )

    def _prefill(self, status) -> tuple[int, Any]:
        """Pre-send segments to seed the ESP32 planner queue before RMT starts.

        The prefill target is computed from the startup speed estimate
        (_initial_steps_per_segment), not the live segment state, to avoid
        low-speed misclassification at startup.

        Speed tiers (steps_per_segment):
          - < 10: 64 segments (low speed, conservative fill)
          - >= 10: required_lookahead(initial_steps) (speed-appropriate fill)

        The _prefilling flag is set for the duration of this call so that
        _check_planner_pressure() does not prematurely gate sends before the
        target depth has been reached.

        Returns (total_segments_sent, last_status).
        """
        # Use initial startup speed estimate, not the live segment state.
        initial_steps = self._initial_steps_per_segment
        if initial_steps < 10:
            prefill_target = 64
        else:
            prefill_target = self.required_lookahead(initial_steps)

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
                # Prevent spamming SPI when streaming large prefill blocks
        finally:
            self._prefilling = False
        return total, last_status

    def _ring_critically_low(self, status) -> bool:
        """Return True when the ESP32 ring has fewer than 256 steps buffered.

        When True, the host must send immediately regardless of
        _buffered_time_s to prevent a ring underrun.

        256 steps ≈ 51 ms at 1500 RPM (5000 steps/s).
        Threshold: ring_free >= STEP_RING_CAPACITY - 256 = 3840.
        A high ring_free means few steps remain (ring is nearly empty).
        """
        ring_free = getattr(status, "ring_free_slots", ())
        return any(
            0 <= axis_id < len(ring_free)
            and int(ring_free[axis_id]) >= self.STEP_RING_CAPACITY - 256
            for axis_id in self._axis_ids
        )

    def _should_sleep(self, status) -> float:
        """Return sleep duration in seconds from real firmware queue pressure.

        The old policy slept for up to one full segment duration (4 ms) based on
        the host-side buffered_time estimate. That estimate can lag behind the
        real ring/planner drain, which is exactly how we ended up idling while
        the ESP32 was already running dry.

        New rule:
          - no sleep while the ring is critical or planner is below target
          - otherwise only a short poll-interval sleep
        """
        if self._ring_critically_low(status):
            return 0.0
        if self._planner_buffer_deficit(status) > 0:
            return 0.0
        return self._poll_interval_s

    # -- Main streaming loop ---------------------------------------------------

    def stream_all(self) -> int:
        self._generator_finished = False
        status = self._transport.get_status()
        self._update_confirmed_motion_sequence(status)
        axes_enabled = False
        total_segments = 0
        stream_error: Exception | None = None

        try:
            self._enable_axes()
            axes_enabled = True
            status = self._transport.get_status()
            self._update_confirmed_motion_sequence(status)

            self._check_endstop(status)
            if not self._stop_requested:
                total_segments, status = self._prefill(status)

            while True:
                if self._stop_requested:
                    self._flush_requested_stop()
                    break

                # Reuse status from the last send/prefill instead of a dedicated
                # get_status() call.  The SPI full-duplex response already contains
                # fresh status, so an extra round-trip would waste ~250 µs per
                # iteration and halve the effective SPI bandwidth.
                self._remove_confirmed_segments(status)
                self._log_runtime_diagnostics(status)
                if self._stop_requested:
                    self._flush_requested_stop()
                    break
                self._check_premature_completion(status)
                if self._check_stall(status):
                    self._flush_requested_stop()
                    break

                if self._check_endstop(status):
                    self._flush_requested_stop()
                    break

                # Rate-limit sends to MAX_SEGMENTS_PER_CYCLE per polling iteration to
                # prevent burst-after-throttle that overflows the ESP32 defer ring.
                cycle_segments_sent = 0
                while not self._generator_finished:
                    if self._stop_requested or self._endstop_triggered:
                        break
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
                    # Tight-loop pacing is provided by the per-transfer guard
                    # in spi_transport plus the outer-loop sleep policy. Avoid
                    # inserting another ad hoc per-batch sleep here unless a
                    # new bench run shows the mode-1 link still needs it.

                if self._flush_sequence_requested is not None:
                    self._flush_requested_stop()

                if self._generator_finished and not self._inflight and self._retry_batch is None:
                    break

                # If no segments were sent this cycle, we have no fresh status
                # from a send response.  Poll once to keep confirmation flowing.
                if cycle_segments_sent == 0:
                    status = self._transport.get_status()
                    self._remove_confirmed_segments(status)

                sleep_s = self._should_sleep(status)
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        except Exception as exc:
            stream_error = exc
            raise
        finally:
            cleanup_error: Exception | None = None
            if axes_enabled:
                try:
                    self._disable_axes()
                except Exception as exc:
                    cleanup_error = exc
                    logger.exception("failed to disable axes")
            if cleanup_error is not None and stream_error is None:
                raise RuntimeError(f"stream cleanup failed: {cleanup_error}") from cleanup_error

        self._write_send_log()
        return total_segments
