# PERF_ANALYSIS.md

## 1. Causes racines identifiées

| Fichier | Bug | Impact mesuré / estimé |
|---|---|---|
| src/rpi/transport/streamer.py | `_current_steps_per_segment` restait à `0` via `__init__` (RampMove), donc classification « low speed » au démarrage. | À 1500 RPM (
$160000$ steps/s), prefill forcé à 64 segments au lieu d’un prefill haute vitesse (16), soit $64\times 4\,ms = 256\,ms$ de latence de démarrage et pression inutile sur pipeline. |
| src/rpi/transport/streamer.py | `_safe_buffer_time_s` clampait `requested_time_s` en entrée avant calcul ring-safety. | Couplage incorrect entre politique UI/host et capacité ring. Peut forcer une cible incohérente avec la capacité réelle si paramètres évoluent. |
| src/rpi/transport/streamer.py | `STEP_RING_CAPACITY` hardcodé sans vérification firmware directe. | Risque de divergence host/firmware si `STEP_RING_SIZE` change. Dans cette base: valeur confirmée à 4096, donc pas de mismatch actuel. |
| src/rpi/transport/streamer.py | `_prefill()` basé implicitement sur l’état live, vulnérable à une mauvaise initialisation de vitesse. | Au démarrage, préremplissage surdimensionné (64) même en croisière rapide. |
| src/rpi/transport/streamer.py | Stall detection active trop tôt + timeout court (2.0s) face à notifications différées. | Faux positifs possibles en phase de démarrage/prefill long, surtout sous charge système. |
| src/esp32/src/motion_planner.cpp | `PLANNER_PRIO=12` > `SPI_TASK_PRIO=10` (inversion de priorité potentielle). | Backpressure pouvant retarder SPI et créer blocage de progression dans certains régimes de remplissage/vidage de queues. |

---

## 2. Sources complètes modifiées

### transport/streamer.py

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
        self._premature_notify_count: int = 0
        self._premature_notify_window_start: float = 0.0
        self._last_sequence_advance_time: float = time.time()
        self._last_sequence_advance_value: int = -1
        self._stall_timeout_s: float = 5.0  # stall if no progress for 5s

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
        if max_hz <= 0.0:
            return max(self.MIN_BUFFER_TIME_S, min(self.MAX_BUFFER_TIME_S, requested_time_s))
        safe_time_s = (self.STEP_RING_CAPACITY * self.RING_BUFFER_HEADROOM) / max_hz
        # Use the minimum of what was requested and what the ring can hold.
        # Never go below MIN_BUFFER_TIME_S (needed for SPI pipeline latency).
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

        # Do not arm stall detection until the first executed segment
        # has been confirmed by firmware.
        if self._last_confirmed_sequence < 0:
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
        # Use initial startup speed estimate, not the live segment state.
        initial_steps = self._initial_steps_per_segment
        if initial_steps < 10:
            prefill_target = 64   # half of SEGMENT_QUEUE_DEPTH
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
static constexpr UBaseType_t PLANNER_PRIO  = 8;   // Below SPI (10), above idle
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

---

## 3. Sources complètes fournies pour analyse

### src/esp32/src/stepper_driver.h

```cpp
/**
 * @file stepper_driver.h
 * @brief Physical-layer RMT stepper driver — one instance per motor axis.
 *
 * Converts pre-timed step_block_t arrays into a gapless RMT symbol stream
 * using a simple_encoder callback (FastAccelStepper-style ping-pong).
 *
 * ── Streaming architecture ─────────────────────────────────────────────────
 *   Producer (pushBlock, task context):
 *     1. Convert step_block_t → ring_entry_t[] in the lock-free ring buffer.
 *     2. If the ring is full, block on a task notification from the ISR.
 *     3. If RMT is not streaming, start a new rmt_transmit().
 *
 *   Consumer (encode_steps callback, ISR context):
 *     4. RMT hardware calls encode_steps() when it needs more symbols.
 *     5. Callback reads PART_SIZE entries from the ring buffer, converts
 *        each to one rmt_symbol_word_t (PULSE_TICKS HIGH, remainder LOW).
 *     6. On starvation, callback emits one pause chunk, arms stop, and ends
 *        the transaction on the next callback (FastAccelStepper-style).
 *
 *   on_trans_done ISR:
 *     7. Marks rmt_running_ = false so the next pushBlock() restarts.
 *
 *   This eliminates inter-block gaps in the normal case without task-side
 *   busy-wait loops.
 *
 * ── Direction constraint ───────────────────────────────────────────────────
 *   Direction changes are handled in ISR context via gpio_ll (register-level).
 *   When a ring entry has toggle_dir=1 and the previous chunk contained step
 *   pulses, a pause chunk is emitted first to meet the driver IC's direction
 *   setup time, and the toggle is deferred to the next callback invocation.
 */

#pragma once

#include <atomic>
#include <driver/rmt_tx.h>
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include "step_types.h"

class StepperDriver {
public:
    // ─── RMT clock constants (typed aliases — authoritative values in step_types.h) ──
    // These use different names to avoid colliding with the same-named macros.
    //
    // Key relationships:
    //   RMT_CLK_HZ = 80 MHz  →  1 tick = 12.5 ns
    //   RMT_TICKS_PER_US_C = RMT_CLK_HZ / 1e6 = 80 ticks/µs
    //   RMT_MIN_TICKS_C = 16  →  max_step_rate = 80e6 / 16 = 5 MHz
    //   max_rpm = max_step_rate / (steps_per_rev × microsteps)
    //           = 5 000 000 / (200 × 32) = 781 RPM
    //   At cruise 160 kHz: interval_ticks = 80e6 / 160000 = 500 ticks
    static constexpr uint32_t RMT_CLK_HZ          = 80000000UL;          // 80 MHz
    static constexpr uint32_t RMT_TICKS_PER_US_C  = RMT_CLK_HZ / 1000000UL; // 80
    static constexpr uint32_t RMT_PULSE_TICKS_C   = 8U;   // 8 × 12.5 ns = 100 ns HIGH
    static constexpr uint32_t RMT_MIN_TICKS_C     = 16U;  // 16 × 12.5 ns = 200 ns → 5 MHz ceiling
    static constexpr uint32_t RMT_MAX_TICKS_C     = 0xFFFFU;
    /**
     * @brief Construct a StepperDriver.
     *
     * @param step_pin  GPIO for the STEP signal (RMT output).
     * @param dir_pin   GPIO for the DIR signal.
     * @param en_pin    GPIO for the EN signal (active-LOW on DRV8825/A4988).
     * @param motor_id  Logical motor index (0-based, for logging).
     */
    StepperDriver(gpio_num_t step_pin, gpio_num_t dir_pin,
                  gpio_num_t en_pin,   uint8_t    motor_id);

    /** @brief Initialise GPIO, create RMT channel and simple encoder, register ISR. */
    esp_err_t init();

    /** @brief Assert EN pin (driver IC powered, coils energised). */
    void enable();

    /** @brief De-assert EN pin (driver IC off, coils de-energised). */
    void disable();

    /** @brief True if the driver output is currently enabled. */
    bool isEnabled() const { return enabled_; }

    /**
     * @brief Immediate stop: flush the RMT TX queue and reset ring buffer.
     *
     * Called from task context only.
     */
    void emergencyStop();

    /**
     * @brief Gracefully end the RMT stream.  Blocks until the current
     *        transmission finishes.  Safe to call from task context only.
     */
    void stopStream();

    /**
     * @brief Signal the RMT stream to stop after the current ring contents
     *        have been consumed.  Does NOT reset the ring buffer.
     *
     * Contrast with emergencyStop() which flushes the ring immediately.
     * Safe to call from task context only.
     */
    void gracefulStop();

    /**
     * @brief Push a block of steps into the ring buffer for streaming.
     *
     * Converts step_block_t → ring_entry_t[] and writes them to the SPSC
     * ring buffer.  If the ring is full, blocks on a task notification from
     * the ISR until space becomes available.
     *
     * @param block        Block of pre-timed step commands.
     * @param caller_task  Handle of the calling task; stored as producer_task_
     *                     so the ISR ring-space notification wakes the right
     *                     task.  Pass xTaskGetCurrentTaskHandle() from the
     *                     caller (StepperQueue::pushExpandedBlock).
     * @return ESP_OK, or an RMT error code on stream start failure.
     */
    esp_err_t pushBlock(const step_block_t& block, TaskHandle_t caller_task);

    /** @brief Return the motor id supplied at construction (0 or 1). */
    uint8_t motorId() const { return motor_id_; }

    /**
     * @brief Start (or restart) RMT streaming.
     *
     * Must be called by the executor task after draining all available step
     * blocks into the ring buffer so that the ring is maximally full before
     * the RMT hardware starts consuming entries.  Safe to call from task
     * context only.
     */
    esp_err_t startStream();

    /** @brief Approximate number of free slots remaining in the software ring. */
    uint32_t ringFreeSlots() const { return ringFree(); }

    /** @brief True while an RMT transaction is currently active. */
    bool isStreaming() const { return rmt_running_.load(std::memory_order_acquire); }
    bool isStopped()  const { return rmt_stopped_.load(std::memory_order_acquire); }

    // ─── Ring buffer (SPSC: task writes, ISR reads) ────────────────────────
    // Public because the C encoder callback needs direct access in ISR context.

    ring_entry_t          ring_[STEP_RING_SIZE];     /**< Step ring buffer      */
    /**
     * Ring write index. Written ONLY by the producer task, read by ISR.
     * std::atomic with release/acquire ordering guarantees the ISR sees
     * fully-written ring_entry_t data before the index advances.
     */
    std::atomic<uint32_t> ring_write_ {0};
    /**
     * Ring read index. Written ONLY by the ISR (encode_steps), read by
     * the producer task for free-slot calculation.
     */
    std::atomic<uint32_t> ring_read_  {0};

    gpio_num_t            dir_pin_;                  /**< For ISR gpio_ll       */
    /**
     * Set by encode_steps() ISR to signal the RMT transaction should end.
     * Read by pushBlock()/startStream() in task context.
     */
    std::atomic<bool>     rmt_stopped_ {true};
    bool                  last_chunk_had_steps_ {false}; /**< Dir-change safety (ISR only) */
    uint16_t              last_ticks_ {RMT_STEP_DEFAULT_TICKS}; /**< Last step interval (ISR only) */

    /**
     * @brief Task handle of the current ring producer.
     *
     * Set to the calling task every time pushBlock() is entered so the ring
     * back-pressure (ulTaskNotifyTake) always wakes the correct task.
     * Written from task context, read from ISR — std::atomic for safety.
     */
    std::atomic<TaskHandle_t> producer_task_ {nullptr};

    /**
     * @brief Task handle of the multi-axis executor (Core 1).
     *
     * Set once at startup by CommInterface via setExecutorTask().
     * encode_steps notifies BOTH this handle and producer_task_ so the
     * executor can pre-emptively refill the ring before it stalls.
     */
    std::atomic<TaskHandle_t> executor_task_ {nullptr};

    /** Register the multi-axis executor task handle for ring-low wakeups. */
    void setExecutorTask(TaskHandle_t t) { executor_task_.store(t, std::memory_order_release); }

    /**
     * @brief Set by the GPIO endstop ISR when contact is detected.
     * Read by encode_steps() in ISR context to stop the RMT immediately.
     * Cleared by the host via SPI ENABLE_ENDSTOP command or when the
     * endstop sensor returns to open state.
     * std::atomic for ISR ↔ task safety.
     */
    std::atomic<bool> endstop_active_ {false};

    /** @brief Arm the endstop — ISR will stop motion on trigger. */
    void armEndstop() {
        endstop_active_.store(false, std::memory_order_release);
        endstop_armed_.store(true, std::memory_order_release);
    }

    /** @brief Disarm the endstop — ISR will not stop motion on trigger.
     *  Use during intentional clearance moves commanded by the host. */
    void disarmEndstop() { endstop_armed_.store(false, std::memory_order_release); }

    /** @brief True if the endstop is currently armed. */
    bool isEndstopArmed() const { return endstop_armed_.load(std::memory_order_acquire); }

    /** @brief True if the endstop is currently triggered. */
    bool isEndstopActive() const { return endstop_active_.load(std::memory_order_acquire); }

    /**
     * @brief Install GPIO edge-triggered ISR on the NO/NC endstop pins.
     *
     * Called once from CommInterface::init() after the driver is ready.
     * Stores the pin numbers so the static ISR can access them via the
     * driver pointer without touching CommInterface state.
     *
     * @param no_pin  GPIO of the Normally-Open contact  (GPIO_NUM_NC = skip).
     * @param nc_pin  GPIO of the Normally-Closed contact (GPIO_NUM_NC = skip).
     */
    esp_err_t initEndstopIsr(gpio_num_t no_pin, gpio_num_t nc_pin);

    /**
     * Incremented in ISR each time encode_steps() finds the ring empty
     * and emits a pause chunk before stopping the transaction.
     * std::atomic for ISR ↔ task safety.
     */
    std::atomic<uint32_t> ring_underrun_count_ {0};

    uint32_t getUnderrunCount()  const { return ring_underrun_count_.load(std::memory_order_relaxed); }
    void     resetUnderrunCount()      { ring_underrun_count_.store(0, std::memory_order_relaxed); }
private:
    gpio_num_t            step_pin_;
    gpio_num_t            en_pin_;
    uint8_t               motor_id_;

    rmt_channel_handle_t  channel_   {nullptr};
    rmt_encoder_handle_t  encoder_   {nullptr};
    rmt_transmit_config_t tx_config_ {};

    std::atomic<bool>     rmt_running_ {false};
    bool                  last_dir_    {true};
    bool                  enabled_     {false};
    std::atomic<bool>     endstop_armed_  {false};

    /** Endstop pin numbers — set by initEndstopIsr(), read by endstopIsrHandler(). */
    gpio_num_t            endstop_no_pin_ {GPIO_NUM_NC};
    gpio_num_t            endstop_nc_pin_ {GPIO_NUM_NC};

    /** @brief Number of free slots in the ring buffer. */
    uint32_t ringFree() const {
        return STEP_RING_SIZE
            - (ring_write_.load(std::memory_order_relaxed)
               - ring_read_.load(std::memory_order_relaxed));
    }

    // ── Static ISR callbacks ────────────────────────────────────────────────
    static bool on_trans_done_isr(rmt_channel_handle_t tx_chan,
                                  const rmt_tx_done_event_data_t* edata,
                                  void* user_ctx);

    /**
     * @brief GPIO ISR — fires on any edge of either endstop pin (NO or NC).
     * arg = StepperDriver* that owns the endstop.
     * Validates NO/NC logic and sets endstop_active_ for sub-100 µs RMT stop.
     */
    static void IRAM_ATTR endstopIsrHandler(void* arg);
};

/**
 * @brief Simple encoder callback — called from ISR context by the RMT driver.
 *
 * Reads up to PART_SIZE entries from the ring buffer and converts them to
 * RMT symbols.  Handles direction changes with safety pauses.
 */
extern "C" size_t encode_steps(const void* data, size_t data_size,
                                          size_t symbols_written,
                                          size_t symbols_free,
                                          rmt_symbol_word_t* symbols,
                                          bool* done, void* arg);
```

### src/esp32/src/stepper_queue.cpp

```cpp
/**
 * @file stepper_queue.cpp
 * @brief Per-motor FreeRTOS queue + executor task implementation.
 */

#include "stepper_queue.h"

#include <esp_log.h>
#include <esp_check.h>

static const char* TAG = "stepper_queue";

// Compile-time invariant: the auto-start fill threshold must be large enough
// that the second encode_steps ping-pong callback never immediately underruns.
static_assert(STEP_STREAM_START_FILL >= 2 * PART_SIZE,
              "STEP_STREAM_START_FILL must be >= 2 * PART_SIZE to prevent "
              "immediate ISR underrun on second encoder callback");

// Task parameters
static constexpr uint32_t EXECUTOR_STACK_WORDS = 4096;
static constexpr UBaseType_t EXECUTOR_PRIORITY  = 24;
static constexpr BaseType_t  EXECUTOR_CORE      = 1; // Pro CPU (real-time)

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

StepperQueue::StepperQueue(StepperDriver& driver, uint8_t motor_id)
    : driver_(driver)
    , motor_id_(motor_id)
{}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::init()
{
    // Each queue item is one `motion_block_t`, which can carry either a legacy
    // `step_block_t` or a compressed `segment_block_t`.
    queue_ = xQueueCreate(STEPPER_QUEUE_DEPTH, sizeof(motion_block_t));
    ESP_RETURN_ON_FALSE(queue_ != nullptr, ESP_ERR_NO_MEM, TAG,
                        "motor%u: failed to create step queue", motor_id_);

    // ── Executor task ───────────────────────────────────────────────────────
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "stepper_%u", motor_id_);

    BaseType_t rc = xTaskCreatePinnedToCore(
        &StepperQueue::executorTask,
        task_name,
        EXECUTOR_STACK_WORDS,
        this,
        EXECUTOR_PRIORITY,
        &task_,
        EXECUTOR_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "motor%u: failed to create executor task", motor_id_);

    ESP_LOGI(TAG, "motor%u: queue depth=%d  task priority=%d  core=%d",
             motor_id_, STEPPER_QUEUE_DEPTH,
             (int)EXECUTOR_PRIORITY, (int)EXECUTOR_CORE);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// enqueueMotionBlock() / enqueueStepBlock() / enqueueSegmentBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::enqueueMotionBlock(const motion_block_t& block,
                                           uint32_t timeout_ms)
{
    const TickType_t ticks = (timeout_ms == portMAX_DELAY)
                             ? portMAX_DELAY
                             : pdMS_TO_TICKS(timeout_ms);

    if (xQueueSend(queue_, &block, ticks) != pdTRUE) {
        ESP_LOGW(TAG, "motor%u: queue full — motion block dropped",
                 motor_id_);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t StepperQueue::enqueueStepBlock(const step_block_t& block,
                                         uint32_t timeout_ms)
{
    motion_block_t motion {};
    motion.kind = MOTION_BLOCK_KIND_STEP;
    motion.payload.step = block;
    return enqueueMotionBlock(motion, timeout_ms);
}

esp_err_t StepperQueue::enqueueSegmentBlock(const segment_block_t& block,
                                            uint32_t timeout_ms)
{
    motion_block_t motion {};
    motion.kind = MOTION_BLOCK_KIND_SEGMENT;
    motion.payload.segment = block;
    return enqueueMotionBlock(motion, timeout_ms);
}

// ---------------------------------------------------------------------------
// available()
// ---------------------------------------------------------------------------

uint32_t StepperQueue::available() const
{
    return static_cast<uint32_t>(uxQueueSpacesAvailable(queue_));
}

// ---------------------------------------------------------------------------
// executeConstantRateBlock() / kickStart()
// ---------------------------------------------------------------------------
//
// executeConstantRateBlock() deliberately does NOT call maybeStartDriver().
// The start decision belongs to the caller: multiAxisExecutorTask drain loop
// in comm_interface.cpp calls kickStart() ONCE per block batch, after ALL
// available blocks have been written to the ring.  This guarantees the ring
// is pre-filled with multiple segments of look-ahead before RMT starts,
// preventing the per-segment underruns that occur at low speed when each
// segment contributes only 2–5 steps.

esp_err_t StepperQueue::executeConstantRateBlock(bool direction,
                                                  uint16_t step_count,
                                                  uint32_t duration_us)
{
    if (step_count == 0) {
        return ESP_OK;
    }

    // Compute uniform interval: RMT clock is 80 MHz → 80 ticks/µs.
    uint32_t interval_ticks = (duration_us * RMT_TICKS_PER_US) / step_count;
    if (interval_ticks < RMT_STEP_MIN_TICKS) {
        interval_ticks = RMT_STEP_MIN_TICKS;
    }
    if (interval_ticks > RMT_STEP_MAX_TICKS) {
        interval_ticks = RMT_STEP_MAX_TICKS;
    }

    uint32_t remaining = step_count;
    while (remaining > 0) {
        step_block_t expanded {};
        expanded.count = (remaining > STEP_BLOCK_SIZE) ? STEP_BLOCK_SIZE : remaining;
        for (uint32_t i = 0; i < expanded.count; ++i) {
            expanded.steps[i].interval_ticks = interval_ticks;
            expanded.steps[i].direction       = direction;
        }
        esp_err_t err = pushExpandedBlock(driver_, expanded);
        if (err != ESP_OK) {
            return err;
        }
        remaining -= expanded.count;
    }
    return ESP_OK;
}

esp_err_t StepperQueue::kickStart()
{
    return maybeStartDriver(driver_, true);
}

void StepperQueue::gracefulStop()
{
    driver_.gracefulStop();
}

// ---------------------------------------------------------------------------
// maybeStartDriver() / pushExpandedBlock() / executeSegmentBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::maybeStartDriver(StepperDriver& driver, bool force_start)
{
    const uint32_t buffered_steps = STEP_RING_SIZE - driver.ringFreeSlots();
    const bool ring_has_data = buffered_steps > 0;
    if (!ring_has_data) {
        return ESP_OK;
    }

    // Guard force_start: never begin streaming with fewer than PART_SIZE steps.
    // The RMT ping-pong encoder's first callback requests PART_SIZE symbols; if
    // fewer steps are available it immediately underruns and halts the motor.
    // At slow speeds (2-9 steps/segment during acceleration) this was causing a
    // stutter on every segment.  STEP_STREAM_START_FILL and the ring-full case
    // are unaffected — those scenarios already imply >= PART_SIZE steps buffered.
    const bool is_restart = !driver.isStreaming()
                            && buffered_steps > 0
                            && driver.getUnderrunCount() > 0;
    const bool should_start = (force_start && buffered_steps >= PART_SIZE)
        || (buffered_steps >= STEP_STREAM_START_FILL)
        || (driver.ringFreeSlots() == 0)
        || (is_restart && buffered_steps >= STEP_STREAM_RESTART_FILL);
    if (!should_start) {
        return ESP_OK;
    }

    if (driver.isStreaming() && !driver.isStopped()) {
        return ESP_OK;
    }

    if (driver.isStreaming()) {
        driver.stopStream();
    }
    return driver.startStream();
}

esp_err_t StepperQueue::pushExpandedBlock(StepperDriver& driver, const step_block_t& block)
{
    if (!driver.isStreaming() && driver.ringFreeSlots() == 0) {
        esp_err_t err = maybeStartDriver(driver, true);
        if (err != ESP_OK) {
            return err;
        }
    }

    // Pass the current task handle so ISR ring-space notifications wake
    // whichever task is currently blocked on this ring (multiAxisExecutorTask
    // or per-axis executorTask).
    esp_err_t err = driver.pushBlock(block, xTaskGetCurrentTaskHandle());
    if (err != ESP_OK) {
        return err;
    }
    return maybeStartDriver(driver, false);
}

esp_err_t StepperQueue::executeSegmentBlock(StepperDriver& driver, const segment_block_t& block)
{
    for (uint32_t seg_index = 0; seg_index < block.count; ++seg_index) {
        const motion_segment_t& seg = block.segments[seg_index];
        if (seg.step_count == 0) {
            continue;
        }

        uint32_t remaining = seg.step_count;
        int32_t current_ticks = seg.start_ticks;
        while (remaining > 0) {
            step_block_t expanded {};
            expanded.count = (remaining > STEP_BLOCK_SIZE) ? STEP_BLOCK_SIZE : remaining;
            for (uint32_t i = 0; i < expanded.count; ++i) {
                uint32_t clamped_ticks = static_cast<uint32_t>(current_ticks);
                if (clamped_ticks < RMT_STEP_MIN_TICKS) {
                    clamped_ticks = RMT_STEP_MIN_TICKS;
                }
                if (clamped_ticks > RMT_STEP_MAX_TICKS) {
                    clamped_ticks = RMT_STEP_MAX_TICKS;
                }
                expanded.steps[i].interval_ticks = clamped_ticks;
                expanded.steps[i].direction = (seg.direction != 0);
                current_ticks += seg.add_ticks;
            }

            esp_err_t err = pushExpandedBlock(driver, expanded);
            if (err != ESP_OK) {
                return err;
            }
            remaining -= expanded.count;
        }
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// executorTask()  — Core 1, priority 24
// ---------------------------------------------------------------------------
void StepperQueue::executorTask(void* arg)
{
    StepperQueue* self = static_cast<StepperQueue*>(arg);
    StepperDriver& driver = self->driver_;
    motion_block_t block;

    ESP_LOGI(TAG, "motor%u: executor task started", self->motor_id_);

    constexpr int WORK_BUDGET = 8;  // nombre de blocks à traiter avant pause

    for (;;) {
        // Bloque jusqu'à avoir du travail (parfait 👍)
        if (xQueueReceive(self->queue_, &block, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int work_done = 0;

        do {
            esp_err_t err = ESP_OK;

            if (block.kind == MOTION_BLOCK_KIND_SEGMENT) {
                err = executeSegmentBlock(driver, block.payload.segment);
            } else {
                err = pushExpandedBlock(driver, block.payload.step);
            }

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "motor%u: motion execute error: %s",
                         self->motor_id_, esp_err_to_name(err));
            }

            work_done++;

            // 🔥 POINT CLÉ : respiration contrôlée
            if (work_done >= WORK_BUDGET) {
                work_done = 0;

                // Option 1 (rapide)
                //taskYIELD();

                // Option 2 (ultra safe watchdog)
                 vTaskDelay(1);
            }

        } while (xQueueReceive(self->queue_, &block, 0) == pdTRUE);

        // Start uniquement après batch complet (logique déjà bonne 👍)
        esp_err_t err = maybeStartDriver(driver, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "motor%u: startStream error: %s",
                     self->motor_id_, esp_err_to_name(err));
        }
    }
}
```

### src/esp32/src/stepper_queue.h

```cpp
/**
 * @file stepper_queue.h
 * @brief Per-motor FreeRTOS queue + executor task for motion blocks.
 *
 * The host now sends compressed motion segments rather than only explicit
 * per-step blocks. `StepperQueue` remains the boundary between transport and
 * execution:
 *
 * - Core 0 / SPI task enqueues `motion_block_t`
 * - Core 1 / executor task expands segments into `step_block_t`
 * - `StepperDriver` streams the concrete steps via RMT
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_err.h>
#include "step_types.h"
#include "stepper_driver.h"

class StepperQueue {
public:
    /**
     * @brief Construct a StepperQueue bound to a StepperDriver.
     *
     * @param driver    Initialised StepperDriver instance for this motor.
     * @param motor_id  Logical motor index (0-based, for logging).
     */
    StepperQueue(StepperDriver& driver, uint8_t motor_id);

    /**
     * @brief Create the FreeRTOS queue and launch the executor task.
     *
     * Must be called after StepperDriver::init().
     */
    esp_err_t init();

    /**
     * @brief Enqueue a step block for execution.
     *
     * Called by the communication layer (producer side).  Blocks for up to
     * @p timeout_ms milliseconds if the queue is full.
     *
     * @param block       Block of pre-timed step commands.
     * @param timeout_ms  Maximum wait time in ms (0 = non-blocking).
     * @return ESP_OK on success, ESP_ERR_TIMEOUT if the queue was full.
     */
    esp_err_t enqueueMotionBlock(const motion_block_t& block,
                                 uint32_t timeout_ms = portMAX_DELAY);

    esp_err_t enqueueStepBlock(const step_block_t& block,
                               uint32_t timeout_ms = portMAX_DELAY);

    esp_err_t enqueueSegmentBlock(const segment_block_t& block,
                                  uint32_t timeout_ms = portMAX_DELAY);

    /** @brief Legacy compatibility wrapper for old explicit-step producers. */
    esp_err_t enqueueBlock(const step_block_t& block,
                           uint32_t timeout_ms = portMAX_DELAY) {
        return enqueueStepBlock(block, timeout_ms);
    }

    /**
     * @brief Emit a constant-rate step burst for use by the multi-axis executor.
     *
     * Computes a uniform step interval from @p duration_us / @p step_count,
     * clamps it to the RMT hardware limits, then calls pushExpandedBlock()
     * to fill the RMT ring.
     *
     * This method is intended to be called from the global multi-axis executor
     * task (Core 1) when the per-axis queue is empty and not competing for the
     * driver.  It must NOT be called concurrently with the per-axis executor
     * task for the same motor.
     *
     * @param direction   true = forward, false = reverse.
     * @param step_count  Number of steps to emit.
     * @param duration_us Segment wall-clock duration in microseconds.
     * @return ESP_OK on success, error code on RMT/ring error.
     */
    esp_err_t executeConstantRateBlock(bool direction,
                                       uint16_t step_count,
                                       uint32_t duration_us);

    /**
     * @brief Force-start the RMT stream if ring has data and is not running.
     *
     * Call after distributing steps across all axes in a multi-axis segment
     * to ensure all drivers begin streaming simultaneously.
     */
    esp_err_t kickStart();

    /**
     * @brief Signal the motor to stop after the current ring contents drain.
     *
     * Does NOT flush the ring buffer (contrast with emergencyStop via driver).
     * The motor decelerates naturally to zero as pre-queued steps are consumed.
     */
    void gracefulStop();

    /**
     * @brief Number of free slots remaining in the block queue.
     *
     * Use for flow control: signal the host when this drops below
     * FLOW_CONTROL_THRESHOLD.
     */
    uint32_t available() const;

    /** @brief Return the motor id (0 or 1). */
    uint8_t motorId() const { return motor_id_; }

    /** @brief Access the bound driver (for status / enable / estop handling). */
    StepperDriver& driver() { return driver_; }

    /** @brief Const access to the bound driver. */
    const StepperDriver& driver() const { return driver_; }

private:
    StepperDriver& driver_;
    uint8_t        motor_id_;

    QueueHandle_t  queue_  {nullptr};
    TaskHandle_t   task_   {nullptr};

    static esp_err_t maybeStartDriver(StepperDriver& driver, bool force_start);
    static esp_err_t pushExpandedBlock(StepperDriver& driver, const step_block_t& block);
    static esp_err_t executeSegmentBlock(StepperDriver& driver, const segment_block_t& block);

    /**
     * @brief Executor task body.
     *
    * Pinned to Core 1, priority 24. Dequeues `motion_block_t`, expands any
    * compressed segments into `step_block_t`, and keeps the software ring as
    * full as possible before starting / restarting the RMT stream.
     */
    static void executorTask(void* arg);
};
```

### src/esp32/src/step_types.h

```cpp
/**
 * @file step_types.h
 * @brief Shared data types for the Klipper-style stepper architecture.
 *
 * The host (Linux / Raspberry Pi) pre-computes all trajectories and sends
 * blocks of pre-timed step commands.  The ESP32 is a pure executor: it has
 * no knowledge of acceleration, kinematics, or wire geometry.
 *
 * ── RMT streaming architecture (FastAccelStepper-style) ────────────────────
 *   A simple_encoder callback streams step pulses from a lock-free ring buffer.
 *   The RMT hardware calls the encoder on-demand (ISR context), filling
 *   PART_SIZE symbols at a time in ping-pong fashion.  This eliminates the
 *   inter-block gaps that caused step loss with the old copy_encoder approach.
 *
 * ── RMT resolution analysis ────────────────────────────────────────────────
 *   Resolution : 80 MHz  (1 tick = 12.5 ns)
 *   Step shape : one RMT symbol per step
 *     HIGH = ticks / 2  (balanced pulse, FastAccelStepper-style)
 *     LOW  = ticks − HIGH
 *     Minimum: HIGH = LOW = RMT_STEP_PULSE_TICKS = 8 ticks = 100 ns each
 *
 *   Max step rate : 80 MHz / RMT_STEP_MIN_TICKS(16) = 5 000 000 steps/sec
 *   Max RPM       : 5 000 000 / (200 × 32) = 781 RPM  (at minimum ticks)
 *   160 kHz target: interval = 80 000 000 / 160 000 = 500 ticks  (6.25 µs)
 *   100 Hz  min   : interval = 800 000 ticks → clamped to 0xFFFF (65535)
 *
 *   PART_SIZE=16: one encoder callback per 16 steps.
 *     At 160 kHz: callback every 100 µs — well within FreeRTOS tick budget.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// RMT timing constants
// ---------------------------------------------------------------------------

/** RMT TX channel resolution: 80 MHz  (1 tick = 12.5 ns) */
#define RMT_STEP_RESOLUTION_HZ  80000000UL

/** Ticks per microsecond derived from RMT_STEP_RESOLUTION_HZ (80 at 80 MHz). */
#define RMT_TICKS_PER_US        (RMT_STEP_RESOLUTION_HZ / 1000000UL)

/** Minimum half-period guard in RMT ticks (100 ns — 8 × 12.5 ns at 80 MHz;
 *  meets DRV8825/A4988 STEP pulse width minimum of 1 µs when low half is added). */
#define RMT_STEP_PULSE_TICKS    8U

/**
 * Minimum total interval in ticks.
 * 16 ticks × 12.5 ns = 200 ns → 5 MHz step rate ceiling at hardware level.
 * Ensures duration1 = interval_ticks − PULSE_TICKS ≥ 8 ticks.
 */
#define RMT_STEP_MIN_TICKS      16U

/** Maximum interval in ticks: 16-bit RMT field → 65535 ticks = ~819 µs → ~1.2 kHz floor */
#define RMT_STEP_MAX_TICKS      0xFFFFU

/** Default hold interval before any step has been consumed (= minimum interval).
 *  Prevents duration1 wraparound to ~65535 ticks on first ring-empty hold. */
#define RMT_STEP_DEFAULT_TICKS  RMT_STEP_MIN_TICKS

// ---------------------------------------------------------------------------
// RMT streaming constants (FastAccelStepper-style ping-pong)
// ---------------------------------------------------------------------------

/** Symbols per ping-pong half-buffer.  Must divide RMT_MEM_SYMBOLS evenly.
 *  Hardware requires `mem_block_symbols` to be even and at least 64, so the
 *  minimum practical PART_SIZE is 32 (2 × PART_SIZE = 64 symbols per channel).
 *  PART_SIZE=32 gives one encoder callback per 32 steps. */
#define PART_SIZE               32U

/** Total RMT hardware memory per channel (2 × PART_SIZE for ping-pong).
 *  Must be >= 64 for IDF RMT driver constraints. */
#define RMT_MEM_SYMBOLS         (2U * PART_SIZE)

/**
 * Minimum command duration in ticks (200 µs at 2 MHz = 400 ticks).
 * Used as pause filler when the ring buffer empties or on direction changes.
 */
#define MIN_CMD_TICKS           400U

// ---------------------------------------------------------------------------
// Ring buffer sizing
// ---------------------------------------------------------------------------

/** Ring buffer size — must be a power of 2.
 *  4096 entries = 58ms at 70kHz (cruise speed, ~660 RPM).
 *  Must cover the worst-case host re-fill time: 25 segments × 1ms SPI = 25ms,
 *  plus OS jitter.  4096 gives 58ms >> 25ms, preventing ring starvation when
 *  all deferred notifications fire simultaneously. */
#define STEP_RING_SIZE          4096U

/** Bit mask for ring buffer index wrap-around. */
#define STEP_RING_MASK          (STEP_RING_SIZE - 1U)

// ---------------------------------------------------------------------------
// Block / queue sizing
// ---------------------------------------------------------------------------

/** Number of step commands per block. */
#define STEP_BLOCK_SIZE         64

/** Number of compressed motion segments per transport block. */
#define SEGMENT_BLOCK_SIZE      60

/** Buffered step target before starting/restarting the RMT stream.
 *  Must satisfy: STEP_STREAM_START_FILL >= 2 * PART_SIZE. For
 *  PART_SIZE=32 the minimum safe value is 64. */
#define STEP_STREAM_START_FILL  64U

/**
 * Minimum steps required to RESTART the RMT stream after an underrun.
 * Lower than STEP_STREAM_START_FILL: at low speed the ring drains faster
 * than the inter-segment gap, so we must restart with fewer steps buffered.
 * PART_SIZE/2 = 16 steps guarantees at least one half-callback of data.
 */
#define STEP_STREAM_RESTART_FILL  (PART_SIZE / 2U)   // = 16

/**
 * Depth of the FreeRTOS step-block queue (per motor).
 * Increased to 24 to handle slower host SPI latency and maintain RMT fill rate.
 * Provides ~(STEPPER_QUEUE_DEPTH × STEP_BLOCK_SIZE) steps of look-ahead.
 */
#define STEPPER_QUEUE_DEPTH     24

/**
 * Flow-control threshold: the comm layer sends a NACK / buffer-full warning
 * to the host when the number of free queue slots drops below this value.
 */
#define FLOW_CONTROL_THRESHOLD  4

/** Microstep denominator — override with -DMICROSTEPS=N in build flags. */
#ifndef MICROSTEPS
#define MICROSTEPS 32
#endif

// ---------------------------------------------------------------------------
// Core data types
// ---------------------------------------------------------------------------

/**
 * @brief A single pre-timed step command.
 *
 * @note  interval_ticks == 0  →  stop marker (end of move).
 * @note  All steps within one block MUST share the same direction.
 *        The host is responsible for splitting trajectories at direction
 *        reversals; crossing a reversal within a block is undefined behaviour.
 */
typedef struct {
    uint32_t interval_ticks; /**< RMT ticks until the NEXT step (0 = stop)          */
    bool     direction;      /**< true = forward / CW, false = reverse / CCW         */
} step_cmd_t;

/**
 * @brief A pre-computed block of step commands.
 *
 * The host fills steps[0..count-1] and sets count.  The executor iterates
 * only up to count; remaining entries are ignored.
 */
typedef struct {
    step_cmd_t steps[STEP_BLOCK_SIZE]; /**< Pre-timed step commands              */
    uint32_t   count;                  /**< Number of valid entries in steps[]   */
} step_block_t;

/**
 * @brief A compressed motion segment.
 *
 * Represents `step_count` successive steps where the interval evolves as:
 *
 *   ticks[n] = start_ticks + n * add_ticks
 *
 * This is the same basic representation used by Klipper-style trapezoid
 * segments: the host sends a compact arithmetic description, the MCU expands
 * it locally into concrete step timings.
 */
typedef struct {
    uint16_t step_count;   /**< Number of steps encoded by the segment          */
    uint16_t start_ticks;  /**< Interval for the first step in RMT ticks        */
    int16_t  add_ticks;    /**< Delta applied after each emitted step           */
    uint8_t  direction;    /**< true = forward / CW, false = reverse / CCW      */
    uint8_t  reserved;     /**< Padding / future flags                          */
} motion_segment_t;

/**
 * @brief A transport block of compressed motion segments.
 */
typedef struct {
    motion_segment_t segments[SEGMENT_BLOCK_SIZE];
    uint32_t         count;   /**< Number of valid segments[] entries             */
} segment_block_t;

typedef enum {
    MOTION_BLOCK_KIND_STEP = 0,
    MOTION_BLOCK_KIND_SEGMENT = 1,
} motion_block_kind_t;

/**
 * @brief Queue item exchanged between the comm task and the executor task.
 *
 * The comm task can enqueue either legacy per-step blocks or compressed
 * segment blocks. The executor expands segment blocks into `step_block_t`
 * chunks locally before feeding the driver ring.
 */
typedef struct {
    uint8_t kind;
    uint8_t reserved[3];
    union {
        step_block_t    step;
        segment_block_t segment;
    } payload;
} motion_block_t;

/**
 * @brief Ring buffer entry consumed by the RMT encoder callback (ISR context).
 *
 * Produced by the executor task, consumed by the simple_encoder callback.
 * SPSC: one writer (task), one reader (ISR).
 */
typedef struct {
    uint16_t ticks;      /**< Total step period in RMT ticks (2 MHz).  0 = invalid. */
    uint8_t  toggle_dir; /**< 1 = toggle DIR pin before this step.                   */
    uint8_t  pad;        /**< Padding for 4-byte alignment.                          */
} ring_entry_t;

/**
 * @brief Wire packet received from the host over UART/SPI.
 *
 * The comm layer deserialises this from the byte stream and routes it to
 * the appropriate motor queue based on motor_id.
 */
typedef struct {
    uint8_t    motor_id;               /**< Target motor: 0 or 1                 */
    uint8_t    block_seq;              /**< Rolling sequence number (loss detect) */
    uint32_t   count;                  /**< Number of valid steps in payload      */
    step_cmd_t steps[STEP_BLOCK_SIZE]; /**< Step payload                          */
} comm_packet_t;

// ---------------------------------------------------------------------------
// Multi-axis synchronised segment block (MULTI_AXIS_SEGMENT_BLOCK = 0x13)
// ---------------------------------------------------------------------------

/** Maximum number of axes in a multi-axis segment block. */
#define MULTI_AXIS_MAX_AXES  4

/** Maximum number of segments per multi-axis block. */
#define MULTI_AXIS_BLOCK_SIZE  60

/**
 * @brief One synchronised multi-axis time-based segment.
 *
 * The host sends one of these records per segment.  All axes execute their
 * respective step_counts over the shared duration_us, with steps distributed
 * evenly in time by the MCU.  direction_mask bit i = 1 means axis i reverses.
 */
typedef struct {
    uint16_t motion_sequence; /**< Globally increasing motion identifier       */
    uint16_t duration_us;     /**< Wall-clock duration of this segment in µs   */
    uint16_t direction_mask;  /**< Bit i=1: axis i runs in reverse direction   */
    uint16_t step_counts[MULTI_AXIS_MAX_AXES]; /**< Steps per axis             */
} multi_axis_segment_t;

/**
 * @brief Block of synchronised multi-axis segments received from the host.
 *
 * Enqueued into the global multi-axis queue and consumed by the executor.
 */
typedef struct {
    uint8_t              axis_ids[MULTI_AXIS_MAX_AXES]; /**< Logical axis IDs  */
    uint8_t              axis_count;                    /**< Valid entries      */
    uint8_t              segment_count;                 /**< Valid segments     */
    multi_axis_segment_t segments[MULTI_AXIS_BLOCK_SIZE];
} multi_axis_block_t;

/**
 * @brief Flush request: discard all segments with motion_sequence > threshold.
 */
typedef struct {
    uint16_t flush_sequence; /**< Keep segments ≤ this; discard the rest       */
} flush_request_t;

#ifdef __cplusplus
} /* extern "C" */
#endif
```

---

## 4. Vérification numérique

Hypothèses demandées:
- Vitesse: 1500 RPM
- Pas/rev: 6400
- Segment duration: $0.004\,s$

### 4.1 Correction 1 — `_current_steps_per_segment`

Calcul fréquence pas:
$$
Hz = \frac{1500}{60} \times 6400 = 160000\ \text{steps/s}
$$

Calcul pas par segment:
$$
steps/segment = 160000 \times 0.004 = 640
$$

- **Avant** (`__init__`): `_current_steps_per_segment = 0`
- **Après**: `_initial_steps_per_segment = _current_steps_per_segment = 640`

Impact direct: classification vitesse passe de « low speed » à « high speed » dès le démarrage.

### 4.2 Correction 2 — `_safe_buffer_time_s`

Capacité safe ring:
$$
safe\_time = \frac{STEP\_RING\_CAPACITY \times HEADROOM}{max\_hz}
= \frac{4096 \times 0.8}{160000}
= 0.02048\,s
$$

Avec `requested_time_s=0.10`:
- **Avant**: clamp entrée à `[0.06, 0.12]` => `0.10`, puis résultat
  $$max(0.06, min(0.10, 0.02048)) = 0.06\,s$$
- **Après**: calcul direct puis borne basse
  $$max(0.06, min(0.10, 0.02048)) = 0.06\,s$$

Résultat numérique inchangé dans ce cas précis, mais logique assainie (plus de plafond `MAX_BUFFER_TIME_S` appliqué avant le calcul de sécurité ring).

### 4.3 Correction 3 — `STEP_RING_CAPACITY`

- Firmware: `STEP_RING_SIZE = 4096` (confirmé dans `step_types.h`).
- Host: `STEP_RING_CAPACITY = 4096` (déjà cohérent).

Donc pas de changement numérique nécessaire sur cette base.

### 4.4 Correction 4 — prefill cible

Règle `required_lookahead()`:
- `<10` => 48
- `<50` => 32
- `>=50` => 16

- **Avant**: `_current_steps_per_segment=0` au démarrage => prefill forcé à 64.
- **Après**: prefill basé sur `initial_steps=640` => `required_lookahead(640)=16`.

Temps de prefill (4 ms/segment):
- Avant: $64 \times 0.004 = 0.256\,s$
- Après: $16 \times 0.004 = 0.064\,s$

### 4.5 Correction 5 — stall timeout et armement

- **Avant**: timeout `2.0s`, active même avant première confirmation.
- **Après**: timeout `5.0s`, et désactivé tant que `_last_confirmed_sequence < 0`.

Impact:
- Fenêtre de tolérance multipliée par $2.5$.
- Suppression des faux stalls pendant phase d’amorçage / notifications différées.

### 4.6 Correction 6 — priorité planner

- **Avant**: `PLANNER_PRIO = 12`, SPI task à 10.
- **Après**: `PLANNER_PRIO = 8` (< SPI).

Effet attendu: le SPI task garde la priorité sur ingestion/routage des blocs, limitant le risque de verrouillage de progression sous backpressure.

---

## 5. Points d'attention restants

1. **Exposition dynamique de la taille ring (demande 3.c) non implémentée**.
   - Le code reste sur constante host `STEP_RING_CAPACITY`.
   - Recommandation: ajouter `step_ring_size` dans `StatusPayload` firmware et l’utiliser au bootstrap host.

2. **Commentaires `_prefill()`**
   - Le docstring mentionne encore `required_lookahead(current_steps_per_segment)` alors que l’implémentation utilise désormais l’estimation initiale.

3. **Validation runtime terrain à refaire**
   - Vérifier logs `ring_free`, `underrun`, `inflight`, `planner_queue_free` à 250 RPM, 660 RPM, 1500 RPM.
   - Objectif: `underrun_count` stable à 0 durant `running=1`.

4. **`STEP_RING_SIZE=4096`** confirmé dans cette branche.
   - Si une autre branche/board profile modifie cette valeur, la synchronisation host/firmware devra être automatisée.
