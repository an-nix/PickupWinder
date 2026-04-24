from __future__ import annotations

from dataclasses import dataclass
import logging
import time
from typing import Protocol

from transport.messages import (
    LATERAL_ENDSTOP_ABSENT,
    LATERAL_ENDSTOP_PRESENT_CLOSED,
    LATERAL_ENDSTOP_PRESENT_OPEN,
    SpiMessageResult,
)


logger = logging.getLogger(__name__)


MIN_QUEUE_FREE = 8
_MAX_BATCH_SEGMENTS = 16
_INVALID_STATE_MAX_POLLS = 3
_ABSENT_CONFIRM_POLLS = 5


def sequence_signed_distance_u16(a: int, b: int) -> int:
    """Return signed 16-bit distance from *b* to *a*."""
    return ((a - b + 0x8000) & 0xFFFF) - 0x8000


def sequence_is_newer_u16(a: int, b: int) -> bool:
    """Wrap-aware 16-bit comparison: True when *a* is newer than *b*."""
    return sequence_signed_distance_u16(a, b) > 0


def sequence_is_newer_or_equal_u16(a: int, b: int) -> bool:
    return sequence_signed_distance_u16(a, b) >= 0


@dataclass(slots=True)
class SegmentSpec:
    motion_sequence: int
    duration_us: int
    step_counts: list[int]
    direction_mask: int


class HomingTimeoutError(TimeoutError):
    """Raised when a homing phase exceeds its allowed timeout."""


class HomingFaultError(RuntimeError):
    """Raised when homing cannot continue due to protocol/sensor faults."""


class SpiInterfaceProtocol(Protocol):
    def send_enable_endstop(self, axis_id: int, arm: int) -> object:
        ...

    def send_flush(self, flush_sequence: int) -> object:
        ...

    def send_segments(self, segments: list[SegmentSpec]) -> object:
        ...

    def get_status(self) -> object:
        ...

    def send_estop(self, axis_id: int) -> object:
        ...


@dataclass(slots=True)
class _SeekPhaseResult:
    hit_detected: bool
    hit_sequence: int | None
    last_sent_sequence: int | None


class HomingController:
    def __init__(self, spi_interface: SpiInterfaceProtocol, axis_id: int = 1):
        self._spi = spi_interface
        self._axis_id = int(axis_id)
        self._motion_sequence = 0
        self._state = "IDLE"
        self._consecutive_absent_polls = 0

    def home(
        self,
        fast_speed_steps_per_seg: int,
        slow_speed_steps_per_seg: int,
        backoff_distance_steps: int,
        segment_duration_us: int = 4000,
        timeout_s: float = 10.0,
    ) -> int:
        """Returns the motion_sequence at which the slow hit was confirmed."""
        if fast_speed_steps_per_seg <= 0:
            raise ValueError("fast_speed_steps_per_seg must be > 0")
        if slow_speed_steps_per_seg <= 0:
            raise ValueError("slow_speed_steps_per_seg must be > 0")
        if backoff_distance_steps <= 0:
            raise ValueError("backoff_distance_steps must be > 0")
        if segment_duration_us <= 0:
            raise ValueError("segment_duration_us must be > 0")

        failed = True
        try:
            self._set_state("PRE_CHECK")
            pre_status = self._poll_status(timeout_s=timeout_s)
            self._validate_endstop_present(pre_status)
            self._initialize_motion_sequence(pre_status)

            self._set_state("ARM_FAST_SEEK")
            self._send_enable_endstop(arm=1, timeout_s=timeout_s)

            self._set_state("FAST_SEEK")
            fast_result = self._run_seek_phase(
                steps_per_segment=fast_speed_steps_per_seg,
                toward_switch=True,
                segment_duration_us=segment_duration_us,
                timeout_s=timeout_s,
            )

            self._set_state("WAIT_HIT_SETTLE")
            self._wait_for_closed(timeout_s=timeout_s)

            if fast_result.last_sent_sequence is None:
                raise HomingFaultError("FAST_SEEK sent no segments")

            self._set_state("SEND_FLUSH")
            self._send_flush_and_wait_recovery(
                flush_sequence=fast_result.last_sent_sequence,
                timeout_s=timeout_s,
            )

            self._set_state("REARM")
            self._send_enable_endstop(arm=1, timeout_s=timeout_s)

            self._set_state("BACKOFF")
            self._run_backoff_phase(
                distance_steps=backoff_distance_steps,
                steps_per_segment=fast_speed_steps_per_seg,
                segment_duration_us=segment_duration_us,
                timeout_s=timeout_s,
            )

            self._set_state("ARM_SLOW_SEEK")
            self._send_enable_endstop(arm=1, timeout_s=timeout_s)

            self._set_state("SLOW_SEEK")
            slow_result = self._run_seek_phase(
                steps_per_segment=slow_speed_steps_per_seg,
                toward_switch=True,
                segment_duration_us=segment_duration_us,
                timeout_s=timeout_s,
            )

            self._set_state("WAIT_SLOW_HIT_SETTLE")
            self._wait_for_closed(timeout_s=timeout_s)

            if slow_result.hit_sequence is None:
                raise HomingFaultError("SLOW_SEEK did not confirm a hit sequence")

            self._set_state("DISARM")
            self._send_enable_endstop(arm=0, timeout_s=timeout_s)

            self._set_state("DONE")
            failed = False
            return slow_result.hit_sequence
        finally:
            if failed:
                try:
                    self.cleanup(timeout_s=timeout_s)
                except Exception:
                    logger.exception("homing cleanup failed")

    def cleanup(self, timeout_s: float = 2.0) -> None:
        """Best-effort emergency cleanup: ESTOP then endstop disarm."""
        status = self._spi.send_estop(self._axis_id)
        if int(getattr(status, "last_result", int(SpiMessageResult.OK))) != int(SpiMessageResult.OK):
            raise HomingFaultError(
                f"ESTOP failed with result=0x{int(getattr(status, 'last_result', 0xFF)):02X}"
            )
        self._send_enable_endstop(arm=0, timeout_s=timeout_s)

    def _set_state(self, state: str) -> None:
        self._state = state
        logger.info("homing state -> %s", state)

    def _next_motion_sequence(self) -> int:
        seq = self._motion_sequence & 0xFFFF
        self._motion_sequence = (self._motion_sequence + 1) & 0xFFFF
        return seq

    def _initialize_motion_sequence(self, status: object) -> None:
        last_executed = int(getattr(status, "last_executed_sequence", 0xFFFF)) & 0xFFFF
        self._motion_sequence = (last_executed + 1) & 0xFFFF

    def _poll_status(self, *, timeout_s: float) -> object:
        _ = timeout_s
        status = self._spi.get_status()
        logger.debug(
            "status poll: state=%s lateral=0x%02X hit_mask=0x%02X armed_mask=0x%02X last_exec=%u planner_free=%u",
            self._state,
            int(getattr(status, "lateral_endstop_state", 0xFF)),
            int(getattr(status, "endstop_hit_mask", 0)),
            int(getattr(status, "endstop_armed_mask", 0)),
            int(getattr(status, "last_executed_sequence", 0)) & 0xFFFF,
            int(getattr(status, "planner_queue_free", 0)),
        )
        return status

    def _validate_endstop_present(self, status: object) -> None:
        state = int(getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT))
        if self._update_absent_counter(state):
            raise HomingFaultError("lateral endstop is ABSENT")

    def _send_enable_endstop(self, *, arm: int, timeout_s: float) -> object:
        status = self._spi.send_enable_endstop(self._axis_id, int(arm))
        return self._wait_result_ok(status, timeout_s=timeout_s, action=f"ENABLE_ENDSTOP(arm={arm})")

    def _send_flush_and_wait_recovery(self, *, flush_sequence: int, timeout_s: float) -> object:
        status = self._spi.send_flush(flush_sequence)
        status = self._wait_result_ok(status, timeout_s=timeout_s, action="FLUSH")
        start = time.monotonic()
        while True:
            if int(getattr(status, "planner_queue_free", 0)) > 0:
                return status
            if time.monotonic() - start > timeout_s:
                raise HomingTimeoutError("timeout waiting planner_queue_free recovery after FLUSH")
            status = self._poll_status(timeout_s=timeout_s)

    def _wait_result_ok(self, status: object, *, timeout_s: float, action: str) -> object:
        start = time.monotonic()
        current = status
        while True:
            result = int(getattr(current, "last_result", int(SpiMessageResult.OK)))
            if result == int(SpiMessageResult.OK):
                return current
            if time.monotonic() - start > timeout_s:
                raise HomingTimeoutError(f"timeout waiting ACK OK for {action}")
            current = self._poll_status(timeout_s=timeout_s)

    def _run_seek_phase(
        self,
        *,
        steps_per_segment: int,
        toward_switch: bool,
        segment_duration_us: int,
        timeout_s: float,
    ) -> _SeekPhaseResult:
        start = time.monotonic()
        hit_detected = False
        hit_sequence: int | None = None
        last_sent_sequence: int | None = None
        invalid_state_polls = 0

        while True:
            if time.monotonic() - start > timeout_s:
                raise HomingTimeoutError(f"timeout in {self._state}")

            status = self._poll_status(timeout_s=timeout_s)
            hit_mask = int(getattr(status, "endstop_hit_mask", 0))
            lateral_state = int(getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT))
            last_executed = int(getattr(status, "last_executed_sequence", 0)) & 0xFFFF
            mask_hit = bool(hit_mask & (1 << self._axis_id))
            state_hit = lateral_state == LATERAL_ENDSTOP_PRESENT_CLOSED

            if mask_hit != state_hit:
                logger.warning(
                    "endstop disagreement: hit_mask=%s lateral_state=0x%02X",
                    mask_hit,
                    lateral_state,
                )

            if mask_hit or state_hit:
                hit_detected = True
                if hit_sequence is None:
                    hit_sequence = last_executed

            if lateral_state not in (
                LATERAL_ENDSTOP_PRESENT_OPEN,
                LATERAL_ENDSTOP_PRESENT_CLOSED,
                LATERAL_ENDSTOP_ABSENT,
            ):
                invalid_state_polls += 1
                if invalid_state_polls >= _INVALID_STATE_MAX_POLLS:
                    raise HomingFaultError(
                        f"persistent invalid lateral_endstop_state=0x{lateral_state:02X}"
                    )
            else:
                invalid_state_polls = 0

            if self._update_absent_counter(lateral_state):
                raise HomingFaultError("lateral endstop became ABSENT while homing")

            if last_sent_sequence is not None and sequence_is_newer_or_equal_u16(last_executed, last_sent_sequence):
                if hit_detected:
                    return _SeekPhaseResult(
                        hit_detected=True,
                        hit_sequence=hit_sequence,
                        last_sent_sequence=last_sent_sequence,
                    )
                return _SeekPhaseResult(
                    hit_detected=False,
                    hit_sequence=None,
                    last_sent_sequence=last_sent_sequence,
                )

            if hit_detected:
                # Stop sending immediately once a hit is observed.
                continue

            planner_free = int(getattr(status, "planner_queue_free", 0))
            if planner_free < MIN_QUEUE_FREE:
                continue

            batch_len = min(planner_free, _MAX_BATCH_SEGMENTS)
            batch: list[SegmentSpec] = []
            for _ in range(batch_len):
                seq = self._next_motion_sequence()
                batch.append(
                    SegmentSpec(
                        motion_sequence=seq,
                        duration_us=segment_duration_us,
                        step_counts=self._make_step_counts(steps_per_segment),
                        direction_mask=self._direction_mask_for_phase(toward_switch=toward_switch),
                    )
                )
                last_sent_sequence = seq

            send_status = self._spi.send_segments(batch)
            self._wait_result_ok(send_status, timeout_s=timeout_s, action="MULTI_AXIS_SEGMENT_BLOCK")

    def _run_backoff_phase(
        self,
        *,
        distance_steps: int,
        steps_per_segment: int,
        segment_duration_us: int,
        timeout_s: float,
    ) -> None:
        start = time.monotonic()
        sent_steps = 0
        last_sent_sequence: int | None = None

        while sent_steps < distance_steps:
            if time.monotonic() - start > timeout_s:
                raise HomingTimeoutError("timeout while sending BACKOFF segments")
            status = self._poll_status(timeout_s=timeout_s)
            lateral_state = int(getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT))
            if self._update_absent_counter(lateral_state):
                raise HomingFaultError("lateral endstop became ABSENT during BACKOFF")

            planner_free = int(getattr(status, "planner_queue_free", 0))
            if planner_free < MIN_QUEUE_FREE:
                continue

            remaining_steps = distance_steps - sent_steps
            max_segments_by_distance = (remaining_steps + steps_per_segment - 1) // steps_per_segment
            batch_len = min(planner_free, _MAX_BATCH_SEGMENTS, max_segments_by_distance)
            batch: list[SegmentSpec] = []
            for _ in range(batch_len):
                seg_steps = min(steps_per_segment, distance_steps - sent_steps)
                seq = self._next_motion_sequence()
                batch.append(
                    SegmentSpec(
                        motion_sequence=seq,
                        duration_us=segment_duration_us,
                        step_counts=self._make_step_counts(seg_steps),
                        direction_mask=self._direction_mask_for_phase(toward_switch=False),
                    )
                )
                sent_steps += seg_steps
                last_sent_sequence = seq
                if sent_steps >= distance_steps:
                    break

            if batch:
                send_status = self._spi.send_segments(batch)
                self._wait_result_ok(send_status, timeout_s=timeout_s, action="MULTI_AXIS_SEGMENT_BLOCK")

        if last_sent_sequence is None:
            return

        while True:
            if time.monotonic() - start > timeout_s:
                raise HomingTimeoutError("timeout waiting BACKOFF execution catch-up")
            status = self._poll_status(timeout_s=timeout_s)
            if sequence_is_newer_or_equal_u16(
                int(getattr(status, "last_executed_sequence", 0)) & 0xFFFF,
                last_sent_sequence,
            ):
                return

    def _wait_for_closed(self, *, timeout_s: float) -> None:
        start = time.monotonic()
        invalid_state_polls = 0
        while True:
            if time.monotonic() - start > timeout_s:
                raise HomingTimeoutError(f"timeout waiting for CLOSED in state {self._state}")
            status = self._poll_status(timeout_s=timeout_s)
            lateral_state = int(getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT))

            if lateral_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
                self._consecutive_absent_polls = 0
                return
            if self._update_absent_counter(lateral_state):
                raise HomingFaultError("lateral endstop became ABSENT while waiting for CLOSED")
            if lateral_state not in (LATERAL_ENDSTOP_PRESENT_OPEN, LATERAL_ENDSTOP_PRESENT_CLOSED):
                invalid_state_polls += 1
                if invalid_state_polls >= _INVALID_STATE_MAX_POLLS:
                    raise HomingFaultError(
                        f"persistent invalid lateral_endstop_state=0x{lateral_state:02X}"
                    )
            else:
                invalid_state_polls = 0

    def _update_absent_counter(self, lateral_state: int) -> bool:
        if lateral_state == LATERAL_ENDSTOP_ABSENT:
            self._consecutive_absent_polls += 1
        else:
            self._consecutive_absent_polls = 0
        return self._consecutive_absent_polls >= _ABSENT_CONFIRM_POLLS

    def _make_step_counts(self, axis_steps: int) -> list[int]:
        counts = [0] * (self._axis_id + 1)
        counts[self._axis_id] = int(axis_steps)
        return counts

    def _direction_mask_for_phase(self, *, toward_switch: bool) -> int:
        if toward_switch:
            return 0
        return 1 << self._axis_id
