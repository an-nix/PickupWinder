from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Iterator

from transport.messages import MultiAxisSegment


@dataclass(slots=True)
class AxisStepProfile:
    axis_index: int
    step_at: Callable[[float], float]
    reverse_direction: bool = False
    total_duration: float = 0.0


class BaseSegmentGenerator(ABC):
    """Common iteration behavior for multi-axis segment generators.

    Supports adaptive segment duration: when the estimated step rate is low,
    the segment duration is stretched so each segment contains at least
    MIN_STEPS_PER_SEGMENT steps (matching firmware PART_SIZE).  This prevents
    the RMT ring from draining between tiny segments at low RPM.
    """

    # Minimum steps per segment to keep the RMT ring well-fed.
    # Must be >= firmware PART_SIZE (currently 8).
    MIN_STEPS_PER_SEGMENT = 32
    MIN_DURATION_S = 0.002
    MAX_DURATION_S = 0.050

    def __init__(self, *, segment_duration_s: float = 0.004, start_sequence: int = 0) -> None:
        self._base_segment_duration_s = max(self.MIN_DURATION_S, min(self.MAX_DURATION_S, segment_duration_s))
        self.segment_duration_s = self._base_segment_duration_s
        self._sequence = start_sequence & 0xFFFF
        self._time_cursor = 0.0
        self.overall_duration = 0.0

    def _adaptive_duration(self, estimated_step_rate: float) -> float:
        """Compute segment duration ensuring at least MIN_STEPS_PER_SEGMENT steps.

        Args:
            estimated_step_rate: Current step rate in steps/s across all axes.
                If <= 0, falls back to _base_segment_duration_s.

        Returns:
            Duration in seconds, clamped to [MIN_DURATION_S, MAX_DURATION_S].
        """
        if estimated_step_rate <= 0.0:
            return self._base_segment_duration_s
        min_duration = self.MIN_STEPS_PER_SEGMENT / estimated_step_rate
        return max(self.MIN_DURATION_S, min(self.MAX_DURATION_S, max(min_duration, self._base_segment_duration_s)))

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        while self._time_cursor < self.overall_duration:
            next_cursor = min(self._time_cursor + self.segment_duration_s, self.overall_duration)
            duration_s = next_cursor - self._time_cursor
            duration_us = int(round(duration_s * 1_000_000))

            steps, direction_mask = self._compute_segment(self._time_cursor, next_cursor)

            yield MultiAxisSegment(
                sequence=self._sequence,
                duration_us=duration_us,
                steps=steps,
                direction_mask=direction_mask,
            )
            self._sequence = (self._sequence + 1) & 0xFFFF
            self._time_cursor = next_cursor

            # Adapt segment duration for the next iteration based on observed step rate.
            total_steps = sum(steps)
            if duration_s > 0.0 and total_steps > 0:
                estimated_rate = total_steps / duration_s
                self.segment_duration_s = self._adaptive_duration(estimated_rate)

    @abstractmethod
    def _compute_segment(self, time_start: float, time_end: float) -> tuple[list[int], int]:
        """Return the next segment payload for the current time window."""
        ...


class StepProfileSegmentGenerator(BaseSegmentGenerator):
    def __init__(self, axis_profiles: list[AxisStepProfile], *, segment_duration_s: float = 0.004, start_sequence: int = 0) -> None:
        super().__init__(segment_duration_s=segment_duration_s, start_sequence=start_sequence)
        self.axis_profiles = axis_profiles
        self._axis_errors = [0.0 for _ in axis_profiles]
        self._current_steps = [profile.step_at(0.0) for profile in axis_profiles]
        self.overall_duration = max((profile.total_duration for profile in axis_profiles), default=0.0)

    def _compute_segment(self, time_start: float, time_end: float) -> tuple[list[int], int]:
        steps = [0] * len(self.axis_profiles)
        direction_mask = 0

        for index, profile in enumerate(self.axis_profiles):
            target_steps = profile.step_at(time_end)
            delta_steps = target_steps - self._current_steps[index]
            count = int(round(self._axis_errors[index] + delta_steps))
            self._axis_errors[index] += delta_steps - float(count)
            self._current_steps[index] = target_steps

            is_negative = count < 0
            if is_negative:
                count = abs(count)
            if is_negative ^ profile.reverse_direction:
                direction_mask |= (1 << index)
            steps[index] = count

        return steps, direction_mask
