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
