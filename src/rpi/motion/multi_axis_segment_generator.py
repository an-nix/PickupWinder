from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, List, Union

from .ramp_config import RampConfig
from transport.messages import (
        SEGMENT_BLOCK_SIZE,
        STEP_BLOCK_SIZE,
        MotionSegment,
        MultiAxisSegment,
        SegmentBlockPayload,
        StepBlockPayload,
        StepEntry,
    )



@dataclass(slots=True)
class AxisMotionConfig:
    axis_id: int
    ramp: RampConfig


class MultiAxisSegmentGenerator:
    def __init__(self, axis_configs: list[AxisMotionConfig], *, segment_duration_s: float = 0.004, start_sequence: int = 0):
        self.axis_configs = axis_configs
        self.segment_duration_s = max(0.002, min(0.005, segment_duration_s))
        self._sequence = start_sequence & 0xFFFF
        self._axis_errors = [0.0 for _ in axis_configs]
        self._axis_total_durations = [config.ramp.total_duration for config in axis_configs]
        self._overall_duration = max(self._axis_total_durations) if axis_configs else 0.0

    def _hz_at_time(self, config: RampConfig, t: float) -> float:
        if t < 0.0 or t >= config.total_duration:
            return 0.0

        if t < config.accel_s:
            accel_rate = 0.0 if config.accel_s <= 0.0 else (config.target_hz - config.start_hz) / config.accel_s
            return config.start_hz + accel_rate * t

        cruise_end = config.accel_s + config.cruise_s
        if t < cruise_end:
            return config.target_hz

        decel_t = t - cruise_end
        decel_rate = 0.0 if config.decel_s <= 0.0 else (config.target_hz - config.start_hz) / config.decel_s
        return max(config.target_hz - decel_rate * decel_t, 0.0)

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        time_cursor = 0.0
        while time_cursor < self._overall_duration:
            next_cursor = min(time_cursor + self.segment_duration_s, self._overall_duration)
            duration_s = next_cursor - time_cursor
            duration_us = int(round(duration_s * 1_000_000))

            steps: list[int] = []
            directions: list[int] = []
            for index, config in enumerate(self.axis_configs):
                hz0 = self._hz_at_time(config.ramp, time_cursor)
                hz1 = self._hz_at_time(config.ramp, next_cursor)
                delta_steps = ((hz0 + hz1) * 0.5) * duration_s
                count = int(round(self._axis_errors[index] + delta_steps))
                self._axis_errors[index] += delta_steps - float(count)
                if count < 0:
                    count = 0
                steps.append(count)
                directions.append(1 if config.ramp.reverse_direction else 0)

            yield MultiAxisSegment(
                sequence=self._sequence,
                duration_us=duration_us,
                steps=steps,
                directions=directions,
            )
            self._sequence = (self._sequence + 1) & 0xFFFF
            time_cursor = next_cursor
