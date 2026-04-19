from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

from .ramp_config import RampConfig
from .segment_generator import AxisStepProfile, StepProfileSegmentGenerator
from transport.messages import MultiAxisSegment



@dataclass(slots=True)
class AxisMotionConfig:
    axis_id: int
    ramp: RampConfig


class MultiAxisSegmentGenerator(StepProfileSegmentGenerator):
    def __init__(
        self,
        axis_configs: list[AxisMotionConfig],
        *,
        segment_duration_s: float = 0.004,
        start_sequence: int = 0,
    ):
        self.axis_configs = axis_configs

        axis_profiles = [
            AxisStepProfile(
                axis_index=config.axis_id,
                step_at=lambda t, ramp=config.ramp: ramp.steps_at(t),
                reverse_direction=config.ramp.reverse_direction,
                total_duration=config.ramp.total_duration,
            )
            for config in axis_configs
        ]

        super().__init__(axis_profiles, segment_duration_s=segment_duration_s, start_sequence=start_sequence)
