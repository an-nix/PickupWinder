from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

from .segment_generator import AxisStepProfile, StepProfileSegmentGenerator
from .spindle_kinematics import SpindleKinematics
from .winding_pattern import WindingPattern
from .scatter_engine import ScatterEngine
from transport.messages import MultiAxisSegment


@dataclass(slots=True)
class SyncAxisConfig:
    axis_index: int
    steps_per_unit: float
    reverse_direction: bool = False


class SynchronizedSegmentGenerator(StepProfileSegmentGenerator):
    """Generates strictly synchronized motion blocks (Electronic Gearing) for winding."""

    def __init__(
        self,
        spindle_kinematics: SpindleKinematics,
        pattern: WindingPattern,
        scatter: ScatterEngine,
        spindle_config: SyncAxisConfig,
        traverse_config: SyncAxisConfig,
        segment_duration_s: float = 0.004,
        start_sequence: int = 0,
    ):
        self.kinematics = spindle_kinematics
        self.pattern = pattern
        self.scatter = scatter
        self.spindle_config = spindle_config
        self.traverse_config = traverse_config

        def spindle_steps_at(t: float) -> float:
            return spindle_kinematics.turns_at(t) * spindle_config.steps_per_unit

        def traverse_steps_at(t: float) -> float:
            turns = spindle_kinematics.turns_at(t)
            base_traverse_mm = pattern.guide_pos_mm(turns)
            scatter_offset = scatter.get_offset(
                turns,
                base_traverse_mm,
                pattern.bobbin_width_mm,
            )
            return (base_traverse_mm + scatter_offset) * traverse_config.steps_per_unit

        axis_profiles = [
            AxisStepProfile(
                axis_index=spindle_config.axis_index,
                step_at=spindle_steps_at,
                reverse_direction=spindle_config.reverse_direction,
                total_duration=spindle_kinematics.total_duration,
            ),
            AxisStepProfile(
                axis_index=traverse_config.axis_index,
                step_at=traverse_steps_at,
                reverse_direction=traverse_config.reverse_direction,
                total_duration=spindle_kinematics.total_duration,
            ),
        ]

        super().__init__(axis_profiles, segment_duration_s=segment_duration_s, start_sequence=start_sequence)
