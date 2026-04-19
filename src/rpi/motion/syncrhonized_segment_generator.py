from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

from .spindle_kinematics import SpindleKinematics
from .winding_pattern import WindingPattern
from .scatter_engine import ScatterEngine
from transport.messages import MultiAxisSegment


@dataclass(slots=True)
class SyncAxisConfig:
    axis_index: int
    steps_per_unit: float
    reverse_direction: bool = False


class SynchronizedSegmentGenerator:
    """Generates strictly synchronized motion blocks (Electronic Gearing) for winding."""
    def __init__(
        self,
        spindle_kinematics: SpindleKinematics,
        pattern: WindingPattern,
        scatter: ScatterEngine,
        spindle_config: SyncAxisConfig,
        traverse_config: SyncAxisConfig,
        segment_duration_s: float = 0.004,
        start_sequence: int = 0
    ):
        self.kinematics = spindle_kinematics
        self.pattern = pattern
        self.scatter = scatter
        self.spindle_config = spindle_config
        self.traverse_config = traverse_config
        self.segment_duration_s = max(0.002, min(0.005, segment_duration_s))
        self._sequence = start_sequence & 0xFFFF
        
        self.time_cursor = 0.0
        self.overall_duration = self.kinematics.total_duration
        
        self.current_spindle_steps = 0
        self.current_traverse_steps = 0

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        while self.time_cursor < self.overall_duration:
            next_cursor = min(self.time_cursor + self.segment_duration_s, self.overall_duration)
            duration_s = next_cursor - self.time_cursor
            duration_us = int(round(duration_s * 1_000_000))

            target_turns = self.kinematics.turns_at(next_cursor)
            target_spindle_steps = int(round(target_turns * self.spindle_config.steps_per_unit))
            delta_spindle_steps = target_spindle_steps - self.current_spindle_steps
            
            base_traverse_mm = self.pattern.guide_pos_mm(target_turns)
            scatter_offset = self.scatter.get_offset(target_turns, base_traverse_mm, self.pattern.bobbin_width_mm)
            target_traverse_mm = base_traverse_mm + scatter_offset
            
            target_traverse_steps = int(round(target_traverse_mm * self.traverse_config.steps_per_unit))
            delta_traverse_steps = target_traverse_steps - self.current_traverse_steps
            
            spindle_dir = 1 if (delta_spindle_steps < 0) ^ self.spindle_config.reverse_direction else 0
            traverse_dir = 1 if (delta_traverse_steps < 0) ^ self.traverse_config.reverse_direction else 0
            
            self.current_spindle_steps = target_spindle_steps
            self.current_traverse_steps = target_traverse_steps

            max_axis = max(self.spindle_config.axis_index, self.traverse_config.axis_index)
            steps = [0] * (max_axis + 1)
            directions = [0] * (max_axis + 1)
            
            steps[self.spindle_config.axis_index] = abs(delta_spindle_steps)
            directions[self.spindle_config.axis_index] = spindle_dir
            
            steps[self.traverse_config.axis_index] = abs(delta_traverse_steps)
            directions[self.traverse_config.axis_index] = traverse_dir

            yield MultiAxisSegment(
                sequence=self._sequence,
                duration_us=duration_us,
                steps=steps,
                directions=directions,
            )
            self._sequence = (self._sequence + 1) & 0xFFFF
            self.time_cursor = next_cursor
