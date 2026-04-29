from __future__ import annotations

from typing import Iterator

from motion.move import Move
from motion import SpindleKinematics
from winding.winding_pattern import WindingPattern
from winding.scatter_engine import ScatterEngine
from winding.synchronized_segment_generator import SyncAxisConfig, SynchronizedSegmentGenerator
from transport.messages import MultiAxisSegment


class WoundMove(Move):
    """
    A single continuous move executing the Electronic Gearing winding pattern.
    """

    def __init__(
        self,
        name: str,
        kinematics: SpindleKinematics,
        pattern: WindingPattern,
        scatter: ScatterEngine,
        spindle_cfg: SyncAxisConfig,
        traverse_cfg: SyncAxisConfig,
        segment_duration_s: float = 0.004,
    ) -> None:
        super().__init__(name)
        if spindle_cfg.axis_index == traverse_cfg.axis_index:
            raise ValueError("spindle_cfg.axis_index and traverse_cfg.axis_index must differ")
        if segment_duration_s <= 0.0:
            raise ValueError("segment_duration_s must be positive")
        if kinematics.total_duration <= 0.0:
            raise ValueError("kinematics.total_duration must be positive")

        self.kinematics = kinematics
        self.pattern = pattern
        self.scatter = scatter
        self.spindle_cfg = spindle_cfg
        self.traverse_cfg = traverse_cfg
        self.segment_duration_s = segment_duration_s

    def segments(self) -> Iterator[MultiAxisSegment]:
        gen = SynchronizedSegmentGenerator(
            self.kinematics,
            self.pattern,
            self.scatter,
            self.spindle_cfg,
            self.traverse_cfg,
            segment_duration_s=self.segment_duration_s,
        )
        yield from gen

    def expected_delta_steps(self, axis_id: int) -> int | None:
        return None

    @property
    def axis_ids(self) -> list[int]:
        return [self.spindle_cfg.axis_index, self.traverse_cfg.axis_index]
