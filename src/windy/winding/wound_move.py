from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Iterator

from motion import SpindleKinematics
from motion.move import Move
from winding.winding_pattern import WindingPattern
from winding.scatter_engine import ScatterEngine
from winding.synchronized_segment_generator import SyncAxisConfig, SynchronizedSegmentGenerator
from transport.messages import MultiAxisSegment


class SynchronizedMove(Move, ABC):
    """Abstract base for moves that stream spindle+traverse in lock-step.

    Both ``WoundMove`` and ``AdaptiveWindingMove`` satisfy this contract.
    ``MoveQueue`` dispatches on ``isinstance(move, SynchronizedMove)`` so
    that neither subclass needs duck-type markers.

    Concrete subclasses must expose ``kinematics``, ``spindle_cfg``, and
    ``segment_duration_s`` as instance attributes (assigned in ``__init__``).
    These are declared as abstract properties so that mypy enforces the
    contract statically on every subclass.
    """

    @property
    @abstractmethod
    def kinematics(self) -> SpindleKinematics: ...

    @property
    @abstractmethod
    def spindle_cfg(self) -> SyncAxisConfig: ...

    @property
    @abstractmethod
    def segment_duration_s(self) -> float: ...


class WoundMove(SynchronizedMove):
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

        self._kinematics = kinematics
        self.pattern = pattern
        self.scatter = scatter
        self._spindle_cfg = spindle_cfg
        self.traverse_cfg = traverse_cfg
        self._segment_duration_s = segment_duration_s

    @property
    def kinematics(self) -> SpindleKinematics:
        return self._kinematics

    @property
    def spindle_cfg(self) -> SyncAxisConfig:
        return self._spindle_cfg

    @property
    def segment_duration_s(self) -> float:
        return self._segment_duration_s

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


def build_wound_move(
    name: str,
    *,
    spindle_rpm: float,
    accel_s: float,
    cruise_s: float,
    decel_s: float,
    bobbin_width_mm: float,
    turns_per_mm: float,
    scatter_amplitude_mm: float = 0.0,
    scatter_damping_margin_mm: float = 0.0,
    scatter_freq1: float = 1.0,
    scatter_freq2: float = 1.618,
    spindle_axis_id: int = 0,
    spindle_steps_per_rev: float,
    spindle_reverse: bool = False,
    lateral_axis_id: int = 1,
    lateral_steps_per_mm: float,
    lateral_reverse: bool = False,
) -> WoundMove:
    """Centralised factory for synchronised winding moves (Electronic Gearing).

    Both ``WindingEngine._run_layer()`` and ``MotionCommandService.wound_run()``
    delegate to this function so that ``WoundMove`` construction stays DRY and
    ``lateral_steps_per_mm`` is always sourced from ``AppConfiguration``.
    """
    return WoundMove(
        name=name,
        kinematics=SpindleKinematics(
            target_rpm=spindle_rpm,
            start_rpm=0.0,
            accel_s=accel_s,
            cruise_s=cruise_s,
            decel_s=decel_s,
        ),
        pattern=WindingPattern(
            bobbin_width_mm=bobbin_width_mm,
            turns_per_mm=turns_per_mm,
        ),
        scatter=ScatterEngine(
            amplitude_mm=scatter_amplitude_mm,
            freq1=scatter_freq1,
            freq2=scatter_freq2,
            damping_margin_mm=scatter_damping_margin_mm,
        ),
        spindle_cfg=SyncAxisConfig(
            axis_index=spindle_axis_id,
            steps_per_unit=spindle_steps_per_rev,
            reverse_direction=spindle_reverse,
        ),
        traverse_cfg=SyncAxisConfig(
            axis_index=lateral_axis_id,
            steps_per_unit=lateral_steps_per_mm,
            reverse_direction=lateral_reverse,
        ),
    )
