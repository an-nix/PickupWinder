from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Iterator

from motion.ramp import AxisMotionConfig, MultiAxisSegmentGenerator, RampConfig
from transport.messages import MultiAxisSegment


class MoveState(Enum):
    PENDING = auto()
    RUNNING = auto()
    COMPLETED = auto()
    ABORTED = auto()
    FAILED = auto()


class Move(ABC):
    """
    Abstract base class for all motion commands.

    A Move knows how to produce segments (via segments()) and reports
    its outcome via state and result. The MoveQueue executes moves one
    at a time by calling segments() and feeding them to the
    StreamingEngine.

    Subclasses must implement:
      - segments(): yields MultiAxisSegment objects
      - expected_delta_steps(axis_id): returns expected step delta for
        position tracking (return None if unknown / variable)
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self._state = MoveState.PENDING
        self._started_at: float | None = None
        self._completed_at: float | None = None
        self._error: str | None = None
        self._aborted_by_endstop: bool = False

    @property
    def state(self) -> MoveState:
        return self._state

    @property
    def done(self) -> bool:
        return self._state in (MoveState.COMPLETED, MoveState.ABORTED, MoveState.FAILED)

    @property
    def aborted_by_endstop(self) -> bool:
        return self._aborted_by_endstop

    def mark_running(self) -> None:
        self._state = MoveState.RUNNING
        self._started_at = time.monotonic()

    def mark_completed(self) -> None:
        self._state = MoveState.COMPLETED
        self._completed_at = time.monotonic()

    def mark_aborted(self, reason: str, by_endstop: bool = False) -> None:
        self._state = MoveState.ABORTED
        self._completed_at = time.monotonic()
        self._error = reason
        self._aborted_by_endstop = by_endstop

    def mark_failed(self, error: str) -> None:
        self._state = MoveState.FAILED
        self._completed_at = time.monotonic()
        self._error = error

    def snapshot(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "type": type(self).__name__,
            "state": self._state.name,
            "started_at": self._started_at,
            "completed_at": self._completed_at,
            "error": self._error,
            "aborted_by_endstop": self._aborted_by_endstop,
        }

    @abstractmethod
    def segments(self) -> Iterator[MultiAxisSegment]:
        """Yield segments to be sent to the firmware."""
        ...

    @abstractmethod
    def expected_delta_steps(self, axis_id: int) -> int | None:
        """
        Expected step delta for axis_id after this move completes.
        Return None if not applicable or variable (e.g. homing moves).
        """
        ...


@dataclass(slots=True)
class RampMoveConfig:
    """Configuration for a simple ramp move on one or more axes."""

    axis_configs: list[AxisMotionConfig]
    segment_duration_s: float = 0.004


class RampMove(Move):
    """
    A move that follows a trapezoidal velocity ramp on one or more axes.
    This is the standard move type for spindle and lateral axis motion.

    Example:
        move = RampMove(
            name="spindle_run",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=0,
                        ramp=RampConfig(target_rpm=1500, accel_s=0.5,
                                        cruise_s=5.0, decel_s=0.5),
                    )
                ]
            ),
        )
    """

    def __init__(self, name: str, config: RampMoveConfig) -> None:
        super().__init__(name)
        self._config = config

    def segments(self) -> Iterator[MultiAxisSegment]:
        gen = MultiAxisSegmentGenerator(
            self._config.axis_configs,
            segment_duration_s=self._config.segment_duration_s,
        )
        yield from gen

    def expected_delta_steps(self, axis_id: int) -> int | None:
        # Total steps = integral of hz over time. For position tracking
        # this is an estimate — return None to avoid false precision.
        # Subclass and override if exact step count is needed.
        return None


class HomingMove(Move):
    """
    A homing sequence on a single axis.

    Phase 1 — fast approach: move toward_endstop at approach_rpm until
               the endstop fires or max_approach_steps is reached.
    Phase 2 — backoff: move away from endstop by backoff_steps.
    Phase 3 — slow search: move toward_endstop at search_rpm until the
               endstop fires again. This is the true home position.
    Phase 4 — set position to home_position_steps.

    The MoveQueue must arm the endstop before each approach phase and
    disarm it during the backoff phase.

    Phases are exposed as separate sub-moves so MoveQueue can arm/disarm
    the endstop and update axis state between phases.
    """

    def __init__(
        self,
        name: str,
        axis_id: int,
        steps_per_rev: int,
        approach_rpm: float,
        search_rpm: float,
        backoff_steps: int,
        max_approach_steps: int,
        home_position_steps: int = 0,
        segment_duration_s: float = 0.004,
        reverse_direction: bool = False,
    ) -> None:
        super().__init__(name)
        self.axis_id = axis_id
        self.steps_per_rev = steps_per_rev
        self.approach_rpm = approach_rpm
        self.search_rpm = search_rpm
        self.backoff_steps = backoff_steps
        self.max_approach_steps = max_approach_steps
        self.home_position_steps = home_position_steps
        self.segment_duration_s = segment_duration_s
        self.reverse_direction = reverse_direction

        # HomingMove is composed of sub-phases.
        # MoveQueue uses these directly, not segments().
        self._current_phase: str = "approach"

    def _make_approach_move(self) -> RampMove:
        """Phase 1: fast move toward endstop."""
        total_s = (self.max_approach_steps / float(self.steps_per_rev)) / (
            self.approach_rpm / 60.0
        )
        return RampMove(
            name=f"{self.name}:approach",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=self.axis_id,
                        ramp=RampConfig(
                            axis_id=self.axis_id,
                            steps_per_rev=self.steps_per_rev,
                            target_rpm=self.approach_rpm,
                            accel_s=min(0.2, total_s * 0.2),
                            cruise_s=max(total_s - 0.4, 0.0),
                            decel_s=min(0.2, total_s * 0.2),
                            reverse_direction=self.reverse_direction,
                        ),
                    )
                ],
                segment_duration_s=self.segment_duration_s,
            ),
        )

    def _make_backoff_move(self) -> RampMove:
        """Phase 2: move away from endstop."""
        total_s = (self.backoff_steps / float(self.steps_per_rev)) / (
            self.search_rpm / 60.0
        )
        return RampMove(
            name=f"{self.name}:backoff",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=self.axis_id,
                        ramp=RampConfig(
                            axis_id=self.axis_id,
                            steps_per_rev=self.steps_per_rev,
                            target_rpm=self.search_rpm,
                            accel_s=min(0.1, total_s * 0.3),
                            cruise_s=max(total_s - 0.2, 0.0),
                            decel_s=min(0.1, total_s * 0.3),
                            # Backoff moves AWAY from endstop = opposite direction
                            reverse_direction=not self.reverse_direction,
                        ),
                    )
                ],
                segment_duration_s=self.segment_duration_s,
            ),
        )

    def _make_search_move(self) -> RampMove:
        """Phase 3: slow move toward endstop for precise home."""
        total_s = (self.backoff_steps * 2 / float(self.steps_per_rev)) / (
            self.search_rpm / 60.0
        )
        return RampMove(
            name=f"{self.name}:search",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=self.axis_id,
                        ramp=RampConfig(
                            axis_id=self.axis_id,
                            steps_per_rev=self.steps_per_rev,
                            target_rpm=self.search_rpm,
                            accel_s=min(0.1, total_s * 0.2),
                            cruise_s=max(total_s - 0.2, 0.0),
                            decel_s=min(0.1, total_s * 0.2),
                            reverse_direction=self.reverse_direction,
                        ),
                    )
                ],
                segment_duration_s=self.segment_duration_s,
            ),
        )

    def phases(self) -> list[tuple[str, RampMove, bool]]:
        """
        Returns list of (phase_name, sub_move, endstop_armed).
        MoveQueue iterates this list, arming/disarming between phases.
        """
        return [
            ("approach", self._make_approach_move(), True),
            ("backoff", self._make_backoff_move(), False),
            ("search", self._make_search_move(), True),
        ]

    def segments(self) -> Iterator[MultiAxisSegment]:
        # HomingMove is executed phase-by-phase by MoveQueue.
        # This method is not used directly.
        raise NotImplementedError(
            "HomingMove is executed via phases(), not segments(). "
            "Use MoveQueue.enqueue() instead of running it directly."
        )

    def expected_delta_steps(self, axis_id: int) -> int | None:
        return None  # Position is set explicitly after homing completes.


class JogMove(Move):
    """
    Move an axis by a fixed number of steps at a given speed.
    Useful for manual positioning and clearance moves.

    Example:
        move = JogMove(
            name="clear_endstop",
            axis_id=1,
            steps_per_rev=200 * 32,
            steps=3200,           # 0.5 rev at 32 microstep
            rpm=100.0,
            reverse_direction=True,
        )
    """

    def __init__(
        self,
        name: str,
        axis_id: int,
        steps_per_rev: int,
        steps: int,
        rpm: float,
        reverse_direction: bool = False,
        segment_duration_s: float = 0.004,
    ) -> None:
        super().__init__(name)
        self.axis_id = axis_id
        self._steps = steps
        self._reverse = reverse_direction
        total_s = (steps / float(steps_per_rev)) / (rpm / 60.0)
        self._config = RampMoveConfig(
            axis_configs=[
                AxisMotionConfig(
                    axis_id=axis_id,
                    ramp=RampConfig(
                        axis_id=axis_id,
                        steps_per_rev=steps_per_rev,
                        target_rpm=rpm,
                        accel_s=min(0.15, total_s * 0.2),
                        cruise_s=max(total_s - 0.3, 0.0),
                        decel_s=min(0.15, total_s * 0.2),
                        reverse_direction=reverse_direction,
                    ),
                )
            ],
            segment_duration_s=segment_duration_s,
        )

    def segments(self) -> Iterator[MultiAxisSegment]:
        gen = MultiAxisSegmentGenerator(
            self._config.axis_configs,
            segment_duration_s=self._config.segment_duration_s,
        )
        yield from gen

    def expected_delta_steps(self, axis_id: int) -> int | None:
        if axis_id != self.axis_id:
            return None
        return -self._steps if self._reverse else self._steps
