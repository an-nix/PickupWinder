# ---------------------------------------------------------------------------
# NOTE D'ARCHITECTURE — Infrastructure homing gelée
# ---------------------------------------------------------------------------
# RampMove, RampMoveConfig et AxisMotionConfig sont conservés intentionnellement
# comme infrastructure interne de HomingMove.
# Ils NE font PAS partie de l'API publique de mouvement.
# HomingMove._make_approach_move(), _make_backoff_move(), _make_search_move()
# sont corrects, testés et gelés — toute modification nécessite une tâche
# spécifique au homing avec revue dédiée.
# Voir : doc/architecture.md § Politique d'isolement du homing
# ---------------------------------------------------------------------------

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Iterator

from motion import (
    AxisMotionConfig,
    MultiAxisSegmentGenerator,
    RampConfig,
)
from transport.messages import MultiAxisSegment


class MoveState(Enum):
    PENDING = auto()
    RUNNING = auto()
    COMPLETED = auto()
    ABORTED = auto()
    FAILED = auto()


class BaseMove(ABC):
    """
    Shared lifecycle base for all motion commands.

    Carries state machine, timing, error, and snapshot logic.
    Does NOT prescribe a segment interface — concrete sub-hierarchies
    define their own execution contracts (``Move`` and ``CompositeMove``).

    Public properties
    -----------------
    state               Current MoveState.
    done                True when the move has reached a terminal state.
    error               Last error/abort message (None if none).
    aborted_by_endstop  True if an endstop triggered the abort.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self._state = MoveState.PENDING
        self._started_at: float | None = None
        self._completed_at: float | None = None
        self._error: str | None = None
        self._aborted_by_endstop: bool = False

    # ── Read-only public interface ─────────────────────────────────────────

    @property
    def state(self) -> MoveState:
        return self._state

    @property
    def done(self) -> bool:
        return self._state in (MoveState.COMPLETED, MoveState.ABORTED, MoveState.FAILED)

    @property
    def error(self) -> str | None:
        """Last error or abort reason; ``None`` if the move has not failed."""
        return self._error

    @property
    def aborted_by_endstop(self) -> bool:
        return self._aborted_by_endstop

    # ── Lifecycle transitions ─────────────────────────────────────────────

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
    def expected_delta_steps(self, axis_id: int) -> int | None:
        """Expected step delta for *axis_id* after this move completes.

        Return ``None`` if not applicable or variable (e.g. homing moves).
        """
        ...

    @property
    @abstractmethod
    def axis_ids(self) -> list[int]:
        """List of axis IDs involved in this move."""
        ...


class Move(BaseMove, ABC):
    """
    Abstract base for *segmented* moves — moves that produce a stream of
    ``MultiAxisSegment`` objects fed directly to the firmware streamer.

    Subclasses must implement:
      - ``segments()``            — yields ``MultiAxisSegment`` objects.
      - ``expected_delta_steps()``— inherited from ``BaseMove``.
    """

    @abstractmethod
    def segments(self) -> Iterator[MultiAxisSegment]:
        """Yield segments to be sent to the firmware."""
        ...

    @property
    def axis_configs(self) -> list[AxisMotionConfig] | None:
        """Optional public exposure of axis configs for streamer construction."""
        return None


class CompositeMove(BaseMove, ABC):
    """
    Abstract base for *composite* (multi-phase) moves that cannot be
    expressed as a single contiguous segment stream.

    ``CompositeMove`` deliberately does **not** declare ``segments()`` as
    obligatory, avoiding an LSP violation for moves like homing that
    require the MoveQueue to arm/disarm endstops between phases.

    Subclasses must implement:
      - ``phases()``              — returns ordered phase descriptors.
      - ``expected_delta_steps()``— inherited from ``BaseMove``.
    """

    @abstractmethod
    def phases(self) -> list[tuple[str, "Move", bool]]:
        """Return an ordered list of ``(phase_name, sub_move, endstop_armed)``
        tuples.  The ``MoveQueue`` iterates these, arming / disarming the
        endstop between phases.
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

    @property
    def axis_ids(self) -> list[int]:
        return [cfg.axis_id for cfg in self._config.axis_configs]

    @property
    def axis_configs(self) -> list[AxisMotionConfig]:
        return self._config.axis_configs

    def expected_delta_steps(self, axis_id: int) -> int | None:
        for axis_config in self._config.axis_configs:
            if axis_config.axis_id != axis_id:
                continue
            total_steps = int(round(axis_config.ramp.steps_at(axis_config.ramp.total_duration)))
            return -total_steps if axis_config.ramp.reverse_direction else total_steps
        return None


class HomingMove(CompositeMove):
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

    ``HomingMove`` extends ``CompositeMove``: execution is driven by
    ``phases()``, not by ``segments()``.  This satisfies the Liskov
    Substitution Principle — ``HomingMove`` never claims to be a
    ``Move`` and will never be passed to a segment streamer directly.
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
        backoff_rpm = max(self.search_rpm, self.approach_rpm * 0.5)
        total_s = (self.backoff_steps / float(self.steps_per_rev)) / (
            backoff_rpm / 60.0
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
                            target_rpm=backoff_rpm,
                            accel_s=min(0.1, total_s * 0.1),
                            cruise_s=max(total_s - 0.2, 0.0),
                            decel_s=min(0.1, total_s * 0.1),
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

    def phases(self) -> list[tuple[str, Move, bool]]:
        """
        Return an ordered list of ``(phase_name, sub_move, endstop_armed)``.
        The ``MoveQueue`` iterates this list, arming/disarming the endstop
        between phases and updating ``AxisState`` after search completes.
        """
        return [
            ("approach", self._make_approach_move(), True),
            ("backoff", self._make_backoff_move(), False),
            ("search", self._make_search_move(), True),
        ]

    def expected_delta_steps(self, axis_id: int) -> int | None:
        return None  # Position is set explicitly after homing completes.

    @property
    def axis_ids(self) -> list[int]:
        return [self.axis_id]
