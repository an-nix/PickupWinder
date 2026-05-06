from __future__ import annotations

from dataclasses import dataclass

from transport.messages import LATERAL_ENDSTOP_PRESENT_CLOSED
from threading import Lock


@dataclass
class AxisLimits:
    """Software travel limits in steps. None = no limit."""

    min_steps: int | None = None
    max_steps: int | None = None


class AxisState:
    """
    Tracks the known state of one physical motor axis.

    Position is maintained in microsteps. It is updated by the MoveQueue
    after each Move completes. If a move is aborted (endstop, stop request),
    position is marked unknown until homing is performed.

    Thread-safe: all reads and writes go through a Lock.
    """

    def __init__(
        self,
        axis_id: int,
        steps_per_rev: int = 200 * 32,
        steps_per_mm: float | None = None,
        limits: AxisLimits | None = None,
        homed: bool = False,
        position_steps: int | None = None,
    ) -> None:
        self.axis_id = axis_id
        self.steps_per_rev = steps_per_rev
        self.steps_per_mm = steps_per_mm
        self.limits = limits or AxisLimits()
        self._lock = Lock()
        initial_position = position_steps
        if homed and initial_position is None:
            initial_position = 0
        self._position_steps: int | None = initial_position
        self._homed: bool = bool(homed)
        self._endstop_state: int = 255  # 255 = absent, 0 = open, 1 = closed

    # ── Position ────────────────────────────────────────────────────────

    @property
    def position_steps(self) -> int | None:
        with self._lock:
            return self._position_steps

    @property
    def position_mm(self) -> float | None:
        """Return position in mm when ``steps_per_mm`` is configured."""
        with self._lock:
            if self._position_steps is None or self.steps_per_mm is None:
                return None
            return float(self._position_steps) / float(self.steps_per_mm)

    def set_position(self, steps: int) -> None:
        """Set known position. Called after homing or on explicit reset."""
        with self._lock:
            self._position_steps = steps

    def advance_position(self, delta_steps: int) -> None:
        """Add delta_steps to current position. No-op if position unknown."""
        with self._lock:
            if self._position_steps is not None:
                self._position_steps += delta_steps

    def invalidate_position(self) -> None:
        """Mark position unknown. Called on emergency stop or aborted move."""
        with self._lock:
            self._position_steps = None
            self._homed = False

    # ── Homing ──────────────────────────────────────────────────────────

    @property
    def homed(self) -> bool:
        with self._lock:
            return self._homed

    def mark_homed(self, position_steps: int = 0) -> None:
        with self._lock:
            self._position_steps = position_steps
            self._homed = True

    # ── Soft limits ─────────────────────────────────────────────────────

    def check_move(self, delta_steps: int, *, strict: bool = False) -> bool:
        """
        Returns True if moving delta_steps from current position is within
        soft limits.

        If the current position is unknown:
          - strict=False: returns True because limits cannot be checked.
          - strict=True: returns False and the move is refused.
        """
        with self._lock:
            if self._position_steps is None:
                return not strict
            target = self._position_steps + delta_steps
            if self.limits.min_steps is not None and target < self.limits.min_steps:
                return False
            if self.limits.max_steps is not None and target > self.limits.max_steps:
                return False
            return True

    # ── Endstop ─────────────────────────────────────────────────────────

    def update_endstop_state(self, state: int) -> None:
        """Called by MoveQueue after each status poll. state from firmware."""
        with self._lock:
            self._endstop_state = state

    @property
    def endstop_triggered(self) -> bool:
        """True if endstop is in PRESENT_CLOSED state (value 0x01)."""
        with self._lock:
            return self._endstop_state == LATERAL_ENDSTOP_PRESENT_CLOSED

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "axis_id": self.axis_id,
                "position_steps": self._position_steps,
                "position_mm": (
                    None
                    if self._position_steps is None or self.steps_per_mm is None
                    else float(self._position_steps) / float(self.steps_per_mm)
                ),
                "homed": self._homed,
                "endstop_state": self._endstop_state,
                "endstop_triggered": self._endstop_state == LATERAL_ENDSTOP_PRESENT_CLOSED,
                "soft_limit_min_steps": self.limits.min_steps,
                "soft_limit_max_steps": self.limits.max_steps,
            }
