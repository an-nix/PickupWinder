"""MotionCoordinator — single entry point for system-wide state transitions.

Centralises the stop and fault-clear operations that previously were duplicated
across ``WindingRpcHandler``, ``WinderApplication`` and
``AdaptiveWindingService``.  All callers that need a global stop or fault
acknowledgement should go through this class rather than calling individual
services directly.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Iterable

if TYPE_CHECKING:
    from core.engine import WindingEngine
    from core.shared_state import SharedState
    from motion.move_queue import MoveQueue
    from winding.service import AdaptiveWindingService


class MotionStopMode(str, Enum):
    """Explicit motion control modes used by the host runtime.

    `pause`
        Controlled, resumable interruption. Homed / authoritative axes remain
        enabled so their position authority is preserved.
    `stop`
        Operational stop. Current motion is aborted, queued moves are drained,
        and in-flight axis positions are invalidated unless explicitly proven.
    `emergency_stop`
        Immediate safety-driven stop. The shutdown path may escalate to this
        mode when normal stop/disable sequencing cannot be verified.
    """

    PAUSE = "pause"
    STOP = "stop"
    EMERGENCY_STOP = "emergency_stop"

@dataclass(frozen=True, slots=True)
class MotionStopPlan:
    """Resolved stop policy for a motion interruption or shutdown path."""

    mode: MotionStopMode
    keep_enabled_axes: frozenset[int]
    invalidate_positions: frozenset[int]
    reason: str | None = None

    @classmethod
    def pause(
        cls,
        axis_states: Mapping[int, object],
        axis_ids: Iterable[int] | None = None,
        *,
        reason: str | None = None,
    ) -> "MotionStopPlan":
        resolved_axis_ids = tuple(axis_ids or axis_states.keys())
        keep_enabled = frozenset(
            axis_id
            for axis_id in resolved_axis_ids
            if getattr(axis_states.get(axis_id), "homed", False)
        )
        return cls(
            mode=MotionStopMode.PAUSE,
            keep_enabled_axes=keep_enabled,
            invalidate_positions=frozenset(),
            reason=reason,
        )

    @classmethod
    def stop(
        cls,
        axis_states: Mapping[int, object],
        axis_ids: Iterable[int] | None = None,
        *,
        reason: str | None = None,
    ) -> "MotionStopPlan":
        resolved_axis_ids = frozenset(axis_ids or axis_states.keys())
        return cls(
            mode=MotionStopMode.STOP,
            keep_enabled_axes=frozenset(),
            invalidate_positions=resolved_axis_ids,
            reason=reason,
        )

    @classmethod
    def emergency_stop(
        cls,
        axis_states: Mapping[int, object],
        axis_ids: Iterable[int] | None = None,
        *,
        reason: str | None = None,
    ) -> "MotionStopPlan":
        resolved_axis_ids = frozenset(axis_ids or axis_states.keys())
        return cls(
            mode=MotionStopMode.EMERGENCY_STOP,
            keep_enabled_axes=frozenset(),
            invalidate_positions=resolved_axis_ids,
            reason=reason,
        )

    def snapshot(self) -> dict[str, object]:
        return {
            "mode": self.mode.value,
            "keep_enabled_axes": sorted(self.keep_enabled_axes),
            "invalidate_positions": sorted(self.invalidate_positions),
            "reason": self.reason,
        }

class MotionCoordinator:
    """Single entry point for system-wide state transitions.

    Parameters
    ----------
    engine:
        The classical ``WindingEngine`` instance.
    adaptive_winding:
        The ``AdaptiveWindingService`` instance.
    move_queue:
        The shared ``MoveQueue``.
    shared_state:
        The shared ``SharedState``.
    """

    def __init__(
        self,
        *,
        engine: WindingEngine,
        adaptive_winding: AdaptiveWindingService,
        move_queue: MoveQueue,
        shared_state: SharedState,
    ) -> None:
        self._engine = engine
        self._adaptive = adaptive_winding
        self._queue = move_queue
        self._state = shared_state

    def _active_axis_ids(self) -> list[int]:
        current_move = self._queue.current_move
        if current_move is not None:
            return list(current_move.axis_ids)
        return list(self._state.axis_states.keys())

    def pause_plan(self, *, reason: str | None = None) -> MotionStopPlan:
        return MotionStopPlan.pause(
            self._state.axis_states,
            self._active_axis_ids(),
            reason=reason,
        )

    def stop_plan(self, *, reason: str | None = None) -> MotionStopPlan:
        return MotionStopPlan.stop(
            self._state.axis_states,
            self._active_axis_ids(),
            reason=reason,
        )

    def emergency_stop_plan(self, *, reason: str | None = None) -> MotionStopPlan:
        return MotionStopPlan.emergency_stop(
            self._state.axis_states,
            self._active_axis_ids(),
            reason=reason,
        )

    def request_pause(self, *, pause_at_turn: float | None = None) -> dict[str, object]:
        """Request a controlled, resumable pause.

        The current implementation only supports pausing an adaptive winding
        session, because classical program execution has no resumable pause
        contract yet. The explicit `MotionStopPlan` still formalises which axes
        must remain enabled while the system is paused.
        """
        plan = self.pause_plan(reason="pause requested")
        snapshot = self._adaptive.pause_session(pause_at_turn=pause_at_turn)
        return {
            "status": "pausing",
            "stop_plan": plan.snapshot(),
            "session": snapshot,
        }

    def request_stop(self) -> MotionStopPlan:
        """Clean stop: request stop on both services then drain the queue.

        Each service ignores the call if it is not active, so calling both
        unconditionally is safe and eliminates the TOCTOU race that exists when
        checking activity before stopping.
        """
        plan = self.stop_plan(reason="stop requested")
        self._adaptive.request_stop(stop_plan=plan, clear_queue=False)
        self._engine.request_stop(stop_plan=plan, clear_queue=False)
        # engine.request_stop() already calls queue.clear(), but we call it
        # explicitly here to guarantee the semantics regardless of future
        # changes to either service.
        self._queue.clear(stop_plan=plan)
        return plan

    def request_emergency_stop(self) -> MotionStopPlan:
        """Escalate to an immediate emergency stop plan."""
        plan = self.emergency_stop_plan(reason="emergency stop requested")
        self._adaptive.request_stop(stop_plan=plan, clear_queue=False)
        self._engine.request_stop(stop_plan=plan, clear_queue=False)
        self._queue.clear(stop_plan=plan)
        return plan

    def clear_fault(self) -> None:
        """Acknowledge a FAULT if no motion is currently in progress.

        Raises
        ------
        RuntimeError
            If a move is still active or pending in the queue.
        """
        if self._queue.current_move is not None or self._queue.pending_count > 0:
            raise RuntimeError("Cannot clear fault while motion is in progress")
        self._engine.clear_fault()
