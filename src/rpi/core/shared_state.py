from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

from motion.axis_state import AxisState
from winding.program import WindingProgram


class EngineState(Enum):
    IDLE      = auto()
    HOMING    = auto()
    RUNNING   = auto()
    PAUSED    = auto()
    STOPPING  = auto()
    FAULT     = auto()


@dataclass(slots=True)
class LayerProgress:
    layer_index: int
    total_layers: int
    direction: str        # "forward" | "reverse"
    started_at: float
    completed_at: float | None = None


class SharedState:
    """
    Thread-safe shared data store between WindingEngine and JsonRpcServer.

    WindingEngine writes. JsonRpcServer reads.
    All public methods are protected by a single RLock (reentrant so
    the engine can call multiple setters in sequence without deadlock).
    """

    def __init__(self, axis_states: dict[int, AxisState]) -> None:
        self._lock = threading.RLock()
        self._engine_state = EngineState.IDLE
        self._current_program: WindingProgram | None = None
        self._current_layer: LayerProgress | None = None
        self._completed_layers: int = 0
        self._winding_session: dict[str, Any] | None = None
        self._fault_message: str | None = None
        self._started_at: float | None = None
        self._completed_at: float | None = None
        self.axis_states = axis_states  # AxisState is itself thread-safe

    # ── Engine state ──────────────────────────────────────────────────────

    def set_engine_state(self, state: EngineState) -> None:
        with self._lock:
            self._engine_state = state

    @property
    def engine_state(self) -> EngineState:
        with self._lock:
            return self._engine_state

    # ── Program ────────────────────────────────────────────────────────────

    def set_program(self, program: WindingProgram | None) -> None:
        with self._lock:
            self._current_program = program
            self._completed_layers = 0
            self._fault_message = None
            self._started_at = time.monotonic() if program else None
            self._completed_at = None

    @property
    def current_program(self) -> WindingProgram | None:
        with self._lock:
            return self._current_program

    # ── Layer progress ─────────────────────────────────────────────────────

    def start_layer(self, index: int, total: int, direction: str) -> None:
        with self._lock:
            self._current_layer = LayerProgress(
                layer_index=index,
                total_layers=total,
                direction=direction,
                started_at=time.monotonic(),
            )

    def complete_layer(self) -> None:
        with self._lock:
            if self._current_layer is not None:
                self._current_layer.completed_at = time.monotonic()
                self._completed_layers += 1

    @property
    def completed_layers(self) -> int:
        with self._lock:
            return self._completed_layers

    # ── Adaptive winding session ─────────────────────────────────────────

    def set_winding_session(self, snapshot: dict[str, Any] | None) -> None:
        with self._lock:
            self._winding_session = snapshot

    # ── Atomic composite transitions ─────────────────────────────────────
    # These methods update engine_state and winding_session in a single lock
    # acquisition, eliminating the TOCTOU window that exists when callers
    # call set_engine_state() and set_winding_session() separately.

    def transition_to_paused(self, session_snapshot: dict[str, Any]) -> None:
        """Atomically set engine state to PAUSED and update the session snapshot."""
        with self._lock:
            self._engine_state = EngineState.PAUSED
            self._winding_session = session_snapshot

    def transition_to_running(self, session_snapshot: dict[str, Any]) -> None:
        """Atomically set engine state to RUNNING and update the session snapshot."""
        with self._lock:
            self._engine_state = EngineState.RUNNING
            self._winding_session = session_snapshot

    def transition_to_idle_session(
        self, session_snapshot: dict[str, Any] | None = None
    ) -> None:
        """Atomically set engine state to IDLE and update the session snapshot."""
        with self._lock:
            self._engine_state = EngineState.IDLE
            self._winding_session = session_snapshot

    def transition_to_stopping(self, session_snapshot: dict[str, Any]) -> None:
        """Atomically set engine state to STOPPING and update the session snapshot."""
        with self._lock:
            self._engine_state = EngineState.STOPPING
            self._winding_session = session_snapshot

    # ── Fault ──────────────────────────────────────────────────────────────

    def set_fault(self, message: str) -> None:
        with self._lock:
            self._engine_state = EngineState.FAULT
            self._fault_message = message
            self._completed_at = time.monotonic()

    def clear_fault(self) -> None:
        with self._lock:
            if self._engine_state == EngineState.FAULT:
                self._engine_state = EngineState.IDLE
                self._fault_message = None

    # ── Snapshot ───────────────────────────────────────────────────────────

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            layer_snap = None
            if self._current_layer is not None:
                layer_snap = {
                    "index": self._current_layer.layer_index,
                    "total": self._current_layer.total_layers,
                    "direction": self._current_layer.direction,
                    "started_at": self._current_layer.started_at,
                    "completed_at": self._current_layer.completed_at,
                }
            return {
                "engine_state": self._engine_state.name,
                "program": self._current_program.snapshot()
                           if self._current_program else None,
                "current_layer": layer_snap,
                "completed_layers": self._completed_layers,
                "winding_session": self._winding_session,
                "fault_message": self._fault_message,
                "started_at": self._started_at,
                "completed_at": self._completed_at,
                "axis_states": {
                    ax_id: state.snapshot()
                    for ax_id, state in self.axis_states.items()
                },
            }
