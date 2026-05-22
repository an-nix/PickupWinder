"""RPC handler for the adaptive winding session lifecycle."""

from __future__ import annotations

from typing import Any

from core.coordinator import MotionCoordinator
from core.events import EventBus, EventKind
from core.shared_state import SharedState
from jsonrpc.handlers import RpcHandler
from jsonrpc.protocol import JsonRpcError
from winding import ProgramNotFoundError, ProgramStore
from winding.program import WindingProgram
from winding.service import AdaptiveWindingService
from winding.session import SessionParams


_ALLOWED_UPDATE_FIELDS: frozenset[str] = frozenset({
    "spindle_rpm",
    "total_turns",
    "window_low_mm",
    "window_high_mm",
})


class SessionRpcHandler:
    """Handles winding.start_session, update_session, pause, resume, session_status."""

    def __init__(
        self,
        *,
        adaptive_winding: AdaptiveWindingService,
        coordinator: MotionCoordinator,
        shared_state: SharedState,
        event_bus: EventBus,
        program_store: ProgramStore,
    ) -> None:
        self._adaptive_winding = adaptive_winding
        self._coordinator = coordinator
        self._state = shared_state
        self._events = event_bus
        self._program_store = program_store

    def register_all(self, handler: RpcHandler) -> None:
        handler.register_method("winding.start_session", self.start_session)
        handler.register_method("winding.update_session", self.update_session)
        handler.register_method("winding.pause", self.pause)
        handler.register_method("winding.resume_session", self.resume_session)
        handler.register_method("winding.session_status", self.session_status)

    # ── Methods ────────────────────────────────────────────────────────────

    def start_session(
        self,
        program_id: str | None = None,
        program: dict[str, Any] | None = None,
        spindle_rpm: float | None = None,
        total_turns: float | None = None,
        window_low_mm: float | None = None,
        window_high_mm: float | None = None,
        chunk_time_s: float | None = None,
        load: bool = True,
    ) -> dict[str, Any]:
        """Start an adaptive winding session.

        Requires ``program_id`` (or inline ``program``) plus ``spindle_rpm``.
        All other parameters are optional overrides adjustable live via
        ``winding.update_session``.
        """
        if spindle_rpm is None:
            raise JsonRpcError(-32602, "Invalid params: spindle_rpm is required")
        try:
            resolved = self._resolve_program(program=program, program_id=program_id)
            params = SessionParams(
                spindle_rpm=float(spindle_rpm),
                total_turns=float(total_turns) if total_turns is not None else None,
                window_low_mm=float(window_low_mm) if window_low_mm is not None else None,
                window_high_mm=float(window_high_mm) if window_high_mm is not None else None,
                chunk_time_s=float(chunk_time_s) if chunk_time_s is not None else 0.25,
            )
            if load:
                self._state.set_loaded_program(resolved)
                self._events.publish(EventKind.PROGRAM_LOADED, program=resolved.snapshot())
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        except ValueError as exc:
            raise JsonRpcError(-32602, str(exc)) from exc
        snapshot = self._adaptive_winding.start_session(resolved, params)
        return {"status": "started", "session": snapshot}

    def update_session(self, **params: Any) -> dict[str, Any]:
        """Update live session controls.

        Allowed fields: ``spindle_rpm``, ``total_turns``,
        ``window_low_mm``, ``window_high_mm``.
        """
        unknown = set(params) - _ALLOWED_UPDATE_FIELDS
        if unknown:
            raise JsonRpcError(
                -32602,
                f"Invalid params: unknown update field(s): {sorted(unknown)}",
            )
        if not params:
            raise JsonRpcError(-32602, "Invalid params: expected at least one control field")
        snapshot = self._adaptive_winding.update_session(**params)
        return {"status": "updated", "session": snapshot}

    def pause(self, pause_at_turn: float | None = None) -> dict[str, Any]:
        """Request a controlled, resumable pause."""
        try:
            result = self._coordinator.request_pause(pause_at_turn=pause_at_turn)
        except RuntimeError as exc:
            raise JsonRpcError(-32000, str(exc)) from exc
        return result

    def resume_session(self, _params: Any | None = None) -> dict[str, Any]:
        snapshot = self._adaptive_winding.resume_session()
        return {"status": "running", "session": snapshot}

    def session_status(self, _params: Any | None = None) -> dict[str, Any]:
        return self._adaptive_winding.session_status()

    # ── Helpers ────────────────────────────────────────────────────────────

    def _resolve_program(
        self,
        *,
        program: dict[str, Any] | None,
        program_id: str | None,
    ) -> WindingProgram:
        if program_id is not None:
            return self._program_store.get_program(program_id)
        if program is not None:
            return self._coerce_program(program)
        loaded_program = self._state.loaded_program
        if loaded_program is None:
            raise ValueError("Either program, program_id, or a loaded program is required")
        return loaded_program

    def _coerce_program(self, program: dict[str, Any]) -> WindingProgram:
        if not isinstance(program, dict):
            raise JsonRpcError(-32602, "Invalid params: expected program object")
        return WindingProgram.from_payload(program)
