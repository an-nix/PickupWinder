"""RPC handler for winding execution: submit_program, wound_run, stop, status."""

from __future__ import annotations

from typing import Any

from core import AppConfiguration, WindingEngine
from core.command_service import MotionCommandService
from core.coordinator import MotionCoordinator
from core.events import EventBus, EventKind
from core.shared_state import SharedState
from core.status import RuntimeStatusService
from jsonrpc.handlers import RpcHandler
from jsonrpc.protocol import JsonRpcError
from winding import ProgramNotFoundError, ProgramStore
from winding.program import WindingProgram
from winding.session import SessionParams


class ExecutionRpcHandler:
    """Handles winding.submit_program, wound_run, flush_until, status, axis_state, stop."""

    def __init__(
        self,
        *,
        engine: WindingEngine,
        commands: MotionCommandService,
        coordinator: MotionCoordinator,
        status_service: RuntimeStatusService,
        shared_state: SharedState,
        event_bus: EventBus,
        program_store: ProgramStore,
        config: AppConfiguration,
    ) -> None:
        self._engine = engine
        self._commands = commands
        self._coordinator = coordinator
        self._status_service = status_service
        self._state = shared_state
        self._events = event_bus
        self._program_store = program_store
        self._config = config

    def register_all(self, handler: RpcHandler) -> None:
        handler.register_method("winding.submit_program", self.submit_program)
        handler.register_method("winding.wound_run", self.wound_run)
        handler.register_method("winding.flush_until", self.flush_until)
        handler.register_method("winding.status", self.status)
        handler.register_method("winding.axis_state", self.axis_state)
        handler.register_method("winding.stop", self.stop)

    # ── Methods ────────────────────────────────────────────────────────────

    def submit_program(
        self,
        program: dict[str, Any] | None = None,
        program_id: str | None = None,
        spindle_rpm: float | None = None,
        save: bool = False,
        load: bool = True,
    ) -> dict[str, Any]:
        """Queue a full winding program for deterministic layer execution."""
        try:
            p = self._resolve_program(program=program, program_id=program_id)
            if spindle_rpm is None:
                raise ValueError("spindle_rpm is required to submit a program")
            params = SessionParams(spindle_rpm=float(spindle_rpm))
            params.validate()
            if save and program is not None:
                p = self._program_store.save_program(p, program_id=program_id)
                self._events.publish(EventKind.PROGRAM_SAVED, program=p.snapshot())
            if load:
                self._state.set_loaded_program(p)
                self._events.publish(EventKind.PROGRAM_LOADED, program=p.snapshot())
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        except ValueError as exc:
            raise JsonRpcError(-32602, str(exc)) from exc
        self._engine.submit_program(p, params)
        return {"status": "queued", "program": p.snapshot()}

    def wound_run(
        self,
        spindle_axis_id: int,
        traverse_axis_id: int,
        target_rpm: float,
        bobbin_width_mm: float,
        turns_per_mm: float,
        accel_s: float | None = None,
        cruise_s: float | None = None,
        decel_s: float | None = None,
        scatter_amplitude_mm: float = 0.0,
        scatter_damping_margin_mm: float = 0.0,
        scatter_freq1: float = 1.0,
        scatter_freq2: float = 1.618,
        spindle_reverse: bool = False,
        traverse_reverse: bool = False,
    ) -> dict[str, Any]:
        """Execute a synchronized winding operation (low-level / diagnostic)."""
        self._commands.wound_run(
            spindle_axis_id=spindle_axis_id,
            traverse_axis_id=traverse_axis_id,
            target_rpm=target_rpm,
            accel_s=accel_s,
            cruise_s=cruise_s,
            decel_s=decel_s,
            bobbin_width_mm=bobbin_width_mm,
            turns_per_mm=turns_per_mm,
            scatter_amplitude_mm=scatter_amplitude_mm,
            scatter_damping_margin_mm=scatter_damping_margin_mm,
            scatter_freq1=scatter_freq1,
            scatter_freq2=scatter_freq2,
            spindle_reverse=spindle_reverse,
            traverse_reverse=traverse_reverse,
        )
        return {"status": "queued"}

    def flush_until(self, sequence: int) -> dict[str, Any]:
        if sequence < 0 or sequence > 0xFFFF:
            raise JsonRpcError(-32602, "Invalid params: sequence must be 0-65535")
        status = self._commands.flush_until(sequence)
        return {
            "status": "flushed",
            "flush_sequence": sequence,
            "firmware_status": status,
        }

    def status(self, _params: Any | None = None) -> dict[str, Any]:
        return self._status_service.engine_status()

    def axis_state(self, axis_id: int) -> dict[str, Any]:
        try:
            return self._status_service.axis_state(axis_id)
        except RuntimeError as exc:
            raise JsonRpcError(-32602, str(exc)) from exc

    def stop(self, mode: str = "stop") -> dict[str, Any]:
        """Abort or pause motion."""
        normalized_mode = mode.strip().lower()
        if normalized_mode == "pause":
            try:
                return self._coordinator.request_pause()
            except RuntimeError as exc:
                raise JsonRpcError(-32000, str(exc)) from exc
        if normalized_mode == "emergency_stop":
            plan = self._coordinator.request_emergency_stop()
            return {"status": "emergency_stopping", "stop_plan": plan.snapshot()}
        if normalized_mode != "stop":
            raise JsonRpcError(
                -32602, "Invalid params: mode must be pause, stop, or emergency_stop"
            )
        plan = self._coordinator.request_stop()
        return {"status": "stopping", "stop_plan": plan.snapshot()}

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
