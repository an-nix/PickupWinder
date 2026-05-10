"""RPC handler for all ``winding.*`` methods.

``WindingRpcHandler`` replaces the ad-hoc closure pattern that was
previously used inside ``winding_main._register_winding_rpc_methods()``.

Each public method maps 1-to-1 with a JSON-RPC method name.  The handler
is registered once at startup via ``WindingRpcHandler.register_all()``,
which keeps ``winding_main.py`` free of imperative registration boilerplate.

Compatibility guarantee
-----------------------
All ``winding.*`` method *names* and their JSON parameter contracts are
unchanged.  Callers (e.g. ``run_axis_rpc.py``) require no modification.
"""

from __future__ import annotations

from typing import Any

from core import AppConfiguration, ConfigurationManager, WindingEngine
from core.coordinator import MotionCoordinator
from core.events import EventBus, EventKind
from core.shared_state import SharedState
from core.status import RuntimeStatusService
from jsonrpc.handlers import RpcHandler
from jsonrpc.protocol import JsonRpcError
from core.command_service import MotionCommandService
from winding import AdaptiveWindingSessionConfig, ProgramNotFoundError, ProgramStore
from winding.program import WindingProgram
from winding.service import AdaptiveWindingService


class WindingRpcHandler:
    """Exposes every ``winding.*`` RPC method as a typed public method."""

    def __init__(
        self,
        *,
        engine: WindingEngine,
        commands: MotionCommandService,
        adaptive_winding: AdaptiveWindingService,
        status_service: RuntimeStatusService,
        coordinator: MotionCoordinator,
        config: AppConfiguration,
        config_manager: ConfigurationManager,
        shared_state: SharedState,
        event_bus: EventBus,
        program_store: ProgramStore,
    ) -> None:
        self._engine = engine
        self._commands = commands
        self._adaptive_winding = adaptive_winding
        self._status_service = status_service
        self._coordinator = coordinator
        self._config = config
        self._config_manager = config_manager
        self._state = shared_state
        self._events = event_bus
        self._program_store = program_store

    # ── Registration ───────────────────────────────────────────────────────

    def register_all(self, handler: RpcHandler) -> None:
        """Register every ``winding.*`` method on *handler*."""
        handler.register_method("winding.submit_program", self.submit_program)
        handler.register_method("program.list", self.list_programs)
        handler.register_method("program.get", self.get_program)
        handler.register_method("program.save", self.save_program)
        handler.register_method("program.update", self.update_program)
        handler.register_method("program.load", self.load_program)
        handler.register_method("program.delete", self.delete_program)
        handler.register_method("program.list_revisions", self.list_revisions)
        handler.register_method("program.restore_revision", self.restore_revision)
        handler.register_method("winding.start_session", self.start_session)
        handler.register_method("winding.update_session", self.update_session)
        handler.register_method("winding.pause", self.pause)
        handler.register_method("winding.pause_session", self.pause_session)
        handler.register_method("winding.resume_session", self.resume_session)
        handler.register_method("winding.session_status", self.session_status)
        handler.register_method("winding.stop", self.stop)
        handler.register_method("winding.jog", self.jog)
        handler.register_method("winding.wound_run", self.wound_run)
        handler.register_method("winding.run_axis", self.run_axis)
        handler.register_method("winding.home_lateral", self.home_lateral)
        handler.register_method("winding.move_lateral_mm", self.move_lateral_mm)
        handler.register_method("winding.set_axis_offset", self.set_axis_offset)
        handler.register_method(
            "winding.move_to_start_position",
            self.move_to_start_position,
        )
        handler.register_method("winding.clear_fault", self.clear_fault)
        handler.register_method("winding.flush_until", self.flush_until)
        handler.register_method("winding.status", self.status)
        handler.register_method("winding.axis_state", self.axis_state)
        handler.register_method("winding.arm_endstop", self.arm_endstop)
        handler.register_method("winding.disarm_endstop", self.disarm_endstop)

    # ── RPC methods ────────────────────────────────────────────────────────

    def submit_program(
        self,
        program: dict[str, Any] | None = None,
        program_id: str | None = None,
        save: bool = False,
        load: bool = True,
    ) -> dict[str, Any]:
        """Queue a full winding program for execution."""
        try:
            p = self._resolve_program(program=program, program_id=program_id)
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

        self._engine.submit_program(p)
        return {"status": "queued", "program": p.snapshot()}

    def list_programs(self, include_content: bool = False) -> dict[str, Any]:
        loaded_program = self._state.loaded_program
        return {
            "programs": self._program_store.list_programs(include_content=include_content),
            "loaded_program": loaded_program.snapshot() if loaded_program else None,
        }

    def get_program(self, program_id: str) -> dict[str, Any]:
        try:
            program = self._program_store.get_program(program_id)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        return {"program": program.snapshot()}

    def save_program(
        self,
        program: dict[str, Any],
        program_id: str | None = None,
        load: bool = False,
    ) -> dict[str, Any]:
        try:
            saved = self._program_store.save_program(
                self._coerce_program(program),
                program_id=program_id,
            )
        except ValueError as exc:
            raise JsonRpcError(-32602, str(exc)) from exc

        if load:
            self._state.set_loaded_program(saved)
            self._events.publish(EventKind.PROGRAM_LOADED, program=saved.snapshot())
        self._events.publish(EventKind.PROGRAM_SAVED, program=saved.snapshot())
        return {"status": "saved", "program": saved.snapshot()}

    def update_program(
        self,
        program_id: str,
        changes: dict[str, Any] | None = None,
        program: dict[str, Any] | None = None,
        load: bool = False,
    ) -> dict[str, Any]:
        effective_changes = changes if changes is not None else program
        if not isinstance(effective_changes, dict):
            raise JsonRpcError(-32602, "Invalid params: expected program changes object")
        try:
            updated = self._program_store.update_program(program_id, effective_changes)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        except ValueError as exc:
            raise JsonRpcError(-32602, str(exc)) from exc

        loaded_program = self._state.loaded_program
        if load or (loaded_program is not None and loaded_program.program_id == updated.program_id):
            self._state.set_loaded_program(updated)
            self._events.publish(EventKind.PROGRAM_LOADED, program=updated.snapshot())
        self._events.publish(EventKind.PROGRAM_UPDATED, program=updated.snapshot())
        return {"status": "updated", "program": updated.snapshot()}

    def load_program(self, program_id: str) -> dict[str, Any]:
        try:
            program = self._program_store.get_program(program_id)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        self._state.set_loaded_program(program)
        self._events.publish(EventKind.PROGRAM_LOADED, program=program.snapshot())
        return {"status": "loaded", "program": program.snapshot()}

    def delete_program(self, program_id: str) -> dict[str, Any]:
        current_program = self._state.current_program
        if current_program is not None and current_program.program_id == program_id:
            raise JsonRpcError(-32000, "Cannot delete a program while it is executing")

        try:
            self._program_store.delete_program(program_id)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc

        loaded_program = self._state.loaded_program
        if loaded_program is not None and loaded_program.program_id == program_id:
            self._state.set_loaded_program(None)
        self._events.publish(EventKind.PROGRAM_DELETED, program_id=program_id)
        return {"status": "deleted", "program_id": program_id}

    def list_revisions(self, program_id: str) -> dict[str, Any]:
        """Return the backup revision list for *program_id*."""
        try:
            revisions = self._program_store.list_revisions(program_id)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        return {"program_id": program_id, "revisions": revisions}

    def restore_revision(self, program_id: str, revision: int) -> dict[str, Any]:
        """Restore a backup *revision* of *program_id* as the new head."""
        try:
            restored = self._program_store.restore_revision(program_id, int(revision))
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        self._events.publish(EventKind.PROGRAM_SAVED, program=restored.snapshot())
        return {"status": "restored", "program": restored.snapshot()}

    def start_session(self, session: dict[str, Any]) -> dict[str, Any]:
        """Start an adaptive winding session with live-controllable geometry."""
        if not isinstance(session, dict):
            raise JsonRpcError(-32602, "Invalid params: expected session object")
        config = AdaptiveWindingSessionConfig(**session)
        snapshot = self._adaptive_winding.start_session(config)
        return {"status": "started", "session": snapshot}

    def _coerce_program(self, program: dict[str, Any]) -> WindingProgram:
        if not isinstance(program, dict):
            raise JsonRpcError(-32602, "Invalid params: expected program object")
        return WindingProgram.from_payload(program)

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

    def update_session(self, **params: Any) -> dict[str, Any]:
        """Update the active adaptive winding session controls."""
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

    def pause_session(self, pause_at_turn: float | None = None) -> dict[str, Any]:
        """Request a controlled pause for the adaptive winding session."""
        return self.pause(pause_at_turn=pause_at_turn)

    def resume_session(self, _params: Any | None = None) -> dict[str, Any]:
        """Resume a paused adaptive winding session."""
        snapshot = self._adaptive_winding.resume_session()
        return {"status": "running", "session": snapshot}

    def session_status(self, _params: Any | None = None) -> dict[str, Any]:
        """Return the current adaptive winding session snapshot."""
        return self._adaptive_winding.session_status()

    def stop(self, mode: str = "stop") -> dict[str, Any]:
        """Abort or pause motion using the explicit host stop-mode contract."""
        normalized_mode = mode.strip().lower()
        if normalized_mode == "pause":
            return self.pause()
        if normalized_mode == "emergency_stop":
            plan = self._coordinator.request_emergency_stop()
            return {"status": "emergency_stopping", "stop_plan": plan.snapshot()}
        if normalized_mode != "stop":
            raise JsonRpcError(-32602, "Invalid params: mode must be pause, stop, or emergency_stop")
        plan = self._coordinator.request_stop()
        return {"status": "stopping", "stop_plan": plan.snapshot()}

    def jog(
        self,
        axis_id: int,
        steps: int,
        rpm: float,
        reverse: bool = False,
    ) -> dict[str, Any]:
        """Jog *axis_id* by *steps* at *rpm*."""
        self._commands.jog(axis_id=axis_id, steps=steps, rpm=rpm, reverse=reverse)
        return {"status": "queued"}

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
        """Execute a synchronized winding operation (Electronic Gearing)."""
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

    def run_axis(
        self,
        duration_s: float,
        targets: list[dict[str, Any]],
    ) -> dict[str, Any]:
        """Queue a config-limited trapezoidal ramp for one or two axes."""
        self._commands.run_axis(duration_s=duration_s, targets=targets)
        return {"status": "queued"}

    def home_lateral(
        self,
    ) -> dict[str, Any]:
        """Start the lateral homing sequence and return immediately."""
        result = self._commands.home_lateral()
        return result

    def move_lateral_mm(self, position_mm: float, rpm: float) -> dict[str, Any]:
        """Move the lateral axis to an absolute mm position from home zero."""
        return self._commands.move_lateral_to_mm(position_mm=position_mm, rpm=rpm)

    def set_axis_offset(self, offset_mm: float) -> dict[str, Any]:
        """Set and persist the lateral winding start offset in mm."""
        try:
            offset_mm = float(offset_mm)
        except (TypeError, ValueError) as exc:
            raise JsonRpcError(-32602, f"offset_mm must be a number: {exc}") from exc

        try:
            validated = AppConfiguration(
                **{**vars(self._config), "lateral_axis_offset_mm": offset_mm}
            )
        except ValueError as exc:
            raise JsonRpcError(-32602, str(exc)) from exc

        old_offset_mm = self._config.lateral_axis_offset_mm
        self._config.lateral_axis_offset_mm = offset_mm
        try:
            self._config_manager.save_configuration(self._config)
        except OSError as exc:
            self._config.lateral_axis_offset_mm = old_offset_mm
            raise JsonRpcError(-32000, f"failed to persist configuration: {exc}") from exc

        return {
            "status": "ok",
            "axis_offset_mm": offset_mm,
            "start_position_mm": validated.lateral_start_position_mm,
            "soft_limit_min_mm": validated.lateral_soft_limit_min_mm,
        }

    def move_to_start_position(self, _params: Any | None = None) -> dict[str, Any]:
        """Move the lateral axis to soft_limit_min_mm + lateral_axis_offset_mm."""
        try:
            return self._commands.move_to_start_position()
        except RuntimeError as exc:
            raise JsonRpcError(-32000, str(exc)) from exc

    def clear_fault(self, _params: Any | None = None) -> dict[str, str]:
        """Clear FAULT state so a new program can be submitted."""
        self._coordinator.clear_fault()
        return {"status": "ok"}

    def flush_until(self, sequence: int) -> dict[str, Any]:
        """Request the firmware to flush and wait for the given motion sequence."""
        if sequence < 0 or sequence > 0xFFFF:
            raise JsonRpcError(-32602, "Invalid params: sequence must be 0-65535")
        status = self._commands.flush_until(sequence)
        return {
            "status": "flushed",
            "flush_sequence": sequence,
            "firmware_status": status,
        }

    def status(self, _params: Any | None = None) -> dict[str, Any]:
        """Return combined engine and move-queue status snapshot."""
        return self._status_service.engine_status()

    def axis_state(self, axis_id: int) -> dict[str, Any]:
        """Return the position / state snapshot for a single axis."""
        try:
            return self._status_service.axis_state(axis_id)
        except RuntimeError as exc:
            raise JsonRpcError(-32602, str(exc)) from exc

    def arm_endstop(self, axis_id: int) -> dict[str, Any]:
        """Arm the endstop for *axis_id*."""
        self._commands.arm_endstop(axis_id)
        return {"status": "armed", "axis_id": axis_id}

    def disarm_endstop(self, axis_id: int) -> dict[str, Any]:
        """Disarm the endstop for *axis_id*."""
        self._commands.disarm_endstop(axis_id)
        return {"status": "disarmed", "axis_id": axis_id}
