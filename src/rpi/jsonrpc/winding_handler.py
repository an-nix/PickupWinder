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

from core import WindingEngine
from core.coordinator import MotionCoordinator
from core.status import RuntimeStatusService
from jsonrpc.handlers import RpcHandler
from jsonrpc.protocol import JsonRpcError
from motion.command_service import MotionCommandService
from winding import AdaptiveWindingSessionConfig
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
    ) -> None:
        self._engine = engine
        self._commands = commands
        self._adaptive_winding = adaptive_winding
        self._status_service = status_service
        self._coordinator = coordinator

    # ── Registration ───────────────────────────────────────────────────────

    def register_all(self, handler: RpcHandler) -> None:
        """Register every ``winding.*`` method on *handler*."""
        handler.register_method("winding.submit_program", self.submit_program)
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
        handler.register_method("winding.clear_fault", self.clear_fault)
        handler.register_method("winding.flush_until", self.flush_until)
        handler.register_method("winding.status", self.status)
        handler.register_method("winding.axis_state", self.axis_state)
        handler.register_method("winding.arm_endstop", self.arm_endstop)
        handler.register_method("winding.disarm_endstop", self.disarm_endstop)

    # ── RPC methods ────────────────────────────────────────────────────────

    def submit_program(self, program: dict) -> dict[str, Any]:
        """Queue a full winding program for execution."""
        if not isinstance(program, dict):
            raise JsonRpcError(-32602, "Invalid params: expected program object")
        p = WindingProgram(**program)
        self._engine.submit_program(p)
        return {"status": "queued", "program": p.snapshot()}

    def start_session(self, session: dict[str, Any]) -> dict[str, Any]:
        """Start an adaptive winding session with live-controllable geometry."""
        if not isinstance(session, dict):
            raise JsonRpcError(-32602, "Invalid params: expected session object")
        config = AdaptiveWindingSessionConfig(**session)
        snapshot = self._adaptive_winding.start_session(config)
        return {"status": "started", "session": snapshot}

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
        approach_rpm: float = 100.0,
        search_rpm: float = 20.0,
        backoff_steps: int = 3200,
    ) -> dict[str, Any]:
        """Start the lateral homing sequence and return immediately."""
        result = self._commands.home_lateral(
            approach_rpm=approach_rpm,
            search_rpm=search_rpm,
            backoff_steps=backoff_steps,
        )
        return result

    def move_lateral_mm(self, position_mm: float, rpm: float) -> dict[str, Any]:
        """Move the lateral axis to an absolute mm position from home zero."""
        return self._commands.move_lateral_to_mm(position_mm=position_mm, rpm=rpm)

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
