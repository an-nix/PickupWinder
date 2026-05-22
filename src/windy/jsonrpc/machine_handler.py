"""RPC handler for machine motion: jog, homing, axis moves, endstops, fault clear."""

from __future__ import annotations

from typing import Any

from core import AppConfiguration, ConfigurationManager
from core.command_service import MotionCommandService
from core.coordinator import MotionCoordinator
from jsonrpc.handlers import RpcHandler
from jsonrpc.protocol import JsonRpcError


class MachineRpcHandler:
    """Handles winding.jog, run_axis, home_lateral, move_*, set_axis_offset,
    clear_fault, arm/disarm_endstop."""

    def __init__(
        self,
        *,
        commands: MotionCommandService,
        coordinator: MotionCoordinator,
        config: AppConfiguration,
        config_manager: ConfigurationManager,
    ) -> None:
        self._commands = commands
        self._coordinator = coordinator
        self._config = config
        self._config_manager = config_manager

    def register_all(self, handler: RpcHandler) -> None:
        handler.register_method("winding.jog", self.jog)
        handler.register_method("winding.run_axis", self.run_axis)
        handler.register_method("winding.home_lateral", self.home_lateral)
        handler.register_method("winding.move_lateral_mm", self.move_lateral_mm)
        handler.register_method("winding.set_axis_offset", self.set_axis_offset)
        handler.register_method("winding.move_to_start_position", self.move_to_start_position)
        handler.register_method("winding.clear_fault", self.clear_fault)
        handler.register_method("winding.arm_endstop", self.arm_endstop)
        handler.register_method("winding.disarm_endstop", self.disarm_endstop)

    # ── Methods ────────────────────────────────────────────────────────────

    def jog(
        self,
        axis_id: int,
        steps: int,
        rpm: float,
        reverse: bool = False,
    ) -> dict[str, Any]:
        self._commands.jog(axis_id=axis_id, steps=steps, rpm=rpm, reverse=reverse)
        return {"status": "queued"}

    def run_axis(
        self,
        duration_s: float,
        targets: list[dict[str, Any]],
    ) -> dict[str, Any]:
        self._commands.run_axis(duration_s=duration_s, targets=targets)
        return {"status": "queued"}

    def home_lateral(self) -> dict[str, Any]:
        return self._commands.home_lateral()

    def move_lateral_mm(self, position_mm: float, rpm: float) -> dict[str, Any]:
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
        try:
            return self._commands.move_to_start_position()
        except RuntimeError as exc:
            raise JsonRpcError(-32000, str(exc)) from exc

    def clear_fault(self, _params: Any | None = None) -> dict[str, str]:
        self._coordinator.clear_fault()
        return {"status": "ok"}

    def arm_endstop(self, axis_id: int) -> dict[str, Any]:
        self._commands.arm_endstop(axis_id)
        return {"status": "armed", "axis_id": axis_id}

    def disarm_endstop(self, axis_id: int) -> dict[str, Any]:
        self._commands.disarm_endstop(axis_id)
        return {"status": "disarmed", "axis_id": axis_id}
