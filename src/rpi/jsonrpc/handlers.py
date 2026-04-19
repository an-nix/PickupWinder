from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, Dict

from .protocol import JsonRpcError, JsonRpcMethodNotFoundError

MethodCallback = Callable[[Any | None], Any]


class RpcHandler:
    def __init__(self) -> None:
        self._methods: Dict[str, MethodCallback] = {}

    def register_method(self, method: str, callback: MethodCallback) -> None:
        self._methods[method] = callback

    def dispatch(self, method: str, params: Any | None) -> Any:
        callback = self._methods.get(method)
        if callback is None:
            raise JsonRpcMethodNotFoundError(method)
        return callback(params)


@dataclass(slots=True)
class SpindleCommand:
    duration_s: float
    rpm: float


@dataclass(slots=True)
class MotionCommand:
    duration_s: float
    spindle_rpm: float
    lateral_rpm: float


class AppRpcHandler(RpcHandler):
    def __init__(self, app: Any | None = None) -> None:
        super().__init__()
        self.app = app
        self.started_at = time.time()
        self.register_method("winder.ping", lambda _: self.ping())
        self.register_method("winder.status", lambda _: self.status())
        self.register_method("winder.shutdown", lambda _: self.shutdown())
        self.register_method("winder.config", lambda _: self.config())
        self.register_method("winder.spindle.run", self.spindle_run)
        self.register_method("winder.motion.run", self.motion_run)
        self.register_method("winder.operation.status", lambda _: self.operation_status())
        self.register_method("winder.operation.stop", lambda _: self.operation_stop())
        self.register_method("winder.session.status", lambda _: self.session_status())
        self.register_method("winder.session.wait", self.session_wait)

    def ping(self) -> dict[str, str]:
        return {"message": "pong"}

    def status(self) -> dict[str, Any]:
        return {
            "uptime_s": round(time.time() - self.started_at, 2),
            "configured": bool(self.app is not None),
        }

    def shutdown(self) -> dict[str, str]:
        return {"message": "shutdown-not-implemented"}

    def config(self) -> dict[str, Any]:
        if self.app is None:
            return {"error": "no application attached"}
        if hasattr(self.app, "config_snapshot"):
            return self.app.config_snapshot()
        return {
            "spindle_max_speed_rpm": getattr(self.app, "spindle_max_speed_rpm", None),
            "lateral_max_rpm": getattr(self.app, "lateral_max_rpm", None),
        }

    def spindle_run(self, params: Any | None) -> dict[str, Any]:
        if params is None or not isinstance(params, dict):
            raise JsonRpcError(-32602, "Invalid params: expected object with duration_s and rpm")

        duration_s = params.get("duration_s")
        rpm = params.get("rpm")
        if not isinstance(duration_s, (int, float)) or not isinstance(rpm, (int, float)):
            raise JsonRpcError(-32602, "Invalid params: duration_s and rpm must be numbers")

        command = SpindleCommand(duration_s=float(duration_s), rpm=float(rpm))
        return self._run_spindle(command)

    def motion_run(self, params: Any | None) -> dict[str, Any]:
        if params is None or not isinstance(params, dict):
            raise JsonRpcError(
                -32602,
                "Invalid params: expected object with duration_s, spindle_rpm and lateral_rpm",
            )

        duration_s = params.get("duration_s")
        spindle_rpm = params.get("spindle_rpm")
        lateral_rpm = params.get("lateral_rpm")
        if not isinstance(duration_s, (int, float)) or not isinstance(spindle_rpm, (int, float)) or not isinstance(lateral_rpm, (int, float)):
            raise JsonRpcError(
                -32602,
                "Invalid params: duration_s, spindle_rpm and lateral_rpm must be numbers",
            )

        command = MotionCommand(
            duration_s=float(duration_s),
            spindle_rpm=float(spindle_rpm),
            lateral_rpm=float(lateral_rpm),
        )
        return self._run_motion(command)

    def _run_motion(self, command: MotionCommand) -> dict[str, Any]:
        if self.app is None:
            return {"error": "no application attached"}

        result = self.app.run_multi_axis(
            command.duration_s,
            command.spindle_rpm,
            command.lateral_rpm,
        )
        return {
            "status": "motion_started",
            "details": result,
        }

    def _run_spindle(self, command: SpindleCommand) -> dict[str, Any]:
        if self.app is None:
            return {"error": "no application attached"}

        result = self.app.run_spindle(command.duration_s, command.rpm)
        return {
            "status": "spindle_started",
            "details": result,
        }

    def operation_status(self) -> dict[str, Any]:
        if self.app is None or not hasattr(self.app, "current_operation_status"):
            return {"error": "operation status not available"}
        return self.app.current_operation_status()

    def operation_stop(self) -> dict[str, Any]:
        if self.app is None or not hasattr(self.app, "stop_operation"):
            return {"error": "operation stop not available"}
        return self.app.stop_operation()

    def session_status(self) -> dict[str, Any]:
        if self.app is None or not hasattr(self.app, "current_operation_status"):
            return {"error": "operation status not available"}
        return self.app.current_operation_status()

    def session_wait(self, params: Any | None) -> dict[str, Any]:
        if self.app is None or not hasattr(self.app, "current_operation_status"):
            return {"error": "operation status not available"}

        timeout_s = 300.0
        if params is not None and isinstance(params, dict):
            timeout_value = params.get("timeout_s")
            if isinstance(timeout_value, (int, float)):
                timeout_s = float(timeout_value)

        start = time.monotonic()
        while True:
            status = self.app.current_operation_status()
            if status.get("status") in ("completed", "failed", "idle"):
                return status
            if time.monotonic() - start > timeout_s:
                raise JsonRpcError(-32000, f"Operation wait timed out after {timeout_s}s")
            time.sleep(0.05)
