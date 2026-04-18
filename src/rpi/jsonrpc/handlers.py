from __future__ import annotations

import time
from typing import Any

from .protocol import JsonRpcError, JsonRpcMethodNotFoundError


class RpcHandler:
    def dispatch(self, method: str, params: Any | None) -> Any:
        raise NotImplementedError("RpcHandler.dispatch must be implemented")


class AppRpcHandler(RpcHandler):
    def __init__(self, app: Any | None = None) -> None:
        self.app = app
        self.started_at = time.time()

    def dispatch(self, method: str, params: Any | None) -> Any:
        if method == "winder.ping":
            return self.ping()
        if method == "winder.status":
            return self.status()
        if method == "winder.shutdown":
            return self.shutdown()
        if method == "winder.config":
            return self.config()
        raise JsonRpcMethodNotFoundError(method)

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
        return {
            "spindle_max_speed_rpm": getattr(self.app, "spindle_max_speed_rpm", None),
            "lateral_max_rpm": getattr(self.app, "lateral_max_rpm", None),
        }
