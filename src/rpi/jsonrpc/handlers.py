from __future__ import annotations

import dataclasses
import time
from typing import Any, Callable, Dict

from .protocol import JsonRpcMethodNotFoundError

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
        if params is None:
            return callback()
        if isinstance(params, list):
            return callback(*params)
        if isinstance(params, dict):
            try:
                return callback(**params)
            except TypeError:
                return callback(params)
        return callback(params)


class AppRpcHandler(RpcHandler):
    def __init__(self, app: Any | None = None) -> None:
        super().__init__()
        self.app = app
        self.started_at = time.time()
        self.register_method("winder.ping", self._rpc_ping)
        self.register_method("winder.status", self._rpc_status)
        self.register_method("winder.shutdown", self._rpc_shutdown)
        self.register_method("winder.config", self._rpc_config)

    def _rpc_ping(self, _params: Any | None = None) -> dict[str, str]:
        return self.ping()

    def _rpc_status(self, _params: Any | None = None) -> dict[str, Any]:
        return self.status()

    def _rpc_shutdown(self, _params: Any | None = None) -> dict[str, str]:
        return self.shutdown()

    def _rpc_config(self, _params: Any | None = None) -> dict[str, Any]:
        return self.config()

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
            return {}
        cfg = getattr(self.app, "config", None) or getattr(self.app, "_config", None)
        if cfg is None:
            return {}

        if dataclasses.is_dataclass(cfg):
            cfg_dict = dataclasses.asdict(cfg)
            return {k: v for k, v in cfg_dict.items() if not k.startswith("_")}

        if hasattr(cfg, "__dict__"):
            return {
                k: v for k, v in vars(cfg).items()
                if not k.startswith("_")
            }

        result: dict[str, Any] = {}
        for attr in dir(cfg):
            if attr.startswith("_"):
                continue
            try:
                value = getattr(cfg, attr)
            except Exception:
                continue
            if callable(value):
                continue
            result[attr] = value
        return result

