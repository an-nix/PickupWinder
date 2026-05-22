from __future__ import annotations

import inspect
import time
from typing import Any, Callable

from core.status import RuntimeStatusService, serialize_configuration

from .protocol import JsonRpcInvalidParamsError, JsonRpcMethodNotFoundError

MethodCallback = Callable[..., Any]


class RpcHandler:
    def __init__(self) -> None:
        self._methods: dict[str, MethodCallback] = {}

    def register_method(self, method: str, callback: MethodCallback) -> None:
        self._methods[method] = callback

    def dispatch(self, method: str, params: Any | None) -> Any:
        callback = self._methods.get(method)
        if callback is None:
            raise JsonRpcMethodNotFoundError(method)
        signature = inspect.signature(callback)
        try:
            if params is None:
                signature.bind()
                return callback()
            if isinstance(params, list):
                signature.bind(*params)
                return callback(*params)
            if isinstance(params, dict):
                signature.bind(**params)
                return callback(**params)
            signature.bind(params)
            return callback(params)
        except TypeError as exc:
            raise JsonRpcInvalidParamsError(str(exc)) from exc


class SystemRpcHandler(RpcHandler):
    def __init__(
        self,
        *,
        status_service: RuntimeStatusService | None = None,
        app: Any | None = None,
    ) -> None:
        super().__init__()
        self._status_service = status_service
        self._app = app
        self._started_at = time.monotonic()
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
        if self._status_service is not None:
            return self._status_service.application_status()
        return {
            "uptime_s": round(time.monotonic() - self._started_at, 2),
            "configured": bool(self._app is not None),
        }

    def shutdown(self) -> dict[str, str]:
        return {"message": "shutdown-not-implemented"}

    def config(self) -> dict[str, Any]:
        if self._status_service is not None:
            return self._status_service.configuration_status()
        if self._app is None:
            return {}

        config = getattr(self._app, "config", None) or getattr(self._app, "_config", None)
        if config is None:
            return {}
        return serialize_configuration(config)

