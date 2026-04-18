from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Mapping

JSONRPC_VERSION = "2.0"


class JsonRpcError(Exception):
    def __init__(self, code: int, message: str, data: Any | None = None) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.data = data


class JsonRpcParseError(JsonRpcError):
    def __init__(self, data: Any | None = None) -> None:
        super().__init__(-32700, "Parse error", data)


class JsonRpcInvalidRequestError(JsonRpcError):
    def __init__(self, data: Any | None = None) -> None:
        super().__init__(-32600, "Invalid Request", data)


class JsonRpcMethodNotFoundError(JsonRpcError):
    def __init__(self, method: str) -> None:
        super().__init__(-32601, f"Method not found: {method}", {"method": method})


@dataclass
class JsonRpcRequest:
    method: str
    params: Any | None
    id: Any | None


@dataclass
class JsonRpcResponse:
    result: Any | None = None
    error: dict[str, Any] | None = None
    id: Any | None = None


def parse_json_rpc(payload: str) -> JsonRpcRequest:
    try:
        message = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise JsonRpcParseError(str(exc)) from exc

    if not isinstance(message, dict):
        raise JsonRpcInvalidRequestError(message)

    if message.get("jsonrpc") != JSONRPC_VERSION:
        raise JsonRpcInvalidRequestError(message)

    if "method" not in message or not isinstance(message["method"], str):
        raise JsonRpcInvalidRequestError(message)

    return JsonRpcRequest(
        method=message["method"],
        params=message.get("params"),
        id=message.get("id"),
    )


def make_response(result: Any, request_id: Any | None) -> str:
    return json.dumps({"jsonrpc": JSONRPC_VERSION, "result": result, "id": request_id})


def make_error_response(error: JsonRpcError, request_id: Any | None) -> str:
    payload = {
        "jsonrpc": JSONRPC_VERSION,
        "error": {
            "code": error.code,
            "message": error.message,
        },
        "id": request_id,
    }
    if error.data is not None:
        payload["error"]["data"] = error.data
    return json.dumps(payload)
