from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

JSONRPC_VERSION = "2.0"


class JsonRpcError(Exception):
    def __init__(self, code: int, message: str, data: Any | None = None) -> None:
        self.code = code
        self.message = message
        self.data = data
        super().__init__(message)

    def to_dict(self) -> dict[str, Any]:
        error: dict[str, Any] = {
            "code": self.code,
            "message": self.message,
        }
        if self.data is not None:
            error["data"] = self.data
        return error


class JsonRpcParseError(JsonRpcError):
    def __init__(self, message: str) -> None:
        super().__init__(-32700, message)


class JsonRpcInvalidRequestError(JsonRpcError):
    def __init__(self, message: str) -> None:
        super().__init__(-32600, message)


class JsonRpcMethodNotFoundError(JsonRpcError):
    def __init__(self, method: str) -> None:
        super().__init__(-32601, f"Method not found: {method}")


def parse_json_rpc(raw_payload: str) -> dict[str, Any]:
    try:
        message = json.loads(raw_payload)
    except json.JSONDecodeError as exc:
        raise JsonRpcParseError(f"Parse error: {exc}") from exc

    if not isinstance(message, dict):
        raise JsonRpcInvalidRequestError("Invalid request: JSON-RPC message must be an object")
    if message.get("jsonrpc") != JSONRPC_VERSION:
        raise JsonRpcInvalidRequestError("Invalid request: jsonrpc must be '2.0'")
    if "method" not in message:
        raise JsonRpcInvalidRequestError("Invalid request: missing method")

    return message


def make_response(result: Any, request_id: Any) -> str:
    return json.dumps({"jsonrpc": JSONRPC_VERSION, "result": result, "id": request_id})


def make_error_response(error: JsonRpcError, request_id: Any | None) -> str:
    payload: dict[str, Any] = {
        "jsonrpc": JSONRPC_VERSION,
        "error": error.to_dict(),
        "id": request_id,
    }
    return json.dumps(payload)


def make_request(method: str, params: Any | None = None, request_id: Any | None = None) -> str:
    message: dict[str, Any] = {
        "jsonrpc": JSONRPC_VERSION,
        "method": method,
    }
    if params is not None:
        message["params"] = params
    if request_id is not None:
        message["id"] = request_id
    return json.dumps(message)


@dataclass
class Notification:
    method: str
    params: Any | None = None


class UnixJsonRpcClient:
    def __init__(self, socket_path: str, timeout_s: float = 5.0) -> None:
        self.socket_path = Path(socket_path)
        self.timeout_s = timeout_s
        self._sock: socket.socket | None = None
        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._pending: dict[Any, tuple[threading.Event, dict[str, Any] | None]] = {}
        self._lock = threading.Lock()
        self._notification_callback: Callable[[dict[str, Any]], None] | None = None
        self._buffer = ""

    def set_notification_callback(self, callback: Callable[[dict[str, Any]], None]) -> None:
        self._notification_callback = callback

    def connect(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout_s)
        sock.connect(str(self.socket_path))
        sock.settimeout(1.0)
        self._sock = sock
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True,
            name="wendy_rpc_reader",
        )
        self._reader_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None

    def send_raw(self, raw_payload: str, timeout_s: float | None = None) -> dict[str, Any] | None:
        if self._sock is None:
            self.connect()
        request = parse_json_rpc(raw_payload)
        request_id = request.get("id")
        if not raw_payload.endswith("\n"):
            raw_payload = raw_payload + "\n"

        if request_id is None:
            self._send(raw_payload)
            return None

        event = threading.Event()
        with self._lock:
            self._pending[request_id] = (event, None)
            self._send(raw_payload)

        deadline = time.monotonic() + (timeout_s if timeout_s is not None else self.timeout_s)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if event.wait(timeout=remaining):
                with self._lock:
                    _, response = self._pending.pop(request_id, (None, None))
                if response is None:
                    raise RuntimeError(f"Missing response for request id {request_id}")
                return response

        with self._lock:
            self._pending.pop(request_id, None)
        raise RuntimeError(f"JSON-RPC request timeout for id {request_id}")

    def _send(self, raw_payload: str) -> None:
        if self._sock is None:
            raise RuntimeError("RPC client is not connected")
        self._sock.sendall(raw_payload.encode("utf-8"))

    def _reader_loop(self) -> None:
        assert self._sock is not None
        while not self._stop_event.is_set():
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            self._buffer += chunk.decode("utf-8", errors="replace")
            while "\n" in self._buffer:
                line, self._buffer = self._buffer.split("\n", 1)
                if not line.strip():
                    continue
                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    continue
                self._handle_incoming(message)

    def _handle_incoming(self, message: dict[str, Any]) -> None:
        if "id" in message:
            request_id = message["id"]
            with self._lock:
                pending = self._pending.get(request_id)
                if pending is not None:
                    event, _ = pending
                    self._pending[request_id] = (event, message)
                    event.set()
            return

        if self._notification_callback is not None and "method" in message:
            self._notification_callback(message)
