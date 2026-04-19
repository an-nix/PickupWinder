from __future__ import annotations

import json
import socket
from pathlib import Path
from typing import Any

from .protocol import JSONRPC_VERSION


class UnixJsonRpcClient:
    def __init__(self, socket_path: str, timeout_s: float = 5.0) -> None:
        self.socket_path = Path(socket_path)
        self.timeout_s = timeout_s
        self._closed = False

    def close(self) -> None:
        self._closed = True

    def call(self, method: str, params: Any | None = None, request_id: int = 1) -> Any:
        if self._closed:
            raise RuntimeError("JSON-RPC client is closed")
        payload = {
            "jsonrpc": JSONRPC_VERSION,
            "method": method,
            "params": params,
            "id": request_id,
        }
        encoded = json.dumps(payload).encode("utf-8") + b"\n"

        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(self.timeout_s)
            sock.connect(str(self.socket_path))
            sock.sendall(encoded)
            response = self._read_response(sock)

        return self._parse_response(response)

    def notify(self, method: str, params: Any | None = None) -> None:
        if self._closed:
            raise RuntimeError("JSON-RPC client is closed")
        payload = {
            "jsonrpc": JSONRPC_VERSION,
            "method": method,
            "params": params,
        }
        encoded = json.dumps(payload).encode("utf-8") + b"\n"

        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(self.timeout_s)
            sock.connect(str(self.socket_path))
            sock.sendall(encoded)

    def _read_response(self, sock: socket.socket) -> str:
        data = b""
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            data += chunk
            if b"\n" in chunk:
                break
        return data.decode("utf-8").strip()

    def _parse_response(self, raw: str) -> Any:
        try:
            message = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid JSON-RPC response: {exc}") from exc

        if "error" in message and message["error"] is not None:
            raise RuntimeError(f"JSON-RPC error {message['error']}")
        return message.get("result")
