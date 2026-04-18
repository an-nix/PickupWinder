from __future__ import annotations

import os
import socket
import threading
from pathlib import Path
from typing import Any

from .handlers import RpcHandler
from .protocol import JsonRpcError, JsonRpcInvalidRequestError, JsonRpcMethodNotFoundError, JsonRpcRequest, make_error_response, make_response, parse_json_rpc


class UnixJsonRpcServer:
    def __init__(self, socket_path: str, handler: RpcHandler, backlog: int = 5) -> None:
        self.socket_path = Path(socket_path)
        self.handler = handler
        self.backlog = backlog
        self._server_socket: socket.socket | None = None
        self._accept_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def start(self) -> None:
        if self._server_socket is not None:
            return
        if self.socket_path.exists():
            self.socket_path.unlink()

        self._server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server_socket.bind(str(self.socket_path))
        self._server_socket.listen(self.backlog)
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except Exception:
                pass
            self._server_socket = None
        if self._accept_thread is not None:
            self._accept_thread.join(timeout=1.0)
            self._accept_thread = None
        if self.socket_path.exists():
            try:
                self.socket_path.unlink()
            except Exception:
                pass

    def _accept_loop(self) -> None:
        assert self._server_socket is not None
        while not self._stop_event.is_set():
            try:
                connection, _ = self._server_socket.accept()
            except OSError:
                break
            thread = threading.Thread(target=self._handle_connection, args=(connection,), daemon=True)
            thread.start()

    def _handle_connection(self, connection: socket.socket) -> None:
        with connection:
            file = connection.makefile(mode="r", encoding="utf-8", newline="\n")
            for raw_line in file:
                line = raw_line.strip()
                if not line or self._stop_event.is_set():
                    continue
                self._process_message(connection, line)

    def _process_message(self, connection: socket.socket, raw_message: str) -> None:
        try:
            request = parse_json_rpc(raw_message)
        except JsonRpcError as exc:
            payload = make_error_response(exc, None)
            self._send_payload(connection, payload)
            return

        if request.id is None:
            # Notification: no response expected.
            try:
                self.handler.dispatch(request.method, request.params)
            except JsonRpcError:
                pass
            return

        try:
            result = self.handler.dispatch(request.method, request.params)
            payload = make_response(result, request.id)
        except JsonRpcError as exc:
            payload = make_error_response(exc, request.id)
        except Exception as exc:
            payload = make_error_response(JsonRpcError(-32000, str(exc)), request.id)

        self._send_payload(connection, payload)

    def _send_payload(self, connection: socket.socket, payload: str) -> None:
        try:
            connection.sendall(payload.encode("utf-8") + b"\n")
        except OSError:
            pass
