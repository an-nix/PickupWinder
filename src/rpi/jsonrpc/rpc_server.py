from __future__ import annotations

import json
import logging
import os
import socket
import threading
from typing import Any

from core.events import EventBus, EventKind
from .handlers import RpcHandler
from .protocol import JsonRpcMethodNotFoundError

logger = logging.getLogger(__name__)


class JsonRpcServer:
    """
    JSON-RPC 2.0 server over a Unix domain socket.

    One thread accepts connections (one client at a time).
    One thread drains the EventBus and pushes notifications to the
    connected client as JSON-RPC notifications (no id field).

    The server contains NO business logic. It forwards JSON-RPC
    method calls to a registered RpcHandler.

    Supported methods are defined by the handler registered with the server.

    Notifications pushed to client:
      winding.event  params: {kind, data}
    """

    BUFFER_SIZE = 65536

    def __init__(
        self,
        handler: RpcHandler,
        event_bus: EventBus,
        socket_path: str = "/tmp/winding.sock",
    ) -> None:
        self._handler = handler
        self._events = event_bus
        self._socket_path = socket_path

        self._accept_thread: threading.Thread | None = None
        self._notify_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._client_lock = threading.Lock()
        self._current_client: socket.socket | None = None

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def start(self) -> None:
        if os.path.exists(self._socket_path):
            os.unlink(self._socket_path)
        self._accept_thread = threading.Thread(
            target=self._accept_loop,
            daemon=True,
            name="rpc_accept",
        )
        self._notify_thread = threading.Thread(
            target=self._notify_loop,
            daemon=True,
            name="rpc_notify",
        )
        self._accept_thread.start()
        self._notify_thread.start()
        logger.info("JSON-RPC server listening on %s", self._socket_path)

    def stop(self) -> None:
        self._stop_event.set()
        if os.path.exists(self._socket_path):
            os.unlink(self._socket_path)

    # ── Accept loop ────────────────────────────────────────────────────────

    def _accept_loop(self) -> None:
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(self._socket_path)
        srv.listen(1)
        srv.settimeout(1.0)
        while not self._stop_event.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._client_lock:
                self._current_client = conn
            try:
                self._handle_client(conn)
            finally:
                with self._client_lock:
                    self._current_client = None
                conn.close()
        srv.close()

    def _handle_client(self, conn: socket.socket) -> None:
        """Read newline-delimited JSON-RPC requests from one client."""
        buf = b""
        conn.settimeout(0.5)
        while not self._stop_event.is_set():
            try:
                chunk = conn.recv(self.BUFFER_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.strip():
                    response = self._dispatch(line.decode("utf-8"))
                    if response is not None:
                        try:
                            conn.sendall((json.dumps(response) + "\n").encode())
                        except OSError:
                            return

    # ── Notify loop ────────────────────────────────────────────────────────

    def _notify_loop(self) -> None:
        """Drain EventBus and push notifications to connected client."""
        while not self._stop_event.is_set():
            event = self._events.consume(timeout_s=0.1)
            if event is None:
                continue
            notification = {
                "jsonrpc": "2.0",
                "method": "winding.event",
                "params": {
                    "kind": event.kind.name,
                    "data": event.data,
                },
            }
            with self._client_lock:
                client = self._current_client
            if client is not None:
                try:
                    client.sendall((json.dumps(notification) + "\n").encode())
                except OSError:
                    pass

    # ── JSON-RPC dispatch ──────────────────────────────────────────────────

    def _dispatch(self, raw: str) -> dict | None:
        """Parse one JSON-RPC request and return a response dict or None."""
        try:
            req = json.loads(raw)
        except json.JSONDecodeError as exc:
            return self._error_response(None, -32700, f"Parse error: {exc}")

        req_id = req.get("id")
        method = req.get("method", "")
        params = req.get("params", None)

        try:
            result = self._handler.dispatch(method, params)
        except JsonRpcMethodNotFoundError as exc:
            return self._error_response(req_id, -32601, str(exc))
        except TypeError as exc:
            return self._error_response(req_id, -32602, f"Invalid params: {exc}")
        except Exception as exc:
            logger.exception("RPC method %s raised", method)
            return self._error_response(req_id, -32000, str(exc))

        # Notifications (no id) get no response.
        if req_id is None:
            return None
        return {"jsonrpc": "2.0", "id": req_id, "result": result}

    @staticmethod
    def _error_response(req_id: Any, code: int, message: str) -> dict:
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "error": {"code": code, "message": message},
        }

