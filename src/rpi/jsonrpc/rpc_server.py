from __future__ import annotations

import json
import logging
import os
import socket
import stat
import threading
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeout

from core.events import EventBus, EventKind

from .handlers import RpcHandler
from .protocol import JsonRpcError, make_error_response, make_response, parse_json_rpc

logger = logging.getLogger(__name__)


class JsonRpcServer:
    """JSON-RPC 2.0 server over a Unix domain socket."""

    BUFFER_SIZE = 65536
    MAX_REQUEST_BYTES = 262144
    RPC_CALL_TIMEOUT_S = 30.0
    CLIENT_RECV_TIMEOUT_S = 0.5

    def __init__(
        self,
        handler: RpcHandler,
        event_bus: EventBus,
        socket_path: str = "/tmp/winding.sock",
        *,
        rpc_call_timeout_s: float = RPC_CALL_TIMEOUT_S,
        max_request_bytes: int = MAX_REQUEST_BYTES,
    ) -> None:
        self._handler = handler
        self._events = event_bus
        self._socket_path = socket_path
        self._rpc_call_timeout_s = max(0.1, float(rpc_call_timeout_s))
        self._max_request_bytes = max(1024, int(max_request_bytes))

        self._accept_thread: threading.Thread | None = None
        self._notify_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._client_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._current_client: socket.socket | None = None
        self._server_socket: socket.socket | None = None
        self._call_executor = ThreadPoolExecutor(
            max_workers=4,
            thread_name_prefix="rpc_call",
        )
        self._accept_worker_error: str | None = None
        self._notify_worker_error: str | None = None
        self._timed_out_requests: int = 0

    def start(self) -> None:
        self._safe_remove_socket_path()

        self._stop_event.clear()
        self._accept_worker_error = None
        self._notify_worker_error = None
        self._timed_out_requests = 0
        self._server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server_socket.bind(self._socket_path)
        self._server_socket.listen(1)
        self._server_socket.settimeout(1.0)

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

        with self._client_lock:
            client = self._current_client
            self._current_client = None
        if client is not None:
            try:
                client.close()
            except OSError:
                pass

        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
            self._server_socket = None

        if self._accept_thread is not None:
            self._accept_thread.join(timeout=1.0)
        if self._notify_thread is not None:
            self._notify_thread.join(timeout=1.0)

        self._call_executor.shutdown(wait=False, cancel_futures=True)
        self._safe_remove_socket_path()

        errors: list[str] = []
        if self._accept_thread is not None and self._accept_thread.is_alive():
            errors.append("rpc_accept thread did not stop cleanly")
        if self._notify_thread is not None and self._notify_thread.is_alive():
            errors.append("rpc_notify thread did not stop cleanly")
        if errors:
            raise RuntimeError("; ".join(errors))

    def health_status(self) -> dict[str, object]:
        return {
            "accept_thread_alive": self._accept_thread is not None and self._accept_thread.is_alive(),
            "notify_thread_alive": self._notify_thread is not None and self._notify_thread.is_alive(),
            "accept_thread_faulted": self._accept_worker_error is not None,
            "notify_thread_faulted": self._notify_worker_error is not None,
            "accept_thread_error": self._accept_worker_error,
            "notify_thread_error": self._notify_worker_error,
            "timed_out_requests": self._timed_out_requests,
        }

    def _safe_remove_socket_path(self) -> None:
        if not os.path.exists(self._socket_path):
            return
        file_stat = os.lstat(self._socket_path)
        if not stat.S_ISSOCK(file_stat.st_mode):
            raise RuntimeError(
                f"Refusing to unlink non-socket path at {self._socket_path}"
            )
        os.unlink(self._socket_path)

    def _record_worker_failure(self, worker_name: str, exc: Exception) -> None:
        error = str(exc)
        if worker_name == "rpc_accept":
            self._accept_worker_error = error
        elif worker_name == "rpc_notify":
            self._notify_worker_error = error
        logger.exception("%s worker failed", worker_name)
        self._events.publish(EventKind.WORKER_FAILED, worker=worker_name, error=error)

    def _send_text(self, conn: socket.socket, payload: str) -> bool:
        try:
            with self._write_lock:
                conn.sendall((payload + "\n").encode("utf-8"))
            return True
        except OSError:
            return False

    def _accept_loop(self) -> None:
        server_socket = self._server_socket
        if server_socket is None:
            return

        try:
            while not self._stop_event.is_set():
                try:
                    conn, _ = server_socket.accept()
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
        except Exception as exc:
            self._record_worker_failure("rpc_accept", exc)

    def _handle_client(self, conn: socket.socket) -> None:
        buf = b""
        conn.settimeout(self.CLIENT_RECV_TIMEOUT_S)
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
            if len(buf) > self._max_request_bytes:
                self._send_text(
                    conn,
                    make_error_response(
                        JsonRpcError(
                            -32600,
                            f"Request too large (>{self._max_request_bytes} bytes)",
                        ),
                        None,
                    ),
                )
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    raw_line = line.decode("utf-8")
                except UnicodeDecodeError as exc:
                    if not self._send_text(conn, make_error_response(JsonRpcError(-32700, str(exc)), None)):
                        return
                    continue
                response = self._dispatch(raw_line)
                if response is None:
                    continue
                if not self._send_text(conn, response):
                    return

    def _notify_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                event = self._events.consume(timeout_s=0.1)
                if event is None:
                    continue
                notification = {
                    "jsonrpc": "2.0",
                    "method": "winding.event",
                    "params": {
                        "version": event.version,
                        "kind": event.kind.name,
                        "data": event.data,
                    },
                }
                with self._client_lock:
                    client = self._current_client
                if client is None:
                    continue
                self._send_text(client, json.dumps(notification))
        except Exception as exc:
            self._record_worker_failure("rpc_notify", exc)

    def _dispatch(self, raw: str) -> str | None:
        try:
            request = parse_json_rpc(raw)
        except JsonRpcError as exc:
            return make_error_response(exc, None)

        future = self._call_executor.submit(
            self._handler.dispatch,
            request.method,
            request.params,
        )
        try:
            result = future.result(timeout=self._rpc_call_timeout_s)
        except FuturesTimeout:
            self._timed_out_requests += 1
            future.cancel()
            logger.error(
                "RPC method %s timed out after %.1fs",
                request.method,
                self._rpc_call_timeout_s,
            )
            return make_error_response(
                JsonRpcError(
                    -32000,
                    f"RPC method {request.method} timed out after {self._rpc_call_timeout_s:.1f}s",
                ),
                request.id,
            )
        except JsonRpcError as exc:
            return make_error_response(exc, request.id)
        except Exception as exc:
            logger.exception("RPC method %s raised", request.method)
            return make_error_response(JsonRpcError(-32000, str(exc)), request.id)

        if request.id is None:
            return None
        return make_response(result, request.id)

