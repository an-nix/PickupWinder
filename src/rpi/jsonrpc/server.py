from __future__ import annotations

import os
import socket
import threading
from pathlib import Path
from typing import Any
import asyncio

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


class AsyncUnixJsonRpcServer:
    """Asyncio-based Unix-socket JSON-RPC server.

    Uses asyncio streams and runs handler dispatches in the default executor
    so synchronous handler implementations won't block the event loop.
    """

    def __init__(self, socket_path: str, handler: RpcHandler, backlog: int = 5) -> None:
        self.socket_path = Path(socket_path)
        self.handler = handler
        self.backlog = backlog
        self._server: asyncio.AbstractServer | None = None

    async def start(self) -> None:
        if self._server is not None:
            return
        if self.socket_path.exists():
            try:
                self.socket_path.unlink()
            except Exception:
                pass

        self._server = await asyncio.start_unix_server(self._handle_client, str(self.socket_path), backlog=self.backlog)

    async def stop(self) -> None:
        if self._server is None:
            return
        self._server.close()
        try:
            await self._server.wait_closed()
        except Exception:
            pass
        self._server = None
        if self.socket_path.exists():
            try:
                self.socket_path.unlink()
            except Exception:
                pass

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            while not reader.at_eof():
                raw = await reader.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                await self._process_message(line, writer)
        except asyncio.CancelledError:
            pass
        except Exception:
            pass
        finally:
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

    async def _process_message(self, raw_message: str, writer: asyncio.StreamWriter) -> None:
        loop = asyncio.get_running_loop()
        try:
            request = parse_json_rpc(raw_message)
        except JsonRpcError as exc:
            payload = make_error_response(exc, None)
            await self._send_payload(writer, payload)
            return

        # Notifications: no response expected — dispatch in executor and forget
        if request.id is None:
            loop.run_in_executor(None, self.handler.dispatch, request.method, request.params)
            return

        try:
            result = await loop.run_in_executor(None, self.handler.dispatch, request.method, request.params)
            payload = make_response(result, request.id)
        except JsonRpcError as exc:
            payload = make_error_response(exc, request.id)
        except Exception as exc:
            payload = make_error_response(JsonRpcError(-32000, str(exc)), request.id)

        await self._send_payload(writer, payload)

    async def _send_payload(self, writer: asyncio.StreamWriter, payload: str) -> None:
        try:
            writer.write(payload.encode("utf-8") + b"\n")
            await writer.drain()
        except Exception:
            pass
