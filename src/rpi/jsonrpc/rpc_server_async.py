from __future__ import annotations

import os
import socket
import threading
from pathlib import Path
from typing import Any
import asyncio

from .handlers import RpcHandler
from .protocol import JsonRpcError, JsonRpcInvalidRequestError, JsonRpcMethodNotFoundError, JsonRpcRequest, make_error_response, make_response, parse_json_rpc


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
