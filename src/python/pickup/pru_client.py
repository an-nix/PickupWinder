"""Async PruClient (packaged).

Copy of the async client placed in the `pickup` package to keep imports
consistent for the application entrypoint.
"""

import asyncio
import json
from typing import Awaitable, Callable, Optional

SOCKET_PATH = "/run/pickup-winder.sock"

MODE_FREE    = "free"
MODE_WINDING = "winding"


class PruClient:
    """Async client for pickup_daemon (packaged).

    Minimal copy for completeness; full implementation lives in the top-level
    `pru_client.py` too.
    """

    def __init__(self, socket_path: str = SOCKET_PATH):
        self._path = socket_path
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._event_cb: Optional[Callable[[dict], Awaitable[None]]] = None
        self._running = False
        self._cmd_lock = asyncio.Lock()

    async def connect(self):
        self._reader, self._writer = await asyncio.open_unix_connection(self._path)
        self._running = True

    async def disconnect(self):
        self._running = False
        if self._writer:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass

    def on_event(self, callback: Callable[[dict], Awaitable[None]]):
        self._event_cb = callback

    async def start_event_listener(self):
        while self._running:
            try:
                line = await self._reader.readline()
                if not line:
                    self._running = False
                    break
                msg = json.loads(line.decode().strip())
                if "event" in msg and self._event_cb:
                    await self._event_cb(msg)
            except Exception:
                break

    async def _send(self, obj: dict) -> dict:
        async with self._cmd_lock:
            payload = json.dumps(obj, separators=(",", ":")) + "\n"
            self._writer.write(payload.encode())
            await self._writer.drain()
            try:
                resp_line = await asyncio.wait_for(self._reader.readline(), timeout=2.0)
                return json.loads(resp_line.decode().strip())
            except asyncio.TimeoutError:
                return {"ok": False, "error": "timeout"}
            except json.JSONDecodeError:
                return {"ok": False, "error": "bad json response"}
