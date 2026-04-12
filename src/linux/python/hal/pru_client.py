"""Unified client module: synchronous and asynchronous clients.

Contains `DaemonClient` (synchronous JSON-line Unix socket client) and
`PruClient` (asyncio-based client). Both talk to the same `pickup_daemon`.
Keeping them together avoids duplication and clarifies that both are just
front-ends to the daemon (not direct PRU access).
"""

import json
import socket
import time
import asyncio
from typing import Any, Dict, Optional, Awaitable, Callable

SOCKET_PATH = "/run/pickup-winder.sock"
DEFAULT_TIMEOUT = 2.0



class PruClient:
    """Async client for pickup_daemon (asyncio).

    Provides the same commands as the synchronous client but in coroutine
    form. Useful for UIs or services that need a non-blocking API.
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

    @property
    def is_connected(self) -> bool:
        return self._running and self._writer is not None

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
            except (asyncio.IncompleteReadError, ConnectionError):
                self._running = False
                break
            except json.JSONDecodeError:
                continue
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

    # Convenience command wrappers (examples)
    async def set_speed(self, sp_hz: int = 0, lat_hz: int = 0) -> bool:
        r = await self._send({"cmd": "set_speed", "sp_hz": sp_hz, "lat_hz": lat_hz})
        return bool(r.get("ok"))

    async def enable(self, sp: bool = False, lat: bool = False) -> bool:
        r = await self._send({"cmd": "enable", "sp": int(sp), "lat": int(lat)})
        return bool(r.get("ok"))

    async def emergency_stop(self) -> bool:
        r = await self._send({"cmd": "e_stop"})
        return bool(r.get("ok"))
