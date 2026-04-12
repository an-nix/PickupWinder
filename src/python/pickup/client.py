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


class DaemonClient:
    """Synchronous JSON-line client for pickup_daemon."""

    def __init__(self, path: str = SOCKET_PATH, timeout: float = DEFAULT_TIMEOUT):
        self.path = path
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._buf = ""

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.settimeout(self.timeout)
        self._sock.connect(self.path)

    def send(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        if self._sock is None:
            raise ConnectionError("Daemon client is not connected")

        payload = json.dumps(cmd, separators=(",", ":")) + "\n"
        self._sock.sendall(payload.encode())
        return self._read_json()

    def wait_event(self,
                   event_type: Optional[str] = None,
                   timeout: float = 5.0) -> Optional[Dict[str, Any]]:
        if self._sock is None:
            raise ConnectionError("Daemon client is not connected")

        deadline = time.monotonic() + timeout
        self._sock.settimeout(min(self.timeout, 0.1))
        try:
            while time.monotonic() < deadline:
                try:
                    obj = self._read_json()
                except socket.timeout:
                    continue
                if "event" not in obj:
                    continue
                if event_type is None or obj["event"] == event_type:
                    return obj
        finally:
            self._sock.settimeout(self.timeout)
        return None

    def drain_events(self, timeout: float = 0.1) -> None:
        if self._sock is None:
            return

        self._sock.settimeout(min(self.timeout, timeout))
        try:
            while True:
                self._read_json()
        except (socket.timeout, OSError, ConnectionError):
            pass
        finally:
            if self._sock is not None:
                self._sock.settimeout(self.timeout)

    def _read_json(self) -> Dict[str, Any]:
        if self._sock is None:
            raise ConnectionError("Daemon client is not connected")

        while "\n" not in self._buf:
            chunk = self._sock.recv(4096).decode(errors="replace")
            if not chunk:
                raise ConnectionError("Daemon closed connection")
            self._buf += chunk

        line, self._buf = self._buf.split("\n", 1)
        return json.loads(line.strip())

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


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
