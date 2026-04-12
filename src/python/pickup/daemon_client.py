"""Daemon client for PickupWinder (packaged).

Copied into the `pickup` subpackage to allow stable imports from the
application entrypoint (`pickup_test.py`).
"""

import json
import socket
import time
from typing import Any, Dict, Optional

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
