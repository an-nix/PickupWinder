"""pru_client.py — Async Python client for pickup_daemon (Option A architecture).

Architecture layer 4 — Python application talks to Layer 3 C daemon
over a Unix socket using newline-delimited JSON.

Protocol (Option A — continuous shared parameters + autonomous move_to):

  Python → daemon (commands):
    {"cmd":"set_speed",  "sp_hz":<uint>, "lat_hz":<uint>}
    {"cmd":"enable",     "axis":0|1|255, "value":0|1}
    {"cmd":"e_stop"}
    {"cmd":"home_start"}
    {"cmd":"reset_pos",  "axis":0|1|255}
    {"cmd":"ack_event"}
    {"cmd":"set_limits", "axis":1, "min":<int>, "max":<int>}
    {"cmd":"move_to",    "axis":1, "pos":<int>,
                          "start_hz":<uint>, "max_hz":<uint>,
                          "accel_steps":<uint>}

  daemon → Python (responses):
    {"ok":true}
    {"ok":false,"error":"<reason>"}

  daemon → Python (async events, unsolicited):
    {"event":"endstop_hit"}
    {"event":"home_complete"}
    {"event":"fault",         "sp_faults":<N>, "lat_faults":<N>}
    {"event":"limit_hit",     "axis":<N>, "pos":<N>}
    {"event":"move_complete", "pos":<N>}
    {"event":"telem", "pru1_state":<N>,
     "sp":  {"steps":<N>,"speed_hz":<N>,"faults":<N>},
     "lat": {"steps":<N>,"pos":<N>,"speed_hz":<N>,"faults":<N>},
     "endstop":<N>}

Usage:
    import asyncio
    from pru_client import PruClient

    async def main():
        client = PruClient()
        await client.connect()

        async def on_event(msg):
            print("event:", msg)

        client.on_event(on_event)
        asyncio.create_task(client.start_event_listener())

        await client.set_speed(sp_hz=6400, lat_hz=200)
        await client.enable(sp=True, lat=True)
        await asyncio.sleep(5)
        await client.emergency_stop()
        await client.disconnect()

    asyncio.run(main())
"""

import asyncio
import json
from typing import Awaitable, Callable, Optional

SOCKET_PATH = "/run/pickup-winder.sock"


class PruClient:
    """Async client for pickup_daemon (Option A — continuous shared params).

    All public methods are coroutines. Events from the daemon are dispatched
    via the callback registered with on_event().
    """

    def __init__(self, socket_path: str = SOCKET_PATH):
        self._path = socket_path
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._event_cb: Optional[Callable[[dict], Awaitable[None]]] = None
        self._running = False
        self._cmd_lock = asyncio.Lock()

    # ── Connection ────────────────────────────────────────────────────────

    async def connect(self):
        """Open connection to pickup_daemon."""
        self._reader, self._writer = await asyncio.open_unix_connection(
            self._path
        )
        self._running = True
        print(f"[PruClient] connected to {self._path}")

    async def disconnect(self):
        """Close the connection gracefully."""
        self._running = False
        if self._writer:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass
        print("[PruClient] disconnected")

    @property
    def is_connected(self) -> bool:
        return self._running and self._writer is not None

    # ── Event listener ────────────────────────────────────────────────────

    def on_event(self, callback: Callable[[dict], Awaitable[None]]):
        """Register an async callback for daemon-pushed events.

        Events: endstop_hit, fault, telem.
        """
        self._event_cb = callback

    async def start_event_listener(self):
        """Background coroutine — read unsolicited event lines.

        Run as: asyncio.create_task(client.start_event_listener())
        """
        while self._running:
            try:
                line = await self._reader.readline()
                if not line:
                    print("[PruClient] daemon closed connection")
                    self._running = False
                    break

                msg = json.loads(line.decode().strip())

                if "event" in msg and self._event_cb:
                    await self._event_cb(msg)

            except (asyncio.IncompleteReadError, ConnectionError):
                print("[PruClient] connection lost")
                self._running = False
                break
            except json.JSONDecodeError as e:
                print(f"[PruClient] JSON decode error: {e}")
            except Exception as e:
                print(f"[PruClient] event listener error: {e}")
                break

    # ── Internal ──────────────────────────────────────────────────────────

    async def _send(self, obj: dict) -> dict:
        """Send command JSON and wait for the {"ok":...} response."""
        async with self._cmd_lock:
            payload = json.dumps(obj, separators=(",", ":")) + "\n"
            self._writer.write(payload.encode())
            await self._writer.drain()
            try:
                resp_line = await asyncio.wait_for(
                    self._reader.readline(), timeout=2.0
                )
                return json.loads(resp_line.decode().strip())
            except asyncio.TimeoutError:
                return {"ok": False, "error": "timeout"}
            except json.JSONDecodeError:
                return {"ok": False, "error": "bad json response"}

    # ── Commands ──────────────────────────────────────────────────────────

    async def set_speed(self, sp_hz: int = 0, lat_hz: int = 0) -> bool:
        """Set spindle and lateral speeds in Hz.

        sp_hz:  spindle step frequency (0 = stopped)
        lat_hz: lateral step frequency (0 = stopped)
        """
        r = await self._send({
            "cmd": "set_speed", "sp_hz": sp_hz, "lat_hz": lat_hz
        })
        return bool(r.get("ok"))

    async def enable(self, sp: bool = False, lat: bool = False) -> bool:
        """Enable/disable stepper drivers.

        sp:  True to enable spindle driver
        lat: True to enable lateral driver
        """
        r = await self._send({
            "cmd": "enable", "sp": int(sp), "lat": int(lat)
        })
        return bool(r.get("ok"))

    async def emergency_stop(self) -> bool:
        """Immediate stop on both axes — disable drivers, zero speeds."""
        r = await self._send({"cmd": "e_stop"})
        return bool(r.get("ok"))

    async def home_start(self) -> bool:
        """Start lateral homing sequence (PRU1 orchestrates)."""
        r = await self._send({"cmd": "home_start"})
        return bool(r.get("ok"))

    async def reset_position(self) -> bool:
        """Reset step counters and positions to zero."""
        r = await self._send({"cmd": "reset_pos"})
        return bool(r.get("ok"))

    async def ack_event(self) -> bool:
        """Acknowledge last PRU event (clears event flags and limit locks)."""
        r = await self._send({"cmd": "ack_event"})
        return bool(r.get("ok"))

    async def set_limits(self, axis: int, min_steps: int,
                         max_steps: int) -> bool:
        """Set software position limits for an axis.

        axis:      0=spindle, 1=lateral
        min_steps: minimum allowed position (steps, signed)
        max_steps: maximum allowed position (steps, signed)

        If the axis position leaves the [min, max] range, PRU0 latches the
        axis off and sends a 'limit_hit' event. Call ack_event() to release.
        """
        r = await self._send({
            "cmd": "set_limits", "axis": axis,
            "min": min_steps, "max": max_steps,
        })
        return bool(r.get("ok"))

    async def move_to(self, pos: int, start_hz: int = 200,
                      max_hz: int = 4000, accel_steps: int = 300) -> bool:
        """Move the lateral axis to an absolute position with a trapezoidal
        speed profile.

        pos:         target position in steps (signed, relative to home=0)
        start_hz:    step frequency at the start and end of the ramp (slow)
        max_hz:      cruise step frequency (fast)
        accel_steps: number of steps for the acceleration / deceleration ramp

        The daemon converts Hz to IEP intervals and sends HOST_CMD_MOVE_TO.
        PRU1 executes the profile autonomously — no further Python interaction
        is needed until the 'move_complete' event is received.

        Returns True when the command is acknowledged by the daemon.
        Listen for {'event':'move_complete','pos':<N>} via on_event().
        """
        r = await self._send({
            "cmd":         "move_to",
            "axis":        1,
            "pos":         pos,
            "start_hz":    start_hz,
            "max_hz":      max_hz,
            "accel_steps": accel_steps,
        })
        return bool(r.get("ok"))


# ── Stand-alone smoke test ────────────────────────────────────────────────

async def _test():
    """Connect, print telem for 5 s, then stop."""
    client = PruClient()
    events = []

    async def on_event(msg: dict):
        events.append(msg)
        etype = msg.get("event", "?")
        if etype == "telem":
            sp = msg.get("sp", {})
            lat = msg.get("lat", {})
            print(f"  [TELEM] sp={sp.get('speed_hz',0)} Hz, "
                  f"lat={lat.get('speed_hz',0)} Hz, "
                  f"lat_pos={lat.get('pos',0)}")
        else:
            print(f"  [EVENT] {msg}")

    client.on_event(on_event)
    await client.connect()

    listener = asyncio.create_task(client.start_event_listener())

    print("Enabling drivers...")
    await client.enable(sp=True, lat=True)

    print("Setting speed: sp=6400 Hz, lat=200 Hz...")
    await client.set_speed(sp_hz=6400, lat_hz=200)

    print("Running for 5 s...")
    await asyncio.sleep(5)

    print("Emergency stop...")
    await client.emergency_stop()

    listener.cancel()
    await client.disconnect()
    print(f"Total events received: {len(events)}")


if __name__ == "__main__":
    asyncio.run(_test())
