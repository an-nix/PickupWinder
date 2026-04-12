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
    {"cmd":"move_to",    "axis":1, "pos":<int>}
    {"cmd":"set_accel",  "lat_max_speed":<uint>, "lat_accel":<uint>,
                          "lat_decel":<uint>, "sp_accel":<uint>}
    {"cmd":"set_mode",   "mode":"free"|"winding"}

  Winding modes:
    free    (default) — axes independent. set_speed controls spindle directly.
                        move_to positions lateral without spindle sync.
    winding           — spindle tracks lateral ramps proportionally.
                        set_speed sets the reference speed; PRU0 adjusts
                        spindle interval in real-time with lateral via Q6 ratio.

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

# Winding modes for set_mode()
MODE_FREE    = "free"    # Independent axes; no spindle-lateral sync (default)
MODE_WINDING = "winding" # Spindle tracks lateral ramps; turns/mm stays constant


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

    async def move_to(self, pos: int) -> bool:
        """Move the lateral axis to an absolute position.

        pos: target position in steps (signed, relative to home=0)

        The daemon reads the current lateral speed from PRU telemetry and uses
        the globally configured accel/decel/max_speed (set via set_accel()) to
        plan and execute the full trapezoidal profile autonomously.  Python
        never specifies start speed, cruise speed, or step count.

        Consecutive same-direction calls while the motor is moving trigger a
        hot retarget: the motor never decelerates between waypoints.
        Direction-change calls wait for a full stop then re-accelerate.

        In MODE_WINDING the daemon also sends a Q6 spindle-coordination ratio
        so PRU0 keeps turns/mm constant during lateral ramps.

        Returns True when the command is acknowledged by the daemon.
        Listen for {'event':'move_complete','pos':<N>} via on_event().
        """
        r = await self._send({"cmd": "move_to", "axis": 1, "pos": pos})
        return bool(r.get("ok"))

    async def set_accel(self, lat_max_speed: int = 0, lat_accel: int = 0,
                        lat_decel: int = 0, sp_accel: int = 0) -> bool:
        """Configure per-axis acceleration and max speed limits.

        All parameters are optional — only non-zero values update the daemon's
        globals.  Call once at startup or when changing the winding profile.

        lat_max_speed: lateral cruise speed in Hz       (e.g. 4000)
        lat_accel:     lateral acceleration in steps/s² (e.g. 10000)
        lat_decel:     lateral deceleration in steps/s² (e.g. 10000)
        sp_accel:      spindle acceleration  in steps/s² (e.g. 100000)
        """
        cmd: dict = {"cmd": "set_accel"}
        if lat_max_speed: cmd["lat_max_speed"] = lat_max_speed
        if lat_accel:     cmd["lat_accel"]     = lat_accel
        if lat_decel:     cmd["lat_decel"]     = lat_decel
        if sp_accel:      cmd["sp_accel"]      = sp_accel
        r = await self._send(cmd)
        return bool(r.get("ok"))

    async def set_mode(self, mode: str) -> bool:
        """Set the winding operating mode.

        mode: MODE_FREE (default) or MODE_WINDING.

        MODE_FREE    — axes independent. set_speed controls spindle directly.
                       move_to positions lateral; no spindle sync.
                       Use for: testing, homing, positioning.

        MODE_WINDING — spindle tracks lateral ramps proportionally.
                       Call set_speed() first to establish the reference speed,
                       then move_to() activates coordination. PRU0 adjusts
                       spindle interval every ~5 µs via Q6 ratio so turns/mm
                       stays constant during accel/decel/reversal gaps.
                       Coordination is disabled by any subsequent set_speed()
                       call (Python re-takes direct spindle control).
                       Use for: actual winding runs.
        """
        r = await self._send({"cmd": "set_mode", "mode": mode})
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
