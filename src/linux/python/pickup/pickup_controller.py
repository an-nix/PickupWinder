"""High-level PickupWinder controller (packaged).

Copied into the `pickup` subpackage; keeps application imports simple:
`from pickup.pickup_controller import PickupController`.
"""

from typing import Dict, Optional

from hal import DaemonClient


class PickupController:
    """High-level PickupWinder command wrapper."""

    def __init__(self, client: DaemonClient):
        self._client = client

    def enable(self, sp: bool = False, lat: bool = False) -> bool:
        if sp and lat:
            axis = 0xFF
        elif sp:
            axis = 0
        elif lat:
            axis = 1
        else:
            axis = 0xFF
        value = 1 if (sp or lat) else 0
        return bool(self._client.send({
            "cmd": "enable",
            "axis": axis,
            "value": value,
        }).get("ok"))

    def set_speed(self,
                  sp_hz: int = 0,
                  lat_hz: int = 0,
                  sp_dir: int = 0,
                  lat_dir: int = 0) -> bool:
        return bool(self._client.send({
            "cmd": "set_speed",
            "sp_hz": sp_hz,
            "lat_hz": lat_hz,
            "sp_dir": sp_dir,
            "lat_dir": lat_dir,
        }).get("ok"))

    def emergency_stop(self) -> bool:
        return bool(self._client.send({"cmd": "e_stop"}).get("ok"))

    def home_start(self) -> bool:
        return bool(self._client.send({"cmd": "home_start"}).get("ok"))

    def home(self, timeout: float = 30.0) -> bool:
        if not self.home_start():
            return False
        event = self._client.wait_event("home_complete", timeout=timeout)
        if not event:
            return False
        self.ack_event()
        return True

    def reset_position(self, axis: int = 0xFF) -> bool:
        return bool(self._client.send({
            "cmd": "reset_pos",
            "axis": axis,
        }).get("ok"))

    def ack_event(self) -> bool:
        return bool(self._client.send({"cmd": "ack_event"}).get("ok"))

    def set_limits(self, axis: int, min_steps: int, max_steps: int) -> bool:
        return bool(self._client.send({
            "cmd": "set_limits",
            "axis": axis,
            "min": min_steps,
            "max": max_steps,
        }).get("ok"))

    def set_mode(self, mode: str) -> bool:
        return bool(self._client.send({"cmd": "set_mode", "mode": mode}).get("ok"))

    def set_accel(self,
                  lat_max_speed: int = 0,
                  lat_accel: int = 0,
                  lat_decel: int = 0,
                  sp_accel: int = 0,
                  sp_decel: int = 0) -> bool:
        cmd: Dict[str, int] = {"cmd": "set_accel"}
        if lat_max_speed:
            cmd["lat_max_speed"] = lat_max_speed
        if lat_accel:
            cmd["lat_accel"] = lat_accel
        if lat_decel:
            cmd["lat_decel"] = lat_decel
        if sp_accel:
            cmd["sp_accel"] = sp_accel
        if sp_decel:
            cmd["sp_decel"] = sp_decel
        return bool(self._client.send(cmd).get("ok"))

    def move_to(self, pos: int, axis: int = 1) -> bool:
        return bool(self._client.send({
            "cmd": "move_to",
            "axis": axis,
            "pos": pos,
        }).get("ok"))

    def wait_event(self,
                   event_type: Optional[str] = None,
                   timeout: float = 5.0) -> Optional[Dict[str, object]]:
        return self._client.wait_event(event_type=event_type, timeout=timeout)

    def drain_events(self, timeout: float = 0.1) -> None:
        self._client.drain_events(timeout=timeout)

    def close(self) -> None:
        self._client.close()
