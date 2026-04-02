"""StepperPRU - spindle axis wrapper around IpcChannel.

Mirrors StepperPRU.cpp / StepperPRU.h exactly.
All hardware values use integers only (Hz, steps).
"""
from ipc import (
    IpcChannel,
    CMD_SPINDLE_SET_HZ, CMD_SPINDLE_SET_ACCEL,
    CMD_SPINDLE_START, CMD_SPINDLE_STOP, CMD_SPINDLE_FORCE,
    CMD_SPINDLE_ENABLE, CMD_RESET_POSITION,
    CMD_EMERGENCY_STOP,
    AXIS_SPINDLE, AXIS_ALL,
    STATE_RUNNING, STATE_ENABLED,
)
from config import STEPS_PER_REV, SPEED_HZ_MIN, SPEED_HZ_MAX

_LAT_MIN_HZ_THRESHOLD = 100


def _clamp_hz(v: int) -> int:
    return max(SPEED_HZ_MIN, min(SPEED_HZ_MAX, v))


class StepperPRU:
    """Spindle axis control via PRU IPC.  Mirrors StepperPRU.cpp."""

    def __init__(self, channel: IpcChannel):
        self._ch = channel
        self._speed_hz: int = 0
        self._telem: dict = {}

    def begin(self, accel_hz_per_ms: int = 150):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_SPINDLE_SET_ACCEL, AXIS_SPINDLE, accel_hz_per_ms)
        self._ch.sendCommand(CMD_SPINDLE_ENABLE, AXIS_SPINDLE, 0)  # disabled at boot

    #  Telemetry 
    def _refresh(self):
        self._telem = self._ch.readSpindleTelem()

    def is_running(self) -> bool:
        self._refresh()
        return bool(self._telem.get('state', 0) & STATE_RUNNING)

    def is_enabled(self) -> bool:
        self._refresh()
        return bool(self._telem.get('state', 0) & STATE_ENABLED)

    def get_turns(self) -> int:
        self._refresh()
        sc = self._telem.get('step_count', 0)
        return sc // STEPS_PER_REV if STEPS_PER_REV else 0

    def get_speed_hz(self) -> int:
        self._refresh()
        return self._telem.get('current_hz', 0)

    def get_rpm(self) -> float:
        hz = self.get_speed_hz()
        return hz * 60.0 / STEPS_PER_REV if STEPS_PER_REV else 0.0

    #  Control 
    def set_speed_hz(self, hz: int):
        clamped = _clamp_hz(int(hz))
        if clamped == self._speed_hz:
            return
        self._speed_hz = clamped
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_SPINDLE_SET_HZ, AXIS_SPINDLE, self._speed_hz)

    def start(self, forward: bool = True):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_SPINDLE_ENABLE, AXIS_SPINDLE, 1)
        self._ch.sendCommand(CMD_SPINDLE_SET_HZ, AXIS_SPINDLE, self._speed_hz)
        self._ch.sendCommand(CMD_SPINDLE_START, AXIS_SPINDLE, 1 if forward else 0)

    def stop(self):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_SPINDLE_STOP, AXIS_SPINDLE)

    def force_stop(self):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_SPINDLE_FORCE, AXIS_SPINDLE)
        self._ch.sendCommand(CMD_SPINDLE_ENABLE, AXIS_SPINDLE, 0)

    def disable(self):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_SPINDLE_ENABLE, AXIS_SPINDLE, 0)

    def reset_turns(self):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_RESET_POSITION, AXIS_SPINDLE)

    def emergency_stop(self):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_EMERGENCY_STOP, AXIS_ALL)

    @staticmethod
    def calculate_compensated_spindle_hz(
        nominal_spindle_hz: int,
        current_lat_hz: int,
        nominal_lat_hz: int
    ) -> int:
        """Scale spindle Hz proportionally to actual lateral Hz."""
        if nominal_lat_hz == 0 or nominal_spindle_hz == 0:
            return nominal_spindle_hz
        if current_lat_hz < _LAT_MIN_HZ_THRESHOLD:
            return nominal_spindle_hz
        comp = (nominal_spindle_hz * current_lat_hz) // nominal_lat_hz
        return _clamp_hz(comp)
