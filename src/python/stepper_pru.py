"""StepperPRU - spindle axis wrapper around IpcChannel.

Mirrors StepperPRU.cpp / StepperPRU.h exactly.
Klipper-style: all steps are pushed as (interval, count, add) move triples.
No Hz/accel/start/stop commands are sent to the PRU.
"""
from ipc import (
    IpcChannel,
    CMD_ENABLE, CMD_EMERGENCY_STOP, CMD_RESET_POSITION, CMD_QUEUE_FLUSH,
    AXIS_SPINDLE, AXIS_ALL,
    STATE_RUNNING, STATE_ENABLED,
)
from config import STEPS_PER_REV, SPEED_HZ_MIN, SPEED_HZ_MAX

PRU_CLOCK_HZ = 200_000_000
MIN_INTERVAL  = 625          # = PRU_CLOCK_HZ / (2 * 160000)
_LAT_MIN_HZ_THRESHOLD = 100
_REFILL_MS = 10
_KEEP_SLOTS = 4


def _clamp_hz(v: int) -> int:
    return max(SPEED_HZ_MIN, min(SPEED_HZ_MAX, v))


def _hz_to_interval(hz: int) -> int:
    if hz <= 0:
        return 0
    iv = PRU_CLOCK_HZ // (2 * hz)
    return max(MIN_INTERVAL, iv)


def _push_constant_ms(ch: IpcChannel, hz: int, forward: bool, duration_ms: int) -> bool:
    """Push a constant-speed block for approximately duration_ms."""
    if hz <= 0 or duration_ms <= 0:
        return False
    iv = _hz_to_interval(hz)
    # edges per ms = 2*hz/1000, total edges for duration
    edges = max(1, (2 * hz * duration_ms) // 1000)
    direction = 0 if forward else 1
    return ch.sendMove(AXIS_SPINDLE, iv, edges, 0, direction)


class StepperPRU:
    """Spindle axis control via PRU IPC.  Mirrors StepperPRU.cpp."""

    def __init__(self, channel: IpcChannel):
        self._ch = channel
        self._speed_hz: int = 0
        self._pushed_hz: int = 0
        self._forward: bool = True
        self._streaming: bool = False
        self._speed_changed: bool = False
        self._accel_hz_per_s: int = 150_000
        self._telem: dict = {}

    def begin(self, accel_hz_per_ms: int = 150):
        if not self._ch.isOpen():
            return
        self._accel_hz_per_s = accel_hz_per_ms * 1000
        self._ch.sendCommand(CMD_ENABLE, AXIS_SPINDLE, 0)  # disabled at boot

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

    def get_rpm(self) -> float:
        self._refresh()
        iv = self._telem.get('current_interval', 0)
        if not iv:
            return 0.0
        hz = PRU_CLOCK_HZ / (2 * iv)
        return hz * 60.0 / STEPS_PER_REV if STEPS_PER_REV else 0.0

    #  Control
    def set_speed_hz(self, hz: int):
        clamped = _clamp_hz(int(hz))
        if clamped == self._speed_hz:
            return
        self._speed_hz = clamped
        self._speed_changed = True

    def start(self, forward: bool = True):
        if not self._ch.isOpen():
            return
        self._forward = forward
        self._streaming = True
        self._ch.sendCommand(CMD_ENABLE, AXIS_SPINDLE, 1)
        _push_constant_ms(self._ch, self._speed_hz, forward, _REFILL_MS * _KEEP_SLOTS)
        self._pushed_hz = self._speed_hz
        self._speed_changed = False

    def stop(self):
        if not self._ch.isOpen():
            return
        self._streaming = False
        # Push zero-speed block (decel would need MoveQueue ramp math)
        self._ch.sendCommand(CMD_QUEUE_FLUSH, AXIS_SPINDLE)
        self._pushed_hz = 0

    def force_stop(self):
        if not self._ch.isOpen():
            return
        self._streaming = False
        self._ch.sendCommand(CMD_QUEUE_FLUSH, AXIS_SPINDLE)
        self._ch.sendCommand(CMD_ENABLE, AXIS_SPINDLE, 0)
        self._pushed_hz = 0

    def disable(self):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_ENABLE, AXIS_SPINDLE, 0)

    def reset_turns(self):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_RESET_POSITION, AXIS_SPINDLE)

    def emergency_stop(self):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_EMERGENCY_STOP, AXIS_ALL)

    def tick(self):
        """Refill move ring; handle speed changes.  Call at control cadence."""
        if not self._streaming or not self._ch.isOpen():
            return
        if self._speed_changed:
            self._ch.sendCommand(CMD_QUEUE_FLUSH, AXIS_SPINDLE)
            _push_constant_ms(self._ch, self._speed_hz, self._forward,
                               _REFILL_MS * _KEEP_SLOTS)
            self._pushed_hz = self._speed_hz
            self._speed_changed = False
        else:
            free = self._ch.moveQueueFreeSlots(AXIS_SPINDLE)
            if free >= 4:
                _push_constant_ms(self._ch, self._pushed_hz, self._forward, _REFILL_MS)

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
