"""LateralPRU - lateral traverse axis wrapper around IpcChannel.

Mirrors LateralPRU.cpp / LateralPRU.h exactly.
The state machine runs entirely on the Linux side (Python).
The PRU handles only STEP generation and reports STATE_RUNNING / STATE_AT_HOME.
"""
from ipc import (
    IpcChannel,
    CMD_LATERAL_SET_HZ, CMD_LATERAL_SET_ACCEL,
    CMD_LATERAL_START, CMD_LATERAL_STOP,
    CMD_LATERAL_ENABLE, CMD_RESET_POSITION,
    AXIS_LATERAL,
    STATE_RUNNING, STATE_AT_HOME,
    FAULT_HOME_SENSOR, FAULT_OVERRUN,
)
from config import (
    LAT_STEPS_PER_MM, LAT_HOME_SPEED_HZ, LAT_HOME_OFFSET_DEFAULT_MM,
    STEPS_PER_REV as SPINDLE_STEPS_PER_REV,
)

# State strings
FAULT          = "FAULT"
HOMING         = "HOMING"
HOMING_OFFSET  = "HOMING_OFFSET"
HOMED          = "HOMED"
POSITIONING    = "POSITIONING"
WINDING_FWD    = "WINDING_FWD"
WINDING_BWD    = "WINDING_BWD"

_LAT_ACCEL_HZ_MS = 48    # 48 Hz/ms -> 4800 Hz in 100 ms (mirrors LateralPRU.cpp)
_LAT_MIN_HZ      = 100   # minimum winding lateral Hz


def _steps_to_mm(steps: int) -> float:
    return steps / LAT_STEPS_PER_MM if LAT_STEPS_PER_MM else 0.0


class LateralPRU:
    """Lateral axis - Linux-side state machine delegating to PRU1 via IpcChannel."""

    def __init__(self, channel: IpcChannel):
        self._ch = channel
        self._home_offset_mm: float = float(LAT_HOME_OFFSET_DEFAULT_MM)
        self._state: str = HOMING
        self._telem: dict = {}

        # Motion targets
        self._start_pos_mm: float = 0.0   # carriage start-position target
        self._start_mm: float = 0.0       # winding start bound
        self._end_mm:   float = 0.0       # winding end bound

        # Pass tracking
        self._pass_count: int = 0
        self._last_lat_hz: int = 0

        # One-shot flags
        self._stop_on_next_high:     bool = False
        self._stop_on_next_low:      bool = False
        self._pause_on_next_reversal: bool = False
        self._paused_at_reversal:    bool = False

    # -- Init -----------------------------------------------------------
    def begin(self, home_offset_mm: float = None):
        if home_offset_mm is not None:
            self._home_offset_mm = home_offset_mm
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_LATERAL_SET_ACCEL, AXIS_LATERAL, _LAT_ACCEL_HZ_MS)
        self._ch.sendCommand(CMD_LATERAL_ENABLE, AXIS_LATERAL, 1)
        self.rehome()

    def rehome(self):
        if not self._ch.isOpen():
            return
        self._state = HOMING
        self._ch.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, LAT_HOME_SPEED_HZ)
        self._ch.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, 0)  # 0 = backward toward home

    # -- State machine tick (call every control loop) -------------------
    def update(self):
        if not self._ch.isOpen():
            return
        self._telem = self._ch.readLateralTelem()
        st     = self._telem.get('state', 0)
        faults = self._telem.get('faults', 0)

        if faults & FAULT_HOME_SENSOR:
            self._state = FAULT
            return
        if (faults & FAULT_OVERRUN) and self._state not in (HOMING, HOMING_OFFSET):
            self._state = FAULT
            return

        pos_mm  = self.get_current_position_mm()
        running = bool(st & STATE_RUNNING)
        at_home = bool(st & STATE_AT_HOME)

        if self._state == HOMING:
            if at_home and not running:
                self._state = HOMING_OFFSET
                self._ch.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, LAT_HOME_SPEED_HZ // 2)
                self._ch.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, 1)   # 1 = forward

        elif self._state == HOMING_OFFSET:
            if pos_mm >= self._home_offset_mm:
                self._ch.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL)
                if not running:
                    self._ch.sendCommand(CMD_RESET_POSITION, AXIS_LATERAL)
                    self._state = HOMED

        elif self._state == POSITIONING:
            if abs(pos_mm - self._start_pos_mm) <= 0.05:
                self._ch.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL)
                if not running:
                    self._state = HOMED

        elif self._state == WINDING_FWD:
            if pos_mm >= self._end_mm:
                if self._stop_on_next_high:
                    self._stop_on_next_high = False
                    self._ch.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL)
                    self._state = HOMED
                elif self._pause_on_next_reversal:
                    self._pause_on_next_reversal = False
                    self._paused_at_reversal = True
                    self._ch.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL)
                    self._state = HOMED
                else:
                    self._pass_count += 1
                    self._ch.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, 0)   # backward
                    self._state = WINDING_BWD

        elif self._state == WINDING_BWD:
            if pos_mm <= self._start_mm or at_home:
                if self._stop_on_next_low:
                    self._stop_on_next_low = False
                    self._ch.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL)
                    self._state = HOMED
                else:
                    self._pass_count += 1
                    self._ch.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, 1)   # forward
                    self._state = WINDING_FWD

    # -- Getters --------------------------------------------------------
    def get_state(self) -> str: return self._state

    def is_homed(self) -> bool:
        return self._state in (HOMED, WINDING_FWD, WINDING_BWD)

    def is_fault(self) -> bool: return self._state == FAULT

    def is_busy(self) -> bool:
        return bool(self._telem.get('state', 0) & STATE_RUNNING)

    def is_traversing(self) -> bool:
        return self._state in (WINDING_FWD, WINDING_BWD)

    def is_at_zero(self) -> bool:
        return bool(self._telem.get('state', 0) & STATE_AT_HOME)

    def is_positioned_for_start(self) -> bool:
        return (self._state == HOMED and
                abs(self.get_current_position_mm() - self._start_pos_mm) < 0.1)

    def get_current_position_mm(self) -> float:
        return _steps_to_mm(self._telem.get('position_steps', 0))

    def get_target_position_mm(self) -> float:
        return self._start_pos_mm

    def get_traversal_progress(self) -> float:
        if not self.is_traversing() or abs(self._end_mm - self._start_mm) < 0.001:
            return 0.0
        pos = self.get_current_position_mm()
        return max(0.0, min(1.0, (pos - self._start_mm) / (self._end_mm - self._start_mm)))

    def get_pass_count(self) -> int: return self._pass_count
    def get_actual_velocity_hz(self) -> int: return self._telem.get('current_hz', 0)
    def get_instantaneous_lat_hz_nominal(self) -> int: return self._last_lat_hz
    def get_home_offset(self) -> float: return self._home_offset_mm
    def set_home_offset(self, mm: float): self._home_offset_mm = float(mm)

    def consume_paused_at_reversal(self) -> bool:
        v = self._paused_at_reversal
        if v:
            self._paused_at_reversal = False
        return v

    # -- One-shot flags -------------------------------------------------
    def arm_stop_at_next_high(self): self._stop_on_next_high = True
    def arm_stop_at_next_low(self):  self._stop_on_next_low  = True
    def arm_pause_on_next_reversal(self): self._pause_on_next_reversal = True
    def clear_one_shot_stops(self):
        self._stop_on_next_high      = False
        self._stop_on_next_low       = False
        self._pause_on_next_reversal = False
        self._paused_at_reversal     = False

    # -- Motion commands ------------------------------------------------
    def park_at_zero(self):
        self.prepare_start_position(0.0)

    def stop_winding(self):
        self.stop()

    def stop(self):
        if not self._ch.isOpen():
            return
        self._ch.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL)
        if self._state in (WINDING_FWD, WINDING_BWD):
            self._state = HOMED

    def prepare_start_position(self, target_mm: float, speed_hz: int = LAT_HOME_SPEED_HZ):
        if not self._ch.isOpen():
            return
        self._start_pos_mm = float(target_mm)
        current = self.get_current_position_mm()
        dist = target_mm - current
        if abs(dist) < 0.01:
            return
        self._state = POSITIONING
        fwd = 1 if dist > 0 else 0
        self._ch.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, speed_hz)
        self._ch.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, fwd)

    def jog(self, delta_mm: float):
        if not self._ch.isOpen() or delta_mm == 0:
            return
        fwd = 1 if delta_mm > 0 else 0
        self._ch.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, LAT_HOME_SPEED_HZ // 4)
        self._ch.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, fwd)

    def start_winding(self, hz_nominal: int, tpp: float,
                      start_mm: float, end_mm: float, speed_scale: float = 1.0):
        if not self._ch.isOpen():
            return
        self._start_mm = float(start_mm)
        self._end_mm   = float(end_mm)
        lat_hz = self._calc_lat_hz(hz_nominal, tpp, end_mm - start_mm, speed_scale)
        self._last_lat_hz = lat_hz
        self._ch.setNominalLateralHz(lat_hz)
        self._ch.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, lat_hz)
        self._ch.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, 1)   # forward
        self._state = WINDING_FWD

    def update_winding(self, hz_nominal: int, tpp: float,
                       start_mm: float, end_mm: float, speed_scale: float = 1.0):
        if not self._ch.isOpen():
            return
        if start_mm >= end_mm or tpp <= 0:
            return
        lat_hz = self._calc_lat_hz(hz_nominal, tpp, end_mm - start_mm, speed_scale)
        if lat_hz == self._last_lat_hz:
            return
        self._last_lat_hz = lat_hz
        self._ch.setNominalLateralHz(lat_hz)
        self._ch.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, lat_hz)

    # -- Private helpers ------------------------------------------------
    @staticmethod
    def _calc_lat_hz(spindle_hz: int, tpp: float, eff_width_mm: float,
                     speed_scale: float) -> int:
        """Lateral frequency = effWidth_mm * spindle_Hz / (tpp * STEPS_PER_REV) * scale."""
        if tpp <= 0 or SPINDLE_STEPS_PER_REV == 0 or eff_width_mm <= 0:
            return _LAT_MIN_HZ
        lat_hz = int(eff_width_mm * spindle_hz / (tpp * SPINDLE_STEPS_PER_REV) * speed_scale)
        return max(_LAT_MIN_HZ, lat_hz)
