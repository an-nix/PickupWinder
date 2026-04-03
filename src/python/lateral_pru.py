"""LateralPRU - lateral traverse axis wrapper around IpcChannel.

Mirrors LateralPRU.cpp / LateralPRU.h.
Klipper-style: all steps pushed as (interval, count, add) move triples via sendMove().
CMD_HOME_START triggers PRU1-side homing.  No CMD_LATERAL_* commands.
"""
from ipc import (
    IpcChannel,
    CMD_ENABLE, CMD_EMERGENCY_STOP, CMD_RESET_POSITION,
    CMD_QUEUE_FLUSH, CMD_HOME_START,
    AXIS_LATERAL, AXIS_ALL,
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

PRU_CLOCK_HZ = 200_000_000
MIN_INTERVAL  = 625
_LAT_MIN_HZ   = 100
_REFILL_MS    = 20


def _hz_to_interval(hz: int) -> int:
    if hz <= 0:
        return 0
    return max(MIN_INTERVAL, PRU_CLOCK_HZ // (2 * hz))


def _mm_to_steps(mm: float) -> int:
    return max(0, int(mm * LAT_STEPS_PER_MM)) if LAT_STEPS_PER_MM else 0


def _steps_to_mm(steps: int) -> float:
    return steps / LAT_STEPS_PER_MM if LAT_STEPS_PER_MM else 0.0


def _push_move_mm(ch: IpcChannel, hz: int, forward: bool, dist_mm: float) -> bool:
    """Push a constant-speed block covering dist_mm at hz."""
    if hz <= 0 or dist_mm < 0.001:
        return False
    n_steps = _mm_to_steps(dist_mm)
    if n_steps <= 0:
        return False
    iv = _hz_to_interval(hz)
    direction = 0 if forward else 1
    return ch.sendMove(AXIS_LATERAL, iv, 2 * n_steps, 0, direction)


def _push_constant_ms(ch: IpcChannel, hz: int, forward: bool, duration_ms: int) -> bool:
    """Push a constant-speed block for approximately duration_ms."""
    if hz <= 0 or duration_ms <= 0:
        return False
    iv = _hz_to_interval(hz)
    edges = max(1, (2 * hz * duration_ms) // 1000)
    direction = 0 if forward else 1
    return ch.sendMove(AXIS_LATERAL, iv, edges, 0, direction)


class LateralPRU:
    """Lateral axis - Linux-side state machine delegating to PRU1 via IpcChannel."""

    def __init__(self, channel: IpcChannel):
        self._ch = channel
        self._home_offset_mm: float = float(LAT_HOME_OFFSET_DEFAULT_MM)
        self._state: str = HOMING
        self._telem: dict = {}

        # Motion targets
        self._start_pos_mm: float = 0.0
        self._start_mm: float = 0.0
        self._end_mm:   float = 0.0

        # Pass tracking
        self._pass_count: int = 0
        self._last_lat_hz: int = 0
        self._winding_fwd: bool = True

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
        self._ch.sendCommand(CMD_ENABLE, AXIS_LATERAL, 1)
        self.rehome()

    def rehome(self):
        if not self._ch.isOpen():
            return
        self._state = HOMING
        self._ch.sendCommand(CMD_HOME_START, AXIS_LATERAL)

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
            # PRU1 drives backward and stops at sensor; host waits for idle
            if at_home and not running:
                self._state = HOMING_OFFSET
                if self._home_offset_mm > 0.001:
                    _push_move_mm(self._ch, LAT_HOME_SPEED_HZ // 2, True,
                                  self._home_offset_mm)
                else:
                    self._ch.sendCommand(CMD_RESET_POSITION, AXIS_LATERAL)
                    self._state = HOMED

        elif self._state == HOMING_OFFSET:
            if not running:
                self._ch.sendCommand(CMD_RESET_POSITION, AXIS_LATERAL)
                self._state = HOMED

        elif self._state == POSITIONING:
            if not running:
                self._state = HOMED

        elif self._state == WINDING_FWD:
            if pos_mm >= self._end_mm:
                self._ch.sendCommand(CMD_QUEUE_FLUSH, AXIS_LATERAL)
                if self._stop_on_next_high:
                    self._stop_on_next_high = False
                    self._state = HOMED
                elif self._pause_on_next_reversal:
                    self._pause_on_next_reversal = False
                    self._paused_at_reversal = True
                    self._state = HOMED
                else:
                    self._pass_count += 1
                    width = self._end_mm - self._start_mm
                    _push_move_mm(self._ch, self._last_lat_hz, False, width)
                    _push_constant_ms(self._ch, self._last_lat_hz, False, _REFILL_MS)
                    self._winding_fwd = False
                    self._state = WINDING_BWD

        elif self._state == WINDING_BWD:
            if pos_mm <= self._start_mm or at_home:
                self._ch.sendCommand(CMD_QUEUE_FLUSH, AXIS_LATERAL)
                if self._stop_on_next_low:
                    self._stop_on_next_low = False
                    self._state = HOMED
                else:
                    self._pass_count += 1
                    width = self._end_mm - self._start_mm
                    _push_move_mm(self._ch, self._last_lat_hz, True, width)
                    _push_constant_ms(self._ch, self._last_lat_hz, True, _REFILL_MS)
                    self._winding_fwd = True
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

    def get_actual_velocity_hz(self) -> int:
        iv = self._telem.get('current_interval', 0)
        if not iv:
            return 0
        return PRU_CLOCK_HZ // (2 * iv)

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
        self._ch.sendCommand(CMD_QUEUE_FLUSH, AXIS_LATERAL)
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
        fwd = dist > 0
        _push_move_mm(self._ch, speed_hz, fwd, abs(dist))

    def jog(self, delta_mm: float):
        if not self._ch.isOpen() or delta_mm == 0:
            return
        fwd = delta_mm > 0
        _push_move_mm(self._ch, LAT_HOME_SPEED_HZ // 4, fwd, abs(delta_mm))

    def start_winding(self, hz_nominal: int, tpp: float,
                      start_mm: float, end_mm: float, speed_scale: float = 1.0):
        if not self._ch.isOpen():
            return
        self._start_mm = float(start_mm)
        self._end_mm   = float(end_mm)
        lat_hz = self._calc_lat_hz(hz_nominal, tpp, end_mm - start_mm, speed_scale)
        self._last_lat_hz = lat_hz
        width = end_mm - start_mm
        _push_move_mm(self._ch, lat_hz, True, width)
        _push_constant_ms(self._ch, lat_hz, True, _REFILL_MS)
        self._winding_fwd = True
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
        # Flush existing moves and repush at new speed
        self._ch.sendCommand(CMD_QUEUE_FLUSH, AXIS_LATERAL)
        width = self._end_mm - self._start_mm
        _push_move_mm(self._ch, lat_hz, self._winding_fwd, width)
        _push_constant_ms(self._ch, lat_hz, self._winding_fwd, _REFILL_MS)

    # -- Private helpers ------------------------------------------------
    @staticmethod
    def _calc_lat_hz(spindle_hz: int, tpp: float, eff_width_mm: float,
                     speed_scale: float) -> int:
        """Lateral frequency = effWidth_mm * spindle_Hz / (tpp * STEPS_PER_REV) * scale."""
        if tpp <= 0 or SPINDLE_STEPS_PER_REV == 0 or eff_width_mm <= 0:
            return _LAT_MIN_HZ
        lat_hz = int(eff_width_mm * spindle_hz / (tpp * SPINDLE_STEPS_PER_REV) * speed_scale)
        return max(_LAT_MIN_HZ, lat_hz)
