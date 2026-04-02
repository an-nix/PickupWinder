"""Minimal WinderApp port in Python.

This is a lightweight port of the core control responsibilities:
- map potentiometer -> spindle target Hz
- start/stop spindle on demand
- expose `tick()` to be called at control cadence
- provide `get_status()` reading PRU telemetry

This intentionally keeps logic small so we can iterate quickly.
"""
from ipc import CMD_SPINDLE_SET_HZ, CMD_SPINDLE_START, CMD_SPINDLE_STOP, CMD_SET_NOMINAL_LAT

AXIS_SPINDLE = 0
AXIS_LATERAL = 1


class WinderApp:
    def __init__(self, ipc, nominal_lat_hz=4800):
        self.ipc = ipc
        self.state = 'IDLE'
        self.target_hz = 0
        self.nominal_lat_hz = nominal_lat_hz
        # inform PRU about nominal lateral speed for compensation
        try:
            self.ipc.send_command(CMD_SET_NOMINAL_LAT, AXIS_LATERAL, int(self.nominal_lat_hz), 0)
        except Exception:
            pass

    def begin(self):
        # placeholder for any init steps
        return

    def tick(self, pot_hz, encoder_delta=0):
        """Called periodically (control cadence)."""
        # Simple behaviour: if pot_hz > 0 -> run at that speed, else stop
        if pot_hz is None:
            return

        if pot_hz > 0:
            if int(pot_hz) != int(self.target_hz):
                self.target_hz = int(pot_hz)
                # set target Hz
                try:
                    self.ipc.send_command(CMD_SPINDLE_SET_HZ, AXIS_SPINDLE, int(self.target_hz), 0)
                except Exception:
                    pass
            # ensure spindle is running
            if self.state != 'WINDING':
                try:
                    self.ipc.send_command(CMD_SPINDLE_START, AXIS_SPINDLE, 1, 0)
                except Exception:
                    pass
                self.state = 'WINDING'
        else:
            # pot is zero -> request slow controlled stop
            if self.state == 'WINDING':
                try:
                    self.ipc.send_command(CMD_SPINDLE_STOP, AXIS_SPINDLE, 0, 0)
                except Exception:
                    pass
                self.state = 'PAUSED'
                self.target_hz = 0

    def emergency_stop(self):
        try:
            self.ipc.send_command(CMD_SPINDLE_STOP, AXIS_SPINDLE, 0, 0)
        except Exception:
            pass
        self.state = 'IDLE'
        self.target_hz = 0

    def get_status(self):
        try:
            sp = self.ipc.read_spindle_telem()
        except Exception:
            sp = None
        try:
            lat = self.ipc.read_lateral_telem()
        except Exception:
            lat = None
        return {
            'state': self.state,
            'target_hz': self.target_hz,
            'spindle_telem': sp,
            'lateral_telem': lat,
        }
