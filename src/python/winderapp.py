"""Minimal WinderApp port in Python.

Lightweight port of core control responsibilities:
- map potentiometer -> spindle target Hz
- start/stop spindle on demand
- expose `tick()` to be called at control cadence
- provide `get_status()` reading PRU telemetry

Uses StepperPRU for all spindle motion (Klipper-style sendMove).
No direct IPC commands issued from this layer.
"""
from ipc import IpcChannel
from stepper_pru import StepperPRU


class WinderApp:
    def __init__(self, ipc: IpcChannel, nominal_lat_hz: int = 4800):
        self.ipc = ipc
        self.state = 'IDLE'
        self.target_hz = 0
        self.nominal_lat_hz = nominal_lat_hz
        self._stepper = StepperPRU(ipc)

    def begin(self):
        self._stepper.begin()

    def tick(self, pot_hz, encoder_delta=0):
        """Called periodically (control cadence)."""
        if pot_hz is None:
            return

        if pot_hz > 0:
            self._stepper.set_speed_hz(int(pot_hz))
            self.target_hz = int(pot_hz)
            if self.state != 'WINDING':
                self._stepper.start(forward=True)
                self.state = 'WINDING'
            else:
                self._stepper.tick()
        else:
            if self.state == 'WINDING':
                self._stepper.stop()
                self.state = 'PAUSED'
                self.target_hz = 0

    def emergency_stop(self):
        self._stepper.force_stop()
        self.state = 'IDLE'
        self.target_hz = 0

    def get_status(self):
        try:
            sp = self.ipc.readSpindleTelem()
        except Exception:
            sp = None
        try:
            lat = self.ipc.readLateralTelem()
        except Exception:
            lat = None
        return {
            'state': self.state,
            'target_hz': self.target_hz,
            'spindle_telem': sp,
            'lateral_telem': lat,
        }
