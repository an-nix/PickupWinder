from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from .types import ControlIntent, InputSource, RunMode, SessionState, TickInput

if TYPE_CHECKING:
    from .config import AppConfig


@dataclass
class SessionSnapshot:
    state: SessionState = SessionState.IDLE
    source: InputSource = InputSource.NONE
    run_mode: RunMode = RunMode.NONE


class SessionController:
    """Application-level intent arbitration.

    Reads pot / footswitch / IHM intents and drives WinderApp at the correct
    speed.  Contains no PRU / hardware knowledge.
    """

    def __init__(self, winder_app, cfg: "AppConfig | None" = None):
        if cfg is None:
            from .config import AppConfig as _AppConfig
            cfg = _AppConfig()
        self._pot_threshold: float = cfg.pot_run_threshold

        self._winder = winder_app
        self._state = SessionState.IDLE
        self._pending_intent = ControlIntent.NONE
        self._pending_source = InputSource.NONE
        self._last_source = InputSource.NONE

        self._pot_level = 0.0
        self._pot_above_zero = False
        self._has_pot_sample = False
        self._has_footswitch_sample = False
        self._footswitch = False
        self._pot_needs_rearm = False

        self._run_mode = RunMode.NONE

    def request_start(self) -> None:
        self._record_intent(ControlIntent.START, InputSource.IHM)

    def request_pause(self) -> None:
        self._record_intent(ControlIntent.PAUSE, InputSource.IHM)

    def request_stop(self) -> None:
        self._record_intent(ControlIntent.STOP, InputSource.IHM)

    def _record_intent(self, intent: ControlIntent, source: InputSource) -> None:
        if intent == ControlIntent.START and self._state == SessionState.IDLE and source != InputSource.IHM:
            return
        self._pending_intent = intent
        self._pending_source = source
        self._last_source = source
        if source != InputSource.POT and self._pot_above_zero:
            self._pot_needs_rearm = True

    def _apply_pending_intent(self) -> None:
        if self._pending_intent == ControlIntent.NONE:
            return

        source = self._pending_source

        if self._pending_intent == ControlIntent.STOP:
            self._winder.stop_winding()
            self._state = SessionState.IDLE
            self._run_mode = RunMode.NONE

        elif self._pending_intent == ControlIntent.PAUSE:
            if self._state != SessionState.PAUSED:
                self._winder.pause_winding()
                self._state = SessionState.PAUSED
            self._run_mode = RunMode.NONE

        elif self._pending_intent == ControlIntent.START:
            if self._state == SessionState.IDLE and source != InputSource.IHM:
                self._pending_intent = ControlIntent.NONE
                self._pending_source = InputSource.NONE
                return
            self._winder.start_winding()
            self._state = SessionState.ARMED_OR_RUNNING
            if source == InputSource.POT:
                self._run_mode = RunMode.POT
            else:
                self._run_mode = RunMode.MAX
            self._pot_needs_rearm = False

        self._pending_intent = ControlIntent.NONE
        self._pending_source = InputSource.NONE

    def _apply_power(self) -> None:
        if self._state == SessionState.ARMED_OR_RUNNING:
            if self._run_mode == RunMode.POT:
                hz = int(self._pot_level * self._winder.max_speed_hz)
            elif self._run_mode == RunMode.MAX:
                hz = int(self._winder.max_speed_hz)
            else:
                hz = 0
            self._winder.set_control_hz(hz)
        else:
            self._winder.set_control_hz(0)

    def tick(self, ti: TickInput) -> None:
        if ti.has_pot:
            level = max(0.0, min(1.0, float(ti.pot_level)))
            self._pot_level = level
            pot_above_zero = level > self._pot_threshold

            if not self._has_pot_sample:
                self._has_pot_sample = True
                self._pot_above_zero = pot_above_zero
            elif pot_above_zero != self._pot_above_zero:
                self._pot_above_zero = pot_above_zero
                if self._state != SessionState.IDLE:
                    if not pot_above_zero:
                        self._pot_needs_rearm = False
                        self._record_intent(ControlIntent.PAUSE, InputSource.POT)
                    elif not self._pot_needs_rearm:
                        self._record_intent(ControlIntent.START, InputSource.POT)

            if self._state == SessionState.ARMED_OR_RUNNING and pot_above_zero:
                self._run_mode = RunMode.POT

        if ti.has_footswitch:
            if not self._has_footswitch_sample:
                self._has_footswitch_sample = True
                self._footswitch = ti.footswitch
            elif self._footswitch != ti.footswitch:
                self._footswitch = ti.footswitch
                if self._state != SessionState.IDLE:
                    intent = ControlIntent.START if self._footswitch else ControlIntent.PAUSE
                    self._record_intent(intent, InputSource.FOOTSWITCH)

        self._apply_pending_intent()
        self._apply_power()
        self._winder.tick()

    def snapshot(self) -> SessionSnapshot:
        return SessionSnapshot(state=self._state, source=self._last_source, run_mode=self._run_mode)
