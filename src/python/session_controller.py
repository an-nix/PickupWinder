"""SessionController - input arbitration (pot / IHM / footswitch).

Mirrors SessionController.cpp / SessionController.h.
Last-event-wins. Pot-lock when another source takes over while pot > 0.
IDLE: only IHM can produce a Start intent.
"""
from config import SPEED_HZ_MIN, SPEED_HZ_MAX

# ── Source constants ───────────────────────────────────────────────────────────
SRC_NONE = 0
SRC_POT = 1
SRC_IHM = 2
SRC_FOOTSWITCH = 3

# ── Run-mode constants ─────────────────────────────────────────────────────────
MODE_NONE = 0
MODE_MAX = 1      # IHM or footswitch: run at max_speed_hz
MODE_POT = 2      # Pot: proportional


class TickInput:
    """Collected inputs for a single control tick."""
    __slots__ = ['pot_hz', 'footswitch_pressed', 'footswitch_changed', 'pending_commands']

    def __init__(self):
        self.pot_hz: int = 0
        self.footswitch_pressed: bool = False
        self.footswitch_changed: bool = False
        self.pending_commands: list = []   # list of (cmd_id, value)


class SessionController:
    """Arbitrates between Pot, IHM (WebSocket/UART), and Footswitch sources."""

    def __init__(self, winder_app):
        self._app = winder_app

        # Arbitration state
        self._active_source = SRC_NONE
        self._run_mode = MODE_NONE
        self._pot_locked = False
        self._pot_hz_last = 0
        self._footswitch_last = False

    # ── Main tick ──────────────────────────────────────────────────────────────
    def tick(self, inp: TickInput):
        # 1. Process queued IHM commands first
        for (cmd_id, value) in inp.pending_commands:
            self._app.handle_command(cmd_id, value)

        # 2. Footswitch edge detection
        if inp.footswitch_changed:
            self._on_footswitch(inp.footswitch_pressed)

        # 3. Pot logic
        pot_hz = inp.pot_hz
        self._update_pot_lock(pot_hz)
        if not self._pot_locked:
            self._on_pot(pot_hz)

        # 4. Push current control intent to app
        self._apply_speed_intent()

    # ── IHM start/stop ────────────────────────────────────────────────────────
    def ihm_start(self):
        """Called when UI sends a Start command."""
        self._active_source = SRC_IHM
        self._run_mode = MODE_MAX
        self._lock_pot_if_active()
        self._app.handle_command("start", "")

    def ihm_pause(self):
        self._active_source = SRC_IHM
        self._run_mode = MODE_NONE
        self._app.handle_command("pause", "")

    def ihm_stop(self):
        self._active_source = SRC_NONE
        self._run_mode = MODE_NONE
        self._pot_locked = False
        self._app.handle_command("stop", "")

    # ── Internals ─────────────────────────────────────────────────────────────
    def _on_footswitch(self, pressed: bool):
        if pressed:
            # Toggle: if running/paused → pause; if paused → resume; if IDLE → start
            state = self._app._state
            if state == "WINDING":
                self._active_source = SRC_FOOTSWITCH
                self._run_mode = MODE_NONE
                self._lock_pot_if_active()
                self._app.pause_winding()
            elif state == "PAUSED":
                self._active_source = SRC_FOOTSWITCH
                self._run_mode = MODE_MAX
                self._lock_pot_if_active()
                self._app._toWinding()
            # IDLE: footswitch cannot start; only IHM can
        else:
            # Release: if we were the active source running at MAX, go to NONE
            if self._active_source == SRC_FOOTSWITCH and self._run_mode == MODE_MAX:
                pass  # keep state — user must press again to toggle

    def _on_pot(self, pot_hz: int):
        if pot_hz > 0:
            if self._active_source != SRC_POT:
                # Pot only takes over if not in IDLE
                state = self._app._state
                if state not in ("IDLE", "TARGET_REACHED"):
                    self._active_source = SRC_POT
                    self._run_mode = MODE_POT
        else:
            if self._active_source == SRC_POT:
                self._active_source = SRC_NONE
                self._run_mode = MODE_NONE
        self._pot_hz_last = pot_hz

    def _update_pot_lock(self, pot_hz: int):
        """Release pot lock once pot returns to zero."""
        if self._pot_locked and pot_hz == 0:
            self._pot_locked = False

    def _lock_pot_if_active(self):
        """Lock pot if it is currently non-zero."""
        if self._pot_hz_last > 0:
            self._pot_locked = True

    def _apply_speed_intent(self):
        state = self._app._state
        if self._run_mode == MODE_POT:
            hz = self._pot_hz_last
            if state == "PAUSED" and hz > 0:
                self._app._toWinding()
            self._app.set_control_hz(hz)
        elif self._run_mode == MODE_MAX:
            self._app.set_control_hz(self._app.get_max_speed_hz())
        else:
            self._app.set_control_hz(0)
