"""core/lateral_controller.py — Python-level lateral axis manager.

Actual stepper motion is handled by the PRU via the daemon.  This class
tracks high-level lateral state and issues move_to / home_start commands
through PickupController (HAL boundary).

Architecture: APPLICATION layer.  Receives a PickupController and an
optional AppConfig.  Calls only standard socket commands.

State machine updated by on_event(event_dict), which must be called for
every daemon event received by the main loop.
"""

from __future__ import annotations

import logging
from enum import Enum, auto
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .config import AppConfig

_log = logging.getLogger(__name__)


class LatState(Enum):
    FAULT      = auto()
    HOMING     = auto()
    HOMED      = auto()
    POSITIONING = auto()
    WINDING_FWD = auto()
    WINDING_BWD = auto()


class LateralController:
    """Python lateral axis manager for the BBB / daemon architecture."""

    def __init__(self, controller, cfg: "AppConfig | None" = None) -> None:
        if cfg is None:
            from .config import AppConfig
            cfg = AppConfig()

        # Cache hardware values for hot-path use
        hw = cfg.hw
        self._spm: int        = hw.lat_steps_per_mm
        self._max_mm: float   = hw.lat_traverse_max_mm
        self._home_hz: int    = hw.lat_home_hz
        self._trav_hz: int    = hw.lat_traverse_hz
        self._rodage_hz: int  = hw.lat_rodage_hz
        self._accel: int      = hw.lat_accel
        self._sp_rev: int     = hw.sp_steps_per_rev

        self._controller = controller

        self._state: LatState = LatState.HOMED  # assume homed until rehome needed
        self._pass_count: int = 0

        self._lat_start_steps: int = 0
        self._lat_end_steps: int   = 0
        self._lat_hz: int          = 0

        self._last_dir_fwd: bool         = True
        self._pause_on_next_reversal: bool = False
        self._stop_on_next_high: bool      = False
        self._stop_on_next_low: bool       = False
        self._paused_at_reversal: bool     = False

        self._position_steps: int = 0

    # ── State queries ──────────────────────────────────────────────────────────

    @property
    def state(self) -> LatState:
        return self._state

    def is_homed(self) -> bool:
        """True when the axis is at a known safe position (idle or traversing)."""
        return self._state in (
            LatState.HOMED,
            LatState.WINDING_FWD,
            LatState.WINDING_BWD,
        )

    def is_fault(self) -> bool:
        return self._state == LatState.FAULT

    def is_traversing(self) -> bool:
        return self._state in (LatState.WINDING_FWD, LatState.WINDING_BWD)

    def is_busy(self) -> bool:
        """True while a move or homing sequence is in progress."""
        return self._state in (
            LatState.HOMING,
            LatState.POSITIONING,
            LatState.WINDING_FWD,
            LatState.WINDING_BWD,
        )

    def is_positioned_for_start(self) -> bool:
        return (
            self._state == LatState.HOMED
            and abs(self._position_steps - self._lat_start_steps) <= 32
        )

    def is_at_zero(self) -> bool:
        return self._state == LatState.HOMED and abs(self._position_steps) <= 32

    def get_pass_count(self) -> int:
        return self._pass_count

    def get_current_position_mm(self) -> float:
        return self._position_steps / self._spm

    def get_traversal_progress(self) -> float:
        """Normalised 0…1 progress in the current traversal direction."""
        span = self._lat_end_steps - self._lat_start_steps
        if span <= 0:
            return 0.0
        pos = self._position_steps - self._lat_start_steps
        progress = pos / span
        if self._state == LatState.WINDING_BWD:
            progress = 1.0 - progress
        return max(0.0, min(1.0, progress))

    def get_nominal_lat_hz(self) -> int:
        return self._lat_hz

    def is_stop_on_next_high_armed(self) -> bool:
        return self._stop_on_next_high

    def is_stop_on_next_low_armed(self) -> bool:
        return self._stop_on_next_low

    def has_stop_at_next_bound_armed(self) -> bool:
        return (
            self._stop_on_next_high
            or self._stop_on_next_low
            or self._pause_on_next_reversal
        )

    def consume_paused_at_reversal(self) -> bool:
        """Consume and reset the paused-at-reversal one-shot latch."""
        v = self._paused_at_reversal
        self._paused_at_reversal = False
        return v

    # ── Commands ───────────────────────────────────────────────────────────────

    def home(self) -> None:
        """Start homing sequence via daemon."""
        _log.info("[Lateral] Homing started")
        self._state = LatState.HOMING
        self._pass_count = 0
        self._controller.home_start()

    def rehome(self) -> None:
        """Restart homing (e.g. after lat_offset change)."""
        self._state = LatState.HOMING
        self._pass_count = 0
        self._lat_start_steps = 0
        self._lat_end_steps = 0
        self._clear_one_shot_stops()
        _log.info("[Lateral] Rehoming")
        self._controller.home_start()

    def prepare_start_position(self,
                                start_mm: float,
                                speed_hz: int | None = None) -> None:
        """Move carriage to *start_mm* (mm from bobbin base)."""
        if self._state not in (LatState.HOMED, LatState.POSITIONING):
            _log.info("[Lateral] prepare_start_position ignored — state: %s",
                      self._state.name)
            return

        hz = speed_hz if speed_hz is not None else self._trav_hz
        target = int(start_mm * self._spm)
        self._lat_start_steps = target
        self._lat_end_steps   = target

        if abs(self._position_steps - target) <= 32:
            self._state = LatState.HOMED
            return

        self._controller.set_accel(lat_max_speed=hz, lat_accel=self._accel)
        self._controller.move_to(pos=target, axis=1)
        self._state = LatState.POSITIONING
        _log.info("[Lateral] Positioning → %.2f mm (%d steps)", start_mm, target)

    def rodage_to_position(self, pos_mm: float) -> None:
        """Move to *pos_mm* using the break-in (rodage) speed."""
        self.prepare_start_position(pos_mm, self._rodage_hz)

    def park_at_zero(self) -> None:
        """Move carriage back to the home end-stop."""
        self.prepare_start_position(0.0)

    def jog(self, delta_mm: float) -> None:
        """Relative manual jog from the current / in-flight target position."""
        if self._state not in (LatState.HOMED, LatState.POSITIONING):
            return
        base_mm = (
            self._lat_start_steps / self._spm
            if self._state == LatState.POSITIONING
            else self.get_current_position_mm()
        )
        target_mm = max(0.0, min(base_mm + delta_mm, self._max_mm))
        target = int(target_mm * self._spm)
        self._lat_start_steps = target
        self._lat_end_steps   = target
        self._controller.set_accel(lat_max_speed=self._trav_hz,
                                   lat_accel=self._accel)
        self._controller.move_to(pos=target, axis=1)
        self._state = LatState.POSITIONING
        _log.info("[Lateral] Jog → %.2f mm", target_mm)

    def arm_pause_on_next_reversal(self) -> None:
        self._pause_on_next_reversal = True
        self._paused_at_reversal = False

    def arm_stop_at_next_high(self) -> None:
        self._stop_on_next_high = True
        self._paused_at_reversal = False

    def arm_stop_at_next_low(self) -> None:
        self._stop_on_next_low = True
        self._paused_at_reversal = False

    def clear_one_shot_stops(self) -> None:
        self._clear_one_shot_stops()

    def start_winding(self,
                      winding_hz: int,
                      tpp: int,
                      start_mm: float,
                      end_mm: float,
                      speed_scale: float = 1.0) -> None:
        """Start synchronised lateral traversal for winding."""
        if self.is_traversing():
            self.update_winding(winding_hz, tpp, start_mm, end_mm, speed_scale)
            return
        if self._state != LatState.HOMED:
            _log.info("[Lateral] start_winding ignored — state: %s",
                      self._state.name)
            return
        if winding_hz == 0 or tpp <= 0 or end_mm <= start_mm:
            _log.warning(
                "[Lateral] start_winding invalid params: hz=%d tpp=%d "
                "start=%.2f end=%.2f",
                winding_hz, tpp, start_mm, end_mm,
            )
            return

        self._set_traverse_bounds(start_mm, end_mm)
        self._lat_hz = self._calc_lat_hz(winding_hz, tpp,
                                         end_mm - start_mm, speed_scale)
        self._pass_count = 0
        self._paused_at_reversal = False

        self._controller.set_accel(lat_max_speed=max(1, self._lat_hz),
                                   lat_accel=self._accel)

        pos = self._position_steps
        mid = (self._lat_start_steps + self._lat_end_steps) // 2
        # Resume in last direction if paused mid-pass; else go toward near bound
        go_fwd = (
            self._last_dir_fwd
            if self._lat_start_steps < pos < self._lat_end_steps
            else pos <= mid
        )

        if go_fwd:
            self._controller.move_to(pos=self._lat_end_steps, axis=1)
            self._state = LatState.WINDING_FWD
        else:
            self._controller.move_to(pos=self._lat_start_steps, axis=1)
            self._state = LatState.WINDING_BWD

        _log.info("[Lateral] Traversal: %d Hz, tpp=%d, dir=%s",
                  self._lat_hz, tpp, "FWD" if go_fwd else "BWD")

    def update_winding(self,
                       winding_hz: int,
                       tpp: int,
                       start_mm: float,
                       end_mm: float,
                       speed_scale: float = 1.0) -> None:
        """Hot-retarget: update speed / bounds without stopping the motor."""
        if not self.is_traversing():
            return

        new_hz = max(1, self._calc_lat_hz(winding_hz, tpp,
                                          end_mm - start_mm, speed_scale))
        if new_hz != self._lat_hz:
            self._lat_hz = new_hz
            self._controller.set_accel(lat_max_speed=self._lat_hz,
                                       lat_accel=self._accel)

        s = max(0.0, start_mm)
        e = max(s, end_mm)
        new_start = int(s * self._spm)
        new_end   = int(e * self._spm)

        # Only update the bound the carriage is currently heading toward
        if self._state == LatState.WINDING_FWD:
            self._lat_end_steps   = max(new_end,   self._lat_start_steps)
        else:
            self._lat_start_steps = min(new_start, self._lat_end_steps)

    def stop_winding(self) -> None:
        """Stop traversal immediately and return to HOMED."""
        if self._state == LatState.POSITIONING:
            self._state = LatState.HOMED
            _log.info("[Lateral] Positioning cancelled")
            return
        if not self.is_traversing():
            return
        self._last_dir_fwd = (self._state == LatState.WINDING_FWD)
        self._clear_one_shot_stops()
        self._state = LatState.HOMED
        _log.info("[Lateral] Traversal stopped")

    # ── Daemon event handler ───────────────────────────────────────────────────

    def on_event(self, event: dict) -> None:
        """Process one daemon event.  Must be called for every received event."""
        ev = event.get("event")

        if ev == "telem":
            lat = event.get("lat", {})
            self._position_steps = int(lat.get("pos", self._position_steps))
            return

        if ev == "home_complete":
            if self._state == LatState.HOMING:
                self._position_steps = 0
                self._state = LatState.HOMED
                _log.info("[Lateral] ✓ Home complete")
            return

        if ev == "move_complete":
            pos = int(event.get("pos", self._position_steps))
            self._position_steps = pos
            if self._state == LatState.POSITIONING:
                self._state = LatState.HOMED
                _log.info("[Lateral] Positioning complete at %.2f mm",
                          self.get_current_position_mm())
            elif self._state == LatState.WINDING_FWD:
                self._position_steps = self._lat_end_steps
                self._pass_count += 1
                self._on_reversal_fwd()
            elif self._state == LatState.WINDING_BWD:
                self._position_steps = self._lat_start_steps
                self._pass_count += 1
                self._on_reversal_bwd()
            return

        if ev in ("endstop_hit", "fault", "limit_hit"):
            _log.warning("[Lateral] Safety event '%s' — stopping", ev)
            if self.is_traversing() or self._state == LatState.POSITIONING:
                self._state = LatState.HOMED
            return

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _on_reversal_fwd(self) -> None:
        _log.info("[Lateral] ↩ FWD pass %d done at %.2f mm",
                  self._pass_count, self.get_current_position_mm())
        if self._pause_on_next_reversal or self._stop_on_next_high:
            self._pause_on_next_reversal = False
            self._stop_on_next_high = False
            self._paused_at_reversal = True
            self._state = LatState.HOMED
            _log.info("[Lateral] ⏸ Stopped at high bound")
            return
        self._controller.move_to(pos=self._lat_start_steps, axis=1)
        self._state = LatState.WINDING_BWD

    def _on_reversal_bwd(self) -> None:
        _log.info("[Lateral] ↪ BWD pass %d done at %.2f mm",
                  self._pass_count, self.get_current_position_mm())
        if self._pause_on_next_reversal or self._stop_on_next_low:
            self._pause_on_next_reversal = False
            self._stop_on_next_low = False
            self._paused_at_reversal = True
            self._state = LatState.HOMED
            _log.info("[Lateral] ⏸ Stopped at low bound")
            return
        self._controller.move_to(pos=self._lat_end_steps, axis=1)
        self._state = LatState.WINDING_FWD

    def _calc_lat_hz(self,
                     winding_hz: int,
                     tpp: int,
                     eff_width_mm: float,
                     speed_scale: float) -> int:
        """Lateral traversal speed in steps/s.

        Formula:  lat_hz = eff_width_steps × winding_hz / (tpp × sp_steps_per_rev)
        """
        if tpp <= 0 or winding_hz == 0 or eff_width_mm <= 0.0:
            return 0
        eff_steps = eff_width_mm * self._spm
        hz = eff_steps * winding_hz / (tpp * self._sp_rev)
        hz *= max(0.4, min(1.8, speed_scale))
        return max(1, int(hz))

    def _set_traverse_bounds(self, start_mm: float, end_mm: float) -> None:
        s = max(0.0, start_mm)
        e = max(s, end_mm)
        self._lat_start_steps = int(s * self._spm)
        self._lat_end_steps   = int(e * self._spm)

    def _clear_one_shot_stops(self) -> None:
        self._pause_on_next_reversal = False
        self._stop_on_next_high      = False
        self._stop_on_next_low       = False
        self._paused_at_reversal     = False
