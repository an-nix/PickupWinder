"""core/winder_app.py — Application-layer winding domain controller.

Full migration of WinderApp (WinderApp.Core/Commands/GeometryPattern/Telemetry).

HAL boundary: this class only talks to hardware through PickupController.
No PRU addresses, no IEP knowledge, no raw step intervals here.

All tunables (speeds, turns windows, rodage defaults …) come from AppConfig
so they can be adjusted in config.json without touching Python source.
"""

from __future__ import annotations

import logging
from dataclasses import asdict
from typing import TYPE_CHECKING, Optional

from .geometry import BOBBIN_PRESETS, WindingGeometry, load_presets
from .lateral_controller import LatState, LateralController
from .pattern_planner import WindingPatternPlanner
from .recipe import WindingRecipe
from .types import (
    TickInput,
    TraversePlan,
    WindingEndPos,
    WindingState,
    WindingStyle,
)

if TYPE_CHECKING:
    from .config import AppConfig

_log = logging.getLogger(__name__)


class WinderApp:
    """Application-layer winding state machine.

    Sits between SessionController (input arbitration) and
    PickupController (HAL / daemon socket).
    All constants come from AppConfig to remain JSON-configurable.
    """

    def __init__(self, controller, cfg: "AppConfig | None" = None) -> None:
        if cfg is None:
            from .config import AppConfig
            cfg = AppConfig()

        # Cache frequently used values
        hw = cfg.hw
        self._sp_rev:          int   = hw.sp_steps_per_rev
        self._sp_hz_min:       int   = hw.sp_hz_min
        self._sp_hz_max:       int   = hw.sp_hz_max
        self._lat_max_mm:      float = hw.lat_traverse_max_mm
        self._approach_turns:  int   = cfg.approach_turns
        self._approach_hz_floor: int = cfg.approach_hz_floor
        # Load external presets if present; fall back to built-in BOBBIN_PRESETS
        self._cfg_presets:     list | None = load_presets()

        self._controller = controller
        self._lateral = LateralController(controller, cfg)

        self._state:        WindingState  = WindingState.IDLE
        self._turns:        int           = 0
        self._sp_steps_raw: int           = 0

        self._recipe:      WindingRecipe       = WindingRecipe().normalized()
        self._geom:        WindingGeometry     = self._recipe.geometry
        self._planner:     WindingPatternPlanner = WindingPatternPlanner(self._recipe)
        self._active_plan: TraversePlan        = TraversePlan()

        self._max_speed_hz:  int  = self._sp_hz_max
        self._input_hz:      int  = 0
        self._pause_requested: bool = False
        self._pending_disable: bool = False

        self._verify_low_pending:   bool = False
        self._verify_high_pending:  bool = False
        self._positioning_to_low:   bool = False
        self._end_pos_armed:        bool = False

        # Rodage state — initialised from config defaults
        self._rodage_dist_mm: float = cfg.rodage_dist_mm
        self._rodage_passes:  int   = cfg.rodage_passes
        self._rodage_pass_done: int = 0
        self._rodage_fwd:     bool  = True

        self._on_recipe_changed: Optional[object] = None

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def begin(self) -> None:
        self._state = WindingState.IDLE
        self._controller.emergency_stop()
        self._active_plan = self._planner.get_plan(0, 0.0)
        _log.info(
            "[Winder] Ready — %.1f mm usable, %d tpp, style=%s",
            self._geom.effective_width(),
            self._geom.turns_per_pass(),
            self._recipe.style.value,
        )

    def apply_recipe(self, recipe: WindingRecipe) -> None:
        prev_offset = self._recipe.lat_offset_mm
        self._recipe = recipe.normalized()
        self._geom   = self._recipe.geometry
        self._planner.set_recipe(self._recipe)
        self._active_plan = self._planner.get_plan(self._turns, 0.0)
        if abs(self._recipe.lat_offset_mm - prev_offset) > 0.01:
            _log.info("[Recipe] lat_offset changed — rehoming lateral")
            self._lateral.rehome()

    def set_recipe_changed_callback(self, cb) -> None:
        self._on_recipe_changed = cb

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def state(self) -> WindingState:
        return self._state

    @property
    def max_speed_hz(self) -> int:
        return self._max_speed_hz

    @property
    def turns(self) -> int:
        return self._turns

    @property
    def lateral(self) -> LateralController:
        return self._lateral

    @property
    def geometry(self) -> WindingGeometry:
        return self._geom

    @property
    def recipe(self) -> WindingRecipe:
        return self._recipe

    # ── SessionController interface ────────────────────────────────────────────

    def set_control_hz(self, hz: int) -> None:
        self._input_hz = max(0, int(hz))

    def set_target_turns(self, turns: int) -> None:
        turns = max(1, int(turns))
        self._recipe.target_turns = turns
        if self._state == WindingState.TARGET_REACHED and turns > self._turns:
            self._to_paused()
            _log.info("[PAUSED] Target raised — resume possible")
        self._save_recipe()

    def set_freerun(self, enabled: bool) -> None:
        self._recipe.freerun = bool(enabled)
        self._save_recipe()

    def set_direction_cw(self, cw: bool) -> None:
        self._recipe.direction_cw = bool(cw)
        self._save_recipe()

    def set_max_rpm(self, rpm: int) -> None:
        max_rpm = max(1, int(self._sp_hz_max * 60 / self._sp_rev))
        rpm = max(10, min(max_rpm, int(rpm)))
        self._max_speed_hz = int(rpm * self._sp_rev / 60)
        _log.info("[MaxRPM] %d RPM → %d Hz", rpm, self._max_speed_hz)

    def start_winding(self) -> None:
        if self._state in (WindingState.IDLE, WindingState.TARGET_REACHED):
            if not self._lateral.is_homed() or self._lateral.is_busy():
                _log.warning("[Start] Lateral axis not ready — home first")
                return
            self._controller.reset_position(axis=0)
            self._turns = 0
            self._sp_steps_raw = 0
            self._planner.reset()
            self._verify_low_pending  = True
            self._verify_high_pending = True
            self._positioning_to_low  = True
            self._state = WindingState.PAUSED
            self._pending_disable = False
            self._lateral.prepare_start_position(self._winding_start_mm())
            _log.info("[START] Positioning to low bound %.2f mm",
                      self._winding_start_mm())
            return

        if self._state == WindingState.PAUSED:
            if self._positioning_to_low:
                _log.warning("[Start] Still positioning to low bound — wait")
                return
            self._to_winding()

    def pause_winding(self) -> None:
        self._pause_requested = True

    def stop_winding(self) -> None:
        self._to_idle()

    # ── Daemon event routing ───────────────────────────────────────────────────

    def on_event(self, event: dict) -> None:
        """Route one daemon event.  Call for every event received by the loop."""
        ev = event.get("event")
        if ev == "telem":
            sp_steps = int(event.get("sp", {}).get("steps", self._sp_steps_raw))
            if sp_steps != self._sp_steps_raw:
                self._sp_steps_raw = sp_steps
                self._turns = self._sp_steps_raw // self._sp_rev
        self._lateral.on_event(event)

    # ── Main tick ──────────────────────────────────────────────────────────────

    def tick(self) -> None:
        if self._pause_requested:
            self._pause_requested = False
            if self._state not in (WindingState.IDLE, WindingState.TARGET_REACHED):
                self._to_paused()
        self._handle_lateral_events()
        self._process_input_hz(self._input_hz)
        self._check_auto_stop()
        self._apply_deferred_disable()

    # ── Command dispatch ───────────────────────────────────────────────────────

    def handle_command(self, cmd: str, value: str = "") -> bool:
        if self._handle_immediate_command(cmd, value):
            return True
        if self._handle_geometry_command(cmd, value):
            return True
        if self._parameters_locked():
            _log.info("[Lock] '%s' ignored during active session (%s)",
                      cmd, self._state.name)
            return True
        return self._handle_pattern_command(cmd, value)

    # ── Status ─────────────────────────────────────────────────────────────────

    def status_dict(self) -> dict:
        g   = self._geom
        lat = self._lateral
        return {
            "state":          self._state.name,
            "turns":          self._turns,
            "target_turns":   self._recipe.target_turns,
            "freerun":        self._recipe.freerun,
            "direction_cw":   self._recipe.direction_cw,
            "max_speed_hz":   self._max_speed_hz,
            "input_hz":       self._input_hz,
            "rodage_pass_done": self._rodage_pass_done,
            "rodage_passes":  self._rodage_passes,
            "rodage_dist_mm": self._rodage_dist_mm,
            "active_plan":    asdict(self._active_plan),
            "lat_state":      lat.state.name,
            "lat_pass_count": lat.get_pass_count(),
            "lat_position_mm": lat.get_current_position_mm(),
            "lat_progress":   lat.get_traversal_progress(),
            "geom": {
                "tpp":               g.turns_per_pass(),
                "tpp_calc":          g.turns_per_pass_calc(),
                "tpp_offset":        g.turns_per_pass_offset,
                "scatter_factor":    g.scatter_factor,
                "effective_width_mm": g.effective_width(),
                "winding_start_mm":  g.winding_start_mm(),
                "winding_end_mm":    g.winding_end_mm(),
                "total_width_mm":    g.total_width_mm,
                "flange_bottom_mm":  g.flange_bottom_mm,
                "flange_top_mm":     g.flange_top_mm,
                "margin_mm":         g.margin_mm,
                "winding_start_trim_mm": g.winding_start_trim_mm,
                "winding_end_trim_mm":   g.winding_end_trim_mm,
                "wire_diameter_mm":  g.wire_diameter_mm,
            },
            "style":          self._recipe.style.value,
            "seed":           self._recipe.seed,
            "end_pos":        self._recipe.end_pos.value,
            "end_pos_turns":  self._recipe.end_pos_turns,
            "lat_offset_mm":  self._recipe.lat_offset_mm,
        }

    def recipe_dict(self) -> dict:
        return asdict(self._recipe)

    def handle_encoder_delta(self, delta: int) -> None:
        """Encoder jog during PAUSED — adjusts the nearest winding bound trim."""
        if delta == 0 or self._state != WindingState.PAUSED:
            return
        ENC_STEP_MM = 0.05
        pos = self._lateral.get_current_position_mm()
        if abs(pos - self._winding_end_mm()) < 0.5:
            self._lateral.jog(delta * ENC_STEP_MM)
            new_pos = self._lateral.get_current_position_mm()
            self._geom.winding_end_trim_mm = max(
                -5.0, min(5.0, new_pos - (
                    self._geom.total_width_mm
                    - self._geom.flange_top_mm
                    - self._geom.margin_mm)))
            self._save_recipe()
        elif abs(pos - self._winding_start_mm()) < 0.5:
            self._lateral.jog(delta * ENC_STEP_MM)
            new_pos = self._lateral.get_current_position_mm()
            self._geom.winding_start_trim_mm = max(
                -5.0, min(5.0, new_pos - (
                    self._geom.flange_bottom_mm + self._geom.margin_mm)))
            self._save_recipe()

    # ── State transitions ──────────────────────────────────────────────────────

    def _to_idle(self) -> None:
        prev = self._state
        self._state = WindingState.IDLE
        self._pending_disable = True
        self._verify_low_pending  = False
        self._verify_high_pending = False
        self._positioning_to_low  = False
        self._end_pos_armed       = False
        self._turns        = 0
        self._sp_steps_raw = 0
        self._controller.set_speed(sp_hz=0, lat_hz=0)
        self._lateral.stop_winding()
        self._lateral.park_at_zero()
        self._planner.reset()
        self._active_plan = self._planner.get_plan(0, 0.0)
        _log.info("[IDLE] %s → IDLE — carriage to home, counter reset", prev.name)

    def _to_paused(self) -> None:
        self._state = WindingState.PAUSED
        self._pending_disable = True
        self._controller.set_speed(sp_hz=0, lat_hz=0)
        self._lateral.stop_winding()
        _log.info("[PAUSED] Resume when input goes up or press Start")

    def _to_winding(self) -> None:
        self._state = WindingState.WINDING
        self._pending_disable    = False
        self._positioning_to_low = False
        self._end_pos_armed      = False
        _log.info("[WINDING] %.2f → %.2f mm",
                  self._winding_start_mm(), self._winding_end_mm())

    def _to_target_reached(self) -> None:
        self._state = WindingState.TARGET_REACHED
        self._pending_disable = True
        self._controller.set_speed(sp_hz=0, lat_hz=0)
        self._lateral.stop_winding()
        self._end_pos_armed = False
        _log.info("[TARGET_REACHED] %d turns — raise target or press Stop",
                  self._turns)

    def _to_rodage(self) -> None:
        self._state = WindingState.RODAGE
        self._pending_disable  = True
        self._rodage_pass_done = 0
        self._rodage_fwd       = True
        self._controller.set_speed(sp_hz=0, lat_hz=0)
        self._lateral.stop_winding()
        if self._lateral.is_homed():
            self._lateral.rodage_to_position(self._rodage_dist_mm)
        _log.info("[RODAGE] %d passes × %.1f mm",
                  self._rodage_passes, self._rodage_dist_mm)

    # ── Tick helpers ───────────────────────────────────────────────────────────

    def _winding_start_mm(self) -> float:
        return self._geom.winding_start_mm()

    def _winding_end_mm(self) -> float:
        return self._geom.winding_end_mm()

    def _parameters_locked(self) -> bool:
        return self._state in (WindingState.WINDING, WindingState.PAUSED)

    def _handle_lateral_events(self) -> None:
        # Positioning-to-low sequence complete → PAUSED for verify
        if (self._positioning_to_low
                and self._lateral.state == LatState.HOMED
                and not self._lateral.is_busy()):
            self._positioning_to_low = False
            self._verify_low_pending = False
            self._state = WindingState.PAUSED
            self._pending_disable = True
            _log.info("[VERIFY] Low bound %.2f mm reached — press Start",
                      self._winding_start_mm())
            return

        # One-shot bound stop (verify high / endpos)
        if (self._state == WindingState.WINDING
                and self._lateral.consume_paused_at_reversal()):
            if self._verify_high_pending:
                self._verify_high_pending = False
                self._to_paused()
                _log.info("[VERIFY] High bound %.2f mm — press Start to begin",
                          self._winding_end_mm())
            else:
                self._to_paused()
                _log.info("[WINDING] Bound stop → PAUSED")
            return

        # Rodage back-and-forth
        if (self._state == WindingState.RODAGE
                and self._lateral.state == LatState.HOMED
                and not self._lateral.is_busy()):
            if self._rodage_fwd:
                self._rodage_fwd = False
                self._lateral.rodage_to_position(0.0)
            else:
                self._rodage_pass_done += 1
                _log.info("[RODAGE] Pass %d/%d",
                          self._rodage_pass_done, self._rodage_passes)
                if self._rodage_pass_done >= self._rodage_passes:
                    _log.info("[RODAGE] Complete (%d passes) → IDLE",
                              self._rodage_pass_done)
                    self._to_idle()
                else:
                    self._rodage_fwd = True
                    self._lateral.rodage_to_position(self._rodage_dist_mm)

    def _process_input_hz(self, hz: int) -> None:
        if self._state == WindingState.IDLE:
            # Auto-park when carriage drifts away from zero
            if (self._lateral.is_homed()
                    and not self._lateral.is_busy()
                    and not self._lateral.is_at_zero()):
                self._lateral.park_at_zero()
            return
        if self._state == WindingState.WINDING:
            if hz > 0:
                self._run_winding_at_hz(hz)
            else:
                self._to_paused()

    def _run_winding_at_hz(self, hz: int) -> None:
        hz = min(int(hz), self._max_speed_hz)

        # Decelerate spindle in approach window
        approach_cap = self._sp_hz_max
        if not self._recipe.freerun:
            remaining = self._recipe.target_turns - self._turns
            if 0 < remaining <= self._approach_turns:
                ratio = remaining / self._approach_turns
                approach_cap = int(
                    self._approach_hz_floor
                    + ratio * (self._sp_hz_max - self._approach_hz_floor))

        self._active_plan = self._planner.get_plan(
            self._turns, self._lateral.get_traversal_progress())
        traverse_scale = self._active_plan.speed_scale

        if self._end_pos_armed and self._recipe.end_pos != WindingEndPos.NONE:
            traverse_scale = self._calc_end_pos_scale(traverse_scale)

        self._active_plan.speed_scale = traverse_scale
        hz = min(hz, approach_cap)

        if self._lateral.has_stop_at_next_bound_armed():
            hz = self._apply_stop_slowdown(hz)

        hz = max(self._sp_hz_min, hz)
        sp_dir = 0 if self._recipe.direction_cw else 1
        self._controller.enable(sp=True, lat=True)
        self._controller.set_speed(sp_hz=hz, lat_hz=0, sp_dir=sp_dir, lat_dir=0)

        # Arm end-position stop when close enough to target
        if not self._recipe.freerun and self._recipe.end_pos != WindingEndPos.NONE:
            remaining = self._recipe.target_turns - self._turns
            if remaining <= self._recipe.end_pos_turns and not self._end_pos_armed:
                self._end_pos_armed = True
                if self._recipe.end_pos == WindingEndPos.TOP:
                    self._lateral.arm_stop_at_next_high()
                else:
                    self._lateral.arm_stop_at_next_low()
                _log.info("[EndPos] Armed %s stop — %d turns to go",
                          self._recipe.end_pos.value, remaining)

        # Drive lateral traversal
        if self._lateral.state == LatState.HOMED:
            self._lateral.start_winding(
                hz, self._active_plan.turns_per_pass,
                self._winding_start_mm(), self._winding_end_mm(),
                traverse_scale)
        elif self._lateral.is_traversing():
            self._lateral.update_winding(
                hz, self._active_plan.turns_per_pass,
                self._winding_start_mm(), self._winding_end_mm(),
                traverse_scale)

        if self._verify_high_pending:
            self._lateral.arm_stop_at_next_high()

    def _calc_end_pos_scale(self, nominal_scale: float) -> float:
        remaining    = max(0, self._recipe.target_turns - self._turns)
        hold         = max(1, self._recipe.end_pos_turns)
        move_remaining = max(0.25, remaining - hold)
        progress     = self._lateral.get_traversal_progress()
        lat_state    = self._lateral.state
        path_units   = 0.0
        at_target_bound = False

        if self._recipe.end_pos == WindingEndPos.TOP:
            if lat_state == LatState.WINDING_FWD:
                path_units = 1.0 - progress
            elif lat_state == LatState.WINDING_BWD:
                path_units = 2.0 - progress
            elif (lat_state == LatState.HOMED and
                  abs(self._lateral.get_current_position_mm()
                      - self._winding_end_mm()) <= 0.10):
                at_target_bound = True
        elif self._recipe.end_pos == WindingEndPos.BOTTOM:
            if lat_state == LatState.WINDING_BWD:
                path_units = 1.0 - progress
            elif lat_state == LatState.WINDING_FWD:
                path_units = 2.0 - progress
            elif (lat_state == LatState.HOMED and
                  abs(self._lateral.get_current_position_mm()
                      - self._winding_start_mm()) <= 0.10):
                at_target_bound = True

        if (not at_target_bound
                and path_units > 0.0
                and self._active_plan.turns_per_pass > 0):
            nominal = path_units * self._active_plan.turns_per_pass
            desired = nominal / move_remaining
            return max(0.40, min(1.80, desired))
        return nominal_scale

    def _apply_stop_slowdown(self, hz: int) -> int:
        """Slow the spindle as the carriage approaches an armed stop bound."""
        lat_state = self._lateral.state
        slow = (
            (lat_state == LatState.WINDING_FWD
             and self._lateral.is_stop_on_next_high_armed())
            or
            (lat_state == LatState.WINDING_BWD
             and self._lateral.is_stop_on_next_low_armed())
        )
        if not slow:
            return hz
        progress = self._lateral.get_traversal_progress()
        if progress >= 0.80:
            t      = min(1.0, (progress - 0.80) / 0.20)
            factor = 1.0 - 0.70 * t
            return max(self._sp_hz_min, int(hz * factor))
        return hz

    def _check_auto_stop(self) -> None:
        if self._state != WindingState.WINDING or self._recipe.freerun:
            return
        if self._turns >= self._recipe.target_turns:
            self._to_target_reached()

    def _apply_deferred_disable(self) -> None:
        """Disable drivers once all motion has settled."""
        if not self._pending_disable or self._lateral.is_busy():
            return
        self._pending_disable = False
        self._controller.enable(sp=False, lat=False)
        _log.info("[Winder] Drivers disabled")

    # ── Command handlers ───────────────────────────────────────────────────────

    def _handle_immediate_command(self, cmd: str, value: str) -> bool:
        if cmd == "stop":
            self._to_idle()
        elif cmd == "reset":
            self._to_idle()
        elif cmd == "start":
            self.start_winding()
        elif cmd == "pause":
            self._pause_requested = True
        elif cmd == "resume":
            if self._state == WindingState.PAUSED:
                self._to_winding()
        elif cmd == "max_rpm":
            self.set_max_rpm(int(float(value)) if value else 600)
        elif cmd == "stop_next_high":
            self._lateral.arm_stop_at_next_high()
        elif cmd == "stop_next_low":
            self._lateral.arm_stop_at_next_low()
        elif cmd == "end_pos":
            try:
                self._recipe.end_pos = WindingEndPos(value.lower())
            except ValueError:
                self._recipe.end_pos = WindingEndPos.NONE
            self._save_recipe()
        elif cmd == "end_pos_turns":
            self._recipe.end_pos_turns = max(1, min(20, int(float(value))))
            self._save_recipe()
        elif cmd == "rodage_dist":
            self._rodage_dist_mm = max(5.0, min(float(value), self._lat_max_mm))
        elif cmd == "rodage_passes":
            self._rodage_passes = max(1, min(200, int(float(value))))
        elif cmd == "rodage":
            if self._state == WindingState.IDLE:
                self._to_rodage()
        elif cmd == "rodage_stop":
            if self._state == WindingState.RODAGE:
                self._to_idle()
        elif cmd == "target":
            t = int(float(value)) if value else 0
            if t > 0:
                self.set_target_turns(t)
        elif cmd == "freerun":
            self.set_freerun(value.lower() in ("true", "1", "yes", "on"))
        elif cmd == "direction":
            self.set_direction_cw(value.lower() in ("cw", "1", "true"))
        elif cmd == "lat_offset":
            self._recipe.lat_offset_mm = max(0.0, float(value))
            self._save_recipe()
            self._lateral.rehome()
            _log.info("[Lateral] Offset %.2f mm — rehoming", self._recipe.lat_offset_mm)
        else:
            return False
        return True

    def _handle_geometry_command(self, cmd: str, value: str) -> bool:
        g = self._geom

        def _nudge(d_start: float, d_end: float) -> None:
            g.winding_start_trim_mm = max(-5.0, min(5.0,
                                          g.winding_start_trim_mm + d_start))
            g.winding_end_trim_mm   = max(-5.0, min(5.0,
                                          g.winding_end_trim_mm   + d_end))
            self._refresh_carriage(d_start != 0.0, d_end != 0.0)
            self._save_recipe()

        if cmd == "geom_start_trim":
            g.winding_start_trim_mm = max(-5.0, min(5.0, float(value)))
            self._refresh_carriage(True, False)
            self._save_recipe()
        elif cmd == "geom_end_trim":
            g.winding_end_trim_mm = max(-5.0, min(5.0, float(value)))
            self._refresh_carriage(False, True)
            self._save_recipe()
        elif cmd == "geom_start_trim_nudge":
            _nudge(max(-1.0, min(1.0, float(value))), 0.0)
        elif cmd == "geom_end_trim_nudge":
            _nudge(0.0, max(-1.0, min(1.0, float(value))))
        elif cmd == "window_shift":
            d = max(-5.0, min(5.0, float(value)))
            _nudge(d, d)
        elif cmd == "geom_preset":
            idx = int(float(value))
            active_presets = self._cfg_presets or None
            g.apply_preset(idx, active_presets)
            self._refresh_carriage(True, True)
            src = active_presets or BOBBIN_PRESETS
            if 0 <= idx < len(src):
                _log.info("Bobbin preset: %s — %d tpp",
                          src[idx]["name"], g.turns_per_pass())
            self._save_recipe()
        elif cmd == "geom_total":
            g.total_width_mm = max(0.0, min(200.0, float(value)))
            self._refresh_carriage(True, True)
            self._save_recipe()
        elif cmd == "geom_bottom":
            g.flange_bottom_mm = max(0.0, min(50.0, float(value)))
            self._refresh_carriage(True, False)
            self._save_recipe()
        elif cmd == "geom_top":
            g.flange_top_mm = max(0.0, min(50.0, float(value)))
            self._refresh_carriage(False, True)
            self._save_recipe()
        elif cmd == "geom_margin":
            g.margin_mm = max(0.0, min(20.0, float(value)))
            self._refresh_carriage(True, True)
            self._save_recipe()
        elif cmd == "geom_wire":
            g.wire_diameter_mm = max(0.01, min(1.0, float(value)))
            _log.info("Wire %.4f mm → %d tpp (calc %d)",
                      g.wire_diameter_mm, g.turns_per_pass(), g.turns_per_pass_calc())
            self._save_recipe()
        elif cmd == "geom_tpp_offset":
            g.turns_per_pass_offset = max(-2000, min(2000, int(float(value))))
            _log.info("TPP offset %+d → %d tpp",
                      g.turns_per_pass_offset, g.turns_per_pass())
            self._save_recipe()
        elif cmd == "geom_scatter":
            g.scatter_factor = max(0.5, min(5.0, float(value)))
            _log.info("Scatter %.2f → %d tpp", g.scatter_factor, g.turns_per_pass())
            self._save_recipe()
        else:
            return False
        return True

    def _handle_pattern_command(self, cmd: str, value: str) -> bool:
        if cmd == "winding_style":
            try:
                self._recipe.style = WindingStyle(value.lower())
            except ValueError:
                self._recipe.style = WindingStyle.STRAIGHT
            self._planner.set_recipe(self._recipe)
            _log.info("Style: %s", self._recipe.style.value)
            self._save_recipe()
        elif cmd == "winding_seed":
            self._recipe.seed = max(1, int(float(value)))
            self._planner.set_recipe(self._recipe)
            self._save_recipe()
        elif cmd == "winding_layer_jitter":
            self._recipe.layer_jitter_pct = max(0.0, min(0.45, float(value)))
            self._planner.set_recipe(self._recipe)
            self._save_recipe()
        elif cmd == "winding_layer_speed":
            self._recipe.layer_speed_pct = max(0.0, min(0.45, float(value)))
            self._planner.set_recipe(self._recipe)
            self._save_recipe()
        elif cmd == "winding_human_traverse":
            self._recipe.human_traverse_pct = max(0.0, min(0.45, float(value)))
            self._planner.set_recipe(self._recipe)
            self._save_recipe()
        elif cmd == "winding_human_speed":
            self._recipe.human_speed_pct = max(0.0, min(0.45, float(value)))
            self._planner.set_recipe(self._recipe)
            self._save_recipe()
        elif cmd == "winding_first_pass_traverse":
            self._recipe.first_pass_traverse_factor = max(0.40, min(1.80, float(value)))
            self._planner.set_recipe(self._recipe)
            self._save_recipe()
        else:
            return False
        return True

    def _refresh_carriage(self, start_changed: bool, end_changed: bool) -> None:
        if not self._lateral.is_homed() or self._lateral.is_busy():
            return
        if self._state != WindingState.PAUSED:
            return
        pos = self._lateral.get_current_position_mm()
        if start_changed and abs(pos - self._winding_start_mm()) <= 1.0:
            self._lateral.prepare_start_position(self._winding_start_mm())
        elif end_changed and abs(pos - self._winding_end_mm()) <= 1.0:
            self._lateral.prepare_start_position(self._winding_end_mm())

    def _save_recipe(self) -> None:
        if self._on_recipe_changed is not None:
            self._on_recipe_changed(self._recipe)
