"""WinderApp - full port of WinderApp.Core, Commands, GeometryPattern, Telemetry, Recipe.

The hardware-facing operations (spindle/lateral step generation) are
delegated to StepperProxy / LateralProxy, which talk to the PRU via IPC.
"""
import math
from config import (
    STEPS_PER_REV, SPEED_HZ_MIN, SPEED_HZ_MAX,
    APPROACH_TURNS, APPROACH_SPEED_HZ_FLOOR,
    ENC_STEP_MM, LAT_RODAGE_SPEED_HZ, LAT_TRAVERSE_MM,
    DEFAULT_TARGET_TURNS, WINDING_MOTOR_INVERTED,
)
from geometry import WindingGeometry, BOBBIN_PRESETS
from pattern import WindingPatternPlanner, WindingRecipe, TraversePlan, STYLE_KEYS
from recipe_store import WindingRecipeStore
from command_registry import CommandId

# ── State enum ────────────────────────────────────────────────────────────────
IDLE = "IDLE"
WINDING = "WINDING"
PAUSED = "PAUSED"
TARGET_REACHED = "TARGET_REACHED"
RODAGE = "RODAGE"

END_POS_NONE = 0
END_POS_HIGH = 1
END_POS_LOW = 2


class WinderApp:
    """Main domain controller.  All hardware calls go through stepper/lateral proxies."""

    def __init__(self, stepper, lateral, recipe_store: WindingRecipeStore | None = None):
        self._stepper = stepper
        self._lateral = lateral
        self._recipe_store = recipe_store or WindingRecipeStore()
        self._recipe = WindingRecipe()
        self._geom = WindingGeometry()
        self._planner = WindingPatternPlanner()
        self._active_plan = TraversePlan()

        # ── State machine ──────────────────────────────────────────────────────
        self._state = IDLE
        self._direction_cw = True
        self._target_turns = DEFAULT_TARGET_TURNS
        self._max_speed_hz = SPEED_HZ_MAX
        self._input_hz = 0
        self._freerun = False
        self._pending_disable = False
        self._pause_requested = False

        # ── Verify flags ───────────────────────────────────────────────────────
        self._verify_low_pending = False
        self._verify_high_pending = False
        self._positioning_to_low = False
        self._end_pos_armed = False

        # ── Rodage ─────────────────────────────────────────────────────────────
        self._rodage_passes = 10
        self._rodage_dist_mm = 80.0
        self._rodage_pass_done = 0
        self._rodage_fwd = True

    # ── Init ──────────────────────────────────────────────────────────────────
    def begin(self):
        self._recipe_store.begin()
        self._recipe = WindingRecipe()
        if self._recipe_store.load(self._recipe):
            print("[Recipe] Recipe restored.")
        else:
            print("[Recipe] Default recipe.")
        self._apply_recipe(self._recipe, persist=False)
        self._active_plan = self._planner.get_plan(0, 0.0)

    # ── Public speed/mode setters (called by SessionController) ───────────────
    def set_control_hz(self, hz: int): self._input_hz = int(hz)
    def get_max_speed_hz(self) -> int: return self._max_speed_hz
    def set_target_turns(self, t: int):
        if t > 0: self._target_turns = t
    def set_freerun(self, f: bool): self._freerun = f
    def set_direction(self, cw: bool): self._direction_cw = cw

    def set_max_rpm(self, rpm: int):
        rpm = max(10, min(1500, rpm))
        self._max_speed_hz = rpm * STEPS_PER_REV // 60

    def pause_winding(self): self._toPaused()
    def stop_winding(self): self._toIdle()

    # ── Main tick ─────────────────────────────────────────────────────────────
    def tick(self):
        if self._pause_requested:
            self._pause_requested = False
            if self._state not in (IDLE, TARGET_REACHED):
                self._toPaused()

        self._handle_lateral_events()
        self._process_input_hz(self._input_hz)
        self._check_auto_stop()
        self._apply_deferred_disable()

    # ── State transitions ─────────────────────────────────────────────────────
    def _toIdle(self):
        prev = self._state
        self._state = IDLE
        self._pending_disable = True
        self._verify_low_pending = False
        self._verify_high_pending = False
        self._positioning_to_low = False
        self._end_pos_armed = False
        self._stepper.stop()
        self._lateral.stop_winding()
        self._lateral.park_at_zero()
        self._stepper.reset_turns()
        self._planner.reset()
        self._active_plan = self._planner.get_plan(0, 0.0)
        print(f"[IDLE] {prev} -> IDLE")

    def _toWinding(self):
        self._state = WINDING
        self._pending_disable = False
        self._positioning_to_low = False
        self._end_pos_armed = False
        print(f"[WINDING] {self._winding_start_mm():.2f} -> {self._winding_end_mm():.2f} mm")

    def _toPaused(self):
        self._state = PAUSED
        self._pending_disable = True
        self._stepper.stop()
        self._lateral.stop_winding()
        print("[PAUSED]")

    def _toTargetReached(self):
        self._state = TARGET_REACHED
        self._pending_disable = True
        self._stepper.stop()
        self._lateral.stop_winding()
        self._end_pos_armed = False
        print(f"[TARGET_REACHED] {self._stepper.get_turns()} turns")

    def _toRodage(self):
        self._state = RODAGE
        self._pending_disable = True
        self._stepper.stop()
        self._lateral.stop_winding()
        self._rodage_pass_done = 0
        self._rodage_fwd = True
        if self._lateral.is_homed():
            self._lateral.prepare_start_position(self._rodage_dist_mm, LAT_RODAGE_SPEED_HZ)
        print(f"[RODAGE] {self._rodage_passes} passes, dist={self._rodage_dist_mm:.1f} mm")

    # ── Geometry helpers ──────────────────────────────────────────────────────
    def _winding_start_mm(self): return self._geom.winding_start_mm()
    def _winding_end_mm(self): return self._geom.winding_end_mm()

    # ── Lateral events ────────────────────────────────────────────────────────
    def _handle_lateral_events(self):
        self._lateral.update()

        if self._positioning_to_low and self._lateral.is_homed() and not self._lateral.is_busy():
            self._positioning_to_low = False
            self._verify_low_pending = False
            self._state = PAUSED
            self._pending_disable = True
            print(f"[VERIFY] Low bound reached at {self._winding_start_mm():.2f} mm")
            return

        if self._state == WINDING and self._lateral.consume_paused_at_reversal():
            if self._verify_high_pending:
                self._verify_high_pending = False
                self._toPaused()
                print(f"[VERIFY] High bound at {self._winding_end_mm():.2f} mm")
            else:
                self._toPaused()
            return

        if self._state == RODAGE and self._lateral.is_homed():
            if self._rodage_fwd:
                self._rodage_fwd = False
                self._lateral.prepare_start_position(0.0, LAT_RODAGE_SPEED_HZ)
            else:
                self._rodage_pass_done += 1
                if self._rodage_pass_done >= self._rodage_passes:
                    print(f"[RODAGE] Complete ({self._rodage_pass_done} passes) - IDLE")
                    self._toIdle()
                else:
                    self._rodage_fwd = True
                    self._lateral.prepare_start_position(self._rodage_dist_mm, LAT_RODAGE_SPEED_HZ)

    # ── Input processing ──────────────────────────────────────────────────────
    def _process_input_hz(self, hz: int):
        if self._state == IDLE:
            if self._lateral.is_homed() and not self._lateral.is_busy() and not self._lateral.is_at_zero():
                self._lateral.park_at_zero()
        elif self._state == WINDING:
            if hz > 0:
                self._run_winding_at_hz(hz)
            else:
                self._toPaused()

    def _run_winding_at_hz(self, hz: int):
        hz = min(hz, self._max_speed_hz)

        # Approach zone cap
        approach_cap = 2**31
        if not self._freerun and self._stepper.is_running():
            remaining = self._target_turns - self._stepper.get_turns()
            if 0 < remaining <= APPROACH_TURNS:
                ratio = remaining / APPROACH_TURNS
                approach_cap = int(APPROACH_SPEED_HZ_FLOOR + ratio * (SPEED_HZ_MAX - APPROACH_SPEED_HZ_FLOOR))

        self._active_plan = self._planner.get_plan(
            self._stepper.get_turns(),
            self._lateral.get_traversal_progress()
        )

        hz_nominal = hz

        # Compensation
        if self._lateral.is_traversing():
            nom_lat = self._lateral.get_instantaneous_lat_hz_nominal()
            act_lat = self._lateral.get_actual_velocity_hz()
            if nom_lat > 0:
                hz = int(hz * act_lat / nom_lat)
                hz = max(SPEED_HZ_MIN, min(SPEED_HZ_MAX, hz))

        # Approach cap
        if approach_cap < 2**31:
            hz = min(hz, approach_cap)

        self._stepper.set_speed_hz(hz)

        if not self._stepper.is_running():
            forward = self._direction_cw != WINDING_MOTOR_INVERTED
            self._state = WINDING
            self._stepper.start(forward)
            self._lateral.start_winding(
                hz_nominal, self._active_plan.turns_per_pass,
                self._winding_start_mm(), self._winding_end_mm(),
                self._active_plan.speed_scale
            )
            return

        if self._state == WINDING and self._end_pos_armed:
            if self._recipe.end_pos == END_POS_HIGH:
                self._lateral.arm_stop_at_next_high()
            elif self._recipe.end_pos == END_POS_LOW:
                self._lateral.arm_stop_at_next_low()
            if self._lateral.get_state() == "HOMED":
                return

        if self._verify_high_pending:
            self._lateral.arm_stop_at_next_high()

        if self._lateral.get_state() == "HOMED":
            self._lateral.start_winding(
                hz_nominal, self._active_plan.turns_per_pass,
                self._winding_start_mm(), self._winding_end_mm(),
                self._active_plan.speed_scale
            )
        else:
            self._lateral.update_winding(
                hz_nominal, self._active_plan.turns_per_pass,
                self._winding_start_mm(), self._winding_end_mm(),
                self._active_plan.speed_scale
            )

    def _check_auto_stop(self):
        if self._state != WINDING or self._freerun or not self._stepper.is_running():
            return
        if self._recipe.end_pos != END_POS_NONE:
            remaining = self._target_turns - self._stepper.get_turns()
            if remaining <= self._recipe.end_pos_turns and not self._end_pos_armed:
                self._end_pos_armed = True
                if self._recipe.end_pos == END_POS_HIGH:
                    self._lateral.arm_stop_at_next_high()
                elif self._recipe.end_pos == END_POS_LOW:
                    self._lateral.arm_stop_at_next_low()
        if self._stepper.get_turns() >= self._target_turns:
            self._toTargetReached()

    def _apply_deferred_disable(self):
        if self._pending_disable:
            self._pending_disable = False
            self._stepper.disable()

    # ── Command dispatch ─────────────────────────────────────────────────────
    def handle_command(self, cmd_id: str, value: str):
        if self._handle_immediate_command(cmd_id, value): return
        if self._handle_geometry_command(cmd_id, value): return
        self._handle_other_command(cmd_id, value)

    def _handle_immediate_command(self, cmd_id: str, value: str) -> bool:
        if cmd_id == CommandId.Stop:
            self._toIdle(); return True
        if cmd_id == CommandId.Reset:
            self._toIdle(); print("[IDLE] Turn counter reset"); return True
        if cmd_id == CommandId.Start:
            if self._state in (IDLE, TARGET_REACHED):
                if not self._lateral.is_homed() or self._lateral.is_busy():
                    print("[Start] Lateral axis not ready"); return True
                self._stepper.reset_turns()
                self._planner.reset()
                self._verify_low_pending = True
                self._verify_high_pending = True
                self._positioning_to_low = True
                self._state = PAUSED
                self._pending_disable = False
                self._lateral.prepare_start_position(self._winding_start_mm())
                print(f"[START] Positioning to low bound {self._winding_start_mm():.2f} mm")
                return True
            if self._state == PAUSED:
                if self._positioning_to_low:
                    print("[Start] Ignored - carriage positioning"); return True
                self._toWinding(); return True
            return True
        if cmd_id == CommandId.Pause:
            if self._state in (IDLE, TARGET_REACHED):
                print("[Pause] Ignored")
            else:
                self._pause_requested = True
            return True
        if cmd_id == CommandId.Resume:
            if self._state == PAUSED:
                self._toWinding()
            return True
        if cmd_id == CommandId.MaxRpm:
            rpm = max(10, min(1500, int(value)))
            self._max_speed_hz = rpm * STEPS_PER_REV // 60
            return True
        if cmd_id == CommandId.StopNextHigh:
            self._lateral.arm_stop_at_next_high(); return True
        if cmd_id == CommandId.StopNextLow:
            self._lateral.arm_stop_at_next_low(); return True
        if cmd_id == CommandId.EndPos:
            m = {"none": 0, "high": 1, "low": 2}
            self._recipe.end_pos = m.get(value, 0)
            self._save_recipe(); return True
        if cmd_id == CommandId.EndPosTurns:
            self._recipe.end_pos_turns = max(1, min(20, int(value)))
            self._save_recipe(); return True
        if cmd_id == CommandId.RodageDist:
            self._rodage_dist_mm = max(5.0, min(float(LAT_TRAVERSE_MM), float(value)))
            return True
        if cmd_id == CommandId.RodagePasses:
            self._rodage_passes = max(1, min(200, int(value))); return True
        if cmd_id == CommandId.Rodage:
            if self._state == IDLE: self._toRodage()
            return True
        if cmd_id == CommandId.RodageStop:
            if self._state == RODAGE: self._toIdle()
            return True
        return False

    def _handle_geometry_command(self, cmd_id: str, value: str) -> bool:
        def clamp(v, lo, hi): return max(lo, min(hi, v))

        def apply_trim(d_start, d_end):
            self._geom.winding_start_trim_mm = clamp(self._geom.winding_start_trim_mm + d_start, -5.0, 5.0)
            self._geom.winding_end_trim_mm = clamp(self._geom.winding_end_trim_mm + d_end, -5.0, 5.0)
            self._save_recipe()

        if cmd_id == CommandId.GeomStartTrim:
            self._geom.winding_start_trim_mm = clamp(float(value), -5.0, 5.0)
            self._save_recipe(); return True
        if cmd_id == CommandId.GeomEndTrim:
            self._geom.winding_end_trim_mm = clamp(float(value), -5.0, 5.0)
            self._save_recipe(); return True
        if cmd_id == CommandId.WindowShift:
            d = clamp(float(value), -5.0, 5.0); apply_trim(d, d); return True
        if cmd_id == CommandId.GeomStartTrimNudge:
            apply_trim(clamp(float(value), -1.0, 1.0), 0.0); return True
        if cmd_id == CommandId.GeomEndTrimNudge:
            apply_trim(0.0, clamp(float(value), -1.0, 1.0)); return True
        if cmd_id == CommandId.GeomPreset:
            self._geom.apply_preset(int(value)); self._save_recipe(); return True
        if cmd_id == CommandId.GeomTotal:
            self._geom.total_width_mm = clamp(float(value), 0.0, 200.0); self._save_recipe(); return True
        if cmd_id == CommandId.GeomBottom:
            self._geom.flange_bottom_mm = clamp(float(value), 0.0, 50.0); self._save_recipe(); return True
        if cmd_id == CommandId.GeomTop:
            self._geom.flange_top_mm = clamp(float(value), 0.0, 50.0); self._save_recipe(); return True
        if cmd_id == CommandId.GeomMargin:
            self._geom.margin_mm = clamp(float(value), 0.0, 20.0); self._save_recipe(); return True
        if cmd_id == CommandId.GeomWire:
            self._geom.wire_diameter_mm = clamp(float(value), 0.01, 1.0); self._save_recipe(); return True
        if cmd_id == CommandId.GeomTppOffset:
            self._geom.turns_per_pass_offset = clamp(int(value), -2000, 2000); self._save_recipe(); return True
        if cmd_id == CommandId.GeomScatter:
            self._geom.scatter_factor = clamp(float(value), 0.5, 5.0); self._save_recipe(); return True
        return False

    def _handle_other_command(self, cmd_id: str, value: str):
        if cmd_id == CommandId.Target:
            t = int(value)
            if t > 0:
                self._target_turns = t
                if self._state == TARGET_REACHED and t > self._stepper.get_turns():
                    self._toPaused()
                self._save_recipe()
        elif cmd_id == CommandId.Freerun:
            self._freerun = (value == "true")
            self._save_recipe()
        elif cmd_id == CommandId.Direction:
            self._direction_cw = (value == "cw")
            self._save_recipe()
        elif cmd_id == CommandId.RecipeImport:
            r = WindingRecipe()
            if self._recipe_store.from_json(value, r):
                self._apply_recipe(r, persist=True)
                print("[Recipe] Imported")
            else:
                print("[Recipe] ERROR - invalid JSON")
        elif cmd_id == CommandId.LatOffset:
            mm = max(0.0, min(float(LAT_TRAVERSE_MM), float(value)))
            self._lateral.set_home_offset(mm)
            self._toIdle()
            self._lateral.rehome()
            self._save_recipe()
        elif cmd_id in (
            CommandId.WindingStyle, CommandId.WindingSeed,
            CommandId.WindingLayerJitter, CommandId.WindingLayerSpeed,
            CommandId.WindingHumanTraverse, CommandId.WindingHumanSpeed,
            CommandId.WindingFirstPassTraverse,
        ):
            # Pattern commands (locked during session)
            if self._state != IDLE:
                print(f"[Lock] Ignored during session ({self._state})")
                return
            self._handle_pattern_command(cmd_id, value)

    def _handle_pattern_command(self, cmd_id: str, value: str):
        def cl(v, lo, hi): return max(lo, min(hi, v))
        if cmd_id == CommandId.WindingStyle:
            if value in STYLE_KEYS:
                self._recipe.style = value
                self._planner.set_recipe(self._capture_recipe())
                self._save_recipe()
        elif cmd_id == CommandId.WindingSeed:
            self._recipe.seed = max(1, int(value))
            self._planner.set_recipe(self._capture_recipe()); self._save_recipe()
        elif cmd_id == CommandId.WindingLayerJitter:
            self._recipe.layer_jitter_pct = cl(float(value), 0.0, 0.45)
            self._planner.set_recipe(self._capture_recipe()); self._save_recipe()
        elif cmd_id == CommandId.WindingLayerSpeed:
            self._recipe.layer_speed_pct = cl(float(value), 0.0, 0.45)
            self._planner.set_recipe(self._capture_recipe()); self._save_recipe()
        elif cmd_id == CommandId.WindingHumanTraverse:
            self._recipe.human_traverse_pct = cl(float(value), 0.0, 0.45)
            self._planner.set_recipe(self._capture_recipe()); self._save_recipe()
        elif cmd_id == CommandId.WindingHumanSpeed:
            self._recipe.human_speed_pct = cl(float(value), 0.0, 0.45)
            self._planner.set_recipe(self._capture_recipe()); self._save_recipe()
        elif cmd_id == CommandId.WindingFirstPassTraverse:
            self._recipe.first_pass_traverse_factor = cl(float(value), 0.40, 1.80)
            self._planner.set_recipe(self._capture_recipe()); self._save_recipe()

    def handle_encoder_delta(self, delta: int):
        if delta == 0 or self._state != PAUSED:
            return
        pos = self._lateral.get_current_position_mm()
        if abs(pos - self._winding_end_mm()) < 0.5:
            step = delta * ENC_STEP_MM
            self._lateral.jog(step)
            new_pos = self._lateral.get_target_position_mm()
            self._geom.winding_end_trim_mm = max(-5.0, min(5.0,
                new_pos - (self._geom.total_width_mm - self._geom.flange_top_mm - self._geom.margin_mm)))
            self._save_recipe()
        elif abs(pos - self._winding_start_mm()) < 0.5:
            step = delta * ENC_STEP_MM
            self._lateral.jog(step)
            new_pos = self._lateral.get_target_position_mm()
            self._geom.winding_start_trim_mm = max(-5.0, min(5.0,
                new_pos - (self._geom.flange_bottom_mm + self._geom.margin_mm)))
            self._save_recipe()

    # ── Recipe helpers ───────────────────────────────────────────────────────
    def _apply_recipe(self, recipe: WindingRecipe, persist: bool):
        self._recipe = recipe
        self._geom = recipe.geometry
        self._target_turns = recipe.target_turns
        self._freerun = recipe.freerun
        self._direction_cw = recipe.direction_cw
        self._state = IDLE
        self._pending_disable = False
        self._lateral.set_home_offset(recipe.lat_offset_mm)
        self._planner.set_recipe(self._recipe)
        self._active_plan = self._planner.get_plan(0, 0.0)
        if persist:
            self._save_recipe()

    def _capture_recipe(self) -> WindingRecipe:
        from config import PICKUP_RECIPE_FORMAT_VERSION
        r = WindingRecipe()
        r.version = PICKUP_RECIPE_FORMAT_VERSION
        r.target_turns = self._target_turns
        r.freerun = self._freerun
        r.direction_cw = self._direction_cw
        r.style = self._recipe.style
        r.seed = self._recipe.seed
        r.layer_jitter_pct = self._recipe.layer_jitter_pct
        r.layer_speed_pct = self._recipe.layer_speed_pct
        r.human_traverse_pct = self._recipe.human_traverse_pct
        r.human_speed_pct = self._recipe.human_speed_pct
        r.first_pass_traverse_factor = self._recipe.first_pass_traverse_factor
        r.lat_offset_mm = self._lateral.get_home_offset()
        r.end_pos = self._recipe.end_pos
        r.end_pos_turns = self._recipe.end_pos_turns
        r.geometry = self._geom
        return r

    def _save_recipe(self):
        self._recipe = self._capture_recipe()
        self._planner.set_recipe(self._recipe)
        self._recipe_store.save(self._recipe)

    def recipe_json(self) -> str:
        return self._recipe_store.to_json(self._capture_recipe())

    # ── Status snapshot ─────────────────────────────────────────────────────
    def get_status(self) -> dict:
        sp_hz = self._stepper.get_speed_hz()
        rpm = sp_hz * 60.0 / STEPS_PER_REV if STEPS_PER_REV else 0.0
        return {
            "rpm": round(rpm, 1),
            "hz": sp_hz,
            "turns": self._stepper.get_turns(),
            "target": self._target_turns,
            "maxRpm": self._max_speed_hz * 60 // STEPS_PER_REV,
            "running": self._stepper.is_running(),
            "enabled": self._state == WINDING,
            "startRequested": self._state != IDLE,
            "carriageReady": self._lateral.is_positioned_for_start(),
            "freerun": self._freerun,
            "cw": self._direction_cw,
            "tpp": float(self._geom.turns_per_pass()),
            "tppCalc": float(self._geom.turns_per_pass_calc()),
            "tppOfs": float(self._geom.turns_per_pass_offset),
            "scatter": self._geom.scatter_factor,
            "pass": self._lateral.get_pass_count(),
            "activeTpp": self._active_plan.turns_per_pass,
            "latScale": self._active_plan.speed_scale,
            "latProgress": self._lateral.get_traversal_progress(),
            "latPos": self._lateral.get_current_position_mm(),
            "wStart": self._winding_start_mm(),
            "wEnd": self._winding_end_mm(),
            "wStartTrim": self._geom.winding_start_trim_mm,
            "wEndTrim": self._geom.winding_end_trim_mm,
            "eff": self._geom.effective_width(),
            "gt": self._geom.total_width_mm,
            "gb": self._geom.flange_bottom_mm,
            "gtp": self._geom.flange_top_mm,
            "gm": self._geom.margin_mm,
            "gw": self._geom.wire_diameter_mm,
            "latOfs": self._lateral.get_home_offset(),
            "wStyle": self._recipe.style,
            "seed": self._recipe.seed,
            "layerJitter": self._recipe.layer_jitter_pct,
            "layerSpeed": self._recipe.layer_speed_pct,
            "humanTraverse": self._recipe.human_traverse_pct,
            "humanSpeed": self._recipe.human_speed_pct,
            "firstPassTraverse": self._recipe.first_pass_traverse_factor,
            "rodageMode": self._state == RODAGE,
            "rodagePass": self._rodage_pass_done,
            "rodagePasses": self._rodage_passes,
            "rodageDist": self._rodage_dist_mm,
            "endPos": self._recipe.end_pos,
            "endPosTurns": self._recipe.end_pos_turns,
            "verifyLow": self._verify_low_pending or self._positioning_to_low,
            "verifyHigh": self._verify_high_pending,
            "state": self._state,
        }
