"""Motion command service — builds moves and issues transport commands.

``MotionCommandService`` owns all "build a move and enqueue it" logic that was
previously scattered inside ``WindingEngine``.  The engine delegates every
motion-building call here and remains focused on the IDLE/HOMING/RUNNING state
machine and program execution.

The module also exports the free function ``adjust_duration_for_ramp_deficit``
so that ``WindingEngine._run_layer`` can share the same calculation without
creating a circular dependency.
"""

from __future__ import annotations

import logging
import threading
from typing import Any

from core.config import AppConfiguration
from core.lateral import LateralAxisController
from core.shared_state import EngineState, SharedState
from motion import AxisMotionConfig, RampConfig
from motion.move import RampMove, RampMoveConfig
from motion.move_builders import build_jog_move
from motion.move_queue import MoveQueue
from motion.ramp_config import compute_ramp_times
from motion.spindle_kinematics import SpindleKinematics
from transport.spi_transport import Esp32SpiTransport
from winding import ScatterEngine, SyncAxisConfig, WindingPattern, WoundMove


logger = logging.getLogger(__name__)


def _coerce_bool_param(value: Any, *, field_name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        if value in (0, 0.0):
            return False
        if value in (1, 1.0):
            return True
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off", ""}:
            return False
    raise ValueError(f"{field_name} must be a boolean value")


def adjust_duration_for_ramp_deficit(
    *,
    total_turns: float,
    target_rps: float,
    accel_s: float,
    decel_s: float,
) -> float:
    """Return duration corrected for turns lost during accel/decel ramps."""
    if target_rps <= 0.0:
        return 0.05
    turns_deficit = 0.5 * target_rps * (max(accel_s, 0.0) + max(decel_s, 0.0))
    adjusted_turns = max(total_turns, 0.0) + turns_deficit
    return max(adjusted_turns / target_rps, 0.05)


class MotionCommandService:
    """Build motion moves and issue transport commands.

    Owns no state of its own — reads state from *shared_state* and
    *lateral_controller*, and writes to *move_queue* and *transport*.
    The engine calls these methods when it wants to start motion.
    """

    def __init__(
        self,
        *,
        transport: Esp32SpiTransport,
        shared_state: SharedState,
        move_queue: MoveQueue,
        lateral_controller: LateralAxisController,
        config: AppConfiguration,
    ) -> None:
        self._transport = transport
        self._state = shared_state
        self._move_queue = move_queue
        self._lateral = lateral_controller
        self._config = config
        self._manual_home_lock = threading.Lock()
        self._manual_home_thread: threading.Thread | None = None

    # ── Transport commands ──────────────────────────────────────────────────

    def flush_until(self, sequence: int) -> dict[str, Any]:
        """Send a flush request to the firmware and return the resulting status."""
        from core.status import serialize_firmware_status
        status = self._transport.flush_until(sequence)
        return serialize_firmware_status(status)

    def arm_endstop(self, axis_id: int) -> None:
        """Arm the specified endstop through the transport."""
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=True)
        self._transport.wait_for_request_result(sequence)

    def disarm_endstop(self, axis_id: int) -> None:
        """Disarm the specified endstop through the transport."""
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=False)
        self._transport.wait_for_request_result(sequence)

    # ── Move builders ───────────────────────────────────────────────────────

    def jog(
        self,
        axis_id: int,
        steps: int,
        rpm: float,
        reverse: bool = False,
    ) -> None:
        """Execute a jog move immediately. Only allowed when engine is IDLE."""
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError("Jog only allowed when engine is IDLE")
        if steps <= 0:
            raise ValueError("steps must be positive")

        if axis_id == self._config.spindle_axis_id:
            steps_per_rev = (
                self._config.spindle_steps_per_revolution
                * self._config.spindle_microstepping
            )
        elif axis_id == self._config.lateral_axis_id:
            steps_per_rev = (
                self._config.lateral_steps_per_revolution
                * self._config.lateral_microstepping
            )
            if axis_id in self._state.axis_states:
                self._lateral.require_homed()
                delta_steps = -steps if reverse else steps
                self._lateral.ensure_delta_allowed(delta_steps)
        else:
            raise ValueError(f"jog: unsupported axis_id {axis_id}")

        move = build_jog_move(
            name=f"jog_{axis_id}",
            axis_id=axis_id,
            steps=steps,
            steps_per_rev=steps_per_rev,
            rpm=rpm,
            reverse=reverse,
        )
        self._move_queue.enqueue(move)

    def home_lateral(
        self,
    ) -> dict[str, Any]:
        """Start lateral homing asynchronously and return immediately.

        ``backoff_steps`` defaults to 2 full motor revolutions when *None* is
        passed.  This guarantees at least 2 × pitch_mm of mechanical travel
        regardless of the configured leadscrew pitch, which is sufficient to
        release any standard mechanical endstop (typical release travel ≤ 2 mm).
        Pass an explicit integer to override (e.g. for non-standard hardware).
        """
        approach_rpm = self._config.lateral_homing_approach_rpm
        search_rpm = self._config.lateral_homing_search_rpm
        backoff_steps = self._config.lateral_homing_backoff_steps

        if backoff_steps is None:
            steps_per_rev = (
                self._config.lateral_steps_per_revolution
                * self._config.lateral_microstepping
            )
            # 2 full revolutions — pitch-agnostic safe default for endstop release.
            backoff_steps = steps_per_rev * 2
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError(
                "home_lateral only allowed when engine is IDLE; if a FAULT occurred, "
                "acknowledge it with winding.clear_fault first"
            )

        with self._manual_home_lock:
            if self._manual_home_thread is not None and self._manual_home_thread.is_alive():
                raise RuntimeError("home_lateral is already in progress")

        self._state.set_engine_state(EngineState.HOMING)
        monitor = threading.Thread(
            target=self._run_lateral_home,
            args=(approach_rpm, search_rpm, int(backoff_steps)),
            daemon=True,
            name="manual_home_monitor",
        )
        with self._manual_home_lock:
            self._manual_home_thread = monitor
        monitor.start()

        return {
            "status": "started",
            "axis_id": self._config.lateral_axis_id,
            "approach_rpm": approach_rpm,
            "search_rpm": search_rpm,
            "backoff_steps": backoff_steps,  # actual value after default expansion
        }

    def _run_lateral_home(
        self,
        approach_rpm: float,
        search_rpm: float,
        backoff_steps: int,
    ) -> None:
        try:
            success, reason = self._lateral.home(
                axis_id=self._config.lateral_axis_id,
                approach_rpm=approach_rpm,
                search_rpm=search_rpm,
                backoff_steps=backoff_steps,
            )
            if success and self._state.engine_state == EngineState.HOMING:
                self._state.set_engine_state(EngineState.IDLE)
            elif not success and self._state.engine_state == EngineState.HOMING:
                self._state.set_fault(reason or "lateral homing failed")
        except Exception as exc:
            logger.exception("manual lateral homing failed")
            self._state.set_fault(f"manual lateral homing failed: {exc}")
        finally:
            with self._manual_home_lock:
                self._manual_home_thread = None


    def move_lateral_to_mm(
        self,
        position_mm: float,
        rpm: float,
    ) -> dict[str, Any]:
        """Queue a lateral move to an absolute position in mm from home zero."""
        if self._state.engine_state != EngineState.IDLE:
            if self._state.engine_state == EngineState.FAULT:
                raise RuntimeError(
                    "move_lateral_to_mm not allowed while controller is in FAULT; "
                    "acknowledge it with winding.clear_fault before retrying"
                )
            raise RuntimeError("move_lateral_to_mm only allowed when engine is IDLE")

        axis_state = self._lateral.require_homed()
        current_steps = axis_state.position_steps
        if current_steps is None:
            raise RuntimeError("Lateral axis position is unknown; home the axis first")

        target_steps = self._lateral.mm_to_steps(position_mm)
        delta_steps = target_steps - current_steps
        self._lateral.ensure_delta_allowed(delta_steps)

        if delta_steps == 0:
            return {"status": "already_at_position", "position_mm": position_mm}

        self.jog(
            axis_id=self._config.lateral_axis_id,
            steps=abs(delta_steps),
            rpm=rpm,
            reverse=(delta_steps < 0),
        )
        return {
            "status": "queued",
            "target_position_mm": position_mm,
            "target_position_steps": target_steps,
        }

    def wound_run(
        self,
        spindle_axis_id: int,
        traverse_axis_id: int,
        target_rpm: float,
        accel_s: float | None,
        cruise_s: float | None,
        decel_s: float | None,
        bobbin_width_mm: float,
        turns_per_mm: float,
        scatter_amplitude_mm: float = 0.0,
        scatter_damping_margin_mm: float = 0.0,
        scatter_freq1: float = 1.0,
        scatter_freq2: float = 1.618,
        spindle_reverse: bool = False,
        traverse_reverse: bool = False,
    ) -> None:
        """Execute a synchronized winding operation (Electronic Gearing).
        Only allowed when engine is IDLE."""
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError("Winding run only allowed when engine is IDLE")
        if (
            traverse_axis_id == self._config.lateral_axis_id
            and traverse_axis_id in self._state.axis_states
        ):
            self._lateral.require_homed(traverse_axis_id)

        if accel_s is None or cruise_s is None or decel_s is None:
            total_turns = 2.0 * bobbin_width_mm * turns_per_mm
            steps_per_rev = (
                self._config.spindle_steps_per_revolution
                * self._config.spindle_microstepping
            )
            _, computed_accel_s, computed_cruise_s, computed_decel_s = (
                self._compute_duration_and_ramp_times(
                    target_rpm=target_rpm,
                    total_turns=total_turns,
                    max_accel_steps_per_s2=(
                        self._config.spindle_max_acceleration_steps_per_s2
                    ),
                    max_decel_steps_per_s2=(
                        self._config.spindle_max_deceleration_steps_per_s2
                    ),
                    steps_per_rev=steps_per_rev,
                )
            )
            accel_s = accel_s if accel_s is not None else computed_accel_s
            cruise_s = cruise_s if cruise_s is not None else computed_cruise_s
            decel_s = decel_s if decel_s is not None else computed_decel_s

        expected_turns = 2.0 * bobbin_width_mm * turns_per_mm
        profile = SpindleKinematics(
            target_rpm=target_rpm,
            start_rpm=0.0,
            accel_s=accel_s,
            cruise_s=cruise_s,
            decel_s=decel_s,
        )
        profile_turns = profile.turns_at(accel_s + cruise_s + decel_s)
        if expected_turns > 0.0:
            error_pct = abs(profile_turns - expected_turns) / expected_turns * 100.0
            if error_pct > 5.0:
                logger.warning(
                    "wound_run: profile produces %.1f turns but geometry expects"
                    " %.1f turns (%.1f%% error)",
                    profile_turns,
                    expected_turns,
                    error_pct,
                )

        move = WoundMove(
            name="winding_electronic_gearing",
            kinematics=SpindleKinematics(
                target_rpm=target_rpm,
                start_rpm=0.0,
                accel_s=accel_s,
                cruise_s=cruise_s,
                decel_s=decel_s,
            ),
            pattern=WindingPattern(
                bobbin_width_mm=bobbin_width_mm,
                turns_per_mm=turns_per_mm,
            ),
            scatter=ScatterEngine(
                amplitude_mm=scatter_amplitude_mm,
                freq1=scatter_freq1,
                freq2=scatter_freq2,
                damping_margin_mm=scatter_damping_margin_mm,
            ),
            spindle_cfg=SyncAxisConfig(
                axis_index=spindle_axis_id,
                steps_per_unit=(
                    self._config.spindle_steps_per_revolution
                    * self._config.spindle_microstepping
                ),
                reverse_direction=spindle_reverse,
            ),
            traverse_cfg=SyncAxisConfig(
                axis_index=traverse_axis_id,
                steps_per_unit=self._config.lateral_steps_per_mm,
                reverse_direction=traverse_reverse,
            ),
        )
        self._move_queue.enqueue(move)

    def run_axis(
        self,
        duration_s: float,
        targets: list[dict[str, Any]],
    ) -> None:
        """Queue one or two axes for a trapezoidal ramp move."""
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError("run_axis only allowed when engine is IDLE")
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if not targets:
            raise ValueError("targets must contain at least one axis")
        if len(targets) > 2:
            raise ValueError("run_axis supports at most two axes")

        axis_ids: set[int] = set()
        axis_configs: list[AxisMotionConfig] = []

        logger.info("Processing run_axis target: %s", targets)

        for target in targets:
            if not isinstance(target, dict):
                raise TypeError("each target must be a dict")
            if "axis_id" not in target or "rpm" not in target:
                raise ValueError("each target must contain axis_id and rpm")

            axis_id = int(target["axis_id"])
            rpm = float(target["rpm"])
            reverse = _coerce_bool_param(
                target.get("reverse", False),
                field_name="reverse",
            )
            logger.info("Received run_axis target: axis_id=%d, rpm=%.1f, reverse=%s", axis_id, rpm, reverse)
            if axis_id in axis_ids:
                raise ValueError(f"duplicate axis_id {axis_id}")
            axis_ids.add(axis_id)

            if axis_id == self._config.spindle_axis_id:
                max_rpm = float(self._config.spindle_max_speed_rpm)
                steps_per_rev = (
                    self._config.spindle_steps_per_revolution
                    * self._config.spindle_microstepping
                )
                max_accel = self._config.spindle_max_acceleration_steps_per_s2
                max_decel = self._config.spindle_max_deceleration_steps_per_s2
                invert_direction = bool(self._config.spindle_invert_direction)
            elif axis_id == self._config.lateral_axis_id:
                max_rpm = float(self._config.lateral_max_rpm)
                steps_per_rev = (
                    self._config.lateral_steps_per_revolution
                    * self._config.lateral_microstepping
                )
                max_accel = self._config.lateral_max_acceleration_steps_per_s2
                max_decel = self._config.lateral_max_deceleration_steps_per_s2
                invert_direction = bool(self._config.lateral_invert_direction)
            else:
                raise ValueError(f"Unsupported axis_id {axis_id}")

            if rpm <= 0.0:
                raise ValueError("rpm must be positive")

            target_rpm = min(rpm, max_rpm)
            effective_reverse = invert_direction ^ reverse
            accel_s, cruise_s, decel_s = compute_ramp_times(
                target_rpm=target_rpm,
                duration_s=duration_s,
                max_accel_steps_per_s2=max_accel,
                max_decel_steps_per_s2=max_decel,
                steps_per_rev=steps_per_rev,
            )

            ramp = RampConfig(
                axis_id=axis_id,
                steps_per_rev=steps_per_rev,
                target_rpm=target_rpm,
                accel_s=accel_s,
                cruise_s=cruise_s,
                decel_s=decel_s,
                reverse_direction=effective_reverse,
            )

            if (
                axis_id == self._config.lateral_axis_id
                and axis_id in self._state.axis_states
            ):
                self._lateral.require_homed()
                self._lateral.ensure_delta_allowed(
                    self._lateral.ramp_delta_steps(ramp)
                )

            axis_configs.append(
                AxisMotionConfig(
                    axis_id=axis_id,
                    ramp=ramp,
                )
            )

        move = RampMove(
            name="run_axis",
            config=RampMoveConfig(axis_configs=axis_configs),
        )
        self._move_queue.enqueue(move)

    # ── Private helpers ─────────────────────────────────────────────────────

    def _compute_duration_and_ramp_times(
        self,
        *,
        target_rpm: float,
        total_turns: float,
        max_accel_steps_per_s2: float,
        max_decel_steps_per_s2: float,
        steps_per_rev: int,
    ) -> tuple[float, float, float, float]:
        """Compute corrected duration and final ramp times with two-pass estimation."""
        target_rps = target_rpm / 60.0
        if target_rps <= 0.0:
            base_duration_s = 0.05
        else:
            base_duration_s = max(total_turns / target_rps, 0.05)

        accel_s_p1, _cruise_s_p1, decel_s_p1 = compute_ramp_times(
            target_rpm=target_rpm,
            duration_s=base_duration_s,
            max_accel_steps_per_s2=max_accel_steps_per_s2,
            max_decel_steps_per_s2=max_decel_steps_per_s2,
            steps_per_rev=steps_per_rev,
        )

        duration_s = adjust_duration_for_ramp_deficit(
            total_turns=total_turns,
            target_rps=target_rps,
            accel_s=accel_s_p1,
            decel_s=decel_s_p1,
        )
        accel_s, cruise_s, decel_s = compute_ramp_times(
            target_rpm=target_rpm,
            duration_s=duration_s,
            max_accel_steps_per_s2=max_accel_steps_per_s2,
            max_decel_steps_per_s2=max_decel_steps_per_s2,
            steps_per_rev=steps_per_rev,
        )
        return duration_s, accel_s, cruise_s, decel_s
