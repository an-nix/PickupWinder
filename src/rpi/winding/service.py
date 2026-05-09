from __future__ import annotations

import logging
import threading
from typing import Any, Literal, overload

from core.config import AppConfiguration
from core.coordinator import MotionStopPlan
from core.events import EventBus, EventKind
from core.lateral import LateralAxisController
from core.shared_state import EngineState, SharedState
from motion.move_builders import build_jog_move
from motion.move_queue import MoveQueue
from motion.spindle_kinematics import SpindleKinematics
from winding.adaptive import (
    AdaptiveWindingMove,
    AdaptiveWindingRuntime,
    AdaptiveWindingSessionConfig,
    WindingWindow,
    plan_next_chunk,
)
from winding.synchronized_segment_generator import SyncAxisConfig


logger = logging.getLogger(__name__)
_EPSILON = 1e-6


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _spindle_rate_rps2(max_steps_per_s2: float, steps_per_rev: int) -> float:
    if steps_per_rev <= 0:
        raise ValueError("steps_per_rev must be positive")
    return float(max_steps_per_s2) / float(steps_per_rev)


def _build_spindle_profile(
    *,
    start_rpm: float,
    end_rpm: float,
    duration_s: float,
) -> SpindleKinematics:
    if end_rpm > start_rpm + _EPSILON:
        return SpindleKinematics(
            start_rpm=start_rpm,
            target_rpm=end_rpm,
            accel_s=duration_s,
            cruise_s=0.0,
            decel_s=0.0,
        )
    if end_rpm + _EPSILON < start_rpm:
        return SpindleKinematics(
            start_rpm=end_rpm,
            target_rpm=start_rpm,
            accel_s=0.0,
            cruise_s=0.0,
            decel_s=duration_s,
        )
    return SpindleKinematics(
        start_rpm=start_rpm,
        target_rpm=start_rpm,
        accel_s=0.0,
        cruise_s=duration_s,
        decel_s=0.0,
    )


def _solve_scatter_reference(
    *,
    position_mm: float,
    absolute_turns: float,
    window: WindingWindow,
    runtime: AdaptiveWindingRuntime,
) -> float:
    scatter = runtime.build_scatter_engine()
    reference = 0.0
    for _ in range(8):
        base_position_mm = position_mm - reference
        relative_base_mm = _clamp(
            base_position_mm - window.low_mm,
            0.0,
            window.width_mm,
        )
        next_reference = scatter.get_offset(
            absolute_turns,
            relative_base_mm,
            window.width_mm,
        )
        if abs(next_reference - reference) <= 1e-6:
            return next_reference
        reference = next_reference
    return reference


class AdaptiveWindingService:
    """Run a live-controllable winding session on top of the shared move queue."""

    def __init__(
        self,
        *,
        shared_state: SharedState,
        move_queue: MoveQueue,
        lateral_controller: LateralAxisController,
        event_bus: EventBus,
        config: AppConfiguration,
    ) -> None:
        self._state = shared_state
        self._move_queue = move_queue
        self._lateral = lateral_controller
        self._events = event_bus
        self._config = config
        self._lock = threading.RLock()
        self._wake_event = threading.Event()
        self._worker: threading.Thread | None = None
        self._active_session: AdaptiveWindingRuntime | None = None
        self._last_worker_error: str | None = None

    def start_session(self, session: AdaptiveWindingSessionConfig) -> dict[str, Any]:
        session.validate()
        self._validate_window(session.window)

        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("An adaptive winding session is already running")
            if self._state.engine_state != EngineState.IDLE:
                raise RuntimeError(
                    "Adaptive winding can only start when controller state is IDLE"
                )

            spindle_steps_per_rev = (
                self._config.spindle_steps_per_revolution
                * self._config.spindle_microstepping
            )
            runtime = AdaptiveWindingRuntime(
                session,
                spindle_steps_per_rev=spindle_steps_per_rev,
                lateral_steps_per_mm=self._config.lateral_steps_per_mm,
            )
            self._active_session = runtime
            self._state.set_program(None)
            self._state.set_winding_session(runtime.snapshot())
            self._wake_event.clear()

            self._worker = threading.Thread(
                target=self._run_session,
                args=(runtime,),
                daemon=True,
                name="adaptive_winding",
            )
            self._last_worker_error = None
            self._worker.start()
            return runtime.snapshot()

    def update_session(
        self,
        *,
        target_rpm: float | None = None,
        window_low_mm: float | None = None,
        window_high_mm: float | None = None,
        wire_diameter_mm: float | None = None,
        wire_awg: int | None = None,
        turns_per_mm: float | None = None,
        pitch_factor: float | None = None,
    ) -> dict[str, Any]:
        runtime = self._require_session()
        current_window = runtime.current_window
        new_window = WindingWindow(
            current_window.low_mm if window_low_mm is None else float(window_low_mm),
            current_window.high_mm if window_high_mm is None else float(window_high_mm),
        )
        self._validate_window(new_window)
        runtime.update_controls(
            target_rpm=target_rpm,
            window_low_mm=window_low_mm,
            window_high_mm=window_high_mm,
            wire_diameter_mm=wire_diameter_mm,
            wire_awg=wire_awg,
            turns_per_mm=turns_per_mm,
            pitch_factor=pitch_factor,
        )
        self._publish_status(runtime)
        self._wake_event.set()
        return runtime.snapshot()

    def pause_session(self, *, pause_at_turn: float | None = None) -> dict[str, Any]:
        runtime = self._require_session()
        runtime.request_pause(pause_at_turn=pause_at_turn)
        self._publish_status(runtime)
        self._wake_event.set()
        return runtime.snapshot()

    def resume_session(self) -> dict[str, Any]:
        runtime = self._require_session()
        with self._lock:
            worker_alive = self._worker is not None and self._worker.is_alive()
        if not worker_alive:
            raise RuntimeError(
                "Adaptive winding worker is no longer running; "
                "the session may have failed. Check session_status() for details."
            )
        if runtime.snapshot()["target_rpm"] <= 0.0:
            raise RuntimeError("Set a positive target_rpm before resuming the winding session")
        runtime.resume()
        self._publish_status(runtime)
        self._wake_event.set()
        return runtime.snapshot()

    def request_stop(
        self,
        stop_plan: MotionStopPlan | None = None,
        *,
        clear_queue: bool = True,
    ) -> dict[str, Any]:
        runtime = self._require_session(allow_terminal=True)
        if runtime is None:
            return {"active": False, "session": None}
        if runtime.state in {"completed", "stopped", "fault"}:
            return runtime.snapshot()
        runtime.request_stop()
        snapshot = runtime.snapshot()
        self._state.transition_to_stopping(snapshot)
        if clear_queue:
            self._move_queue.clear(
                stop_plan=stop_plan or MotionStopPlan.stop(
                    self._state.axis_states,
                    reason="adaptive stop requested",
                )
            )
        self._events.publish(EventKind.STATUS_UPDATE, winding_session=snapshot)
        self._wake_event.set()
        return snapshot

    def stop(self, timeout_s: float = 5.0) -> None:
        runtime = self._require_session(allow_terminal=True)
        worker = self._worker
        if runtime is not None and runtime.state not in {"completed", "stopped", "fault"}:
            runtime.request_stop()
            self._move_queue.clear(
                stop_plan=MotionStopPlan.stop(
                    self._state.axis_states,
                    reason="adaptive worker shutdown",
                )
            )
            self._wake_event.set()
        if worker is not None and worker.is_alive():
            worker.join(timeout=timeout_s)
        if worker is not None and worker.is_alive():
            self._last_worker_error = (
                f"adaptive winding worker did not stop within {timeout_s:.1f}s"
            )
            raise RuntimeError(self._last_worker_error)

    def health_status(self) -> dict[str, Any]:
        return {
            "name": "adaptive_winding",
            "thread_alive": self._worker is not None and self._worker.is_alive(),
            "thread_faulted": self._last_worker_error is not None,
            "last_error": self._last_worker_error,
            "session_active": self._active_session is not None,
        }

    def session_status(self) -> dict[str, Any]:
        with self._lock:
            runtime = self._active_session
        if runtime is None:
            return {"active": False, "session": None}
        snapshot = runtime.snapshot()
        return {
            "active": snapshot["state"] not in {"completed", "stopped", "fault"},
            "session": snapshot,
        }

    @overload
    def _require_session(
        self,
        *,
        allow_terminal: Literal[True],
    ) -> AdaptiveWindingRuntime | None: ...

    @overload
    def _require_session(
        self,
        *,
        allow_terminal: Literal[False] = False,
    ) -> AdaptiveWindingRuntime: ...

    def _require_session(self, *, allow_terminal: bool = False) -> AdaptiveWindingRuntime | None:
        with self._lock:
            runtime = self._active_session
        if runtime is None:
            if allow_terminal:
                return None
            raise RuntimeError("No adaptive winding session is active")
        if not allow_terminal and runtime.state in {"completed", "stopped", "fault"}:
            raise RuntimeError("Adaptive winding session is no longer active")
        return runtime

    def _publish_status(self, runtime: AdaptiveWindingRuntime) -> None:
        snapshot = runtime.snapshot()
        self._state.set_winding_session(snapshot)
        self._events.publish(EventKind.STATUS_UPDATE, winding_session=snapshot)

    def _validate_window(self, window: WindingWindow) -> None:
        minimum = self._config.lateral_soft_limit_min_mm
        maximum = self._config.lateral_soft_limit_max_mm
        if minimum is not None and window.low_mm < minimum:
            raise ValueError(
                f"window_low_mm={window.low_mm:.3f} is below configured minimum {minimum:.3f}"
            )
        if maximum is not None and window.high_mm > maximum:
            raise ValueError(
                f"window_high_mm={window.high_mm:.3f} is above configured maximum {maximum:.3f}"
            )

    def _run_session(self, runtime: AdaptiveWindingRuntime) -> None:
        spindle_accel_rps2 = _spindle_rate_rps2(
            self._config.spindle_max_acceleration_steps_per_s2,
            self._config.spindle_steps_per_revolution * self._config.spindle_microstepping,
        )
        spindle_decel_rps2 = _spindle_rate_rps2(
            self._config.spindle_max_deceleration_steps_per_s2,
            self._config.spindle_steps_per_revolution * self._config.spindle_microstepping,
        )

        try:
            session_config = runtime.session_config()
            if session_config.target_rpm <= 0.0:
                raise ValueError("Adaptive winding session must start with target_rpm > 0")

            if runtime.current_window.low_mm < -_EPSILON:
                raise ValueError("window_low_mm must be >= 0 relative to home")

            if runtime.state == "queued":
                self._state.set_engine_state(
                    EngineState.HOMING
                    if session_config.home_before_start
                    else EngineState.RUNNING
                )
            self._publish_status(runtime)

            config = session_config
            if config.home_before_start:
                success, reason = self._lateral.home(
                    axis_id=self._config.lateral_axis_id,
                    approach_rpm=config.home_approach_rpm,
                    search_rpm=config.home_search_rpm,
                    backoff_steps=config.home_backoff_steps,
                )
                if not success:
                    raise RuntimeError(reason or "lateral homing failed")

            # Post-home start position and adaptive window low bound are distinct
            # concepts. `home()` parks at soft_limit_min + axis_offset, then the
            # session moves to the active winding window if needed.
            self._move_lateral_to(runtime, runtime.current_window.low_mm)
            runtime.mark_running()
            self._state.set_engine_state(EngineState.RUNNING)
            self._publish_status(runtime)

            while True:
                if runtime.stop_requested and runtime.current_rpm <= _EPSILON and self._move_queue.current_move is None:
                    runtime.mark_stopped("stop requested")
                    break

                reposition_target = runtime.consume_pending_reposition()
                if reposition_target is not None and runtime.current_rpm <= _EPSILON:
                    self._move_lateral_to(runtime, reposition_target)
                    self._publish_status(runtime)
                    continue

                if runtime.pause_requested and runtime.current_rpm <= _EPSILON:
                    runtime.mark_paused()
                    self._state.transition_to_paused(runtime.snapshot())
                    self._events.publish(EventKind.STATUS_UPDATE, winding_session=runtime.snapshot())
                    self._wake_event.wait(timeout=0.1)
                    self._wake_event.clear()
                    if runtime.stop_requested:
                        continue
                    if not runtime.pause_requested:
                        runtime.mark_running()
                        self._state.transition_to_running(runtime.snapshot())
                        self._events.publish(EventKind.STATUS_UPDATE, winding_session=runtime.snapshot())
                    continue

                plan = plan_next_chunk(
                    runtime.planning_snapshot(),
                    spindle_accel_rps2=spindle_accel_rps2,
                    spindle_decel_rps2=spindle_decel_rps2,
                )
                if plan is None:
                    if runtime.stop_requested:
                        runtime.mark_stopped("stop requested")
                    elif runtime.pause_requested:
                        runtime.mark_paused()
                    else:
                        runtime.mark_completed()
                    break

                self._lateral.require_homed(self._config.lateral_axis_id)
                move = self._build_chunk_move(runtime, plan)
                self._move_queue.enqueue(move)
                self._move_queue.wait_until_idle(
                    timeout_s=max(move.kinematics.total_duration * 4.0, 10.0)
                )

                if move.state.name == "FAILED":
                    raise RuntimeError(move.error or "adaptive winding move failed")
                if move.state.name == "ABORTED":
                    if runtime.stop_requested:
                        runtime.mark_stopped(move.error or "stop requested")
                        break
                    raise RuntimeError(move.error or "adaptive winding move aborted")

                runtime.apply_completed_move(plan, move)
                self._state.transition_to_running(runtime.snapshot())
                self._events.publish(EventKind.STATUS_UPDATE, winding_session=runtime.snapshot())

            snapshot = runtime.snapshot()
            if runtime.state == "completed":
                self._state.transition_to_idle_session(snapshot)
            elif runtime.state == "paused":
                self._state.transition_to_paused(snapshot)
            elif runtime.state == "stopped":
                self._state.transition_to_idle_session(snapshot)
            self._events.publish(EventKind.STATUS_UPDATE, winding_session=snapshot)
        except Exception as exc:
            self._last_worker_error = str(exc)
            logger.exception("adaptive winding session failed")
            runtime.mark_fault(str(exc))
            self._state.set_fault(str(exc))
            self._events.publish(
                EventKind.WORKER_FAILED,
                worker="adaptive_winding",
                error=str(exc),
            )
            self._publish_status(runtime)
        finally:
            with self._lock:
                self._worker = None

    def _build_chunk_move(
        self,
        runtime: AdaptiveWindingRuntime,
        plan,
    ) -> AdaptiveWindingMove:
        kinematics = _build_spindle_profile(
            start_rpm=plan.start_rpm,
            end_rpm=plan.end_rpm,
            duration_s=plan.duration_s,
        )
        spindle_cfg = SyncAxisConfig(
            axis_index=self._config.spindle_axis_id,
            steps_per_unit=(
                self._config.spindle_steps_per_revolution
                * self._config.spindle_microstepping
            ),
            reverse_direction=self._config.spindle_invert_direction,
        )
        traverse_cfg = SyncAxisConfig(
            axis_index=self._config.lateral_axis_id,
            steps_per_unit=self._config.lateral_steps_per_mm,
            reverse_direction=self._config.lateral_invert_direction,
        )
        return AdaptiveWindingMove(
            name=f"adaptive_chunk_{runtime.current_turns:.3f}",
            kinematics=kinematics,
            scatter=runtime.build_scatter_engine(),
            window=runtime.current_window,
            turns_per_mm=runtime.turns_per_mm,
            direction_sign=runtime.direction_sign,
            start_turns_abs=runtime.current_turns,
            start_guide_mm=runtime.current_guide_mm,
            start_scatter_reference_mm=runtime.current_scatter_reference_mm,
            spindle_cfg=spindle_cfg,
            traverse_cfg=traverse_cfg,
        )

    def _move_lateral_to(self, runtime: AdaptiveWindingRuntime, target_mm: float) -> None:
        axis_state = self._lateral.require_homed()
        current_steps = axis_state.position_steps
        if current_steps is None:
            raise RuntimeError("Lateral position is unknown; home the axis first")

        target_steps = self._lateral.mm_to_steps(target_mm)
        delta_steps = target_steps - current_steps
        self._lateral.ensure_delta_allowed(delta_steps)
        if delta_steps == 0:
            scatter_reference = _solve_scatter_reference(
                position_mm=target_mm,
                absolute_turns=runtime.current_turns,
                window=runtime.current_window,
                runtime=runtime,
            )
            runtime.set_repositioned_guide(
                target_mm,
                scatter_reference_mm=scatter_reference,
            )
            return

        move = build_jog_move(
            name="adaptive_window_reposition",
            axis_id=self._config.lateral_axis_id,
            steps=abs(delta_steps),
            steps_per_rev=(
                self._config.lateral_steps_per_revolution
                * self._config.lateral_microstepping
            ),
            rpm=min(max(runtime.snapshot()["target_rpm"], 60.0), float(self._config.lateral_max_rpm)),
            reverse=(delta_steps < 0),
        )
        self._move_queue.enqueue(move)
        self._move_queue.wait_until_idle(timeout_s=max(move.axis_configs[0].ramp.total_duration * 4.0, 10.0))
        if move.state.name != "COMPLETED":
            raise RuntimeError(move.error or "failed to reposition lateral axis")

        scatter_reference = _solve_scatter_reference(
            position_mm=target_mm,
            absolute_turns=runtime.current_turns,
            window=runtime.current_window,
            runtime=runtime,
        )
        runtime.set_repositioned_guide(
            target_mm,
            scatter_reference_mm=scatter_reference,
        )

