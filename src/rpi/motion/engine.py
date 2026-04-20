from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

from motion.axis_state import AxisState
from motion.move import HomingMove, JogMove, RampMove, RampMoveConfig, WoundMove
from motion.move_queue import MoveQueue
from motion import (
    AxisMotionConfig,
    RampConfig,
    SpindleKinematics,
    WindingPattern,
    ScatterEngine,
    SyncAxisConfig,
)
from motion.ramp_config import compute_ramp_times
from transport.spi_transport import Esp32SpiTransport
from core.config import AppConfiguration
from core.events import EventBus, EventKind
from winding.program import WindingProgram
from core.shared_state import EngineState, SharedState


logger = logging.getLogger(__name__)

_DEFAULT_HOME_APPROACH_RPM = 100.0
_DEFAULT_HOME_SEARCH_RPM = 20.0
_DEFAULT_HOME_BACKOFF_STEPS = 3200


class WindingEngine:
    """
    Executes winding programs in a single daemon thread.

    The engine owns the MoveQueue and is the only producer of motion.
    The JsonRpcServer sends commands by calling public methods on the
    engine — these methods are thread-safe and return immediately.
    The engine thread reads a _pending_program queue and executes it.

    State machine:
      IDLE -> HOMING -> RUNNING -> IDLE       (normal completion)
      IDLE -> HOMING -> FAULT               (homing failed)
      RUNNING -> STOPPING -> IDLE           (stop requested)
      RUNNING -> FAULT                     (endstop / error)
    """

    def __init__(
        self,
        transport: Esp32SpiTransport,
        shared_state: SharedState,
        event_bus: EventBus,
        config: AppConfiguration | None = None,
    ) -> None:
        self._transport = transport
        self._state = shared_state
        self._events = event_bus
        self._config = config or AppConfiguration()

        self._move_queue = MoveQueue(
            transport=transport,
            axis_states=shared_state.axis_states,
            poll_interval_s=0.005,
            print_every=8,
        )

        self._stop_event = threading.Event()
        self._program_event = threading.Event()
        self._pending_program: WindingProgram | None = None
        self._program_lock = threading.Lock()
        self._thread: threading.Thread | None = None

    @property
    def config(self) -> AppConfiguration:
        return self._config

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the engine thread and the MoveQueue."""
        self._move_queue.start()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="winding_engine",
        )
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        """Stop the engine thread and MoveQueue cleanly."""
        self._stop_event.set()
        self._program_event.set()
        self._move_queue.stop(timeout_s=timeout_s)
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)

    # ── Command API (called from JsonRpcServer thread) ─────────────────────

    def submit_program(self, program: WindingProgram) -> None:
        """
        Queue a winding program for execution.
        Raises RuntimeError if the engine is not IDLE.
        """
        with self._program_lock:
            if self._state.engine_state not in (
                EngineState.IDLE, EngineState.FAULT
            ):
                raise RuntimeError(
                    f"Cannot submit program: engine is "
                    f"{self._state.engine_state.name}"
                )
            program.validate()
            self._pending_program = program
            self._stop_event.clear()
            self._program_event.set()

    def request_stop(self) -> None:
        """Request stop. Current move is aborted, queue is cleared."""
        self._stop_event.set()
        self._move_queue.clear()
        self._state.set_engine_state(EngineState.STOPPING)

    def flush_until(self, sequence: int) -> dict[str, Any]:
        """Send a flush request to the firmware and return the resulting status."""
        status = self._transport.flush_until(sequence)
        return {
            "uptime_ms": status.uptime_ms,
            "queue_free_slots": list(status.queue_free_slots),
            "ring_free_slots": list(status.ring_free_slots),
            "underrun_count": list(status.underrun_count),
            "last_rx_sequence": status.last_rx_sequence,
            "last_rx_type": status.last_rx_type,
            "last_result": status.last_result,
            "protocol_version": status.protocol_version,
            "enabled_mask": status.enabled_mask,
            "running_mask": status.running_mask,
            "lateral_endstop_state": status.lateral_endstop_state,
            "endstop_armed_mask": status.endstop_armed_mask,
            "last_executed_sequence": status.last_executed_sequence,
            "multi_axis_queue_free": status.multi_axis_queue_free,
            "planner_queue_free": status.planner_queue_free,
            "last_planned_sequence": status.last_planned_sequence,
            "segments_dropped": status.segments_dropped,
        }

    def clear_fault(self) -> None:
        """Clear fault state so a new program can be submitted."""
        self._state.clear_fault()

    def arm_endstop(self, axis_id: int) -> None:
        """Arm the specified endstop through the transport."""
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=True)
        self._transport.wait_for_request_result(sequence)

    def disarm_endstop(self, axis_id: int) -> None:
        """Disarm the specified endstop through the transport."""
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=False)
        self._transport.wait_for_request_result(sequence)

    def home_lateral(
        self,
        approach_rpm: float = _DEFAULT_HOME_APPROACH_RPM,
        search_rpm: float = _DEFAULT_HOME_SEARCH_RPM,
        backoff_steps: int = _DEFAULT_HOME_BACKOFF_STEPS,
    ) -> dict[str, Any]:
        """Synchronously home the lateral axis and keep its driver enabled."""
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError(
                "home_lateral only allowed when engine is IDLE; if a FAULT occurred, acknowledge it with winding.clear_fault first"
            )

        self._state.set_engine_state(EngineState.HOMING)
        success, reason = self._home_lateral_axis(
            axis_id=self._config.lateral_axis_id,
            approach_rpm=approach_rpm,
            search_rpm=search_rpm,
            backoff_steps=backoff_steps,
        )
        if not success:
            if reason:
                raise RuntimeError(f"Lateral homing failed: {reason}")
            raise RuntimeError("Lateral homing failed")

        self._state.set_engine_state(EngineState.IDLE)
        return self._require_axis_state(self._config.lateral_axis_id).snapshot()

    def move_lateral_to_mm(
        self,
        position_mm: float,
        rpm: float,
    ) -> dict[str, Any]:
        """Queue a lateral move to an absolute position in mm from home zero."""
        if self._state.engine_state != EngineState.IDLE:
            if self._state.engine_state == EngineState.FAULT:
                raise RuntimeError(
                    "move_lateral_to_mm not allowed while controller is in FAULT; acknowledge it with winding.clear_fault before retrying"
                )
            raise RuntimeError("move_lateral_to_mm only allowed when engine is IDLE")

        axis_state = self._require_lateral_homed()
        current_steps = axis_state.position_steps
        if current_steps is None:
            raise RuntimeError("Lateral axis position is unknown; home the axis first")

        target_steps = self._mm_to_lateral_steps(position_mm)
        delta_steps = target_steps - current_steps
        self._ensure_lateral_delta_allowed(delta_steps)

        if delta_steps == 0:
            return {
                "status": "already_at_position",
                "position_mm": position_mm,
            }

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
        """
        Execute a synchronized winding operation (Electronic Gearing).
        Only allowed when engine is IDLE.
        """
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError("Winding run only allowed when engine is IDLE")
        if (
            traverse_axis_id == self._config.lateral_axis_id
            and traverse_axis_id in self._state.axis_states
        ):
            self._require_lateral_homed()

        if accel_s is None or cruise_s is None or decel_s is None:
            total_turns = 2.0 * bobbin_width_mm * turns_per_mm
            steps_per_rev = (
                self._config.spindle_steps_per_revolution
                * self._config.spindle_microstepping
            )
            _, computed_accel_s, computed_cruise_s, computed_decel_s = self._compute_duration_and_ramp_times(
                target_rpm=target_rpm,
                total_turns=total_turns,
                max_accel_steps_per_s2=self._config.spindle_max_acceleration_steps_per_s2,
                max_decel_steps_per_s2=self._config.spindle_max_deceleration_steps_per_s2,
                steps_per_rev=steps_per_rev,
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
                    "wound_run: profile produces %.1f turns but geometry expects %.1f turns (%.1f%% error)",
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
                steps_per_unit=(
                    self._config.lateral_steps_per_revolution
                    * self._config.lateral_microstepping
                ),
                reverse_direction=traverse_reverse,
            ),
        )
        self._move_queue.enqueue(move)

    def run_axis(
        self,
        duration_s: float,
        targets: list[dict[str, Any]],
    ) -> None:
        """
        Queue one or two axes for a trapezoidal ramp move.
        The acceleration and deceleration times are calculated from the
        application configuration limits.
        """
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

        for target in targets:
            if not isinstance(target, dict):
                raise TypeError("each target must be a dict")
            if "axis_id" not in target or "rpm" not in target:
                raise ValueError("each target must contain axis_id and rpm")

            axis_id = int(target["axis_id"])
            rpm = float(target["rpm"])
            reverse = bool(target.get("reverse", False))

            if axis_id in axis_ids:
                raise ValueError(f"duplicate axis_id {axis_id}")
            axis_ids.add(axis_id)

            if axis_id == self._config.spindle_axis_id:
                max_rpm = float(self._config.spindle_max_speed_rpm)
                steps_per_rev = self._config.spindle_steps_per_revolution * self._config.spindle_microstepping
                max_accel = self._config.spindle_max_acceleration_steps_per_s2
                max_decel = self._config.spindle_max_deceleration_steps_per_s2
            elif axis_id == self._config.lateral_axis_id:
                max_rpm = float(self._config.lateral_max_rpm)
                steps_per_rev = self._config.lateral_steps_per_revolution * self._config.lateral_microstepping
                max_accel = self._config.lateral_max_acceleration_steps_per_s2
                max_decel = self._config.lateral_max_deceleration_steps_per_s2
            else:
                raise ValueError(f"Unsupported axis_id {axis_id}")

            if rpm <= 0.0:
                raise ValueError("rpm must be positive")

            target_rpm = min(rpm, max_rpm)
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
                reverse_direction=reverse,
            )

            if (
                axis_id == self._config.lateral_axis_id
                and axis_id in self._state.axis_states
            ):
                self._require_lateral_homed()
                self._ensure_lateral_delta_allowed(self._ramp_delta_steps(ramp))

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

    def jog(
        self,
        axis_id: int,
        steps: int,
        rpm: float,
        reverse: bool = False,
    ) -> None:
        """
        Execute a jog move immediately.
        Only allowed when engine is IDLE.
        """
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
                self._require_lateral_homed()
                delta_steps = -steps if reverse else steps
                self._ensure_lateral_delta_allowed(delta_steps)
        else:
            raise ValueError(f"jog: unsupported axis_id {axis_id}")

        move = JogMove(
            name=f"jog_{axis_id}",
            axis_id=axis_id,
            steps_per_rev=steps_per_rev,
            steps=steps,
            rpm=rpm,
            reverse_direction=reverse,
        )
        self._move_queue.enqueue(move)

    # ── Engine thread ──────────────────────────────────────────────────────

    def _run(self) -> None:
        """Main engine loop — waits for programs and executes them."""
        while not self._stop_event.is_set():
            self._program_event.wait(timeout=1.0)
            self._program_event.clear()

            with self._program_lock:
                program = self._pending_program
                self._pending_program = None

            if program is None:
                continue
            if self._stop_event.is_set():
                break

            self._execute_program(program)

    def _execute_program(self, program: WindingProgram) -> None:
        """
        Execute one complete winding program.

        Sequence:
          1. Optional homing on lateral axis
          2. For each layer:
             a. Start layer in shared state
             b. Enqueue synchronised RampMove (spindle + lateral)
             c. Wait for MoveQueue to drain
             d. Complete layer in shared state
          3. Mark program complete or aborted
        """
        self._state.set_program(program)
        self._state.set_engine_state(
            EngineState.HOMING if program.home_before_start else EngineState.RUNNING
        )
        self._events.publish(EventKind.PROGRAM_STARTED, program=program.snapshot())

        # ── Phase 1: homing ────────────────────────────────────────────────
        if program.home_before_start:
            success = self._home_lateral_axis(
                axis_id=program.lateral_axis_id,
                approach_rpm=program.home_approach_rpm,
                search_rpm=program.home_search_rpm,
                backoff_steps=program.home_backoff_steps,
            )
            if not success:
                return
            self._state.set_engine_state(EngineState.RUNNING)
        else:
            self._require_lateral_homed(program.lateral_axis_id)

        # ── Phase 2: winding layers ────────────────────────────────────────
        for layer_index in range(program.num_layers):
            if self._stop_event.is_set():
                self._state.set_engine_state(EngineState.IDLE)
                self._events.publish(EventKind.PROGRAM_ABORTED, layer=layer_index)
                return

            direction = "forward" if layer_index % 2 == 0 else "reverse"
            self._state.start_layer(layer_index, program.num_layers, direction)
            self._events.publish(
                EventKind.LAYER_STARTED, layer=layer_index, direction=direction
            )

            ok = self._run_layer(program, layer_index, direction)
            if not ok:
                return

            self._state.complete_layer()
            self._events.publish(EventKind.LAYER_COMPLETED, layer=layer_index)

        # ── Phase 3: completion ────────────────────────────────────────────
        self._state.set_engine_state(EngineState.IDLE)
        self._state.set_program(None)
        self._events.publish(EventKind.PROGRAM_COMPLETED, program=program.snapshot())

    def _home_lateral_axis(
        self,
        *,
        axis_id: int,
        approach_rpm: float,
        search_rpm: float,
        backoff_steps: int,
    ) -> tuple[bool, str | None]:
        """
        Home the lateral axis. Returns (success, error_message).
        Sets FAULT state and publishes event on failure.
        """
        self._events.publish(EventKind.HOMING_STARTED, axis_id=axis_id)
        steps_per_rev = (
            self._config.lateral_steps_per_revolution
            * self._config.lateral_microstepping
        )
        move = HomingMove(
            name="home_lateral",
            axis_id=axis_id,
            steps_per_rev=steps_per_rev,
            approach_rpm=approach_rpm,
            search_rpm=search_rpm,
            backoff_steps=backoff_steps,
            max_approach_steps=int(steps_per_rev * 20),
            reverse_direction=self._config.lateral_invert_direction,
        )
        self._move_queue.enqueue(move)
        self._wait_for_move_queue()

        if move.state.name == "COMPLETED":
            self._events.publish(EventKind.HOMING_COMPLETED, axis_id=axis_id)
            return True, None
        else:
            msg = f"Homing failed: {move.error}"
            self._state.set_fault(msg)
            self._events.publish(
                EventKind.HOMING_FAILED,
                axis_id=axis_id,
                error=msg,
            )
            return False, move.error

    def _run_layer(
        self,
        program: WindingProgram,
        layer_index: int,
        direction: str,
    ) -> bool:
        """
        Execute one winding layer: spindle + lateral move in sync.
        Returns True on completion, False on abort or fault.
        """
        reverse_lateral = (direction == "reverse")
        total_turns = 2.0 * program.bobbin_width_mm * program.turns_per_mm
        target_rps = program.spindle_rpm / 60.0
        duration_s = self._adjust_duration_for_ramp_deficit(
            total_turns=total_turns,
            target_rps=target_rps,
            accel_s=program.accel_s,
            decel_s=program.decel_s,
        )
        cruise_s = max(duration_s - program.accel_s - program.decel_s, 0.0)

        move = WoundMove(
            name=f"layer_{layer_index}",
            kinematics=SpindleKinematics(
                target_rpm=program.spindle_rpm,
                start_rpm=0.0,
                accel_s=program.accel_s,
                cruise_s=cruise_s,
                decel_s=program.decel_s,
            ),
            pattern=WindingPattern(
                bobbin_width_mm=program.bobbin_width_mm,
                turns_per_mm=program.turns_per_mm,
            ),
            scatter=ScatterEngine(
                amplitude_mm=program.scatter_amplitude_mm,
                freq1=program.scatter_freq1,
                freq2=program.scatter_freq2,
                damping_margin_mm=program.scatter_damping_margin_mm,
            ),
            spindle_cfg=SyncAxisConfig(
                axis_index=program.spindle_axis_id,
                steps_per_unit=(
                    self._config.spindle_steps_per_revolution
                    * self._config.spindle_microstepping
                ),
            ),
            traverse_cfg=SyncAxisConfig(
                axis_index=program.lateral_axis_id,
                steps_per_unit=program.lateral_steps_per_mm,
                reverse_direction=reverse_lateral,
            ),
        )
        self._move_queue.enqueue(move)
        self._wait_for_move_queue()

        if move.aborted_by_endstop:
            msg = f"Endstop triggered during layer {layer_index}"
            self._state.set_fault(msg)
            self._events.publish(
                EventKind.ENDSTOP_TRIGGERED,
                layer=layer_index,
                axis=program.lateral_axis_id,
            )
            return False

        if move.state.name == "FAILED":
            self._state.set_fault(move.error or "unknown error")
            return False

        if self._stop_event.is_set():
            self._state.set_engine_state(EngineState.IDLE)
            self._events.publish(EventKind.PROGRAM_ABORTED, layer=layer_index)
            return False

        return True

    def status(self) -> dict[str, Any]:
        """Return a combined shared state and move queue status snapshot."""
        self._refresh_lateral_home_state()
        return {
            "shared_state": self._state.snapshot(),
            "move_queue": self._move_queue.status(),
        }

    def _require_axis_state(self, axis_id: int) -> AxisState:
        axis_state = self._state.axis_states.get(axis_id)
        if axis_state is None:
            raise RuntimeError(f"Unknown axis_id {axis_id}")
        return axis_state

    def _refresh_lateral_home_state(self) -> None:
        axis_state = self._state.axis_states.get(self._config.lateral_axis_id)
        if axis_state is None or not axis_state.homed:
            return
        try:
            status = self._transport.get_status()
        except Exception:
            return

        enabled_mask = int(getattr(status, "enabled_mask", 0))
        if (enabled_mask & (1 << self._config.lateral_axis_id)) == 0:
            logger.warning(
                "Lateral axis enable lost; invalidating homing state and position"
            )
            axis_state.invalidate_position()

    def _require_lateral_homed(self, axis_id: int | None = None) -> AxisState:
        lateral_axis_id = self._config.lateral_axis_id if axis_id is None else axis_id
        if lateral_axis_id != self._config.lateral_axis_id:
            return self._require_axis_state(lateral_axis_id)

        self._refresh_lateral_home_state()
        axis_state = self._require_axis_state(lateral_axis_id)
        if not axis_state.homed or axis_state.position_steps is None:
            raise RuntimeError("Lateral axis must be homed before motion")
        return axis_state

    def _ensure_lateral_delta_allowed(self, delta_steps: int) -> None:
        axis_state = self._require_lateral_homed()
        if axis_state.check_move(delta_steps):
            return
        target_steps = (axis_state.position_steps or 0) + delta_steps
        target_mm = self._lateral_steps_to_mm(target_steps)
        raise ValueError(
            f"Lateral target {target_mm:.3f} mm is outside configured soft limits"
        )

    def _mm_to_lateral_steps(self, position_mm: float) -> int:
        return int(round(float(position_mm) * self._config.lateral_steps_per_mm))

    def _lateral_steps_to_mm(self, position_steps: int) -> float:
        return float(position_steps) / float(self._config.lateral_steps_per_mm)

    @staticmethod
    def _ramp_delta_steps(ramp: RampConfig) -> int:
        total_steps = int(round(ramp.steps_at(ramp.total_duration)))
        return -total_steps if ramp.reverse_direction else total_steps

    def _wait_for_move_queue(
        self,
        poll_s: float = 0.05,
        timeout_s: float = 60.0,
    ) -> None:
        """
        Block until the MoveQueue has no pending or running moves,
        until a stop is requested, or until *timeout_s* seconds have
        elapsed.

        On timeout the engine transitions to FAULT so the caller can
        detect the condition via ``move.state``.
        """
        deadline = time.monotonic() + timeout_s
        while (
            not self._stop_event.is_set()
            and (
                self._move_queue.pending_count > 0
                or self._move_queue.current_move is not None
            )
        ):
            if time.monotonic() >= deadline:
                msg = (
                    f"_wait_for_move_queue timed out after {timeout_s:.1f} s — "
                    "firmware may have stopped responding"
                )
                self._state.set_fault(msg)
                break
            time.sleep(poll_s)

    @staticmethod
    def _adjust_duration_for_ramp_deficit(
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

        # Pass 1: estimate accel/decel from a duration that ignores ramp deficits.
        accel_s_p1, _cruise_s_p1, decel_s_p1 = compute_ramp_times(
            target_rpm=target_rpm,
            duration_s=base_duration_s,
            max_accel_steps_per_s2=max_accel_steps_per_s2,
            max_decel_steps_per_s2=max_decel_steps_per_s2,
            steps_per_rev=steps_per_rev,
        )

        # Pass 2: extend duration to compensate turns not produced at cruise speed during ramps.
        duration_s = self._adjust_duration_for_ramp_deficit(
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
