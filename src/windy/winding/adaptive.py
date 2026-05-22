from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any, Iterator

from motion.segment_generator import AxisStepProfile, StepProfileSegmentGenerator
from motion.spindle_kinematics import SpindleKinematics
from transport.messages import MultiAxisSegment
from winding.program import WindingProgram
from winding.scatter_engine import ScatterEngine
from winding.session import SessionParams
from winding.synchronized_segment_generator import SyncAxisConfig
from winding.wound_move import SynchronizedMove

_EPSILON = 1e-6


def awg_to_diameter_mm(awg: int) -> float:
    """Return the nominal copper wire diameter for an AWG gauge."""
    if awg < 0 or awg > 60:
        raise ValueError("awg must be between 0 and 60")
    diameter_inch = 0.005 * (92.0 ** ((36.0 - float(awg)) / 39.0))
    return diameter_inch * 25.4


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _rpm_to_rps(rpm: float) -> float:
    return float(rpm) / 60.0


def _rps_to_rpm(rps: float) -> float:
    return float(rps) * 60.0


def _distance_for_speed_change(start_rps: float, end_rps: float, rate_rps2: float) -> float:
    if rate_rps2 <= 0.0 or abs(end_rps - start_rps) <= _EPSILON:
        return 0.0
    return abs((end_rps * end_rps) - (start_rps * start_rps)) / (2.0 * rate_rps2)


def _duration_for_speed_change(start_rps: float, end_rps: float, rate_rps2: float) -> float:
    if rate_rps2 <= 0.0 or abs(end_rps - start_rps) <= _EPSILON:
        return 0.0
    return abs(end_rps - start_rps) / rate_rps2


def _peak_rps_for_stop(
    start_rps: float,
    distance_turns: float,
    accel_rps2: float,
    decel_rps2: float,
) -> float:
    if distance_turns <= 0.0:
        return 0.0
    if accel_rps2 <= 0.0 or decel_rps2 <= 0.0:
        return start_rps
    numerator = (2.0 * distance_turns) + ((start_rps * start_rps) / accel_rps2)
    denominator = (1.0 / accel_rps2) + (1.0 / decel_rps2)
    return math.sqrt(max(numerator / denominator, 0.0))


@dataclass(slots=True)
class WindingWindow:
    low_mm: float
    high_mm: float

    def __post_init__(self) -> None:
        if self.high_mm <= self.low_mm:
            raise ValueError("window_high_mm must be greater than window_low_mm")

    @property
    def width_mm(self) -> float:
        return self.high_mm - self.low_mm

    def clamp_mm(self, position_mm: float) -> float:
        return _clamp(position_mm, self.low_mm, self.high_mm)

    def fraction_for_position(self, position_mm: float) -> float:
        if self.width_mm <= 0.0:
            return 0.0
        return _clamp((position_mm - self.low_mm) / self.width_mm, 0.0, 1.0)

    def position_for_fraction(self, fraction: float) -> float:
        return self.low_mm + (_clamp(fraction, 0.0, 1.0) * self.width_mm)


@dataclass(slots=True)
class AdaptivePlanningSnapshot:
    total_turns: float
    completed_turns: float
    current_rpm: float
    target_rpm: float
    turns_per_mm: float
    chunk_time_s: float
    current_guide_mm: float
    direction_sign: int
    window: WindingWindow
    pause_requested: bool
    pause_at_turn: float | None


@dataclass(slots=True)
class AdaptiveChunkPlan:
    reason: str
    start_rpm: float
    end_rpm: float
    duration_s: float
    turns_delta: float
    direction_sign: int
    stop_at_end: bool
    reached_edge: bool

    @property
    def done_turns(self) -> float:
        return self.turns_delta


def plan_next_chunk(
    snapshot: AdaptivePlanningSnapshot,
    *,
    spindle_accel_rps2: float,
    spindle_decel_rps2: float,
) -> AdaptiveChunkPlan | None:
    remaining_turns = max(snapshot.total_turns - snapshot.completed_turns, 0.0)
    if remaining_turns <= _EPSILON:
        return None

    current_rps = max(_rpm_to_rps(snapshot.current_rpm), 0.0)
    target_rps = max(_rpm_to_rps(snapshot.target_rpm), 0.0)

    if snapshot.direction_sign > 0:
        edge_mm = snapshot.window.high_mm
    else:
        edge_mm = snapshot.window.low_mm
    turns_to_edge = max(
        abs(edge_mm - snapshot.current_guide_mm) * snapshot.turns_per_mm,
        0.0,
    )

    turn_limit = remaining_turns
    stop_reason = "complete"
    reached_edge = False

    if turns_to_edge + _EPSILON < turn_limit:
        turn_limit = turns_to_edge
        stop_reason = "edge"
        reached_edge = True

    if snapshot.pause_at_turn is not None:
        pause_remaining = max(snapshot.pause_at_turn - snapshot.completed_turns, 0.0)
        if pause_remaining <= _EPSILON:
            if current_rps <= _EPSILON:
                return None
            turn_limit = min(turn_limit, _distance_for_speed_change(current_rps, 0.0, spindle_decel_rps2))
            stop_reason = "pause"
            reached_edge = False
        elif pause_remaining + _EPSILON < turn_limit:
            turn_limit = pause_remaining
            stop_reason = "pause_turn"
            reached_edge = False

    if snapshot.pause_requested:
        if current_rps <= _EPSILON:
            return None
        turn_limit = min(
            turn_limit,
            max(_distance_for_speed_change(current_rps, 0.0, spindle_decel_rps2), _EPSILON),
        )
        stop_reason = "pause"
        reached_edge = False

    if turn_limit <= _EPSILON:
        return None

    stop_distance = _distance_for_speed_change(current_rps, 0.0, spindle_decel_rps2)
    must_stop_at_limit = stop_reason in {"complete", "edge", "pause", "pause_turn"}

    if current_rps > _EPSILON and must_stop_at_limit and turn_limit <= stop_distance + _EPSILON:
        if current_rps <= _EPSILON:
            return None
        duration_s = (2.0 * turn_limit) / max(current_rps, _EPSILON)
        return AdaptiveChunkPlan(
            reason=stop_reason,
            start_rpm=snapshot.current_rpm,
            end_rpm=0.0,
            duration_s=max(duration_s, 0.001),
            turns_delta=turn_limit,
            direction_sign=snapshot.direction_sign,
            stop_at_end=True,
            reached_edge=reached_edge and stop_reason == "edge",
        )

    if current_rps > target_rps + _EPSILON:
        next_rps = max(target_rps, current_rps - (spindle_decel_rps2 * snapshot.chunk_time_s))
        turns_delta = _distance_for_speed_change(current_rps, next_rps, spindle_decel_rps2)
        if turns_delta > turn_limit:
            next_rps = math.sqrt(max((current_rps * current_rps) - (2.0 * spindle_decel_rps2 * turn_limit), 0.0))
            next_rps = max(next_rps, target_rps)
            turns_delta = _distance_for_speed_change(current_rps, next_rps, spindle_decel_rps2)
        return AdaptiveChunkPlan(
            reason="decel",
            start_rpm=snapshot.current_rpm,
            end_rpm=_rps_to_rpm(next_rps),
            duration_s=max(_duration_for_speed_change(current_rps, next_rps, spindle_decel_rps2), 0.001),
            turns_delta=max(min(turns_delta, turn_limit), _EPSILON),
            direction_sign=snapshot.direction_sign,
            stop_at_end=False,
            reached_edge=False,
        )

    if current_rps + _EPSILON < target_rps:
        reachable_peak_rps = target_rps
        if must_stop_at_limit:
            reachable_peak_rps = min(
                reachable_peak_rps,
                _peak_rps_for_stop(
                    current_rps,
                    turn_limit,
                    spindle_accel_rps2,
                    spindle_decel_rps2,
                ),
            )
        next_rps = min(
            reachable_peak_rps,
            current_rps + (spindle_accel_rps2 * snapshot.chunk_time_s),
        )
        turns_delta = _distance_for_speed_change(current_rps, next_rps, spindle_accel_rps2)
        if turns_delta > turn_limit:
            next_rps = math.sqrt(max((current_rps * current_rps) + (2.0 * spindle_accel_rps2 * turn_limit), 0.0))
            next_rps = min(next_rps, reachable_peak_rps)
            turns_delta = _distance_for_speed_change(current_rps, next_rps, spindle_accel_rps2)
        if next_rps > current_rps + _EPSILON and turns_delta > _EPSILON:
            return AdaptiveChunkPlan(
                reason="accel",
                start_rpm=snapshot.current_rpm,
                end_rpm=_rps_to_rpm(next_rps),
                duration_s=max(_duration_for_speed_change(current_rps, next_rps, spindle_accel_rps2), 0.001),
                turns_delta=max(min(turns_delta, turn_limit), _EPSILON),
                direction_sign=snapshot.direction_sign,
                stop_at_end=False,
                reached_edge=False,
            )

    if current_rps <= _EPSILON:
        return None

    if must_stop_at_limit:
        cruise_turns = max(turn_limit - stop_distance, 0.0)
        if cruise_turns <= _EPSILON:
            duration_s = (2.0 * turn_limit) / max(current_rps, _EPSILON)
            return AdaptiveChunkPlan(
                reason=stop_reason,
                start_rpm=snapshot.current_rpm,
                end_rpm=0.0,
                duration_s=max(duration_s, 0.001),
                turns_delta=turn_limit,
                direction_sign=snapshot.direction_sign,
                stop_at_end=True,
                reached_edge=reached_edge and stop_reason == "edge",
            )
    else:
        cruise_turns = turn_limit

    turns_delta = min(cruise_turns, current_rps * snapshot.chunk_time_s)
    if turns_delta <= _EPSILON:
        turns_delta = min(turn_limit, max(current_rps * snapshot.chunk_time_s, _EPSILON))
    duration_s = turns_delta / max(current_rps, _EPSILON)
    return AdaptiveChunkPlan(
        reason="cruise",
        start_rpm=snapshot.current_rpm,
        end_rpm=snapshot.current_rpm,
        duration_s=max(duration_s, 0.001),
        turns_delta=max(min(turns_delta, turn_limit), _EPSILON),
        direction_sign=snapshot.direction_sign,
        stop_at_end=False,
        reached_edge=False,
    )


class AdaptiveWindingRuntime:
    """Thread-safe mutable control state for one adaptive winding session."""

    def __init__(
        self,
        program: WindingProgram,
        params: SessionParams,
        *,
        window_low_mm: float,
        window_high_mm: float,
        spindle_steps_per_rev: int,
        lateral_steps_per_mm: float,
    ) -> None:
        self._lock = threading.RLock()
        self._program = program
        self._params = params
        self._total_turns: float = (
            float(params.total_turns)
            if params.total_turns is not None
            else program.total_turns()
        )
        self._chunk_time_s: float = params.chunk_time_s
        self._window = WindingWindow(window_low_mm, window_high_mm)
        self._wire_diameter_mm: float = program.wire_diameter_mm
        self._turns_per_mm_override: float | None = None
        self._pitch_factor: float = program.layer_pitch_mm / program.wire_diameter_mm
        self._target_rpm: float = params.spindle_rpm
        self._current_turns = 0.0
        self._current_rpm = 0.0
        self._current_guide_mm = self._window.low_mm
        self._current_scatter_reference_mm = 0.0
        self._direction_sign = 1
        self._pause_requested = False
        self._pause_at_turn: float | None = None
        self._stop_requested = False
        self._pending_reposition_mm: float | None = None
        self._state = "queued"
        self._last_error: str | None = None
        self._spindle_steps_per_rev = spindle_steps_per_rev
        self._lateral_steps_per_mm = lateral_steps_per_mm

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def stop_requested(self) -> bool:
        with self._lock:
            return self._stop_requested

    @property
    def pause_requested(self) -> bool:
        with self._lock:
            return self._pause_requested

    @property
    def current_rpm(self) -> float:
        with self._lock:
            return self._current_rpm

    @property
    def current_guide_mm(self) -> float:
        with self._lock:
            return self._current_guide_mm

    @property
    def current_turns(self) -> float:
        with self._lock:
            return self._current_turns

    @property
    def current_scatter_reference_mm(self) -> float:
        with self._lock:
            return self._current_scatter_reference_mm

    @property
    def current_window(self) -> WindingWindow:
        with self._lock:
            return WindingWindow(self._window.low_mm, self._window.high_mm)

    @property
    def direction_sign(self) -> int:
        with self._lock:
            return self._direction_sign

    @property
    def turns_per_mm(self) -> float:
        with self._lock:
            if self._turns_per_mm_override is not None:
                return float(self._turns_per_mm_override)
            return 1.0 / (self._wire_diameter_mm * self._pitch_factor)

    def build_scatter_engine(self) -> ScatterEngine:
        with self._lock:
            return ScatterEngine(
                amplitude_mm=self._program.scatter_amplitude_mm,
                freq1=self._program.scatter_freq1,
                freq2=self._program.scatter_freq2,
                damping_margin_mm=self._program.scatter_damping_margin_mm,
            )

    @property
    def target_rpm(self) -> float:
        with self._lock:
            return self._target_rpm

    def planning_snapshot(self) -> AdaptivePlanningSnapshot:
        with self._lock:
            return AdaptivePlanningSnapshot(
                total_turns=self._total_turns,
                completed_turns=self._current_turns,
                current_rpm=self._current_rpm,
                target_rpm=self._target_rpm,
                turns_per_mm=self.turns_per_mm,
                chunk_time_s=self._chunk_time_s,
                current_guide_mm=self._current_guide_mm,
                direction_sign=self._direction_sign,
                window=WindingWindow(self._window.low_mm, self._window.high_mm),
                pause_requested=self._pause_requested,
                pause_at_turn=self._pause_at_turn,
            )

    def request_stop(self) -> None:
        with self._lock:
            self._stop_requested = True
            if self._state not in {"completed", "fault", "stopped"}:
                self._state = "stopping"

    def request_pause(self, *, pause_at_turn: float | None = None) -> None:
        with self._lock:
            self._pause_requested = True
            if pause_at_turn is not None:
                self._pause_at_turn = max(float(pause_at_turn), self._current_turns)

    def resume(self) -> None:
        with self._lock:
            self._pause_requested = False
            self._pause_at_turn = None
            if self._state == "paused":
                self._state = "running"

    def mark_homing(self) -> None:
        """Transition to the pre-winding homing/repositioning state."""
        with self._lock:
            if self._state == "queued":
                self._state = "homing"

    def mark_running(self) -> None:
        with self._lock:
            self._state = "running"

    def mark_paused(self) -> None:
        with self._lock:
            self._state = "paused"
            self._current_rpm = 0.0

    def mark_completed(self) -> None:
        with self._lock:
            self._state = "completed"
            self._current_rpm = 0.0
            self._pause_requested = False
            self._pause_at_turn = None
            self._pending_reposition_mm = None

    def mark_stopped(self, reason: str | None = None) -> None:
        with self._lock:
            self._state = "stopped"
            self._current_rpm = 0.0
            self._pause_requested = False
            self._pause_at_turn = None
            if reason:
                self._last_error = reason

    def mark_fault(self, error: str) -> None:
        with self._lock:
            self._state = "fault"
            self._current_rpm = 0.0
            self._last_error = error

    def update_controls(
        self,
        *,
        spindle_rpm: float | None = None,
        total_turns: float | None = None,
        window_low_mm: float | None = None,
        window_high_mm: float | None = None,
    ) -> None:
        with self._lock:
            if spindle_rpm is not None:
                if spindle_rpm < 0.0:
                    raise ValueError("spindle_rpm must be >= 0")
                self._target_rpm = float(spindle_rpm)
                if self._target_rpm <= _EPSILON:
                    self._pause_requested = True

            if total_turns is not None:
                if total_turns <= 0.0:
                    raise ValueError("total_turns must be positive")
                self._total_turns = float(total_turns)

            if window_low_mm is not None or window_high_mm is not None:
                old_window = self._window
                new_window = WindingWindow(
                    old_window.low_mm if window_low_mm is None else float(window_low_mm),
                    old_window.high_mm if window_high_mm is None else float(window_high_mm),
                )
                fraction = old_window.fraction_for_position(self._current_guide_mm)
                target_mm = new_window.position_for_fraction(fraction)
                self._window = new_window
                self._pending_reposition_mm = target_mm
                if self._current_rpm > _EPSILON:
                    self._pause_requested = True

    def consume_pending_reposition(self) -> float | None:
        with self._lock:
            target = self._pending_reposition_mm
            self._pending_reposition_mm = None
            return target

    def set_repositioned_guide(
        self,
        position_mm: float,
        *,
        scatter_reference_mm: float,
    ) -> None:
        with self._lock:
            self._current_guide_mm = self._window.clamp_mm(position_mm)
            self._current_scatter_reference_mm = scatter_reference_mm

    def apply_completed_move(self, plan: AdaptiveChunkPlan, move: "AdaptiveWindingMove") -> None:
        with self._lock:
            self._current_turns = min(
                self._total_turns,
                self._current_turns + move.spindle_turns_delta,
            )
            self._current_rpm = plan.end_rpm
            self._current_guide_mm = move.end_guide_mm
            self._current_scatter_reference_mm = move.end_scatter_offset_mm
            if plan.stop_at_end:
                self._current_rpm = 0.0
            if plan.reached_edge:
                self._direction_sign *= -1
            if self._pause_at_turn is not None and self._current_turns >= self._pause_at_turn - _EPSILON:
                self._pause_requested = True
                self._pause_at_turn = None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            remaining_turns = max(self._total_turns - self._current_turns, 0.0)
            return {
                "state": self._state,
                "name": self._program.name,
                "target_turns": self._total_turns,
                "completed_turns": self._current_turns,
                "remaining_turns": remaining_turns,
                "spindle_rpm": self._target_rpm,
                "target_rpm": self._target_rpm,
                "current_rpm": self._current_rpm,
                "direction": "forward" if self._direction_sign > 0 else "reverse",
                "guide_position_mm": self._current_guide_mm,
                "window_low_mm": self._window.low_mm,
                "window_high_mm": self._window.high_mm,
                "window_width_mm": self._window.width_mm,
                "turns_per_mm": self.turns_per_mm,
                "wire_diameter_mm": self._wire_diameter_mm,
                "pitch_factor": self._pitch_factor,
                "chunk_time_s": self._chunk_time_s,
                "scatter_amplitude_mm": self._program.scatter_amplitude_mm,
                "scatter_damping_margin_mm": self._program.scatter_damping_margin_mm,
                "scatter_freq1": self._program.scatter_freq1,
                "scatter_freq2": self._program.scatter_freq2,
                "pause_requested": self._pause_requested,
                "pause_at_turn": self._pause_at_turn,
                "stop_requested": self._stop_requested,
                "pending_reposition_mm": self._pending_reposition_mm,
                "spindle_steps_completed": int(round(self._current_turns * self._spindle_steps_per_rev)),
                "spindle_steps_remaining": int(round(remaining_turns * self._spindle_steps_per_rev)),
                "lateral_position_steps": int(round(self._current_guide_mm * self._lateral_steps_per_mm)),
                "last_error": self._last_error,
            }


class AdaptiveWindingMove(SynchronizedMove):
    """Tracked synchronized winding chunk with mutable session controls."""

    def __init__(
        self,
        *,
        name: str,
        kinematics: SpindleKinematics,
        scatter: ScatterEngine,
        window: WindingWindow,
        turns_per_mm: float,
        direction_sign: int,
        start_turns_abs: float,
        start_guide_mm: float,
        start_scatter_reference_mm: float,
        spindle_cfg: SyncAxisConfig,
        traverse_cfg: SyncAxisConfig,
        segment_duration_s: float = 0.004,
    ) -> None:
        super().__init__(name)
        if direction_sign not in (-1, 1):
            raise ValueError("direction_sign must be -1 or 1")
        if turns_per_mm <= 0.0:
            raise ValueError("turns_per_mm must be positive")

        self._kinematics = kinematics
        self.scatter = scatter
        self.window = window
        self.turns_per_mm = turns_per_mm
        self.direction_sign = direction_sign
        self.start_turns_abs = start_turns_abs
        self.start_guide_mm = start_guide_mm
        self.start_scatter_reference_mm = start_scatter_reference_mm
        self._spindle_cfg = spindle_cfg
        self.traverse_cfg = traverse_cfg
        self._segment_duration_s = segment_duration_s

        self.spindle_turns_delta = self._kinematics.turns_at(self._kinematics.total_duration)
        self.end_guide_mm = self._guide_position_mm_at(self.spindle_turns_delta)
        self.end_scatter_offset_mm = self._scatter_offset_at(self.spindle_turns_delta)
        self._spindle_delta_steps = int(round(self.spindle_turns_delta * self._spindle_cfg.steps_per_unit))
        self._traverse_delta_steps = int(round((self.end_guide_mm - self.start_guide_mm) * self.traverse_cfg.steps_per_unit))

    def _base_start_mm(self) -> float:
        return self.start_guide_mm - self.start_scatter_reference_mm

    @property
    def kinematics(self) -> SpindleKinematics:
        return self._kinematics

    @property
    def spindle_cfg(self) -> SyncAxisConfig:
        return self._spindle_cfg

    @property
    def segment_duration_s(self) -> float:
        return self._segment_duration_s

    def _scatter_offset_at(self, delta_turns: float) -> float:
        absolute_turns = self.start_turns_abs + delta_turns
        base_mm = self._base_start_mm() + (self.direction_sign * (delta_turns / self.turns_per_mm))
        relative_base_mm = _clamp(base_mm - self.window.low_mm, 0.0, self.window.width_mm)
        return self.scatter.get_offset(
            absolute_turns,
            relative_base_mm,
            self.window.width_mm,
        )

    def _guide_position_mm_at(self, delta_turns: float) -> float:
        base_mm = self._base_start_mm() + (self.direction_sign * (delta_turns / self.turns_per_mm))
        position_mm = base_mm + self._scatter_offset_at(delta_turns)
        return self.window.clamp_mm(position_mm)

    def segments(self) -> Iterator[MultiAxisSegment]:
        def spindle_steps_at(t: float) -> float:
            return self.kinematics.turns_at(t) * self.spindle_cfg.steps_per_unit

        def traverse_steps_at(t: float) -> float:
            delta_turns = self.kinematics.turns_at(t)
            return self._guide_position_mm_at(delta_turns) * self.traverse_cfg.steps_per_unit

        axis_profiles = [
            AxisStepProfile(
                axis_index=self.spindle_cfg.axis_index,
                step_at=spindle_steps_at,
                reverse_direction=self.spindle_cfg.reverse_direction,
                total_duration=self.kinematics.total_duration,
            ),
            AxisStepProfile(
                axis_index=self.traverse_cfg.axis_index,
                step_at=traverse_steps_at,
                reverse_direction=self.traverse_cfg.reverse_direction,
                total_duration=self.kinematics.total_duration,
            ),
        ]
        yield from StepProfileSegmentGenerator(
            axis_profiles,
            segment_duration_s=self.segment_duration_s,
        )

    def expected_delta_steps(self, axis_id: int) -> int | None:
        if axis_id == self.spindle_cfg.axis_index:
            return -self._spindle_delta_steps if self.spindle_cfg.reverse_direction else self._spindle_delta_steps
        if axis_id == self.traverse_cfg.axis_index:
            return self._traverse_delta_steps
        return None

    @property
    def axis_ids(self) -> list[int]:
        return [self.spindle_cfg.axis_index, self.traverse_cfg.axis_index]