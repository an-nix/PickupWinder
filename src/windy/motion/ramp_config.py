
from dataclasses import dataclass

from .trapezoidal_profile import TrapezoidalMotionProfile


def compute_ramp_times(
    target_rpm: float,
    duration_s: float,
    max_accel_steps_per_s2: float,
    max_decel_steps_per_s2: float,
    steps_per_rev: int,
    start_rpm: float = 0.0,
    min_ramp_s: float = 0.05,
    max_ramp_fraction: float = 0.25,
) -> tuple[float, float, float]:
    """Compute trapezoidal ramp times from machine acceleration limits.

    Implements:  ramp_s = clamp(physics_required, min_ramp_s, duration_s * max_ramp_fraction)

    The *physics_required* time is the minimum duration needed to reach
    ``target_rpm`` from ``start_rpm`` at the given acceleration limit.
    If the limit is zero (unconstrained), ``min_ramp_s`` is used directly.

    Args:
        target_rpm:             Target rotational speed in RPM.
        duration_s:             Total move duration in seconds.
        max_accel_steps_per_s2: Acceleration limit in steps/s².
        max_decel_steps_per_s2: Deceleration limit in steps/s².
        steps_per_rev:          Steps per motor revolution (full × microstep).
        start_rpm:              Initial speed in RPM (default 0).
        min_ramp_s:             Hard floor for accel / decel time (default 0.05 s).
        max_ramp_fraction:      Maximum fraction of ``duration_s`` allowed for
                                each ramp phase (default 0.25, i.e. 25 %).

    Returns:
        Tuple ``(accel_s, cruise_s, decel_s)`` that sum to at most ``duration_s``.
    """
    start_hz: float = start_rpm / 60.0 * float(steps_per_rev)
    target_hz: float = target_rpm / 60.0 * float(steps_per_rev)
    delta_hz: float = max(target_hz - start_hz, 0.0)

    if max_accel_steps_per_s2 > 0.0:
        physics_accel_s = delta_hz / max_accel_steps_per_s2
    else:
        physics_accel_s = min_ramp_s

    if max_decel_steps_per_s2 > 0.0:
        physics_decel_s = delta_hz / max_decel_steps_per_s2
    else:
        physics_decel_s = min_ramp_s

    cap = duration_s * max_ramp_fraction
    accel_s = max(min_ramp_s, min(physics_accel_s, cap))
    decel_s = max(min_ramp_s, min(physics_decel_s, cap))
    cruise_s = max(duration_s - accel_s - decel_s, 0.0)
    return accel_s, cruise_s, decel_s




@dataclass(slots=True)
class RampConfig:
    axis_id: int = 0
    steps_per_rev: int = 200 * 32
    start_rpm: float = 0.0
    target_rpm: float = 1000.0
    accel_s: float = 10.0
    cruise_s: float = 3.0
    decel_s: float = 10.0
    # Must match RMT_STEP_RESOLUTION_HZ in stepper_driver.h (80 MHz).
    resolution_hz: int = 40_000_000
    reverse_direction: bool = False

    @property
    def start_hz(self) -> float:
        return self.start_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def target_hz(self) -> float:
        return self.target_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def profile(self) -> TrapezoidalMotionProfile:
        return TrapezoidalMotionProfile(
            start_rpm=self.start_rpm,
            target_rpm=self.target_rpm,
            accel_s=self.accel_s,
            cruise_s=self.cruise_s,
            decel_s=self.decel_s,
        )

    @property
    def total_duration(self) -> float:
        return self.profile.total_duration

    def hz_at_time(self, t: float) -> float:
        return self.profile.rps_at(t) * float(self.steps_per_rev)

    def steps_at(self, t: float) -> float:
        return self.profile.steps_at(t, self.steps_per_rev)

    def step_delta(self, time_start: float, time_end: float) -> float:
        return self.profile.step_delta(time_start, time_end, self.steps_per_rev)

