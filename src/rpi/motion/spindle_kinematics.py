from __future__ import annotations

from dataclasses import dataclass

from .trapezoidal_profile import TrapezoidalMotionProfile


@dataclass
class SpindleKinematics(TrapezoidalMotionProfile):
    """Calculates absolute angular position (in turns) of the Spindle at time t."""
    target_rpm: float
    start_rpm: float = 0.0
    accel_s: float = 2.0
    cruise_s: float = 10.0
    decel_s: float = 2.0

    def __post_init__(self) -> None:
        super().__init__(
            start_rpm=self.start_rpm,
            target_rpm=self.target_rpm,
            accel_s=self.accel_s,
            cruise_s=self.cruise_s,
            decel_s=self.decel_s,
        )

