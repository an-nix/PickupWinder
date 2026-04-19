
from dataclasses import dataclass




@dataclass(slots=True)
class RampConfig:
    axis_id: int = 0
    steps_per_rev: int = 200 * 32
    start_hz: float = 200.0
    target_rpm: float = 1000.0
    accel_s: float = 10.0
    cruise_s: float = 3.0
    decel_s: float = 10.0
    # Must match RMT_STEP_RESOLUTION_HZ in stepper_driver.h (80 MHz).
    resolution_hz: int = 80_000_000
    reverse_direction: bool = False
    phase_segments: int = 8
    segment_duration_s: float = 0.05

    @property
    def target_hz(self) -> float:
        return self.target_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def total_duration(self) -> float:
        return self.accel_s + self.cruise_s + self.decel_s

