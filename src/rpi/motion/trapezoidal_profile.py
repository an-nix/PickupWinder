from __future__ import annotations


class TrapezoidalMotionProfile:
    def __init__(
        self,
        start_rpm: float = 0.0,
        target_rpm: float = 1000.0,
        accel_s: float = 0.0,
        cruise_s: float = 0.0,
        decel_s: float = 0.0,
    ) -> None:
        if start_rpm < 0.0 or target_rpm < 0.0:
            raise ValueError("start_rpm and target_rpm must be non-negative")
        if accel_s < 0.0 or cruise_s < 0.0 or decel_s < 0.0:
            raise ValueError("accel_s, cruise_s, and decel_s must be >= 0")

        self.start_rpm = start_rpm
        self.target_rpm = target_rpm
        self.accel_s = accel_s
        self.cruise_s = cruise_s
        self.decel_s = decel_s

    @property
    def total_duration(self) -> float:
        return self.accel_s + self.cruise_s + self.decel_s

    def _clamp_time(self, t: float) -> float:
        return min(max(t, 0.0), self.total_duration)

    def rps_at(self, t: float) -> float:
        t = self._clamp_time(t)
        start_rps = self.start_rpm / 60.0
        target_rps = self.target_rpm / 60.0

        if t < self.accel_s:
            if self.accel_s <= 0.0:
                return target_rps
            rate = (target_rps - start_rps) / self.accel_s
            return start_rps + rate * t

        t -= self.accel_s
        if t < self.cruise_s:
            return target_rps

        t -= self.cruise_s
        if self.decel_s <= 0.0:
            return target_rps
        rate = (target_rps - start_rps) / self.decel_s
        return max(target_rps - rate * t, 0.0)

    def rpm_at(self, t: float) -> float:
        return self.rps_at(t) * 60.0

    def turns_at(self, t: float) -> float:
        t = self._clamp_time(t)
        start_rps = self.start_rpm / 60.0
        target_rps = self.target_rpm / 60.0

        if t < self.accel_s:
            if self.accel_s <= 0.0:
                return target_rps * t
            rate = (target_rps - start_rps) / self.accel_s
            return start_rps * t + 0.5 * rate * t * t

        turns = 0.0
        if self.accel_s > 0.0:
            rate = (target_rps - start_rps) / self.accel_s
            turns += start_rps * self.accel_s + 0.5 * rate * self.accel_s * self.accel_s
        else:
            turns += target_rps * self.accel_s

        t -= self.accel_s
        if t < self.cruise_s:
            return turns + target_rps * t

        turns += target_rps * self.cruise_s
        t -= self.cruise_s
        if self.decel_s <= 0.0:
            return max(turns + target_rps * t, 0.0)

        rate = (target_rps - start_rps) / self.decel_s
        result = turns + target_rps * t - 0.5 * rate * t * t
        return max(result, 0.0)

    def steps_at(self, t: float, steps_per_rev: int) -> float:
        return self.turns_at(t) * float(steps_per_rev)

    def step_delta(self, time_start: float, time_end: float, steps_per_rev: int) -> float:
        return self.steps_at(time_end, steps_per_rev) - self.steps_at(time_start, steps_per_rev)
