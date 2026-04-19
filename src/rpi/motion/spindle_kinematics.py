from __future__ import annotations

from dataclasses import dataclass



@dataclass(slots=True)
class SpindleKinematics:
    """Calculates absolute angular position (in turns) of the Spindle at time t."""
    target_rpm: float
    start_rpm: float = 0.0
    accel_s: float = 2.0
    cruise_s: float = 10.0
    decel_s: float = 2.0

    @property
    def total_duration(self) -> float:
        return self.accel_s + self.cruise_s + self.decel_s

    def turns_at(self, t: float) -> float:
        t = min(max(t, 0.0), self.total_duration)
        start_rps = self.start_rpm / 60.0
        target_rps = self.target_rpm / 60.0
        
        pos = 0.0
        
        # Accel phase
        t_accel = min(t, self.accel_s)
        if self.accel_s > 0:
            rate = (target_rps - start_rps) / self.accel_s
            pos += start_rps * t_accel + 0.5 * rate * t_accel**2
        else:
            pos += target_rps * t_accel
            
        if t <= self.accel_s:
            return pos
            
        # Cruise phase
        t_cruise = min(t - self.accel_s, self.cruise_s)
        pos += target_rps * t_cruise
        
        if t <= self.accel_s + self.cruise_s:
            return pos
            
        # Decel phase
        t_decel = min(t - self.accel_s - self.cruise_s, self.decel_s)
        if self.decel_s > 0:
            rate = (target_rps - start_rps) / self.decel_s
            pos += target_rps * t_decel - 0.5 * rate * t_decel**2
        else:
            pos += target_rps * t_decel
            
        return pos

