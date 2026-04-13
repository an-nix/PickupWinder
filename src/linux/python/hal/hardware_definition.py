from dataclasses import dataclass, field
from typing import Optional

@dataclass
class MotorDefinition:
    name: str = ""
    steps_per_revolution: int = 200
    microstepping: int = 32
    # For translation axes (e.g. lateral) prefer steps_per_mm as canonical.
    # If None, the caller may compute it from steps_per_revolution and
    # microstepping plus a lead-screw pitch provided elsewhere.
    steps_per_mm: Optional[int] = None


@dataclass
class HardWareDefinition:
    spindle: MotorDefinition = field(default_factory=lambda: MotorDefinition(name="spindle"))
    lateral: MotorDefinition = field(default_factory=lambda: MotorDefinition(name="lateral"))
    tensionner: MotorDefinition = field(default_factory=lambda: MotorDefinition(name="tensionner"))

    # Lateral axis convenience fields (kept for compatibility with existing
    # config JSON keys). These mirror values available under `lateral`.
    lat_steps_per_mm: int = 3072       # 96-step motor × 32 µstep, M6 1 mm pitch
    lat_traverse_max_mm: float = 20.0  # Hard software travel limit (mm)
    lat_home_hz: int = 2000            # Homing approach speed (steps/s)
    lat_traverse_hz: int = 15000       # Positioning / jog speed (steps/s)
    lat_rodage_hz: int = 5000          # Break-in (rodage) traversal speed (steps/s)
    lat_accel: int = 100_000           # Lateral acceleration (steps/s²)

    # Spindle convenience fields (mirrors `spindle` motor info)
    sp_steps_per_rev: int = 6400       # 200-step motor × 32 µstep
    sp_hz_min: int = 1067              # ≈ 10 RPM
    sp_hz_max: int = 160_000           # ≈ 1500 RPM

    def __post_init__(self):
        # Keep lateral.steps_per_mm synced with lat_steps_per_mm when one is set
        if self.lateral.steps_per_mm is None and self.lat_steps_per_mm is not None:
            self.lateral.steps_per_mm = int(self.lat_steps_per_mm)
        else:
            # If lateral carries a value prefer it and mirror back
            try:
                if self.lateral.steps_per_mm is not None:
                    self.lat_steps_per_mm = int(self.lateral.steps_per_mm)
            except Exception:
                pass

        # Mirror spindle steps-per-rev convenience field and base motor fields
        try:
            if self.spindle.steps_per_revolution is None and self.sp_steps_per_rev:
                # derive base step count if microstepping available
                if self.spindle.microstepping:
                    self.spindle.steps_per_revolution = int(self.sp_steps_per_rev // self.spindle.microstepping)
            else:
                # ensure sp_steps_per_rev reflects spindle * microstepping
                self.sp_steps_per_rev = int(self.spindle.steps_per_revolution * (self.spindle.microstepping or 1))
        except Exception:
            pass


