from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class WindingPattern:
    """Converts Spindle position (turns) to Guide position (mm) in a triangular wave."""
    bobbin_width_mm: float
    turns_per_mm: float

    def guide_pos_mm(self, spindle_turns: float) -> float:
        if self.turns_per_mm <= 0 or self.bobbin_width_mm <= 0:
            return 0.0
            
        total_dist = spindle_turns / self.turns_per_mm
        cycle_length = 2.0 * self.bobbin_width_mm
            
        mod_dist = total_dist % cycle_length
        if mod_dist <= self.bobbin_width_mm:
            return mod_dist
        else:
            return cycle_length - mod_dist
