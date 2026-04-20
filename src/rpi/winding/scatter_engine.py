from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(slots=True)
class ScatterEngine:
    """Adds a spatial offset to the guide position to avoid exact layer stacking."""
    amplitude_mm: float = 0.0
    freq1: float = 1.0     # rad/turn
    freq2: float = 1.618   # rad/turn
    damping_margin_mm: float = 1.0

    def __post_init__(self) -> None:
        if self.amplitude_mm < 0.0:
            raise ValueError("amplitude_mm must be >= 0")
        if self.freq1 <= 0.0:
            raise ValueError("freq1 must be positive")
        if self.freq2 <= 0.0:
            raise ValueError("freq2 must be positive")
        if self.damping_margin_mm < 0.0:
            raise ValueError("damping_margin_mm must be >= 0")

    def get_offset(self, spindle_turns: float, base_guide_pos_mm: float, bobbin_width_mm: float) -> float:
        if self.amplitude_mm <= 0.0:
            return 0.0

        raw_scatter = (math.sin(self.freq1 * spindle_turns) + math.sin(self.freq2 * spindle_turns)) / 2.0
        offset = raw_scatter * self.amplitude_mm

        dist_to_0 = base_guide_pos_mm
        dist_to_end = bobbin_width_mm - base_guide_pos_mm
        min_dist = min(dist_to_0, dist_to_end)

        if min_dist < self.damping_margin_mm and self.damping_margin_mm > 0:
            damping_factor = max(0.0, min_dist / self.damping_margin_mm)
            offset *= damping_factor

        return offset
