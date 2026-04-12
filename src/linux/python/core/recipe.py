"""core/recipe.py — Winding recipe dataclass.

Migrated from resources/esp32/include/WindingPattern.h (WindingRecipe struct)
and resources/esp32/src/WindingRecipeStore.cpp.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .geometry import WindingGeometry
from .types import WindingEndPos, WindingStyle


PICKUP_RECIPE_FORMAT_VERSION: int = 2


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class WindingRecipe:
    """All persistent parameters for a winding session.

    Mirrors the C++ WindingRecipe struct.  The `geometry` field replaces
    the old `turns_per_pass_base` — use `geometry.turns_per_pass()` instead.
    """

    version: int = PICKUP_RECIPE_FORMAT_VERSION

    # Session parameters
    target_turns: int = 7500
    freerun: bool = False
    direction_cw: bool = True

    # Pattern parameters
    style: WindingStyle = WindingStyle.STRAIGHT
    seed: int = 1337
    layer_jitter_pct: float = 0.0
    layer_speed_pct: float = 0.0
    human_traverse_pct: float = 0.0
    human_speed_pct: float = 0.0
    first_pass_traverse_factor: float = 1.0

    # End-position handling
    end_pos: WindingEndPos = WindingEndPos.NONE
    end_pos_turns: int = 3

    # Lateral axis
    lat_offset_mm: float = 0.0

    # Bobbin geometry (replaces the old turns_per_pass_base scalar)
    geometry: WindingGeometry = field(default_factory=WindingGeometry)

    def normalized(self) -> "WindingRecipe":
        """Clamp all fields to safe runtime ranges. Returns self."""
        self.target_turns = max(1, int(self.target_turns))
        self.seed = max(1, int(self.seed))
        self.end_pos_turns = max(1, min(20, int(self.end_pos_turns)))
        self.lat_offset_mm = max(0.0, float(self.lat_offset_mm))

        self.layer_jitter_pct = _clamp(float(self.layer_jitter_pct), 0.0, 0.45)
        self.layer_speed_pct = _clamp(float(self.layer_speed_pct), 0.0, 0.45)
        self.human_traverse_pct = _clamp(float(self.human_traverse_pct), 0.0, 0.45)
        self.human_speed_pct = _clamp(float(self.human_speed_pct), 0.0, 0.45)
        self.first_pass_traverse_factor = _clamp(float(self.first_pass_traverse_factor), 0.40, 1.80)

        self.geometry.scatter_factor = _clamp(float(self.geometry.scatter_factor), 0.5, 4.0)
        return self
