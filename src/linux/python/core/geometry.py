"""core/geometry.py — Bobbin geometry and wire-gauge constants.

Migrated from resources/esp32/include/WindingGeometry.h.
Pure Python, no hardware dependencies.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import floor


# ── Wire gauge constants (insulated diameter, mm) ─────────────────────────────

class WireGauge(float, Enum):
    """Standard insulated wire diameters in mm.

    Inherits from float so values work directly in arithmetic:
        spacing = WireGauge.AWG42 * scatter_factor   # no .value needed
    """
    AWG42 = 0.071   # Stratocaster / standard single-coil
    AWG43 = 0.064
    AWG44 = 0.058
    AWG46 = 0.047   # Very fine vintage pickups


# ── Bobbin presets ─────────────────────────────────────────────────────────────

BOBBIN_PRESETS: list[dict] = [
    {"name": "Strat",      "total_mm": 17.0, "flange_bot_mm": 1.5, "flange_top_mm": 1.5, "wire_mm": WireGauge.AWG42},
    {"name": "Telecaster", "total_mm": 18.5, "flange_bot_mm": 1.5, "flange_top_mm": 1.5, "wire_mm": WireGauge.AWG42},
    {"name": "P90",        "total_mm": 30.0, "flange_bot_mm": 2.0, "flange_top_mm": 2.0, "wire_mm": WireGauge.AWG42},
    {"name": "Humbucker",  "total_mm": 38.0, "flange_bot_mm": 2.0, "flange_top_mm": 2.0, "wire_mm": WireGauge.AWG43},
]


@dataclass
class WindingGeometry:
    """Geometric parameters of the bobbin and wire.

    Mirrors the C++ WindingGeometry struct from WindingGeometry.h.
    All lengths in millimetres, all calculations are pure float math.
    """

    total_width_mm: float = 17.0
    flange_bottom_mm: float = 1.5
    flange_top_mm: float = 1.5
    margin_mm: float = 0.5
    winding_start_trim_mm: float = 0.0
    winding_end_trim_mm: float = 0.0
    wire_diameter_mm: float = float(WireGauge.AWG42)  # WireGauge is a float Enum
    turns_per_pass_offset: int = 0
    scatter_factor: float = 1.0

    # ── Bounds ─────────────────────────────────────────────────────────────────

    def winding_start_mm(self) -> float:
        """Low / start bound in mm from bobbin base."""
        return max(0.0, self.flange_bottom_mm + self.margin_mm + self.winding_start_trim_mm)

    def winding_end_mm(self) -> float:
        """High / end bound in mm from bobbin base."""
        end = (self.total_width_mm - self.flange_top_mm - self.margin_mm
               + self.winding_end_trim_mm)
        return max(self.winding_start_mm(), end)

    def effective_width(self) -> float:
        """Usable winding width in mm (end - start, clamped >= 0)."""
        return max(0.0, self.winding_end_mm() - self.winding_start_mm())

    # ── Turns per pass ─────────────────────────────────────────────────────────

    def turns_per_pass_calc(self) -> int:
        """Calculated turns-per-pass from geometry, before offset."""
        if self.wire_diameter_mm <= 0.0:
            return 1
        spacing = self.wire_diameter_mm * max(0.5, self.scatter_factor)
        return max(1, int(self.effective_width() / spacing))

    def turns_per_pass(self) -> int:
        """Effective turns-per-pass = calc + manual offset, clamped >= 1."""
        return max(1, self.turns_per_pass_calc() + self.turns_per_pass_offset)

    # ── Presets ────────────────────────────────────────────────────────────────

    def apply_preset(self, idx: int,
                     presets: list | None = None) -> None:
        """Apply a bobbin geometry preset by index.

        *presets* — optional list of preset dicts (e.g. from config.json).
        Falls back to the built-in BOBBIN_PRESETS when None or empty.
        Accepts both legacy keys (total/flange_bot/flange_top/wire) and
        the preferred _mm suffix form (total_mm/flange_bot_mm/…).
        """
        effective = presets if presets else BOBBIN_PRESETS
        if idx < 0 or idx >= len(effective):
            return
        p = effective[idx]
        self.total_width_mm       = float(p.get("total_mm",      p.get("total",      17.0)))
        self.flange_bottom_mm     = float(p.get("flange_bot_mm", p.get("flange_bot",  1.5)))
        self.flange_top_mm        = float(p.get("flange_top_mm", p.get("flange_top",  1.5)))
        self.winding_start_trim_mm = 0.0
        self.winding_end_trim_mm   = 0.0
        self.wire_diameter_mm     = float(p.get("wire_mm", p.get("wire", WireGauge.AWG42.value)))
        self.turns_per_pass_offset = 0

    # ── Traverse quality ───────────────────────────────────────────────────────

    def compute_traverse_ratio(self) -> float:
        """Turns per mm.  Should be non-integer for good cross-hatch pattern.

        If ratio is an integer (3, 4, 5 …) wires stack vertically on repeating
        layers → visible banding and uneven density.
        """
        w = self.effective_width()
        if w <= 0.0:
            return 0.0
        return self.turns_per_pass() / w

    @staticmethod
    def is_traverse_ratio_problematic(ratio: float, tolerance: float = 0.15) -> bool:
        """Return True if ratio is within *tolerance* of an integer (banding risk)."""
        if ratio <= 0.0:
            return False
        frac = ratio - floor(ratio)
        return frac <= tolerance or frac >= (1.0 - tolerance)
