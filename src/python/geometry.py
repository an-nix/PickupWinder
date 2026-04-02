"""WindingGeometry - mirrors WindingGeometry.h / struct WindingGeometry."""

from config import LAT_STEPS_PER_MM


class WireGauge:
    AWG42 = 0.071
    AWG43 = 0.064
    AWG44 = 0.058
    AWG46 = 0.047


BOBBIN_PRESETS = [
    {"name": "Strat",      "total": 17.0, "flangeBot": 1.5, "flangeTop": 1.5, "wire": WireGauge.AWG42},
    {"name": "Telecaster", "total": 18.5, "flangeBot": 1.5, "flangeTop": 1.5, "wire": WireGauge.AWG42},
    {"name": "P90",        "total": 30.0, "flangeBot": 2.0, "flangeTop": 2.0, "wire": WireGauge.AWG42},
    {"name": "Humbucker",  "total": 38.0, "flangeBot": 2.0, "flangeTop": 2.0, "wire": WireGauge.AWG43},
]


class WindingGeometry:
    def __init__(self):
        self.total_width_mm = 17.0
        self.flange_bottom_mm = 1.5
        self.flange_top_mm = 1.5
        self.margin_mm = 0.5
        self.winding_start_trim_mm = 0.0
        self.winding_end_trim_mm = 0.0
        self.wire_diameter_mm = WireGauge.AWG42
        self.turns_per_pass_offset = 0
        self.scatter_factor = 1.0

    def winding_start_mm(self) -> float:
        return max(0.0, self.flange_bottom_mm + self.margin_mm + self.winding_start_trim_mm)

    def winding_end_mm(self) -> float:
        end = self.total_width_mm - self.flange_top_mm - self.margin_mm + self.winding_end_trim_mm
        return max(self.winding_start_mm(), end)

    def effective_width(self) -> float:
        return max(0.0, self.winding_end_mm() - self.winding_start_mm())

    def turns_per_pass_calc(self) -> int:
        if self.wire_diameter_mm <= 0.0:
            return 1
        spacing = self.wire_diameter_mm * max(0.5, self.scatter_factor)
        return max(1, int(self.effective_width() / spacing))

    def turns_per_pass(self) -> int:
        return max(1, self.turns_per_pass_calc() + self.turns_per_pass_offset)

    def apply_preset(self, idx: int):
        if idx < 0 or idx >= len(BOBBIN_PRESETS):
            return
        p = BOBBIN_PRESETS[idx]
        self.total_width_mm = p["total"]
        self.flange_bottom_mm = p["flangeBot"]
        self.flange_top_mm = p["flangeTop"]
        self.winding_start_trim_mm = 0.0
        self.winding_end_trim_mm = 0.0
        self.wire_diameter_mm = p["wire"]
        self.turns_per_pass_offset = 0

    def to_dict(self) -> dict:
        return {
            "totalWidthMm": self.total_width_mm,
            "flangeBottomMm": self.flange_bottom_mm,
            "flangeTopMm": self.flange_top_mm,
            "marginMm": self.margin_mm,
            "windingStartTrimMm": self.winding_start_trim_mm,
            "windingEndTrimMm": self.winding_end_trim_mm,
            "wireDiameterMm": self.wire_diameter_mm,
            "turnsPerPassOffset": self.turns_per_pass_offset,
            "scatterFactor": self.scatter_factor,
        }

    def from_dict(self, d: dict):
        self.total_width_mm = float(d.get("totalWidthMm", self.total_width_mm))
        self.flange_bottom_mm = float(d.get("flangeBottomMm", self.flange_bottom_mm))
        self.flange_top_mm = float(d.get("flangeTopMm", self.flange_top_mm))
        self.margin_mm = float(d.get("marginMm", self.margin_mm))
        self.winding_start_trim_mm = float(d.get("windingStartTrimMm", 0.0))
        self.winding_end_trim_mm = float(d.get("windingEndTrimMm", 0.0))
        self.wire_diameter_mm = float(d.get("wireDiameterMm", self.wire_diameter_mm))
        self.turns_per_pass_offset = int(d.get("turnsPerPassOffset", 0))
        self.scatter_factor = max(0.5, min(4.0, float(d.get("scatterFactor", 1.0))))
