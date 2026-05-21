from __future__ import annotations

import dataclasses
from dataclasses import dataclass, fields
from typing import Any


@dataclass(slots=True)
class WindingProgram:
    """
    Persistent recipe describing WHAT to wind.

    Machine settings (axis IDs, ramp times, homing parameters) and
    execution context (spindle RPM) live in ``AppConfiguration`` and
    ``SessionParams`` respectively — not here.

    Fields:
      name              Human-readable program name
      num_layers        Total number of winding layers
      layer_pitch_mm    Lateral advance per spindle revolution (mm)
      wire_diameter_mm  Wire diameter (mm)
      bobbin_width_mm   Physical winding window width (mm)
      scatter_*         Scatter-winding parameters
    """
    name: str
    num_layers: int
    layer_pitch_mm: float
    wire_diameter_mm: float
    program_id: str | None = None
    bobbin_width_mm: float = 15.0
    scatter_amplitude_mm: float = 0.0
    scatter_damping_margin_mm: float = 0.0
    scatter_freq1: float = 1.0
    scatter_freq2: float = 1.618
    revision: int = 1
    created_at: str | None = None
    updated_at: str | None = None

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> WindingProgram:
        if not isinstance(payload, dict):
            raise ValueError("program payload must be an object")

        known_fields = {f.name for f in fields(cls)}
        normalized: dict[str, Any] = {}

        for key, value in payload.items():
            # Alias RPC: "id" → "program_id"
            if key == "id":
                normalized["program_id"] = value
            elif key in known_fields:
                normalized[key] = value
            # Computed, obsolete, or hardware-only fields are silently ignored:
            # turns_per_mm, layer_duration_s, lateral_steps_per_mm

        return cls(**normalized)

    def to_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)

    def validate(self) -> None:
        """Raise ValueError if any field is out of range."""
        if not self.name.strip():
            raise ValueError("name must not be empty")
        if self.program_id is not None and not str(self.program_id).strip():
            raise ValueError("program_id must not be empty when provided")
        if self.num_layers < 1:
            raise ValueError("num_layers must be >= 1")
        if self.layer_pitch_mm <= 0.0:
            raise ValueError("layer_pitch_mm must be positive")
        if self.wire_diameter_mm <= 0.0:
            raise ValueError("wire_diameter_mm must be positive")
        if self.bobbin_width_mm <= 0.0:
            raise ValueError("bobbin_width_mm must be positive")
        if self.scatter_amplitude_mm < 0.0:
            raise ValueError("scatter_amplitude_mm must be >= 0")
        if self.scatter_damping_margin_mm < 0.0:
            raise ValueError("scatter_damping_margin_mm must be >= 0")
        if self.scatter_freq1 <= 0.0:
            raise ValueError("scatter_freq1 must be positive")
        if self.scatter_freq2 <= 0.0:
            raise ValueError("scatter_freq2 must be positive")
        if self.revision < 1:
            raise ValueError("revision must be >= 1")

    @property
    def turns_per_mm(self) -> float:
        return 1.0 / self.layer_pitch_mm

    def layer_duration_s(self, spindle_rpm: float) -> float:
        """
        Duration of a full winding layer in seconds at the given RPM.

        A single layer is a forward/backward pass across the bobbin width:

            total_turns = 2 * bobbin_width_mm * turns_per_mm

        The layer duration is the total spindle turns divided by
        spindle revolutions per second.
        """
        spindle_rps = spindle_rpm / 60.0
        if spindle_rps <= 0.0:
            raise ValueError("spindle_rpm must be positive to compute layer duration")
        total_turns = 2.0 * self.bobbin_width_mm * self.turns_per_mm
        return total_turns / spindle_rps

    def total_turns(self) -> float:
        """Return the total spindle turns for the full classic program."""
        return float(self.num_layers) * 2.0 * self.bobbin_width_mm * self.turns_per_mm

    def snapshot(self) -> dict[str, Any]:
        snapshot = self.to_dict()
        snapshot["id"] = self.program_id
        snapshot["turns_per_mm"] = self.turns_per_mm
        snapshot["total_turns"] = self.total_turns()
        return snapshot
