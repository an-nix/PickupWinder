from __future__ import annotations

import dataclasses
import math
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
      name                       Human-readable program name
      target_turns               Total number of spindle turns to wind. The number of
                                 layer passes (num_layers) is derived automatically from
                                 bobbin_width_mm and layer_pitch_mm.
      layer_pitch_mm             Lateral advance per spindle revolution (mm)
      wire_diameter_mm           Wire diameter (mm)
      bobbin_width_mm            Interior winding window width (mm), i.e. flatwork-to-flatwork
      flatwork_thickness_mm      Bobbin flatwork/cheek thickness (mm). Added to soft_limit_min to
                                 compute winding start before applying start clearance.
      window_start_clearance_mm  Safety gap between flatwork top and first wire turn (mm).
                                 Overrides AppConfiguration.window_start_clearance_mm when set.
      window_end_clearance_mm    Safety reduction at the far end of the winding window (mm).
                                 Overrides AppConfiguration.window_end_clearance_mm when set.
      scatter_*                  Scatter-winding parameters
    """
    name: str
    target_turns: int
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
    flatwork_thickness_mm: float = 0.0
    window_start_clearance_mm: float | None = None
    window_end_clearance_mm: float | None = None

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
            # num_layers (computed from target_turns + geometry), turns_per_mm,
            # layer_duration_s, lateral_steps_per_mm

        return cls(**normalized)

    def to_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)

    def validate(self) -> None:
        """Raise ValueError if any field is out of range."""
        if not self.name.strip():
            raise ValueError("name must not be empty")
        if self.program_id is not None and not str(self.program_id).strip():
            raise ValueError("program_id must not be empty when provided")
        if self.target_turns < 1:
            raise ValueError("target_turns must be >= 1")
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
        if self.flatwork_thickness_mm < 0.0:
            raise ValueError("flatwork_thickness_mm must be >= 0")
        if self.window_start_clearance_mm is not None and self.window_start_clearance_mm < 0.0:
            raise ValueError("window_start_clearance_mm must be >= 0 when specified")
        if self.window_end_clearance_mm is not None and self.window_end_clearance_mm < 0.0:
            raise ValueError("window_end_clearance_mm must be >= 0 when specified")
        if (
            self.window_end_clearance_mm is not None
            and self.window_end_clearance_mm >= self.bobbin_width_mm
        ):
            raise ValueError("window_end_clearance_mm must be less than bobbin_width_mm")

    @property
    def turns_per_mm(self) -> float:
        return 1.0 / self.layer_pitch_mm

    @property
    def num_layers(self) -> int:
        """Number of full forward/backward passes to reach *target_turns*.

        Computed as::

            ceil(target_turns / (2 * bobbin_width_mm * turns_per_mm))

        Uses ``bobbin_width_mm`` (not the effective width with end_clearance) so
        the property stays self-contained without a config dependency.
        """
        turns_per_pass = 2.0 * self.bobbin_width_mm * self.turns_per_mm
        return max(1, math.ceil(self.target_turns / turns_per_pass))

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
        """Return the total target spindle turns for the program."""
        return float(self.target_turns)

    def effective_winding_width_mm(self, *, default_end_clearance_mm: float) -> float:
        """Effective lateral traversal width after applying end clearance.

        Uses ``window_end_clearance_mm`` if set on this program, otherwise
        falls back to *default_end_clearance_mm* from the machine config.
        """
        end_clearance = (
            self.window_end_clearance_mm
            if self.window_end_clearance_mm is not None
            else default_end_clearance_mm
        )
        return max(0.0, self.bobbin_width_mm - end_clearance)

    def effective_window(
        self,
        *,
        soft_limit_min_mm: float,
        machine_offset_mm: float,
        default_start_clearance_mm: float,
        default_end_clearance_mm: float,
    ) -> tuple[float, float]:
        """Compute the absolute winding window [window_low, window_high].

        Chain::

            window_low  = soft_limit_min + machine_offset + flatwork_thickness + start_clearance
            window_high = window_low + effective_winding_width

        *machine_offset_mm* is ``AppConfiguration.lateral_axis_offset_mm`` — a
        machine-level fine-tuning offset that is ``0.0`` when *soft_limit_min*
        is perfectly at the plateau edge.

        Per-program clearance overrides are applied when set; otherwise the
        *default_*_clearance_mm* values from the machine config are used.
        """
        start_clearance = (
            self.window_start_clearance_mm
            if self.window_start_clearance_mm is not None
            else default_start_clearance_mm
        )
        window_low = (
            soft_limit_min_mm
            + machine_offset_mm
            + self.flatwork_thickness_mm
            + start_clearance
        )
        window_high = window_low + self.effective_winding_width_mm(
            default_end_clearance_mm=default_end_clearance_mm
        )
        return window_low, window_high

    def snapshot(self) -> dict[str, Any]:
        snapshot = self.to_dict()
        snapshot["id"] = self.program_id
        snapshot["num_layers"] = self.num_layers
        snapshot["turns_per_mm"] = self.turns_per_mm
        snapshot["total_turns"] = self.total_turns()
        return snapshot
