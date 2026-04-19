from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class WindingProgram:
    """
    Describes a complete winding operation.

    The engine executes layers in order, alternating traverse direction
    on each layer. Each layer consists of:
      - spindle rotating at spindle_rpm for the duration of the layer
      - lateral axis traversing layer_pitch_mm * num_passes_per_layer
        at a speed derived from spindle_rpm and the wire geometry

    Fields:
      name              Human-readable program name
      num_layers        Total number of winding layers
      spindle_rpm       Spindle rotation speed in RPM
      layer_pitch_mm    Lateral advance per spindle revolution (mm)
      wire_diameter_mm  Wire diameter used to compute traverse speed
      accel_s           Acceleration time for both axes (seconds)
      decel_s           Deceleration time for both axes (seconds)
      spindle_axis_id   Axis ID of the spindle (default 0)
      lateral_axis_id   Axis ID of the lateral traverse (default 1)
      lateral_steps_per_mm  Steps per mm on the lateral axis
      home_before_start     If True, home lateral axis before starting
      home_approach_rpm     RPM for homing approach phase
      home_search_rpm       RPM for homing search phase
      home_backoff_steps    Steps to back off after first endstop contact
    """
    name: str
    num_layers: int
    spindle_rpm: float
    layer_pitch_mm: float
    wire_diameter_mm: float
    bobbin_width_mm: float = 15.0
    scatter_amplitude_mm: float = 0.0
    scatter_damping_margin_mm: float = 0.0
    accel_s: float = 0.5
    decel_s: float = 0.5
    spindle_axis_id: int = 0
    lateral_axis_id: int = 1
    lateral_steps_per_mm: float = 200.0 * 32.0 / 8.0  # 200step * 32µstep / 8mm/rev
    home_before_start: bool = True
    home_approach_rpm: float = 100.0
    home_search_rpm: float = 20.0
    home_backoff_steps: int = 3200

    def validate(self) -> None:
        """Raise ValueError if any field is out of range."""
        if self.num_layers < 1:
            raise ValueError("num_layers must be >= 1")
        if self.spindle_rpm <= 0.0:
            raise ValueError("spindle_rpm must be positive")
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
        if self.accel_s < 0.0 or self.decel_s < 0.0:
            raise ValueError("accel_s and decel_s must be >= 0")
        if self.lateral_steps_per_mm <= 0.0:
            raise ValueError("lateral_steps_per_mm must be positive")

    @property
    def turns_per_mm(self) -> float:
        return 1.0 / self.layer_pitch_mm

    def lateral_rpm_for_layer(self) -> float:
        """
        Compute the lateral traverse speed in RPM needed to advance
        layer_pitch_mm per spindle revolution.

        lateral_speed_mm_s = spindle_rps * layer_pitch_mm
        lateral_rpm = lateral_speed_mm_s * 60 / (2π * lateral_radius_mm)

        Since we work in steps/mm directly:
        lateral_hz = spindle_hz * layer_pitch_mm * lateral_steps_per_mm
        lateral_rpm = lateral_hz / (200 * 32) * 60
        """
        spindle_hz = self.spindle_rpm / 60.0
        lateral_hz = spindle_hz * self.layer_pitch_mm * self.lateral_steps_per_mm
        return (lateral_hz / (200.0 * 32.0)) * 60.0

    def layer_duration_s(self) -> float:
        """
        Duration of a full winding layer in seconds.

        A single layer is defined as a forward/backward pass across the bobbin
        width. The total spindle turns required for one layer are:

            total_turns = 2 * bobbin_width_mm * turns_per_mm

        The layer duration is therefore the total spindle turns divided by
        spindle revolutions per second.
        """
        spindle_rps = self.spindle_rpm / 60.0
        if spindle_rps <= 0.0:
            raise ValueError("spindle_rpm must be positive to compute layer duration")
        total_turns = 2.0 * self.bobbin_width_mm * self.turns_per_mm
        return total_turns / spindle_rps

    def snapshot(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "num_layers": self.num_layers,
            "spindle_rpm": self.spindle_rpm,
            "layer_pitch_mm": self.layer_pitch_mm,
            "wire_diameter_mm": self.wire_diameter_mm,
            "bobbin_width_mm": self.bobbin_width_mm,
            "turns_per_mm": self.turns_per_mm,
            "scatter_amplitude_mm": self.scatter_amplitude_mm,
            "scatter_damping_margin_mm": self.scatter_damping_margin_mm,
            "accel_s": self.accel_s,
            "decel_s": self.decel_s,
            "lateral_rpm": self.lateral_rpm_for_layer(),
            "layer_duration_s": self.layer_duration_s(),
        }
