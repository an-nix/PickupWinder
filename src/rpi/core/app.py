"""Legacy core application stub.

This module formerly contained a standalone host application driver for
PickupWinder. The implementation has been deprecated in favor of
`WindingEngine` and JSON-RPC orchestration.
"""

from __future__ import annotations

from .config import AppConfiguration


class WinderApp:
    """Legacy compatibility stub for the deprecated PickupWinder host app."""

    def __init__(self, config: AppConfiguration | None = None) -> None:
        del config
        raise RuntimeError(
            "WinderApp is deprecated. Use WindingEngine via winding_main.py instead."
        )

    def endstop_status(self) -> dict[str, Any]:
        """Return current endstop state from a live status poll."""
        if self.transport is None:
            raise RuntimeError("Application transport is not started")
        status = self.transport.get_status()
        return {
            "lateral_endstop_state": status.lateral_endstop_state,
            "endstop_armed_mask": status.endstop_armed_mask,
        }

    def config_snapshot(self) -> dict[str, Any]:
        return {
            "spi_device": self.config.spi_device,
            "spi_speed_hz": self.config.spi_speed_hz,
            "spindle_max_speed_rpm": self.config.spindle_max_speed_rpm,
            "spindle_max_acceleration_rpm": self.config.spindle_max_acceleration_rpm,
            "spindle_max_deceleration_rpm": self.config.spindle_max_deceleration_rpm,
            "lateral_max_rpm": self.config.lateral_max_rpm,
            "lateral_max_acceleration_mm_per_s2": self.config.lateral_max_acceleration_mm_per_s2,
            "lateral_max_deceleration_mm_per_s2": self.config.lateral_max_deceleration_mm_per_s2,
            "lateral_traverse_pitch_mm": self.config.lateral_traverse_pitch_mm,
        }
