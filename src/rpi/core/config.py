from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import json
from pathlib import Path
import re

@dataclass
class AppConfiguration:
    """Configuration parameters for the PickupWinder host application."""

    rpc_socket_path: str = "/tmp/winding.sock"
    spi_device: str = "/dev/spidev0.0"
    spi_speed_hz: int = 4_000_000
    spi_ready_gpio_chip: str | None = "/dev/gpiochip0"
    spi_ready_gpio_line: int | None = 17
    spi_ready_active_high: bool = True

    spindle_axis_id: int = 0
    spindle_steps_per_revolution: int = 200
    spindle_microstepping: int = 32
    spindle_invert_direction: bool = True
    spindle_max_speed_rpm: int = 1750
    # Unit: RPM/s (revolutions per minute gained per second).
    spindle_max_acceleration_rpm: float | None = 500
    # Unit: RPM/s (revolutions per minute lost per second).
    spindle_max_deceleration_rpm: float | None = None
    # Ramp durations for classic winding layers (seconds).
    # These control how quickly the spindle accelerates/decelerates on each
    # layer; they are fixed machine parameters, not per-program values.
    spindle_accel_s: float = 0.5
    spindle_decel_s: float = 0.5

    lateral_axis_id: int = 1
    lateral_steps_per_revolution: int = 96
    lateral_microstepping: int = 32
    lateral_invert_direction: bool = True
    lateral_max_rpm: int = 1000
    # Unit: mm/s² on traverse axis.
    lateral_max_acceleration_mm_per_s2: float | None = None
    # Unit: mm/s² on traverse axis.
    lateral_max_deceleration_mm_per_s2: float | None = None

    # Leadscrew/traverse pitch in mm per revolution for the lateral axis.
    # Used to compute steps/mm: steps_per_rev * microstepping / pitch_mm
    lateral_traverse_pitch_mm: float = 1.0
    # Optional explicit override for lateral steps-per-mm. If set, this
    # value takes precedence over the computed value.
    lateral_steps_per_mm_override: float | None = None
    # Soft travel window for the lateral axis relative to the homing zero.
    # None disables the corresponding bound.
    lateral_soft_limit_min_mm: float | None = 30.0
    lateral_soft_limit_max_mm: float | None = None
    # Physical travel length used to size the maximum homing approach move.
    # If unset, a conservative fallback based on motor revolutions is used.
    lateral_axis_length_mm: float | None = 130

    # Homing parameters
    home_before_start: bool = True
    lateral_homing_approach_rpm: float = 120.0
    lateral_homing_search_rpm: float = 20.0
    lateral_homing_backoff_steps: int | None = 6144

    # Target speed for post-homing and explicit start-position moves.
    # Unit: RPM on the lateral motor. Capped at lateral_max_rpm at runtime.
    lateral_target_speed: float = 120.0

    # Winding start position offset applied on top of lateral_soft_limit_min_mm.
    # Defines where the axis parks after homing and before winding starts.
    # Modifiable at runtime via RPC without re-homing.
    # Can be negative (start before soft_limit_min) or positive (start after).
    # Constraints:
    #   soft_limit_min_mm + axis_offset_mm >= soft_limit_min_mm (or unbounded if None)
    #   soft_limit_min_mm + axis_offset_mm <= soft_limit_max_mm (if set)
    lateral_axis_offset_mm: float = 0.0

    def __post_init__(self) -> None:
        if self.spindle_steps_per_revolution <= 0:
            raise ValueError("spindle_steps_per_revolution must be positive")
        if self.spindle_microstepping <= 0:
            raise ValueError("spindle_microstepping must be positive")
        if self.spindle_max_speed_rpm <= 0:
            raise ValueError("spindle_max_speed_rpm must be positive")
        if self.lateral_steps_per_revolution <= 0:
            raise ValueError("lateral_steps_per_revolution must be positive")
        if self.lateral_microstepping <= 0:
            raise ValueError("lateral_microstepping must be positive")
        if self.lateral_max_rpm <= 0:
            raise ValueError("lateral_max_rpm must be positive")
        if self.lateral_target_speed <= 0:
            raise ValueError("lateral_target_speed must be positive")
        if self.lateral_traverse_pitch_mm <= 0.0:
            raise ValueError("lateral_traverse_pitch_mm must be positive")
        if self.spindle_accel_s < 0.0:
            raise ValueError("spindle_accel_s must be >= 0")
        if self.spindle_decel_s < 0.0:
            raise ValueError("spindle_decel_s must be >= 0")
        if self.lateral_axis_length_mm is not None and self.lateral_axis_length_mm <= 0.0:
            raise ValueError("lateral_axis_length_mm must be positive")
        if (
            self.lateral_steps_per_mm_override is not None
            and self.lateral_steps_per_mm_override <= 0.0
        ):
            raise ValueError("lateral_steps_per_mm_override must be positive")
        if not re.fullmatch(r"/dev/spidev\d+\.\d+", self.spi_device):
            raise ValueError("spi_device must be in the form /dev/spidev<bus>.<device>")
        if self.spi_ready_gpio_chip is not None and not str(self.spi_ready_gpio_chip).startswith("/dev/gpiochip"):
            raise ValueError("spi_ready_gpio_chip must be in the form /dev/gpiochipN")
        if self.spi_ready_gpio_line is not None and self.spi_ready_gpio_line < 0:
            raise ValueError("spi_ready_gpio_line must be non-negative")
        if (
            self.lateral_soft_limit_min_mm is not None
            and self.lateral_soft_limit_max_mm is not None
            and self.lateral_soft_limit_max_mm <= self.lateral_soft_limit_min_mm
        ):
            raise ValueError(
                "lateral_soft_limit_max_mm must be greater than lateral_soft_limit_min_mm"
            )

        start_mm = (self.lateral_soft_limit_min_mm or 0.0) + self.lateral_axis_offset_mm

        if ( self.lateral_soft_limit_min_mm is not None and start_mm < self.lateral_soft_limit_min_mm ):
            raise ValueError(
                f"lateral_soft_limit_min_mm ({self.lateral_soft_limit_min_mm}) "
                f"+ lateral_axis_offset_mm ({self.lateral_axis_offset_mm}) "
                f"= {start_mm:.3f} mm is below lateral_soft_limit_min_mm "
                f"({self.lateral_soft_limit_min_mm})"
            )

        if ( self.lateral_soft_limit_max_mm is not None and start_mm > self.lateral_soft_limit_max_mm):
            raise ValueError(
                f"lateral_soft_limit_min_mm ({self.lateral_soft_limit_min_mm}) "
                f"+ lateral_axis_offset_mm ({self.lateral_axis_offset_mm}) "
                f"= {start_mm:.3f} mm exceeds lateral_soft_limit_max_mm "
                f"({self.lateral_soft_limit_max_mm})"
            )



    @property
    def lateral_steps_per_mm(self) -> float:
        """Return lateral axis steps per millimetre.

        Computed as: (steps_per_revolution * microstepping) / traverse_pitch_mm.
        If `lateral_steps_per_mm_override` is provided, it is returned instead.
        """
        if self.lateral_steps_per_mm_override is not None:
            return float(self.lateral_steps_per_mm_override)
        return (self.lateral_steps_per_revolution * self.lateral_microstepping) / float(self.lateral_traverse_pitch_mm)

    @property
    def lateral_soft_limit_min_steps(self) -> int | None:
        if self.lateral_soft_limit_min_mm is None:
            return None
        return int(round(float(self.lateral_soft_limit_min_mm) * self.lateral_steps_per_mm))

    @property
    def lateral_soft_limit_max_steps(self) -> int | None:
        if self.lateral_soft_limit_max_mm is None:
            return None
        return int(round(float(self.lateral_soft_limit_max_mm) * self.lateral_steps_per_mm))

    @property
    def lateral_axis_length_steps(self) -> int | None:
        if self.lateral_axis_length_mm is None:
            return None
        return int(round(float(self.lateral_axis_length_mm) * self.lateral_steps_per_mm))

    @property
    def spindle_max_acceleration_steps_per_s2(self) -> float:
        """Compute spindle acceleration in steps/s^2.

        Uses `spindle_max_acceleration_rpm` (RPM/s) if provided.
        Otherwise returns a safe default.
        """
        steps_per_rev = self.spindle_steps_per_revolution * self.spindle_microstepping
        if self.spindle_max_acceleration_rpm is not None:
            return (self.spindle_max_acceleration_rpm / 60.0) * steps_per_rev
        return 100_000.0

    @property
    def spindle_max_deceleration_steps_per_s2(self) -> float:
        """Compute spindle deceleration in steps/s^2.

        Uses `spindle_max_deceleration_rpm` (RPM/s) if provided. Otherwise falls back
        to the configured spindle acceleration limit.
        """
        steps_per_rev = self.spindle_steps_per_revolution * self.spindle_microstepping
        if self.spindle_max_deceleration_rpm is not None:
            return (self.spindle_max_deceleration_rpm / 60.0) * steps_per_rev
        return self.spindle_max_acceleration_steps_per_s2

    @property
    def lateral_max_acceleration_steps_per_s2(self) -> float:
        """Compute lateral acceleration in steps/s^2.

        Uses `lateral_max_acceleration_mm_per_s2` if provided.
        Otherwise returns a safe default.
        """
        if self.lateral_max_acceleration_mm_per_s2 is not None:
            return float(self.lateral_max_acceleration_mm_per_s2) * self.lateral_steps_per_mm
        return 100_000.0

    @property
    def lateral_max_deceleration_steps_per_s2(self) -> float:
        """Compute lateral deceleration in steps/s^2.

        Uses `lateral_max_deceleration_mm_per_s2` if provided.
        Otherwise falls back to the configured lateral acceleration limit.
        """
        if self.lateral_max_deceleration_mm_per_s2 is not None:
            return float(self.lateral_max_deceleration_mm_per_s2) * self.lateral_steps_per_mm
        return self.lateral_max_acceleration_steps_per_s2

    @property
    def lateral_start_position_mm(self) -> float:
        """Winding start position = soft_limit_min_mm + axis_offset_mm."""
        return (self.lateral_soft_limit_min_mm or 0.0) + self.lateral_axis_offset_mm

    @property
    def lateral_start_position_steps(self) -> int:
        """Winding start position converted to steps."""
        return int(round(self.lateral_start_position_mm * self.lateral_steps_per_mm))


class ConfigurationManager:

    def __init__(self, config_file_path: str | Path):
        self._config_file_path = Path(config_file_path)
        self.active_configuration = AppConfiguration()

    @staticmethod
    def _configuration_field_names() -> set[str]:
        return {field.name for field in fields(AppConfiguration)}

    @classmethod
    def _configuration_from_mapping(cls, payload: dict) -> AppConfiguration:
        if not isinstance(payload, dict):
            raise ValueError("configuration payload must be a JSON object")
        allowed_fields = cls._configuration_field_names()
        filtered_payload = {
            key: value
            for key, value in payload.items()
            if key in allowed_fields
        }
        return AppConfiguration(**filtered_payload)


    def load_configuration(self) -> AppConfiguration:
        with self._config_file_path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        self.active_configuration = self._configuration_from_mapping(payload)
        return self.active_configuration

    def save_configuration(
        self,
        configuration: AppConfiguration | None = None,
    ) -> AppConfiguration:
        config = configuration or self.active_configuration
        self._config_file_path.parent.mkdir(parents=True, exist_ok=True)
        with self._config_file_path.open("w", encoding="utf-8") as handle:
            json.dump(asdict(config), handle, indent=2, sort_keys=True)
        self.active_configuration = config
        return config

    def get_saved_configuration(self) -> AppConfiguration | None:
        if not self._config_file_path.exists():
            return None
        with self._config_file_path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        return self._configuration_from_mapping(payload)

    def get_active_configuration(self) -> AppConfiguration:
        return self.active_configuration

    # Return RPC Socket path
    def get_rpc_socket_path(self) -> str:
        return self.active_configuration.rpc_socket_path

    # Return SPI Config Tuples (dev, speed)
    def get_spi_device(self) -> tuple[str, int]:
        return (
            self.active_configuration.spi_device,
            self.active_configuration.spi_speed_hz,
        )