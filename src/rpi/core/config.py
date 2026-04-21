from dataclasses import dataclass
from typing import Optional

@dataclass
class AppConfiguration:
    """Configuration parameters for the PickupWinder host application."""

    rpc_socket_path: str = "/tmp/pickup_winder_rpc.sock"
    spi_device: str = "/dev/spidev0.0"
    spi_speed_hz: int = 4_000_000

    spindle_axis_id: int = 0
    spindle_steps_per_revolution: int = 200
    spindle_microstepping: int = 32
    spindle_invert_direction: bool = False
    spindle_max_speed_rpm: int = 1750
    # Unit: RPM/s (revolutions per minute gained per second).
    spindle_max_acceleration_rpm: Optional[float] = 500
    # Unit: RPM/s (revolutions per minute lost per second).
    spindle_max_deceleration_rpm: Optional[float] = None

    lateral_axis_id: int = 1
    lateral_steps_per_revolution: int = 200
    lateral_microstepping: int = 32
    lateral_invert_direction: bool = False
    lateral_max_rpm: int = 1000     
    # Unit: mm/s² on traverse axis.
    lateral_max_acceleration_mm_per_s2: Optional[float] = None
    # Unit: mm/s² on traverse axis.
    lateral_max_deceleration_mm_per_s2: Optional[float] = None
    
    # Leadscrew/traverse pitch in mm per revolution for the lateral axis.
    # Used to compute steps/mm: steps_per_rev * microstepping / pitch_mm
    lateral_traverse_pitch_mm: float = 1.0
    # Optional explicit override for lateral steps-per-mm. If set, this
    # value takes precedence over the computed value.
    lateral_steps_per_mm_override: Optional[float] = None
    # Soft travel window for the lateral axis relative to the homing zero.
    # ``None`` disables the corresponding bound.
    lateral_soft_limit_min_mm: Optional[float] = 0.0
    lateral_soft_limit_max_mm: Optional[float] = None

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
    def lateral_soft_limit_min_steps(self) -> Optional[int]:
        if self.lateral_soft_limit_min_mm is None:
            return None
        return int(round(float(self.lateral_soft_limit_min_mm) * self.lateral_steps_per_mm))

    @property
    def lateral_soft_limit_max_steps(self) -> Optional[int]:
        if self.lateral_soft_limit_max_mm is None:
            return None
        return int(round(float(self.lateral_soft_limit_max_mm) * self.lateral_steps_per_mm))

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


class ConfigurationManager:
    
    def __init__(self,config_file_path):
        self._config_file_path = config_file_path
        self.active_configuration = AppConfiguration()


    def load_configuration(self):
        pass

    def save_configration(self):
        pass

    def get_saved_configuration(self):
        pass

    def get_activate_configuration(self):
        pass

    # Return RPC Socket path
    def get_rpc_socket_path(self) -> str:
        return self.active_configuration.rpc_socket_path

    # Return SPI Config Tuples (dev, speed)
    def get_spi_device(self) -> str:
        return (self.active_configuration.spi_device,self.active_configuration.spi_speed_hz)