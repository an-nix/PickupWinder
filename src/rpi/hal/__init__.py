"""HAL package — Hardware Abstraction Layer.

All ESP32 communication goes through this layer.
No other module should import spidev or know about SPI frames.
"""

from .protocol import (
    AxisId,
    CmdFlags,
    CmdOpcode,
    EventType,
    MachineStatus,
    AxisStatus as AxisStatusFrame,
    StatusFlags,
)
from .spi_transport import SpiTransport
from .esp32_controller import ESP32Controller
from .axis import Axis, AxisConfig

__all__ = [
    "AxisId",
    "CmdFlags",
    "CmdOpcode",
    "EventType",
    "MachineStatus",
    "AxisStatusFrame",
    "StatusFlags",
    "SpiTransport",
    "ESP32Controller",
    "Axis",
    "AxisConfig",
]
