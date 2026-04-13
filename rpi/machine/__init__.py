"""machine — High-level winding business logic.

This package contains:
- coil_winder: Full winding program orchestration
- tensioner: Wire tension PID control loop
- homing: Axis homing sequences
"""

from .coil_winder import CoilWinder, WindingProgram, WindingState, ProgressCallback
from .tensioner import Tensioner, TensionConfig
from .homing import home_axis, home_all, HomingConfig, LATERAL_HOME

__all__ = [
    "CoilWinder",
    "WindingProgram",
    "WindingState",
    "ProgressCallback",
    "Tensioner",
    "TensionConfig",
    "home_axis",
    "home_all",
    "HomingConfig",
    "LATERAL_HOME",
]
