"""Motion planning and control for PickupWinder.

Lazy imports to avoid circular dependencies with transport module.
"""

__all__ = [
    "RampConfig",
    "compute_ramp_times",
    "AxisMotionConfig",
    "MultiAxisSegmentGenerator",
    "SpindleKinematics",
]


def __getattr__(name: str):
    if name == "RampConfig":
        from .ramp_config import RampConfig
        return RampConfig
    if name == "compute_ramp_times":
        from .ramp_config import compute_ramp_times
        return compute_ramp_times
    if name == "AxisMotionConfig":
        from .multi_axis_segment_generator import AxisMotionConfig
        return AxisMotionConfig
    if name == "MultiAxisSegmentGenerator":
        from .multi_axis_segment_generator import MultiAxisSegmentGenerator
        return MultiAxisSegmentGenerator
    if name == "SpindleKinematics":
        from .spindle_kinematics import SpindleKinematics
        return SpindleKinematics
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return __all__
