"""Motion planning and control for PickupWinder.

Lazy imports to avoid circular dependencies with transport module.
"""

__all__ = [
    "RampConfig",
    "compute_ramp_times",
    "AxisMotionConfig",
    "MultiAxisSegmentGenerator",
    "SpindleKinematics",
    "WindingPattern",
    "ScatterEngine",
    "SyncAxisConfig",
    "SynchronizedSegmentGenerator",
    "WindingEngine",
]


def __getattr__(name: str):
    if name == "RampConfig":
        from .ramp_config import RampConfig
        return RampConfig
    if name == "compute_ramp_times":
        from .ramp_config import compute_ramp_times
        return compute_ramp_times
    if name == "AxisMotionConfig":
        from .ramp import AxisMotionConfig
        return AxisMotionConfig
    if name == "MultiAxisSegmentGenerator":
        from .ramp import MultiAxisSegmentGenerator
        return MultiAxisSegmentGenerator
    if name == "SpindleKinematics":
        from .spindle_kinematics import SpindleKinematics
        return SpindleKinematics
    if name == "WindingPattern":
        from .winding_pattern import WindingPattern
        return WindingPattern
    if name == "ScatterEngine":
        from .scatter_engine import ScatterEngine
        return ScatterEngine
    if name == "SyncAxisConfig":
        from .synchronized_segment_generator import SyncAxisConfig
        return SyncAxisConfig
    if name == "SynchronizedSegmentGenerator":
        from .synchronized_segment_generator import SynchronizedSegmentGenerator
        return SynchronizedSegmentGenerator
    if name == "WindingEngine":
        from .engine import WindingEngine
        return WindingEngine
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return __all__
