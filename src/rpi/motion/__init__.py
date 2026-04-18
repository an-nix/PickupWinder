"""Motion planning and control for PickupWinder.

Lazy imports to avoid circular dependencies with transport module.
"""


def __getattr__(name: str):
    if name == "RampConfig":
        from .ramp import RampConfig
        return RampConfig
    if name == "AxisMotionConfig":
        from .ramp import AxisMotionConfig
        return AxisMotionConfig
    if name == "MultiAxisSegmentGenerator":
        from .ramp import MultiAxisSegmentGenerator
        return MultiAxisSegmentGenerator
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return ["RampConfig", "AxisMotionConfig", "MultiAxisSegmentGenerator"]