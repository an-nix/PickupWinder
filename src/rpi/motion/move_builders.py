"""Utility functions for constructing common move types.

Centralises the repeated RampMove/RampMoveConfig/AxisMotionConfig/RampConfig
construction pattern so call sites stay concise.
"""
from __future__ import annotations

from motion import AxisMotionConfig, RampConfig
from motion.move import RampMove, RampMoveConfig


def build_jog_move(
    *,
    name: str,
    axis_id: int,
    steps: int,
    steps_per_rev: int,
    rpm: float,
    reverse: bool = False,
) -> RampMove:
    """Build a single-axis jog RampMove from step count and RPM.

    The trapezoidal ramp times are derived automatically:
      accel_s = decel_s = min(0.15 s, 20 % of total_s)
      cruise_s = max(total_s − 0.3 s, 0)
    """
    total_s = (steps / float(steps_per_rev)) / (rpm / 60.0)
    return RampMove(
        name=name,
        config=RampMoveConfig(
            axis_configs=[
                AxisMotionConfig(
                    axis_id=axis_id,
                    ramp=RampConfig(
                        axis_id=axis_id,
                        steps_per_rev=steps_per_rev,
                        target_rpm=rpm,
                        accel_s=min(0.15, total_s * 0.2),
                        cruise_s=max(total_s - 0.3, 0.0),
                        decel_s=min(0.15, total_s * 0.2),
                        reverse_direction=reverse,
                    ),
                )
            ],
        ),
    )
