import os
import sys

root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "rpi"))
if root not in sys.path:
    sys.path.insert(0, root)

import pytest

from motion.trapezoidal_profile import TrapezoidalMotionProfile
from motion.spindle_kinematics import SpindleKinematics
from motion.winding_pattern import WindingPattern
from motion.scatter_engine import ScatterEngine
from motion.move import WoundMove
from motion.syncrhonized_segment_generator import SyncAxisConfig
from winding.program import WindingProgram


@pytest.mark.parametrize(
    "time_s, expected_turns",
    [
        (2.5, 18.75),
        (3.0, 20.0),
    ],
)
def test_trapezoidal_turns_at_deceleration(time_s: float, expected_turns: float):
    profile = TrapezoidalMotionProfile(
        start_rpm=0.0,
        target_rpm=600.0,
        accel_s=1.0,
        cruise_s=1.0,
        decel_s=1.0,
    )

    assert pytest.approx(profile.turns_at(time_s), rel=1e-6) == expected_turns


def test_spindle_kinematics_validates_after_dataclass_init():
    engine = SpindleKinematics(
        target_rpm=1000.0,
        start_rpm=0.0,
        accel_s=1.0,
        cruise_s=1.0,
        decel_s=1.0,
    )

    assert engine.total_duration == 3.0
    assert pytest.approx(engine.turns_at(3.0), rel=1e-6) == 33.333333333333336

    with pytest.raises(ValueError):
        SpindleKinematics(
            target_rpm=1000.0,
            start_rpm=0.0,
            accel_s=-0.1,
            cruise_s=1.0,
            decel_s=1.0,
        )


@pytest.mark.parametrize(
    "bobbin_width_mm, turns_per_mm",
    [
        (0.0, 10.0),
        (10.0, 0.0),
        (-5.0, 10.0),
    ],
)
def test_winding_pattern_rejects_invalid_geometry(bobbin_width_mm: float, turns_per_mm: float):
    with pytest.raises(ValueError):
        WindingPattern(bobbin_width_mm=bobbin_width_mm, turns_per_mm=turns_per_mm)


def test_winding_program_layer_duration_computes_from_geometry():
    program = WindingProgram(
        name="test",
        num_layers=1,
        spindle_rpm=1200.0,
        layer_pitch_mm=0.5,
        wire_diameter_mm=0.25,
        bobbin_width_mm=10.0,
    )

    assert program.turns_per_mm == 2.0
    assert pytest.approx(program.layer_duration_s(), rel=1e-6) == 2.0
    snapshot = program.snapshot()
    assert snapshot["bobbin_width_mm"] == 10.0
    assert snapshot["turns_per_mm"] == 2.0
    assert "scatter_amplitude_mm" in snapshot
    assert "layer_duration_s" in snapshot


def test_wound_move_rejects_duplicate_axis_indices():
    kinematics = SpindleKinematics(
        target_rpm=1000.0,
        accel_s=0.5,
        cruise_s=1.0,
        decel_s=0.5,
    )
    pattern = WindingPattern(bobbin_width_mm=15.0, turns_per_mm=10.0)
    scatter = ScatterEngine(amplitude_mm=0.1, damping_margin_mm=1.0)
    spindle_cfg = SyncAxisConfig(axis_index=0, steps_per_unit=6400.0)
    traverse_cfg = SyncAxisConfig(axis_index=0, steps_per_unit=1000.0)

    with pytest.raises(ValueError):
        WoundMove(
            name="bad_layer",
            kinematics=kinematics,
            pattern=pattern,
            scatter=scatter,
            spindle_cfg=spindle_cfg,
            traverse_cfg=traverse_cfg,
        )
