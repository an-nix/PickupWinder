import os
import sys
from importlib import import_module

root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "rpi"))
if root not in sys.path:
    sys.path.insert(0, root)

adaptive = import_module("winding.adaptive")

AdaptivePlanningSnapshot = adaptive.AdaptivePlanningSnapshot
AdaptiveWindingRuntime = adaptive.AdaptiveWindingRuntime
AdaptiveWindingSessionConfig = adaptive.AdaptiveWindingSessionConfig
WindingWindow = adaptive.WindingWindow
awg_to_diameter_mm = adaptive.awg_to_diameter_mm
plan_next_chunk = adaptive.plan_next_chunk


def test_awg_42_diameter_is_close_to_nominal_value():
    assert abs(awg_to_diameter_mm(42) - 0.0635) < 0.001


def test_plan_next_chunk_accelerates_when_far_from_edge():
    snapshot = AdaptivePlanningSnapshot(
        total_turns=1000.0,
        completed_turns=0.0,
        current_rpm=0.0,
        target_rpm=600.0,
        turns_per_mm=12.0,
        chunk_time_s=0.25,
        current_guide_mm=0.0,
        direction_sign=1,
        window=WindingWindow(0.0, 10.0),
        pause_requested=False,
        pause_at_turn=None,
    )

    plan = plan_next_chunk(
        snapshot,
        spindle_accel_rps2=20.0,
        spindle_decel_rps2=20.0,
    )

    assert plan is not None
    assert plan.reason == "accel"
    assert plan.end_rpm > plan.start_rpm
    assert plan.turns_delta > 0.0


def test_plan_next_chunk_brakes_to_zero_at_edge():
    snapshot = AdaptivePlanningSnapshot(
        total_turns=1000.0,
        completed_turns=0.0,
        current_rpm=600.0,
        target_rpm=600.0,
        turns_per_mm=10.0,
        chunk_time_s=0.25,
        current_guide_mm=1.9,
        direction_sign=1,
        window=WindingWindow(0.0, 2.0),
        pause_requested=False,
        pause_at_turn=None,
    )

    plan = plan_next_chunk(
        snapshot,
        spindle_accel_rps2=20.0,
        spindle_decel_rps2=50.0,
    )

    assert plan is not None
    assert plan.reason == "edge"
    assert plan.end_rpm == 0.0
    assert plan.stop_at_end is True
    assert plan.reached_edge is True


def test_runtime_window_update_requests_fraction_preserving_reposition():
    runtime = AdaptiveWindingRuntime(
        AdaptiveWindingSessionConfig(
            name="test",
            total_turns=500.0,
            target_rpm=500.0,
            window_low_mm=0.0,
            window_high_mm=10.0,
            wire_diameter_mm=0.0635,
        ),
        spindle_steps_per_rev=6400,
        lateral_steps_per_mm=800.0,
    )
    runtime.set_repositioned_guide(5.0, scatter_reference_mm=0.0)

    runtime.update_controls(window_low_mm=2.0, window_high_mm=12.0)

    reposition_target = runtime.consume_pending_reposition()
    assert reposition_target is not None
    assert abs(reposition_target - 7.0) < 1e-6