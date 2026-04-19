import os
import sys
import logging
import importlib
import warnings
from types import SimpleNamespace

root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "rpi"))
if root not in sys.path:
    sys.path.insert(0, root)

import pytest

from motion.trapezoidal_profile import TrapezoidalMotionProfile
from motion.spindle_kinematics import SpindleKinematics
from motion.winding_pattern import WindingPattern
from motion.scatter_engine import ScatterEngine
from motion.move import HomingMove, WoundMove
from motion.axis_state import AxisState
from motion.ramp_config import RampConfig
from motion.move_queue import MoveQueue
from motion.synchronized_segment_generator import SyncAxisConfig
from motion.engine import WindingEngine
from transport.streamer import MultiAxisRampStreamer, StreamAxisConfig
from winding.program import WindingProgram
from core.config import AppConfiguration
from core.events import EventBus
from core.shared_state import SharedState


def test_motion_public_import_does_not_emit_deprecation_warning():
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        from motion import AxisMotionConfig as ImportedAxisMotionConfig

    assert ImportedAxisMotionConfig is not None
    assert not any(issubclass(item.category, DeprecationWarning) for item in caught)


def test_streamer_set_generator_replaces_generator_and_resets_finished_state():
    class FakeTransport:
        def get_status(self):
            return SimpleNamespace(last_executed_sequence=0xFFFF)

    streamer = MultiAxisRampStreamer(
        transport=FakeTransport(),
        axis_streams=[StreamAxisConfig(axis_id=0, ramp=RampConfig(target_rpm=300.0))],
    )
    original = streamer._generator
    streamer._generator_finished = True

    replacement = iter([SimpleNamespace(axis_steps=[0], duration_us=1000, direction_mask=0)])
    streamer.set_generator(replacement)

    assert streamer._generator is replacement
    assert streamer._generator is not original
    assert streamer._generator_finished is False


def test_engine_exposes_public_config_property():
    config = AppConfiguration()
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        event_bus=EventBus(),
        config=config,
    )

    assert engine.config is config


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


def test_trapezoidal_turns_clamp_beyond_total_duration():
    profile = TrapezoidalMotionProfile(
        start_rpm=0.0,
        target_rpm=600.0,
        accel_s=1.0,
        cruise_s=1.0,
        decel_s=1.0,
    )
    total = profile.total_duration
    assert profile.turns_at(0.0) == 0.0
    assert pytest.approx(profile.turns_at(total + 1e-9), rel=1e-12) == profile.turns_at(total)


def test_trapezoidal_deceleration_non_negative_with_nonzero_start_rpm():
    profile = TrapezoidalMotionProfile(
        start_rpm=300.0,
        target_rpm=600.0,
        accel_s=0.5,
        cruise_s=0.5,
        decel_s=0.5,
    )
    assert profile.rps_at(profile.total_duration) >= 0.0
    assert profile.turns_at(profile.total_duration) >= 0.0


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


def test_app_config_spindle_accel_unit_conversion_rpm_per_s_to_steps_per_s2():
    cfg = AppConfiguration(
        spindle_steps_per_revolution=200,
        spindle_microstepping=32,
        spindle_max_acceleration_rpm=600.0,
    )
    assert cfg.spindle_max_acceleration_steps_per_s2 == 64000.0


def test_scatter_engine_rejects_zero_freq1():
    with pytest.raises(ValueError):
        ScatterEngine(freq1=0.0)


def test_scatter_engine_rejects_negative_amplitude():
    with pytest.raises(ValueError):
        ScatterEngine(amplitude_mm=-1.0)


def test_wound_run_two_pass_duration_keeps_positive_cruise_for_10s_case():
    config = AppConfiguration(
        spindle_steps_per_revolution=200,
        spindle_microstepping=32,
        spindle_max_acceleration_rpm=300.0,
        spindle_max_deceleration_rpm=300.0,
    )
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        event_bus=EventBus(),
        config=config,
    )

    captured: list[WoundMove] = []
    engine._move_queue.enqueue = lambda move: captured.append(move)  # type: ignore[assignment]

    # 10 s at cruise: total_turns = target_rps * 10 = 10 * 10 = 100 turns.
    engine.wound_run(
        spindle_axis_id=0,
        traverse_axis_id=1,
        target_rpm=600.0,
        accel_s=None,
        cruise_s=None,
        decel_s=None,
        bobbin_width_mm=10.0,
        turns_per_mm=5.0,
    )

    assert captured, "wound_run must enqueue one move"
    move = captured[0]
    assert move.kinematics.accel_s == pytest.approx(2.0, rel=1e-6)
    assert move.kinematics.decel_s == pytest.approx(2.0, rel=1e-6)
    assert move.kinematics.cruise_s > 0.0


def test_execute_homing_waits_for_endstop_request_confirmation(monkeypatch):
    class FakeTransport:
        def __init__(self) -> None:
            self._seq = 0
            self.wait_calls: list[int] = []

        def get_status(self):
            return SimpleNamespace(last_executed_sequence=0xFFFF)

        def enable_endstop_request(self, axis_id: int, arm: bool):
            self._seq += 1
            return self._seq, SimpleNamespace()

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001):
            self.wait_calls.append(sequence)
            return SimpleNamespace(last_result=0)

    class FakeStreamer:
        endstop_triggered = True

        def __init__(self) -> None:
            self._generator = None
            self._generator_finished = False

        def set_generator(self, generator) -> None:
            self._generator = generator
            self._generator_finished = False

        def stream_all(self) -> int:
            return 0

    axis_state = AxisState(axis_id=1)
    transport = FakeTransport()
    queue = MoveQueue(
        transport=transport,
        axis_states={1: axis_state},
        poll_interval_s=0.001,
        print_every=1,
    )
    monkeypatch.setattr(queue, "_make_streamer", lambda axis_configs: FakeStreamer())

    move = HomingMove(
        name="home_test",
        axis_id=1,
        steps_per_rev=6400,
        approach_rpm=100.0,
        search_rpm=20.0,
        backoff_steps=3200,
        max_approach_steps=6400,
    )

    queue._execute_homing(move)

    assert move.state.name == "COMPLETED"
    # approach arm + backoff disarm + search arm + final disarm
    assert len(transport.wait_calls) == 4


def test_execute_wound_move_invalidates_positions_on_stop_requested(monkeypatch):
    class FakeTransport:
        def get_status(self):
            return SimpleNamespace(last_executed_sequence=0xFFFF)

    class FakeStreamer:
        def __init__(self, queue: MoveQueue) -> None:
            self._generator = None
            self._generator_finished = False
            self.endstop_triggered = False
            self._queue = queue

        def set_generator(self, generator) -> None:
            self._generator = generator
            self._generator_finished = False

        def stream_all(self) -> int:
            self._queue._stop_requested = True
            return 0

    spindle_state = AxisState(axis_id=0)
    traverse_state = AxisState(axis_id=1)
    spindle_state.mark_homed(100)
    traverse_state.mark_homed(200)

    queue = MoveQueue(
        transport=FakeTransport(),
        axis_states={0: spindle_state, 1: traverse_state},
        poll_interval_s=0.001,
        print_every=1,
    )
    monkeypatch.setattr(queue, "_make_wound_streamer", lambda move: FakeStreamer(queue))

    move = WoundMove(
        name="wound_stop",
        kinematics=SpindleKinematics(target_rpm=600.0, accel_s=0.2, cruise_s=0.2, decel_s=0.2),
        pattern=WindingPattern(bobbin_width_mm=10.0, turns_per_mm=5.0),
        scatter=ScatterEngine(amplitude_mm=0.0),
        spindle_cfg=SyncAxisConfig(axis_index=0, steps_per_unit=6400.0),
        traverse_cfg=SyncAxisConfig(axis_index=1, steps_per_unit=800.0),
    )

    queue._execute_wound_move(move)

    assert move.state.name == "ABORTED"
    assert spindle_state.position_steps is None
    assert traverse_state.position_steps is None


def test_wound_run_logs_warning_for_inconsistent_explicit_profile(caplog):
    config = AppConfiguration()
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        event_bus=EventBus(),
        config=config,
    )
    engine._move_queue.enqueue = lambda move: None  # type: ignore[assignment]

    with caplog.at_level(logging.WARNING, logger="motion.engine"):
        engine.wound_run(
            spindle_axis_id=0,
            traverse_axis_id=1,
            target_rpm=600.0,
            accel_s=0.1,
            cruise_s=0.1,
            decel_s=0.1,
            bobbin_width_mm=10.0,
            turns_per_mm=5.0,
        )

    assert "wound_run: profile produces" in caplog.text


def test_from_axis_ids_respects_segment_duration_s_parameter():
    class FakeTransport:
        def get_status(self):
            return SimpleNamespace(last_executed_sequence=0xFFFF)

    streamer = MultiAxisRampStreamer.from_axis_ids(
        FakeTransport(),
        [0, 1],
        target_hz=1000.0,
        segment_duration_s=0.003,
    )
    assert streamer._segment_duration_s == pytest.approx(0.003)


def test_legacy_syncrhonized_module_emits_deprecation_warning():
    module_name = "motion.syncrhonized_segment_generator"
    sys.modules.pop(module_name, None)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", DeprecationWarning)
        importlib.import_module(module_name)
    assert any(
        isinstance(w.message, DeprecationWarning)
        and "deprecated" in str(w.message)
        for w in caught
    )
