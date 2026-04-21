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
from winding import WindingPattern, ScatterEngine, SyncAxisConfig, WoundMove
from motion.move import HomingMove
from motion.axis_state import AxisLimits, AxisState
from motion.ramp_config import RampConfig
from motion.move_queue import MoveQueue
from core import WindingEngine
from transport.streamer import MultiAxisRampStreamer, StreamAxisConfig
from transport.messages import (
    LATERAL_ENDSTOP_ABSENT,
    LATERAL_ENDSTOP_PRESENT_CLOSED,
    LATERAL_ENDSTOP_PRESENT_OPEN,
    MultiAxisSegment,
    SpiMessageResult,
)
from transport import MockSpiTransport
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


def test_streamer_keep_enabled_axes_preserves_axis_enable_state():
    transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[
            StreamAxisConfig(axis_id=1, ramp=RampConfig(axis_id=1, target_rpm=300.0))
        ],
        keep_enabled_axes={1},
    )
    streamer.set_generator(
        iter([
            MultiAxisSegment(sequence=0, duration_us=4000, steps=[0, 12], directions=[0, 0]),
        ])
    )

    streamer.stream_all()

    assert 1 in transport._enabled_axes


def test_streamer_collect_batch_uses_confirmed_ack_status():
    class FakeTransport:
        def __init__(self) -> None:
            self.wait_calls: list[int] = []
            self._send_seq = 7

        def get_status(self):
            return SimpleNamespace(
                last_executed_sequence=0xFFFF,
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                last_result=int(SpiMessageResult.OK),
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0xFF,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
            )

        def send_multi_axis_segment_block_request(self, payload):
            seq = self._send_seq
            self._send_seq += 1
            return seq, SimpleNamespace(last_result=int(SpiMessageResult.QUEUE_FULL))

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001, timeout_s: float = 1.5):
            self.wait_calls.append(sequence)
            return SimpleNamespace(
                last_result=int(SpiMessageResult.OK),
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0xFF,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
            )

    transport = FakeTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=0, ramp=RampConfig(target_rpm=300.0))],
    )
    streamer.set_generator(iter([
        MultiAxisSegment(sequence=0, duration_us=4000, steps=[12], directions=[0]),
    ]))

    sent, _status = streamer._collect_and_send_batch(transport.get_status())

    assert sent == 1
    assert transport.wait_calls == [7]
    assert len(streamer._inflight) == 1
    assert streamer._retry_batch is None


def test_streamer_single_axis_id_one_generates_one_axis_segment():
    transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=1, ramp=RampConfig(axis_id=1, target_rpm=300.0))],
    )
    streamer.set_generator(iter([
        MultiAxisSegment(sequence=0, duration_us=4000, steps=[12], directions=[0]),
    ]))

    sent, _status = streamer._collect_and_send_batch(transport.get_status())

    assert sent == 1
    assert transport.sent_payloads
    payload = transport.sent_payloads[-1]
    assert payload.axis_ids == [1]
    assert payload.segments[0].steps == [12]
    assert payload.segments[0].directions == [0]
    assert len(streamer._inflight) == 1
    assert streamer._retry_batch is None


def test_streamer_retries_same_batch_after_confirmed_queue_full():
    class FakeTransport:
        def __init__(self) -> None:
            self.wait_calls: list[int] = []
            self.sent_sequences: list[list[int]] = []
            self._send_seq = 21
            self._wait_results = [
                int(SpiMessageResult.QUEUE_FULL),
                int(SpiMessageResult.OK),
            ]

        def get_status(self):
            return SimpleNamespace(
                last_executed_sequence=0xFFFF,
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                last_result=int(SpiMessageResult.OK),
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0xFF,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
            )

        def send_multi_axis_segment_block_request(self, payload):
            seq = self._send_seq
            self._send_seq += 1
            self.sent_sequences.append([segment.sequence for segment in payload.segments])
            return seq, SimpleNamespace(last_result=int(SpiMessageResult.OK))

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001, timeout_s: float = 1.5):
            self.wait_calls.append(sequence)
            result = self._wait_results.pop(0)
            return SimpleNamespace(
                last_result=result,
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0xFF,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
            )

    transport = FakeTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=0, ramp=RampConfig(target_rpm=300.0))],
    )
    streamer.set_generator(iter([
        MultiAxisSegment(sequence=0, duration_us=4000, steps=[12], directions=[0]),
    ]))

    first_sent, _first_status = streamer._collect_and_send_batch(transport.get_status())
    second_sent, _second_status = streamer._collect_and_send_batch(transport.get_status())

    assert first_sent == 0
    assert second_sent == 1
    assert transport.wait_calls == [21, 22]
    assert transport.sent_sequences == [[0], [0]]
    assert len(streamer._inflight) == 1
    assert streamer._retry_batch is None


def test_streamer_treats_endstop_blocked_ack_as_triggered():
    class FakeTransport:
        def __init__(self) -> None:
            self.wait_calls: list[int] = []
            self._send_seq = 41

        def get_status(self):
            return SimpleNamespace(
                last_executed_sequence=0xFFFF,
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                last_result=int(SpiMessageResult.OK),
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0x00,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
            )

        def send_multi_axis_segment_block_request(self, payload):
            seq = self._send_seq
            self._send_seq += 1
            return seq, SimpleNamespace(last_result=int(SpiMessageResult.OK))

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001, timeout_s: float = 1.5):
            self.wait_calls.append(sequence)
            return SimpleNamespace(
                last_result=int(SpiMessageResult.ENDSTOP_BLOCKED),
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=LATERAL_ENDSTOP_PRESENT_CLOSED,
                endstop_armed_mask=1 << 1,
                endstop_hit_mask=1 << 1,
            )

        def flush_until(self, sequence: int):
            return self.get_status()

    transport = FakeTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=1, ramp=RampConfig(axis_id=1, target_rpm=300.0))],
    )
    streamer.set_generator(
        iter([MultiAxisSegment(sequence=0, duration_us=4000, steps=[12], directions=[0])])
    )

    sent, status = streamer._collect_and_send_batch(transport.get_status())

    assert sent == 0
    assert status.last_result == int(SpiMessageResult.ENDSTOP_BLOCKED)
    assert streamer.endstop_triggered is True
    assert streamer.has_stop_been_requested() is True


def test_streamer_detects_closed_endstop_when_local_arm_tracking_is_set():
    transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=1, ramp=RampConfig(axis_id=1, target_rpm=300.0))],
    )
    streamer.note_endstop_armed(1, True)

    triggered = streamer._check_endstop(
        SimpleNamespace(
            lateral_endstop_state=LATERAL_ENDSTOP_PRESENT_CLOSED,
            endstop_armed_mask=0,
            endstop_hit_mask=0,
            running_mask=0,
            segments_dropped=0,
        )
    )

    assert triggered is True
    assert streamer.endstop_triggered is True
    assert streamer.has_stop_been_requested() is True


def test_streamer_detects_endstop_when_closed_and_running_mask_drops():
    transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=1, ramp=RampConfig(axis_id=1, target_rpm=300.0))],
    )
    streamer.note_endstop_armed(1, True)

    triggered = streamer._check_endstop(
        SimpleNamespace(
            lateral_endstop_state=LATERAL_ENDSTOP_PRESENT_CLOSED,
            endstop_armed_mask=1 << 1,
            endstop_hit_mask=0,
            running_mask=0,
            segments_dropped=0,
        )
    )

    assert triggered is True
    assert streamer.endstop_triggered is True


def test_streamer_treats_endstop_hit_mask_as_canonical_trigger():
    transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=1, ramp=RampConfig(axis_id=1, target_rpm=300.0))],
    )
    streamer.note_endstop_armed(1, True)

    triggered = streamer._check_endstop(
        SimpleNamespace(
            lateral_endstop_state=LATERAL_ENDSTOP_PRESENT_OPEN,
            endstop_armed_mask=0,
            endstop_hit_mask=1 << 1,
            running_mask=1 << 1,
            segments_dropped=0,
        )
    )

    assert triggered is True
    assert streamer.endstop_triggered is True


def test_streamer_ignores_planner_drops_while_armed_axis_is_still_running():
    transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=1, ramp=RampConfig(axis_id=1, target_rpm=300.0))],
    )
    streamer.note_endstop_armed(1, True)
    streamer._last_segments_dropped = 4

    triggered = streamer._check_endstop(
        SimpleNamespace(
            segments_dropped=6,
            lateral_endstop_state=LATERAL_ENDSTOP_PRESENT_OPEN,
            endstop_armed_mask=1 << 1,
            endstop_hit_mask=0,
            running_mask=1 << 1,
        )
    )

    assert triggered is False
    assert streamer.endstop_triggered is False


def test_streamer_treats_planner_drops_during_armed_stopped_move_as_endstop_recovery():
    transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=1, ramp=RampConfig(axis_id=1, target_rpm=300.0))],
    )
    streamer.note_endstop_armed(1, True)
    streamer._last_segments_dropped = 4

    triggered = streamer._check_endstop(
        SimpleNamespace(
            segments_dropped=6,
            lateral_endstop_state=LATERAL_ENDSTOP_PRESENT_OPEN,
            endstop_armed_mask=1 << 1,
            endstop_hit_mask=0,
            running_mask=0,
        )
    )

    assert triggered is True
    assert streamer.endstop_triggered is True
    assert streamer.has_stop_been_requested() is True


def test_streamer_flushes_to_last_confirmed_sequence_on_endstop():
    transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer(
        transport=transport,
        axis_streams=[StreamAxisConfig(axis_id=1, ramp=RampConfig(axis_id=1, target_rpm=300.0))],
    )
    streamer._last_sent_motion_seq = 18
    streamer._last_confirmed_motion_seq = 12

    streamer._mark_endstop_triggered()

    assert streamer.has_stop_been_requested() is True
    assert streamer._flush_sequence_requested == 12


def test_axis_state_reports_closed_endstop_from_protocol_value():
    axis_state = AxisState(axis_id=1)

    axis_state.update_endstop_state(LATERAL_ENDSTOP_PRESENT_CLOSED)

    assert axis_state.endstop_triggered is True
    assert axis_state.snapshot()["endstop_triggered"] is True


def test_streamer_rejects_non_monotonic_sequences_inside_batch():
    class FakeTransport:
        def get_status(self):
            return SimpleNamespace(
                last_executed_sequence=0xFFFF,
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                last_result=int(SpiMessageResult.OK),
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0xFF,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
            )

        def send_multi_axis_segment_block_request(self, payload):
            return 1, SimpleNamespace(last_result=int(SpiMessageResult.OK))

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001, timeout_s: float = 1.5):
            return SimpleNamespace(
                last_result=int(SpiMessageResult.OK),
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0xFF,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
            )

    streamer = MultiAxisRampStreamer(
        transport=FakeTransport(),
        axis_streams=[StreamAxisConfig(axis_id=0, ramp=RampConfig(target_rpm=300.0))],
    )
    streamer.set_generator(iter([
        MultiAxisSegment(sequence=10, duration_us=4000, steps=[12], directions=[0]),
        MultiAxisSegment(sequence=12, duration_us=4000, steps=[12], directions=[0]),
        MultiAxisSegment(sequence=11, duration_us=4000, steps=[12], directions=[0]),
    ]))

    with pytest.raises(RuntimeError, match="motion sequence not strictly increasing"):
        streamer._collect_and_send_batch(streamer._transport.get_status())


def test_streamer_allows_wrapped_sequences_inside_batch():
    class FakeTransport:
        def get_status(self):
            return SimpleNamespace(
                last_executed_sequence=0xFFFD,
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                last_result=int(SpiMessageResult.OK),
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0xFF,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
            )

        def send_multi_axis_segment_block_request(self, payload):
            return 1, SimpleNamespace(last_result=int(SpiMessageResult.OK))

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001, timeout_s: float = 1.5):
            return SimpleNamespace(
                last_result=int(SpiMessageResult.OK),
                queue_free_slots=(128, 128, 128, 128),
                ring_free_slots=(4096, 4096, 4096, 4096),
                underrun_count=(0, 0, 0, 0),
                planner_queue_free=128,
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0xFF,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
            )

    streamer = MultiAxisRampStreamer(
        transport=FakeTransport(),
        axis_streams=[StreamAxisConfig(axis_id=0, ramp=RampConfig(target_rpm=300.0))],
    )
    streamer.set_generator(iter([
        MultiAxisSegment(sequence=0xFFFE, duration_us=4000, steps=[12], directions=[0]),
        MultiAxisSegment(sequence=0xFFFF, duration_us=4000, steps=[12], directions=[0]),
        MultiAxisSegment(sequence=0x0000, duration_us=4000, steps=[12], directions=[0]),
        MultiAxisSegment(sequence=0x0001, duration_us=4000, steps=[12], directions=[0]),
    ]))

    sent, _status = streamer._collect_and_send_batch(streamer._transport.get_status())

    assert sent == 4
    assert streamer._last_sent_motion_seq == 0x0001


def test_mock_spi_transport_preserves_wrapped_execution_order():
    transport = MockSpiTransport()
    payload = SimpleNamespace(
        segments=[
            MultiAxisSegment(sequence=0xFFFE, duration_us=4000, steps=[12], directions=[0]),
            MultiAxisSegment(sequence=0xFFFF, duration_us=4000, steps=[12], directions=[0]),
            MultiAxisSegment(sequence=0x0000, duration_us=4000, steps=[12], directions=[0]),
            MultiAxisSegment(sequence=0x0001, duration_us=4000, steps=[12], directions=[0]),
        ]
    )

    _seq, first_status = transport.send_multi_axis_segment_block_request(payload)
    second_status = transport.get_status()
    third_status = transport.get_status()
    fourth_status = transport.get_status()

    assert first_status.last_executed_sequence == 0xFFFE
    assert second_status.last_executed_sequence == 0xFFFF
    assert third_status.last_executed_sequence == 0x0000
    assert fourth_status.last_executed_sequence == 0x0001


def test_engine_exposes_public_config_property():
    config = AppConfiguration()
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        event_bus=EventBus(),
        config=config,
    )

    assert engine.config is config


def test_engine_request_stop_keeps_worker_thread_alive(monkeypatch):
    class FakeMoveQueue:
        def __init__(self, transport, axis_states, poll_interval_s, print_every):
            self._current_move = None

        def start(self) -> None:
            pass

        def stop(self, timeout_s: float = 5.0) -> None:
            pass

        def clear(self) -> None:
            pass

        @property
        def pending_count(self) -> int:
            return 0

        @property
        def current_move(self):
            return self._current_move

        def status(self) -> dict[str, object]:
            return {
                "running": False,
                "current_move": None,
                "pending_moves": [],
                "history": [],
                "axis_states": {},
            }

    monkeypatch.setattr("core.engine.MoveQueue", FakeMoveQueue)

    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        event_bus=EventBus(),
        config=AppConfiguration(),
    )

    engine.start()
    engine.request_stop()
    engine._program_event.set()
    engine._thread.join(timeout=0.2)

    assert engine._thread is not None
    assert engine._thread.is_alive() is True

    engine.stop(timeout_s=0.2)


def test_execute_program_returns_early_after_homing_failure(monkeypatch):
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        event_bus=EventBus(),
        config=AppConfiguration(),
    )
    program = WindingProgram(
        name="failing_home",
        num_layers=1,
        spindle_rpm=120.0,
        layer_pitch_mm=0.1,
        wire_diameter_mm=0.05,
        home_before_start=True,
    )

    monkeypatch.setattr(
        engine,
        "_home_lateral_axis",
        lambda **kwargs: (False, "simulated homing failure"),
    )

    def _unexpected_run_layer(*args, **kwargs):
        raise AssertionError("_run_layer must not be called when homing fails")

    monkeypatch.setattr(engine, "_run_layer", _unexpected_run_layer)

    engine._execute_program(program)

    assert engine._state.engine_state.name == "HOMING"


def test_engine_jog_rejects_unhomed_lateral_axis():
    config = AppConfiguration()
    lateral_state = AxisState(
        axis_id=config.lateral_axis_id,
        steps_per_mm=config.lateral_steps_per_mm,
        limits=AxisLimits(
            min_steps=config.lateral_soft_limit_min_steps,
            max_steps=config.lateral_soft_limit_max_steps,
        ),
    )
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={config.lateral_axis_id: lateral_state}),
        event_bus=EventBus(),
        config=config,
    )

    with pytest.raises(RuntimeError, match="must be homed"):
        engine.jog(axis_id=config.lateral_axis_id, steps=100, rpm=50.0)


def test_engine_home_lateral_requires_clear_fault_after_fault():
    config = AppConfiguration()
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        event_bus=EventBus(),
        config=config,
    )
    engine._state.set_fault("motor error")

    with pytest.raises(RuntimeError, match="winding.clear_fault"):
        engine.home_lateral()


def test_engine_move_lateral_to_mm_queues_signed_jog():
    config = AppConfiguration(lateral_soft_limit_max_mm=12.0)
    lateral_state = AxisState(
        axis_id=config.lateral_axis_id,
        steps_per_mm=config.lateral_steps_per_mm,
        limits=AxisLimits(
            min_steps=config.lateral_soft_limit_min_steps,
            max_steps=config.lateral_soft_limit_max_steps,
        ),
    )
    lateral_state.mark_homed(0)
    engine = WindingEngine(
        transport=SimpleNamespace(
            get_status=lambda: SimpleNamespace(enabled_mask=(1 << config.lateral_axis_id))
        ),
        shared_state=SharedState(axis_states={config.lateral_axis_id: lateral_state}),
        event_bus=EventBus(),
        config=config,
    )

    captured: list[tuple[int, int, float, bool]] = []

    def _capture(move) -> None:
        captured.append(
            (
                move.axis_id,
                move._steps,
                move._config.axis_configs[0].ramp.target_rpm,
                move._reverse,
            )
        )

    engine._move_queue.enqueue = _capture  # type: ignore[assignment]

    result = engine.move_lateral_to_mm(position_mm=5.0, rpm=40.0)

    assert result["status"] == "queued"
    assert captured == [
        (
            config.lateral_axis_id,
            int(round(5.0 * config.lateral_steps_per_mm)),
            40.0,
            False,
        )
    ]


def test_engine_run_axis_rejects_lateral_target_beyond_soft_limit():
    config = AppConfiguration(
        lateral_soft_limit_max_mm=0.1,
        lateral_max_rpm=60,
    )
    lateral_state = AxisState(
        axis_id=config.lateral_axis_id,
        steps_per_mm=config.lateral_steps_per_mm,
        limits=AxisLimits(
            min_steps=config.lateral_soft_limit_min_steps,
            max_steps=config.lateral_soft_limit_max_steps,
        ),
    )
    lateral_state.mark_homed(0)
    engine = WindingEngine(
        transport=SimpleNamespace(
            get_status=lambda: SimpleNamespace(enabled_mask=(1 << config.lateral_axis_id))
        ),
        shared_state=SharedState(axis_states={config.lateral_axis_id: lateral_state}),
        event_bus=EventBus(),
        config=config,
    )

    with pytest.raises(ValueError, match="outside configured soft limits"):
        engine.run_axis(
            duration_s=2.0,
            targets=[{"axis_id": config.lateral_axis_id, "rpm": 60.0}],
        )


def test_engine_refresh_lateral_home_state_invalidates_after_enable_loss():
    config = AppConfiguration()
    lateral_state = AxisState(
        axis_id=config.lateral_axis_id,
        steps_per_mm=config.lateral_steps_per_mm,
    )
    lateral_state.mark_homed(0)
    engine = WindingEngine(
        transport=SimpleNamespace(get_status=lambda: SimpleNamespace(enabled_mask=0)),
        shared_state=SharedState(axis_states={config.lateral_axis_id: lateral_state}),
        event_bus=EventBus(),
        config=config,
    )

    status = engine.status()

    assert status["shared_state"]["axis_states"][config.lateral_axis_id]["homed"] is False


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
            self.arm_state = False
            self.endstop_state = LATERAL_ENDSTOP_PRESENT_OPEN

        def get_status(self):
            return SimpleNamespace(
                last_executed_sequence=0xFFFF,
                lateral_endstop_state=self.endstop_state,
                endstop_armed_mask=(1 << 1) if self.arm_state else 0,
            )

        def enable_endstop_request(self, axis_id: int, arm: bool):
            self._seq += 1
            self.arm_state = arm
            return self._seq, SimpleNamespace()

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001):
            self.wait_calls.append(sequence)
            return SimpleNamespace(
                last_result=0,
                lateral_endstop_state=self.endstop_state,
                endstop_armed_mask=(1 << 1) if self.arm_state else 0,
            )

    class FakeStreamer:
        def __init__(self) -> None:
            self.endstop_triggered = True
            self._generator = None
            self._generator_finished = False
            self.note_calls: list[tuple[int, bool]] = []

        def note_endstop_armed(self, axis_id: int, arm: bool) -> None:
            self.note_calls.append((axis_id, arm))

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
    monkeypatch.setattr(
        queue,
        "_make_streamer",
        lambda axis_configs, keep_enabled_axes=None: FakeStreamer(),
    )

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


def test_execute_homing_fails_when_endstop_not_open_before_start(monkeypatch):
    class FakeTransport:
        def __init__(self) -> None:
            self._seq = 0
            self.arm_state = False
            self.endstop_state = LATERAL_ENDSTOP_PRESENT_CLOSED

        def get_status(self):
            return SimpleNamespace(
                last_executed_sequence=0xFFFF,
                lateral_endstop_state=self.endstop_state,
                endstop_armed_mask=(1 << 1) if self.arm_state else 0,
            )

        def enable_endstop_request(self, axis_id: int, arm: bool):
            self._seq += 1
            self.arm_state = arm
            return self._seq, SimpleNamespace()

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001):
            return SimpleNamespace(
                last_result=0,
                lateral_endstop_state=self.endstop_state,
                endstop_armed_mask=(1 << 1) if self.arm_state else 0,
            )

    class FakeStreamer:
        def __init__(self, transport: FakeTransport, phase_name: str) -> None:
            self._transport = transport
            self._phase_name = phase_name
            self.endstop_triggered = phase_name in ("approach", "search")

        def note_endstop_armed(self, axis_id: int, arm: bool) -> None:
            return None

        def set_generator(self, generator) -> None:
            self._generator = generator

        def stream_all(self) -> int:
            if self._phase_name == "preclear":
                self._transport.endstop_state = LATERAL_ENDSTOP_PRESENT_OPEN
            elif self._phase_name == "approach":
                self._transport.endstop_state = LATERAL_ENDSTOP_PRESENT_CLOSED
            elif self._phase_name == "backoff":
                self._transport.endstop_state = LATERAL_ENDSTOP_PRESENT_OPEN
            elif self._phase_name == "search":
                self._transport.endstop_state = LATERAL_ENDSTOP_PRESENT_CLOSED
            return 0

    transport = FakeTransport()
    queue = MoveQueue(
        transport=transport,
        axis_states={1: AxisState(axis_id=1)},
        poll_interval_s=0.001,
        print_every=1,
    )
    phase_order = iter(["preclear", "approach", "backoff", "search"])
    monkeypatch.setattr(
        queue,
        "_make_streamer",
        lambda axis_configs, keep_enabled_axes=None: FakeStreamer(transport, next(phase_order)),
    )

    move = HomingMove(
        name="home_closed",
        axis_id=1,
        steps_per_rev=6400,
        approach_rpm=100.0,
        search_rpm=20.0,
        backoff_steps=3200,
        max_approach_steps=6400,
    )

    queue._execute_homing(move)

    assert move.state.name == "COMPLETED"


def test_execute_homing_fails_when_endstop_sensor_is_absent(monkeypatch):
    class FakeTransport:
        def get_status(self):
            return SimpleNamespace(
                last_executed_sequence=0xFFFF,
                lateral_endstop_state=LATERAL_ENDSTOP_ABSENT,
                endstop_armed_mask=0,
            )

        def enable_endstop_request(self, axis_id: int, arm: bool):
            raise AssertionError("homing must not arm when endstop sensor is absent")

    queue = MoveQueue(
        transport=FakeTransport(),
        axis_states={1: AxisState(axis_id=1)},
        poll_interval_s=0.001,
        print_every=1,
    )
    monkeypatch.setattr(
        queue,
        "_make_streamer",
        lambda axis_configs, keep_enabled_axes=None: (_ for _ in ()).throw(AssertionError("streamer must not be created")),
    )

    move = HomingMove(
        name="home_absent",
        axis_id=1,
        steps_per_rev=6400,
        approach_rpm=100.0,
        search_rpm=20.0,
        backoff_steps=3200,
        max_approach_steps=6400,
    )

    queue._execute_homing(move)

    assert move.state.name == "FAILED"
    assert move.error is not None
    assert "ABSENT" in move.error


def test_execute_homing_disarms_before_backoff_and_waits_for_release(monkeypatch):
    class FakeTransport:
        def __init__(self) -> None:
            self._seq = 0
            self.arm_state = False
            self.endstop_state = LATERAL_ENDSTOP_PRESENT_OPEN
            self.arm_history: list[bool] = []

        def get_status(self):
            return SimpleNamespace(
                last_executed_sequence=0xFFFF,
                lateral_endstop_state=self.endstop_state,
                endstop_armed_mask=(1 << 1) if self.arm_state else 0,
            )

        def enable_endstop_request(self, axis_id: int, arm: bool):
            self._seq += 1
            self.arm_state = arm
            self.arm_history.append(arm)
            return self._seq, SimpleNamespace()

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001):
            return SimpleNamespace(
                last_result=0,
                lateral_endstop_state=self.endstop_state,
                endstop_armed_mask=(1 << 1) if self.arm_state else 0,
            )

    class FakeStreamer:
        def __init__(self, transport: FakeTransport, phase_name: str) -> None:
            self._generator = None
            self._generator_finished = False
            self.endstop_triggered = phase_name in ("approach", "search")
            self._transport = transport
            self._phase_name = phase_name

        def note_endstop_armed(self, axis_id: int, arm: bool) -> None:
            return None

        def set_generator(self, generator) -> None:
            self._generator = generator
            self._generator_finished = False

        def stream_all(self) -> int:
            if self._phase_name == "approach":
                self._transport.endstop_state = LATERAL_ENDSTOP_PRESENT_CLOSED
            elif self._phase_name == "backoff":
                assert self._transport.arm_state is False
                self._transport.endstop_state = LATERAL_ENDSTOP_PRESENT_OPEN
            elif self._phase_name == "search":
                self._transport.endstop_state = LATERAL_ENDSTOP_PRESENT_CLOSED
            return 0

    transport = FakeTransport()
    queue = MoveQueue(
        transport=transport,
        axis_states={1: AxisState(axis_id=1)},
        poll_interval_s=0.001,
        print_every=1,
    )

    def make_streamer(axis_configs, keep_enabled_axes=None):
        phase_name = axis_configs[0].ramp.name.split(":")[-1] if hasattr(axis_configs[0].ramp, "name") else ""
        return FakeStreamer(transport, phase_name)

    phase_order = iter(["approach", "backoff", "search"])
    monkeypatch.setattr(
        queue,
        "_make_streamer",
        lambda axis_configs, keep_enabled_axes=None: FakeStreamer(transport, next(phase_order)),
    )

    move = HomingMove(
        name="home_backoff",
        axis_id=1,
        steps_per_rev=6400,
        approach_rpm=100.0,
        search_rpm=20.0,
        backoff_steps=3200,
        max_approach_steps=6400,
    )

    queue._execute_homing(move)

    assert move.state.name == "COMPLETED"
    assert transport.arm_history == [True, False, True, False]


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
    monkeypatch.setattr(
        queue,
        "_make_wound_streamer",
        lambda move, keep_enabled_axes=None: FakeStreamer(queue),
    )

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


