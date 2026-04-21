import os
import sys
from importlib import import_module
from types import SimpleNamespace


root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "rpi"))
if root not in sys.path:
    sys.path.insert(0, root)

pytest = import_module("pytest")

config_module = import_module("core.config")
engine_module = import_module("core.engine")
events_module = import_module("core.events")
shared_state_module = import_module("core.shared_state")
status_module = import_module("core.status")
winding_handler_module = import_module("jsonrpc.winding_handler")
axis_state_module = import_module("motion.axis_state")
command_service_module = import_module("motion.command_service")
segment_json_module = import_module("motion.segment_json")
messages_module = import_module("transport.messages")
streamer_module = import_module("transport.streamer")
program_module = import_module("winding.program")
adaptive_service_module = import_module("winding.service")

AppConfiguration = config_module.AppConfiguration
ConfigurationManager = config_module.ConfigurationManager
WindingEngine = engine_module.WindingEngine
EventBus = events_module.EventBus
EventKind = events_module.EventKind
SharedState = shared_state_module.SharedState
RuntimeStatusService = status_module.RuntimeStatusService
WindingRpcHandler = winding_handler_module.WindingRpcHandler
AxisState = axis_state_module.AxisState
MotionCommandService = command_service_module.MotionCommandService
dump_segment_json = segment_json_module.dump_segment_json
load_segments = segment_json_module.load_segments
MultiAxisSegment = messages_module.MultiAxisSegment
MultiAxisRampStreamer = streamer_module.MultiAxisRampStreamer
WindingProgram = program_module.WindingProgram
AdaptiveWindingService = adaptive_service_module.AdaptiveWindingService


def test_winding_rpc_stop_routes_through_coordinator() -> None:
    stop_calls: list[str] = []
    stop_plan = {
        "mode": "stop",
        "keep_enabled_axes": [],
        "invalidate_positions": [0, 1],
        "reason": "stop requested",
    }

    handler = WindingRpcHandler(
        engine=SimpleNamespace(),
        commands=SimpleNamespace(),
        adaptive_winding=SimpleNamespace(),
        status_service=SimpleNamespace(),
        coordinator=SimpleNamespace(
            request_stop=lambda: stop_calls.append("stop") or SimpleNamespace(snapshot=lambda: stop_plan)
        ),
    )

    assert handler.stop() == {"status": "stopping", "stop_plan": stop_plan}
    assert stop_calls == ["stop"]


def test_engine_wait_for_move_queue_clears_queue_on_timeout() -> None:
    class FakeMoveQueue:
        def __init__(self) -> None:
            self.clear_called = False
            self.current_move = object()
            self.pending_count = 1

        def clear(self) -> None:
            self.clear_called = True

    move_queue = FakeMoveQueue()
    state = SharedState(axis_states={})
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=state,
        move_queue=move_queue,
        lateral_controller=SimpleNamespace(require_homed=lambda axis_id=None: None),
        event_bus=EventBus(),
        config=AppConfiguration(),
    )

    engine._wait_for_move_queue(poll_s=0.0, timeout_s=0.0)

    assert move_queue.clear_called is True
    assert state.engine_state.name == "FAULT"


def test_motion_command_wound_run_uses_lateral_steps_per_mm() -> None:
    captured_moves: list[object] = []
    config = AppConfiguration(lateral_traverse_pitch_mm=2.0)
    commands = MotionCommandService(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        move_queue=SimpleNamespace(enqueue=lambda move: captured_moves.append(move)),
        lateral_controller=SimpleNamespace(require_homed=lambda axis_id=None: None),
        config=config,
    )

    commands.wound_run(
        spindle_axis_id=config.spindle_axis_id,
        traverse_axis_id=config.lateral_axis_id,
        target_rpm=600.0,
        accel_s=0.1,
        cruise_s=0.2,
        decel_s=0.1,
        bobbin_width_mm=10.0,
        turns_per_mm=5.0,
    )

    assert captured_moves
    move = captured_moves[0]
    assert move.traverse_cfg.steps_per_unit == pytest.approx(config.lateral_steps_per_mm)


def test_segment_json_load_segments_round_trip(tmp_path) -> None:
    path = tmp_path / "segments.json"
    expected = [
        MultiAxisSegment(
            sequence=1,
            duration_us=4000,
            steps=[12, 34],
            directions=[0, 1],
        )
    ]

    dump_segment_json(path, iter(expected), axis_ids=[0, 1], metadata={"name": "test"})
    loaded = load_segments(path)

    assert len(loaded) == 1
    assert loaded[0].sequence == expected[0].sequence
    assert loaded[0].duration_us == expected[0].duration_us
    assert loaded[0].steps == expected[0].steps
    assert loaded[0].directions == expected[0].directions


def test_app_configuration_rejects_invalid_traverse_pitch() -> None:
    with pytest.raises(ValueError, match="lateral_traverse_pitch_mm must be positive"):
        AppConfiguration(lateral_traverse_pitch_mm=0.0)


def test_configuration_manager_round_trips_json_configuration(tmp_path) -> None:
    path = tmp_path / "config.json"
    manager = ConfigurationManager(path)
    manager.active_configuration = AppConfiguration(spi_speed_hz=2_000_000)

    manager.save_configuration()
    loaded = manager.load_configuration()

    assert loaded.spi_speed_hz == 2_000_000
    assert manager.get_saved_configuration() is not None


def test_adaptive_resume_session_rejects_dead_worker() -> None:
    service = AdaptiveWindingService(
        shared_state=SharedState(axis_states={}),
        move_queue=SimpleNamespace(clear=lambda: None),
        lateral_controller=SimpleNamespace(),
        event_bus=EventBus(),
        config=AppConfiguration(),
    )

    class FakeRuntime:
        state = "paused"

        def __init__(self) -> None:
            self.resume_called = False

        def snapshot(self) -> dict[str, float]:
            return {"target_rpm": 250.0}

        def resume(self) -> None:
            self.resume_called = True

    runtime = FakeRuntime()
    service._active_session = runtime
    service._worker = None

    with pytest.raises(RuntimeError, match="worker is no longer running"):
        service.resume_session()

    assert runtime.resume_called is False


def test_event_bus_events_are_versioned() -> None:
    bus = EventBus()
    bus.publish(EventKind.STATUS_UPDATE, value=123)

    event = bus.consume(timeout_s=0.0)

    assert event is not None
    assert event.version == 1
    assert event.data == {"value": 123}


def test_runtime_status_exposes_transport_diagnostics() -> None:
    service = RuntimeStatusService(
        shared_state=SharedState(axis_states={}),
        move_queue_status_provider=lambda: {"pending": 0},
        lateral_controller=SimpleNamespace(refresh_home_state=lambda: None),
        config=AppConfiguration(),
        transport_diagnostics_provider=lambda: {"bad_crc": 2},
    )

    status = service.engine_status()

    assert status["transport"] == {"bad_crc": 2}


def test_axis_state_check_move_strict_rejects_unknown_position() -> None:
    axis = AxisState(axis_id=1)

    assert axis.check_move(100) is True
    assert axis.check_move(100, strict=True) is False


def test_winding_program_snapshot_contains_dataclass_fields() -> None:
    program = WindingProgram(
        name="snapshot",
        num_layers=2,
        spindle_rpm=600.0,
        layer_pitch_mm=0.1,
        wire_diameter_mm=0.05,
        bobbin_width_mm=12.0,
        home_backoff_steps=1234,
    )

    snapshot = program.snapshot()

    assert snapshot["home_before_start"] is True
    assert snapshot["home_backoff_steps"] == 1234
    assert snapshot["spindle_axis_id"] == 0
    assert snapshot["lateral_axis_id"] == 1


def test_streamer_accepts_custom_stall_timeout() -> None:
    streamer = MultiAxisRampStreamer.from_axis_ids(
        transport=SimpleNamespace(get_status=lambda: SimpleNamespace(last_executed_sequence=0xFFFF)),
        axis_ids=[0, 1],
        target_hz=1000.0,
        stall_timeout_s=9.5,
    )

    assert streamer._stall_timeout_s == pytest.approx(9.5)