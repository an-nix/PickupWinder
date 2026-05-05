import os
import sys
import time
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
move_queue_module = import_module("motion.move_queue")
segment_json_module = import_module("motion.segment_json")
messages_module = import_module("transport.messages")
mock_transport_module = import_module("transport.mock_spi_transport")
spi_transport_module = import_module("transport.spi_transport")
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
MockSpiTransport = mock_transport_module.MockSpiTransport
Esp32SpiTransport = spi_transport_module.Esp32SpiTransport
MultiAxisRampStreamer = streamer_module.MultiAxisRampStreamer
WindingProgram = program_module.WindingProgram
AdaptiveWindingService = adaptive_service_module.AdaptiveWindingService
SpiMessageResult = messages_module.SpiMessageResult
StatusPayload = messages_module.StatusPayload
SPI_MSG_VERSION = messages_module.SPI_MSG_VERSION


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


def test_motion_command_home_lateral_returns_immediately_and_completes_async() -> None:
    config = AppConfiguration()
    state = SharedState(axis_states={})
    started: list[tuple[int, float, float, int]] = []

    def _home(*, axis_id: int, approach_rpm: float, search_rpm: float, backoff_steps: int):
        started.append((axis_id, approach_rpm, search_rpm, backoff_steps))
        time.sleep(0.05)
        return True, None

    commands = MotionCommandService(
        transport=SimpleNamespace(),
        shared_state=state,
        move_queue=SimpleNamespace(),
        lateral_controller=SimpleNamespace(
            home=_home,
        ),
        config=config,
    )

    result = commands.home_lateral(approach_rpm=20.0, search_rpm=10.0, backoff_steps=3200)

    assert result == {
        "status": "started",
        "axis_id": config.lateral_axis_id,
        "approach_rpm": 20.0,
        "search_rpm": 10.0,
        "backoff_steps": 3200,
    }
    assert state.engine_state.name == "HOMING"

    deadline = time.monotonic() + 1.0
    while state.engine_state.name != "IDLE" and time.monotonic() < deadline:
        time.sleep(0.01)

    assert started == [(config.lateral_axis_id, 20.0, 10.0, 3200)]
    assert state.engine_state.name == "IDLE"


def test_winding_rpc_home_lateral_returns_started_response() -> None:
    config = AppConfiguration()
    handler = WindingRpcHandler(
        engine=SimpleNamespace(),
        commands=SimpleNamespace(
            home_lateral=lambda **kwargs: {
                "status": "started",
                "axis_id": config.lateral_axis_id,
                **kwargs,
            }
        ),
        adaptive_winding=SimpleNamespace(),
        status_service=SimpleNamespace(),
        coordinator=SimpleNamespace(),
    )

    response = handler.home_lateral(approach_rpm=20.0, search_rpm=10.0, backoff_steps=3200)

    assert response == {
        "status": "started",
        "axis_id": config.lateral_axis_id,
        "approach_rpm": 20.0,
        "search_rpm": 10.0,
        "backoff_steps": 3200,
    }


def test_streamer_prefill_caps_long_segment_batch_size() -> None:
    transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer.from_axis_ids(
        transport,
        [1],
        target_hz=1066.0,
        target_buffer_time_s=0.2,
    )
    streamer._prefilling = True
    streamer.set_generator(
        iter(
            [
                MultiAxisSegment(
                    sequence=index,
                    duration_us=30_000,
                    steps=[32],
                    directions=[0],
                )
                for index in range(60)
            ]
        )
    )

    count, _status = streamer._collect_and_send_batch(transport.get_status())

    assert 1 <= count <= 17
    assert transport.sent_payloads
    assert len(transport.sent_payloads[0].segments) == count
    assert count < 60


def test_wait_for_request_result_ignores_transient_protocol_error_after_matching_sequence() -> None:
    transport = Esp32SpiTransport.__new__(Esp32SpiTransport)

    statuses = iter(
        [
            StatusPayload(
                uptime_ms=0,
                queue_free_slots=(0, 0, 0, 0),
                ring_free_slots=(0, 0, 0, 0),
                underrun_count=(0, 0, 0, 0),
                last_rx_sequence=42,
                last_rx_type=0,
                last_result=int(SpiMessageResult.BAD_MAGIC),
                protocol_version=SPI_MSG_VERSION,
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
                last_executed_sequence=0,
                multi_axis_queue_free=0,
                planner_queue_free=0,
                last_planned_sequence=0,
                segments_dropped=0,
            ),
            StatusPayload(
                uptime_ms=0,
                queue_free_slots=(0, 0, 0, 0),
                ring_free_slots=(0, 0, 0, 0),
                underrun_count=(0, 0, 0, 0),
                last_rx_sequence=42,
                last_rx_type=0,
                last_result=int(SpiMessageResult.OK),
                protocol_version=SPI_MSG_VERSION,
                enabled_mask=0,
                running_mask=0,
                lateral_endstop_state=0,
                endstop_armed_mask=0,
                endstop_hit_mask=0,
                last_executed_sequence=0,
                multi_axis_queue_free=0,
                planner_queue_free=0,
                last_planned_sequence=0,
                segments_dropped=0,
            ),
        ]
    )

    transport.get_status = lambda timeout_s=1.0, allow_stale=False: next(statuses)

    status = Esp32SpiTransport.wait_for_request_result(
        transport,
        42,
        poll_interval_s=0.0,
        timeout_s=0.1,
    )

    assert status.last_rx_sequence == 42
    assert status.last_result == int(SpiMessageResult.OK)


def test_spi_transport_disables_ready_handshake_after_repeated_timeouts() -> None:
    class FakeReadyMonitor:
        def value(self) -> int:
            return 0

    transport = Esp32SpiTransport.__new__(Esp32SpiTransport)
    transport._ready_monitor = FakeReadyMonitor()
    transport._ready_active_level = 1
    transport._ready_wait_timeout_s = 0.0
    transport._ready_poll_sleep_s = 0.0
    transport._ready_timeout_streak = 0
    transport._ready_timeout_disable_threshold = 3
    transport._ready_handshake_disabled = False
    transport._diag_ready_timeouts = 0
    transport._diag_lifetime_ready_timeouts = 0
    transport._device_path = "/dev/spidev0.0"
    transport._inter_transfer_guard_s = 0.0
    transport._last_xfer_end_ts = 0.0

    Esp32SpiTransport._wait_until_ready(transport)
    Esp32SpiTransport._wait_until_ready(transport)
    assert transport._ready_handshake_disabled is False

    Esp32SpiTransport._wait_until_ready(transport)
    assert transport._ready_handshake_disabled is True
    assert transport._diag_lifetime_ready_timeouts == 3

    Esp32SpiTransport._wait_until_ready(transport)
    assert transport._diag_lifetime_ready_timeouts == 3


def test_streamer_does_not_false_trigger_on_initial_segments_dropped_baseline() -> None:
    streamer = MultiAxisRampStreamer.from_axis_ids(
        transport=SimpleNamespace(get_status=lambda: SimpleNamespace(last_executed_sequence=0xFFFF)),
        axis_ids=[1],
        target_hz=100.0,
        initial_segments_dropped=7,
    )
    streamer.note_endstop_armed(1, True)

    status = SimpleNamespace(
        endstop_armed_mask=1 << 1,
        lateral_endstop_state=messages_module.LATERAL_ENDSTOP_PRESENT_OPEN,
        running_mask=0,
        endstop_hit_mask=0,
        segments_dropped=7,
    )

    assert streamer._check_endstop(status) is False
    assert streamer.endstop_triggered is False


def test_move_queue_set_endstop_armed_uses_ack_mask_without_extra_poll() -> None:
    status = SimpleNamespace(
        last_result=int(SpiMessageResult.OK),
        endstop_armed_mask=1 << 1,
        lateral_endstop_state=messages_module.LATERAL_ENDSTOP_PRESENT_OPEN,
    )

    class FakeTransport:
        def enable_endstop_request(self, axis_id: int, arm: bool):
            assert axis_id == 1
            assert arm is True
            return 123, status

        def wait_for_request_result(self, sequence: int, *, poll_interval_s: float = 0.001):
            assert sequence == 123
            return status

    queue = move_queue_module.MoveQueue(
        transport=FakeTransport(),
        axis_states={},
        poll_interval_s=0.0,
    )

    def _unexpected_wait(*_args, **_kwargs):
        raise AssertionError("_wait_for_endstop_arm_state should not be called")

    queue._wait_for_endstop_arm_state = _unexpected_wait

    returned = queue._set_endstop_armed(1, arm=True)

    assert returned is status


def test_move_queue_next_motion_sequence_requires_fresh_status() -> None:
    allow_stale_calls: list[bool] = []

    class FakeTransport:
        def get_status(self, *, allow_stale: bool = True):
            allow_stale_calls.append(allow_stale)
            return SimpleNamespace(last_executed_sequence=7)

    queue = move_queue_module.MoveQueue(
        transport=FakeTransport(),
        axis_states={},
        poll_interval_s=0.0,
    )

    assert queue._next_motion_sequence() == 8
    assert allow_stale_calls == [False]


def test_move_queue_homing_start_requires_fresh_status() -> None:
    allow_stale_calls: list[bool] = []

    class FakeTransport:
        def get_status(self, *, allow_stale: bool = True):
            allow_stale_calls.append(allow_stale)
            return SimpleNamespace(
                lateral_endstop_state=messages_module.LATERAL_ENDSTOP_PRESENT_OPEN
            )

    queue = move_queue_module.MoveQueue(
        transport=FakeTransport(),
        axis_states={},
        poll_interval_s=0.0,
    )

    queue._ensure_homing_can_start(1, "start")

    assert allow_stale_calls == [False]


def test_move_queue_resumes_after_expected_endstop_from_flush_floor() -> None:
    class FakeTransport:
        def get_status(self):
            return SimpleNamespace(last_executed_sequence=9)

    queue = move_queue_module.MoveQueue(
        transport=FakeTransport(),
        axis_states={},
        poll_interval_s=0.0,
    )

    streamer = SimpleNamespace(flush_floor_sequence=20)

    assert queue._next_sequence_after_streamer(streamer) == 21


def test_move_queue_does_not_preclear_on_unstable_initial_closed_state(monkeypatch) -> None:
    closed = SimpleNamespace(lateral_endstop_state=messages_module.LATERAL_ENDSTOP_PRESENT_CLOSED)
    open_state = SimpleNamespace(lateral_endstop_state=messages_module.LATERAL_ENDSTOP_PRESENT_OPEN)
    statuses = iter([closed, open_state])

    queue = move_queue_module.MoveQueue(
        transport=SimpleNamespace(get_status=lambda: next(statuses)),
        axis_states={},
        poll_interval_s=0.0,
    )

    monkeypatch.setattr(move_queue_module.time, "sleep", lambda _s: None)

    state = queue._confirm_initial_closed_endstop(1)

    assert state == messages_module.LATERAL_ENDSTOP_PRESENT_OPEN


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