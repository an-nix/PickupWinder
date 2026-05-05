import json
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
coordinator_module = import_module("core.coordinator")
shared_state_module = import_module("core.shared_state")
status_module = import_module("core.status")
handlers_module = import_module("jsonrpc.handlers")
protocol_module = import_module("jsonrpc.protocol")
rpc_server_module = import_module("jsonrpc.rpc_server")
axis_state_module = import_module("motion.axis_state")
transport_module = import_module("transport.spi_transport")
events_module = import_module("core.events")

AppConfiguration = config_module.AppConfiguration
MotionStopMode = coordinator_module.MotionStopMode
MotionStopPlan = coordinator_module.MotionStopPlan
SharedState = shared_state_module.SharedState
RuntimeStatusService = status_module.RuntimeStatusService
RpcHandler = handlers_module.RpcHandler
JsonRpcInvalidParamsError = protocol_module.JsonRpcInvalidParamsError
JsonRpcInvalidRequestError = protocol_module.JsonRpcInvalidRequestError
parse_json_rpc = protocol_module.parse_json_rpc
JsonRpcServer = rpc_server_module.JsonRpcServer
AxisState = axis_state_module.AxisState
Esp32SpiTransport = transport_module.Esp32SpiTransport
EventBus = events_module.EventBus


def test_motion_stop_plan_pause_keeps_only_homed_axes_enabled() -> None:
    axis_states = {
        0: AxisState(axis_id=0, homed=True),
        1: AxisState(axis_id=1, homed=False),
    }

    plan = MotionStopPlan.pause(axis_states, [0, 1], reason="pause requested")

    assert plan.mode is MotionStopMode.PAUSE
    assert plan.keep_enabled_axes == frozenset({0})
    assert plan.invalidate_positions == frozenset()


def test_runtime_status_service_faults_when_rpc_worker_faults() -> None:
    service = RuntimeStatusService(
        shared_state=SharedState(axis_states={}),
        move_queue_status_provider=lambda: {"pending": 0},
        lateral_controller=SimpleNamespace(refresh_home_state=lambda: None),
        config=AppConfiguration(),
        rpc_health_provider=lambda: {
            "accept_thread_alive": False,
            "notify_thread_alive": True,
            "accept_thread_faulted": True,
            "notify_thread_faulted": False,
        },
    )

    status = service.application_status()

    assert status["health"] == "faulted"
    assert "rpc_server" in status["workers"]


def test_rpc_handler_rejects_signature_mismatch_as_invalid_params() -> None:
    handler = RpcHandler()
    handler.register_method("demo.required", lambda value: value)

    with pytest.raises(JsonRpcInvalidParamsError, match="missing a required argument"):
        handler.dispatch("demo.required", {})


def test_parse_json_rpc_rejects_scalar_params() -> None:
    with pytest.raises(JsonRpcInvalidRequestError):
        parse_json_rpc('{"jsonrpc":"2.0","method":"demo","params":1,"id":1}')


def test_json_rpc_server_dispatch_times_out_long_running_method() -> None:
    handler = RpcHandler()
    handler.register_method("demo.slow", lambda: time.sleep(0.05))
    server = JsonRpcServer(
        handler=handler,
        event_bus=EventBus(),
        socket_path="/tmp/pw-test.sock",
        rpc_call_timeout_s=0.01,
    )

    try:
        response = json.loads(
            server._dispatch('{"jsonrpc":"2.0","method":"demo.slow","id":7}')
        )
    finally:
        server._call_executor.shutdown(wait=False, cancel_futures=True)

    assert response["error"]["code"] == -32000
    assert "timed out" in response["error"]["message"]
    assert response["id"] == 7
    assert server.health_status()["timed_out_requests"] == 1


def test_transport_safe_shutdown_preserves_requested_axes() -> None:
    transport = object.__new__(Esp32SpiTransport)
    calls: list[tuple[object, ...]] = []

    transport.stop_axis = lambda axis_id=0xFF: calls.append(("stop_axis", axis_id))
    transport.set_axis_enabled = lambda axis_id, enable: calls.append(("set_axis_enabled", axis_id, enable))
    transport.get_status = lambda timeout_s=1.0, allow_stale=False: SimpleNamespace(
        enabled_mask=1 << 0,
        running_mask=0,
    )
    transport.emergency_stop = lambda axis_id=0xFF: calls.append(("emergency_stop", axis_id))
    transport.disable_all = lambda: calls.append(("disable_all",))

    status = transport.safe_shutdown(
        axis_ids=[0, 1],
        keep_enabled_axes={0},
        emergency_on_failure=False,
    )

    assert status.enabled_mask == 1 << 0
    assert calls == [
        ("stop_axis", 0xFF),
        ("set_axis_enabled", 1, False),
    ]


def test_transport_safe_shutdown_raises_when_unexpected_axis_stays_enabled() -> None:
    transport = object.__new__(Esp32SpiTransport)
    transport.stop_axis = lambda axis_id=0xFF: None
    transport.set_axis_enabled = lambda axis_id, enable: None
    transport.get_status = lambda timeout_s=1.0, allow_stale=False: SimpleNamespace(
        enabled_mask=1 << 1,
        running_mask=0,
    )
    transport.emergency_stop = lambda axis_id=0xFF: None
    transport.disable_all = lambda: None

    with pytest.raises(RuntimeError, match="enabled axes remain active"):
        transport.safe_shutdown(
            axis_ids=[0, 1],
            keep_enabled_axes={0},
            emergency_on_failure=False,
        )