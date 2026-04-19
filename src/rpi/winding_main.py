from __future__ import annotations

import logging
import signal
import sys
import time
from typing import Any

from core.config import AppConfiguration
from core.events import EventBus
from core.shared_state import SharedState
from motion.axis_state import AxisState
from motion.engine import WindingEngine
from transport.spi_transport import Esp32SpiTransport
from winding.program import WindingProgram
from jsonrpc import AppRpcHandler, JsonRpcServer
from jsonrpc.protocol import JsonRpcError

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(name)s %(levelname)s %(message)s",
)
logger = logging.getLogger("main")


def _parse_spi_device(device_path: str) -> tuple[int, int]:
    import re

    match = re.fullmatch(r"/dev/spidev(\d+)\.(\d+)", device_path)
    if match is None:
        raise ValueError("spi_device must be in the form /dev/spidev<bus>.<device>")
    return int(match.group(1)), int(match.group(2))


def _register_winding_rpc_methods(
    handler: AppRpcHandler,
    engine: WindingEngine,
    shared_state: SharedState,
) -> None:
    def submit_program(program: dict) -> dict[str, Any]:
        if not isinstance(program, dict):
            raise JsonRpcError(-32602, "Invalid params: expected program object")
        p = WindingProgram(**program)
        engine.submit_program(p)
        return {"status": "queued", "program": p.snapshot()}

    def stop(_params: Any | None = None) -> dict[str, str]:
        engine.request_stop()
        return {"status": "stopping"}

    def jog(axis_id: int, steps: int, rpm: float, reverse: bool = False) -> dict[str, Any]:
        engine.jog(axis_id=axis_id, steps=steps, rpm=rpm, reverse=reverse)
        return {"status": "queued"}

    def wound_run(
        spindle_axis_id: int,
        traverse_axis_id: int,
        target_rpm: float,
        bobbin_width_mm: float,
        turns_per_mm: float,
        accel_s: float = 2.0,
        cruise_s: float = 10.0,
        decel_s: float = 2.0,
        scatter_amplitude_mm: float = 0.0,
        scatter_damping_margin_mm: float = 0.0,
        spindle_reverse: bool = False,
        traverse_reverse: bool = False,
    ) -> dict[str, Any]:
        engine.wound_run(
            spindle_axis_id=spindle_axis_id,
            traverse_axis_id=traverse_axis_id,
            target_rpm=target_rpm,
            accel_s=accel_s,
            cruise_s=cruise_s,
            decel_s=decel_s,
            bobbin_width_mm=bobbin_width_mm,
            turns_per_mm=turns_per_mm,
            scatter_amplitude_mm=scatter_amplitude_mm,
            scatter_damping_margin_mm=scatter_damping_margin_mm,
            spindle_reverse=spindle_reverse,
            traverse_reverse=traverse_reverse,
        )
        return {"status": "queued"}

    def clear_fault(_params: Any | None = None) -> dict[str, str]:
        engine.clear_fault()
        return {"status": "ok"}

    def status(_params: Any | None = None) -> dict[str, Any]:
        return engine.status()

    def axis_state(axis_id: int) -> dict[str, Any]:
        state = shared_state.axis_states.get(axis_id)
        if state is None:
            raise JsonRpcError(-32602, f"Unknown axis_id: {axis_id}")
        return state.snapshot()

    def arm_endstop(axis_id: int) -> dict[str, Any]:
        engine.arm_endstop(axis_id)
        return {"status": "armed", "axis_id": axis_id}

    def disarm_endstop(axis_id: int) -> dict[str, Any]:
        engine.disarm_endstop(axis_id)
        return {"status": "disarmed", "axis_id": axis_id}

    handler.register_method("winding.submit_program", submit_program)
    handler.register_method("winding.stop", stop)
    handler.register_method("winding.jog", jog)
    handler.register_method("winding.wound_run", wound_run)
    handler.register_method("winding.clear_fault", clear_fault)
    handler.register_method("winding.status", status)
    handler.register_method("winding.axis_state", axis_state)
    handler.register_method("winding.arm_endstop", arm_endstop)
    handler.register_method("winding.disarm_endstop", disarm_endstop)


def main() -> None:
    config = AppConfiguration()

    bus, device = _parse_spi_device(config.spi_device)
    logger.info("Opening SPI transport on %s @ %d Hz", config.spi_device, config.spi_speed_hz)
    transport = Esp32SpiTransport(
        bus=bus,
        device=device,
        speed_hz=config.spi_speed_hz,
        mode=0,
    )

    axis_states = {
        config.spindle_axis_id: AxisState(
            axis_id=config.spindle_axis_id,
            steps_per_rev=config.spindle_steps_per_revolution * config.spindle_microstepping,
        ),
        config.lateral_axis_id: AxisState(
            axis_id=config.lateral_axis_id,
            steps_per_rev=config.lateral_steps_per_revolution * config.lateral_microstepping,
        ),
    }

    shared_state = SharedState(axis_states=axis_states)
    event_bus = EventBus()

    engine = WindingEngine(
        transport=transport,
        shared_state=shared_state,
        event_bus=event_bus,
    )
    rpc_handler = AppRpcHandler()
    _register_winding_rpc_methods(rpc_handler, engine, shared_state)
    rpc_server = JsonRpcServer(
        handler=rpc_handler,
        event_bus=event_bus,
        socket_path="/tmp/winding.sock",
    )

    engine.start()
    rpc_server.start()
    logger.info("Winding controller started")

    def _shutdown(sig, frame) -> None:
        logger.info("Shutdown requested (signal %s)", sig)
        engine.stop()
        rpc_server.stop()
        transport.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
