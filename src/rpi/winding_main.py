from __future__ import annotations

import logging
import signal
import sys
import time

from core.config import AppConfiguration
from core.events import EventBus
from core.shared_state import SharedState
from motion.axis_state import AxisState
from motion.engine import WindingEngine
from transport.spi_transport import Esp32SpiTransport
from jsonrpc import AppRpcHandler, JsonRpcServer
from jsonrpc.winding_handler import WindingRpcHandler

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
        config=config,
    )
    rpc_handler = AppRpcHandler(app=engine)
    WindingRpcHandler(engine=engine, shared_state=shared_state).register_all(rpc_handler)
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
