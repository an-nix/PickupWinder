from __future__ import annotations

import logging
from pathlib import Path
import re

from core import ConfigurationManager, WindingEngine
from core.config import AppConfiguration
from core.coordinator import MotionCoordinator
from core.events import EventBus
from core.lateral import LateralAxisController
from core.shared_state import SharedState
from core.status import RuntimeStatusService
from jsonrpc import JsonRpcServer, SystemRpcHandler
from jsonrpc.winding_handler import WindingRpcHandler
from motion.axis_state import AxisLimits, AxisState
from core.command_service import MotionCommandService
from motion.move_queue import MoveQueue
from transport.spi_transport import Esp32SpiTransport
from winding.service import AdaptiveWindingService
from winding.program_store import ProgramStore


logger = logging.getLogger(__name__)


def _default_data_dir() -> Path:
    return Path.home() / "data"


def _default_config_file_path() -> Path:
    return _default_data_dir() / "config.json"


def _default_program_store_dir() -> Path:
    return _default_data_dir() / "programs"


def _parse_spi_device(device_path: str) -> tuple[int, int]:
    match = re.fullmatch(r"/dev/spidev(\d+)\.(\d+)", device_path)
    if match is None:
        raise ValueError("spi_device must be in the form /dev/spidev<bus>.<device>")
    return int(match.group(1)), int(match.group(2))


def _build_axis_states(config: AppConfiguration) -> dict[int, AxisState]:
    return {
        config.spindle_axis_id: AxisState(
            axis_id=config.spindle_axis_id,
            steps_per_rev=(
                config.spindle_steps_per_revolution * config.spindle_microstepping
            ),
        ),
        config.lateral_axis_id: AxisState(
            axis_id=config.lateral_axis_id,
            steps_per_rev=(
                config.lateral_steps_per_revolution * config.lateral_microstepping
            ),
            steps_per_mm=config.lateral_steps_per_mm,
            limits=AxisLimits(
                min_steps=config.lateral_soft_limit_min_steps,
                max_steps=config.lateral_soft_limit_max_steps,
            ),
        ),
    }


def _create_transport(config: AppConfiguration) -> Esp32SpiTransport:
    bus, device = _parse_spi_device(config.spi_device)
    logger.info(
        "Opening SPI transport on %s @ %d Hz (mode 1)",
        config.spi_device,
        config.spi_speed_hz,
    )
    return Esp32SpiTransport(
        bus=bus,
        device=device,
        speed_hz=config.spi_speed_hz,
        mode=1,
        ready_gpio_chip=config.spi_ready_gpio_chip,
        ready_gpio_line=config.spi_ready_gpio_line,
        ready_active_high=config.spi_ready_active_high,
    )


class WinderApplication:
    """Compose the host runtime and own its process lifecycle."""

    def __init__(
        self,
        config: AppConfiguration | None = None,
        config_file_path: str | Path | None = None,
        program_store_dir: str | Path | None = None,
    ) -> None:
        resolved_config_path = (
            Path(config_file_path)
            if config_file_path is not None
            else _default_config_file_path()
        )
        self.config_manager = ConfigurationManager(resolved_config_path)
        if config is None and resolved_config_path.exists():
            self.config = self.config_manager.load_configuration()
        else:
            self.config = config or self.config_manager.active_configuration
            self.config_manager.active_configuration = self.config

        self.transport = _create_transport(self.config)
        resolved_program_store_dir = (
            Path(program_store_dir)
            if program_store_dir is not None
            else _default_program_store_dir()
        )
        self.program_store = ProgramStore(resolved_program_store_dir)
        self.shared_state = SharedState(axis_states=_build_axis_states(self.config))
        self.event_bus = EventBus()

        self.move_queue = MoveQueue(
            transport=self.transport,
            axis_states=self.shared_state.axis_states,
            poll_interval_s=0.005,
            print_every=8,
        )
        self.lateral_controller = LateralAxisController(
            transport=self.transport,
            shared_state=self.shared_state,
            move_queue=self.move_queue,
            event_bus=self.event_bus,
            config=self.config,
        )
        self.commands = MotionCommandService(
            transport=self.transport,
            shared_state=self.shared_state,
            move_queue=self.move_queue,
            lateral_controller=self.lateral_controller,
            config=self.config,
        )
        self.adaptive_winding = AdaptiveWindingService(
            shared_state=self.shared_state,
            move_queue=self.move_queue,
            lateral_controller=self.lateral_controller,
            event_bus=self.event_bus,
            config=self.config,
        )
        self.engine = WindingEngine(
            transport=self.transport,
            shared_state=self.shared_state,
            move_queue=self.move_queue,
            lateral_controller=self.lateral_controller,
            commands=self.commands,
            event_bus=self.event_bus,
            config=self.config,
        )
        self.coordinator = MotionCoordinator(
            engine=self.engine,
            adaptive_winding=self.adaptive_winding,
            move_queue=self.move_queue,
            shared_state=self.shared_state,
        )
        self.status_service = RuntimeStatusService(
            shared_state=self.shared_state,
            move_queue_status_provider=self.move_queue.status,
            lateral_controller=self.lateral_controller,
            config=self.config,
            transport_diagnostics_provider=self.transport.transport_diagnostics,
            engine_health_provider=self.engine.health_status,
            adaptive_health_provider=self.adaptive_winding.health_status,
            rpc_health_provider=lambda: (
                self.rpc_server.health_status()
                if hasattr(self, "rpc_server")
                else {"status": "initializing"}
            ),
        )
        self.rpc_handler = SystemRpcHandler(status_service=self.status_service)
        WindingRpcHandler(
            engine=self.engine,
            commands=self.commands,
            adaptive_winding=self.adaptive_winding,
            status_service=self.status_service,
            coordinator=self.coordinator,
            config=self.config,
            config_manager=self.config_manager,
            shared_state=self.shared_state,
            event_bus=self.event_bus,
            program_store=self.program_store,
        ).register_all(self.rpc_handler)
        self.rpc_server = JsonRpcServer(
            handler=self.rpc_handler,
            event_bus=self.event_bus,
            socket_path=self.config.rpc_socket_path,
        )

        self._started = False
        self._stopped = False

    def start(self) -> None:
        if self._started:
            return
        self.engine.start()
        self.rpc_server.start()
        self._started = True

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        errors: list[str] = []

        if self._started:
            stop_plan = self.coordinator.stop_plan(reason="application shutdown")
            try:
                self.adaptive_winding.request_stop(stop_plan=stop_plan, clear_queue=False)
            except Exception as exc:
                errors.append(f"adaptive stop request failed: {exc}")
            try:
                self.engine.request_stop(stop_plan=stop_plan, clear_queue=False)
            except Exception as exc:
                errors.append(f"engine stop request failed: {exc}")
            try:
                self.move_queue.clear(stop_plan=stop_plan)
            except Exception as exc:
                errors.append(f"move queue clear failed: {exc}")
            try:
                self.rpc_server.stop()
            except Exception as exc:
                errors.append(f"rpc server stop failed: {exc}")
            try:
                self.adaptive_winding.stop()
            except Exception as exc:
                errors.append(f"adaptive winding stop failed: {exc}")
            try:
                self.engine.stop()
            except Exception as exc:
                errors.append(f"engine stop failed: {exc}")
            try:
                self.transport.safe_shutdown(
                    axis_ids=self.shared_state.axis_states.keys(),
                    keep_enabled_axes=stop_plan.keep_enabled_axes,
                )
            except Exception as exc:
                errors.append(f"transport safe shutdown failed: {exc}")
            self._started = False

        try:
            self.transport.close()
        except Exception as exc:
            errors.append(f"transport close failed: {exc}")

        if errors:
            raise RuntimeError("; ".join(errors))
