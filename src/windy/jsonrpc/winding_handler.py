"""Composition façade for all ``winding.*`` and ``program.*`` RPC methods.

Delegates to four focused handlers:
- ``ProgramRpcHandler``  — program.* CRUD and revisions
- ``SessionRpcHandler``  — winding.start_session, update_session, pause, resume
- ``MachineRpcHandler``  — winding.jog, home, axis moves, endstops, fault clear
- ``ExecutionRpcHandler``— winding.submit_program, wound_run, stop, status

The RPC method names and parameter contracts are unchanged.
``runtime.py`` continues to instantiate a single ``WindingRpcHandler``.
"""

from __future__ import annotations

from core import AppConfiguration, ConfigurationManager, WindingEngine
from core.command_service import MotionCommandService
from core.coordinator import MotionCoordinator
from core.events import EventBus
from core.shared_state import SharedState
from core.status import RuntimeStatusService
from jsonrpc.execution_handler import ExecutionRpcHandler
from jsonrpc.handlers import RpcHandler
from jsonrpc.machine_handler import MachineRpcHandler
from jsonrpc.program_handler import ProgramRpcHandler
from jsonrpc.session_handler import SessionRpcHandler
from winding.program_store import ProgramStore
from winding.service import AdaptiveWindingService


class WindingRpcHandler:
    """Instantiates all four domain handlers and wires them to the RPC dispatcher."""

    def __init__(
        self,
        *,
        engine: WindingEngine,
        commands: MotionCommandService,
        adaptive_winding: AdaptiveWindingService,
        status_service: RuntimeStatusService,
        coordinator: MotionCoordinator,
        config: AppConfiguration,
        config_manager: ConfigurationManager,
        shared_state: SharedState,
        event_bus: EventBus,
        program_store: ProgramStore,
    ) -> None:
        self._program = ProgramRpcHandler(
            program_store=program_store,
            shared_state=shared_state,
            event_bus=event_bus,
        )
        self._session = SessionRpcHandler(
            adaptive_winding=adaptive_winding,
            coordinator=coordinator,
            shared_state=shared_state,
            event_bus=event_bus,
            program_store=program_store,
        )
        self._machine = MachineRpcHandler(
            commands=commands,
            coordinator=coordinator,
            config=config,
            config_manager=config_manager,
        )
        self._execution = ExecutionRpcHandler(
            engine=engine,
            commands=commands,
            coordinator=coordinator,
            status_service=status_service,
            shared_state=shared_state,
            event_bus=event_bus,
            program_store=program_store,
            config=config,
        )

    def register_all(self, handler: RpcHandler) -> None:
        self._program.register_all(handler)
        self._session.register_all(handler)
        self._machine.register_all(handler)
        self._execution.register_all(handler)
