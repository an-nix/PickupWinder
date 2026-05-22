"""RPC handler for all ``program.*`` JSON-RPC methods (CRUD + revisions)."""

from __future__ import annotations

from typing import Any

from core.events import EventBus, EventKind
from core.shared_state import SharedState
from jsonrpc.handlers import RpcHandler
from jsonrpc.protocol import JsonRpcError
from winding import ProgramNotFoundError, ProgramStore
from winding.program import WindingProgram


class ProgramRpcHandler:
    """Handles CRUD and revision management for persisted winding programs."""

    def __init__(
        self,
        *,
        program_store: ProgramStore,
        shared_state: SharedState,
        event_bus: EventBus,
    ) -> None:
        self._program_store = program_store
        self._state = shared_state
        self._events = event_bus

    def register_all(self, handler: RpcHandler) -> None:
        handler.register_method("program.list", self.list_programs)
        handler.register_method("program.get", self.get_program)
        handler.register_method("program.save", self.save_program)
        handler.register_method("program.update", self.update_program)
        handler.register_method("program.load", self.load_program)
        handler.register_method("program.delete", self.delete_program)
        handler.register_method("program.list_revisions", self.list_revisions)
        handler.register_method("program.restore_revision", self.restore_revision)

    # ── Methods ────────────────────────────────────────────────────────────

    def list_programs(self, include_content: bool = False) -> dict[str, Any]:
        loaded_program = self._state.loaded_program
        return {
            "programs": self._program_store.list_programs(include_content=include_content),
            "loaded_program": loaded_program.snapshot() if loaded_program else None,
        }

    def get_program(self, program_id: str) -> dict[str, Any]:
        try:
            program = self._program_store.get_program(program_id)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        return {"program": program.snapshot()}

    def save_program(
        self,
        program: dict[str, Any],
        program_id: str | None = None,
        load: bool = False,
    ) -> dict[str, Any]:
        try:
            saved = self._program_store.save_program(
                self._coerce_program(program),
                program_id=program_id,
            )
        except ValueError as exc:
            raise JsonRpcError(-32602, str(exc)) from exc
        if load:
            self._state.set_loaded_program(saved)
            self._events.publish(EventKind.PROGRAM_LOADED, program=saved.snapshot())
        self._events.publish(EventKind.PROGRAM_SAVED, program=saved.snapshot())
        return {"status": "saved", "program": saved.snapshot()}

    def update_program(
        self,
        program_id: str,
        changes: dict[str, Any] | None = None,
        program: dict[str, Any] | None = None,
        load: bool = False,
    ) -> dict[str, Any]:
        effective_changes = changes if changes is not None else program
        if not isinstance(effective_changes, dict):
            raise JsonRpcError(-32602, "Invalid params: expected program changes object")
        try:
            updated = self._program_store.update_program(program_id, effective_changes)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        except ValueError as exc:
            raise JsonRpcError(-32602, str(exc)) from exc
        loaded_program = self._state.loaded_program
        if load or (loaded_program is not None and loaded_program.program_id == updated.program_id):
            self._state.set_loaded_program(updated)
            self._events.publish(EventKind.PROGRAM_LOADED, program=updated.snapshot())
        self._events.publish(EventKind.PROGRAM_UPDATED, program=updated.snapshot())
        return {"status": "updated", "program": updated.snapshot()}

    def load_program(self, program_id: str) -> dict[str, Any]:
        try:
            program = self._program_store.get_program(program_id)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        self._state.set_loaded_program(program)
        self._events.publish(EventKind.PROGRAM_LOADED, program=program.snapshot())
        return {"status": "loaded", "program": program.snapshot()}

    def delete_program(self, program_id: str) -> dict[str, Any]:
        current_program = self._state.current_program
        if current_program is not None and current_program.program_id == program_id:
            raise JsonRpcError(-32000, "Cannot delete a program while it is executing")
        try:
            self._program_store.delete_program(program_id)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        loaded_program = self._state.loaded_program
        if loaded_program is not None and loaded_program.program_id == program_id:
            self._state.set_loaded_program(None)
        self._events.publish(EventKind.PROGRAM_DELETED, program_id=program_id)
        return {"status": "deleted", "program_id": program_id}

    def list_revisions(self, program_id: str) -> dict[str, Any]:
        try:
            revisions = self._program_store.list_revisions(program_id)
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        return {"program_id": program_id, "revisions": revisions}

    def restore_revision(self, program_id: str, revision: int) -> dict[str, Any]:
        try:
            restored = self._program_store.restore_revision(program_id, int(revision))
        except ProgramNotFoundError as exc:
            raise JsonRpcError(-32004, str(exc)) from exc
        self._events.publish(EventKind.PROGRAM_SAVED, program=restored.snapshot())
        return {"status": "restored", "program": restored.snapshot()}

    # ── Helpers ────────────────────────────────────────────────────────────

    def _coerce_program(self, program: dict[str, Any]) -> WindingProgram:
        if not isinstance(program, dict):
            raise JsonRpcError(-32602, "Invalid params: expected program object")
        return WindingProgram.from_payload(program)
