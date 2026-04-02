"""CommandController - async command queue, normalization, and dispatch to SessionController.

Mirrors CommandController.cpp / CommandController.h.
Commands are enqueued from WebSocket/UART handlers (running in asyncio tasks)
and drained once per control tick into the SessionController.
"""
import asyncio
from command_registry import CommandRegistry, CommandId

_REGISTRY = CommandRegistry()

# Commands that must bypass the queue and go straight through (immediate)
_IMMEDIATE_IDS = {
    CommandId.Stop, CommandId.EmergencyStop, CommandId.Reset,
}


class CommandController:
    """Thread-safe (asyncio) command queue with validation."""

    def __init__(self, session_controller):
        self._session = session_controller
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=32)
        self._pushed = 0
        self._dropped = 0
        self._processed = 0

    # ── Push (from WebSocket / UART task) ─────────────────────────────────────
    def push_command(self, raw_key: str, raw_value: str):
        """Validate and enqueue a command.  Returns True if accepted."""
        ok, cmd_id, norm_value, err = _REGISTRY.normalize_and_validate(raw_key, raw_value)
        if not ok:
            print(f"[CMD] Rejected '{raw_key}={raw_value}': {err}")
            return False
        try:
            self._queue.put_nowait((cmd_id, norm_value))
            self._pushed += 1
            return True
        except asyncio.QueueFull:
            self._dropped += 1
            print(f"[CMD] Queue full - dropped '{cmd_id}'")
            return False

    # ── Drain into TickInput (called from control loop) ───────────────────────
    def drain(self, tick_input):
        """
        Move all pending commands into tick_input.pending_commands.
        Direct-dispatch for IMMEDIATE commands.
        tick_input: TickInput instance from session_controller module.
        """
        while True:
            try:
                cmd_id, value = self._queue.get_nowait()
                tick_input.pending_commands.append((cmd_id, value))
                self._processed += 1
            except asyncio.QueueEmpty:
                break

    # ── Stats ──────────────────────────────────────────────────────────────────
    def stats(self) -> dict:
        return {
            "qPushed": self._pushed,
            "qDropped": self._dropped,
            "qProcessed": self._processed,
            "qPending": self._queue.qsize(),
        }
