from __future__ import annotations
import queue
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any


class EventKind(Enum):
    PROGRAM_STARTED    = auto()
    PROGRAM_COMPLETED  = auto()
    PROGRAM_ABORTED    = auto()
    PROGRAM_FAILED     = auto()
    LAYER_STARTED      = auto()
    LAYER_COMPLETED    = auto()
    HOMING_STARTED     = auto()
    HOMING_COMPLETED   = auto()
    HOMING_FAILED      = auto()
    ENDSTOP_TRIGGERED  = auto()
    AXIS_STATE_CHANGED = auto()
    STATUS_UPDATE      = auto()


@dataclass(slots=True)
class Event:
    kind: EventKind
    data: dict[str, Any] = field(default_factory=dict)


class EventBus:
    """
    Single-producer (WindingEngine), single-consumer (JsonRpcServer) queue.
    All events are non-blocking on the producer side — if the consumer
    is slow, old events are dropped rather than blocking the engine.
    The queue is capped at MAX_EVENTS to prevent unbounded growth.
    """
    MAX_EVENTS = 256

    def __init__(self) -> None:
        self._q: queue.Queue[Event] = queue.Queue(maxsize=self.MAX_EVENTS)

    def publish(self, kind: EventKind, **data: Any) -> None:
        """Non-blocking publish. Drops event if queue is full."""
        try:
            self._q.put_nowait(Event(kind=kind, data=data))
        except queue.Full:
            pass

    def consume(self, timeout_s: float = 0.1) -> Event | None:
        """Blocking consume with timeout. Returns None on timeout."""
        try:
            return self._q.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def drain(self) -> list[Event]:
        """Return all currently queued events without blocking."""
        events = []
        while True:
            try:
                events.append(self._q.get_nowait())
            except queue.Empty:
                break
        return events
