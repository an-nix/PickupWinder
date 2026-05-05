from __future__ import annotations
import logging
import queue
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any


logger = logging.getLogger(__name__)


class EventKind(Enum):
    PROGRAM_STARTED    = auto()
    PROGRAM_COMPLETED  = auto()
    PROGRAM_ABORTED    = auto()
    PROGRAM_FAILED     = auto()
    WORKER_FAILED      = auto()
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
    version: int = 1
    data: dict[str, Any] = field(default_factory=dict)


class EventBus:
    """
    Multi-producer, single-consumer queue.

    All events are non-blocking on the producer side — if the consumer is slow,
    old events are dropped rather than blocking motion or RPC worker threads.
    The queue is capped at MAX_EVENTS to prevent unbounded growth.
    """
    MAX_EVENTS = 256

    def __init__(self) -> None:
        self._q: queue.Queue[Event] = queue.Queue(maxsize=self.MAX_EVENTS)
        self._dropped_count: int = 0

    def publish(self, kind: EventKind, **data: Any) -> None:
        """Non-blocking publish. Drops event if queue is full."""
        try:
            self._q.put_nowait(Event(kind=kind, data=data))
        except queue.Full:
            self._dropped_count += 1
            if self._dropped_count % 10 == 1:
                logger.warning(
                    "EventBus full: dropped event %s (total drops: %d)",
                    kind.name,
                    self._dropped_count,
                )

    @property
    def dropped_count(self) -> int:
        return self._dropped_count

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
