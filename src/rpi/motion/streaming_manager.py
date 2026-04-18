"""Background streaming manager for non-blocking motion control."""

from __future__ import annotations

import logging
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Any, Callable

from transport import MultiAxisRampStreamer, StreamAxisConfig

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class StreamingSession:
    """Represents an active or completed streaming session."""
    session_id: int
    started_at: float
    completed_at: float | None = None
    block_count: int = 0
    error: str | None = None
    status: str = "queued"  # queued, running, completed, failed


class StreamingManager:
    """Manages background streaming threads to prevent JSON-RPC handler blocking."""

    def __init__(self, max_concurrent: int = 4) -> None:
        self.max_concurrent = max_concurrent
        self._lock = threading.Lock()
        self._session_counter = 0
        self._active_threads: dict[int, threading.Thread] = {}
        self._sessions: dict[int, StreamingSession] = {}
        self._next_session_id = 1

    def stream_async(
        self,
        streamer: MultiAxisRampStreamer,
        name: str = "stream",
        on_complete: Callable[[int, StreamingSession], None] | None = None,
    ) -> int:
        """Start streaming in background thread. Returns session_id."""
        with self._lock:
            session_id = self._next_session_id
            self._next_session_id += 1

            session = StreamingSession(
                session_id=session_id,
                started_at=time.monotonic(),
                status="queued",
            )
            self._sessions[session_id] = session

        logger.info(f"Starting streaming session {session_id}: {name}")

        def _stream_worker() -> None:
            try:
                session.status = "running"
                logger.info(f"Session {session_id}: Starting stream_all()")
                block_count = streamer.stream_all()
                session.block_count = block_count
                session.status = "completed"
                session.completed_at = time.monotonic()
                logger.info(f"Session {session_id}: Completed successfully ({block_count} blocks)")
            except Exception as e:
                session.error = str(e)
                session.status = "failed"
                session.completed_at = time.monotonic()
                logger.error(f"Session {session_id}: Streaming failed: {e}")
                logger.error(f"Traceback: {traceback.format_exc()}")
            finally:
                if on_complete:
                    on_complete(session_id, session)
                with self._lock:
                    self._active_threads.pop(session_id, None)

        thread = threading.Thread(
            target=_stream_worker,
            name=f"{name}-{session_id}",
            daemon=True,
        )
        with self._lock:
            self._active_threads[session_id] = thread
        thread.start()

        return session_id

    def get_session(self, session_id: int) -> StreamingSession | None:
        """Retrieve session info."""
        with self._lock:
            return self._sessions.get(session_id)

    def wait_session(
        self,
        session_id: int,
        timeout_s: float | None = None,
    ) -> StreamingSession:
        """Block until session completes."""
        start = time.monotonic()
        while True:
            with self._lock:
                session = self._sessions.get(session_id)
                if session is None:
                    raise ValueError(f"Session {session_id} not found")
                thread = self._active_threads.get(session_id)

            if session.status in ("completed", "failed"):
                return session

            if timeout_s is not None:
                elapsed = time.monotonic() - start
                if elapsed > timeout_s:
                    raise TimeoutError(f"Streaming session {session_id} timed out after {timeout_s}s")

            time.sleep(0.01)

    def active_count(self) -> int:
        """Number of currently active threads."""
        with self._lock:
            return len(self._active_threads)
