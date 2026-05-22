from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Any, Iterator

from core.coordinator import MotionStopPlan
from core.events import EventBus, EventKind
from motion.axis_state import AxisState
from motion.move import BaseMove, CompositeMove, Move, RampMove
from motion.multi_axis_segment_generator import AxisMotionConfig
from winding.wound_move import SynchronizedMove
from transport.messages import (
    LATERAL_ENDSTOP_ABSENT,
    LATERAL_ENDSTOP_PRESENT_CLOSED,
    LATERAL_ENDSTOP_PRESENT_OPEN,
    MultiAxisSegment,
    SpiMessageResult,
    sequence_is_greater,
)
from transport.streamer import MultiAxisRampStreamer, StreamAxisConfig
from transport.spi_transport import Esp32SpiTransport

_MAX_HISTORY = 50
_ENDSTOP_RELEASE_TIMEOUT_S = 3.0
_ENDSTOP_OPEN_CONFIRM_SAMPLES = 3
_ENDSTOP_OPEN_CONFIRM_INTERVAL_S = 0.015
_INITIAL_ENDSTOP_CONFIRM_SAMPLES = 3
_INITIAL_ENDSTOP_CONFIRM_INTERVAL_S = 0.01
_MULTI_AXIS_QUEUE_DEPTH = 64
_HOMING_RELEASE_RETRY_COUNT = 3


logger = logging.getLogger(__name__)


class MoveQueue:
    """
    Executes Move objects in FIFO order, one at a time, in a daemon thread.

    Each Move produces segments which are fed to a fresh MultiAxisRampStreamer.
    HomingMove (via CompositeMove) is handled specially: MoveQueue executes its phases in order,
    arming/disarming the endstop between phases and updating AxisState after
    the search phase completes.

    Thread safety:
      enqueue() is safe to call from any thread.
      start() / stop() are safe to call from any thread.
      The execution thread is a single daemon thread.
    """

    def __init__(
        self,
        transport: Esp32SpiTransport,
        axis_states: dict[int, AxisState],
        *,
        event_bus: EventBus | None = None,
        poll_interval_s: float = 0.001,
        print_every: int = 1,
    ) -> None:
        self._transport = transport
        self._axis_states = axis_states
        self._events = event_bus
        self._poll_interval_s = poll_interval_s
        self._print_every = print_every

        self._queue: deque[BaseMove] = deque()
        self._queue_lock = threading.Lock()
        self._queue_event = threading.Event()

        self._stop_requested = False
        self._thread: threading.Thread | None = None
        self._current_move: BaseMove | None = None
        self._current_streamer: MultiAxisRampStreamer | None = None
        self._history: list[BaseMove] = []
        self._active_stop_plan: MotionStopPlan | None = None
        self._last_worker_error: str | None = None

    # ── Public API ───────────────────────────────────────────────────────

    def enqueue(self, move: BaseMove) -> None:
        """Add a move to the queue. Safe to call from any thread."""
        with self._queue_lock:
            self._queue.append(move)
        self._queue_event.set()

    def start(self) -> None:
        """Start the execution thread."""
        self._stop_requested = False
        self._last_worker_error = None
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="move_queue"
        )
        self._thread.start()

    def stop(self, timeout_s: float = 3.0) -> None:
        """
        Request stop. The current move is aborted, the queue is cleared.
        Blocks until the execution thread exits.
        """
        self._stop_requested = True
        self._request_current_streamer_stop(
            MotionStopPlan.stop(
                self._axis_states,
                self._current_move.axis_ids if self._current_move is not None else None,
                reason="move queue stop requested",
            )
        )
        self._queue_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout_s)
        if self._thread is not None and self._thread.is_alive():
            self._last_worker_error = (
                f"move queue worker did not stop within {timeout_s:.1f}s"
            )
            raise RuntimeError(self._last_worker_error)

    def clear(self, stop_plan: MotionStopPlan | None = None) -> None:
        """Remove all pending moves from the queue without stopping."""
        cleared_moves: list[BaseMove] = []
        effective_plan = stop_plan or self._active_stop_plan
        with self._queue_lock:
            cleared_moves = list(self._queue)
            self._queue.clear()
        for move in cleared_moves:
            if not move.done:
                reason = "queue cleared"
                if effective_plan is not None:
                    reason = f"{effective_plan.mode.value} requested before execution"
                move.mark_aborted(reason)
            self._append_history(move)
        self._active_stop_plan = effective_plan
        self._request_current_streamer_stop(effective_plan)

    @property
    def current_move(self) -> BaseMove | None:
        return self._current_move

    @property
    def pending_count(self) -> int:
        with self._queue_lock:
            return len(self._queue)

    def status(self) -> dict[str, Any]:
        with self._queue_lock:
            queue_snapshot = [m.snapshot() for m in self._queue]
        return {
            "running": self._thread is not None and self._thread.is_alive(),
            "worker_faulted": self._last_worker_error is not None,
            "worker_error": self._last_worker_error,
            "current_move": (
                self._current_move.snapshot() if self._current_move else None
            ),
            "pending_moves": queue_snapshot,
            "history": [m.snapshot() for m in self._history[-10:]],
            "active_stop_plan": (
                None if self._active_stop_plan is None else self._active_stop_plan.snapshot()
            ),
            "axis_states": {
                ax_id: state.snapshot()
                for ax_id, state in self._axis_states.items()
            },
        }

    def worker_health(self) -> dict[str, Any]:
        return {
            "name": "move_queue",
            "thread_alive": self._thread is not None and self._thread.is_alive(),
            "thread_faulted": self._last_worker_error is not None,
            "last_error": self._last_worker_error,
        }

    def wait_until_idle(
        self,
        *,
        poll_interval_s: float | None = None,
        timeout_s: float = 60.0,
    ) -> None:
        """Block until no move is running and no move remains queued."""
        poll_interval = self._poll_interval_s if poll_interval_s is None else poll_interval_s
        deadline = time.monotonic() + timeout_s
        while self.current_move is not None or self.pending_count > 0:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"move queue did not drain after {timeout_s:.1f} s"
                )
            time.sleep(poll_interval)

    # ── Execution thread ─────────────────────────────────────────────────

    def _run(self) -> None:
        try:
            while not self._stop_requested:
                self._queue_event.wait(timeout=1.0)
                self._queue_event.clear()

                while not self._stop_requested:
                    with self._queue_lock:
                        if not self._queue:
                            break
                        move = self._queue.popleft()

                    self._current_move = move
                    try:
                        self._execute_move(move)
                    finally:
                        self._current_move = None
                        self._active_stop_plan = None

                    self._append_history(move)
        except Exception as exc:
            self._last_worker_error = str(exc)
            logger.exception("move queue worker failed")
            if self._current_move is not None and not self._current_move.done:
                self._current_move.mark_failed(self._last_worker_error)
            if self._events is not None:
                self._events.publish(
                    EventKind.WORKER_FAILED,
                    worker="move_queue",
                    error=self._last_worker_error,
                )

    def _append_history(self, move: BaseMove) -> None:
        self._history.append(move)
        if len(self._history) > _MAX_HISTORY:
            self._history = self._history[-_MAX_HISTORY:]

    def _execute_move(self, move: BaseMove) -> None:
        """Dispatch to the correct executor based on move type."""
        if self._stop_requested:
            move.mark_aborted("stop requested before execution")
            return
        try:
            if isinstance(move, CompositeMove):
                self._execute_composite(move)
            elif isinstance(move, SynchronizedMove):
                self._execute_wound_move(move)
            elif isinstance(move, Move):
                self._execute_ramp_move(move)
            else:
                move.mark_failed(f"unsupported move type: {type(move).__name__}")
        except Exception as exc:
            move.mark_failed(str(exc))

    def _axes_to_keep_enabled(self, axis_ids: list[int]) -> set[int]:
        keep_enabled: set[int] = set()
        for axis_id in axis_ids:
            axis_state = self._axis_states.get(axis_id)
            if axis_state is not None and axis_state.homed:
                keep_enabled.add(axis_id)
        return keep_enabled

    def _request_current_streamer_stop(
        self,
        stop_plan: MotionStopPlan | None = None,
    ) -> None:
        streamer = self._current_streamer
        if streamer is not None:
            if stop_plan is not None:
                self._active_stop_plan = stop_plan
                streamer.request_stop(keep_enabled_axes=set(stop_plan.keep_enabled_axes))
                return
            streamer.request_stop()

    def _apply_stop_plan(self, axis_ids: list[int], stop_plan: MotionStopPlan) -> None:
        for axis_id in axis_ids:
            if axis_id not in stop_plan.invalidate_positions:
                continue
            axis_state = self._axis_states.get(axis_id)
            if axis_state is not None:
                axis_state.invalidate_position()

    def _default_stop_plan(self, axis_ids: list[int], reason: str) -> MotionStopPlan:
        return MotionStopPlan.stop(self._axis_states, axis_ids, reason=reason)

    def _make_streamer(
        self,
        axis_configs: list[AxisMotionConfig],
        *,
        keep_enabled_axes: set[int] | None = None,
        initial_segments_dropped: int = 0,
    ) -> MultiAxisRampStreamer:
        """Create a fresh streamer for a list of AxisMotionConfig."""
        return MultiAxisRampStreamer(
            self._transport,
            [
                StreamAxisConfig(axis_id=cfg.axis_id, ramp=cfg.ramp)
                for cfg in axis_configs
            ],
            poll_interval_s=self._poll_interval_s,
            print_every=self._print_every,
            target_buffer_time_s=0.200,
            keep_enabled_axes=keep_enabled_axes,
            initial_segments_dropped=initial_segments_dropped,
        )

    def _next_motion_sequence(self) -> int:
        status = self._read_status(allow_stale=False)
        last_executed = int(getattr(status, "last_executed_sequence", -1))
        if last_executed == 0xFFFF or last_executed < 0:
            return 0
        return (last_executed + 1) & 0xFFFF

    def _update_axis_endstop_state(self, axis_id: int, status: Any) -> None:
        axis_state = self._axis_states.get(axis_id)
        if axis_state is None:
            return
        endstop_state = getattr(status, "lateral_endstop_state", None)
        if endstop_state is None:
            return
        axis_state.update_endstop_state(int(endstop_state))

    def _axis_mask(self, axis_id: int) -> int:
        return 1 << axis_id

    def _status_has_endstop_armed(self, status: Any, axis_id: int, arm: bool) -> bool:
        armed_mask = int(getattr(status, "endstop_armed_mask", 0))
        return bool(armed_mask & self._axis_mask(axis_id)) is arm

    def _read_status(
        self,
        axis_id: int | None = None,
        *,
        allow_stale: bool = True,
    ) -> Any:
        status = self._transport.get_status(allow_stale=allow_stale)
        if axis_id is not None:
            self._update_axis_endstop_state(axis_id, status)
        return status

    def _wait_for_transport_request_result(
        self,
        sequence: int,
        *,
        send_status: Any | None = None,
    ) -> Any:
        if send_status is not None:
            return self._transport.wait_for_request_result(
                sequence,
                hint_status=send_status,
                poll_interval_s=self._poll_interval_s,
            )
        return self._transport.wait_for_request_result(
            sequence,
            poll_interval_s=self._poll_interval_s,
        )

    def _confirm_initial_closed_endstop(self, axis_id: int) -> int:
        """Confirm a startup CLOSED reading before launching preclear.

        A single stale or noisy status snapshot at homing start should not send the
        axis into preclear. Require a few consecutive CLOSED reads; otherwise treat
        the startup state as the most recent non-CLOSED observation.
        """
        confirmed_state = LATERAL_ENDSTOP_PRESENT_CLOSED
        for _ in range(_INITIAL_ENDSTOP_CONFIRM_SAMPLES - 1):
            time.sleep(_INITIAL_ENDSTOP_CONFIRM_INTERVAL_S)
            status = self._read_status(axis_id, allow_stale=False)
            state = int(
                getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
            )
            if state != LATERAL_ENDSTOP_PRESENT_CLOSED:
                logger.warning(
                    "homing axis %s: startup CLOSED state was not stable; using latest state 0x%02X",
                    axis_id,
                    state,
                )
                confirmed_state = state
                break
        return confirmed_state

    def _ensure_homing_can_start(self, axis_id: int, phase_name: str) -> None:
        status = self._read_status(axis_id, allow_stale=False)
        lateral_state = int(getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT))
        if lateral_state == LATERAL_ENDSTOP_ABSENT:
            raise RuntimeError(
                f"homing {phase_name} cannot start on axis {axis_id}: "
                "lateral endstop sensor is ABSENT (cable disconnected or not installed)"
            )
        if lateral_state != LATERAL_ENDSTOP_PRESENT_OPEN:
            raise RuntimeError(
                f"homing {phase_name} cannot start on axis {axis_id}: "
                f"lateral_endstop_state=0x{lateral_state:02X} (expected PRESENT_OPEN)"
            )

    @staticmethod
    def _compute_backoff_timeout(sub_move: RampMove, margin: float = 1.5) -> float:
        """Return a streaming timeout for a backoff/preclear RampMove.

        timeout = total_duration × margin + 0.5 s recovery guard, minimum 1.0 s.
        """
        total_s = sub_move.axis_configs[0].ramp.total_duration
        return max(1.0, total_s * margin + 0.5)

    def _wait_for_post_hit_recovery(
        self,
        axis_id: int,
        *,
        stop_timeout_s: float = 1.0,
        recovery_guard_s: float = 0.150,
        drain_timeout_s: float = 1.0,
    ) -> Any:
        """Wait for the firmware to finish stop + RECOVERY after an endstop hit.

        The approach phase stops asynchronously in firmware: the executor still
        has to finish its emergency-stop / planner-flush / RECOVERY cycle after
        the host-side streamer observes ``endstop_triggered``.  Starting the
        backoff too early can cause the first reverse segments to be drained
        before the executor returns to IDLE, leaving the switch physically
        closed and causing a host-side timeout on ``_wait_for_endstop_open``.
        """
        deadline = time.monotonic() + stop_timeout_s
        last_status = None
        while time.monotonic() < deadline:
            last_status = self._read_status(axis_id, allow_stale=False)
            running = int(getattr(last_status, "running_mask", 0))
            if (running & (1 << axis_id)) == 0:
                break
            time.sleep(0.010)
        else:
            running_mask = int(getattr(last_status, "running_mask", 0xFF)) if last_status else 0xFF
            raise RuntimeError(
                f"post-hit stop timeout on axis {axis_id}: running_mask=0x{running_mask:02X}"
            )

        quiescent_deadline = time.monotonic() + drain_timeout_s
        quiescent_since: float | None = None
        status = last_status
        while time.monotonic() < quiescent_deadline:
            status = self._read_status(axis_id, allow_stale=False)
            running_mask = int(getattr(status, "running_mask", 0))
            axis_stopped = (running_mask & (1 << axis_id)) == 0

            ring_free = tuple(int(v) for v in getattr(status, "ring_free_slots", ()))
            axis_ring_empty = (
                0 <= axis_id < len(ring_free)
                and ring_free[axis_id] >= MultiAxisRampStreamer.STEP_RING_CAPACITY
            )

            planner_free = int(
                getattr(
                    status,
                    "planner_queue_free",
                    MultiAxisRampStreamer.SEGMENT_QUEUE_DEPTH,
                )
            )
            planner_empty = planner_free >= MultiAxisRampStreamer.SEGMENT_QUEUE_DEPTH

            multi_axis_free = int(
                getattr(status, "multi_axis_queue_free", _MULTI_AXIS_QUEUE_DEPTH)
            )
            multi_axis_empty = multi_axis_free >= _MULTI_AXIS_QUEUE_DEPTH

            if axis_stopped and axis_ring_empty and planner_empty and multi_axis_empty:
                if quiescent_since is None:
                    quiescent_since = time.monotonic()
                elif (time.monotonic() - quiescent_since) >= recovery_guard_s:
                    break
            else:
                quiescent_since = None

            time.sleep(0.010)
        else:
            running_mask = int(getattr(status, "running_mask", 0xFF)) if status else 0xFF
            ring_repr = tuple(int(v) for v in getattr(status, "ring_free_slots", ())) if status else ()
            planner_free = int(getattr(status, "planner_queue_free", -1)) if status else -1
            multi_axis_free = int(getattr(status, "multi_axis_queue_free", -1)) if status else -1
            raise RuntimeError(
                f"post-hit recovery did not quiesce on axis {axis_id}: "
                f"running_mask=0x{running_mask:02X}, "
                f"ring_free={ring_repr}, "
                f"planner_queue_free={planner_free}, "
                f"multi_axis_queue_free={multi_axis_free}"
            )

        lateral_state = int(
            getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
        )
        if lateral_state == LATERAL_ENDSTOP_ABSENT:
            raise RuntimeError(
                f"endstop became ABSENT after hit on axis {axis_id} — check wiring"
            )
        if lateral_state == LATERAL_ENDSTOP_PRESENT_OPEN:
            logger.warning(
                "endstop on axis %s returned OPEN during post-hit recovery "
                "(possible bounce or fast backoff) — proceeding with backoff",
                axis_id,
            )
        return status

    def _wait_for_endstop_latch_cleared(
        self,
        axis_id: int,
        *,
        timeout_s: float = 0.5,
    ) -> None:
        """Wait until endstop_hit_mask reports no hit for the given axis.

        ENABLE_ENDSTOP(arm=1) clears the firmware latch, but the cleared state
        may not be visible in the status payload until the next SPI exchange.
        Starting a homing stream before confirming the latch is clear risks the
        streamer detecting a stale hit immediately and terminating the phase
        without any real movement.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self._read_status(axis_id, allow_stale=False)
            hit_mask = int(getattr(status, "endstop_hit_mask", 0xFF))
            if (hit_mask & (1 << axis_id)) == 0:
                return
            time.sleep(0.010)
        raise RuntimeError(
            f"endstop latch not cleared after arm on axis {axis_id}: "
            f"endstop_hit_mask still set after {timeout_s:.1f}s"
        )

    def _wait_for_endstop_arm_state(
        self,
        axis_id: int,
        arm: bool,
        timeout_s: float | None = None,
    ) -> Any:
        # R8: timeout adaptatif — 20 cycles SPI minimum, jamais moins de 0.5s.
        # À poll_interval_s=0.001, 20 cycles = 20ms (3 aller-retours SPI suffisent).
        if timeout_s is None:
            timeout_s = max(0.5, 20 * self._poll_interval_s)
        deadline = time.monotonic() + timeout_s
        last_status = None
        while time.monotonic() < deadline:
            last_status = self._read_status(axis_id, allow_stale=False)
            if self._status_has_endstop_armed(last_status, axis_id, arm):
                return last_status
            time.sleep(self._poll_interval_s)
        armed_mask = int(getattr(last_status, "endstop_armed_mask", 0)) if last_status else -1
        raise RuntimeError(
            f"endstop arm state timeout on axis {axis_id}: "
            f"wanted arm={arm}, "
            f"endstop_armed_mask=0x{armed_mask:02X} after {timeout_s:.2f}s"
        )

    def _wait_for_endstop_open(self, axis_id: int, timeout_s: float = _ENDSTOP_RELEASE_TIMEOUT_S) -> Any:
        deadline = time.monotonic() + timeout_s
        consecutive_open = 0
        last_status = None
        while time.monotonic() < deadline:
            last_status = self._read_status(axis_id, allow_stale=False)
            state = int(
                getattr(last_status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
            )
            if state == LATERAL_ENDSTOP_PRESENT_OPEN:
                consecutive_open += 1
                if consecutive_open >= _ENDSTOP_OPEN_CONFIRM_SAMPLES:
                    return last_status
            else:
                consecutive_open = 0
            time.sleep(_ENDSTOP_OPEN_CONFIRM_INTERVAL_S)
        raise RuntimeError(
            f"endstop release timeout on axis {axis_id}: state=0x{int(getattr(last_status, 'lateral_endstop_state', 0xFF)):02X}"
        )

    def _stream_homing_sub_move(
        self,
        move: CompositeMove,
        *,
        phase_name: str,
        sub_move: Move,
        arm_endstop: bool,
        start_sequence: int | None = None,
    ) -> MultiAxisRampStreamer:
        sub_move_axis_configs = sub_move.axis_configs
        if not sub_move_axis_configs:
            raise RuntimeError(
                f"homing sub-move {phase_name} has no public axis_configs"
            )

        baseline_status = self._read_status(move.axis_id, allow_stale=False)
        baseline_segments_dropped = int(
            getattr(baseline_status, "segments_dropped", 0)
        )

        if start_sequence is None:
            start_sequence = self._next_motion_sequence()

        streamer = self._make_streamer(
            sub_move_axis_configs,
            keep_enabled_axes={move.axis_id},
            initial_segments_dropped=baseline_segments_dropped,
        )
        self._current_streamer = streamer
        streamer.note_endstop_armed(move.axis_id, arm_endstop)
        streamer.set_homing_mode(True)
        streamer.set_generator(
            self._wrap_segment_sequence(
                sub_move.segments(),
                start_sequence,
            )
        )
        try:
            streamer.stream_all()
        finally:
            self._current_streamer = None
        return streamer

    def _next_sequence_after_streamer(self, streamer: MultiAxisRampStreamer) -> int:
        last_sent = streamer.last_sent_motion_seq
        if last_sent >= 0:
            return (last_sent + 1) & 0xFFFF
        next_from_status = self._next_motion_sequence()
        flush_floor = streamer.flush_floor_sequence
        if flush_floor < 0:
            return next_from_status
        candidate = (flush_floor + 1) & 0xFFFF
        if next_from_status < 0 or sequence_is_greater(candidate, next_from_status):
            return candidate
        return next_from_status

    @staticmethod
    def _streamer_stop_requested(streamer: MultiAxisRampStreamer) -> bool:
        return streamer.has_stop_been_requested()

    def _clear_closed_endstop_before_homing(self, move: CompositeMove) -> None:
        logger.info(
            "homing axis %s: endstop already closed at start, running preclear",
            move.axis_id,
        )
        clearance_move = move.preclear_move()
        last_error: RuntimeError | None = None
        for attempt in range(1, _HOMING_RELEASE_RETRY_COUNT + 1):
            if attempt > 1:
                logger.warning(
                    "homing axis %s: preclear retry %s/%s after CLOSED endstop",
                    move.axis_id,
                    attempt,
                    _HOMING_RELEASE_RETRY_COUNT,
                )

            self._set_endstop_armed(move.axis_id, arm=False)
            streamer = self._stream_homing_sub_move(
                move,
                phase_name="preclear",
                sub_move=clearance_move,
                arm_endstop=False,
            )
            if streamer.endstop_triggered:
                raise RuntimeError(
                    f"preclear failed: unexpected endstop trigger on axis {move.axis_id}"
                )

            try:
                self._wait_for_endstop_open(
                    move.axis_id,
                    timeout_s=self._compute_backoff_timeout(clearance_move),
                )
            except RuntimeError as exc:
                last_error = exc
                continue

            # R5: attente de stabilisation mécanique (debounce) — 2 cycles SPI minimum
            # pour que le GPIO se stabilise après relâchement du contact physique.
            time.sleep(0.020)
            # Relecture finale pour confirmer l'état avant d'armer la phase approach.
            final_status = self._read_status(move.axis_id, allow_stale=False)
            lateral_state = int(
                getattr(final_status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
            )
            if lateral_state == LATERAL_ENDSTOP_PRESENT_OPEN:
                return

            last_error = RuntimeError(
                f"preclear did not clear the endstop on axis {move.axis_id}: "
                f"lateral_endstop_state=0x{lateral_state:02X} after stabilisation wait"
            )

        if last_error is not None:
            raise last_error

    def _check_armed_phase_result(
        self,
        move: CompositeMove,
        phase_name: str,
        streamer: "MultiAxisRampStreamer",
    ) -> None:
        """Vérifie qu'une phase armée s'est bien terminée par un déclenchement endstop.

        Lève RuntimeError avec un message diagnostique si ce n'est pas le cas.
        """
        if streamer.endstop_triggered:
            return  # succès nominal

        # Lire le status pour diagnostiquer
        status = self._read_status(move.axis_id, allow_stale=False)
        lateral_state = int(
            getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
        )
        last_exec = int(getattr(status, "last_executed_sequence", 0xFFFF))
        last_sent = streamer.last_sent_motion_seq
        running   = int(getattr(status, "running_mask", 0))

        state_names = {
            LATERAL_ENDSTOP_PRESENT_OPEN: "PRESENT_OPEN",
            LATERAL_ENDSTOP_PRESENT_CLOSED: "PRESENT_CLOSED",
            LATERAL_ENDSTOP_ABSENT: "ABSENT",
        }
        state_str = state_names.get(lateral_state, f"0x{lateral_state:02X}")

        raise RuntimeError(
            f"homing phase '{phase_name}' on axis {move.axis_id} ended without "
            f"endstop trigger. "
            f"lateral_state={state_str}, "
            f"running=0x{running:02X}, "
            f"last_exec={last_exec}, "
            f"last_sent={last_sent}"
        )

    def _set_endstop_armed(self, axis_id: int, arm: bool) -> Any:
        sequence, send_status = self._transport.enable_endstop_request(axis_id, arm=arm)
        status = self._wait_for_transport_request_result(
            sequence,
            send_status=send_status,
        )
        self._update_axis_endstop_state(axis_id, status)
        if int(getattr(status, "last_result", SpiMessageResult.OK)) != int(SpiMessageResult.OK):
            raise RuntimeError(
                f"enable_endstop axis {axis_id} arm={int(arm)} failed with result=0x{int(status.last_result):02X}"
            )
        if self._status_has_endstop_armed(status, axis_id, arm):
            return status
        return self._wait_for_endstop_arm_state(axis_id, arm)

    def _wrap_segment_sequence(self, generator: Iterator[MultiAxisSegment], start_sequence: int) -> Iterator[MultiAxisSegment]:
        sequence = start_sequence & 0xFFFF
        for segment in generator:
            segment.sequence = sequence
            yield segment
            sequence = (sequence + 1) & 0xFFFF

    def _finalize_streamer_move(
        self,
        move: Move,
        streamer: MultiAxisRampStreamer,
        axis_ids: list[int],
    ) -> None:
        """Handle post-stream outcome: endstop abort, stop request, or completion.

        Updates axis positions: advances if delta is known, invalidates if None.
        """
        if streamer.endstop_triggered:
            for ax_id in axis_ids:
                if ax_id in self._axis_states:
                    self._axis_states[ax_id].invalidate_position()
            move.mark_aborted("endstop triggered", by_endstop=True)
            return

        if self._stop_requested or streamer.has_stop_been_requested():
            stop_plan = self._active_stop_plan or self._default_stop_plan(
                axis_ids,
                "stop requested",
            )
            self._apply_stop_plan(axis_ids, stop_plan)
            move.mark_aborted(f"{stop_plan.mode.value} requested")
            return

        for ax_id in axis_ids:
            if ax_id not in self._axis_states:
                continue
            delta = move.expected_delta_steps(ax_id)
            if delta is None:
                self._axis_states[ax_id].invalidate_position()
                continue
            self._axis_states[ax_id].advance_position(delta)

        move.mark_completed()

    def _execute_ramp_move(self, move: Move) -> None:
        """Execute a RampMove via MultiAxisRampStreamer."""
        move.mark_running()
        axis_configs = move.axis_configs
        if not axis_configs:
            move.mark_failed(
                f"{type(move).__name__} has no axis_configs; use _execute_wound_move for synchronized moves"
            )
            return
        axis_ids = move.axis_ids

        streamer = self._make_streamer(
            axis_configs,
            keep_enabled_axes=self._axes_to_keep_enabled(axis_ids),
        )
        self._current_streamer = streamer
        # Override the generator to use the move's segments() method, but align
        # motion_sequence values with the ESP32 last_executed_sequence.
        streamer.set_generator(
            self._wrap_segment_sequence(
                move.segments(),
                self._next_motion_sequence(),
            )
        )

        try:
            streamer.stream_all()
        except Exception as exc:
            move.mark_failed(str(exc))
            return
        finally:
            self._current_streamer = None

        self._finalize_streamer_move(move, streamer, axis_ids)

    def _make_wound_streamer(self, move: SynchronizedMove, *, keep_enabled_axes: set[int] | None = None) -> MultiAxisRampStreamer:
        target_hz = (
            max(move.kinematics.target_rpm, 1.0) / 60.0
            * float(move.spindle_cfg.steps_per_unit)
        )
        stall_timeout_s = max(5.0, move.kinematics.total_duration * 2.0)
        return MultiAxisRampStreamer.from_axis_ids(
            self._transport,
            move.axis_ids,
            target_hz=max(target_hz, 1.0),
            segment_duration_s=move.segment_duration_s,
            poll_interval_s=self._poll_interval_s,
            stall_timeout_s=stall_timeout_s,
            print_every=self._print_every,
            target_buffer_time_s=0.200,
            keep_enabled_axes=keep_enabled_axes,
        )

    def _execute_wound_move(self, move: SynchronizedMove) -> None:
        """Execute a SynchronizedMove with explicit spindle/traverse streamer setup."""
        move.mark_running()
        axis_ids = move.axis_ids

        streamer = self._make_wound_streamer(
            move,
            keep_enabled_axes=self._axes_to_keep_enabled(axis_ids),
        )
        self._current_streamer = streamer
        streamer.set_generator(
            self._wrap_segment_sequence(
                move.segments(),
                self._next_motion_sequence(),
            )
        )

        try:
            streamer.stream_all()
        except Exception as exc:
            move.mark_failed(str(exc))
            return
        finally:
            self._current_streamer = None

        self._finalize_streamer_move(move, streamer, axis_ids)

    def _execute_composite(self, move: CompositeMove) -> None:
        """
        Execute a CompositeMove phase by phase via HomingPhaseDescriptor.

        Phase sequence is driven by move.phases():
          - approach (endstop armed) — stops when endstop fires
          - backoff  (endstop disarmed, wait_for_open) — moves away from endstop
          - search   (endstop armed) — slow approach for precise home
        After all phases: set_position to move.home_position_steps.
        The endstop is always disarmed in a finally block.
        """
        move.mark_running()
        axis_state = self._axis_states.get(move.axis_id)

        try:
            initial_status = self._read_status(move.axis_id, allow_stale=False)
            initial_state = int(
                getattr(initial_status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
            )
            if initial_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
                initial_state = self._confirm_initial_closed_endstop(move.axis_id)
            if initial_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
                self._clear_closed_endstop_before_homing(move)
            self._ensure_homing_can_start(move.axis_id, "start")
        except Exception as exc:
            move.mark_failed(str(exc))
            return

        next_sequence = self._next_motion_sequence()

        try:
            for descriptor in move.phases():
                logger.info(
                    "homing phase=%s arm=%s reverse=%s",
                    descriptor.name,
                    descriptor.arm_endstop,
                    descriptor.move.axis_configs[0].ramp.reverse_direction,
                )
                if self._stop_requested:
                    stop_plan = self._active_stop_plan or self._default_stop_plan(
                        [move.axis_id],
                        "stop requested during homing",
                    )
                    self._apply_stop_plan([move.axis_id], stop_plan)
                    move.mark_aborted(f"{stop_plan.mode.value} requested during homing")
                    return

                if descriptor.arm_endstop:
                    try:
                        self._ensure_homing_can_start(move.axis_id, descriptor.name)
                    except Exception as exc:
                        move.mark_failed(str(exc))
                        return

                # Arm or disarm endstop for this phase and verify the mask in status.
                try:
                    self._set_endstop_armed(move.axis_id, arm=descriptor.arm_endstop)
                    if descriptor.arm_endstop:
                        self._wait_for_endstop_latch_cleared(move.axis_id)
                except Exception as exc:
                    move.mark_failed(str(exc))
                    return

                try:
                    streamer = self._stream_homing_sub_move(
                        move,
                        phase_name=descriptor.name,
                        sub_move=descriptor.move,
                        arm_endstop=descriptor.arm_endstop,
                        start_sequence=next_sequence,
                    )
                except Exception as exc:
                    move.mark_failed(str(exc))
                    return

                next_sequence = self._next_sequence_after_streamer(streamer)

                phase_completed_on_expected_endstop = descriptor.arm_endstop and streamer.endstop_triggered
                if self._stop_requested or (
                    self._streamer_stop_requested(streamer)
                    and not phase_completed_on_expected_endstop
                ):
                    stop_plan = self._active_stop_plan or self._default_stop_plan(
                        [move.axis_id],
                        "stop requested during homing",
                    )
                    self._apply_stop_plan([move.axis_id], stop_plan)
                    move.mark_aborted(f"{stop_plan.mode.value} requested during homing")
                    return

                if descriptor.expect_endstop_hit:
                    try:
                        self._check_armed_phase_result(move, descriptor.name, streamer)
                    except RuntimeError as exc:
                        # Endstop did not fire — homing failed with diagnostics.
                        move.mark_failed(str(exc))
                        return

                if descriptor.expect_endstop_hit and streamer.endstop_triggered:
                    try:
                        self._wait_for_post_hit_recovery(move.axis_id)
                    except Exception as exc:
                        move.mark_failed(str(exc))
                        return

                if descriptor.wait_for_open:
                    release_timeout = self._compute_backoff_timeout(descriptor.move)
                    release_error: Exception | None = None
                    for release_attempt in range(1, _HOMING_RELEASE_RETRY_COUNT + 1):
                        try:
                            self._wait_for_endstop_open(
                                move.axis_id,
                                timeout_s=release_timeout,
                            )
                            release_error = None
                            break
                        except Exception as exc:
                            release_error = exc
                            if release_attempt >= _HOMING_RELEASE_RETRY_COUNT:
                                break
                            logger.warning(
                                "homing axis %s: backoff retry %s/%s after CLOSED endstop: %s",
                                move.axis_id,
                                release_attempt + 1,
                                _HOMING_RELEASE_RETRY_COUNT,
                                exc,
                            )
                            try:
                                self._set_endstop_armed(move.axis_id, arm=False)
                                retry_streamer = self._stream_homing_sub_move(
                                    move,
                                    phase_name=f"{descriptor.name}_retry_{release_attempt + 1}",
                                    sub_move=descriptor.move,
                                    arm_endstop=False,
                                )
                            except Exception as retry_exc:
                                move.mark_failed(str(retry_exc))
                                return

                            if retry_streamer.endstop_triggered:
                                move.mark_failed(
                                    f"{descriptor.name} retry failed: unexpected endstop trigger on axis {move.axis_id}"
                                )
                                return

                    if release_error is not None:
                        move.mark_failed(str(release_error))
                        return

                    # Do NOT wait for running_mask to drop here.
                    # In coast-mode streaming the firmware can keep the RMT
                    # transaction alive with pause symbols after the last real
                    # backoff step has executed. That leaves running_mask high
                    # even though the motor is physically idle and the endstop
                    # is already OPEN. The next homing phase can safely append
                    # new segments into the live stream, so the correct gate is
                    # the released endstop, not running_mask == 0.
                    self._read_status(move.axis_id, allow_stale=False)

                if self._stop_requested:
                    stop_plan = self._active_stop_plan or self._default_stop_plan(
                        [move.axis_id],
                        "stop requested during homing",
                    )
                    self._apply_stop_plan([move.axis_id], stop_plan)
                    move.mark_aborted(f"{stop_plan.mode.value} requested during homing")
                    return

        finally:
            self._set_endstop_armed(move.axis_id, arm=False)

        if axis_state is not None:
            axis_state.mark_homed(move.home_position_steps)

        move.mark_completed()
