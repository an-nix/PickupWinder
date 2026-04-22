from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Any

from core.coordinator import MotionStopPlan
from core.events import EventBus, EventKind
from motion.axis_state import AxisState
from motion.move import BaseMove, CompositeMove, HomingMove, Move
from winding.wound_move import WoundMove
from transport.messages import (
    LATERAL_ENDSTOP_ABSENT,
    LATERAL_ENDSTOP_PRESENT_CLOSED,
    LATERAL_ENDSTOP_PRESENT_OPEN,
    SpiMessageResult,
    sequence_is_greater,
)
from transport.streamer import MultiAxisRampStreamer, StreamAxisConfig
from transport.spi_transport import Esp32SpiTransport

_MAX_HISTORY = 50
_ENDSTOP_VERIFY_TIMEOUT_S = 0.5
_ENDSTOP_RELEASE_TIMEOUT_S = 1.5
_INITIAL_ENDSTOP_CONFIRM_SAMPLES = 3
_INITIAL_ENDSTOP_CONFIRM_INTERVAL_S = 0.01


logger = logging.getLogger(__name__)


class MoveQueue:
    """
    Executes Move objects in FIFO order, one at a time, in a daemon thread.

    Each Move produces segments which are fed to a fresh MultiAxisRampStreamer.
    HomingMove is handled specially: MoveQueue executes its phases in order,
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
                self._execute_homing(move)  # type: ignore[arg-type]
            elif isinstance(move, WoundMove) or getattr(move, "is_synchronized_move", False):
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
        axis_configs,
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
        status = self._transport.get_status()
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

    def _read_status(self, axis_id: int | None = None) -> Any:
        status = self._transport.get_status()
        if axis_id is not None:
            self._update_axis_endstop_state(axis_id, status)
        return status

    def _confirm_initial_closed_endstop(self, axis_id: int) -> int:
        """Confirm a startup CLOSED reading before launching preclear.

        A single stale or noisy status snapshot at homing start should not send the
        axis into preclear. Require a few consecutive CLOSED reads; otherwise treat
        the startup state as the most recent non-CLOSED observation.
        """
        confirmed_state = LATERAL_ENDSTOP_PRESENT_CLOSED
        for _ in range(_INITIAL_ENDSTOP_CONFIRM_SAMPLES - 1):
            time.sleep(_INITIAL_ENDSTOP_CONFIRM_INTERVAL_S)
            status = self._read_status(axis_id)
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
        status = self._read_status(axis_id)
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
    def _compute_backoff_timeout(sub_move: Move, margin: float = 1.5) -> float:
        """Calcule un timeout basé sur la durée estimée du sous-mouvement.

        Cherche les attributs dans l'ordre de priorité suivant :
          1. sub_move.estimated_duration_s  (attribut ajouté par HomingMove)
          2. sub_move.ramp.total_duration   (RampMove standard)
          3. steps / target_hz              (estimation depuis les paramètres bruts)
          4. 2.0 s                          (fallback de sécurité)

        Args:
            sub_move: Le Move correspondant au backoff ou preclear.
            margin:   Facteur multiplicatif de sécurité (défaut 1.5×).

        Returns:
            Timeout en secondes, minimum 1.0 s.
        """
        # Priorité 1 : attribut explicite
        estimated = getattr(sub_move, "estimated_duration_s", None)
        if estimated is not None and estimated > 0:
            return max(1.0, float(estimated) * margin)

        # Priorité 2 : RampMove.ramp.total_duration (via axis_configs[0].ramp ou sub_move.ramp)
        ramp = getattr(sub_move, "ramp", None)
        if ramp is None:
            axis_configs = getattr(sub_move, "axis_configs", None) or []
            ramp = getattr(axis_configs[0], "ramp", None) if axis_configs else None
        if ramp is not None:
            total = getattr(ramp, "total_duration", None)
            if total is not None and total > 0:
                return max(1.0, float(total) * margin)

        # Priorité 3 : estimation brute steps / hz
        steps = getattr(sub_move, "total_steps", None) or getattr(sub_move, "step_count", None)
        hz    = getattr(sub_move, "target_hz", None)
        if hz is None and ramp is not None:
            hz = getattr(ramp, "target_hz", None)
        if steps and hz and float(steps) > 0 and float(hz) > 0:
            return max(1.0, (abs(float(steps)) / float(hz)) * margin)

        # Fallback
        return max(2.0, _ENDSTOP_RELEASE_TIMEOUT_S)

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
            last_status = self._read_status(axis_id)
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
        last_status = None
        while time.monotonic() < deadline:
            last_status = self._read_status(axis_id)
            if int(getattr(last_status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)) == LATERAL_ENDSTOP_PRESENT_OPEN:
                return last_status
            time.sleep(self._poll_interval_s)
        raise RuntimeError(
            f"endstop release timeout on axis {axis_id}: state=0x{int(getattr(last_status, 'lateral_endstop_state', 0xFF)):02X}"
        )

    def _stream_homing_sub_move(
        self,
        move: HomingMove,
        *,
        phase_name: str,
        sub_move: Move,
        arm_endstop: bool,
        start_sequence: int,
    ) -> MultiAxisRampStreamer:
        sub_move_axis_configs = sub_move.axis_configs
        if not sub_move_axis_configs:
            raise RuntimeError(
                f"homing sub-move {phase_name} has no public axis_configs"
            )

        baseline_status = self._read_status(move.axis_id)
        baseline_segments_dropped = int(
            getattr(baseline_status, "segments_dropped", 0)
        )

        streamer = self._make_streamer(
            sub_move_axis_configs,
            keep_enabled_axes={move.axis_id},
            initial_segments_dropped=baseline_segments_dropped,
        )
        self._current_streamer = streamer
        if hasattr(streamer, "note_endstop_armed"):
            streamer.note_endstop_armed(move.axis_id, arm_endstop)
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

    def _next_sequence_after_streamer(self, streamer: Any) -> int:
        next_from_status = self._next_motion_sequence()
        flush_floor = int(getattr(streamer, "flush_floor_sequence", -1))
        if flush_floor < 0:
            return next_from_status
        candidate = (flush_floor + 1) & 0xFFFF
        if next_from_status < 0 or sequence_is_greater(candidate, next_from_status):
            return candidate
        return next_from_status

    @staticmethod
    def _streamer_stop_requested(streamer: Any) -> bool:
        callback = getattr(streamer, "has_stop_been_requested", None)
        if not callable(callback):
            return False
        return bool(callback())

    def _clear_closed_endstop_before_homing(self, move: HomingMove) -> None:
        logger.info(
            "homing axis %s: endstop already closed at start, running preclear",
            move.axis_id,
        )
        clearance_move = move._make_backoff_move()
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
        self._wait_for_endstop_open(
            move.axis_id,
            timeout_s=self._compute_backoff_timeout(clearance_move),
        )
        # R5: attente de stabilisation mécanique (debounce) — 2 cycles SPI minimum
        # pour que le GPIO se stabilise après relâchement du contact physique.
        time.sleep(0.020)
        # Relecture finale pour confirmer l'état avant d'armer la phase approach.
        final_status = self._read_status(move.axis_id)
        lateral_state = int(
            getattr(final_status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
        )
        if lateral_state != LATERAL_ENDSTOP_PRESENT_OPEN:
            raise RuntimeError(
                f"preclear did not clear the endstop on axis {move.axis_id}: "
                f"lateral_endstop_state=0x{lateral_state:02X} after stabilisation wait"
            )

    def _check_armed_phase_result(
        self,
        move: HomingMove,
        phase_name: str,
        streamer: "MultiAxisRampStreamer",
    ) -> None:
        """Vérifie qu'une phase armée s'est bien terminée par un déclenchement endstop.

        Lève RuntimeError avec un message diagnostique si ce n'est pas le cas.
        """
        if streamer.endstop_triggered:
            return  # succès nominal

        # Lire le status pour diagnostiquer
        status = self._read_status(move.axis_id)
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
        status = self._transport.wait_for_request_result(
            sequence,
            hint_status=send_status,
            poll_interval_s=self._poll_interval_s,
        )
        self._update_axis_endstop_state(axis_id, status)
        if int(getattr(status, "last_result", SpiMessageResult.OK)) != int(SpiMessageResult.OK):
            raise RuntimeError(
                f"enable_endstop axis {axis_id} arm={int(arm)} failed with result=0x{int(status.last_result):02X}"
            )
        if self._status_has_endstop_armed(status, axis_id, arm):
            return status
        return self._wait_for_endstop_arm_state(axis_id, arm)

    def _wrap_segment_sequence(self, generator: Any, start_sequence: int):
        sequence = start_sequence & 0xFFFF
        for segment in generator:
            segment.sequence = sequence
            yield segment
            sequence = (sequence + 1) & 0xFFFF

    def _execute_ramp_move(self, move: Move) -> None:
        """Execute a RampMove or JogMove via MultiAxisRampStreamer."""
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

        # Update position for axes with known delta.
        for ax_id in axis_ids:
            delta = move.expected_delta_steps(ax_id)
            if delta is not None and ax_id in self._axis_states:
                self._axis_states[ax_id].advance_position(delta)

        move.mark_completed()

    def _make_wound_streamer(self, move: WoundMove, *, keep_enabled_axes: set[int] | None = None) -> MultiAxisRampStreamer:
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

    def _execute_wound_move(self, move: WoundMove) -> None:
        """Execute a WoundMove with explicit spindle/traverse streamer setup."""
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

        for ax_id in move.axis_ids:
            if ax_id not in self._axis_states:
                continue
            delta = move.expected_delta_steps(ax_id)
            if delta is None:
                self._axis_states[ax_id].invalidate_position()
                continue
            self._axis_states[ax_id].advance_position(delta)
        move.mark_completed()

    def _execute_homing(self, move: HomingMove) -> None:
        """
        Execute a HomingMove phase by phase.

        Phase sequence:
          1. approach (endstop armed) — stops when endstop fires
          2. backoff  (endstop disarmed) — moves away from endstop
          3. search   (endstop armed) — slow approach for precise home
          4. set_position to move.home_position_steps
        """
        move.mark_running()
        axis_state = self._axis_states.get(move.axis_id)

        try:
            initial_status = self._read_status(move.axis_id)
            initial_state = int(
                getattr(initial_status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
            )
            if initial_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
                initial_state = self._confirm_initial_closed_endstop(move.axis_id)
            if initial_state == LATERAL_ENDSTOP_ABSENT:
                self._ensure_homing_can_start(move.axis_id, "start")
            elif initial_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
                self._clear_closed_endstop_before_homing(move)
                self._ensure_homing_can_start(move.axis_id, "start")
            else:
                self._ensure_homing_can_start(move.axis_id, "start")
        except Exception as exc:
            move.mark_failed(str(exc))
            return

        next_sequence = self._next_motion_sequence()

        for phase_name, sub_move, arm_endstop in move.phases():
            if self._stop_requested:
                self._set_endstop_armed(move.axis_id, arm=False)
                stop_plan = self._active_stop_plan or self._default_stop_plan(
                    [move.axis_id],
                    "stop requested during homing",
                )
                self._apply_stop_plan([move.axis_id], stop_plan)
                move.mark_aborted(f"{stop_plan.mode.value} requested during homing")
                return

            if arm_endstop:
                try:
                    self._ensure_homing_can_start(move.axis_id, phase_name)
                except Exception as exc:
                    move.mark_failed(str(exc))
                    return

            # Arm or disarm endstop for this phase and verify the mask in status.
            try:
                self._set_endstop_armed(move.axis_id, arm=arm_endstop)
            except Exception as exc:
                move.mark_failed(str(exc))
                return

            try:
                streamer = self._stream_homing_sub_move(
                    move,
                    phase_name=phase_name,
                    sub_move=sub_move,
                    arm_endstop=arm_endstop,
                    start_sequence=next_sequence,
                )
            except Exception as exc:
                self._set_endstop_armed(move.axis_id, arm=False)
                move.mark_failed(str(exc))
                return

            next_sequence = self._next_sequence_after_streamer(streamer)

            phase_completed_on_expected_endstop = arm_endstop and streamer.endstop_triggered
            if self._stop_requested or (
                self._streamer_stop_requested(streamer)
                and not phase_completed_on_expected_endstop
            ):
                self._set_endstop_armed(move.axis_id, arm=False)
                stop_plan = self._active_stop_plan or self._default_stop_plan(
                    [move.axis_id],
                    "stop requested during homing",
                )
                self._apply_stop_plan([move.axis_id], stop_plan)
                move.mark_aborted(f"{stop_plan.mode.value} requested during homing")
                return

            if phase_name in ("approach", "search"):
                try:
                    self._check_armed_phase_result(move, phase_name, streamer)
                except RuntimeError as exc:
                    # Endstop did not fire — homing failed with diagnostics.
                    self._set_endstop_armed(move.axis_id, arm=False)
                    move.mark_failed(str(exc))
                    return

            if phase_name == "backoff":
                try:
                    self._wait_for_endstop_open(
                        move.axis_id,
                        timeout_s=self._compute_backoff_timeout(sub_move),
                    )
                except Exception as exc:
                    self._set_endstop_armed(move.axis_id, arm=False)
                    move.mark_failed(str(exc))
                    return

            if self._stop_requested:
                self._set_endstop_armed(move.axis_id, arm=False)
                stop_plan = self._active_stop_plan or self._default_stop_plan(
                    [move.axis_id],
                    "stop requested during homing",
                )
                self._apply_stop_plan([move.axis_id], stop_plan)
                move.mark_aborted(f"{stop_plan.mode.value} requested during homing")
                return

        # All phases complete — disarm endstop and set home position.
        self._set_endstop_armed(move.axis_id, arm=False)
        if axis_state is not None:
            axis_state.mark_homed(move.home_position_steps)

        move.mark_completed()
