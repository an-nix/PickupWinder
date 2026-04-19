from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any

from motion.axis_state import AxisState
from motion.move import BaseMove, CompositeMove, HomingMove, Move, WoundMove
from transport.streamer import MultiAxisRampStreamer, StreamAxisConfig
from transport.spi_transport import Esp32SpiTransport

_MAX_HISTORY = 50


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
        poll_interval_s: float = 0.001,
        print_every: int = 1,
    ) -> None:
        self._transport = transport
        self._axis_states = axis_states
        self._poll_interval_s = poll_interval_s
        self._print_every = print_every

        self._queue: deque[BaseMove] = deque()
        self._queue_lock = threading.Lock()
        self._queue_event = threading.Event()

        self._stop_requested = False
        self._thread: threading.Thread | None = None
        self._current_move: BaseMove | None = None
        self._history: list[BaseMove] = []

    # ── Public API ───────────────────────────────────────────────────────

    def enqueue(self, move: BaseMove) -> None:
        """Add a move to the queue. Safe to call from any thread."""
        with self._queue_lock:
            self._queue.append(move)
        self._queue_event.set()

    def start(self) -> None:
        """Start the execution thread."""
        self._stop_requested = False
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
        self._queue_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout_s)

    def clear(self) -> None:
        """Remove all pending moves from the queue without stopping."""
        with self._queue_lock:
            self._queue.clear()

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
            "current_move": (
                self._current_move.snapshot() if self._current_move else None
            ),
            "pending_moves": queue_snapshot,
            "history": [m.snapshot() for m in self._history[-10:]],
            "axis_states": {
                ax_id: state.snapshot()
                for ax_id, state in self._axis_states.items()
            },
        }

    # ── Execution thread ─────────────────────────────────────────────────

    def _run(self) -> None:
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
                    # Always clear current_move, even if execution failed
                    self._current_move = None
                
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
            elif isinstance(move, WoundMove):
                self._execute_wound_move(move)
            elif isinstance(move, Move):
                self._execute_ramp_move(move)
            else:
                move.mark_failed(f"unsupported move type: {type(move).__name__}")
        except Exception as exc:
            move.mark_failed(str(exc))

    def _make_streamer(self, axis_configs) -> MultiAxisRampStreamer:
        """Create a fresh streamer for a list of AxisMotionConfig."""
        return MultiAxisRampStreamer(
            self._transport,
            [
                StreamAxisConfig(axis_id=cfg.axis_id, ramp=cfg.ramp)
                for cfg in axis_configs
            ],
            poll_interval_s=self._poll_interval_s,
            print_every=self._print_every,
            target_buffer_time_s=0.150,
        )

    def _next_motion_sequence(self) -> int:
        status = self._transport.get_status()
        last_executed = int(getattr(status, "last_executed_sequence", -1))
        if last_executed == 0xFFFF or last_executed < 0:
            return 0
        return (last_executed + 1) & 0xFFFF

    def _set_endstop_armed(self, axis_id: int, arm: bool) -> None:
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=arm)
        self._transport.wait_for_request_result(
            sequence,
            poll_interval_s=self._poll_interval_s,
        )

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

        streamer = self._make_streamer(axis_configs)
        # Override the generator to use the move's segments() method, but align
        # motion_sequence values with the ESP32 last_executed_sequence.
        streamer._generator = self._wrap_segment_sequence(
            move.segments(),
            self._next_motion_sequence(),
        )
        streamer._generator_finished = False

        try:
            streamer.stream_all()
        except Exception as exc:
            move.mark_failed(str(exc))
            return

        if streamer.endstop_triggered:
            for ax_id in axis_ids:
                if ax_id in self._axis_states:
                    self._axis_states[ax_id].invalidate_position()
            move.mark_aborted("endstop triggered", by_endstop=True)
            return

        if self._stop_requested:
            move.mark_aborted("stop requested")
            return

        # Update position for axes with known delta.
        for ax_id in axis_ids:
            delta = move.expected_delta_steps(ax_id)
            if delta is not None and ax_id in self._axis_states:
                self._axis_states[ax_id].advance_position(delta)

        move.mark_completed()

    def _make_wound_streamer(self, move: WoundMove) -> MultiAxisRampStreamer:
        target_hz = (
            max(move.kinematics.target_rpm, 1.0) / 60.0
            * float(move.spindle_cfg.steps_per_unit)
        )
        return MultiAxisRampStreamer.from_axis_ids(
            self._transport,
            move.axis_ids,
            target_hz=max(target_hz, 1.0),
            segment_duration_s=move.segment_duration_s,
            poll_interval_s=self._poll_interval_s,
            print_every=self._print_every,
            target_buffer_time_s=0.150,
        )

    def _execute_wound_move(self, move: WoundMove) -> None:
        """Execute a WoundMove with explicit spindle/traverse streamer setup."""
        move.mark_running()
        axis_ids = move.axis_ids

        streamer = self._make_wound_streamer(move)
        streamer._generator = self._wrap_segment_sequence(
            move.segments(),
            self._next_motion_sequence(),
        )
        streamer._generator_finished = False

        try:
            streamer.stream_all()
        except Exception as exc:
            move.mark_failed(str(exc))
            return

        if streamer.endstop_triggered:
            for ax_id in axis_ids:
                if ax_id in self._axis_states:
                    self._axis_states[ax_id].invalidate_position()
            move.mark_aborted("endstop triggered", by_endstop=True)
            return

        if self._stop_requested:
            for ax_id in axis_ids:
                if ax_id in self._axis_states:
                    self._axis_states[ax_id].invalidate_position()
            move.mark_aborted("stop requested")
            return

        for ax_id in move.axis_ids:
            if ax_id in self._axis_states:
                self._axis_states[ax_id].invalidate_position()
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

        for phase_name, sub_move, arm_endstop in move.phases():
            if self._stop_requested:
                self._set_endstop_armed(move.axis_id, arm=False)
                move.mark_aborted("stop requested during homing")
                return

            # Arm or disarm endstop for this phase.
            self._set_endstop_armed(move.axis_id, arm=arm_endstop)

            # Execute the sub-move.
            sub_move_axis_configs = sub_move.axis_configs
            if not sub_move_axis_configs:
                self._set_endstop_armed(move.axis_id, arm=False)
                move.mark_failed(
                    f"homing sub-move {phase_name} has no public axis_configs"
                )
                return
            streamer = self._make_streamer(sub_move_axis_configs)
            streamer._generator = self._wrap_segment_sequence(
                sub_move.segments(),
                self._next_motion_sequence(),
            )
            streamer._generator_finished = False
            streamer.stream_all()

            if phase_name in ("approach", "search") and not streamer.endstop_triggered:
                # Endstop did not fire — homing failed.
                self._set_endstop_armed(move.axis_id, arm=False)
                move.mark_failed(
                    f"homing {phase_name} phase completed without "
                    f"endstop trigger on axis {move.axis_id}"
                )
                return

            if self._stop_requested:
                self._set_endstop_armed(move.axis_id, arm=False)
                move.mark_aborted("stop requested during homing")
                return

        # All phases complete — disarm endstop and set home position.
        self._set_endstop_armed(move.axis_id, arm=False)
        if axis_state is not None:
            axis_state.mark_homed(move.home_position_steps)

        move.mark_completed()
