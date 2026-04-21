from __future__ import annotations

import logging
import threading
import time
from typing import Any

from core.config import AppConfiguration
from core.coordinator import MotionStopPlan
from core.events import EventBus, EventKind
from core.lateral import LateralAxisController
from core.shared_state import EngineState, SharedState
from motion import SpindleKinematics
from motion.command_service import adjust_duration_for_ramp_deficit
from motion.move_queue import MoveQueue
from transport.spi_transport import Esp32SpiTransport
from winding import ScatterEngine, SyncAxisConfig, WindingPattern, WoundMove
from winding.program import WindingProgram


logger = logging.getLogger(__name__)

_DEFAULT_HOME_APPROACH_RPM = 100.0
_DEFAULT_HOME_SEARCH_RPM = 20.0
_DEFAULT_HOME_BACKOFF_STEPS = 3200


class WindingEngine:
    """
    Executes winding programs in a single daemon thread.

    The engine owns the MoveQueue and is the only producer of motion.
    The JsonRpcServer sends commands by calling public methods on the
    engine — these methods are thread-safe and return immediately.
    The engine thread reads a _pending_program queue and executes it.

    State machine:
      IDLE -> HOMING -> RUNNING -> IDLE       (normal completion)
      IDLE -> HOMING -> FAULT                 (homing failed)
      RUNNING -> STOPPING -> IDLE             (stop requested)
      RUNNING -> FAULT                        (endstop / error)
    """

    def __init__(self,*,transport: Esp32SpiTransport,shared_state: SharedState,move_queue: MoveQueue,
                 lateral_controller: LateralAxisController,event_bus: EventBus,config: AppConfiguration | None = None,) -> None:
        self._transport = transport
        self._state = shared_state
        self._move_queue = move_queue
        self._lateral = lateral_controller
        self._events = event_bus
        self._config = config or AppConfiguration()

        self._shutdown_event = threading.Event()
        self._stop_request_event = threading.Event()
        self._program_event = threading.Event()
        self._pending_program: WindingProgram | None = None
        self._program_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._last_worker_error: str | None = None

    @property
    def config(self) -> AppConfiguration:
        return self._config

    def start(self) -> None:
        """Start the engine thread and the MoveQueue."""
        if self._thread is not None and self._thread.is_alive():
            return

        self._shutdown_event.clear()
        self._stop_request_event.clear()
        self._last_worker_error = None
        self._move_queue.start()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="winding_engine",
        )
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        """Stop the engine thread and MoveQueue cleanly."""
        self._shutdown_event.set()
        self._stop_request_event.set()
        self._program_event.set()
        self._move_queue.stop(timeout_s=timeout_s)
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
        if self._thread is not None and self._thread.is_alive():
            self._last_worker_error = (
                f"winding engine worker did not stop within {timeout_s:.1f}s"
            )
            raise RuntimeError(self._last_worker_error)

    def submit_program(self, program: WindingProgram) -> None:
        """
        Queue a winding program for execution.
        Raises RuntimeError if the engine is not IDLE.
        """
        with self._program_lock:
            if self._state.engine_state not in (EngineState.IDLE, EngineState.FAULT):
                raise RuntimeError(
                    f"Cannot submit program: engine is {self._state.engine_state.name}"
                )
            program.validate()
            self._pending_program = program
            self._stop_request_event.clear()
            self._program_event.set()

    def request_stop(
        self,
        stop_plan: MotionStopPlan | None = None,
        *,
        clear_queue: bool = True,
    ) -> None:
        """Request stop. Current move is aborted, queue is cleared."""
        self._stop_request_event.set()
        if clear_queue:
            self._move_queue.clear(
                stop_plan=stop_plan or MotionStopPlan.stop(
                    self._state.axis_states,
                    reason="engine stop requested",
                )
            )
        if self._state.engine_state in (EngineState.HOMING, EngineState.RUNNING):
            self._state.set_engine_state(EngineState.STOPPING)

    def clear_fault(self) -> None:
        """Clear fault state so a new program can be submitted."""
        self._state.clear_fault()

    def health_status(self) -> dict[str, Any]:
        return {
            "name": "winding_engine",
            "thread_alive": self._thread is not None and self._thread.is_alive(),
            "thread_faulted": self._last_worker_error is not None,
            "last_error": self._last_worker_error,
            "stop_requested": self._stop_requested(),
        }



    def _run(self) -> None:
        """Main engine loop — waits for programs and executes them."""
        try:
            while not self._shutdown_event.is_set():
                self._program_event.wait(timeout=1.0)
                self._program_event.clear()

                with self._program_lock:
                    program = self._pending_program
                    self._pending_program = None

                if program is None:
                    continue
                if self._shutdown_event.is_set():
                    break

                self._execute_program(program)
        except Exception as exc:
            self._last_worker_error = str(exc)
            logger.exception("winding engine worker failed")
            self._state.set_fault(f"winding engine worker failed: {exc}")
            self._events.publish(
                EventKind.WORKER_FAILED,
                worker="winding_engine",
                error=str(exc),
            )

    def _execute_program(self, program: WindingProgram) -> None:
        """
        Execute one complete winding program.

        Sequence:
          1. Optional homing on lateral axis
          2. For each layer:
             a. Start layer in shared state
             b. Enqueue synchronised RampMove (spindle + lateral)
             c. Wait for MoveQueue to drain
             d. Complete layer in shared state
          3. Mark program complete or aborted
        """
        self._stop_request_event.clear()
        self._state.set_program(program)
        self._state.set_engine_state(
            EngineState.HOMING if program.home_before_start else EngineState.RUNNING
        )
        self._events.publish(EventKind.PROGRAM_STARTED, program=program.snapshot())

        if program.home_before_start:
            success, _reason = self._lateral.home(
                axis_id=program.lateral_axis_id,
                approach_rpm=program.home_approach_rpm,
                search_rpm=program.home_search_rpm,
                backoff_steps=program.home_backoff_steps,
            )
            if not success:
                return
            self._state.set_engine_state(EngineState.RUNNING)
        else:
            self._lateral.require_homed(program.lateral_axis_id)

        for layer_index in range(program.num_layers):
            if self._stop_requested():
                self._state.set_engine_state(EngineState.IDLE)
                self._events.publish(EventKind.PROGRAM_ABORTED, layer=layer_index)
                return

            direction = "forward" if layer_index % 2 == 0 else "reverse"
            self._state.start_layer(layer_index, program.num_layers, direction)
            self._events.publish(
                EventKind.LAYER_STARTED, layer=layer_index, direction=direction
            )

            ok = self._run_layer(program, layer_index, direction)
            if not ok:
                return

            self._state.complete_layer()
            self._events.publish(EventKind.LAYER_COMPLETED, layer=layer_index)

        self._state.set_engine_state(EngineState.IDLE)
        self._state.set_program(None)
        self._events.publish(EventKind.PROGRAM_COMPLETED, program=program.snapshot())

    def _run_layer(
        self,
        program: WindingProgram,
        layer_index: int,
        direction: str,
    ) -> bool:
        """
        Execute one winding layer: spindle + lateral move in sync.
        Returns True on completion, False on abort or fault.
        """
        self._lateral.require_homed(program.lateral_axis_id)
        reverse_lateral = direction == "reverse"
        total_turns = 2.0 * program.bobbin_width_mm * program.turns_per_mm
        target_rps = program.spindle_rpm / 60.0
        duration_s = adjust_duration_for_ramp_deficit(
            total_turns=total_turns,
            target_rps=target_rps,
            accel_s=program.accel_s,
            decel_s=program.decel_s,
        )
        cruise_s = max(duration_s - program.accel_s - program.decel_s, 0.0)

        move = WoundMove(
            name=f"layer_{layer_index}",
            kinematics=SpindleKinematics(
                target_rpm=program.spindle_rpm,
                start_rpm=0.0,
                accel_s=program.accel_s,
                cruise_s=cruise_s,
                decel_s=program.decel_s,
            ),
            pattern=WindingPattern(
                bobbin_width_mm=program.bobbin_width_mm,
                turns_per_mm=program.turns_per_mm,
            ),
            scatter=ScatterEngine(
                amplitude_mm=program.scatter_amplitude_mm,
                freq1=program.scatter_freq1,
                freq2=program.scatter_freq2,
                damping_margin_mm=program.scatter_damping_margin_mm,
            ),
            spindle_cfg=SyncAxisConfig(
                axis_index=program.spindle_axis_id,
                steps_per_unit=(
                    self._config.spindle_steps_per_revolution
                    * self._config.spindle_microstepping
                ),
            ),
            traverse_cfg=SyncAxisConfig(
                axis_index=program.lateral_axis_id,
                steps_per_unit=program.lateral_steps_per_mm,
                reverse_direction=reverse_lateral,
            ),
        )
        self._move_queue.enqueue(move)
        self._wait_for_move_queue()

        if move.aborted_by_endstop:
            msg = f"Endstop triggered during layer {layer_index}"
            self._state.set_fault(msg)
            self._events.publish(
                EventKind.ENDSTOP_TRIGGERED,
                layer=layer_index,
                axis=program.lateral_axis_id,
            )
            return False

        if move.state.name == "FAILED":
            self._state.set_fault(move.error or "unknown error")
            return False

        if self._stop_requested():
            self._state.set_engine_state(EngineState.IDLE)
            self._events.publish(EventKind.PROGRAM_ABORTED, layer=layer_index)
            return False

        return True

    def _stop_requested(self) -> bool:
        return self._shutdown_event.is_set() or self._stop_request_event.is_set()

    def _wait_for_move_queue(
        self,
        poll_s: float = 0.05,
        timeout_s: float = 60.0,
    ) -> None:
        """
        Block until the MoveQueue has no pending or running moves,
        until a stop is requested, or until *timeout_s* seconds have
        elapsed.

        On timeout the engine transitions to FAULT so the caller can
        detect the condition via ``move.state``.
        """
        deadline = time.monotonic() + timeout_s
        while (
            not self._stop_requested()
            and (
                self._move_queue.pending_count > 0
                or self._move_queue.current_move is not None
            )
        ):
            if time.monotonic() >= deadline:
                msg = (
                    f"_wait_for_move_queue timed out after {timeout_s:.1f} s — "
                    "firmware may have stopped responding"
                )
                self._state.set_fault(msg)
                self._move_queue.clear()
                break
            time.sleep(poll_s)


