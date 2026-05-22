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
from core.command_service import MotionCommandService, adjust_duration_for_ramp_deficit
from motion.move_queue import MoveQueue
from transport.spi_transport import Esp32SpiTransport
from winding import build_wound_move
from winding.program import WindingProgram
from winding.session import SessionParams


logger = logging.getLogger(__name__)

_DEFAULT_HOME_APPROACH_RPM = 100.0
_DEFAULT_HOME_SEARCH_RPM = 20.0
_DEFAULT_HOME_BACKOFF_STEPS = 6144


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

    def __init__(
        self,
        *,
        transport: Esp32SpiTransport,
        shared_state: SharedState,
        move_queue: MoveQueue | None = None,
        lateral_controller: LateralAxisController | None = None,
        commands: MotionCommandService | None = None,
        event_bus: EventBus,
        config: AppConfiguration | None = None,
    ) -> None:
        resolved_config = config or AppConfiguration()
        resolved_move_queue = move_queue or MoveQueue(
            transport=transport,
            axis_states=shared_state.axis_states,
            poll_interval_s=0.001,
            print_every=1,
        )
        resolved_lateral = lateral_controller or LateralAxisController(
            transport=transport,
            shared_state=shared_state,
            move_queue=resolved_move_queue,
            event_bus=event_bus,
            config=resolved_config,
        )

        self._transport = transport
        self._state = shared_state
        self._move_queue = resolved_move_queue
        self._lateral = resolved_lateral
        self._events = event_bus
        self._config = resolved_config
        self._commands = commands or MotionCommandService(
            transport=transport,
            shared_state=shared_state,
            move_queue=resolved_move_queue,
            lateral_controller=resolved_lateral,
            config=resolved_config,
        )

        self._shutdown_event = threading.Event()
        self._stop_request_event = threading.Event()
        self._program_event = threading.Event()
        self._pending_program: tuple[WindingProgram, SessionParams] | None = None
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

    def submit_program(
        self,
        program: WindingProgram,
        params: SessionParams,
    ) -> None:
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
            params.validate()
            self._pending_program = (program, params)
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
            effective_stop_plan = stop_plan or MotionStopPlan.stop(
                self._state.axis_states,
                reason="engine stop requested",
            )
            self._move_queue.clear(stop_plan=effective_stop_plan)

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

    def status(self) -> dict[str, Any]:
        self._lateral.refresh_home_state()
        move_queue_status = self._move_queue.status()

        return {
            "shared_state": self._state.snapshot(),
            "move_queue": move_queue_status,
        }

    def jog(
        self,
        *,
        axis_id: int,
        steps: int,
        rpm: float,
        reverse: bool = False,
    ) -> dict[str, Any]:
        self._commands.jog(axis_id=axis_id, steps=steps, rpm=rpm, reverse=reverse)
        return {
            "status": "queued",
            "axis_id": axis_id,
            "steps": steps,
            "rpm": rpm,
            "reverse": reverse,
        }

    def home_lateral(
        self,
        *,
        approach_rpm: float = _DEFAULT_HOME_APPROACH_RPM,
        search_rpm: float = _DEFAULT_HOME_SEARCH_RPM,
        backoff_steps: int = _DEFAULT_HOME_BACKOFF_STEPS,
    ) -> dict[str, Any]:
        return self._commands.home_lateral(
            approach_rpm=approach_rpm,
            search_rpm=search_rpm,
            backoff_steps=backoff_steps,
        )

    def move_lateral_to_mm(self, *, position_mm: float, rpm: float) -> dict[str, Any]:
        return self._commands.move_lateral_to_mm(position_mm=position_mm, rpm=rpm)

    def wound_run(self, **kwargs: Any) -> dict[str, Any]:
        self._commands.wound_run(**kwargs)
        return {"status": "queued"}

    def run_axis(self, *, duration_s: float, targets: list[dict[str, Any]]) -> dict[str, Any]:
        self._commands.run_axis(duration_s=duration_s, targets=targets)
        return {"status": "queued"}

    def _home_lateral_axis(
        self,
        *,
        axis_id: int,
        approach_rpm: float,
        search_rpm: float,
        backoff_steps: int,
    ) -> tuple[bool, str | None]:
        return self._lateral.home(
            axis_id=axis_id,
            approach_rpm=approach_rpm,
            search_rpm=search_rpm,
            backoff_steps=backoff_steps,
        )



    def _run(self) -> None:
        """Main engine loop — waits for programs and executes them."""
        try:
            while not self._shutdown_event.is_set():
                self._program_event.wait(timeout=1.0)
                self._program_event.clear()

                with self._program_lock:
                    pending = self._pending_program
                    self._pending_program = None

                if pending is None:
                    continue
                if self._shutdown_event.is_set():
                    break

                self._execute_program(*pending)
        except Exception as exc:
            self._last_worker_error = str(exc)
            logger.exception("winding engine worker failed")
            self._state.set_fault(f"winding engine worker failed: {exc}")
            self._events.publish(
                EventKind.WORKER_FAILED,
                worker="winding_engine",
                error=str(exc),
            )

    def _execute_program(
        self,
        program: WindingProgram,
        params: SessionParams,
    ) -> None:
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
            EngineState.HOMING if self._config.home_before_start else EngineState.RUNNING
        )
        self._events.publish(EventKind.PROGRAM_STARTED, program=program.snapshot())

        if self._config.home_before_start:
            success, _reason = self._home_lateral_axis(
                axis_id=self._config.lateral_axis_id,
                approach_rpm=self._config.lateral_homing_approach_rpm,
                search_rpm=self._config.lateral_homing_search_rpm,
                backoff_steps=self._config.lateral_homing_backoff_steps or 6144,
            )
            if not success:
                return
            self._state.set_engine_state(EngineState.RUNNING)
        else:
            self._lateral.require_homed(self._config.lateral_axis_id)

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

            ok = self._run_layer(program, params, layer_index, direction)
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
        params: SessionParams,
        layer_index: int,
        direction: str,
    ) -> bool:
        """
        Execute one winding layer: spindle + lateral move in sync.
        Returns True on completion, False on abort or fault.
        """
        self._lateral.require_homed(self._config.lateral_axis_id)
        reverse_lateral = direction == "reverse"
        total_turns = 2.0 * program.bobbin_width_mm * program.turns_per_mm
        target_rps = params.spindle_rpm / 60.0
        duration_s = adjust_duration_for_ramp_deficit(
            total_turns=total_turns,
            target_rps=target_rps,
            accel_s=self._config.spindle_accel_s,
            decel_s=self._config.spindle_decel_s,
        )
        cruise_s = max(duration_s - self._config.spindle_accel_s - self._config.spindle_decel_s, 0.0)

        move = build_wound_move(
            name=f"layer_{layer_index}",
            spindle_rpm=params.spindle_rpm,
            accel_s=self._config.spindle_accel_s,
            cruise_s=cruise_s,
            decel_s=self._config.spindle_decel_s,
            bobbin_width_mm=program.bobbin_width_mm,
            turns_per_mm=program.turns_per_mm,
            scatter_amplitude_mm=program.scatter_amplitude_mm,
            scatter_damping_margin_mm=program.scatter_damping_margin_mm,
            scatter_freq1=program.scatter_freq1,
            scatter_freq2=program.scatter_freq2,
            spindle_axis_id=self._config.spindle_axis_id,
            spindle_steps_per_rev=(
                self._config.spindle_steps_per_revolution
                * self._config.spindle_microstepping
            ),
            lateral_axis_id=self._config.lateral_axis_id,
            lateral_steps_per_mm=self._config.lateral_steps_per_mm,
            lateral_reverse=reverse_lateral,
        )
        estimated_duration_s = program.layer_duration_s(params.spindle_rpm)
        wait_timeout_s = min(max(estimated_duration_s * 3.0, 60.0), 300.0)
        self._move_queue.enqueue(move)
        self._wait_for_move_queue(timeout_s=wait_timeout_s)

        if move.aborted_by_endstop:
            msg = f"Endstop triggered during layer {layer_index}"
            self._state.set_fault(msg)
            self._events.publish(
                EventKind.ENDSTOP_TRIGGERED,
                layer=layer_index,
                axis=self._config.lateral_axis_id,
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


