from __future__ import annotations

import threading
import time
from typing import Any

from motion.axis_state import AxisState
from motion.move import HomingMove, JogMove, RampMove, RampMoveConfig, WoundMove
from motion.move_queue import MoveQueue
from motion import (
    AxisMotionConfig,
    RampConfig,
    SpindleKinematics,
    WindingPattern,
    ScatterEngine,
    SyncAxisConfig,
)
from transport.spi_transport import Esp32SpiTransport
from core.events import EventBus, EventKind
from winding.program import WindingProgram
from core.shared_state import EngineState, SharedState


class WindingEngine:
    """
    Executes winding programs in a single daemon thread.

    The engine owns the MoveQueue and is the only producer of motion.
    The JsonRpcServer sends commands by calling public methods on the
    engine — these methods are thread-safe and return immediately.
    The engine thread reads a _pending_program queue and executes it.

    State machine:
      IDLE -> HOMING -> RUNNING -> IDLE       (normal completion)
      IDLE -> HOMING -> FAULT               (homing failed)
      RUNNING -> STOPPING -> IDLE           (stop requested)
      RUNNING -> FAULT                     (endstop / error)
    """

    def __init__(
        self,
        transport: Esp32SpiTransport,
        shared_state: SharedState,
        event_bus: EventBus,
    ) -> None:
        self._transport = transport
        self._state = shared_state
        self._events = event_bus

        self._move_queue = MoveQueue(
            transport=transport,
            axis_states=shared_state.axis_states,
            poll_interval_s=0.005,
            print_every=8,
        )

        self._stop_event = threading.Event()
        self._program_event = threading.Event()
        self._pending_program: WindingProgram | None = None
        self._program_lock = threading.Lock()
        self._thread: threading.Thread | None = None

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the engine thread and the MoveQueue."""
        self._move_queue.start()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="winding_engine",
        )
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        """Stop the engine thread and MoveQueue cleanly."""
        self._stop_event.set()
        self._program_event.set()
        self._move_queue.stop(timeout_s=timeout_s)
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)

    # ── Command API (called from JsonRpcServer thread) ─────────────────────

    def submit_program(self, program: WindingProgram) -> None:
        """
        Queue a winding program for execution.
        Raises RuntimeError if the engine is not IDLE.
        """
        with self._program_lock:
            if self._state.engine_state not in (
                EngineState.IDLE, EngineState.FAULT
            ):
                raise RuntimeError(
                    f"Cannot submit program: engine is "
                    f"{self._state.engine_state.name}"
                )
            program.validate()
            self._pending_program = program
            self._stop_event.clear()
            self._program_event.set()

    def request_stop(self) -> None:
        """Request stop. Current move is aborted, queue is cleared."""
        self._stop_event.set()
        self._move_queue.clear()
        self._state.set_engine_state(EngineState.STOPPING)

    def clear_fault(self) -> None:
        """Clear fault state so a new program can be submitted."""
        self._state.clear_fault()

    def arm_endstop(self, axis_id: int) -> None:
        """Arm the specified endstop through the transport."""
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=True)
        self._transport.wait_for_request_result(sequence)

    def disarm_endstop(self, axis_id: int) -> None:
        """Disarm the specified endstop through the transport."""
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=False)
        self._transport.wait_for_request_result(sequence)

    def wound_run(
        self,
        spindle_axis_id: int,
        traverse_axis_id: int,
        target_rpm: float,
        accel_s: float,
        cruise_s: float,
        decel_s: float,
        bobbin_width_mm: float,
        turns_per_mm: float,
        scatter_amplitude_mm: float = 0.0,
        scatter_damping_margin_mm: float = 0.0,
        spindle_reverse: bool = False,
        traverse_reverse: bool = False,
    ) -> None:
        """
        Execute a synchronized winding operation (Electronic Gearing).
        Only allowed when engine is IDLE.
        """
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError("Winding run only allowed when engine is IDLE")
        
        move = WoundMove(
            name="winding_electronic_gearing",
            kinematics=SpindleKinematics(
                target_rpm=target_rpm,
                start_rpm=0.0,    
                accel_s=accel_s,
                cruise_s=cruise_s,
                decel_s=decel_s
            ),
            pattern=WindingPattern(
                bobbin_width_mm=bobbin_width_mm,
                turns_per_mm=turns_per_mm
            ),
            scatter=ScatterEngine(
                amplitude_mm=scatter_amplitude_mm,
                damping_margin_mm=scatter_damping_margin_mm
            ),
            spindle_cfg=SyncAxisConfig(
                axis_index=spindle_axis_id,
                steps_per_unit=6400.0,
                reverse_direction=spindle_reverse
            ),
            traverse_cfg=SyncAxisConfig(
                axis_index=traverse_axis_id,
                steps_per_unit=3072.0,
                reverse_direction=traverse_reverse
            )
        )
        self._move_queue.enqueue(move)

    def jog(
        self,
        axis_id: int,
        steps: int,
        rpm: float,
        reverse: bool = False,
    ) -> None:
        """
        Execute a jog move immediately.
        Only allowed when engine is IDLE.
        """
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError("Jog only allowed when engine is IDLE")
        move = JogMove(
            name=f"jog_{axis_id}",
            axis_id=axis_id,
            steps_per_rev=200 * 32,
            steps=steps,
            rpm=rpm,
            reverse_direction=reverse,
        )
        self._move_queue.enqueue(move)

    # ── Engine thread ──────────────────────────────────────────────────────

    def _run(self) -> None:
        """Main engine loop — waits for programs and executes them."""
        while not self._stop_event.is_set():
            self._program_event.wait(timeout=1.0)
            self._program_event.clear()

            with self._program_lock:
                program = self._pending_program
                self._pending_program = None

            if program is None:
                continue
            if self._stop_event.is_set():
                break

            self._execute_program(program)

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
        self._state.set_program(program)
        self._state.set_engine_state(
            EngineState.HOMING if program.home_before_start else EngineState.RUNNING
        )
        self._events.publish(EventKind.PROGRAM_STARTED, program=program.snapshot())

        # ── Phase 1: homing ────────────────────────────────────────────────
        if program.home_before_start:
            success = self._home_lateral(program)
            if not success:
                return
            self._state.set_engine_state(EngineState.RUNNING)

        # ── Phase 2: winding layers ────────────────────────────────────────
        for layer_index in range(program.num_layers):
            if self._stop_event.is_set():
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

        # ── Phase 3: completion ────────────────────────────────────────────
        self._state.set_engine_state(EngineState.IDLE)
        self._state.set_program(None)
        self._events.publish(EventKind.PROGRAM_COMPLETED, program=program.snapshot())

    def _home_lateral(self, program: WindingProgram) -> bool:
        """
        Home the lateral axis. Returns True on success, False on failure.
        Sets FAULT state and publishes event on failure.
        """
        self._events.publish(EventKind.HOMING_STARTED, axis_id=program.lateral_axis_id)
        move = HomingMove(
            name="home_lateral",
            axis_id=program.lateral_axis_id,
            steps_per_rev=200 * 32,
            approach_rpm=program.home_approach_rpm,
            search_rpm=program.home_search_rpm,
            backoff_steps=program.home_backoff_steps,
            max_approach_steps=int(200 * 32 * 20),
        )
        self._move_queue.enqueue(move)
        self._wait_for_move_queue()

        if move.state.name == "COMPLETED":
            self._events.publish(
                EventKind.HOMING_COMPLETED, axis_id=program.lateral_axis_id
            )
            return True
        else:
            msg = f"Homing failed: {move._error}"
            self._state.set_fault(msg)
            self._events.publish(
                EventKind.HOMING_FAILED,
                axis_id=program.lateral_axis_id,
                error=msg,
            )
            return False

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
        reverse_lateral = (direction == "reverse")
        lateral_rpm = program.lateral_rpm_for_layer()
        duration_s = program.layer_duration_s()
        cruise_s = max(duration_s - program.accel_s - program.decel_s, 0.0)

        move = RampMove(
            name=f"layer_{layer_index}",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=program.spindle_axis_id,
                        ramp=RampConfig(
                            axis_id=program.spindle_axis_id,
                            target_rpm=program.spindle_rpm,
                            accel_s=program.accel_s,
                            cruise_s=cruise_s,
                            decel_s=program.decel_s,
                            reverse_direction=False,
                        ),
                    ),
                    AxisMotionConfig(
                        axis_id=program.lateral_axis_id,
                        ramp=RampConfig(
                            axis_id=program.lateral_axis_id,
                            target_rpm=lateral_rpm,
                            accel_s=program.accel_s,
                            cruise_s=cruise_s,
                            decel_s=program.decel_s,
                            reverse_direction=reverse_lateral,
                        ),
                    ),
                ],
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
            self._state.set_fault(move._error or "unknown error")
            return False

        if self._stop_event.is_set():
            self._state.set_engine_state(EngineState.IDLE)
            self._events.publish(EventKind.PROGRAM_ABORTED, layer=layer_index)
            return False

        return True

    def status(self) -> dict[str, Any]:
        """Return a combined shared state and move queue status snapshot."""
        return {
            "shared_state": self._state.snapshot(),
            "move_queue": self._move_queue.status(),
        }

    def _wait_for_move_queue(self, poll_s: float = 0.05) -> None:
        """
        Block until the MoveQueue has no pending or running moves,
        or until a stop is requested.
        """
        while (
            not self._stop_event.is_set()
            and (
                self._move_queue.pending_count > 0
                or self._move_queue.current_move is not None
            )
        ):
            time.sleep(poll_s)
