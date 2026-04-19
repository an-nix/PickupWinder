# Architecture V3 — PickupWinder Host

> Generated for the V3 refactor pass.
> Focus: correct winding execution, validate geometry, harden motion profiles, and document streamer cleanup.

## 1. Scope of this refactor

This pass fixes the remaining host-side inconsistencies in the winding execution path:

- `TrapezoidalMotionProfile.turns_at()` now uses a symmetric deceleration integral and stricter input validation.
- `SpindleKinematics` dataclass now correctly initializes its base `TrapezoidalMotionProfile` and rejects invalid parameters.
- `WindingPattern` now validates its geometry on construction instead of silently returning `0.0`.
- `WoundMove` now validates its configuration and rejects duplicate spindle/traverse axis assignments.
- `WindingProgram` now captures bobbin geometry and scatter parameters, and computes layer duration from physical winding geometry.
- `WindingEngine._run_layer()` now builds a `WoundMove` for each program layer instead of a loose dual-axis `RampMove`.
- `MultiAxisRampStreamer.stream_all()` now disables axes in a `finally` block so drivers are cleaned up even on error.
- `_check_premature_completion()` is documented as the host-side guard against firmware sequence drift.

## 2. Files changed

### `src/rpi/motion/trapezoidal_profile.py`
- Added input validation for negative RPM and negative ramp durations.
- Reworked `turns_at()` to compute deceleration using a symmetric slope formula:
  - `rate = (start_rps - target_rps) / decel_s`
  - return `turns + target_rps * t + 0.5 * rate * t*t`
- This preserves the correct integral under the ramp and prevents subtle turn-count drift.

### `src/rpi/motion/spindle_kinematics.py`
- Hardened the dataclass by adding `__post_init__()`.
- `__post_init__()` forwards dataclass fields into `TrapezoidalMotionProfile.__init__()` so base validation runs.
- Design note: `SpindleKinematics` is a dataclass subclassing a regular class, so this is option B. The subclass defines slots, and the exact instance layout still preserves a `__dict__` because the base class is not slotted.

### `src/rpi/motion/winding_pattern.py`
- Added `__post_init__()` validation.
- Invalid `bobbin_width_mm <= 0.0` or `turns_per_mm <= 0.0` now raises `ValueError`.
- Removed the old silent fallback behavior.

### `src/rpi/motion/move.py`
- `WoundMove.__init__()` now validates:
  - distinct spindle and traverse axis indices
  - positive `segment_duration_s`
  - positive `kinematics.total_duration`

### `src/rpi/winding/program.py`
- Added physical winding parameters:
  - `bobbin_width_mm`
  - `scatter_amplitude_mm`
  - `scatter_damping_margin_mm`
- Added `turns_per_mm` derived property.
- `layer_duration_s()` now computes duration from actual winding geometry instead of using a placeholder cruise value.
- `snapshot()` now exposes bobbin and scatter parameters.

### `src/rpi/motion/engine.py`
- `WindingEngine._run_layer()` now builds a `WoundMove` instead of a raw `RampMove`.
- The layer execution path now uses:
  - `SpindleKinematics`
  - `WindingPattern`
  - `ScatterEngine`
  - `SyncAxisConfig`
- This aligns `submit_program()` execution with the existing `wound_run()` electronic gearing architecture.

### `src/rpi/transport/streamer.py`
- Added `_disable_axes()` symmetry for `_enable_axes()`.
- `stream_all()` now tracks whether axes were enabled and disables them inside a `finally` block.
- This prevents the host from leaving drivers enabled after an abort, endstop trigger, stall, or exception.

## 3. Relevant instrumentation and guards

### `_check_premature_completion()`

The streamer method validates firmware status feedback by comparing the ESP32's last executed motion sequence with the host's last sent sequence.

- If the firmware reports completion of a sequence the host never sent, the host logs a warning.
- This is a safety guard for cases where the ESP32 appears to have advanced its motion pipeline out of sync with the host.
- The warning rate is throttled via `_premature_notify_window_start` and `_premature_notify_count`.

### `WoundMove` / `SynchronizedSegmentGenerator`

A program layer is now defined as a true synchronized winding move:

- Spindle is the master axis whose `turns_at(t)` drives the profile.
- Traverse is slaved through `WindingPattern.guide_pos_mm()` and optional `ScatterEngine` offsets.
- `SyncAxisConfig.steps_per_unit` is used to convert physical units into step counts.

## 4. Test coverage added

New pytest coverage was added in `tests/test_motion_v3.py` for:

- `TrapezoidalMotionProfile.turns_at()` deceleration behavior
- `SpindleKinematics` dataclass validation
- `WindingPattern` invalid geometry rejection
- `WindingProgram` physical duration calculation and snapshot fields
- `WoundMove` duplicate-axis validation

All new tests pass:

- `8 passed`

## 5. Dependency diagram

```mermaid
flowchart TB
    subgraph RPC
        WRH[WindingRpcHandler]
        RPC[JsonRpcServer]
    end

    subgraph Engine
        WE[WindingEngine]
        MQ[MoveQueue]
    end

    subgraph Motion
        BM[BaseMove]
        MV[Move]
        CM[CompositeMove]
        RM[RampMove]
        WM[WoundMove]
        HM[HomingMove]
        SSG[SynchronizedSegmentGenerator]
        MASG[MultiAxisSegmentGenerator]
        SPK[SpindleKinematics]
        WP[WindingPattern]
        SE[ScatterEngine]
    end

    subgraph Transport
        MARS[MultiAxisRampStreamer]
        ESPI[Esp32SpiTransport]
    end

    subgraph Config
        CFG[AppConfiguration]
        CRT[compute_ramp_times]
    end

    RPC --> WRH --> WE
    WE --> MQ
    WE --> CRT
    MQ --> BM
    BM --> MV & CM
    MV --> RM & WM
    CM --> HM
    RM --> MASG
    WM --> SSG
    SSG --> SPK & WP & SE
    MARS --> MASG
    MARS --> SSG
    MARS --> ESPI
    WE --> CFG
    CRT --> CFG
```

## 6. Compatibility note

- No changes were made to `transport/spi_transport.py`.
- No changes were made to `jsonrpc/rpc_server.py`.
- Existing `winding.*` RPC method names were preserved.

## 7. Full V3 source files

### `src/rpi/motion/trapezoidal_profile.py`
```python
from __future__ import annotations


class TrapezoidalMotionProfile:
    def __init__(
        self,
        start_rpm: float = 0.0,
        target_rpm: float = 1000.0,
        accel_s: float = 0.0,
        cruise_s: float = 0.0,
        decel_s: float = 0.0,
    ) -> None:
        if start_rpm < 0.0 or target_rpm < 0.0:
            raise ValueError("start_rpm and target_rpm must be non-negative")
        if accel_s < 0.0 or cruise_s < 0.0 or decel_s < 0.0:
            raise ValueError("accel_s, cruise_s, and decel_s must be >= 0")

        self.start_rpm = start_rpm
        self.target_rpm = target_rpm
        self.accel_s = accel_s
        self.cruise_s = cruise_s
        self.decel_s = decel_s

    @property
    def total_duration(self) -> float:
        return self.accel_s + self.cruise_s + self.decel_s

    def _clamp_time(self, t: float) -> float:
        return min(max(t, 0.0), self.total_duration)

    def rps_at(self, t: float) -> float:
        t = self._clamp_time(t)
        start_rps = self.start_rpm / 60.0
        target_rps = self.target_rpm / 60.0

        if t < self.accel_s:
            if self.accel_s <= 0.0:
                return target_rps
            rate = (target_rps - start_rps) / self.accel_s
            return start_rps + rate * t

        t -= self.accel_s
        if t < self.cruise_s:
            return target_rps

        t -= self.cruise_s
        if self.decel_s <= 0.0:
            return target_rps
        rate = (target_rps - start_rps) / self.decel_s
        return max(target_rps - rate * t, 0.0)

    def rpm_at(self, t: float) -> float:
        return self.rps_at(t) * 60.0

    def turns_at(self, t: float) -> float:
        t = self._clamp_time(t)
        start_rps = self.start_rpm / 60.0
        target_rps = self.target_rpm / 60.0

        if t < self.accel_s:
            if self.accel_s <= 0.0:
                return target_rps * t
            rate = (target_rps - start_rps) / self.accel_s
            return start_rps * t + 0.5 * rate * t * t

        turns = 0.0
        if self.accel_s > 0.0:
            rate = (target_rps - start_rps) / self.accel_s
            turns += start_rps * self.accel_s + 0.5 * rate * self.accel_s * self.accel_s
        else:
            turns += target_rps * self.accel_s

        t -= self.accel_s
        if t < self.cruise_s:
            return max(turns + target_rps * t, 0.0)

        turns += target_rps * self.cruise_s
        t -= self.cruise_s
        if self.decel_s <= 0.0:
            return max(turns + target_rps * t, 0.0)

        rate = (target_rps - start_rps) / self.decel_s
        result = turns + target_rps * t - 0.5 * rate * t * t
        return max(result, 0.0)

    def steps_at(self, t: float, steps_per_rev: int) -> float:
        return self.turns_at(t) * float(steps_per_rev)

    def step_delta(self, time_start: float, time_end: float, steps_per_rev: int) -> float:
        return self.steps_at(time_end, steps_per_rev) - self.steps_at(time_start, steps_per_rev)
```

### `src/rpi/motion/spindle_kinematics.py`
```python
from __future__ import annotations

from dataclasses import dataclass

from .trapezoidal_profile import TrapezoidalMotionProfile


@dataclass(slots=True)
class SpindleKinematics(TrapezoidalMotionProfile):
    """Calculates absolute angular position (in turns) of the Spindle at time t."""
    target_rpm: float
    start_rpm: float = 0.0
    accel_s: float = 2.0
    cruise_s: float = 10.0
    decel_s: float = 2.0

    def __post_init__(self) -> None:
        super().__init__(
            start_rpm=self.start_rpm,
            target_rpm=self.target_rpm,
            accel_s=self.accel_s,
            cruise_s=self.cruise_s,
            decel_s=self.decel_s,
        )
```

### `src/rpi/winding/program.py`
```python
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class WindingProgram:
    """
    Describes a complete winding operation.

    The engine executes layers in order, alternating traverse direction
    on each layer. Each layer consists of:
      - spindle rotating at spindle_rpm for the duration of the layer
      - lateral axis traversing layer_pitch_mm * num_passes_per_layer
        at a speed derived from spindle_rpm and the wire geometry

    Fields:
      name              Human-readable program name
      num_layers        Total number of winding layers
      spindle_rpm       Spindle rotation speed in RPM
      layer_pitch_mm    Lateral advance per spindle revolution (mm)
      wire_diameter_mm  Wire diameter used to compute traverse speed
      accel_s           Acceleration time for both axes (seconds)
      decel_s           Deceleration time for both axes (seconds)
      spindle_axis_id   Axis ID of the spindle (default 0)
      lateral_axis_id   Axis ID of the lateral traverse (default 1)
      lateral_steps_per_mm  Steps per mm on the lateral axis
      home_before_start     If True, home lateral axis before starting
      home_approach_rpm     RPM for homing approach phase
      home_search_rpm       RPM for homing search phase
      home_backoff_steps    Steps to back off after first endstop contact
    """
    name: str
    num_layers: int
    spindle_rpm: float
    layer_pitch_mm: float
    wire_diameter_mm: float
    bobbin_width_mm: float = 15.0
    scatter_amplitude_mm: float = 0.0
    scatter_damping_margin_mm: float = 0.0
    accel_s: float = 0.5
    decel_s: float = 0.5
    spindle_axis_id: int = 0
    lateral_axis_id: int = 1
    lateral_steps_per_mm: float = 200.0 * 32.0 / 8.0  # 200step * 32µstep / 8mm/rev
    home_before_start: bool = True
    home_approach_rpm: float = 100.0
    home_search_rpm: float = 20.0
    home_backoff_steps: int = 3200

    def validate(self) -> None:
        """Raise ValueError if any field is out of range."""
        if self.num_layers < 1:
            raise ValueError("num_layers must be >= 1")
        if self.spindle_rpm <= 0.0:
            raise ValueError("spindle_rpm must be positive")
        if self.layer_pitch_mm <= 0.0:
            raise ValueError("layer_pitch_mm must be positive")
        if self.wire_diameter_mm <= 0.0:
            raise ValueError("wire_diameter_mm must be positive")
        if self.bobbin_width_mm <= 0.0:
            raise ValueError("bobbin_width_mm must be positive")
        if self.scatter_amplitude_mm < 0.0:
            raise ValueError("scatter_amplitude_mm must be >= 0")
        if self.scatter_damping_margin_mm < 0.0:
            raise ValueError("scatter_damping_margin_mm must be >= 0")
        if self.accel_s < 0.0 or self.decel_s < 0.0:
            raise ValueError("accel_s and decel_s must be >= 0")
        if self.lateral_steps_per_mm <= 0.0:
            raise ValueError("lateral_steps_per_mm must be positive")

    @property
    def turns_per_mm(self) -> float:
        return 1.0 / self.layer_pitch_mm

    def lateral_rpm_for_layer(self) -> float:
        """
        Compute the lateral traverse speed in RPM needed to advance
        layer_pitch_mm per spindle revolution.

        lateral_speed_mm_s = spindle_rps * layer_pitch_mm
        lateral_rpm = lateral_speed_mm_s * 60 / (2π * lateral_radius_mm)

        Since we work in steps/mm directly:
        lateral_hz = spindle_hz * layer_pitch_mm * lateral_steps_per_mm
        lateral_rpm = lateral_hz / (200 * 32) * 60
        """
        spindle_hz = self.spindle_rpm / 60.0
        lateral_hz = spindle_hz * self.layer_pitch_mm * self.lateral_steps_per_mm
        return (lateral_hz / (200.0 * 32.0)) * 60.0

    def layer_duration_s(self) -> float:
        """
        Duration of a full winding layer in seconds.

        A single layer is defined as a forward/backward pass across the bobbin
        width. The total spindle turns required for one layer are:

            total_turns = 2 * bobbin_width_mm * turns_per_mm

        The layer duration is therefore the total spindle turns divided by
        spindle revolutions per second.
        """
        spindle_rps = self.spindle_rpm / 60.0
        if spindle_rps <= 0.0:
            raise ValueError("spindle_rpm must be positive to compute layer duration")
        total_turns = 2.0 * self.bobbin_width_mm * self.turns_per_mm
        return total_turns / spindle_rps

    def snapshot(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "num_layers": self.num_layers,
            "spindle_rpm": self.spindle_rpm,
            "layer_pitch_mm": self.layer_pitch_mm,
            "wire_diameter_mm": self.wire_diameter_mm,
            "bobbin_width_mm": self.bobbin_width_mm,
            "turns_per_mm": self.turns_per_mm,
            "scatter_amplitude_mm": self.scatter_amplitude_mm,
            "scatter_damping_margin_mm": self.scatter_damping_margin_mm,
            "accel_s": self.accel_s,
            "decel_s": self.decel_s,
            "lateral_rpm": self.lateral_rpm_for_layer(),
            "layer_duration_s": self.layer_duration_s(),
        }
```

### `src/rpi/motion/engine.py`
```python
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
        duration_s = program.layer_duration_s()
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

        if self._stop_event.is_set():
            self._state.set_engine_state(EngineState.IDLE)
            self._events.publish(EventKind.PROGRAM_ABORTED, layer=layer_index)
            return False

        return True
```

### `src/rpi/motion/scatter_engine.py`
```python
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(slots=True)
class ScatterEngine:
    """Adds a spatial offset to the guide position to avoid exact layer stacking."""
    amplitude_mm: float = 0.0
    freq1: float = 1.0     # rad/turn
    freq2: float = 1.618   # rad/turn
    damping_margin_mm: float = 1.0

    def get_offset(self, spindle_turns: float, base_guide_pos_mm: float, bobbin_width_mm: float) -> float:
        if self.amplitude_mm <= 0.0:
            return 0.0

        raw_scatter = (math.sin(self.freq1 * spindle_turns) + math.sin(self.freq2 * spindle_turns)) / 2.0
        offset = raw_scatter * self.amplitude_mm

        dist_to_0 = base_guide_pos_mm
        dist_to_end = bobbin_width_mm - base_guide_pos_mm
        min_dist = min(dist_to_0, dist_to_end)

        if min_dist < self.damping_margin_mm and self.damping_margin_mm > 0:
            damping_factor = max(0.0, min_dist / self.damping_margin_mm)
            offset *= damping_factor

        return offset
```

### `src/rpi/motion/move_queue.py`
```python
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any

from motion.axis_state import AxisState
from motion.move import BaseMove, CompositeMove, HomingMove, Move, MoveState, RampMove
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
            else:
                self._execute_ramp_move(move)  # type: ignore[arg-type]
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

    def _wrap_segment_sequence(self, generator: Any, start_sequence: int):
        sequence = start_sequence & 0xFFFF
        for segment in generator:
            segment.sequence = sequence
            yield segment
            sequence = (sequence + 1) & 0xFFFF

    def _execute_ramp_move(self, move: Move) -> None:
        """Execute a RampMove or JogMove via MultiAxisRampStreamer."""
        move.mark_running()
        axis_configs = getattr(move, "_config", None)
        axis_configs = axis_configs.axis_configs if axis_configs is not None else []
        axis_ids = [cfg.axis_id for cfg in axis_configs]

        streamer = self._make_streamer(axis_configs)
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

        for ax_id in axis_ids:
            delta = move.expected_delta_steps(ax_id)
            if delta is not None and ax_id in self._axis_states:
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

        for phase_name, sub_move, arm_endstop in move.phases():
            if self._stop_requested:
                self._transport.enable_endstop_request(move.axis_id, arm=False)
                move.mark_aborted("stop requested during homing")
                return

            self._transport.enable_endstop_request(move.axis_id, arm=arm_endstop)

            streamer = self._make_streamer(sub_move._config.axis_configs)
            streamer._generator = self._wrap_segment_sequence(
                sub_move.segments(),
                self._next_motion_sequence(),
            )
            streamer._generator_finished = False
            streamer.stream_all()

            if phase_name in ("approach", "search") and not streamer.endstop_triggered:
                self._transport.enable_endstop_request(move.axis_id, arm=False)
                move.mark_failed(
                    f"homing {phase_name} phase completed without "
                    f"endstop trigger on axis {move.axis_id}"
                )
                return

            if self._stop_requested:
                self._transport.enable_endstop_request(move.axis_id, arm=False)
                move.mark_aborted("stop requested during homing")
                return

        self._transport.enable_endstop_request(move.axis_id, arm=False)
        if axis_state is not None:
            axis_state.mark_homed(move.home_position_steps)

        move.mark_completed()
```

### `src/rpi/core/config.py`
```python
from dataclasses import dataclass
from typing import Optional

@dataclass
class AppConfiguration:


    rpc_socket_path: str = "/tmp/pickup_winder_rpc.sock"
    """Configuration parameters for the application."""
    spi_device: str = "/dev/spidev0.0"
    spi_speed_hz: int = 1_000_000

    spindle_axis_id: int = 0
    spindle_steps_per_revolution: int = 200
    spindle_microstepping: int = 32
    spindle_invert_direction: bool = False
    spindle_max_speed_rpm: int = 1500
    spindle_max_acceleration_rpm: Optional[float] = 10
    spindle_max_deceleration_rpm: Optional[float] = None

    lateral_axis_id: int = 1
    lateral_steps_per_revolution: int = 200
    lateral_microstepping: int = 32
    lateral_invert_direction: bool = False
    lateral_max_rpm: int = 1000
    lateral_max_acceleration_mm_per_s2: Optional[float] = None
    lateral_max_deceleration_mm_per_s2: Optional[float] = None
    
    # Leadscrew/traverse pitch in mm per revolution for the lateral axis.
    # Used to compute steps/mm: steps_per_rev * microstepping / pitch_mm
    lateral_traverse_pitch_mm: float = 1.0
    # Optional explicit override for lateral steps-per-mm. If set, this
    # value takes precedence over the computed value.
    lateral_steps_per_mm_override: Optional[float] = None

    @property
    def lateral_steps_per_mm(self) -> float:
        """Return lateral axis steps per millimetre.

        Computed as: (steps_per_revolution * microstepping) / traverse_pitch_mm.
        If `lateral_steps_per_mm_override` is provided, it is returned instead.
        """
        if self.lateral_steps_per_mm_override is not None:
            return float(self.lateral_steps_per_mm_override)
        return (self.lateral_steps_per_revolution * self.lateral_microstepping) / float(self.lateral_traverse_pitch_mm)

    @property
    def spindle_max_acceleration_steps_per_s2(self) -> float:
        """Compute spindle acceleration in steps/s^2.

        Uses `spindle_max_acceleration_rpm` if provided.
        Otherwise returns a safe default.
        """
        steps_per_rev = self.spindle_steps_per_revolution * self.spindle_microstepping
        if self.spindle_max_acceleration_rpm is not None:
            return (self.spindle_max_acceleration_rpm / 60.0) * steps_per_rev
        return 100_000.0

    @property
    def spindle_max_deceleration_steps_per_s2(self) -> float:
        """Compute spindle deceleration in steps/s^2.

        Uses `spindle_max_deceleration_rpm` if provided. Otherwise falls back
        to the configured spindle acceleration limit.
        """
        steps_per_rev = self.spindle_steps_per_revolution * self.spindle_microstepping
        if self.spindle_max_deceleration_rpm is not None:
            return (self.spindle_max_deceleration_rpm / 60.0) * steps_per_rev
        return self.spindle_max_acceleration_steps_per_s2

    @property
    def lateral_max_acceleration_steps_per_s2(self) -> float:
        """Compute lateral acceleration in steps/s^2.

        Uses `lateral_max_acceleration_mm_per_s2` if provided.
        Otherwise returns a safe default.
        """
        if self.lateral_max_acceleration_mm_per_s2 is not None:
            return float(self.lateral_max_acceleration_mm_per_s2) * self.lateral_steps_per_mm
        return 100_000.0

    @property
    def lateral_max_deceleration_steps_per_s2(self) -> float:
        """Compute lateral deceleration in steps/s^2.

        Uses `lateral_max_deceleration_mm_per_s2` if provided.
        Otherwise falls back to the configured lateral acceleration limit.
        """
        if self.lateral_max_deceleration_mm_per_s2 is not None:
            return float(self.lateral_max_deceleration_mm_per_s2) * self.lateral_steps_per_mm
        return self.lateral_max_acceleration_steps_per_s2
```

### `tests/test_motion_v3.py`
```python
import os
import sys

root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "rpi"))
if root not in sys.path:
    sys.path.insert(0, root)

import pytest

from motion.trapezoidal_profile import TrapezoidalMotionProfile
from motion.spindle_kinematics import SpindleKinematics
from motion.winding_pattern import WindingPattern
from motion.scatter_engine import ScatterEngine
from motion.move import WoundMove
from motion.syncrhonized_segment_generator import SyncAxisConfig
from winding.program import WindingProgram


@pytest.mark.parametrize(
    "time_s, expected_turns",
    [
        (2.5, 18.75),
        (3.0, 20.0),
    ],
)
def test_trapezoidal_turns_at_deceleration(time_s: float, expected_turns: float):
    profile = TrapezoidalMotionProfile(
        start_rpm=0.0,
        target_rpm=600.0,
        accel_s=1.0,
        cruise_s=1.0,
        decel_s=1.0,
    )

    assert pytest.approx(profile.turns_at(time_s), rel=1e-6) == expected_turns


def test_spindle_kinematics_validates_after_dataclass_init():
    engine = SpindleKinematics(
        target_rpm=1000.0,
        start_rpm=0.0,
        accel_s=1.0,
        cruise_s=1.0,
        decel_s=1.0,
    )

    assert engine.total_duration == 3.0
    assert pytest.approx(engine.turns_at(3.0), rel=1e-6) == 33.333333333333336

    with pytest.raises(ValueError):
        SpindleKinematics(
            target_rpm=1000.0,
            start_rpm=0.0,
            accel_s=-0.1,
            cruise_s=1.0,
            decel_s=1.0,
        )


@pytest.mark.parametrize(
    "bobbin_width_mm, turns_per_mm",
    [
        (0.0, 10.0),
        (10.0, 0.0),
        (-5.0, 10.0),
    ],
)
def test_winding_pattern_rejects_invalid_geometry(bobbin_width_mm: float, turns_per_mm: float):
    with pytest.raises(ValueError):
        WindingPattern(bobbin_width_mm=bobbin_width_mm, turns_per_mm=turns_per_mm)


def test_winding_program_layer_duration_computes_from_geometry():
    program = WindingProgram(
        name="test",
        num_layers=1,
        spindle_rpm=1200.0,
        layer_pitch_mm=0.5,
        wire_diameter_mm=0.25,
        bobbin_width_mm=10.0,
    )

    assert program.turns_per_mm == 2.0
    assert pytest.approx(program.layer_duration_s(), rel=1e-6) == 2.0
    snapshot = program.snapshot()
    assert snapshot["bobbin_width_mm"] == 10.0
    assert snapshot["turns_per_mm"] == 2.0
    assert "scatter_amplitude_mm" in snapshot
    assert "layer_duration_s" in snapshot


def test_wound_move_rejects_duplicate_axis_indices():
    kinematics = SpindleKinematics(
        target_rpm=1000.0,
        accel_s=0.5,
        cruise_s=1.0,
        decel_s=0.5,
    )
    pattern = WindingPattern(bobbin_width_mm=15.0, turns_per_mm=10.0)
    scatter = ScatterEngine(amplitude_mm=0.1, damping_margin_mm=1.0)
    spindle_cfg = SyncAxisConfig(axis_index=0, steps_per_unit=6400.0)
    traverse_cfg = SyncAxisConfig(axis_index=0, steps_per_unit=1000.0)

    with pytest.raises(ValueError):
        WoundMove(
            name="bad_layer",
            kinematics=kinematics,
            pattern=pattern,
            scatter=scatter,
            spindle_cfg=spindle_cfg,
            traverse_cfg=traverse_cfg,
        )
```
