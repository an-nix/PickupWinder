# Architecture V4 — PickupWinder Host

## 1. Changelog V3 → V4

| Fichier | Correction | Décision technique retenue |
|---|---|---|
| src/rpi/core/config.py | Clarification unités d’accélération/décélération | Unité canonique conservée pour spindle: RPM/s (`spindle_max_acceleration_rpm`, `spindle_max_deceleration_rpm`) avec conversion documentée `(rpm_per_s / 60) * steps_per_rev`. Lateral conservé en mm/s² puis converti via `lateral_steps_per_mm`. |
| src/rpi/motion/move.py | Ajout API publique d’axes impliqués | Ajout propriété abstraite `axis_ids` sur `BaseMove`; implémentée par `RampMove`, `JogMove`, `WoundMove`, `HomingMove`. Ajout propriété publique `axis_configs` sur `Move` (optionnelle, surchargée là où disponible). |
| src/rpi/motion/move_queue.py | Bug silencieux `WoundMove` sans `_config`, accès privé `_config` en homing | Dispatch explicite: `CompositeMove -> _execute_homing`, `WoundMove -> _execute_wound_move`, `Move -> _execute_ramp_move`. Ajout `_make_wound_streamer()` et suppression de l’accès externe à `sub_move._config`. |
| src/rpi/winding/program.py | Méthode morte + hardcode latéral + scatter fréquences non exposées | Suppression `lateral_rpm_for_layer()`. Ajout `scatter_freq1`/`scatter_freq2` avec defaults rétrocompatibles (1.0 / 1.618). Snapshot enrichi. |
| src/rpi/motion/engine.py | Propagation fréquences scatter | `wound_run()` et `_run_layer()` passent `scatter_freq1`/`scatter_freq2` à `ScatterEngine`. |
| src/rpi/jsonrpc/winding_handler.py | Exposition RPC des fréquences scatter | Signature `winding.wound_run` enrichie avec `scatter_freq1`/`scatter_freq2` sans changer le nom RPC. |
| tests/test_motion_v3.py | Compléments de couverture | Ajout cas floating-point sur `turns_at`, cas `t=0`, cas `start_rpm>0`, et test de conversion d’unités config (`600 RPM/s -> 64000 steps/s²`). |

## 2. Sources complètes — tous les fichiers modifiés

### core/config.py
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
    # Unit: RPM/s (revolutions per minute gained per second).
    spindle_max_acceleration_rpm: Optional[float] = 10
    # Unit: RPM/s (revolutions per minute lost per second).
    spindle_max_deceleration_rpm: Optional[float] = None

    lateral_axis_id: int = 1
    lateral_steps_per_revolution: int = 200
    lateral_microstepping: int = 32
    lateral_invert_direction: bool = False
    lateral_max_rpm: int = 1000
    # Unit: mm/s² on traverse axis.
    lateral_max_acceleration_mm_per_s2: Optional[float] = None
    # Unit: mm/s² on traverse axis.
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

        Uses `spindle_max_acceleration_rpm` (RPM/s) if provided.
        Otherwise returns a safe default.
        """
        steps_per_rev = self.spindle_steps_per_revolution * self.spindle_microstepping
        if self.spindle_max_acceleration_rpm is not None:
            return (self.spindle_max_acceleration_rpm / 60.0) * steps_per_rev
        return 100_000.0

    @property
    def spindle_max_deceleration_steps_per_s2(self) -> float:
        """Compute spindle deceleration in steps/s^2.

        Uses `spindle_max_deceleration_rpm` (RPM/s) if provided. Otherwise falls back
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

### motion/move.py
```python
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Iterator

from motion import (
    AxisMotionConfig,
    MultiAxisSegmentGenerator,
    RampConfig,
    SpindleKinematics,
    WindingPattern,
    ScatterEngine,
    SyncAxisConfig,
    SynchronizedSegmentGenerator,
)
from transport.messages import MultiAxisSegment


class MoveState(Enum):
    PENDING = auto()
    RUNNING = auto()
    COMPLETED = auto()
    ABORTED = auto()
    FAILED = auto()


class BaseMove(ABC):
    """
    Shared lifecycle base for all motion commands.

    Carries state machine, timing, error, and snapshot logic.
    Does NOT prescribe a segment interface — concrete sub-hierarchies
    define their own execution contracts (``Move`` and ``CompositeMove``).

    Public properties
    -----------------
    state               Current MoveState.
    done                True when the move has reached a terminal state.
    error               Last error/abort message (None if none).
    aborted_by_endstop  True if an endstop triggered the abort.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self._state = MoveState.PENDING
        self._started_at: float | None = None
        self._completed_at: float | None = None
        self._error: str | None = None
        self._aborted_by_endstop: bool = False

    # ── Read-only public interface ─────────────────────────────────────────

    @property
    def state(self) -> MoveState:
        return self._state

    @property
    def done(self) -> bool:
        return self._state in (MoveState.COMPLETED, MoveState.ABORTED, MoveState.FAILED)

    @property
    def error(self) -> str | None:
        """Last error or abort reason; ``None`` if the move has not failed."""
        return self._error

    @property
    def aborted_by_endstop(self) -> bool:
        return self._aborted_by_endstop

    # ── Lifecycle transitions ─────────────────────────────────────────────

    def mark_running(self) -> None:
        self._state = MoveState.RUNNING
        self._started_at = time.monotonic()

    def mark_completed(self) -> None:
        self._state = MoveState.COMPLETED
        self._completed_at = time.monotonic()

    def mark_aborted(self, reason: str, by_endstop: bool = False) -> None:
        self._state = MoveState.ABORTED
        self._completed_at = time.monotonic()
        self._error = reason
        self._aborted_by_endstop = by_endstop

    def mark_failed(self, error: str) -> None:
        self._state = MoveState.FAILED
        self._completed_at = time.monotonic()
        self._error = error

    def snapshot(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "type": type(self).__name__,
            "state": self._state.name,
            "started_at": self._started_at,
            "completed_at": self._completed_at,
            "error": self._error,
            "aborted_by_endstop": self._aborted_by_endstop,
        }

    @abstractmethod
    def expected_delta_steps(self, axis_id: int) -> int | None:
        """Expected step delta for *axis_id* after this move completes.

        Return ``None`` if not applicable or variable (e.g. homing moves).
        """
        ...

    @property
    @abstractmethod
    def axis_ids(self) -> list[int]:
        """List of axis IDs involved in this move."""
        ...


class Move(BaseMove, ABC):
    """
    Abstract base for *segmented* moves — moves that produce a stream of
    ``MultiAxisSegment`` objects fed directly to the firmware streamer.

    Subclasses must implement:
      - ``segments()``            — yields ``MultiAxisSegment`` objects.
      - ``expected_delta_steps()``— inherited from ``BaseMove``.
    """

    @abstractmethod
    def segments(self) -> Iterator[MultiAxisSegment]:
        """Yield segments to be sent to the firmware."""
        ...

    @property
    def axis_configs(self) -> list[AxisMotionConfig] | None:
        """Optional public exposure of axis configs for streamer construction."""
        return None


class CompositeMove(BaseMove, ABC):
    """
    Abstract base for *composite* (multi-phase) moves that cannot be
    expressed as a single contiguous segment stream.

    ``CompositeMove`` deliberately does **not** declare ``segments()`` as
    obligatory, avoiding an LSP violation for moves like homing that
    require the MoveQueue to arm/disarm endstops between phases.

    Subclasses must implement:
      - ``phases()``              — returns ordered phase descriptors.
      - ``expected_delta_steps()``— inherited from ``BaseMove``.
    """

    @abstractmethod
    def phases(self) -> list[tuple[str, "Move", bool]]:
        """Return an ordered list of ``(phase_name, sub_move, endstop_armed)``
        tuples.  The ``MoveQueue`` iterates these, arming / disarming the
        endstop between phases.
        """
        ...


@dataclass(slots=True)
class RampMoveConfig:
    """Configuration for a simple ramp move on one or more axes."""

    axis_configs: list[AxisMotionConfig]
    segment_duration_s: float = 0.004


class RampMove(Move):
    """
    A move that follows a trapezoidal velocity ramp on one or more axes.
    This is the standard move type for spindle and lateral axis motion.

    Example:
        move = RampMove(
            name="spindle_run",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=0,
                        ramp=RampConfig(target_rpm=1500, accel_s=0.5,
                                        cruise_s=5.0, decel_s=0.5),
                    )
                ]
            ),
        )
    """

    def __init__(self, name: str, config: RampMoveConfig) -> None:
        super().__init__(name)
        self._config = config

    def segments(self) -> Iterator[MultiAxisSegment]:
        gen = MultiAxisSegmentGenerator(
            self._config.axis_configs,
            segment_duration_s=self._config.segment_duration_s,
        )
        yield from gen

    @property
    def axis_ids(self) -> list[int]:
        return [cfg.axis_id for cfg in self._config.axis_configs]

    @property
    def axis_configs(self) -> list[AxisMotionConfig]:
        return self._config.axis_configs

    def expected_delta_steps(self, axis_id: int) -> int | None:
        # Total steps = integral of hz over time. For position tracking
        # this is an estimate — return None to avoid false precision.
        # Subclass and override if exact step count is needed.
        return None


class HomingMove(CompositeMove):
    """
    A homing sequence on a single axis.

    Phase 1 — fast approach: move toward_endstop at approach_rpm until
               the endstop fires or max_approach_steps is reached.
    Phase 2 — backoff: move away from endstop by backoff_steps.
    Phase 3 — slow search: move toward_endstop at search_rpm until the
               endstop fires again. This is the true home position.
    Phase 4 — set position to home_position_steps.

    The MoveQueue must arm the endstop before each approach phase and
    disarm it during the backoff phase.

    ``HomingMove`` extends ``CompositeMove``: execution is driven by
    ``phases()``, not by ``segments()``.  This satisfies the Liskov
    Substitution Principle — ``HomingMove`` never claims to be a
    ``Move`` and will never be passed to a segment streamer directly.
    """

    def __init__(
        self,
        name: str,
        axis_id: int,
        steps_per_rev: int,
        approach_rpm: float,
        search_rpm: float,
        backoff_steps: int,
        max_approach_steps: int,
        home_position_steps: int = 0,
        segment_duration_s: float = 0.004,
        reverse_direction: bool = False,
    ) -> None:
        super().__init__(name)
        self.axis_id = axis_id
        self.steps_per_rev = steps_per_rev
        self.approach_rpm = approach_rpm
        self.search_rpm = search_rpm
        self.backoff_steps = backoff_steps
        self.max_approach_steps = max_approach_steps
        self.home_position_steps = home_position_steps
        self.segment_duration_s = segment_duration_s
        self.reverse_direction = reverse_direction

    def _make_approach_move(self) -> RampMove:
        """Phase 1: fast move toward endstop."""
        total_s = (self.max_approach_steps / float(self.steps_per_rev)) / (
            self.approach_rpm / 60.0
        )
        return RampMove(
            name=f"{self.name}:approach",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=self.axis_id,
                        ramp=RampConfig(
                            axis_id=self.axis_id,
                            steps_per_rev=self.steps_per_rev,
                            target_rpm=self.approach_rpm,
                            accel_s=min(0.2, total_s * 0.2),
                            cruise_s=max(total_s - 0.4, 0.0),
                            decel_s=min(0.2, total_s * 0.2),
                            reverse_direction=self.reverse_direction,
                        ),
                    )
                ],
                segment_duration_s=self.segment_duration_s,
            ),
        )

    def _make_backoff_move(self) -> RampMove:
        """Phase 2: move away from endstop."""
        total_s = (self.backoff_steps / float(self.steps_per_rev)) / (
            self.search_rpm / 60.0
        )
        return RampMove(
            name=f"{self.name}:backoff",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=self.axis_id,
                        ramp=RampConfig(
                            axis_id=self.axis_id,
                            steps_per_rev=self.steps_per_rev,
                            target_rpm=self.search_rpm,
                            accel_s=min(0.1, total_s * 0.3),
                            cruise_s=max(total_s - 0.2, 0.0),
                            decel_s=min(0.1, total_s * 0.3),
                            # Backoff moves AWAY from endstop = opposite direction
                            reverse_direction=not self.reverse_direction,
                        ),
                    )
                ],
                segment_duration_s=self.segment_duration_s,
            ),
        )

    def _make_search_move(self) -> RampMove:
        """Phase 3: slow move toward endstop for precise home."""
        total_s = (self.backoff_steps * 2 / float(self.steps_per_rev)) / (
            self.search_rpm / 60.0
        )
        return RampMove(
            name=f"{self.name}:search",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=self.axis_id,
                        ramp=RampConfig(
                            axis_id=self.axis_id,
                            steps_per_rev=self.steps_per_rev,
                            target_rpm=self.search_rpm,
                            accel_s=min(0.1, total_s * 0.2),
                            cruise_s=max(total_s - 0.2, 0.0),
                            decel_s=min(0.1, total_s * 0.2),
                            reverse_direction=self.reverse_direction,
                        ),
                    )
                ],
                segment_duration_s=self.segment_duration_s,
            ),
        )

    def phases(self) -> list[tuple[str, RampMove, bool]]:
        """
        Return an ordered list of ``(phase_name, sub_move, endstop_armed)``.
        The ``MoveQueue`` iterates this list, arming/disarming the endstop
        between phases and updating ``AxisState`` after search completes.
        """
        return [
            ("approach", self._make_approach_move(), True),
            ("backoff", self._make_backoff_move(), False),
            ("search", self._make_search_move(), True),
        ]

    def expected_delta_steps(self, axis_id: int) -> int | None:
        return None  # Position is set explicitly after homing completes.

    @property
    def axis_ids(self) -> list[int]:
        return [self.axis_id]


class JogMove(Move):
    """
    Move an axis by a fixed number of steps at a given speed.
    Useful for manual positioning and clearance moves.

    Example:
        move = JogMove(
            name="clear_endstop",
            axis_id=1,
            steps_per_rev=200 * 32,
            steps=3200,           # 0.5 rev at 32 microstep
            rpm=100.0,
            reverse_direction=True,
        )
    """

    def __init__(
        self,
        name: str,
        axis_id: int,
        steps_per_rev: int,
        steps: int,
        rpm: float,
        reverse_direction: bool = False,
        segment_duration_s: float = 0.004,
    ) -> None:
        super().__init__(name)
        self.axis_id = axis_id
        self._steps = steps
        self._reverse = reverse_direction
        total_s = (steps / float(steps_per_rev)) / (rpm / 60.0)
        self._config = RampMoveConfig(
            axis_configs=[
                AxisMotionConfig(
                    axis_id=axis_id,
                    ramp=RampConfig(
                        axis_id=axis_id,
                        steps_per_rev=steps_per_rev,
                        target_rpm=rpm,
                        accel_s=min(0.15, total_s * 0.2),
                        cruise_s=max(total_s - 0.3, 0.0),
                        decel_s=min(0.15, total_s * 0.2),
                        reverse_direction=reverse_direction,
                    ),
                )
            ],
            segment_duration_s=segment_duration_s,
        )

    def segments(self) -> Iterator[MultiAxisSegment]:
        gen = MultiAxisSegmentGenerator(
            self._config.axis_configs,
            segment_duration_s=self._config.segment_duration_s,
        )
        yield from gen

    def expected_delta_steps(self, axis_id: int) -> int | None:
        if axis_id != self.axis_id:
            return None
        return -self._steps if self._reverse else self._steps

    @property
    def axis_ids(self) -> list[int]:
        return [self.axis_id]

    @property
    def axis_configs(self) -> list[AxisMotionConfig]:
        return self._config.axis_configs


class WoundMove(Move):
    """
    A single continuous move executing the Electronic Gearing winding pattern.
    Utilizes SynchronizedSegmentGenerator to slave the Traverse to the Spindle.
    """
    def __init__(
        self,
        name: str,
        kinematics: SpindleKinematics,
        pattern: WindingPattern,
        scatter: ScatterEngine,
        spindle_cfg: SyncAxisConfig,
        traverse_cfg: SyncAxisConfig,
        segment_duration_s: float = 0.004,
    ) -> None:
        super().__init__(name)
        if spindle_cfg.axis_index == traverse_cfg.axis_index:
            raise ValueError("spindle_cfg.axis_index and traverse_cfg.axis_index must differ")
        if segment_duration_s <= 0.0:
            raise ValueError("segment_duration_s must be positive")
        if kinematics.total_duration <= 0.0:
            raise ValueError("kinematics.total_duration must be positive")

        self.kinematics = kinematics
        self.pattern = pattern
        self.scatter = scatter
        self.spindle_cfg = spindle_cfg
        self.traverse_cfg = traverse_cfg
        self.segment_duration_s = segment_duration_s

    def segments(self) -> Iterator[MultiAxisSegment]:
        gen = SynchronizedSegmentGenerator(
            self.kinematics,
            self.pattern,
            self.scatter,
            self.spindle_cfg,
            self.traverse_cfg,
            segment_duration_s=self.segment_duration_s,
        )
        yield from gen

    def expected_delta_steps(self, axis_id: int) -> int | None:
        return None

    @property
    def axis_ids(self) -> list[int]:
        return [self.spindle_cfg.axis_index, self.traverse_cfg.axis_index]
```

### motion/move_queue.py
```python
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any

from motion.axis_state import AxisState
from motion.move import BaseMove, CompositeMove, HomingMove, Move, WoundMove
from motion.ramp_config import RampConfig
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
        spindle_steps_per_unit = max(1, int(round(move.spindle_cfg.steps_per_unit)))
        traverse_steps_per_unit = max(1, int(round(move.traverse_cfg.steps_per_unit)))

        spindle_ramp = RampConfig(
            axis_id=move.spindle_cfg.axis_index,
            steps_per_rev=spindle_steps_per_unit,
            target_rpm=max(move.kinematics.target_rpm, 1.0),
            accel_s=max(move.kinematics.accel_s, 0.0),
            cruise_s=max(move.kinematics.cruise_s, 0.0),
            decel_s=max(move.kinematics.decel_s, 0.0),
            reverse_direction=move.spindle_cfg.reverse_direction,
        )
        traverse_ramp = RampConfig(
            axis_id=move.traverse_cfg.axis_index,
            steps_per_rev=traverse_steps_per_unit,
            target_rpm=max(move.kinematics.target_rpm, 1.0),
            accel_s=max(move.kinematics.accel_s, 0.0),
            cruise_s=max(move.kinematics.cruise_s, 0.0),
            decel_s=max(move.kinematics.decel_s, 0.0),
            reverse_direction=move.traverse_cfg.reverse_direction,
        )
        return MultiAxisRampStreamer(
            self._transport,
            [
                StreamAxisConfig(axis_id=move.spindle_cfg.axis_index, ramp=spindle_ramp),
                StreamAxisConfig(axis_id=move.traverse_cfg.axis_index, ramp=traverse_ramp),
            ],
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
            move.mark_aborted("stop requested")
            return

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

            # Arm or disarm endstop for this phase.
            self._transport.enable_endstop_request(move.axis_id, arm=arm_endstop)

            # Execute the sub-move.
            sub_move_axis_configs = sub_move.axis_configs
            if not sub_move_axis_configs:
                self._transport.enable_endstop_request(move.axis_id, arm=False)
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

        # All phases complete — disarm endstop and set home position.
        self._transport.enable_endstop_request(move.axis_id, arm=False)
        if axis_state is not None:
            axis_state.mark_homed(move.home_position_steps)

        move.mark_completed()
```

### winding/program.py
```python
from __future__ import annotations

from dataclasses import dataclass
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
    scatter_freq1: float = 1.0
    scatter_freq2: float = 1.618
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
        if self.scatter_freq1 <= 0.0:
            raise ValueError("scatter_freq1 must be positive")
        if self.scatter_freq2 <= 0.0:
            raise ValueError("scatter_freq2 must be positive")
        if self.accel_s < 0.0 or self.decel_s < 0.0:
            raise ValueError("accel_s and decel_s must be >= 0")
        if self.lateral_steps_per_mm <= 0.0:
            raise ValueError("lateral_steps_per_mm must be positive")

    @property
    def turns_per_mm(self) -> float:
        return 1.0 / self.layer_pitch_mm

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
            "scatter_freq1": self.scatter_freq1,
            "scatter_freq2": self.scatter_freq2,
            "accel_s": self.accel_s,
            "decel_s": self.decel_s,
            "layer_duration_s": self.layer_duration_s(),
        }
```

### motion/engine.py
```python
from __future__ import annotations

import threading
import time
from typing import Any, Optional

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
from motion.ramp_config import compute_ramp_times
from transport.spi_transport import Esp32SpiTransport
from core.config import AppConfiguration
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
        config: AppConfiguration | None = None,
    ) -> None:
        self._transport = transport
        self._state = shared_state
        self._events = event_bus
        self._config = config or AppConfiguration()

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
        accel_s: float | None,
        cruise_s: float | None,
        decel_s: float | None,
        bobbin_width_mm: float,
        turns_per_mm: float,
        scatter_amplitude_mm: float = 0.0,
        scatter_damping_margin_mm: float = 0.0,
        scatter_freq1: float = 1.0,
        scatter_freq2: float = 1.618,
        spindle_reverse: bool = False,
        traverse_reverse: bool = False,
    ) -> None:
        """
        Execute a synchronized winding operation (Electronic Gearing).
        Only allowed when engine is IDLE.
        """
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError("Winding run only allowed when engine is IDLE")

        if accel_s is None or cruise_s is None or decel_s is None:
            total_turns = 2.0 * bobbin_width_mm * turns_per_mm
            target_rps = target_rpm / 60.0
            duration_s = total_turns / target_rps if target_rps > 0.0 else 0.0
            if duration_s <= 0.0:
                duration_s = 0.05
            steps_per_rev = (
                self._config.spindle_steps_per_revolution
                * self._config.spindle_microstepping
            )
            computed_accel_s, computed_cruise_s, computed_decel_s = compute_ramp_times(
                target_rpm=target_rpm,
                duration_s=duration_s,
                max_accel_steps_per_s2=self._config.spindle_max_acceleration_steps_per_s2,
                max_decel_steps_per_s2=self._config.spindle_max_deceleration_steps_per_s2,
                steps_per_rev=steps_per_rev,
            )
            accel_s = accel_s if accel_s is not None else computed_accel_s
            cruise_s = cruise_s if cruise_s is not None else computed_cruise_s
            decel_s = decel_s if decel_s is not None else computed_decel_s

        move = WoundMove(
            name="winding_electronic_gearing",
            kinematics=SpindleKinematics(
                target_rpm=target_rpm,
                start_rpm=0.0,
                accel_s=accel_s,
                cruise_s=cruise_s,
                decel_s=decel_s,
            ),
            pattern=WindingPattern(
                bobbin_width_mm=bobbin_width_mm,
                turns_per_mm=turns_per_mm,
            ),
            scatter=ScatterEngine(
                amplitude_mm=scatter_amplitude_mm,
                freq1=scatter_freq1,
                freq2=scatter_freq2,
                damping_margin_mm=scatter_damping_margin_mm,
            ),
            spindle_cfg=SyncAxisConfig(
                axis_index=spindle_axis_id,
                steps_per_unit=(
                    self._config.spindle_steps_per_revolution
                    * self._config.spindle_microstepping
                ),
                reverse_direction=spindle_reverse,
            ),
            traverse_cfg=SyncAxisConfig(
                axis_index=traverse_axis_id,
                steps_per_unit=(
                    self._config.lateral_steps_per_revolution
                    * self._config.lateral_microstepping
                ),
                reverse_direction=traverse_reverse,
            ),
        )
        self._move_queue.enqueue(move)

    def run_axis(
        self,
        duration_s: float,
        targets: list[dict[str, Any]],
    ) -> None:
        """
        Queue one or two axes for a trapezoidal ramp move.
        The acceleration and deceleration times are calculated from the
        application configuration limits.
        """
        if self._state.engine_state != EngineState.IDLE:
            raise RuntimeError("run_axis only allowed when engine is IDLE")
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if not targets:
            raise ValueError("targets must contain at least one axis")
        if len(targets) > 2:
            raise ValueError("run_axis supports at most two axes")

        axis_ids: set[int] = set()
        axis_configs: list[AxisMotionConfig] = []

        for target in targets:
            if not isinstance(target, dict):
                raise TypeError("each target must be a dict")
            if "axis_id" not in target or "rpm" not in target:
                raise ValueError("each target must contain axis_id and rpm")

            axis_id = int(target["axis_id"])
            rpm = float(target["rpm"])
            reverse = bool(target.get("reverse", False))

            if axis_id in axis_ids:
                raise ValueError(f"duplicate axis_id {axis_id}")
            axis_ids.add(axis_id)

            if axis_id == self._config.spindle_axis_id:
                max_rpm = float(self._config.spindle_max_speed_rpm)
                steps_per_rev = self._config.spindle_steps_per_revolution * self._config.spindle_microstepping
                max_accel = self._config.spindle_max_acceleration_steps_per_s2
                max_decel = self._config.spindle_max_deceleration_steps_per_s2
            elif axis_id == self._config.lateral_axis_id:
                max_rpm = float(self._config.lateral_max_rpm)
                steps_per_rev = self._config.lateral_steps_per_revolution * self._config.lateral_microstepping
                max_accel = self._config.lateral_max_acceleration_steps_per_s2
                max_decel = self._config.lateral_max_deceleration_steps_per_s2
            else:
                raise ValueError(f"Unsupported axis_id {axis_id}")

            if rpm <= 0.0:
                raise ValueError("rpm must be positive")

            target_rpm = min(rpm, max_rpm)
            accel_s, cruise_s, decel_s = compute_ramp_times(
                target_rpm=target_rpm,
                duration_s=duration_s,
                max_accel_steps_per_s2=max_accel,
                max_decel_steps_per_s2=max_decel,
                steps_per_rev=steps_per_rev,
            )

            axis_configs.append(
                AxisMotionConfig(
                    axis_id=axis_id,
                    ramp=RampConfig(
                        axis_id=axis_id,
                        target_rpm=target_rpm,
                        accel_s=accel_s,
                        cruise_s=cruise_s,
                        decel_s=decel_s,
                        reverse_direction=reverse,
                    ),
                )
            )

        move = RampMove(
            name="run_axis",
            config=RampMoveConfig(axis_configs=axis_configs),
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

        if axis_id == self._config.spindle_axis_id:
            steps_per_rev = (
                self._config.spindle_steps_per_revolution
                * self._config.spindle_microstepping
            )
        elif axis_id == self._config.lateral_axis_id:
            steps_per_rev = (
                self._config.lateral_steps_per_revolution
                * self._config.lateral_microstepping
            )
        else:
            raise ValueError(f"jog: unsupported axis_id {axis_id}")

        move = JogMove(
            name=f"jog_{axis_id}",
            axis_id=axis_id,
            steps_per_rev=steps_per_rev,
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
        steps_per_rev = (
            self._config.lateral_steps_per_revolution
            * self._config.lateral_microstepping
        )
        move = HomingMove(
            name="home_lateral",
            axis_id=program.lateral_axis_id,
            steps_per_rev=steps_per_rev,
            approach_rpm=program.home_approach_rpm,
            search_rpm=program.home_search_rpm,
            backoff_steps=program.home_backoff_steps,
            max_approach_steps=int(steps_per_rev * 20),
        )
        self._move_queue.enqueue(move)
        self._wait_for_move_queue()

        if move.state.name == "COMPLETED":
            self._events.publish(
                EventKind.HOMING_COMPLETED, axis_id=program.lateral_axis_id
            )
            return True
        else:
            msg = f"Homing failed: {move.error}"
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
            not self._stop_event.is_set()
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
                break
            time.sleep(poll_s)
```

### tests/test_motion_v3.py
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
from core.config import AppConfiguration


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


def test_trapezoidal_turns_clamp_beyond_total_duration():
    profile = TrapezoidalMotionProfile(
        start_rpm=0.0,
        target_rpm=600.0,
        accel_s=1.0,
        cruise_s=1.0,
        decel_s=1.0,
    )
    total = profile.total_duration
    assert profile.turns_at(0.0) == 0.0
    assert pytest.approx(profile.turns_at(total + 1e-9), rel=1e-12) == profile.turns_at(total)


def test_trapezoidal_deceleration_non_negative_with_nonzero_start_rpm():
    profile = TrapezoidalMotionProfile(
        start_rpm=300.0,
        target_rpm=600.0,
        accel_s=0.5,
        cruise_s=0.5,
        decel_s=0.5,
    )
    assert profile.rps_at(profile.total_duration) >= 0.0
    assert profile.turns_at(profile.total_duration) >= 0.0


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


def test_app_config_spindle_accel_unit_conversion_rpm_per_s_to_steps_per_s2():
    cfg = AppConfiguration(
        spindle_steps_per_revolution=200,
        spindle_microstepping=32,
        spindle_max_acceleration_rpm=600.0,
    )
    assert cfg.spindle_max_acceleration_steps_per_s2 == 64000.0
```

## 3. Sources complètes — fichiers non modifiés nécessaires à l’analyse

### transport/streamer.py
```python
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Any, Iterator, List

from transport.messages import (
    MULTI_AXIS_SEGMENT_BLOCK_SIZE,
    MultiAxisSegment,
    MultiAxisSegmentBlockPayload,
    SpiMessageResult,
    sequence_is_greater,
    sequence_is_less_equal,
)
from motion import AxisMotionConfig, MultiAxisSegmentGenerator, RampConfig
from transport.spi_transport import Esp32SpiTransport


@dataclass(slots=True)
class StreamAxisConfig:
    axis_id: int
    ramp: RampConfig
    minimum_free_blocks: int = 1
    prefill_blocks: int | None = None
    low_watermark_blocks: int | None = None
    max_queued_blocks: int | None = None
    ring_send_threshold: int = 1


class MultiAxisRampStreamer:
    """Minimal deterministic SPI motion streamer.

    The streamer is the host-side source of truth for motion segments.
    It sends pre-computed multi-axis segment blocks over SPI, tracks in-flight
    motion, and uses MCU status feedback to keep the ESP32 queue and ring filled
    without overflowing them.

    Multiple segments are packed per SPI frame (up to MULTI_AXIS_SEGMENT_BLOCK_SIZE)
    to ensure the firmware drain loop has deep look-ahead before starting the RMT.
    """

    TARGET_BUFFER_TIME_S = 0.10
    MIN_BUFFER_TIME_S = 0.06
    MAX_BUFFER_TIME_S = 0.12
    MIN_SEGMENT_TIME_S = 0.002
    MAX_SEGMENT_TIME_S = 0.005
    POLL_SLEEP_S = 0.0005
    MAX_INFLIGHT_SEGMENTS = 24

    # Planner→executor segment queue depth on the ESP32 (matches SEGMENT_QUEUE_DEPTH in firmware).
    SEGMENT_QUEUE_DEPTH = 128
    # Legacy constant kept for reference (= EXEC_BATCH_LIMIT * 2).
    # The active gate is now required_lookahead() which is speed-dependent.
    PLANNER_QUEUE_SEND_THRESHOLD = 32

    # ESP32 step ring capacity in firmware: one step consumes one ring entry.
    STEP_RING_CAPACITY = 4096
    RING_BUFFER_HEADROOM = 0.8

    @staticmethod
    def required_lookahead(steps_per_segment: int) -> int:
        """Speed-dependent minimum segment lookahead depth in the ESP32 planner queue.

        At low speed each segment contains very few steps, so the ring drains
        faster relative to the inter-segment host→ESP32 pipeline latency (~3–5 ms).
        A deeper buffer prevents ring underruns and motor stutter.

        Thresholds match firmware EXEC_BATCH_LIMIT tiers:
          < 10  steps → 48 segments (low speed,  ~50 RPM)
          < 50  steps → 32 segments (mid speed)
          >= 50 steps → 16 segments (high speed, > ~200 RPM)
        """
        if steps_per_segment < 10:
            return 48
        elif steps_per_segment < 50:
            return 32
        else:
            return 16

    def __init__(
        self,
        transport: Esp32SpiTransport,
        axis_streams: list[StreamAxisConfig],
        *,
        segment_duration_s: float = 0.004,
        target_buffer_time_s: float = TARGET_BUFFER_TIME_S,
        poll_interval_s: float = 0.001,
        print_every: int = 1,
        log_each_send: bool = False,
        send_log_path: str | None = None,
    ):
        self._transport = transport
        self._poll_interval_s = poll_interval_s
        self._print_every = max(print_every, 1)
        self._log_each_send = log_each_send
        self._send_log_path = send_log_path
        self._send_events: list[dict] = []
        self._stop_requested = False
        self._flush_sequence_requested: int | None = None
        self._endstop_triggered = False
        self._endstop_armed_axes: set[int] = set()

        self._axis_configs = [AxisMotionConfig(axis_id=s.axis_id, ramp=s.ramp) for s in axis_streams]
        self._axis_ids = [s.axis_id for s in axis_streams]
        self._segment_duration_s = max(self.MIN_SEGMENT_TIME_S, min(self.MAX_SEGMENT_TIME_S, segment_duration_s))
        self._target_buffer_time_s = self._compute_target_buffer_time(target_buffer_time_s)
        self._min_buffer_time_s = min(self.MIN_BUFFER_TIME_S, self._target_buffer_time_s * 0.5)

        self._inflight: deque[tuple[MultiAxisSegment, int]] = deque()
        self._buffered_time_s = 0.0
        self._last_sent_motion_seq = -1
        self._last_sent_transport_seq = -1
        self._planner_under_pressure = False  # True while planner_queue_free < threshold
        self._buffered_segments = 0           # SEGMENT_QUEUE_DEPTH - planner_queue_free
        self._current_steps_per_segment = 0  # updated per-segment; drives required_lookahead()
        self._prefilling = False              # suppresses pressure gate during initial prefill
        # Premature-completion detection
        self._last_confirmed_sequence: int = -1
        self._premature_notify_count: int = 0
        self._premature_notify_window_start: float = 0.0
        self._last_sequence_advance_time: float = time.time()
        self._last_sequence_advance_value: int = -1
        self._stall_timeout_s: float = 2.0  # stall if no progress for 2s

        self._sync_with_firmware_status()
        start_sequence = (
            (self._last_confirmed_sequence + 1) & 0xFFFF
            if self._last_confirmed_sequence >= 0
            else 0
        )
        self._generator = iter(
            MultiAxisSegmentGenerator(
                self._axis_configs,
                segment_duration_s=self._segment_duration_s,
                start_sequence=start_sequence,
            )
        )
        self._generator_finished = False

    # -- Helpers ---------------------------------------------------------------

    @property
    def buffered_segments(self) -> int:
        """Number of segments currently buffered in the planner→executor queue.

        Computed from the last received planner_queue_free field:
            buffered = SEGMENT_QUEUE_DEPTH - planner_queue_free

        This mirrors Klipper's "move queue available" check: when buffered_segments
        approaches SEGMENT_QUEUE_DEPTH the host should stop requesting more motion.
        Value is 0 when no status has been received yet.
        """
        return self._buffered_segments

    def _planner_queue_free(self, status) -> int:
        """Return planner_queue_free from status, defaulting to full if absent."""
        return int(getattr(status, "planner_queue_free", self.SEGMENT_QUEUE_DEPTH))

    def _check_planner_pressure(self, status) -> bool:
        """Return True (blocked) when the ESP32 planner buffer already has enough lookahead.

        Uses Klipper's move-queue model: send if buffered < needed, not if free > threshold.
        During initial prefill (_prefilling=True) the gate is bypassed entirely so
        the host can fill up to the speed-appropriate prefill target without interference.

        Logs edge transitions:
          - 'planner pressure' when planner_queue_free drops below 16
          - 'planner recovered' when planner_queue_free recovers above 64
        """
        pqf = self._planner_queue_free(status)
        self._buffered_segments = self.SEGMENT_QUEUE_DEPTH - pqf

        if pqf < 16 and not self._planner_under_pressure:
            self._planner_under_pressure = True
            print(f"[{self._timestamp()}] planner pressure: planner_queue_free={pqf} (< 16)")
        elif pqf > 64 and self._planner_under_pressure:
            self._planner_under_pressure = False
            print(f"[{self._timestamp()}] planner recovered: planner_queue_free={pqf} (> 64)")

        # During prefill we bypass the pressure gate so the host can seed a deep buffer.
        if self._prefilling:
            return False

        # Klipper model: block if the buffer already holds the required lookahead depth.
        # This inverts the old "send if free slots >= threshold" gate: we now gate on
        # buffered depth rather than remaining free space, which is speed-aware.
        needed = self.required_lookahead(self._current_steps_per_segment)
        return self._buffered_segments >= needed

    def _max_segments_per_cycle(self) -> int:
        """Speed-dependent send cap per polling cycle.

        At low speed (few steps/segment) the defer ring on the ESP32 cannot
        overflow (each segment contributes <10 ring entries) so a higher cap
        is safe and necessary to keep the ring fed between underruns.
        At high speed a lower cap prevents burst-after-throttle overflow.

          < 10  steps/segment → 16 segments/cycle (low speed, 50 RPM)
          < 50  steps/segment →  8 segments/cycle (mid speed)
          >= 50 steps/segment →  4 segments/cycle (high speed, > 200 RPM)
        """
        if self._current_steps_per_segment < 10:
            return 16
        elif self._current_steps_per_segment < 50:
            return 8
        else:
            return 4

    def _max_inflight_segments(self) -> int:
        """Speed-dependent in-flight segment cap.

        At low speed each segment executes slowly so more can be in-flight
        simultaneously without risking the host advancing too far ahead
        of the motor's actual position.
        """
        if self._current_steps_per_segment < 10:
            return 96
        elif self._current_steps_per_segment < 50:
            return 48
        else:
            return 24

    def _timestamp(self) -> str:
        now = time.time()
        seconds = int(now)
        milliseconds = int((now - seconds) * 1000)
        return time.strftime(f"%H:%M:%S.{milliseconds:03d}", time.localtime(now))

    def _compute_target_buffer_time(self, requested_time_s: float) -> float:
        requested_time_s = max(self.MIN_BUFFER_TIME_S, min(self.MAX_BUFFER_TIME_S, requested_time_s))
        if not self._axis_configs:
            return requested_time_s
        max_hz = max(config.ramp.target_hz for config in self._axis_configs)
        if max_hz <= 0.0:
            return requested_time_s
        safe_time_s = (self.STEP_RING_CAPACITY * self.RING_BUFFER_HEADROOM) / max_hz
        return max(self.MIN_BUFFER_TIME_S, min(requested_time_s, safe_time_s))

    def _sync_with_firmware_status(self) -> None:
        """Synchronize stream state with the ESP32's last executed sequence."""
        try:
            status = self._transport.get_status()
        except Exception:
            return

        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return

        self._last_confirmed_sequence = received_sequence
        self._last_sequence_advance_value = received_sequence
        self._last_sequence_advance_time = time.time()

    def _enable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, status = self._transport.set_axis_enabled_request(axis_id, True)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"enable axis {axis_id} failed with result=0x{status.last_result:02X}")

    def _disable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, status = self._transport.set_axis_enabled_request(axis_id, False)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"disable axis {axis_id} failed with result=0x{status.last_result:02X}")

    def _queue_full(self, status) -> bool:
        for axis_id in self._axis_ids:
            if axis_id < len(status.queue_free_slots) and status.queue_free_slots[axis_id] == 0:
                return True
            if hasattr(status, "ring_free_slots") and axis_id < len(status.ring_free_slots) and status.ring_free_slots[axis_id] == 0:
                return True
        return False

    def _remove_confirmed_segments(self, status) -> None:
        last_executed = int(getattr(status, "last_executed_sequence", -1))
        while self._inflight:
            segment, _transport_seq = self._inflight[0]
            if sequence_is_less_equal(segment.sequence, last_executed):
                self._buffered_time_s -= segment.duration_us / 1_000_000.0
                self._buffered_time_s = max(0.0, self._buffered_time_s)
                self._inflight.popleft()
            else:
                break

    def _check_premature_completion(self, status) -> None:
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return

        # True premature: ESP32 reports completion of a seq we never sent.
        if self._last_sent_motion_seq >= 0 and sequence_is_greater(received_sequence, self._last_sent_motion_seq):
            now = time.time()
            if now - self._premature_notify_window_start > 1.0:
                self._premature_notify_count = 0
                self._premature_notify_window_start = now
            self._premature_notify_count += 1
            print(
                f"[{self._timestamp()}] WARNING premature completion: "
                f"got seq={received_sequence} but last sent={self._last_sent_motion_seq} "
                f"— ESP32 reported completion before host sent this segment "
                f"(count={self._premature_notify_count})"
            )

        # Advance confirmed pointer only when sequence strictly increases.
        if self._last_confirmed_sequence < 0 or sequence_is_greater(
            received_sequence, self._last_confirmed_sequence
        ):
            self._last_confirmed_sequence = received_sequence

    def _check_stall(self, status) -> bool:
        """Return True and request stop if last_executed_sequence has not
        advanced for _stall_timeout_s while segments are in flight.

        A stall means the RMT is dead and pushBlock() is likely deadlocked.
        Requesting a stop+flush allows the host to recover gracefully.
        """
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return False
        if not self._inflight:
            # No in-flight segments — not a stall, just idle.
            self._last_sequence_advance_time = time.time()
            return False

        if self._last_sequence_advance_value < 0 or sequence_is_greater(
            received_sequence, self._last_sequence_advance_value
        ):
            self._last_sequence_advance_value = received_sequence
            self._last_sequence_advance_time = time.time()
            return False

        elapsed = time.time() - self._last_sequence_advance_time
        if elapsed > self._stall_timeout_s:
            print(
                f"[{self._timestamp()}] WARNING motor stall detected: "
                f"last_executed_sequence={received_sequence} unchanged for "
                f"{elapsed:.1f}s with {len(self._inflight)} segments in flight "
                f"— requesting stop and flush"
            )
            self.request_stop()
            self.request_flush(self._last_sent_motion_seq)
            return True
        return False

    def _check_endstop(self, status) -> bool:
        """Return True if an endstop was triggered on any armed axis.

        Reads endstop_armed_mask from the status frame. Sets
        _endstop_triggered and requests a stop + flush when triggered.
        """
        armed_mask = int(getattr(status, "endstop_armed_mask", 0))
        lateral_state = int(getattr(status, "lateral_endstop_state", 0xFF))
        # lateral_endstop_state values (from firmware LateralEndstopState):
        #   0x00 = PRESENT_OPEN, 0x01 = PRESENT_CLOSED, 0xFF = ABSENT
        PRESENT_CLOSED = 0x01
        if lateral_state == PRESENT_CLOSED and armed_mask != 0:
            if not self._endstop_triggered:
                self._endstop_triggered = True
                flush_seq = self._last_sent_motion_seq
                self.request_stop()
                self.request_flush(flush_seq)
            return True
        return False

    def _record_send_event(self, segment: MultiAxisSegment, transport_seq: int, status) -> None:
        event = {
            "timestamp": time.time(),
            "timestamp_str": self._timestamp(),
            "transport_sequence": transport_seq,
            "segment_sequence": segment.sequence,
            "duration_us": segment.duration_us,
            "axis_count": len(segment.steps),
            "total_steps": sum(segment.steps),
            "queue_free": list(status.queue_free_slots),
            "ring_free": list(status.ring_free_slots),
            "last_result": int(status.last_result),
            "enabled_mask": int(status.enabled_mask),
            "running_mask": int(status.running_mask),
        }
        self._send_events.append(event)
        if self._log_each_send:
            print(
                f"[{event['timestamp_str']}] send tx_seq={transport_seq} "
                f"motion_seq={segment.sequence} duration_us={segment.duration_us} "
                f"total_steps={event['total_steps']} result=0x{event['last_result']:02X}"
            )

    def _write_send_log(self) -> None:
        if self._send_log_path is None:
            return
        with open(self._send_log_path, "w", encoding="utf-8") as handle:
            json.dump(self._send_events, handle, indent=2)

    # -- Endstop control -------------------------------------------------------

    def arm_endstop(self, axis_id: int) -> None:
        """Send ENABLE_ENDSTOP arm command to firmware and track locally.

        Call before starting a move that should stop on endstop contact.
        """
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=True)
        self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
        self._endstop_armed_axes.add(axis_id)

    def disarm_endstop(self, axis_id: int) -> None:
        """Send ENABLE_ENDSTOP disarm command to firmware.

        Call before a clearance move that must pass through the endstop.
        """
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=False)
        self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
        self._endstop_armed_axes.discard(axis_id)

    @property
    def endstop_triggered(self) -> bool:
        return self._endstop_triggered

    # -- Stop / flush ----------------------------------------------------------

    def request_stop(self) -> None:
        self._stop_requested = True

    def has_stop_been_requested(self) -> bool:
        return self._stop_requested

    def request_flush(self, sequence: int) -> None:
        self._flush_sequence_requested = sequence

    def flush_until(self, sequence: int):
        status = self._transport.flush_until(sequence)
        self._inflight.clear()
        self._buffered_time_s = 0.0
        return status

    # -- Core streaming primitives ---------------------------------------------

    def _collect_and_send_batch(self, status) -> tuple[int, Any] | None:
        """Collect up to MULTI_AXIS_SEGMENT_BLOCK_SIZE segments and send one frame.

        Returns (segments_sent, last_status) on success, (0, status) on
        QUEUE_FULL, or None when nothing can be batched (buffer target
        reached, inflight limit reached, or generator already exhausted).

        Packing multiple segments per frame is critical for ring pre-fill:
        the firmware drain loop processes all queued frames before starting
        the RMT, so more segments per frame = deeper ring buffer at startup.
        """
        if self._generator_finished:
            return None

        batch: list[MultiAxisSegment] = []
        while (
            len(batch) < MULTI_AXIS_SEGMENT_BLOCK_SIZE
            and self._buffered_time_s < self._target_buffer_time_s
            and len(self._inflight) < self._max_inflight_segments()
            and not self._queue_full(status)
            and not self._check_planner_pressure(status)
        ):
            try:
                segment = next(self._generator)
            except StopIteration:
                self._generator_finished = True
                break

            if self._last_sent_motion_seq >= 0 and not sequence_is_greater(
                segment.sequence, self._last_sent_motion_seq
            ):
                raise RuntimeError(
                    f"motion sequence not strictly increasing: "
                    f"got {segment.sequence}, last was {self._last_sent_motion_seq}"
                )
            batch.append(segment)
            # Keep speed estimate current so required_lookahead() uses fresh data.
            if segment.steps:
                self._current_steps_per_segment = sum(segment.steps)

        if not batch:
            return None

        payload = MultiAxisSegmentBlockPayload(
            axis_ids=self._axis_ids,
            block_seq=batch[0].sequence,
            segments=batch,
        )
        transport_seq, send_status = self._transport.send_multi_axis_segment_block_request(payload)

        if send_status.last_result == int(SpiMessageResult.OK):
            for seg in batch:
                self._inflight.append((seg, transport_seq))
                self._buffered_time_s += seg.duration_us / 1_000_000.0
                self._last_sent_motion_seq = seg.sequence
                self._record_send_event(seg, transport_seq, send_status)
            self._last_sent_transport_seq = transport_seq
            return len(batch), send_status
        elif send_status.last_result == int(SpiMessageResult.QUEUE_FULL):
            return 0, send_status
        else:
            raise RuntimeError(
                f"segment batch starting motion_seq={batch[0].sequence} "
                f"failed with result=0x{send_status.last_result:02X}"
            )

    def _prefill(self, status) -> tuple[int, Any]:
        """Pre-send segments to seed the ESP32 planner queue before RMT starts.

        The prefill target is speed-dependent:
          - Low speed  (steps_per_segment < 10): 64 segments (half of SEGMENT_QUEUE_DEPTH)
            because the ring drains very fast at low RPM and needs a large head start.
          - Otherwise: required_lookahead(current_steps_per_segment) segments.

        The _prefilling flag is set for the duration of this call so that
        _check_planner_pressure() does not prematurely gate sends before the
        target depth has been reached.

        Returns (total_segments_sent, last_status).
        """
        is_low_speed = self._current_steps_per_segment < 10
        if is_low_speed:
            prefill_target = 64   # half of SEGMENT_QUEUE_DEPTH
        else:
            prefill_target = self.required_lookahead(self._current_steps_per_segment)

        total = 0
        last_status = status
        self._prefilling = True
        try:
            while total < prefill_target:
                result = self._collect_and_send_batch(last_status)
                if result is None:
                    break
                n, last_status = result
                total += n
                if n == 0:  # QUEUE_FULL — firmware can't accept more right now
                    break
        finally:
            self._prefilling = False
        return total, last_status

    def _should_sleep(self) -> float:
        """Return sleep duration in seconds based on buffer fullness.

        Returns 0.0 if the buffer needs immediate refill.
        """
        if self._buffered_time_s >= self._target_buffer_time_s:
            return self._segment_duration_s
        if self._buffered_time_s >= self._min_buffer_time_s:
            return self._segment_duration_s / 2.0
        return 0.0

    # -- Main streaming loop ---------------------------------------------------

    def stream_all(self) -> int:
        self._generator_finished = False
        status = self._transport.get_status()
        axes_enabled = False

        try:
            self._enable_axes()
            axes_enabled = True
            status = self._transport.get_status()

            total_segments, status = self._prefill(status)

            while True:
                if self._stop_requested:
                    if self._flush_sequence_requested is not None:
                        self.flush_until(self._flush_sequence_requested)
                    break

                status = self._transport.get_status()
                self._remove_confirmed_segments(status)
                self._check_premature_completion(status)
                if self._check_stall(status):
                    break

                if self._check_endstop(status):
                    break

                # Rate-limit sends to MAX_SEGMENTS_PER_CYCLE per polling iteration to
                # prevent burst-after-throttle that overflows the ESP32 defer ring.
                cycle_segments_sent = 0
                while not self._generator_finished:
                    if cycle_segments_sent >= self._max_segments_per_cycle():
                        break
                    result = self._collect_and_send_batch(status)
                    if result is None:
                        break
                    n, status = result
                    if n == 0:  # QUEUE_FULL
                        break
                    cycle_segments_sent += n
                    total_segments += n
                    if total_segments % self._print_every == 0:
                        print(
                            f"[{self._timestamp()}] segments={total_segments} "
                            f"buffered={self._buffered_time_s*1000:.1f}ms "
                            f"inflight={len(self._inflight)} "
                            f"queue_free={status.queue_free_slots} "
                            f"ring_free={status.ring_free_slots} "
                            f"underrun={status.underrun_count}"
                        )

                if self._flush_sequence_requested is not None:
                    self.flush_until(self._flush_sequence_requested)
                    self._flush_sequence_requested = None

                if self._generator_finished and not self._inflight:
                    break

                sleep_s = self._should_sleep()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        finally:
            if axes_enabled:
                try:
                    self._disable_axes()
                except Exception as exc:
                    print(
                        f"[{self._timestamp()}] WARNING failed to disable axes: {exc}"
                    )

        self._write_send_log()
        return total_segments
```

### motion/winding_pattern.py
```python
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class WindingPattern:
    """Converts Spindle position (turns) to Guide position (mm) in a triangular wave."""
    bobbin_width_mm: float
    turns_per_mm: float

    def __post_init__(self) -> None:
        if self.turns_per_mm <= 0.0:
            raise ValueError("turns_per_mm must be positive")
        if self.bobbin_width_mm <= 0.0:
            raise ValueError("bobbin_width_mm must be positive")

    def guide_pos_mm(self, spindle_turns: float) -> float:
        total_dist = spindle_turns / self.turns_per_mm
        cycle_length = 2.0 * self.bobbin_width_mm

        mod_dist = total_dist % cycle_length
        if mod_dist <= self.bobbin_width_mm:
            return mod_dist
        return cycle_length - mod_dist
```

### motion/scatter_engine.py
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

### jsonrpc/winding_handler.py
```python
"""RPC handler for all ``winding.*`` methods.

``WindingRpcHandler`` replaces the ad-hoc closure pattern that was
previously used inside ``winding_main._register_winding_rpc_methods()``.

Each public method maps 1-to-1 with a JSON-RPC method name.  The handler
is registered once at startup via ``WindingRpcHandler.register_all()``,
which keeps ``winding_main.py`` free of imperative registration boilerplate.

Compatibility guarantee
-----------------------
All ``winding.*`` method *names* and their JSON parameter contracts are
unchanged.  Callers (e.g. ``run_axis_rpc.py``) require no modification.
"""

from __future__ import annotations

from typing import Any

from core.shared_state import SharedState
from jsonrpc.handlers import AppRpcHandler
from jsonrpc.protocol import JsonRpcError
from motion.engine import WindingEngine
from winding.program import WindingProgram


class WindingRpcHandler:
    """Exposes every ``winding.*`` RPC method as a typed public method.

    Parameters
    ----------
    engine:
        The ``WindingEngine`` instance that owns the ``MoveQueue``.
    shared_state:
        The ``SharedState`` object, used for axis-state queries.
    """

    def __init__(
        self,
        engine: WindingEngine,
        shared_state: SharedState,
    ) -> None:
        self._engine = engine
        self._state = shared_state

    # ── Registration ───────────────────────────────────────────────────────

    def register_all(self, handler: AppRpcHandler) -> None:
        """Register every ``winding.*`` method on *handler*."""
        handler.register_method("winding.submit_program", self.submit_program)
        handler.register_method("winding.stop", self.stop)
        handler.register_method("winding.jog", self.jog)
        handler.register_method("winding.wound_run", self.wound_run)
        handler.register_method("winding.run_axis", self.run_axis)
        handler.register_method("winding.clear_fault", self.clear_fault)
        handler.register_method("winding.status", self.status)
        handler.register_method("winding.axis_state", self.axis_state)
        handler.register_method("winding.arm_endstop", self.arm_endstop)
        handler.register_method("winding.disarm_endstop", self.disarm_endstop)

    # ── RPC methods ────────────────────────────────────────────────────────

    def submit_program(self, program: dict) -> dict[str, Any]:
        """Queue a full winding program for execution."""
        if not isinstance(program, dict):
            raise JsonRpcError(-32602, "Invalid params: expected program object")
        p = WindingProgram(**program)
        self._engine.submit_program(p)
        return {"status": "queued", "program": p.snapshot()}

    def stop(self, _params: Any | None = None) -> dict[str, str]:
        """Abort the current move and clear the queue."""
        self._engine.request_stop()
        return {"status": "stopping"}

    def jog(
        self,
        axis_id: int,
        steps: int,
        rpm: float,
        reverse: bool = False,
    ) -> dict[str, Any]:
        """Jog *axis_id* by *steps* at *rpm*."""
        self._engine.jog(axis_id=axis_id, steps=steps, rpm=rpm, reverse=reverse)
        return {"status": "queued"}

    def wound_run(
        self,
        spindle_axis_id: int,
        traverse_axis_id: int,
        target_rpm: float,
        bobbin_width_mm: float,
        turns_per_mm: float,
        accel_s: float | None = None,
        cruise_s: float | None = None,
        decel_s: float | None = None,
        scatter_amplitude_mm: float = 0.0,
        scatter_damping_margin_mm: float = 0.0,
        scatter_freq1: float = 1.0,
        scatter_freq2: float = 1.618,
        spindle_reverse: bool = False,
        traverse_reverse: bool = False,
    ) -> dict[str, Any]:
        """Execute a synchronized winding operation (Electronic Gearing)."""
        self._engine.wound_run(
            spindle_axis_id=spindle_axis_id,
            traverse_axis_id=traverse_axis_id,
            target_rpm=target_rpm,
            accel_s=accel_s,
            cruise_s=cruise_s,
            decel_s=decel_s,
            bobbin_width_mm=bobbin_width_mm,
            turns_per_mm=turns_per_mm,
            scatter_amplitude_mm=scatter_amplitude_mm,
            scatter_damping_margin_mm=scatter_damping_margin_mm,
            scatter_freq1=scatter_freq1,
            scatter_freq2=scatter_freq2,
            spindle_reverse=spindle_reverse,
            traverse_reverse=traverse_reverse,
        )
        return {"status": "queued"}

    def run_axis(
        self,
        duration_s: float,
        targets: list[dict[str, Any]],
    ) -> dict[str, Any]:
        """Queue a config-limited trapezoidal ramp for one or two axes."""
        self._engine.run_axis(duration_s=duration_s, targets=targets)
        return {"status": "queued"}

    def clear_fault(self, _params: Any | None = None) -> dict[str, str]:
        """Clear FAULT state so a new program can be submitted."""
        self._engine.clear_fault()
        return {"status": "ok"}

    def status(self, _params: Any | None = None) -> dict[str, Any]:
        """Return combined engine and move-queue status snapshot."""
        return self._engine.status()

    def axis_state(self, axis_id: int) -> dict[str, Any]:
        """Return the position / state snapshot for a single axis."""
        state = self._state.axis_states.get(axis_id)
        if state is None:
            raise JsonRpcError(-32602, f"Unknown axis_id: {axis_id}")
        return state.snapshot()

    def arm_endstop(self, axis_id: int) -> dict[str, Any]:
        """Arm the endstop for *axis_id*."""
        self._engine.arm_endstop(axis_id)
        return {"status": "armed", "axis_id": axis_id}

    def disarm_endstop(self, axis_id: int) -> dict[str, Any]:
        """Disarm the endstop for *axis_id*."""
        self._engine.disarm_endstop(axis_id)
        return {"status": "disarmed", "axis_id": axis_id}
```

## 4. Points d’attention restants

1. **`src/rpi/motion/move_queue.py` — requête endstop non confirmée**  
   `enable_endstop_request()` est utilisé sans `wait_for_request_result()`.  
   **Impact potentiel**: en cas de latence SPI ou erreur firmware, une phase homing peut démarrer avec un état d’armement non garanti.

2. **`src/rpi/motion/move_queue.py` — conversion `steps_per_unit -> steps_per_rev` dans `_make_wound_streamer()`**  
   La conversion est pragmatique pour fabriquer une config streamer minimale, mais sémantiquement `steps_per_unit` n’est pas toujours un “steps/rev”.  
   **Impact potentiel**: influence mineure possible sur heuristiques de buffer/target time si ces champs sont réutilisés au-delà de l’override de générateur.

3. **`src/rpi/core/config.py` — mélange d’unités d’accélération selon axes**  
   Spindle est en RPM/s, latéral en mm/s². C’est documenté mais hétérogène.  
   **Impact potentiel**: risque d’erreur de paramétrage côté opérateur/API si les champs sont manipulés sans UI explicite d’unités.

4. **`src/rpi/motion/scatter_engine.py` — validation des fréquences**  
   `ScatterEngine` n’impose pas directement la validité de `freq1`/`freq2`; la validation est portée par `WindingProgram`.  
   **Impact potentiel**: une création directe de `ScatterEngine` (hors programme) peut passer des fréquences non souhaitées.

## 5. Schéma de dépendances mis à jour (mermaid)

```mermaid
flowchart TB
    subgraph RPC
        RPC[JsonRpcServer]
        WRH[WindingRpcHandler]
    end

    subgraph Engine
        WE[WindingEngine]
        MQ[MoveQueue]
    end

    subgraph Moves
        BM[BaseMove]
        MV[Move]
        CM[CompositeMove]
        RM[RampMove]
        JM[JogMove]
        WM[WoundMove]
        HM[HomingMove]
    end

    subgraph Executors
        EXM[_execute_move]
        EXR[_execute_ramp_move]
        EXW[_execute_wound_move]
        EXH[_execute_homing]
    end

    subgraph Motion
        MASG[MultiAxisSegmentGenerator]
        SSG[SynchronizedSegmentGenerator]
        SPK[SpindleKinematics]
        WP[WindingPattern]
        SE[ScatterEngine]
    end

    subgraph Transport
        STR[MultiAxisRampStreamer]
        SPI[Esp32SpiTransport]
    end

    RPC --> WRH --> WE --> MQ
    MQ --> EXM

    BM --> MV
    BM --> CM
    MV --> RM
    MV --> JM
    MV --> WM
    CM --> HM

    EXM -->|CompositeMove| EXH
    EXM -->|WoundMove| EXW
    EXM -->|Move| EXR

    EXR --> STR
    EXW --> STR
    EXH --> STR

    RM --> MASG
    JM --> MASG
    WM --> SSG
    SSG --> SPK
    SSG --> WP
    SSG --> SE

    STR --> SPI
```
