# Architecture V7 — PickupWinder Host

## 1. Changelog V6 → V7

| Fichier | Correction | Décision technique retenue |
|---|---|---|
| src/rpi/jsonrpc/handlers.py | Suppression des lambdas `lambda _:` incompatibles avec `dispatch(params=None)` | Enregistrement de wrappers `_rpc_*` tolérants à `params` optionnel (`Any | None = None`) pour tous les endpoints `winder.*`. |
| src/rpi/winding_main.py | `AppRpcHandler` construit avec `engine` | `rpc_handler = AppRpcHandler(app=engine)` pour exposer la config réelle via `winder.config`. |
| src/rpi/motion/synchronized_segment_generator.py | Nouveau module au nom corrigé | Renommage logique `syncrhonized` → `synchronized` sans changer le comportement interne. |
| src/rpi/motion/syncrhonized_segment_generator.py | Ajout d’un shim de compatibilité | Shim avec `DeprecationWarning` + re-export des symboles du nouveau module pour compatibilité descendante. |
| src/rpi/motion/__init__.py | Imports lazy corrigés vers `synchronized_segment_generator` | Référence canonique corrigée, plus de dépendance interne au nom fautif. |
| src/rpi/motion/ramp.py | Shim de compatibilité explicitement déprécié | **Option A choisie** : docstring enrichie + `DeprecationWarning`, chemin public recommandé `motion.multi_axis_segment_generator`. |
| src/rpi/motion/move_queue.py | Ordre corrigé dans `_execute_wound_move` | Invalidation de position avant `move.mark_completed()` pour éviter fenêtre d’observation incohérente. |
| src/rpi/transport/streamer.py | Imports `typing` nettoyés | Suppression de `Iterator`/`List` inutilisés, conservation de `Any`. |
| tests/test_motion_v3.py | Imports migrés vers le nouveau module + test de compatibilité | Tous les imports utilisent `motion.synchronized_segment_generator`; ajout d’un test validant le `DeprecationWarning` du shim legacy. |

---

## 2. Sources complètes — tous les fichiers modifiés

### src/rpi/jsonrpc/handlers.py
```python
from __future__ import annotations

import dataclasses
import time
from typing import Any, Callable, Dict

from .protocol import JsonRpcMethodNotFoundError

MethodCallback = Callable[[Any | None], Any]


class RpcHandler:
    def __init__(self) -> None:
        self._methods: Dict[str, MethodCallback] = {}

    def register_method(self, method: str, callback: MethodCallback) -> None:
        self._methods[method] = callback

    def dispatch(self, method: str, params: Any | None) -> Any:
        callback = self._methods.get(method)
        if callback is None:
            raise JsonRpcMethodNotFoundError(method)
        if params is None:
            return callback()
        if isinstance(params, list):
            return callback(*params)
        if isinstance(params, dict):
            try:
                return callback(**params)
            except TypeError:
                return callback(params)
        return callback(params)


class AppRpcHandler(RpcHandler):
    def __init__(self, app: Any | None = None) -> None:
        super().__init__()
        self.app = app
        self.started_at = time.time()
        self.register_method("winder.ping", self._rpc_ping)
        self.register_method("winder.status", self._rpc_status)
        self.register_method("winder.shutdown", self._rpc_shutdown)
        self.register_method("winder.config", self._rpc_config)

    def _rpc_ping(self, _params: Any | None = None) -> dict[str, str]:
        return self.ping()

    def _rpc_status(self, _params: Any | None = None) -> dict[str, Any]:
        return self.status()

    def _rpc_shutdown(self, _params: Any | None = None) -> dict[str, str]:
        return self.shutdown()

    def _rpc_config(self, _params: Any | None = None) -> dict[str, Any]:
        return self.config()

    def ping(self) -> dict[str, str]:
        return {"message": "pong"}

    def status(self) -> dict[str, Any]:
        return {
            "uptime_s": round(time.time() - self.started_at, 2),
            "configured": bool(self.app is not None),
        }

    def shutdown(self) -> dict[str, str]:
        return {"message": "shutdown-not-implemented"}

    def config(self) -> dict[str, Any]:
        if self.app is None:
            return {}
        cfg = getattr(self.app, "_config", None) or getattr(self.app, "config", None)
        if cfg is None:
            return {}

        if dataclasses.is_dataclass(cfg):
            cfg_dict = dataclasses.asdict(cfg)
            return {k: v for k, v in cfg_dict.items() if not k.startswith("_")}

        if hasattr(cfg, "__dict__"):
            return {
                k: v for k, v in vars(cfg).items()
                if not k.startswith("_")
            }

        result: dict[str, Any] = {}
        for attr in dir(cfg):
            if attr.startswith("_"):
                continue
            try:
                value = getattr(cfg, attr)
            except Exception:
                continue
            if callable(value):
                continue
            result[attr] = value
        return result
```

### src/rpi/winding_main.py
```python
from __future__ import annotations

import logging
import signal
import sys
import time

from core.config import AppConfiguration
from core.events import EventBus
from core.shared_state import SharedState
from motion.axis_state import AxisState
from motion.engine import WindingEngine
from transport.spi_transport import Esp32SpiTransport
from jsonrpc import AppRpcHandler, JsonRpcServer
from jsonrpc.winding_handler import WindingRpcHandler

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(name)s %(levelname)s %(message)s",
)
logger = logging.getLogger("main")


def _parse_spi_device(device_path: str) -> tuple[int, int]:
    import re

    match = re.fullmatch(r"/dev/spidev(\d+)\.(\d+)", device_path)
    if match is None:
        raise ValueError("spi_device must be in the form /dev/spidev<bus>.<device>")
    return int(match.group(1)), int(match.group(2))


def main() -> None:
    config = AppConfiguration()

    bus, device = _parse_spi_device(config.spi_device)
    logger.info("Opening SPI transport on %s @ %d Hz", config.spi_device, config.spi_speed_hz)
    transport = Esp32SpiTransport(
        bus=bus,
        device=device,
        speed_hz=config.spi_speed_hz,
        mode=0,
    )

    axis_states = {
        config.spindle_axis_id: AxisState(
            axis_id=config.spindle_axis_id,
            steps_per_rev=config.spindle_steps_per_revolution * config.spindle_microstepping,
        ),
        config.lateral_axis_id: AxisState(
            axis_id=config.lateral_axis_id,
            steps_per_rev=config.lateral_steps_per_revolution * config.lateral_microstepping,
        ),
    }

    shared_state = SharedState(axis_states=axis_states)
    event_bus = EventBus()

    engine = WindingEngine(
        transport=transport,
        shared_state=shared_state,
        event_bus=event_bus,
        config=config,
    )
    rpc_handler = AppRpcHandler(app=engine)
    WindingRpcHandler(engine=engine, shared_state=shared_state).register_all(rpc_handler)
    rpc_server = JsonRpcServer(
        handler=rpc_handler,
        event_bus=event_bus,
        socket_path="/tmp/winding.sock",
    )

    engine.start()
    rpc_server.start()
    logger.info("Winding controller started")

    def _shutdown(sig, frame) -> None:
        logger.info("Shutdown requested (signal %s)", sig)
        engine.stop()
        rpc_server.stop()
        transport.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
```

### src/rpi/motion/synchronized_segment_generator.py
```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

from .segment_generator import AxisStepProfile, StepProfileSegmentGenerator
from .spindle_kinematics import SpindleKinematics
from .winding_pattern import WindingPattern
from .scatter_engine import ScatterEngine
from transport.messages import MultiAxisSegment


@dataclass(slots=True)
class SyncAxisConfig:
    axis_index: int
    steps_per_unit: float
    reverse_direction: bool = False


class SynchronizedSegmentGenerator(StepProfileSegmentGenerator):
    """Generates strictly synchronized motion blocks (Electronic Gearing) for winding."""

    def __init__(
        self,
        spindle_kinematics: SpindleKinematics,
        pattern: WindingPattern,
        scatter: ScatterEngine,
        spindle_config: SyncAxisConfig,
        traverse_config: SyncAxisConfig,
        segment_duration_s: float = 0.004,
        start_sequence: int = 0,
    ):
        self.kinematics = spindle_kinematics
        self.pattern = pattern
        self.scatter = scatter
        self.spindle_config = spindle_config
        self.traverse_config = traverse_config

        def spindle_steps_at(t: float) -> float:
            return spindle_kinematics.turns_at(t) * spindle_config.steps_per_unit

        def traverse_steps_at(t: float) -> float:
            turns = spindle_kinematics.turns_at(t)
            base_traverse_mm = pattern.guide_pos_mm(turns)
            scatter_offset = scatter.get_offset(
                turns,
                base_traverse_mm,
                pattern.bobbin_width_mm,
            )
            return (base_traverse_mm + scatter_offset) * traverse_config.steps_per_unit

        axis_profiles = [
            AxisStepProfile(
                axis_index=spindle_config.axis_index,
                step_at=spindle_steps_at,
                reverse_direction=spindle_config.reverse_direction,
                total_duration=spindle_kinematics.total_duration,
            ),
            AxisStepProfile(
                axis_index=traverse_config.axis_index,
                step_at=traverse_steps_at,
                reverse_direction=traverse_config.reverse_direction,
                total_duration=spindle_kinematics.total_duration,
            ),
        ]

        super().__init__(axis_profiles, segment_duration_s=segment_duration_s, start_sequence=start_sequence)
```

### src/rpi/motion/syncrhonized_segment_generator.py
```python
from __future__ import annotations

import warnings

warnings.warn(
    "syncrhonized_segment_generator is deprecated, use synchronized_segment_generator",
    DeprecationWarning,
    stacklevel=2,
)

from .synchronized_segment_generator import (  # noqa: F401
    SyncAxisConfig,
    SynchronizedSegmentGenerator,
)
```

### src/rpi/motion/ramp.py
```python
from __future__ import annotations

"""Compatibility shim for legacy motion imports.

This module exposes the old motion names while the real implementation
has moved into `multi_axis_segment_generator.py`.

Deprecated: direct imports from motion.multi_axis_segment_generator
are preferred. This shim will be removed in a future version.
"""

import warnings

warnings.warn(
    "motion.ramp is deprecated, use motion.multi_axis_segment_generator",
    DeprecationWarning,
    stacklevel=2,
)

from .multi_axis_segment_generator import AxisMotionConfig, MultiAxisSegmentGenerator

__all__ = ["AxisMotionConfig", "MultiAxisSegmentGenerator"]
```

### src/rpi/motion/__init__.py
```python
"""Motion planning and control for PickupWinder.

Lazy imports to avoid circular dependencies with transport module.
"""

__all__ = [
    "RampConfig",
    "compute_ramp_times",
    "AxisMotionConfig",
    "MultiAxisSegmentGenerator",
    "SpindleKinematics",
    "WindingPattern",
    "ScatterEngine",
    "SyncAxisConfig",
    "SynchronizedSegmentGenerator",
    "WindingEngine",
]


def __getattr__(name: str):
    if name == "RampConfig":
        from .ramp_config import RampConfig
        return RampConfig
    if name == "compute_ramp_times":
        from .ramp_config import compute_ramp_times
        return compute_ramp_times
    if name == "AxisMotionConfig":
        from .ramp import AxisMotionConfig
        return AxisMotionConfig
    if name == "MultiAxisSegmentGenerator":
        from .ramp import MultiAxisSegmentGenerator
        return MultiAxisSegmentGenerator
    if name == "SpindleKinematics":
        from .spindle_kinematics import SpindleKinematics
        return SpindleKinematics
    if name == "WindingPattern":
        from .winding_pattern import WindingPattern
        return WindingPattern
    if name == "ScatterEngine":
        from .scatter_engine import ScatterEngine
        return ScatterEngine
    if name == "SyncAxisConfig":
        from .synchronized_segment_generator import SyncAxisConfig
        return SyncAxisConfig
    if name == "SynchronizedSegmentGenerator":
        from .synchronized_segment_generator import SynchronizedSegmentGenerator
        return SynchronizedSegmentGenerator
    if name == "WindingEngine":
        from .engine import WindingEngine
        return WindingEngine
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return __all__
```

### src/rpi/motion/move_queue.py
```python
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
```

### src/rpi/transport/streamer.py
```python
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Any

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
        self._initialize_streamer_state(
            transport=transport,
            axis_configs=[AxisMotionConfig(axis_id=s.axis_id, ramp=s.ramp) for s in axis_streams],
            axis_ids=[s.axis_id for s in axis_streams],
            segment_duration_s=segment_duration_s,
            target_buffer_time_s=target_buffer_time_s,
            poll_interval_s=poll_interval_s,
            print_every=print_every,
            log_each_send=log_each_send,
            send_log_path=send_log_path,
            explicit_target_hz=None,
        )

    @classmethod
    def from_axis_ids(
        cls,
        transport: Esp32SpiTransport,
        axis_ids: list[int],
        *,
        target_hz: float,
        segment_duration_s: float = 0.004,
        poll_interval_s: float = 0.001,
        print_every: int = 1,
        target_buffer_time_s: float = 0.150,
    ) -> "MultiAxisRampStreamer":
        """Build a streamer from explicit axis IDs and a known target frequency.

        Use this constructor when the move generator is external (e.g. `WoundMove`)
        and no reliable `RampConfig` objects are available.

        Differences vs `__init__`:
          - `__init__`: derives `target_hz` from `RampConfig.target_hz`.
          - `from_axis_ids`: receives `target_hz` explicitly and avoids synthetic ramps.
        """
        if not axis_ids:
            raise ValueError("axis_ids must not be empty")
        if target_hz <= 0.0:
            raise ValueError("target_hz must be positive")

        streamer = cls.__new__(cls)
        streamer._initialize_streamer_state(
            transport=transport,
            axis_configs=[],
            axis_ids=axis_ids,
            segment_duration_s=segment_duration_s,
            target_buffer_time_s=target_buffer_time_s,
            poll_interval_s=poll_interval_s,
            print_every=print_every,
            log_each_send=False,
            send_log_path=None,
            explicit_target_hz=target_hz,
        )
        return streamer

    def _initialize_streamer_state(
        self,
        *,
        transport: Esp32SpiTransport,
        axis_configs: list[AxisMotionConfig],
        axis_ids: list[int],
        segment_duration_s: float,
        target_buffer_time_s: float,
        poll_interval_s: float,
        print_every: int,
        log_each_send: bool,
        send_log_path: str | None,
        explicit_target_hz: float | None,
    ) -> None:
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

        self._axis_configs = axis_configs
        self._axis_ids = list(axis_ids)
        self._segment_duration_s = max(self.MIN_SEGMENT_TIME_S, min(self.MAX_SEGMENT_TIME_S, segment_duration_s))
        if explicit_target_hz is None:
            max_hz = 0.0
            if self._axis_configs:
                max_hz = max(config.ramp.target_hz for config in self._axis_configs)
            self._target_buffer_time_s = self._safe_buffer_time_s(target_buffer_time_s, max_hz)
        else:
            self._target_buffer_time_s = self._safe_buffer_time_s(target_buffer_time_s, explicit_target_hz)
        self._min_buffer_time_s = min(self.MIN_BUFFER_TIME_S, self._target_buffer_time_s * 0.5)

        self._inflight: deque[tuple[MultiAxisSegment, int]] = deque()
        self._buffered_time_s = 0.0
        self._last_sent_motion_seq = -1
        self._last_sent_transport_seq = -1
        self._planner_under_pressure = False  # True while planner_queue_free < threshold
        self._buffered_segments = 0           # SEGMENT_QUEUE_DEPTH - planner_queue_free
        self._current_steps_per_segment = 0  # updated per-segment; drives required_lookahead()
        if explicit_target_hz is not None:
            self._current_steps_per_segment = max(1, int(round(explicit_target_hz * self._segment_duration_s)))
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
        if self._axis_configs:
            self._generator = iter(
                MultiAxisSegmentGenerator(
                    self._axis_configs,
                    segment_duration_s=self._segment_duration_s,
                    start_sequence=start_sequence,
                )
            )
        else:
            self._generator = iter(())
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

    def _safe_buffer_time_s(self, requested_time_s: float, max_hz: float) -> float:
        requested_time_s = max(self.MIN_BUFFER_TIME_S, min(self.MAX_BUFFER_TIME_S, requested_time_s))
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

### tests/test_motion_v3.py
```python
import os
import sys
import logging
import importlib
import warnings
from types import SimpleNamespace

root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "rpi"))
if root not in sys.path:
    sys.path.insert(0, root)

import pytest

from motion.trapezoidal_profile import TrapezoidalMotionProfile
from motion.spindle_kinematics import SpindleKinematics
from motion.winding_pattern import WindingPattern
from motion.scatter_engine import ScatterEngine
from motion.move import HomingMove, WoundMove
from motion.axis_state import AxisState
from motion.move_queue import MoveQueue
from motion.synchronized_segment_generator import SyncAxisConfig
from motion.engine import WindingEngine
from transport.streamer import MultiAxisRampStreamer
from winding.program import WindingProgram
from core.config import AppConfiguration
from core.events import EventBus
from core.shared_state import SharedState


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


def test_scatter_engine_rejects_zero_freq1():
    with pytest.raises(ValueError):
        ScatterEngine(freq1=0.0)


def test_scatter_engine_rejects_negative_amplitude():
    with pytest.raises(ValueError):
        ScatterEngine(amplitude_mm=-1.0)


def test_wound_run_two_pass_duration_keeps_positive_cruise_for_10s_case():
    config = AppConfiguration(
        spindle_steps_per_revolution=200,
        spindle_microstepping=32,
        spindle_max_acceleration_rpm=300.0,
        spindle_max_deceleration_rpm=300.0,
    )
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        event_bus=EventBus(),
        config=config,
    )

    captured: list[WoundMove] = []
    engine._move_queue.enqueue = lambda move: captured.append(move)  # type: ignore[assignment]

    # 10 s at cruise: total_turns = target_rps * 10 = 10 * 10 = 100 turns.
    engine.wound_run(
        spindle_axis_id=0,
        traverse_axis_id=1,
        target_rpm=600.0,
        accel_s=None,
        cruise_s=None,
        decel_s=None,
        bobbin_width_mm=10.0,
        turns_per_mm=5.0,
    )

    assert captured, "wound_run must enqueue one move"
    move = captured[0]
    assert move.kinematics.accel_s == pytest.approx(2.0, rel=1e-6)
    assert move.kinematics.decel_s == pytest.approx(2.0, rel=1e-6)
    assert move.kinematics.cruise_s > 0.0


def test_execute_homing_waits_for_endstop_request_confirmation(monkeypatch):
    class FakeTransport:
        def __init__(self) -> None:
            self._seq = 0
            self.wait_calls: list[int] = []

        def get_status(self):
            return SimpleNamespace(last_executed_sequence=0xFFFF)

        def enable_endstop_request(self, axis_id: int, arm: bool):
            self._seq += 1
            return self._seq, SimpleNamespace()

        def wait_for_request_result(self, sequence: int, poll_interval_s: float = 0.001):
            self.wait_calls.append(sequence)
            return SimpleNamespace(last_result=0)

    class FakeStreamer:
        endstop_triggered = True

        def __init__(self) -> None:
            self._generator = None
            self._generator_finished = False

        def stream_all(self) -> int:
            return 0

    axis_state = AxisState(axis_id=1)
    transport = FakeTransport()
    queue = MoveQueue(
        transport=transport,
        axis_states={1: axis_state},
        poll_interval_s=0.001,
        print_every=1,
    )
    monkeypatch.setattr(queue, "_make_streamer", lambda axis_configs: FakeStreamer())

    move = HomingMove(
        name="home_test",
        axis_id=1,
        steps_per_rev=6400,
        approach_rpm=100.0,
        search_rpm=20.0,
        backoff_steps=3200,
        max_approach_steps=6400,
    )

    queue._execute_homing(move)

    assert move.state.name == "COMPLETED"
    # approach arm + backoff disarm + search arm + final disarm
    assert len(transport.wait_calls) == 4


def test_execute_wound_move_invalidates_positions_on_stop_requested(monkeypatch):
    class FakeTransport:
        def get_status(self):
            return SimpleNamespace(last_executed_sequence=0xFFFF)

    class FakeStreamer:
        def __init__(self, queue: MoveQueue) -> None:
            self._generator = None
            self._generator_finished = False
            self.endstop_triggered = False
            self._queue = queue

        def stream_all(self) -> int:
            self._queue._stop_requested = True
            return 0

    spindle_state = AxisState(axis_id=0)
    traverse_state = AxisState(axis_id=1)
    spindle_state.mark_homed(100)
    traverse_state.mark_homed(200)

    queue = MoveQueue(
        transport=FakeTransport(),
        axis_states={0: spindle_state, 1: traverse_state},
        poll_interval_s=0.001,
        print_every=1,
    )
    monkeypatch.setattr(queue, "_make_wound_streamer", lambda move: FakeStreamer(queue))

    move = WoundMove(
        name="wound_stop",
        kinematics=SpindleKinematics(target_rpm=600.0, accel_s=0.2, cruise_s=0.2, decel_s=0.2),
        pattern=WindingPattern(bobbin_width_mm=10.0, turns_per_mm=5.0),
        scatter=ScatterEngine(amplitude_mm=0.0),
        spindle_cfg=SyncAxisConfig(axis_index=0, steps_per_unit=6400.0),
        traverse_cfg=SyncAxisConfig(axis_index=1, steps_per_unit=800.0),
    )

    queue._execute_wound_move(move)

    assert move.state.name == "ABORTED"
    assert spindle_state.position_steps is None
    assert traverse_state.position_steps is None


def test_wound_run_logs_warning_for_inconsistent_explicit_profile(caplog):
    config = AppConfiguration()
    engine = WindingEngine(
        transport=SimpleNamespace(),
        shared_state=SharedState(axis_states={}),
        event_bus=EventBus(),
        config=config,
    )
    engine._move_queue.enqueue = lambda move: None  # type: ignore[assignment]

    with caplog.at_level(logging.WARNING, logger="motion.engine"):
        engine.wound_run(
            spindle_axis_id=0,
            traverse_axis_id=1,
            target_rpm=600.0,
            accel_s=0.1,
            cruise_s=0.1,
            decel_s=0.1,
            bobbin_width_mm=10.0,
            turns_per_mm=5.0,
        )

    assert "wound_run: profile produces" in caplog.text


def test_from_axis_ids_respects_segment_duration_s_parameter():
    class FakeTransport:
        def get_status(self):
            return SimpleNamespace(last_executed_sequence=0xFFFF)

    streamer = MultiAxisRampStreamer.from_axis_ids(
        FakeTransport(),
        [0, 1],
        target_hz=1000.0,
        segment_duration_s=0.003,
    )
    assert streamer._segment_duration_s == pytest.approx(0.003)


def test_legacy_syncrhonized_module_emits_deprecation_warning():
    module_name = "motion.syncrhonized_segment_generator"
    sys.modules.pop(module_name, None)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", DeprecationWarning)
        importlib.import_module(module_name)
    assert any(
        isinstance(w.message, DeprecationWarning)
        and "deprecated" in str(w.message)
        for w in caught
    )
```

---

## 3. Sources complètes — fichiers non modifiés nécessaires à l’analyse

### src/rpi/motion/segment_generator.py
```python
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Iterator, Tuple

from transport.messages import MultiAxisSegment


@dataclass(slots=True)
class AxisStepProfile:
    axis_index: int
    step_at: Callable[[float], float]
    reverse_direction: bool = False
    total_duration: float = 0.0


class BaseSegmentGenerator(ABC):
    """Common iteration behavior for multi-axis segment generators."""

    def __init__(self, *, segment_duration_s: float = 0.004, start_sequence: int = 0) -> None:
        self.segment_duration_s = max(0.002, min(0.005, segment_duration_s))
        self._sequence = start_sequence & 0xFFFF
        self._time_cursor = 0.0
        self.overall_duration = 0.0

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        while self._time_cursor < self.overall_duration:
            next_cursor = min(self._time_cursor + self.segment_duration_s, self.overall_duration)
            duration_s = next_cursor - self._time_cursor
            duration_us = int(round(duration_s * 1_000_000))

            steps, directions = self._compute_segment(self._time_cursor, next_cursor)

            yield MultiAxisSegment(
                sequence=self._sequence,
                duration_us=duration_us,
                steps=steps,
                directions=directions,
            )
            self._sequence = (self._sequence + 1) & 0xFFFF
            self._time_cursor = next_cursor

    @abstractmethod
    def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
        """Return the next segment payload for the current time window."""
        ...


class StepProfileSegmentGenerator(BaseSegmentGenerator):
    def __init__(self, axis_profiles: list[AxisStepProfile], *, segment_duration_s: float = 0.004, start_sequence: int = 0) -> None:
        super().__init__(segment_duration_s=segment_duration_s, start_sequence=start_sequence)
        self.axis_profiles = axis_profiles
        self._axis_errors = [0.0 for _ in axis_profiles]
        self._current_steps = [profile.step_at(0.0) for profile in axis_profiles]
        self.overall_duration = max((profile.total_duration for profile in axis_profiles), default=0.0)

    def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
        max_axis = max((profile.axis_index for profile in self.axis_profiles), default=-1)
        steps = [0] * (max_axis + 1)
        directions = [0] * (max_axis + 1)

        for index, profile in enumerate(self.axis_profiles):
            target_steps = profile.step_at(time_end)
            delta_steps = target_steps - self._current_steps[index]
            count = int(round(self._axis_errors[index] + delta_steps))
            self._axis_errors[index] += delta_steps - float(count)
            self._current_steps[index] = target_steps

            is_negative = count < 0
            if is_negative:
                count = abs(count)
            directions[profile.axis_index] = 1 if is_negative ^ profile.reverse_direction else 0
            steps[profile.axis_index] = count

        return steps, directions
```

### src/rpi/transport/messages.py
```python
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
from typing import Iterable, List

SPI_MSG_MAGIC = 0x5057
SPI_MSG_VERSION = 1
SPI_FRAME_SIZE = 512
SPI_MAX_AXES = 4
STEP_BLOCK_SIZE = 64
SEGMENT_BLOCK_SIZE = 60
MULTI_AXIS_SEGMENT_BLOCK_SIZE = 60

_HEADER_STRUCT = struct.Struct("<HBBHHHH")
_ENABLE_STRUCT = struct.Struct("<BB2x")
_ESTOP_STRUCT = struct.Struct("<B3x")
_STEP_BLOCK_HEAD_STRUCT = struct.Struct("<BBH")
_STEP_ENTRY_STRUCT = struct.Struct("<IB")
_SEGMENT_BLOCK_HEAD_STRUCT = struct.Struct("<BBH")
_SEGMENT_ENTRY_STRUCT = struct.Struct("<HHhBB")
_MULTI_AXIS_SEGMENT_BLOCK_HEAD_STRUCT = struct.Struct("<HBB")
_MULTI_AXIS_SEGMENT_ENTRY_HEADER_STRUCT = struct.Struct("<HHH")
_STEP_COUNT_STRUCT = struct.Struct("<H")
_FLUSH_STRUCT = struct.Struct("<H2x")
_ENABLE_ENDSTOP_STRUCT = struct.Struct("<BB2x")
_STATUS_STRUCT = struct.Struct("<I4H4H4IHBBBBBBBHB")


class SpiMessageType(IntEnum):  # Must match SpiMessageType in messages.h
    NOP = 0x00
    ENABLE_AXIS = 0x01
    ESTOP = 0x02
    STOP_AXIS = 0x03
    DISABLE_ALL = 0x04
    RESET_STATS = 0x05
    GET_STATUS = 0x06
    STEP_BLOCK = 0x10
    SEGMENT_BLOCK = 0x11
    FLUSH = 0x12
    MULTI_AXIS_SEGMENT_BLOCK = 0x13
    ENABLE_ENDSTOP = 0x14
    PING = 0x7F
    STATUS = 0x80


class SpiMessageResult(IntEnum):
    OK = 0x00
    BAD_MAGIC = 0x01
    BAD_VERSION = 0x02
    BAD_LENGTH = 0x03
    BAD_CRC = 0x04
    UNKNOWN_TYPE = 0x05
    BAD_AXIS = 0x06
    QUEUE_FULL = 0x07
    INTERNAL_ERROR = 0x08
    ENDSTOP_BLOCKED = 0x09


class SpiStepFlags(IntEnum):
    NONE = 0x00
    DIR_REVERSE = 0x01


LATERAL_ENDSTOP_PRESENT_OPEN = 0x00
LATERAL_ENDSTOP_PRESENT_CLOSED = 0x01
LATERAL_ENDSTOP_ABSENT = 0xFF


@dataclass(slots=True)
class MessageHeader:
    magic: int
    version: int
    msg_type: int
    sequence: int
    payload_length: int
    flags: int
    crc16: int = 0

    def pack(self) -> bytes:
        return _HEADER_STRUCT.pack(
            self.magic,
            self.version,
            self.msg_type,
            self.sequence,
            self.payload_length,
            self.flags,
            self.crc16,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "MessageHeader":
        return cls(*_HEADER_STRUCT.unpack(data[: _HEADER_STRUCT.size]))


@dataclass(slots=True)
class EnableAxisPayload:
    axis_id: int
    enable: bool

    def pack(self) -> bytes:
        return _ENABLE_STRUCT.pack(self.axis_id, 1 if self.enable else 0)


@dataclass(slots=True)
class EmergencyStopPayload:
    axis_id: int = 0xFF

    def pack(self) -> bytes:
        return _ESTOP_STRUCT.pack(self.axis_id)


@dataclass(slots=True)
class StepEntry:
    interval_ticks: int
    direction_reverse: bool = False

    def pack(self) -> bytes:
        flags = int(SpiStepFlags.DIR_REVERSE) if self.direction_reverse else int(SpiStepFlags.NONE)
        return _STEP_ENTRY_STRUCT.pack(self.interval_ticks, flags)


@dataclass(slots=True)
class MotionSegment:
    step_count: int
    start_ticks: int
    add_ticks: int
    direction_reverse: bool = False

    def pack(self) -> bytes:
        flags = int(SpiStepFlags.DIR_REVERSE) if self.direction_reverse else int(SpiStepFlags.NONE)
        return _SEGMENT_ENTRY_STRUCT.pack(self.step_count, self.start_ticks, self.add_ticks, flags, 0)


@dataclass(slots=True)
class StepBlockPayload:
    axis_id: int
    block_seq: int
    entries: List[StepEntry]

    def pack(self) -> bytes:
        if len(self.entries) > STEP_BLOCK_SIZE:
            raise ValueError(f"step block too large: {len(self.entries)} > {STEP_BLOCK_SIZE}")
        payload = bytearray()
        payload += _STEP_BLOCK_HEAD_STRUCT.pack(self.axis_id, self.block_seq, len(self.entries))
        for entry in self.entries:
            payload += entry.pack()
        for _ in range(STEP_BLOCK_SIZE - len(self.entries)):
            payload += _STEP_ENTRY_STRUCT.pack(0, 0)
        return bytes(payload)


@dataclass(slots=True)
class SegmentBlockPayload:
    axis_id: int
    block_seq: int
    segments: List[MotionSegment]

    def pack(self) -> bytes:
        if len(self.segments) > SEGMENT_BLOCK_SIZE:
            raise ValueError(f"segment block too large: {len(self.segments)} > {SEGMENT_BLOCK_SIZE}")
        payload = bytearray()
        payload += _SEGMENT_BLOCK_HEAD_STRUCT.pack(self.axis_id, self.block_seq, len(self.segments))
        for segment in self.segments:
            payload += segment.pack()
        for _ in range(SEGMENT_BLOCK_SIZE - len(self.segments)):
            payload += _SEGMENT_ENTRY_STRUCT.pack(0, 0, 0, 0, 0)
        return bytes(payload)


@dataclass(slots=True)
class MultiAxisSegment:
    sequence: int
    duration_us: int
    steps: List[int]
    directions: List[int]


@dataclass(slots=True)
class MultiAxisSegmentBlockPayload:
    axis_ids: List[int]
    block_seq: int
    segments: List[MultiAxisSegment]

    def pack(self) -> bytes:
        if len(self.axis_ids) == 0:
            raise ValueError("multi-axis segment block requires at least one axis")
        if len(self.segments) > MULTI_AXIS_SEGMENT_BLOCK_SIZE:
            raise ValueError(
                f"multi-axis segment block too large: {len(self.segments)} > {MULTI_AXIS_SEGMENT_BLOCK_SIZE}"
            )

        axis_count = len(self.axis_ids)
        payload = bytearray()
        payload += _MULTI_AXIS_SEGMENT_BLOCK_HEAD_STRUCT.pack(self.block_seq, len(self.segments), axis_count)
        payload += bytes(self.axis_ids)
        for segment in self.segments:
            if len(segment.steps) != axis_count:
                raise ValueError(
                    f"segment step count {len(segment.steps)} does not match axis count {axis_count}"
                )
            if len(segment.directions) != axis_count:
                raise ValueError(
                    f"segment direction count {len(segment.directions)} does not match axis count {axis_count}"
                )

            direction_mask = 0
            for axis_index, direction in enumerate(segment.directions):
                if direction:
                    direction_mask |= 1 << axis_index

            payload += _MULTI_AXIS_SEGMENT_ENTRY_HEADER_STRUCT.pack(
                segment.sequence,
                segment.duration_us,
                direction_mask,
            )
            for step in segment.steps:
                payload += _STEP_COUNT_STRUCT.pack(step)
        return bytes(payload)


@dataclass(slots=True)
class FlushPayload:
    flush_sequence: int

    def pack(self) -> bytes:
        return _FLUSH_STRUCT.pack(self.flush_sequence)


@dataclass(slots=True)
class EnableEndstopPayload:
    axis_id: int
    arm: bool  # True = arm, False = disarm

    def pack(self) -> bytes:
        return _ENABLE_ENDSTOP_STRUCT.pack(self.axis_id, int(self.arm))


@dataclass(slots=True)
class StatusPayload:
    uptime_ms: int
    queue_free_slots: tuple[int, int, int, int]
    ring_free_slots: tuple[int, int, int, int]
    underrun_count: tuple[int, int, int, int]
    last_rx_sequence: int
    last_rx_type: int
    last_result: int
    protocol_version: int
    enabled_mask: int
    running_mask: int
    lateral_endstop_state: int
    endstop_armed_mask: int
    last_executed_sequence: int
    planner_queue_free: int

    @classmethod
    def unpack(cls, payload: bytes) -> "StatusPayload":
        values = _STATUS_STRUCT.unpack(payload[: _STATUS_STRUCT.size])
        return cls(
            uptime_ms=values[0],
            queue_free_slots=(values[1], values[2], values[3], values[4]),
            ring_free_slots=(values[5], values[6], values[7], values[8]),
            underrun_count=(values[9], values[10], values[11], values[12]),
            last_rx_sequence=values[13],
            last_rx_type=values[14],
            last_result=values[15],
            protocol_version=values[16],
            enabled_mask=values[17],
            running_mask=values[18],
            lateral_endstop_state=values[19],
            endstop_armed_mask=values[20],
            last_executed_sequence=values[21],
            planner_queue_free=values[22],
        )


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: SpiMessageType, payload: bytes = b"", *, sequence: int = 0, flags: int = 0) -> bytes:
    if len(payload) > SPI_FRAME_SIZE - _HEADER_STRUCT.size:
        raise ValueError("payload too large for one SPI frame")
    header = MessageHeader(
        magic=SPI_MSG_MAGIC,
        version=SPI_MSG_VERSION,
        msg_type=int(msg_type),
        sequence=sequence & 0xFFFF,
        payload_length=len(payload),
        flags=flags & 0xFFFF,
        crc16=0,
    )
    frame = bytearray(SPI_FRAME_SIZE)
    frame[: _HEADER_STRUCT.size] = header.pack()
    frame[_HEADER_STRUCT.size : _HEADER_STRUCT.size + len(payload)] = payload
    header.crc16 = crc16_ccitt(frame[: _HEADER_STRUCT.size + len(payload)])
    frame[: _HEADER_STRUCT.size] = header.pack()
    return bytes(frame)


def parse_status_frame(frame: bytes) -> StatusPayload:
    if len(frame) != SPI_FRAME_SIZE:
        raise ValueError(f"invalid SPI frame size: {len(frame)}")
    header = MessageHeader.unpack(frame)
    if header.magic != SPI_MSG_MAGIC:
        raise ValueError(f"bad magic: 0x{header.magic:04X}")
    if header.version != SPI_MSG_VERSION:
        raise ValueError(f"bad version: {header.version}")
    if header.msg_type != int(SpiMessageType.STATUS):
        raise ValueError(f"unexpected response type: 0x{header.msg_type:02X}")
    if header.payload_length > SPI_FRAME_SIZE - _HEADER_STRUCT.size:
        raise ValueError("invalid payload length in response")

    raw = bytearray(frame[: _HEADER_STRUCT.size + header.payload_length])
    raw[10:12] = b"\x00\x00"
    expected_crc = crc16_ccitt(bytes(raw))
    if expected_crc != header.crc16:
        raise ValueError(f"bad response CRC: expected 0x{expected_crc:04X}, got 0x{header.crc16:04X}")

    payload = frame[_HEADER_STRUCT.size : _HEADER_STRUCT.size + header.payload_length]
    return StatusPayload.unpack(payload)


def sequence_signed_distance(a: int, b: int) -> int:
    """Return the signed 16-bit distance from b to a.

    This is useful for comparing motion_sequence values that wrap at 0xFFFF.
    """
    return ((a - b + 0x8000) & 0xFFFF) - 0x8000


def sequence_is_greater(a: int, b: int) -> bool:
    return sequence_signed_distance(a, b) > 0


def sequence_is_greater_equal(a: int, b: int) -> bool:
    return sequence_signed_distance(a, b) >= 0


def sequence_is_less_equal(a: int, b: int) -> bool:
    return sequence_signed_distance(a, b) <= 0


def increment_sequence(sequence: int, increment: int = 1) -> int:
    return (sequence + increment) & 0xFFFF


def make_nop(sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.NOP, b"", sequence=sequence)


def make_get_status(sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.GET_STATUS, b"", sequence=sequence)


def make_enable_axis(axis_id: int, enable: bool, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.ENABLE_AXIS, EnableAxisPayload(axis_id, enable).pack(), sequence=sequence)


def make_estop(axis_id: int = 0xFF, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.ESTOP, EmergencyStopPayload(axis_id).pack(), sequence=sequence)


def make_stop_axis(axis_id: int = 0xFF, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.STOP_AXIS, EmergencyStopPayload(axis_id).pack(), sequence=sequence)


def make_disable_all(sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.DISABLE_ALL, b"", sequence=sequence)


def make_reset_stats(sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.RESET_STATS, b"", sequence=sequence)


def make_step_block(payload: StepBlockPayload, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.STEP_BLOCK, payload.pack(), sequence=sequence)


def make_segment_block(payload: SegmentBlockPayload, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.SEGMENT_BLOCK, payload.pack(), sequence=sequence)


def make_multi_axis_segment_block(payload: MultiAxisSegmentBlockPayload, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.MULTI_AXIS_SEGMENT_BLOCK, payload.pack(), sequence=sequence)


def make_flush(payload: FlushPayload, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.FLUSH, payload.pack(), sequence=sequence)


def make_enable_endstop(payload: EnableEndstopPayload, sequence: int = 0) -> bytes:
    return build_frame(SpiMessageType.ENABLE_ENDSTOP, payload.pack(), sequence=sequence)
```

### src/rpi/jsonrpc/protocol.py
```python
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Mapping

JSONRPC_VERSION = "2.0"


class JsonRpcError(Exception):
    def __init__(self, code: int, message: str, data: Any | None = None) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.data = data


class JsonRpcParseError(JsonRpcError):
    def __init__(self, data: Any | None = None) -> None:
        super().__init__(-32700, "Parse error", data)


class JsonRpcInvalidRequestError(JsonRpcError):
    def __init__(self, data: Any | None = None) -> None:
        super().__init__(-32600, "Invalid Request", data)


class JsonRpcMethodNotFoundError(JsonRpcError):
    def __init__(self, method: str) -> None:
        super().__init__(-32601, f"Method not found: {method}", {"method": method})


@dataclass
class JsonRpcRequest:
    method: str
    params: Any | None
    id: Any | None


@dataclass
class JsonRpcResponse:
    result: Any | None = None
    error: dict[str, Any] | None = None
    id: Any | None = None


def parse_json_rpc(payload: str) -> JsonRpcRequest:
    try:
        message = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise JsonRpcParseError(str(exc)) from exc

    if not isinstance(message, dict):
        raise JsonRpcInvalidRequestError(message)

    if message.get("jsonrpc") != JSONRPC_VERSION:
        raise JsonRpcInvalidRequestError(message)

    if "method" not in message or not isinstance(message["method"], str):
        raise JsonRpcInvalidRequestError(message)

    return JsonRpcRequest(
        method=message["method"],
        params=message.get("params"),
        id=message.get("id"),
    )


def make_response(result: Any, request_id: Any | None) -> str:
    return json.dumps({"jsonrpc": JSONRPC_VERSION, "result": result, "id": request_id})


def make_error_response(error: JsonRpcError, request_id: Any | None) -> str:
    payload = {
        "jsonrpc": JSONRPC_VERSION,
        "error": {
            "code": error.code,
            "message": error.message,
        },
        "id": request_id,
    }
    if error.data is not None:
        payload["error"]["data"] = error.data
    return json.dumps(payload)
```

### src/rpi/jsonrpc/__init__.py
```python
"""Unix socket JSON-RPC support for the PickupWinder host application."""

from .client import UnixJsonRpcClient
from .handlers import AppRpcHandler, RpcHandler
from .protocol import (
    JSONRPC_VERSION,
    JsonRpcError,
    JsonRpcInvalidRequestError,
    JsonRpcParseError,
    JsonRpcMethodNotFoundError,
    JsonRpcResponse,
    JsonRpcRequest,
    make_error_response,
    make_response,
    parse_json_rpc,
)
from .rpc_server import JsonRpcServer

__all__ = [
    "JsonRpcServer",
    "UnixJsonRpcClient",
    "RpcHandler",
    "AppRpcHandler",
    "JSONRPC_VERSION",
    "JsonRpcError",
    "JsonRpcInvalidRequestError",
    "JsonRpcParseError",
    "JsonRpcMethodNotFoundError",
    "JsonRpcResponse",
    "JsonRpcRequest",
    "make_error_response",
    "make_response",
    "parse_json_rpc",
]
```

---

## 4. Points d’attention restants

| Fichier | Description précise | Impact | Recommandation |
|---|---|---|---|
| src/rpi/motion/ramp.py | Le shim émet désormais un `DeprecationWarning` à l’import. En environnement de test avec filtres stricts, cela peut devenir bloquant. | Bruit dans les tests / CI, potentiels échecs si warnings traités en erreurs. | Migrer progressivement tous les imports vers `motion.multi_axis_segment_generator`, puis supprimer le shim à horizon V8/V9. |
| src/rpi/motion/syncrhonized_segment_generator.py | Le shim legacy est conservé (nom fautif) pour compatibilité. | Dette technique temporaire; maintenance de deux points d’entrée. | Ajouter une date cible de suppression + ticket de migration globale. |
| src/rpi/motion/__init__.py | `AxisMotionConfig` / `MultiAxisSegmentGenerator` transitent encore via `motion.ramp` (donc warning indirect). | Les consommateurs de `from motion import ...` peuvent déclencher la dépréciation du shim. | Faire pointer `__getattr__` directement vers `multi_axis_segment_generator` lorsque la migration sera prête. |
| src/rpi/motion/move_queue.py | `streamer._generator` et `streamer._generator_finished` restent mutés depuis l’extérieur (attributs privés). | Couplage fort entre `MoveQueue` et `MultiAxisRampStreamer`; fragilité lors de refactors. | Introduire une API explicite côté streamer (`set_generator(...)`) et retirer l’accès aux attributs privés. |
| src/rpi/jsonrpc/handlers.py | `RpcHandler.dispatch` garde une stratégie de fallback permissive (`callback(params)` après échec `**params`). | Tolérance utile, mais ambiguïtés possibles si signatures divergentes. | Conserver pour compatibilité JSON-RPC, mais documenter officiellement les signatures de handlers et valider par tests dédiés. |
| src/rpi/winding_main.py | `AppRpcHandler(app=engine)` expose la config de l’engine (via `_config`). | Résout `winder.config`, mais renforce l’accès à un attribut protégé (`_config`). | À moyen terme, exposer `engine.config` en propriété publique readonly. |
| tests/test_motion_v3.py | Le test de warning legacy manipule `sys.modules` pour forcer le réimport. | Peut interférer si parallélisation agressive des tests est activée. | Isoler ce test dans un module dédié de compatibilité ou utiliser un import sandbox si la suite grossit. |
| src/rpi/transport/streamer.py | Plusieurs logs critiques sont encore faits via `print(...)` (et non logger structuré). | Visibilité limitée en production / pas de niveaux configurables. | Migrer vers `logging` module-level avec niveaux (`info`, `warning`) et contextes clés (seq, ring, queue). |

---

## 5. Schéma de dépendances mis à jour (mermaid)

```mermaid
flowchart TD
    WM[winding_main.py] -->|AppRpcHandler(app=engine)| AR[AppRpcHandler]
    WM --> WE[WindingEngine]
    AR --> RH[RpcHandler.dispatch]

    RH -->|winder.ping/status/shutdown/config| WR[_rpc_* wrappers params optionnels]
    WR --> PUB[API publique ping/status/shutdown/config]

    WE --> MQ[MoveQueue]
    MQ --> WOUND[_execute_wound_move]
    WOUND --> INV[Invalidate positions]
    INV --> DONE[move.mark_completed]

    MQ --> STR[MultiAxisRampStreamer]
    STR --> SPI[Esp32SpiTransport]

    MOTION_INIT[motion/__init__.py] --> NEWMOD[motion/synchronized_segment_generator.py]
    LEGACY[motion/syncrhonized_segment_generator.py] -->|DeprecationWarning| NEWMOD

    RAMP_SHIM[motion/ramp.py] -->|DeprecationWarning| MAG[motion/multi_axis_segment_generator.py]

    TESTS[tests/test_motion_v3.py] --> NEWMOD
    TESTS --> LEGACY
```

---

Validation V7 exécutée:
- `tests/test_motion_v3.py` : **19 passed**
- Warnings observés: **1** (`motion.ramp` déprécié, attendu avec l’option A)
