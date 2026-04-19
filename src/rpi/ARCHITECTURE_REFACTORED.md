# Architecture Refactorisée — PickupWinder Host (Python)

> Généré le : 2026-04-19  
> Base de code : `src/rpi/`  
> Python requis : 3.11+

---

## 1. Vue d'ensemble des changements

| Fichier | Problème corrigé | Impact |
|---|---|---|
| `motion/ramp_config.py` | `_compute_ramp_times` avec `min/max` inversés rendant le plafond de vitesse inutile. Logique dupliquée dans `engine.py` et `core/app.py`. | Calcul de `accel_s`/`decel_s` correct et testable de façon unitaire. Fin de la duplication. |
| `motion/move.py` | `HomingMove` héritait de `Move` et levait `NotImplementedError` dans `segments()` — violation du LSP. Accès `move._error` exposé dans `engine.py`. | Hiérarchie propre : `BaseMove` → `Move` (segmenté) / `CompositeMove` (multi-phase). `error` devient une propriété publique. |
| `motion/move_queue.py` | Types codés en dur sur `Move` ; dispatch `isinstance(HomingMove)` trop couplé. | Utilise `BaseMove` et `CompositeMove` — découplage de la hiérarchie. |
| `motion/engine.py` | `_compute_ramp_times` dupliqué en instance ; `jog()` hardcodait `200 * 32` ; accès `move._error` ; `_wait_for_move_queue` sans timeout. | Méthode privée supprimée, `jog()` lit la config, interface publique respectée, timeout de 60 s par défaut. |
| `jsonrpc/winding_handler.py` | *(nouveau)* Les 10 méthodes RPC étaient des closures anonymes dans `winding_main.py`, non testables. | Classe dédiée `WindingRpcHandler` avec méthodes typées, enregistrement via `register_all()`. |
| `winding_main.py` | Fonction `_register_winding_rpc_methods` de 60 lignes de closures. | Réduit à 1 ligne : `WindingRpcHandler(engine, state).register_all(rpc_handler)`. |

---

## 2. Nouveaux modules et classes

### 2.1 `motion/ramp_config.py` — `compute_ramp_times()`

**Rôle :** Fonction pure calculant `(accel_s, cruise_s, decel_s)` à partir des limites machine.

**Interface publique :**

```python
def compute_ramp_times(
    target_rpm: float,
    duration_s: float,
    max_accel_steps_per_s2: float,
    max_decel_steps_per_s2: float,
    steps_per_rev: int,
    start_rpm: float = 0.0,
    min_ramp_s: float = 0.05,
    max_ramp_fraction: float = 0.25,
) -> tuple[float, float, float]:  # (accel_s, cruise_s, decel_s)
```

**Logique (corrigée) :**
```
physics_accel_s = Δhz / max_accel                 # temps physiquement requis
accel_s = clamp(physics_accel_s, min_ramp_s, duration_s * max_ramp_fraction)
decel_s = clamp(physics_decel_s, min_ramp_s, duration_s * max_ramp_fraction)
cruise_s = max(duration_s - accel_s - decel_s, 0)
```

**Correction de l'ancien bug :**

L'ancien code faisait :
```python
# ❌ BUG : min() plafonne à 0.5 s, puis max() force à dépasser ce plafond
accel_s = min(0.5, duration_s * 0.25)              # borne haute
if max_accel > 0:
    accel_s = max(accel_s, max(0.05, delta / max_accel))  # peut dépasser 0.5 s !
```

La borne haute `min(0.5, ...)` était donc complètement inefficace, puisque le
`max()` suivant la remontait systématiquement à la valeur physique sans la
contraindre. Pour `spindle_max_acceleration_rpm = 10 RPM/s` (très lent),
`delta_hz = 1500/60 * 6400 ≈ 160 000 Hz/s → accel_s ≈ 26 s` — la borne de
`0.5 s` était ignorée.

Le nouveau code implémente correctement un `clamp` : la valeur physique est
calculée, puis bornée entre `min_ramp_s` et `duration_s * max_ramp_fraction`.

---

### 2.2 `motion/move.py` — Hiérarchie `BaseMove` / `CompositeMove`

**Rôle :** Corriger la violation LSP de `HomingMove` et exposer `error` publiquement.

**Hiérarchie :**

```
BaseMove (ABC)
├── state, done, error, aborted_by_endstop    ← lecture seule
├── mark_running / mark_completed / mark_aborted / mark_failed
├── snapshot()
└── expected_delta_steps()  [abstract]
    │
    ├── Move(BaseMove, ABC)
    │   └── segments()  [abstract]  ← stream de MultiAxisSegment
    │       ├── RampMove
    │       ├── JogMove
    │       └── WoundMove
    │
    └── CompositeMove(BaseMove, ABC)
        └── phases()  [abstract]  ← list[(name, Move, armed: bool)]
            └── HomingMove
```

**Interface publique de `BaseMove` :**

```python
class BaseMove(ABC):
    name: str

    @property
    def state(self) -> MoveState: ...
    @property
    def done(self) -> bool: ...
    @property
    def error(self) -> str | None: ...          # ← NOUVEAU, remplace _error
    @property
    def aborted_by_endstop(self) -> bool: ...

    def mark_running(self) -> None: ...
    def mark_completed(self) -> None: ...
    def mark_aborted(self, reason: str, by_endstop: bool = False) -> None: ...
    def mark_failed(self, error: str) -> None: ...
    def snapshot(self) -> dict[str, Any]: ...

    @abstractmethod
    def expected_delta_steps(self, axis_id: int) -> int | None: ...
```

**Interface publique de `CompositeMove` :**

```python
class CompositeMove(BaseMove, ABC):
    @abstractmethod
    def phases(self) -> list[tuple[str, Move, bool]]:
        """(phase_name, sub_move, endstop_armed)"""
        ...
```

**Pourquoi `BaseMMove` plutôt que modifier `Move` directement :**  
`HomingMove` *ne peut pas* être une `Move` au sens LSP : une `Move` promet une
interface `segments()` utilisable par n'importe quel streamer. `HomingMove` ne
peut être exécutée que par `MoveQueue._execute_homing()`, qui orchestre les
phases avec arm/disarm. En introduisant `CompositeMove`, le type system empêche
qu'un `HomingMove` soit passé accidentellement à un streamer de segments.

---

### 2.3 `jsonrpc/winding_handler.py` — `WindingRpcHandler`

**Rôle :** Regrouper les 10 méthodes `winding.*` dans une classe testable, en
remplacement des closures anonymes de `winding_main.py`.

**Interface publique complète :**

```python
class WindingRpcHandler:
    def __init__(self, engine: WindingEngine, shared_state: SharedState) -> None: ...
    def register_all(self, handler: AppRpcHandler) -> None: ...

    # Méthodes RPC (mêmes noms/contrats JSON qu'avant)
    def submit_program(self, program: dict) -> dict[str, Any]: ...
    def stop(self, _params: Any | None = None) -> dict[str, str]: ...
    def jog(self, axis_id: int, steps: int, rpm: float, reverse: bool = False) -> dict[str, Any]: ...
    def wound_run(
        self,
        spindle_axis_id: int,
        traverse_axis_id: int,
        target_rpm: float,
        bobbin_width_mm: float,
        turns_per_mm: float,
        accel_s: float = 2.0,
        cruise_s: float = 10.0,
        decel_s: float = 2.0,
        scatter_amplitude_mm: float = 0.0,
        scatter_damping_margin_mm: float = 0.0,
        spindle_reverse: bool = False,
        traverse_reverse: bool = False,
    ) -> dict[str, Any]: ...
    def run_axis(self, duration_s: float, targets: list[dict[str, Any]]) -> dict[str, Any]: ...
    def clear_fault(self, _params: Any | None = None) -> dict[str, str]: ...
    def status(self, _params: Any | None = None) -> dict[str, Any]: ...
    def axis_state(self, axis_id: int) -> dict[str, Any]: ...
    def arm_endstop(self, axis_id: int) -> dict[str, Any]: ...
    def disarm_endstop(self, axis_id: int) -> dict[str, Any]: ...
```

**Code source complet :** voir [jsonrpc/winding_handler.py](jsonrpc/winding_handler.py)

---

## 3. Code source critique — Segment Generators

### 3.1 `motion/__init__.py`

```python
"""Motion planning and control for PickupWinder.

Lazy imports to avoid circular dependencies with transport module.
"""

__all__ = [
    "RampConfig",
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
        from .syncrhonized_segment_generator import SyncAxisConfig
        return SyncAxisConfig
    if name == "SynchronizedSegmentGenerator":
        from .syncrhonized_segment_generator import SynchronizedSegmentGenerator
        return SynchronizedSegmentGenerator
    if name == "WindingEngine":
        from .engine import WindingEngine
        return WindingEngine
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return __all__
```

> **Note :** `compute_ramp_times` est intentionnellement absent de `__init__.py`.
> Elle est importée directement là où elle est nécessaire :
> `from motion.ramp_config import compute_ramp_times`.
> L'exposer dans `__init__` n'apporterait rien et alourdirait l'API publique du package.

---

### 3.2 `motion/ramp_config.py` — version refactorisée avec `compute_ramp_times`

```python
from dataclasses import dataclass
from .trapezoidal_profile import TrapezoidalMotionProfile


def compute_ramp_times(
    target_rpm: float,
    duration_s: float,
    max_accel_steps_per_s2: float,
    max_decel_steps_per_s2: float,
    steps_per_rev: int,
    start_rpm: float = 0.0,
    min_ramp_s: float = 0.05,
    max_ramp_fraction: float = 0.25,
) -> tuple[float, float, float]:
    """Compute trapezoidal ramp times from machine acceleration limits.

    Implements:  ramp_s = clamp(physics_required, min_ramp_s, duration_s * max_ramp_fraction)
    """
    start_hz: float = start_rpm / 60.0 * float(steps_per_rev)
    target_hz: float = target_rpm / 60.0 * float(steps_per_rev)
    delta_hz: float = max(target_hz - start_hz, 0.0)

    if max_accel_steps_per_s2 > 0.0:
        physics_accel_s = delta_hz / max_accel_steps_per_s2
    else:
        physics_accel_s = min_ramp_s

    if max_decel_steps_per_s2 > 0.0:
        physics_decel_s = delta_hz / max_decel_steps_per_s2
    else:
        physics_decel_s = min_ramp_s

    cap = duration_s * max_ramp_fraction
    accel_s = max(min_ramp_s, min(physics_accel_s, cap))
    decel_s = max(min_ramp_s, min(physics_decel_s, cap))
    cruise_s = max(duration_s - accel_s - decel_s, 0.0)
    return accel_s, cruise_s, decel_s


@dataclass(slots=True)
class RampConfig:
    axis_id: int = 0
    steps_per_rev: int = 200 * 32
    start_rpm: float = 0.0
    target_rpm: float = 1000.0
    accel_s: float = 10.0
    cruise_s: float = 3.0
    decel_s: float = 10.0
    resolution_hz: int = 80_000_000
    reverse_direction: bool = False
    phase_segments: int = 8
    segment_duration_s: float = 0.05

    @property
    def start_hz(self) -> float:
        return self.start_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def target_hz(self) -> float:
        return self.target_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def profile(self) -> TrapezoidalMotionProfile:
        return TrapezoidalMotionProfile(
            start_rpm=self.start_rpm,
            target_rpm=self.target_rpm,
            accel_s=self.accel_s,
            cruise_s=self.cruise_s,
            decel_s=self.decel_s,
        )

    @property
    def total_duration(self) -> float:
        return self.profile.total_duration

    def hz_at_time(self, t: float) -> float:
        return self.profile.rps_at(t) * float(self.steps_per_rev)

    def steps_at(self, t: float) -> float:
        return self.profile.steps_at(t, self.steps_per_rev)

    def step_delta(self, time_start: float, time_end: float) -> float:
        return self.profile.step_delta(time_start, time_end, self.steps_per_rev)
```

---

### 3.3 `transport/streamer.py` — `MultiAxisRampStreamer` (complet)

Les méthodes ci-dessous sont citées dans leur intégralité :

```python
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
```

```python
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
```

```python
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
```

```python
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
```

```python
    def stream_all(self) -> int:
        self._generator_finished = False
        status = self._transport.get_status()
        self._enable_axes()
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

        self._write_send_log()
        return total_segments
```

---

### 3.4 `motion/multi_axis_segment_generator.py` + `motion/syncrhonized_segment_generator.py`

Le comportement d'itération vient principalement de `motion/segment_generator.py`.
Voici le code complet de la discrétisation temporelle et de la génération de
segments.

```python
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

```python
class MultiAxisSegmentGenerator(StepProfileSegmentGenerator):
    def __init__(
        self,
        axis_configs: list[AxisMotionConfig],
        *,
        segment_duration_s: float = 0.004,
        start_sequence: int = 0,
    ):
        self.axis_configs = axis_configs

        axis_profiles = [
            AxisStepProfile(
                axis_index=config.axis_id,
                step_at=lambda t, ramp=config.ramp: ramp.steps_at(t),
                reverse_direction=config.ramp.reverse_direction,
                total_duration=config.ramp.total_duration,
            )
            for config in axis_configs
        ]

        super().__init__(axis_profiles, segment_duration_s=segment_duration_s, start_sequence=start_sequence)
```

```python
class SynchronizedSegmentGenerator(StepProfileSegmentGenerator):
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

---

### 3.5 `motion/trapezoidal_profile.py`

```python
class TrapezoidalMotionProfile:
    def __init__(
        self,
        start_rpm: float = 0.0,
        target_rpm: float = 1000.0,
        accel_s: float = 0.0,
        cruise_s: float = 0.0,
        decel_s: float = 0.0,
    ) -> None:
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
            return turns + target_rps * t

        turns += target_rps * self.cruise_s
        t -= self.cruise_s
        if self.decel_s <= 0.0:
            return turns + target_rps * t

        rate = (target_rps - start_rps) / self.decel_s
        return turns + target_rps * t - 0.5 * rate * t * t

    def steps_at(self, t: float, steps_per_rev: int) -> float:
        return self.turns_at(t) * float(steps_per_rev)

    def step_delta(self, time_start: float, time_end: float, steps_per_rev: int) -> float:
        return self.steps_at(time_end, steps_per_rev) - self.steps_at(time_start, steps_per_rev)
```

---

### 3.6 `motion/spindle_kinematics.py` + `motion/winding_pattern.py`

```python
@dataclass(slots=True)
class SpindleKinematics(TrapezoidalMotionProfile):
    """Calculates absolute angular position (in turns) of the Spindle at time t."""
    target_rpm: float
    start_rpm: float = 0.0
    accel_s: float = 2.0
    cruise_s: float = 10.0
    decel_s: float = 2.0
```

```python
@dataclass(slots=True)
class WindingPattern:
    """Converts Spindle position (turns) to Guide position (mm) in a triangular wave."""
    bobbin_width_mm: float
    turns_per_mm: float

    def guide_pos_mm(self, spindle_turns: float) -> float:
        if self.turns_per_mm <= 0 or self.bobbin_width_mm <= 0:
            return 0.0
            
        total_dist = spindle_turns / self.turns_per_mm
        cycle_length = 2.0 * self.bobbin_width_mm
            
        mod_dist = total_dist % cycle_length
        if mod_dist <= self.bobbin_width_mm:
            return mod_dist
        else:
            return cycle_length - mod_dist
```

---

### 3.7 `jsonrpc/winding_handler.py`

```python
from __future__ import annotations

from typing import Any

from core.shared_state import SharedState
from jsonrpc.handlers import AppRpcHandler
from jsonrpc.protocol import JsonRpcError
from motion.engine import WindingEngine
from winding.program import WindingProgram


class WindingRpcHandler:
    """Exposes every ``winding.*`` RPC method as a typed public method."""

    def __init__(self, engine: WindingEngine, shared_state: SharedState) -> None:
        self._engine = engine
        self._state = shared_state

    def register_all(self, handler: AppRpcHandler) -> None:
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

    def submit_program(self, program: dict) -> dict[str, Any]:
        if not isinstance(program, dict):
            raise JsonRpcError(-32602, "Invalid params: expected program object")
        p = WindingProgram(**program)
        self._engine.submit_program(p)
        return {"status": "queued", "program": p.snapshot()}

    def stop(self, _params: Any | None = None) -> dict[str, str]:
        self._engine.request_stop()
        return {"status": "stopping"}

    def jog(
        self,
        axis_id: int,
        steps: int,
        rpm: float,
        reverse: bool = False,
    ) -> dict[str, Any]:
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
        spindle_reverse: bool = False,
        traverse_reverse: bool = False,
    ) -> dict[str, Any]:
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
            spindle_reverse=spindle_reverse,
            traverse_reverse=traverse_reverse,
        )
        return {"status": "queued"}

    def run_axis(
        self,
        duration_s: float,
        targets: list[dict[str, Any]],
    ) -> dict[str, Any]:
        self._engine.run_axis(duration_s=duration_s, targets=targets)
        return {"status": "queued"}

    def clear_fault(self, _params: Any | None = None) -> dict[str, str]:
        self._engine.clear_fault()
        return {"status": "ok"}

    def status(self, _params: Any | None = None) -> dict[str, Any]:
        return self._engine.status()

    def axis_state(self, axis_id: int) -> dict[str, Any]:
        state = self._state.axis_states.get(axis_id)
        if state is None:
            raise JsonRpcError(-32602, f"Unknown axis_id: {axis_id}")
        return state.snapshot()

    def arm_endstop(self, axis_id: int) -> dict[str, Any]:
        self._engine.arm_endstop(axis_id)
        return {"status": "armed", "axis_id": axis_id}

    def disarm_endstop(self, axis_id: int) -> dict[str, Any]:
        self._engine.disarm_endstop(axis_id)
        return {"status": "disarmed", "axis_id": axis_id}
```

---

## 4. Points d'attention restants

### 4.1 `wound_run` avec calcul des accélérations par défaut

`WindingEngine.wound_run()` accepte désormais des valeurs `accel_s`, `cruise_s`
et `decel_s` optionnelles (`None`). Lorsque le client omet ces paramètres,
la méthode calcule des durées de rampe par défaut via
`compute_ramp_times()` en utilisant la géométrie de bobine fournie.

**Amélioration :** l'ancienne API ne reposait plus sur des constantes
hardcodées pour `accel_s`/`decel_s`, ce qui permet d'utiliser les limites de
configuration si le client ne les fournit pas.

### 4.2 `HomingMove` avec `steps_per_rev = 200 * 32` hardcodé

`engine._home_lateral()` ne passe plus une constante fixe.
Il lit maintenant `steps_per_rev` depuis `AppConfiguration` :
`config.lateral_steps_per_revolution * config.lateral_microstepping`.

**Correctif appliqué :** l'homage latéral suit la même configuration de
micro-pas que les autres fonctions de mouvement.

### 4.3 `core/app.py` — `WinderApp` déprécié

`core/app.py` a été vidé et remplacé par un stub de compatibilité qui
lève aujourd'hui une erreur explicite. Cette suppression évite de conserver
un ancien bug de calcul d'accélération et clarifie que l'architecture active
est désormais `WindingEngine` / JSON-RPC.

### 4.4 Absence de tests unitaires sur `compute_ramp_times`

Étant une fonction pure sans dépendances, `compute_ramp_times` est idéale
pour des tests paramétrés couvrant :
- `max_accel = 0` (unconstrained)
- `duration_s` très court (cramp sur `max_ramp_fraction`)
- `start_rpm > 0` (vitesse initiale non nulle)

### 4.5 `MoveQueue._execute_ramp_move` et `WoundMove`

`_execute_ramp_move` récupère `move._config` via `getattr` (accès fragile).
`WoundMove` n'a pas de `_config` — il expose directement ses paramètres.
La récupération des `axis_ids` pour le tracking de position est donc vide
pour `WoundMove`. Ce n'est pas critique (position invalidée) mais mérite
une interface explicite (ex. `move.axis_ids: list[int]` sur `BaseMove`).

### 4.6 Backpressure entre `MoveQueue` et `WindingEngine`

`WindingEngine._wait_for_move_queue()` poll à 50 ms. Si `MoveQueue` est
occupée à streamer et que `WindingEngine._execute_program()` génère les
couches en avance, plusieurs `RampMove` peuvent s'accumuler dans la queue.
Une interface `MoveQueue.ready_for_enqueue()` permettrait un flow-control
plus déterministe.

---

## 5. Schéma de dépendances

```mermaid
graph TD
    subgraph "Entrée RPC"
        RPC["JsonRpcServer\n(Unix socket)"]
        WRH["WindingRpcHandler\n+ register_all()"]
    end

    subgraph "Orchestration"
        WE["WindingEngine\n+ run_axis()\n+ jog()\n+ wound_run()\n+ submit_program()"]
        MQ["MoveQueue\n(daemon thread)\n_execute_move()"]
    end

    subgraph "Moves (BaseMove)"
        BM["BaseMove\n(state, error, snapshot)"]
        MV["Move\n+ segments()"]
        CM["CompositeMove\n+ phases()"]
        RM["RampMove"]
        JM["JogMove"]
        WM["WoundMove"]
        HM["HomingMove"]
    end

    subgraph "Générateurs de segments"
        MASG["MultiAxisSegmentGenerator\n(t → RampConfig.steps_at)"]
        SSG["SynchronizedSegmentGenerator\n(t → turns_at → guide_pos_mm)"]
        SPK["SpindleKinematics"]
        WP["WindingPattern"]
        SE["ScatterEngine"]
    end

    subgraph "Transport"
        MARS["MultiAxisRampStreamer\nstream_all()"]
        ESPI["Esp32SpiTransport\nsend_multi_axis_segment_block_request()"]
    end

    subgraph "Config"
        AC["AppConfiguration\nspindle_max_acceleration_rpm\nlateral_max_rpm\n…"]
        CRT["compute_ramp_times()\n(ramp_config.py)"]
    end

    RPC --> WRH --> WE
    WE --> MQ
    WE --> CRT
    CRT --> AC
    MQ --> MARS --> ESPI

    MQ -->|enqueue BaseMove| BM
    BM --> MV & CM
    MV --> RM & JM & WM
    CM --> HM

    RM -->|segments()| MASG
    JM -->|segments()| MASG
    WM -->|segments()| SSG
    SSG --> SPK & WP & SE

    MARS -->|"next(generator)"| MASG
    MARS -->|"next(generator)"| SSG
```

**Flux d'exécution d'un `run_axis` :**

```
winding.run_axis(duration_s=5, targets=[{axis_id:0, rpm:500}])
    → WindingRpcHandler.run_axis()
    → WindingEngine.run_axis()
        → compute_ramp_times(500, 5, accel_limit, decel_limit, 6400)
        → RampMove(RampConfig(accel_s=…, cruise_s=…, decel_s=…))
        → MoveQueue.enqueue(move)
    → MoveQueue._execute_ramp_move(move)
        → MultiAxisRampStreamer(transport, [StreamAxisConfig(…)])
        → streamer._generator = move.segments()  → MultiAxisSegmentGenerator
        → streamer.stream_all()
            → _prefill() → send batch → poll status → send batch → …
            → Esp32SpiTransport.send_multi_axis_segment_block_request()
```
