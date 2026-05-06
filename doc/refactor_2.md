# Plan de refactoring — Motion Segmentation

> Branche cible : `refactor/motion-segmentation`
> Date : 2026-05-06
> Statut : **Tous les commits 1 à 6 terminés ✅**

---

## Contexte

Le firmware ESP32 n'accepte **qu'un seul type de commande de mouvement** :

```
MULTI_AXIS_SEGMENT_BLOCK
  └─ duration_us   : int
  └─ step_counts[] : int[N]
  └─ direction_mask: bitmask
```

Les types `STEP_BLOCK (0x10)` et `SEGMENT_BLOCK (0x11)` sont définis dans le protocole mais le firmware retourne `ESP_ERR_NOT_SUPPORTED` pour les deux. Le code Python contient néanmoins leurs payloads, leurs fonctions d'envoi, et deux classes de mouvement (`JogMove`, `RampMove`) dont l'une est réellement morte et l'autre est entrelacée avec le homing.

---

## Contrainte absolue — Ne pas toucher le homing

Le homing (déclenché via RPC) est **fonctionnel et gelé**. Aucune modification de :

- `HomingMove` et ses méthodes `_make_approach_move()`, `_make_backoff_move()`, `_make_search_move()`
- `MoveQueue._execute_homing_move()` et toute la logique de phase
- Les interfaces RPC liées au homing

### Conséquence directe sur le plan

`HomingMove` interne utilise `RampMove` → `RampMoveConfig` → `list[AxisMotionConfig]`.  
Ces trois classes **ne peuvent pas être supprimées**. Elles sont reclassées en infrastructure interne du homing, non publiées en API publique.

---

## 1. Ce qui est à supprimer

### `src/rpi/transport/messages.py`

| Symbole | Raison |
|---|---|
| `StepEntry` | Firmware : `ESP_ERR_NOT_SUPPORTED` pour `STEP_BLOCK` — **déjà commenté** |
| `StepBlockPayload` | Idem — **déjà commenté** |
| `_STEP_BLOCK_HEAD_STRUCT` | Utilisé uniquement par `StepBlockPayload` — **déjà commenté** |
| `STEP_BLOCK_SIZE` | Idem — **déjà commenté** |
| `_STEP_ENTRY_STRUCT` | Reliquat : seul appelant était `StepEntry.pack()` — **déjà commenté** |
| `MotionSegment` | Firmware : `ESP_ERR_NOT_SUPPORTED` pour `SEGMENT_BLOCK` — **déjà commenté** |
| `SegmentBlockPayload` | Idem — **déjà commenté** |
| `_SEGMENT_BLOCK_HEAD_STRUCT` | Utilisé uniquement par `SegmentBlockPayload` — **déjà commenté** |
| `SEGMENT_BLOCK_SIZE` | Idem — **déjà commenté** |
| `_SEGMENT_ENTRY_STRUCT` | Reliquat : seul appelant était `MotionSegment.pack()` (supprimé) — **commenté** |
| `make_step_block()` | Aucun appelant actif — **déjà commenté** |
| `make_segment_block()` | Idem — **déjà commenté** |
| `SpiStepFlags` | Dernière référence dans `StepEntry.pack()` (supprimé) — **déjà commenté** |

> `SpiMessageType.STEP_BLOCK` et `SpiMessageType.SEGMENT_BLOCK` sont **conservés** comme constantes de documentation du protocole fil.

### `src/rpi/transport/spi_transport.py`

Méthodes à supprimer (toutes déjà sans appelant) :

```
send_step_block()
send_step_block_request()
send_step_block_with_backpressure()
send_segment_block()
send_segment_block_request()
```

Imports morts à retirer : `StepBlockPayload`, `SegmentBlockPayload`, `make_step_block`, `make_segment_block`.

### `src/rpi/transport/__init__.py`

Retirer des imports et de `__dir__` (déjà commentés, à supprimer physiquement) :
`StepEntry`, `MotionSegment`, `StepBlockPayload`, `SegmentBlockPayload`,
`make_step_block`, `make_segment_block`, `STEP_BLOCK_SIZE`, `SEGMENT_BLOCK_SIZE`, `SpiStepFlags`.

### `src/rpi/motion/move.py`

Supprimer la classe `JogMove` entière.

Les trois sites d'appel sont remplacés par une construction `RampMove` équivalente (voir §5, commit 5).

---

## 2. Ce qui est à refactorer

### 2a. `MultiAxisSegment` — remplacer `directions` par `direction_mask`

**Avant :**
```python
@dataclass(slots=True)
class MultiAxisSegment:
    sequence: int
    duration_us: int
    steps: List[int]
    directions: List[int]   # tableau booléen par axe, converti en masque dans pack()
```

**Après :**
```python
@dataclass(slots=True)
class MultiAxisSegment:
    """Commande de mouvement atomique firmware.

    direction_mask : bit i = 1 → l'axe i avance en sens inverse.
    Le masque est calculé par le générateur amont (SegmentProducer), pas dans pack().
    """
    sequence: int
    duration_us: int
    steps: list[int]
    direction_mask: int
```

`MultiAxisSegmentBlockPayload.pack()` devient :
```python
payload += _MULTI_AXIS_SEGMENT_ENTRY_HEADER_STRUCT.pack(
    segment.sequence,
    segment.duration_us,
    segment.direction_mask,   # déjà un masque — plus de boucle ni de validation directions
)
# La validation len(segment.directions) == axis_count disparaît aussi
```

### 2b. `segment_generator.py` — déplacer le calcul du masque en amont et moderniser les types

`_compute_segment()` retourne actuellement `(list[int], list[int])` (steps, directions).

**Après :** retourne `(list[int], int)` (steps, direction_mask).

Moderniser en même temps les imports `typing` : `Tuple` (majuscule, Python < 3.9) → `tuple` (lowercase, PEP 585).

```python
# Avant
from typing import Callable, Iterator, Tuple

@abstractmethod
def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
    ...

# Après
from typing import Callable, Iterator          # Tuple retiré

@abstractmethod
def _compute_segment(self, time_start: float, time_end: float) -> tuple[list[int], int]:
    ...
```

`BaseSegmentGenerator.__iter__` :
```python
# Avant
steps, directions = self._compute_segment(self._time_cursor, next_cursor)
yield MultiAxisSegment(
    sequence=self._sequence,
    duration_us=duration_us,
    steps=steps,
    directions=directions,
)

# Après
steps, direction_mask = self._compute_segment(self._time_cursor, next_cursor)
yield MultiAxisSegment(
    sequence=self._sequence,
    duration_us=duration_us,
    steps=steps,
    direction_mask=direction_mask,
)
```

`StepProfileSegmentGenerator._compute_segment()` :
```python
# Avant — retourne (list[int], list[int])
def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
    steps = [0] * len(self.axis_profiles)
    directions = [0] * len(self.axis_profiles)

    for index, profile in enumerate(self.axis_profiles):
        target_steps = profile.step_at(time_end)
        delta_steps = target_steps - self._current_steps[index]
        count = int(round(self._axis_errors[index] + delta_steps))
        self._axis_errors[index] += delta_steps - float(count)
        self._current_steps[index] = target_steps

        is_negative = count < 0
        if is_negative:
            count = abs(count)
        directions[index] = 1 if is_negative ^ profile.reverse_direction else 0
        steps[index] = count

    return steps, directions

# Après — retourne (list[int], int)
def _compute_segment(self, time_start: float, time_end: float) -> tuple[list[int], int]:
    steps = [0] * len(self.axis_profiles)
    direction_mask = 0

    for index, profile in enumerate(self.axis_profiles):
        target_steps = profile.step_at(time_end)
        delta_steps = target_steps - self._current_steps[index]
        count = int(round(self._axis_errors[index] + delta_steps))
        self._axis_errors[index] += delta_steps - float(count)
        self._current_steps[index] = target_steps

        is_negative = count < 0
        if is_negative:
            count = abs(count)
        if is_negative ^ profile.reverse_direction:
            direction_mask |= (1 << index)
        steps[index] = count

    return steps, direction_mask
```

`SynchronizedSegmentGenerator` hérite de `StepProfileSegmentGenerator` et n'a pas sa propre `_compute_segment()` — **aucune modification nécessaire dans `synchronized_segment_generator.py`** pour ce changement : le générateur synchronisé hérite directement de la nouvelle implémentation.

### 2c. `transport/__init__.py` — nettoyer les lignes commentées

Retirer physiquement tous les blocs commentés (`#STEP_BLOCK_SIZE`, `#SpiStepFlags`, etc.) qui restent après les commits 1 et 2. Le fichier ne doit plus contenir de vestiges commentés.

### 2d. `transport/streamer.py` — retirer l'import mort de `AxisMotionConfig`/`RampConfig`

Ces imports ne sont plus nécessaires après que `StreamAxisConfig` n'expose plus de champ `ramp` à des appelants externes. Vérifier avant suppression avec :

```bash
grep -n "AxisMotionConfig\|RampConfig" src/rpi/transport/streamer.py
```

### 2e. `motion/move.py` — documenter le statut interne de `RampMove`

Ajouter un commentaire de module en tête de fichier :

```python
# RampMove, RampMoveConfig et AxisMotionConfig sont conservés intentionnellement
# comme infrastructure interne de HomingMove.
# Ils NE font PAS partie de l'API publique de mouvement.
# Voir: doc/architecture.md § Politique d'isolement du homing
```

### 2f. `motion/ramp_config.py` — supprimer les champs orphelins de `RampConfig`

Deux champs sont morts dans `RampConfig` — aucun appelant dans tout le codebase :

```python
# À supprimer si RampConfig n'est pas sérialisé/désérialisé depuis JSON externe
phase_segments: int = 8           # jamais lu nulle part
segment_duration_s: float = 0.05  # jamais lu (porté par RampMoveConfig à la place)
```

**Vérifier avant suppression :**
```bash
grep -rn "phase_segments\|\.segment_duration_s" src/rpi/
# Si zéro résultat hors ramp_config.py lui-même → supprimer sans risque
```

> Si une sérialisation JSON externe charge ces champs (fichiers de config, tests d'intégration), les marquer `deprecated` avec un commentaire plutôt que les supprimer.

---

## 3. Nouveau fichier — `SegmentProducer` (protocole structurel)

**`src/rpi/motion/segment_producer.py`** (nouveau, ~30 lignes)

```python
"""Protocole SegmentProducer.

Tout générateur satisfaisant ce protocole peut être utilisé comme source de
mouvement par MultiAxisRampStreamer.

Le homing est intentionnellement exclu : HomingMove est un CompositeMove
exécuté phase par phase par MoveQueue, pas via SegmentProducer.
"""
from __future__ import annotations

from typing import Iterator, Protocol

from transport.messages import MultiAxisSegment


class SegmentProducer(Protocol):
    """Produit un flux fini de MultiAxisSegment avec des numéros de séquence croissants.

    Implémenteurs existants :
      - StepProfileSegmentGenerator  (rampe trapézoïdale, utilisé par RampMove)
      - SynchronizedSegmentGenerator (engrenage électronique, utilisé par WoundMove)
    """

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        ...
```

Ajouter `SegmentProducer` dans `motion/__init__.py __all__`.

---

## 4. Architecture finale

```
Intention haut niveau
      │
      ▼
  Move.segments() → Iterator[MultiAxisSegment]
      │
      │  implémenté par :
      │    RampMove       — rampe trapézoïdale sur N axes (aussi utilisé par HomingMove)
      │    WoundMove      — engrenage électronique (broche + chariot)
      │
      ▼
  MoveQueue._wrap_segment_sequence()    ← affecte les numéros de séquence fil
      │
      ▼
  MultiAxisRampStreamer.stream_all()    ← back-pressure, retry, flush, endstop
      │
      ▼
  Esp32SpiTransport.send_multi_axis_segment_block_request()
      │
      ▼
  ESP32 ← MULTI_AXIS_SEGMENT_BLOCK uniquement
```

```
HomingMove (gelé — CompositeMove)
      │
      │  phases() :
      │    ("approach", RampMove, armed=True)
      │    ("backoff",  RampMove, armed=False)
      │    ("search",   RampMove, armed=True)
      │
      ▼
  MoveQueue._execute_homing_move()   ← arme/désarme l'endstop entre les phases
      │                                 exécute chaque phase via _execute_ramp_move()
      ▼
  [même pipeline que ci-dessus]
```

**Ce qui disparaît :** `StepBlockPayload`, `SegmentBlockPayload`, `JogMove`, toutes les fonctions d'envoi legacy, le tableau `directions` par axe, `_STEP_ENTRY_STRUCT`, `_SEGMENT_ENTRY_STRUCT`, `phase_segments`, `segment_duration_s` (dans `RampConfig`).

---

## 5. Plan de migration — étape par étape

### Commit 1 — Supprimer le code step-based legacy ✅ FAIT
**Fichiers :** `transport/messages.py`, `transport/spi_transport.py`, `transport/__init__.py`

1. Supprimé `StepEntry`, `StepBlockPayload`, `_STEP_BLOCK_HEAD_STRUCT`, `STEP_BLOCK_SIZE`, `SpiStepFlags`
2. Supprimé `make_step_block()`
3. Supprimé `send_step_block`, `send_step_block_request`, `send_step_block_with_backpressure`
4. Retiré les imports morts
5. Nettoyé `transport/__init__.py`

**État actuel :** code commenté, pas encore effacé physiquement.

---

### Commit 2 — Supprimer l'abstraction mono-axe SEGMENT_BLOCK ✅ FAIT
**Fichiers :** mêmes fichiers

1. Supprimé `MotionSegment`, `SegmentBlockPayload`, `_SEGMENT_BLOCK_HEAD_STRUCT`, `SEGMENT_BLOCK_SIZE`
2. Supprimé `make_segment_block()`
3. Supprimé `send_segment_block`, `send_segment_block_request`
4. Nettoyé `transport/__init__.py`

**État actuel :** code commenté, pas encore effacé physiquement.

---

### Commit 3 — Supprimer le code mort résiduel et remplacer `directions` par `direction_mask`

**Périmètre :** `transport/messages.py`, `transport/__init__.py`, `motion/segment_generator.py`, `transport/spi_transport.py` (si méthodes encore présentes)

#### Étape 3.1 — Effacer physiquement les blocs commentés dans `messages.py`

Supprimer **toutes** les lignes commençant par `#` qui correspondent à du code mort (pas les vrais commentaires) :

- Bloc `StepEntry` (lignes ~5820–5828)
- Bloc `StepBlockPayload` (lignes ~5829–5845)
- Bloc `SpiStepFlags` (lignes ~5846–5848)
- `#def make_step_block(...)` (ligne ~5850–5851)
- Bloc `MotionSegment` (lignes ~5855–5864)
- Bloc `SegmentBlockPayload` (lignes ~5870–5886)
- `#def make_segment_block(...)` (lignes ~5888–5889)
- Commentaires `#STEP_BLOCK_SIZE` et `#SEGMENT_BLOCK_SIZE` en en-tête

Supprimer les deux structs orphelins **actifs** (non commentés) :
```python
# À supprimer — plus d'appelant après commits 1 et 2
_STEP_ENTRY_STRUCT = struct.Struct("<IB")
_SEGMENT_ENTRY_STRUCT = struct.Struct("<HHhBB")
```

**Vérification :**
```bash
grep -n "_STEP_ENTRY_STRUCT\|_SEGMENT_ENTRY_STRUCT" src/rpi/transport/messages.py
# → zéro résultat attendu
```

#### Étape 3.2 — Effacer physiquement les lignes commentées dans `transport/__init__.py`

Transformer les commentaires en vraies suppressions :
```python
# Avant (lignes avec #) :
#STEP_BLOCK_SIZE,
#SEGMENT_BLOCK_SIZE,
#SpiStepFlags,
#StepEntry,
#MotionSegment,
#StepBlockPayload,
#SegmentBlockPayload,
#make_step_block,
#make_segment_block,
# ... et dans __dir__() aussi

# Après : retirées complètement
```

#### Étape 3.3 — Remplacer `MultiAxisSegment.directions` par `direction_mask` dans `messages.py`

```python
# Avant
@dataclass(slots=True)
class MultiAxisSegment:
    sequence: int
    duration_us: int
    steps: List[int]
    directions: List[int]

# Après
@dataclass(slots=True)
class MultiAxisSegment:
    """Commande de mouvement atomique firmware.

    direction_mask : bit i = 1 → l'axe i avance en sens inverse.
    Le masque est calculé par le générateur amont, pas dans pack().
    """
    sequence: int
    duration_us: int
    steps: list[int]
    direction_mask: int
```

Moderniser aussi `from typing import ..., List` → retirer `List` si plus utilisé dans le fichier (passer à `list[...]` lowercase pour les nouvelles annotations).

#### Étape 3.4 — Simplifier `MultiAxisSegmentBlockPayload.pack()` dans `messages.py`

```python
# Avant
for segment in self.segments:
    if len(segment.steps) != axis_count:
        raise ValueError(...)
    if len(segment.directions) != axis_count:   # ← disparaît
        raise ValueError(...)

    direction_mask = 0
    for axis_index, direction in enumerate(segment.directions):  # ← boucle disparaît
        if direction:
            direction_mask |= 1 << axis_index

    payload += _MULTI_AXIS_SEGMENT_ENTRY_HEADER_STRUCT.pack(
        segment.sequence,
        segment.duration_us,
        direction_mask,
    )
    for step in segment.steps:
        payload += _STEP_COUNT_STRUCT.pack(step)

# Après
for segment in self.segments:
    if len(segment.steps) != axis_count:
        raise ValueError(
            f"segment step count {len(segment.steps)} does not match axis count {axis_count}"
        )
    payload += _MULTI_AXIS_SEGMENT_ENTRY_HEADER_STRUCT.pack(
        segment.sequence,
        segment.duration_us,
        segment.direction_mask,   # masque précalculé — zéro boucle
    )
    for step in segment.steps:
        payload += _STEP_COUNT_STRUCT.pack(step)
```

#### Étape 3.5 — Mettre à jour `BaseSegmentGenerator.__iter__` dans `segment_generator.py`

```python
# Avant
steps, directions = self._compute_segment(self._time_cursor, next_cursor)
yield MultiAxisSegment(
    sequence=self._sequence,
    duration_us=duration_us,
    steps=steps,
    directions=directions,
)

# Après
steps, direction_mask = self._compute_segment(self._time_cursor, next_cursor)
yield MultiAxisSegment(
    sequence=self._sequence,
    duration_us=duration_us,
    steps=steps,
    direction_mask=direction_mask,
)
```

#### Étape 3.6 — Mettre à jour la signature abstraite et l'implémentation concrète dans `segment_generator.py`

```python
# Avant — imports
from typing import Callable, Iterator, Tuple

# Après — retirer Tuple (PEP 585, Python ≥ 3.9)
from typing import Callable, Iterator

# Avant — méthode abstraite
@abstractmethod
def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
    ...

# Après
@abstractmethod
def _compute_segment(self, time_start: float, time_end: float) -> tuple[list[int], int]:
    ...

# Avant — StepProfileSegmentGenerator._compute_segment
def _compute_segment(self, time_start: float, time_end: float) -> Tuple[list[int], list[int]]:
    steps = [0] * len(self.axis_profiles)
    directions = [0] * len(self.axis_profiles)
    for index, profile in enumerate(self.axis_profiles):
        ...
        directions[index] = 1 if is_negative ^ profile.reverse_direction else 0
        steps[index] = count
    return steps, directions

# Après
def _compute_segment(self, time_start: float, time_end: float) -> tuple[list[int], int]:
    steps = [0] * len(self.axis_profiles)
    direction_mask = 0
    for index, profile in enumerate(self.axis_profiles):
        ...
        if is_negative ^ profile.reverse_direction:
            direction_mask |= (1 << index)
        steps[index] = count
    return steps, direction_mask
```

> `SynchronizedSegmentGenerator` hérite de `StepProfileSegmentGenerator` sans surcharger `_compute_segment()` — **aucune modification nécessaire** dans `synchronized_segment_generator.py`.

**Test d'invariant commit 3 :** pour un axe unique en sens inverse à l'index 1, `direction_mask == 0b10 == 2`.

**Vérification finale :**
```bash
grep -rn "\.directions" src/rpi/
# → zéro résultat attendu

grep -rn "direction_mask" src/rpi/
# → résultats dans messages.py, segment_generator.py uniquement (+ nouveaux appelants)

mypy --strict src/rpi/transport/messages.py src/rpi/motion/segment_generator.py
```

---

### Commit 4 — Introduire le protocole `SegmentProducer`

**Fichiers :** `motion/segment_producer.py` (nouveau), `motion/__init__.py`

#### Étape 4.1 — Créer `motion/segment_producer.py`

```python
"""Protocole SegmentProducer.

Tout générateur satisfaisant ce protocole peut être utilisé comme source de
mouvement par MultiAxisRampStreamer.

Le homing est intentionnellement exclu : HomingMove est un CompositeMove
exécuté phase par phase par MoveQueue, pas via SegmentProducer.
"""
from __future__ import annotations

from typing import Iterator, Protocol

from transport.messages import MultiAxisSegment


class SegmentProducer(Protocol):
    """Produit un flux fini de MultiAxisSegment avec des numéros de séquence croissants.

    Implémenteurs existants :
      - StepProfileSegmentGenerator  (rampe trapézoïdale, utilisé par RampMove)
      - SynchronizedSegmentGenerator (engrenage électronique, utilisé par WoundMove)

    Les implémenteurs sont des générateurs : __iter__ doit être réentrant (chaque
    appel produit un nouveau flux depuis le début).
    """

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        ...
```

#### Étape 4.2 — Ajouter `SegmentProducer` dans `motion/__init__.py`

```python
# Dans __all__
__all__ = [
    "RampConfig",
    "compute_ramp_times",
    "AxisMotionConfig",
    "MultiAxisSegmentGenerator",
    "SpindleKinematics",
    "SegmentProducer",          # ← nouveau
]

# Dans __getattr__
if name == "SegmentProducer":
    from .segment_producer import SegmentProducer
    return SegmentProducer
```

#### Étape 4.3 — Annoter `MultiAxisRampStreamer.set_generator()` dans `transport/streamer.py`

```python
# Avant (type approximatif ou absent)
def set_generator(self, generator) -> None:
    ...

# Après
from motion.segment_producer import SegmentProducer
from typing import Iterator, TYPE_CHECKING

def set_generator(self, generator: SegmentProducer | Iterator[MultiAxisSegment]) -> None:
    ...
```

> Utiliser `TYPE_CHECKING` si l'import crée une dépendance circulaire entre `transport` et `motion`.

**Vérification :**
```bash
mypy --strict src/rpi/motion/segment_producer.py src/rpi/motion/__init__.py
```

---

### Commit 5 — Supprimer `JogMove`, inliner aux trois sites d'appel

**Fichiers :** `motion/move.py`, `core/command_service.py`, `winding/service.py`

#### Étape 5.1 — Inliner dans `core/command_service.py` (méthode `jog`)

Localisation : méthode `MotionCommandService.jog()`, construction du move (~ligne 405).

```python
# Avant
move = JogMove(
    name=f"jog_{axis_id}",
    axis_id=axis_id,
    steps_per_rev=steps_per_rev,
    steps=steps,
    rpm=rpm,
    reverse_direction=reverse,
)

# Après
total_s = (steps / float(steps_per_rev)) / (rpm / 60.0)
move = RampMove(
    name=f"jog_{axis_id}",
    config=RampMoveConfig(
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
                    reverse_direction=reverse,
                ),
            )
        ],
    ),
)
```

Mettre à jour l'import en tête de `command_service.py` :

```python
# Avant
from motion.move import JogMove, RampMove, RampMoveConfig

# Après
from motion.move import RampMove, RampMoveConfig
```

#### Étape 5.2 — Inliner dans `winding/service.py` (méthode `_move_lateral_to`)

Localisation : méthode `AdaptiveWindingService._move_lateral_to()`, bloc construction du move (~ligne 9466).

```python
# Avant
move = JogMove(
    name="adaptive_window_reposition",
    axis_id=self._config.lateral_axis_id,
    steps_per_rev=(
        self._config.lateral_steps_per_revolution
        * self._config.lateral_microstepping
    ),
    steps=abs(delta_steps),
    rpm=min(max(runtime.snapshot()["target_rpm"], 60.0), float(self._config.lateral_max_rpm)),
    reverse_direction=(delta_steps < 0),
)
self._move_queue.enqueue(move)
self._move_queue.wait_until_idle(timeout_s=max(move.axis_configs[0].ramp.total_duration * 4.0, 10.0))

# Après
steps_per_rev = (
    self._config.lateral_steps_per_revolution
    * self._config.lateral_microstepping
)
rpm = min(max(runtime.snapshot()["target_rpm"], 60.0), float(self._config.lateral_max_rpm))
total_s = (abs(delta_steps) / float(steps_per_rev)) / (rpm / 60.0)
move = RampMove(
    name="adaptive_window_reposition",
    config=RampMoveConfig(
        axis_configs=[
            AxisMotionConfig(
                axis_id=self._config.lateral_axis_id,
                ramp=RampConfig(
                    axis_id=self._config.lateral_axis_id,
                    steps_per_rev=steps_per_rev,
                    target_rpm=rpm,
                    accel_s=min(0.15, total_s * 0.2),
                    cruise_s=max(total_s - 0.3, 0.0),
                    decel_s=min(0.15, total_s * 0.2),
                    reverse_direction=(delta_steps < 0),
                ),
            )
        ],
    ),
)
self._move_queue.enqueue(move)
self._move_queue.wait_until_idle(
    timeout_s=max(move.axis_configs[0].ramp.total_duration * 4.0, 10.0)
)
# Nota : move.axis_configs[0].ramp.total_duration fonctionne car
# RampMove expose axis_configs → list[AxisMotionConfig]
```

Mettre à jour l'import en tête de `winding/service.py` :

```python
# Avant
from motion.move import JogMove

# Après — ajouter les imports nécessaires à l'inlining
from motion.move import RampMove, RampMoveConfig
from motion import AxisMotionConfig, RampConfig
```

#### Étape 5.3 — Supprimer `JogMove` de `motion/move.py`

Supprimer la classe complète `JogMove` (environ 65 lignes, de la docstring jusqu'à la dernière propriété).

Vérifier qu'aucun autre import de `JogMove` ne subsiste :
```bash
grep -rn "JogMove" src/rpi/
# → zéro résultat attendu
```

#### Étape 5.4 — Supprimer `RampMoveConfig` de `motion/ramp_config.py` (si c'est là qu'il vit)

`RampMoveConfig` est défini dans `move.py` (`@dataclass(slots=True) class RampMoveConfig`). Il est conservé car utilisé par `RampMove` et `HomingMove`. **Ne pas supprimer.**

**Vérification finale commit 5 :**
```bash
grep -rn "JogMove" src/rpi/
# → zéro résultat

mypy --strict src/rpi/core/command_service.py src/rpi/winding/service.py src/rpi/motion/move.py
```

---

### Commit 6 — Nettoyage, champs orphelins et documentation

**Fichiers :** `motion/move.py`, `motion/ramp_config.py`, `motion/__init__.py`, `transport/streamer.py`, `README.md`, `doc/architecture.md`, `doc/stepper_engine.md`, `doc/spi_protocol.md`

#### Étape 6.1 — Ajouter le commentaire de gel homing dans `move.py`

En tête de module, après les imports :

```python
# ---------------------------------------------------------------------------
# NOTE D'ARCHITECTURE — Infrastructure homing gelée
# ---------------------------------------------------------------------------
# RampMove, RampMoveConfig et AxisMotionConfig sont conservés intentionnellement
# comme infrastructure interne de HomingMove.
# Ils NE font PAS partie de l'API publique de mouvement.
# HomingMove._make_approach_move(), _make_backoff_move(), _make_search_move()
# sont corrects, testés et gelés — toute modification nécessite une tâche
# spécifique au homing avec revue dédiée.
# Voir : doc/architecture.md § Politique d'isolement du homing
# ---------------------------------------------------------------------------
```

#### Étape 6.2 — Supprimer les champs orphelins dans `motion/ramp_config.py`

Après vérification grep (voir §2f) :

```python
# Avant
@dataclass(slots=True)
class RampConfig:
    axis_id: int = 0
    steps_per_rev: int = 200 * 32
    start_rpm: float = 0.0
    target_rpm: float = 1000.0
    accel_s: float = 10.0
    cruise_s: float = 3.0
    decel_s: float = 10.0
    resolution_hz: int = 40_000_000
    reverse_direction: bool = False
    phase_segments: int = 8           # ← orphelin
    segment_duration_s: float = 0.05  # ← orphelin (porté par RampMoveConfig)

# Après
@dataclass(slots=True)
class RampConfig:
    """Configuration d'une rampe trapézoïdale sur un axe.

    La durée des segments est portée par RampMoveConfig, pas par RampConfig.
    """
    axis_id: int = 0
    steps_per_rev: int = 200 * 32
    start_rpm: float = 0.0
    target_rpm: float = 1000.0
    accel_s: float = 10.0
    cruise_s: float = 3.0
    decel_s: float = 10.0
    resolution_hz: int = 40_000_000
    reverse_direction: bool = False
```

#### Étape 6.3 — Retirer l'import mort dans `transport/streamer.py`

```bash
grep -n "AxisMotionConfig\|RampConfig" src/rpi/transport/streamer.py
```

Supprimer les lignes d'import identifiées si elles ne sont pas utilisées ailleurs dans le fichier.

#### Étape 6.4 — Évaluer `MultiAxisSegmentGenerator` dans `motion/__init__.py`

`MultiAxisSegmentGenerator` était utilisé directement par `JogMove` (supprimé au commit 5). Après commit 5, les seuls appelants sont `RampMove.segments()` (interne) et potentiellement des scripts de test.

```bash
grep -rn "MultiAxisSegmentGenerator" src/rpi/
```

Si aucun appelant externe ne subsiste, le retirer de `__all__` et de `__getattr__` dans `motion/__init__.py`. Il reste accessible via `from motion.multi_axis_segment_generator import MultiAxisSegmentGenerator` pour les tests.

#### Étape 6.5 — Mettre à jour la documentation

| Fichier | Changement requis |
|---|---|
| `README.md` | Mettre à jour le pipeline de mouvement ; supprimer les mentions de step-block et segment-block ; noter l'isolement du homing |
| `doc/architecture.md` | Redessiner la section pipeline mouvement ; documenter `SegmentProducer` ; ajouter l'encadré « Homing isolation » (voir texte ci-dessous) |
| `doc/spi_protocol.md` | Marquer `STEP_BLOCK (0x10)` et `SEGMENT_BLOCK (0x11)` comme **types legacy rejetés par le firmware** (`ESP_ERR_NOT_SUPPORTED`) |
| `doc/stepper_engine.md` | Documenter `MultiAxisSegment.direction_mask` remplaçant `directions` ; documenter le contrat `SegmentProducer` |
| `doc/sequencing.md` | Pas de changement requis sauf si les docs de compteur de séquence référencent le chemin legacy |
| Docstrings inline | `Move.segments()`, `MultiAxisRampStreamer.set_generator()`, `BaseSegmentGenerator.__iter__()` |

Texte à ajouter dans `doc/architecture.md` :

> **Politique d'isolement du homing :** `HomingMove` et son infrastructure interne (`RampMove`, `RampMoveConfig`, `AxisMotionConfig`) sont intentionnellement non refactorisés. Ils sont corrects, testés et gelés. Toute modification nécessite une tâche spécifique au homing avec revue dédiée.

---

## 6. Ce qui n'est PAS touché

| Élément | Raison |
|---|---|
| `HomingMove` et ses `_make_*_move()` | Gelé par contrainte |
| `MoveQueue._execute_homing_move()` | Idem |
| `RampMove`, `RampMoveConfig`, `AxisMotionConfig` | Requis par HomingMove |
| `SpiMessageType.STEP_BLOCK`, `SpiMessageType.SEGMENT_BLOCK` | Constantes de documentation protocole |
| `mock_spi_transport.py` | Infrastructure de test ; `send_multi_axis_segment_block_request` est déjà la seule méthode de mouvement utilisée |
| `WoundMove`, `SynchronizedSegmentGenerator` | Mis à jour uniquement pour le changement de type `direction_mask` (commit 3) |
| Toutes les interfaces RPC (`jsonrpc/`, `wendy/`) | Hors pipeline de mouvement |

---

## 7. Workflow Git

```bash
git checkout -b refactor/motion-segmentation
```

| # | Message de commit | Périmètre | Statut |
|---|---|---|---|
| 1 | `refactor(transport): remove legacy STEP_BLOCK payload and send functions` | `messages.py`, `spi_transport.py`, `__init__.py` | ✅ FAIT |
| 2 | `refactor(transport): remove mono-axis SEGMENT_BLOCK payload and send functions` | Mêmes fichiers | ✅ FAIT |
| 3 | `refactor(motion): purge commented dead code, replace MultiAxisSegment.directions with direction_mask` | `messages.py`, `__init__.py`, `segment_generator.py` | ✅ FAIT |
| 4 | `feat(motion): introduce SegmentProducer structural protocol` | `motion/segment_producer.py` (nouveau), `motion/__init__.py` | ✅ FAIT |
| 5 | `refactor(motion): delete JogMove, inline at all three call sites` | `move.py`, `command_service.py`, `winding/service.py` | ✅ FAIT |
| 6 | `docs+cleanup: remove orphan RampConfig fields, annotate homing isolation, align docs` | `ramp_config.py`, `move.py`, `streamer.py`, `motion/__init__.py`, docs | ✅ FAIT |

Chaque commit doit compiler et passer `mypy --strict` avant le suivant.

---

## 8. Checklist de vérification finale

Après le commit 6, vérifier l'absence totale de code mort :

```bash
# Aucun reliquat step/segment
grep -rn "StepBlock\|SegmentBlock\|StepEntry\|MotionSegment\|SpiStepFlags" src/rpi/
grep -rn "_STEP_ENTRY_STRUCT\|_SEGMENT_ENTRY_STRUCT" src/rpi/

# Aucun JogMove
grep -rn "JogMove" src/rpi/

# Aucune référence à directions (tableau)
grep -rn "\.directions\b" src/rpi/

# Aucun champ orphelin RampConfig
grep -rn "phase_segments\b" src/rpi/

# Mypy propre
mypy --strict src/rpi/
```