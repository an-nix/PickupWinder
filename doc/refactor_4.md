# Analyse de l'état du code — Post refactor_3.md

> Date : 2026-05-06
> Basé sur : dump source complet + `refactor_3.md` (Parties A, B, C ✅ — Partie D partiellement ✅)

---

## Ce que le code confirme

Lecture directe du dump. État réel par rapport au plan documenté.

### Parties A, B, C — Tout confirmé ✅

| Item | Attendu | Observé dans le code |
|---|---|---|
| `HomingPhaseDescriptor` | Introduit (B1) | ✅ Défini dans `move.py` |
| `_execute_composite` générique | B2 | ✅ Présent, dispatche sur `CompositeMove` |
| `try/finally` endstop | B3 | ✅ `finally: self._set_endstop_armed(move.axis_id, arm=False)` |
| `_make_phase_move` helper | B4 | ✅ Présent dans `HomingMove` |
| Import `HomingMove` retiré de `move_queue.py` | B5 | ✅ Import : `BaseMove, CompositeMove, HomingPhaseDescriptor, Move, RampMove` — pas de `HomingMove` |
| `axis_id` / `home_position_steps` sur `CompositeMove` | C1 | ✅ Déclarés comme attributs abstraits |
| `_compute_backoff_timeout` simplifié | C2 | ✅ Accepte `RampMove`, 3 lignes |
| `_ENDSTOP_VERIFY_TIMEOUT_S` supprimé | C5 | ✅ Absent |
| Shims `try/except TypeError` dans `move_queue.py` | C6 | ✅ Supprimés — `_read_status`, `_wait_for_transport_request_result`, `_make_streamer` sont propres |
| `SynchronizedMove` ABC | C4 | ✅ Défini dans `wound_move.py`, dispatch par `isinstance` dans `_execute_move` |
| `build_jog_move` dans `move_builders.py` | A3 | ✅ Présent et utilisé dans `service.py` |
| `Optional` → `X \| None` | A2 | ✅ Non vérifié dans le dump (config.py tronqué) |

### Partie D — État réel

| Item | Plan | Observé |
|---|---|---|
| D1a — `try/except TypeError` dans `engine.py` | ✅ dans plan | ✅ **`request_stop` est propre** — `self._move_queue.clear(stop_plan=effective_stop_plan)` direct, pas de try/except |
| D1b — `hasattr` guard dans `status()` | ✅ dans plan | ✅ **`status()` est propre** — `move_queue_status = self._move_queue.status()` direct |
| D2 — `preclear_move()` sur `CompositeMove` | ⬜ À faire | ✅ **DÉJÀ FAIT** — `preclear_move()` déclaré comme `@abstractmethod` dans `CompositeMove`, implémenté dans `HomingMove` |
| D3 — `_finalize_streamer_move` factorisé | ⬜ À faire | ❌ **PAS FAIT** — duplication confirmée entre `_execute_ramp_move` et `_execute_wound_move` |
| D4 — Export `SynchronizedMove` | ⬜ À faire | ❌ **PAS FAIT** — absent de `winding/__init__.py` |
| D5 — Import mort `Move` dans `adaptive.py` | ⬜ À faire | ❌ **PAS FAIT** — `from motion.move import Move` toujours présent (line ~7851) mais `Move` n'est plus utilisé dans le corps depuis que `AdaptiveWindingMove` hérite de `SynchronizedMove` |

---

## Anomalies trouvées dans le code que le plan ne documente pas

Ces problèmes ont été identifiés lors de la lecture exhaustive. Ils ne figurent dans aucun des plans précédents.

---

### 🔴 BUG ACTIF — `MultiAxisSegmentBlockPayload.pack()` : indentation cassée

**Fichier :** `transport/messages.py` (~ligne 5524)  
**Sévérité : Critique — provoque un `AttributeError` à l'exécution**

```python
# Code actuel — CASSÉ
for segment in self.segments:
    if len(segment.steps) != axis_count:
        raise ValueError(...)
    if len(segment.directions) != axis_count:   # ← champ supprimé, AttributeError garanti
        raise ValueError(
            f"segment direction count {len(segment.directions)} does not match axis count {axis_count}"
        )

            payload += _MULTI_AXIS_SEGMENT_ENTRY_HEADER_STRUCT.pack(   # ← indenté sous le raise ! dead code
            segment.sequence,
            segment.duration_us,
            segment.direction_mask,
        )
    for step in segment.steps:
        payload += _STEP_COUNT_STRUCT.pack(step)
```

Deux problèmes simultanés :
1. `segment.directions` est accédé alors que le champ a été renommé en `direction_mask` — `AttributeError` garanti sur tout appel réel.
2. Le bloc `payload += _MULTI_AXIS_SEGMENT_ENTRY_HEADER_STRUCT.pack(...)` est indenté à l'intérieur du `raise ValueError` (code mort absolu — jamais exécuté).

**Correction :**
```python
for segment in self.segments:
    if len(segment.steps) != axis_count:
        raise ValueError(
            f"segment step count {len(segment.steps)} does not match axis count {axis_count}"
        )
    payload += _MULTI_AXIS_SEGMENT_ENTRY_HEADER_STRUCT.pack(
        segment.sequence,
        segment.duration_us,
        segment.direction_mask,
    )
    for step in segment.steps:
        payload += _STEP_COUNT_STRUCT.pack(step)
```

**Vérification immédiate :**
```bash
python3 -c "
from transport.messages import MultiAxisSegment, MultiAxisSegmentBlockPayload
seg = MultiAxisSegment(sequence=0, duration_us=4000, steps=[100], direction_mask=0)
payload = MultiAxisSegmentBlockPayload(axis_ids=[0], block_seq=0, segments=[seg])
payload.pack()
print('OK')
"
```

---

### 🟠 Import mort — `List` (majuscule) dans `transport/messages.py`

**Fichier :** `transport/messages.py` (ligne 5389)

```python
from typing import Iterable, List
```

`List` majuscule est utilisé dans :
- `MultiAxisSegment.steps: List[int]` — doit passer à `list[int]`
- `MultiAxisSegmentBlockPayload.axis_ids: List[int]` — idem
- `MultiAxisSegmentBlockPayload.segments: List[MultiAxisSegment]` — idem

Alors que `direction_mask: int` a déjà été modernisé. Incohérence dans le même dataclass. `List` peut être retiré des imports une fois les 3 annotations corrigées.

```python
# Avant
from typing import Iterable, List

@dataclass(slots=True)
class MultiAxisSegment:
    sequence: int
    duration_us: int
    steps: List[int]
    direction_mask: int

# Après
from typing import Iterable  # List supprimé

@dataclass(slots=True)
class MultiAxisSegment:
    sequence: int
    duration_us: int
    steps: list[int]
    direction_mask: int
```

---

### 🟠 `_make_streamer` — paramètre `axis_configs` non typé

**Fichier :** `motion/move_queue.py` (~ligne 4181)

```python
def _make_streamer(
    self,
    axis_configs,                        # ← pas de type
    *,
    keep_enabled_axes: set[int] | None = None,
    initial_segments_dropped: int = 0,
) -> MultiAxisRampStreamer:
```

`axis_configs` devrait être typé `list[AxisMotionConfig]`. L'import de `AxisMotionConfig` est nécessaire (il n'est pas dans les imports actuels de `move_queue.py`).

```python
# Ajouter dans les imports
from motion.move import BaseMove, CompositeMove, HomingPhaseDescriptor, Move, RampMove
from motion.multi_axis_segment_generator import AxisMotionConfig  # ← nouveau

def _make_streamer(
    self,
    axis_configs: list[AxisMotionConfig],
    *,
    keep_enabled_axes: set[int] | None = None,
    initial_segments_dropped: int = 0,
) -> MultiAxisRampStreamer:
```

---

### 🟠 `_wrap_segment_sequence` — paramètres non typés

**Fichier :** `motion/move_queue.py` (~ligne 4613)

```python
def _wrap_segment_sequence(self, generator: Any, start_sequence: int) -> Iterator[Any]:
```

`generator` est en réalité un `Iterator[MultiAxisSegment]`. La signature peut être précisée :

```python
from transport.messages import MultiAxisSegment  # déjà importé

def _wrap_segment_sequence(
    self,
    generator: Iterator[MultiAxisSegment],
    start_sequence: int,
) -> Iterator[MultiAxisSegment]:
```

---

### 🟡 `_execute_wound_move` — boucle finale légèrement différente de `_execute_ramp_move`

**Fichier :** `motion/move_queue.py`

La conclusion de `_execute_wound_move` diffère subtilement de `_execute_ramp_move` dans la boucle de position :

```python
# Dans _execute_ramp_move :
for ax_id in axis_ids:
    delta = move.expected_delta_steps(ax_id)
    if delta is not None and ax_id in self._axis_states:
        self._axis_states[ax_id].advance_position(delta)

# Dans _execute_wound_move :
for ax_id in move.axis_ids:
    if ax_id not in self._axis_states:
        continue
    delta = move.expected_delta_steps(ax_id)
    if delta is None:
        self._axis_states[ax_id].invalidate_position()
        continue
    self._axis_states[ax_id].advance_position(delta)
```

`_execute_wound_move` invalide la position si `expected_delta_steps` retourne `None`, ce que `_execute_ramp_move` ne fait pas. `WoundMove.expected_delta_steps()` retourne toujours `None` — donc `_execute_wound_move` invalide systématiquement la position après chaque winding move. Ce comportement est peut-être intentionnel (position inconnue après un mouvement synchronisé), mais il rend impossible l'extraction d'un `_finalize_streamer_move` totalement identique (D3).

**Action :** avant de fusionner (D3), décider explicitement si la sémantique de `_execute_wound_move` est correcte ou si `WoundMove.expected_delta_steps()` doit calculer le delta réel pour le traverse axis.

---

### 🟡 `SynchronizedMove` — attributs de classe non typés comme `ClassVar`

**Fichier :** `winding/wound_move.py` (~ligne 9395)

```python
class SynchronizedMove(Move, ABC):
    kinematics: SpindleKinematics
    spindle_cfg: SyncAxisConfig
    segment_duration_s: float
```

Ces annotations sans `ClassVar` ni `field()` sont des annotations de classe nues — elles documentent l'interface mais ne sont pas des attributs d'instance. Mypy avec `--strict` peut les interpréter comme des attributs d'instance non initialisés et générer des faux positifs. Utiliser `Protocol` ou déclarer des `@abstractmethod @property` serait plus rigoureux, mais casse l'héritage simple. Alternative minimaliste : ajouter un commentaire explicite.

```python
class SynchronizedMove(Move, ABC):
    """Abstract base for synchronized spindle+traverse moves.

    Concrete subclasses must set these instance attributes in __init__:
    """
    # Contract: set by subclass __init__
    kinematics: SpindleKinematics
    spindle_cfg: SyncAxisConfig
    segment_duration_s: float
```

---

### 🟡 `_execute_composite` — bloc `if/elif/else` redondant pour la précondition homing

**Fichier :** `motion/move_queue.py` (~ligne 4767)

```python
if initial_state == LATERAL_ENDSTOP_ABSENT:
    self._ensure_homing_can_start(move.axis_id, "start")
elif initial_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
    self._clear_closed_endstop_before_homing(move)
    self._ensure_homing_can_start(move.axis_id, "start")
else:
    self._ensure_homing_can_start(move.axis_id, "start")
```

`_ensure_homing_can_start` est appelé dans les trois branches. Peut être simplifié :

```python
if initial_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
    self._clear_closed_endstop_before_homing(move)
self._ensure_homing_can_start(move.axis_id, "start")
```

---

### 🟡 `AdaptiveWindingMove` — n'hérite pas de `SynchronizedMove`

**Fichier :** `winding/adaptive.py` (~ligne 8450)

```python
class AdaptiveWindingMove(Move):   # ← hérite de Move, pas de SynchronizedMove
```

Le plan C4 indiquait que le dispatch duck-type `is_synchronized_move` avait été supprimé. En regardant `_execute_move` :

```python
elif isinstance(move, SynchronizedMove):
    self._execute_wound_move(move)
```

Si `AdaptiveWindingMove` hérite de `Move` et pas de `SynchronizedMove`, il tomberait dans la branche `_execute_ramp_move`, qui vérifie `axis_configs` — lequel est `None` sur `AdaptiveWindingMove` — et marquerait le move comme FAILED.

**Vérification urgente :**
```bash
grep -n "class AdaptiveWindingMove" src/rpi/winding/adaptive.py
grep -n "is_synchronized_move" src/rpi/winding/adaptive.py
```

Si `AdaptiveWindingMove` n'hérite toujours pas de `SynchronizedMove`, C4 est partiellement appliqué et le système est cassé pour les sessions adaptatives. La correction est d'ajouter l'héritage :

```python
# winding/adaptive.py
from winding.wound_move import SynchronizedMove

class AdaptiveWindingMove(SynchronizedMove):   # ← changer Move → SynchronizedMove
    ...
```

---

## Plan d'action — Items restants

### Priorité CRITIQUE (bugs actifs)

| # | Fichier | Problème | Action |
|---|---|---|---|
| **E1** | `transport/messages.py` | Bug `pack()` : `segment.directions` + indentation cassée | Corriger immédiatement — 8 lignes |
| **E2** | `winding/adaptive.py` | `AdaptiveWindingMove` hérite de `Move` au lieu de `SynchronizedMove` | Vérifier et corriger si C4 incomplet |

### Priorité HAUTE (D restants)

| # | Fichier | Problème | Action |
|---|---|---|---|
| **D3** | `motion/move_queue.py` | ~15 lignes dupliquées entre `_execute_ramp_move` et `_execute_wound_move` | Extraire `_finalize_streamer_move` — après décision sur la sémantique `expected_delta_steps` pour `WoundMove` |
| **D4** | `winding/__init__.py` | `SynchronizedMove` non exporté | Ajouter 1 ligne d'import |
| **D5** | `winding/adaptive.py` | `from motion.move import Move` mort si E2 corrigé | Supprimer |

### Priorité MOYENNE (qualité / mypy)

| # | Fichier | Problème | Action |
|---|---|---|---|
| **E3** | `transport/messages.py` | `List` (majuscule) non modernisé dans 3 annotations | Passer à `list[...]`, retirer `List` des imports |
| **E4** | `motion/move_queue.py` | `_make_streamer(axis_configs)` non typé | Ajouter `list[AxisMotionConfig]` |
| **E5** | `motion/move_queue.py` | `_wrap_segment_sequence(generator: Any)` | Typer `Iterator[MultiAxisSegment]` |
| **E6** | `motion/move_queue.py` | Triple `_ensure_homing_can_start` redondant | Simplifier en 2 lignes |

### Priorité FAIBLE (cosmétique / documentation)

| # | Fichier | Problème | Action |
|---|---|---|---|
| **E7** | `winding/wound_move.py` | Annotations nues sur `SynchronizedMove` | Ajouter commentaire explicite |
| **E8** | `motion/move_queue.py` | Sémantique différente entre `_execute_ramp_move` et `_execute_wound_move` pour `expected_delta_steps` | Décision de conception à documenter |

---

## Plan de commits

```
E1  fix(transport): repair MultiAxisSegmentBlockPayload.pack — remove directions check, fix indentation
E2  fix(winding): make AdaptiveWindingMove inherit from SynchronizedMove (complete C4)
D3  refactor(move_queue): extract _finalize_streamer_move, eliminate ~15-line duplication
D4  refactor(winding): export SynchronizedMove from winding/__init__.py
D5  refactor(winding): remove dead Move import from adaptive.py (follow-up from E2)
E3  refactor(transport): replace List[...] with list[...] in messages.py, remove typing.List
E4  refactor(move_queue): type axis_configs parameter of _make_streamer
E5  refactor(move_queue): type generator parameter of _wrap_segment_sequence
E6  refactor(move_queue): simplify triple _ensure_homing_can_start in _execute_composite
```

E1 doit être fait en premier — le système ne peut pas envoyer de mouvement dans son état actuel.  
E2 doit être confirmé avant D3 car il affecte la sémantique du dispatch.