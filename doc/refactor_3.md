# Plan de refactoring — Nettoyage post-motion-segmentation + architecture homing

> Date : 2026-05-06
> Prérequis : `refactor_2.md` entièrement appliqué
> Statut : **Parties A, B et C terminées ✅ — Partie D documentée, non implémentée**

---

## Contexte

Suite au refactoring `refactor_2.md` (suppression de `JogMove`, `StepBlockPayload`, `SegmentBlockPayload`, remplacement de `directions` par `direction_mask`, introduction de `SegmentProducer`), plusieurs opportunités de nettoyage subsistent.

Ce document est en deux parties :
- **Partie A** : Nettoyage de surface (types, duplications, imports morts)
- **Partie B** : Refactoring architectural — homing et `MoveQueue`

---

# Partie A — Nettoyage de surface

---

## 1. Annotation de type erronée dans `segment_generator.py` — CORRIGÉE

**Statut : Corrigée dans la même session que refactor_2.**

`StepProfileSegmentGenerator._compute_segment()` utilisait encore `Tuple[list[int], int]` (majuscule) alors que `Tuple` avait été retiré des imports. La signature a été mise à jour en `tuple[list[int], int]` (PEP 585).

---

## 2. `Optional` obsolète — moderniser les annotations

**Priorité : Moyenne**

### 2a. `motion/axis_state.py`

```python
# Avant (ligne 7)
from typing import Optional
...
min_steps: Optional[int] = None
max_steps: Optional[int] = None

# Après
# supprimer l'import Optional
min_steps: int | None = None
max_steps: int | None = None
```

### 2b. `core/config.py`

```python
# Avant (ligne 5)
from typing import Optional

# Après — supprimer l'import ; remplacer tous les Optional[X] par X | None
# Champs concernés (~10 occurrences) :
#   spi_ready_gpio_chip: Optional[str]
#   spi_ready_gpio_line: Optional[int]
#   spindle_max_acceleration_rpm: Optional[float]
#   spindle_max_deceleration_rpm: Optional[float]
#   lateral_max_acceleration_mm_per_s2: Optional[float]
#   lateral_max_deceleration_mm_per_s2: Optional[float]
#   lateral_steps_per_mm_override: Optional[float]
#   lateral_soft_limit_min_mm: Optional[float]
#   lateral_soft_limit_max_mm: Optional[float]
#   lateral_homing_backoff_steps: Optional[int]
#   lateral_soft_limit_min_steps() -> Optional[int]
#   lateral_soft_limit_max_steps() -> Optional[int]
```

**Vérification avant modification :**
```bash
grep -rn "Optional" src/rpi/
# → liste exhaustive des fichiers encore concernés
```

---

## 3. Duplication du patron de construction jog

**Priorité : Moyenne**

Le remplacement de `JogMove` a introduit une duplication : le même bloc de calcul `total_s → RampMoveConfig` apparaît dans deux fichiers :

- `core/command_service.py` méthode `jog()` (~lignes 140-160)
- `winding/service.py` méthode `_move_lateral_to()` (~lignes 507-525)

**Suggestion :** extraire une fonction libre dans `motion/move_builders.py` :

```python
# motion/move_builders.py (nouveau, ~25 lignes)
"""Fonctions utilitaires de construction de mouvements."""
from __future__ import annotations

from motion import AxisMotionConfig, RampConfig
from motion.move import RampMove, RampMoveConfig


def build_jog_move(
    *,
    name: str,
    axis_id: int,
    steps: int,
    steps_per_rev: int,
    rpm: float,
    reverse: bool = False,
) -> RampMove:
    """Construit un RampMove de type jog (durée calculée depuis steps/rpm)."""
    total_s = (steps / float(steps_per_rev)) / (rpm / 60.0)
    return RampMove(
        name=name,
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

Remplacer les deux blocs inline par des appels à `build_jog_move(...)`.

**Note :** ne créer ce fichier que si la duplication persiste ou si un troisième site d'appel apparaît. Pour deux sites, l'inline reste lisible.

---

## 4. `MultiAxisSegmentGenerator` dans `motion/__init__.py`

**Priorité : Faible**

`MultiAxisSegmentGenerator` est listé dans `__all__` de `motion/__init__.py` mais n'est plus utilisé par aucun consommateur externe — seul `RampMove.segments()` l'instancie directement (import interne via `from motion import MultiAxisSegmentGenerator`).

**Option A (conservative)** : retirer de `__all__` uniquement, garder le lazy import dans `__getattr__` pour compatibilité avec des tests éventuels.

**Option B (nettoyage complet)** : retirer de `__all__` et de `__getattr__`. Les imports internes (`from motion.multi_axis_segment_generator import MultiAxisSegmentGenerator`) continuent de fonctionner.

**Vérification avant action :**
```bash
grep -rn "from motion import.*MultiAxisSegmentGenerator\|import motion.MultiAxisSegmentGenerator" src/ tests/
# Si zéro résultat externe → appliquer Option B
```

---

## 5. `StreamAxisConfig.ramp: RampConfig` — couplage résiduel

**Priorité : Faible**

`StreamAxisConfig` a un champ `ramp: RampConfig`. Le streamer l'utilise uniquement pour extraire `config.ramp.target_hz` (lignes 220, 241). Cela force les appelants à construire un `RampConfig` complet même quand seul `target_hz` est nécessaire (cas `from_axis_ids`).

**Proposition :** ne toucher `StreamAxisConfig` que si un nouveau consommateur du streamer n'a pas de `RampConfig` disponible. En l'état actuel, tous les appelants ont un `RampConfig`, donc le couplage est acceptable.

---

## 6. Annotations de retour manquantes dans `transport/streamer.py`

**Priorité : Faible**

Quelques méthodes privées n'ont pas d'annotation de retour explicite. Cela ne pose pas de problème à l'exécution mais dégrade la couverture mypy :

| Méthode | Retour attendu |
|---|---|
| `_wait_for_request_result(sequence, send_status)` | `Any` (StatusPayload) |
| `flush_until(sequence)` | `Any` (StatusPayload) |
| `_wrap_segment_sequence(generator, start_sequence)` dans `move_queue.py` | `Iterator[MultiAxisSegment]` |

---

## 7. Documentation — mentions du pipeline legacy

**Priorité : Faible**

Les docs maintenues (`doc/stepper_engine.md`, `doc/spi_protocol.md`, `doc/architecture.md`) n'ont pas encore été mises à jour pour refléter :

- Suppression de `STEP_BLOCK (0x10)` et `SEGMENT_BLOCK (0x11)` du pipeline hôte (les deux types restent des constantes de protocole rejetées par le firmware avec `ESP_ERR_NOT_SUPPORTED`)
- `MultiAxisSegment.direction_mask` remplaçant `directions[]`
- Protocole `SegmentProducer`
- Isolement architectural du `HomingMove`

Ces mises à jour sont documentaires et ne bloquent aucune fonctionnalité.

---

## Tableau récapitulatif — Partie A

| # | Titre | Fichier(s) | Priorité | Statut |
|---|---|---|---|---|
| A1 | Correction annotation `Tuple` dans `segment_generator.py` | `motion/segment_generator.py` | **Corrigée** | ✅ FAIT |
| A2 | Moderniser `Optional` → `X \| None` | `motion/axis_state.py`, `core/config.py` | Moyenne | ✅ FAIT |
| A3 | Déduplication du patron jog | `core/command_service.py`, `winding/service.py`, `motion/move_builders.py` (nouveau) | Moyenne | ✅ FAIT |
| A4 | Retirer `MultiAxisSegmentGenerator` de `motion/__all__` | `motion/__init__.py` | Faible | ✅ FAIT |
| A5 | `StreamAxisConfig.ramp` couplage | `transport/streamer.py` | Faible | ⬜ À différer |
| A6 | Annotations de retour manquantes streamer/move_queue | `transport/streamer.py`, `motion/move_queue.py` | Faible | ✅ FAIT |
| A7 | Mise à jour docs legacy | `doc/stepper_engine.md`, `doc/spi_protocol.md`, `doc/architecture.md` | Faible | ✅ FAIT |

---

# Partie B — Refactoring architectural : Homing et MoveQueue

## Contexte architecture actuelle

Le homing souffre de trois problèmes structurels indépendants :

1. **Dispatch par nom de phase (string)** : `_execute_homing` en `move_queue.py` branche sur `phase_name in ("approach", "search")` et `phase_name == "backoff"`. La logique post-phase est codée en dur dans l'exécuteur, pas dans le descripteur de phase.
2. **`CompositeMove` non-générique** : `_execute_move` cast en `HomingMove` avec `# type: ignore` — tout futur `CompositeMove` exigerait un nouveau bras de dispatch.
3. **Duplication du patron `RampMove` dans `HomingMove`** : les trois `_make_*_move()` construisent chacun le même `RampMove(config=RampMoveConfig(axis_configs=[AxisMotionConfig(ramp=RampConfig(...))]))`. C'est le même patron verbeux que `JogMove` (supprimé en refactor_2).
4. **Endstop non-garanti disarmé** : le loop de phases contient 5+ `self._set_endstop_armed(arm=False)` séparés aux différents points de sortie. Un `try/finally` manque.

---

## B1 — Introduire `HomingPhaseDescriptor`

**Fichiers :** `motion/move.py`
**Priorité : Haute** (débloque B2 et B3)

Remplacer le tuple `(str, Move, bool)` retourné par `HomingMove.phases()` par un dataclass auto-documenté :

```python
@dataclass(slots=True)
class HomingPhaseDescriptor:
    """Descripteur d'une phase de homing.

    arm_endstop        : l'endstop doit être armé avant l'exécution de cette phase.
    expect_endstop_hit : la phase se termine nominalement par un déclenchement endstop
                         (approach, search). Si False, la phase se termine par
                         épuisement des segments (backoff).
    wait_for_open      : après l'exécution, attendre que l'endstop retourne OPEN
                         (backoff uniquement).
    """
    name: str
    move: RampMove
    arm_endstop: bool
    expect_endstop_hit: bool
    wait_for_open: bool
```

`HomingMove.phases()` devient :
```python
def phases(self) -> list[HomingPhaseDescriptor]:
    return [
        HomingPhaseDescriptor("approach", self._make_approach_move(), arm_endstop=True,  expect_endstop_hit=True,  wait_for_open=False),
        HomingPhaseDescriptor("backoff",  self._make_backoff_move(),  arm_endstop=False, expect_endstop_hit=False, wait_for_open=True),
        HomingPhaseDescriptor("search",   self._make_search_move(),   arm_endstop=True,  expect_endstop_hit=True,  wait_for_open=False),
    ]
```

`CompositeMove.phases()` est mis à jour pour retourner `list[HomingPhaseDescriptor]`.

**Impact dans `move_queue.py`** : le bras `if phase_name in ("approach", "search")` disparaît, remplacé par `if descriptor.expect_endstop_hit`.

---

## B2 — Rendre `_execute_composite` générique

**Fichier :** `motion/move_queue.py`
**Priorité : Haute** (dépend de B1)

Actuellement :
```python
# _execute_move — move_queue.py
if isinstance(move, CompositeMove):
    self._execute_homing(move)  # type: ignore[arg-type]
```

Le `type: ignore` révèle que `_execute_homing` accepte `CompositeMove` mais n'utilise que les attributs de `HomingMove` (`.axis_id`, `.home_position_steps`). Si `HomingPhaseDescriptor` est adopté, l'exécuteur peut être rendu générique :

```python
def _execute_composite(self, move: CompositeMove) -> None:
    """Execute any CompositeMove phase by phase via HomingPhaseDescriptor."""
    ...

# _execute_move
if isinstance(move, CompositeMove):
    self._execute_composite(move)   # plus de type: ignore
```

L'import direct de `HomingMove` dans `move_queue.py` peut être supprimé.

---

## B3 — `try/finally` pour l'endstop dans `_execute_homing`

**Fichier :** `motion/move_queue.py`
**Priorité : Haute** (indépendant de B1/B2, mais facile à faire en même temps)

Le loop actuel a 5 appels séparés `self._set_endstop_armed(move.axis_id, arm=False)` à chaque point de sortie anticipée. Une exception non catchée laisserait l'endstop armé.

**Avant (structure actuelle) :**
```python
for phase_name, sub_move, arm_endstop in move.phases():
    if self._stop_requested:
        self._set_endstop_armed(move.axis_id, arm=False)  # ← sortie 1
        ...
        return
    try:
        self._set_endstop_armed(...)
        ...
    except Exception as exc:
        self._set_endstop_armed(move.axis_id, arm=False)  # ← sortie 2
        move.mark_failed(...)
        return
    ...
    except RuntimeError as exc:
        self._set_endstop_armed(move.axis_id, arm=False)  # ← sortie 3
        ...
```

**Après :**
```python
try:
    for descriptor in move.phases():
        if self._stop_requested:
            ...
            return  # le finally gère le disarm

        self._set_endstop_armed(move.axis_id, arm=descriptor.arm_endstop)
        ...
        # Toutes les sorties anticipées utilisent return sans appel explicite
finally:
    self._set_endstop_armed(move.axis_id, arm=False)

if axis_state is not None:
    axis_state.mark_homed(move.home_position_steps)
move.mark_completed()
```

Cela réduit le corps de `_execute_homing` d'environ 30 %.

---

## B4 — Extraire `_make_phase_move()` dans `HomingMove`

**Fichier :** `motion/move.py`
**Priorité : Moyenne** (dépend optionnellement de A3)

Les trois méthodes `_make_approach_move()`, `_make_backoff_move()`, `_make_search_move()` partagent toutes la même structure :

```
total_s = steps / steps_per_rev / (rpm / 60)
RampMove(config=RampMoveConfig(axis_configs=[AxisMotionConfig(axis_id=..., ramp=RampConfig(
    axis_id=..., steps_per_rev=..., target_rpm=...,
    accel_s=..., cruise_s=..., decel_s=..., reverse_direction=...
))], segment_duration_s=...))
```

Seuls les valeurs de `rpm`, `total_steps`, `reverse_direction` et les coefficients accel/decel changent.

**Option A** : extraire un helper privé `_make_phase_move(name, steps, rpm, reverse, accel_coef, decel_coef)` dans `HomingMove`.

**Option B** : utiliser `build_jog_move()` de A3 si ce helper est créé (dépendance croisée `motion/move.py` → `motion/move_builders.py` — vérifier la circularité des imports avant d'adopter).

Les trois méthodes passent de ~15 lignes chacune (~45 lignes total) à ~3 lignes.

---

## B5 — Supprimer l'import direct de `HomingMove` dans `move_queue.py`

**Fichier :** `motion/move_queue.py`
**Priorité : Faible** (dépend de B1 + B2)

```python
# Avant
from motion.move import BaseMove, CompositeMove, HomingMove, Move

# Après (une fois _execute_composite est générique)
from motion.move import BaseMove, CompositeMove, Move
```

`HomingMove` n'est plus référencé nulle part dans `move_queue.py`, ce qui casse la dépendance directe entre l'exécuteur générique et la classe de mouvement métier.

---

## B6 — Renommer `HomingMove.phases()` → retour `list[HomingPhaseDescriptor]`

**Fichier :** `motion/move.py`, `CompositeMove` ABC
**Priorité : Faible** (dépend de B1)

La signature abstraite de `CompositeMove.phases()` retourne actuellement `list[tuple[str, "Move", bool]]`. Mettre à jour pour retourner `list[HomingPhaseDescriptor]` dans la déclaration abstraite rend le contrat explicite dans l'ABC.

---

## Plan de migration Partie B

| # | Message de commit | Périmètre | Dépendances | Statut |
|---|---|---|---|---|
| B1 | `refactor(motion): introduce HomingPhaseDescriptor, replace tuple phases()` | `motion/move.py` | — | ✅ FAIT |
| B2+B3 | `refactor(move_queue): generic _execute_composite, try/finally endstop safety` | `motion/move_queue.py` | B1 | ✅ FAIT |
| B4 | `refactor(motion): extract _make_phase_move helper in HomingMove` | `motion/move.py` | B1 (ou A3) | ✅ FAIT |
| B5 | `refactor(move_queue): remove direct HomingMove import` | `motion/move_queue.py` | B1+B2 | ✅ FAIT |

---

## Architecture cible après Partie B

```
HomingMove
  └─ phases() → list[HomingPhaseDescriptor]
       HomingPhaseDescriptor
         ├─ name: str
         ├─ move: RampMove          ← construit par _make_phase_move()
         ├─ arm_endstop: bool
         ├─ expect_endstop_hit: bool
         └─ wait_for_open: bool

MoveQueue._execute_composite(move: CompositeMove)
  └─ for descriptor in move.phases():
       ├─ arm/disarm via descriptor.arm_endstop
       ├─ stream via descriptor.move.segments()
       ├─ check hit via descriptor.expect_endstop_hit
       └─ wait open via descriptor.wait_for_open
  finally:
       └─ _set_endstop_armed(arm=False)   ← toujours exécuté
```

**Ce qui disparaît :**
- `if phase_name in ("approach", "search")`
- `if phase_name == "backoff"`
- Les 5 `_set_endstop_armed(arm=False)` redondants
- `# type: ignore[arg-type]` dans `_execute_move`
- `HomingMove` dans les imports de `move_queue.py`

---

# Partie C — Suivi post-B : contrat `CompositeMove`, nettoyage des shims

## Contexte

La Partie B a atteint ses objectifs : `HomingPhaseDescriptor`, `_execute_composite` générique, `try/finally` endstop, helper `_make_phase_move`. L'analyse post-implémentation révèle quatre problèmes résiduels à corriger, plus deux nettoyages de surface.

---

## C1 — `CompositeMove` ABC manque `axis_id` et `home_position_steps`

**Fichier :** `motion/move.py`
**Priorité : Haute** (correction de régression de B2)

`_execute_composite` accède à `move.axis_id` et `move.home_position_steps` sur un paramètre typé `CompositeMove`, mais ces attributs ne sont pas déclarés dans l'ABC. La généralisation de B2 est donc incomplète : le type checker accepte `CompositeMove` mais l'exécuteur échouerait silencieusement sur toute sous-classe sans ces attributs.

**Avant (`CompositeMove` ABC) :**
```python
class CompositeMove(BaseMove, ABC):
    @abstractmethod
    def phases(self) -> "list[HomingPhaseDescriptor]": ...
```

**Après :**
```python
class CompositeMove(BaseMove, ABC):

    @property
    @abstractmethod
    def axis_id(self) -> int:
        """Single axis driven by this composite move."""
        ...

    @property
    @abstractmethod
    def home_position_steps(self) -> int:
        """Position (steps) to set after all phases complete."""
        ...

    @abstractmethod
    def phases(self) -> "list[HomingPhaseDescriptor]": ...
```

`HomingMove` expose déjà `self.axis_id` et `self.home_position_steps` comme attributs d'instance — ajouter `@property` wrappers ou déclarer les champs dans le `__init__` de l'ABC comme stub satisfait le contrat sans casser l'existant.

Les helpers `_check_armed_phase_result`, `_stream_homing_sub_move`, `_clear_closed_endstop_before_homing` dans `move_queue.py` peuvent alors être retypés de `move: Any` → `move: CompositeMove`.

---

## C2 — Simplifier `_compute_backoff_timeout` : abandonner les fallbacks morts

**Fichier :** `motion/move_queue.py`
**Priorité : Haute** (dépend de C1)

`_compute_backoff_timeout(sub_move: Move)` contient une chaîne de 4 priorités conçue pour tolérer des types de sous-mouvements hétérogènes. Depuis B1, `descriptor.move` est toujours un `RampMove` qui expose `axis_configs[0].ramp.total_duration` — les priorités 1, 3 et 4 sont du code mort dans le chemin homing.

**Avant (55 lignes, 4 branches) :**
```python
@staticmethod
def _compute_backoff_timeout(sub_move: Move, margin: float = 1.5) -> float:
    recovery_guard_s = 0.5
    estimated = getattr(sub_move, "estimated_duration_s", None)  # mort
    ...
    ramp = getattr(sub_move, "ramp", None)                        # indirect
    ...
    steps = getattr(sub_move, "total_steps", None)                # mort
    ...
    return max(2.0, ...)                                           # jamais atteint
```

**Après (~10 lignes) :**
```python
@staticmethod
def _compute_backoff_timeout(sub_move: RampMove, margin: float = 1.5) -> float:
    ramp = sub_move.axis_configs[0].ramp
    total_s = ramp.total_duration
    return max(1.0, total_s * margin + 0.5)
```

Signature mise à jour : `sub_move: RampMove` (import de `motion.move`).

---

## C3 — Mettre à jour la bannière de gel en tête de `move.py`

**Fichier :** `motion/move.py`
**Priorité : Moyenne** (cohérence documentaire)

La bannière en tête de fichier cite `_make_approach_move()`, `_make_backoff_move()`, `_make_search_move()` comme "correctes, testées et gelées". Ces méthodes existent toujours mais ont été refactorisées en B4 : elles délèguent désormais à `_make_phase_move()`. La bannière doit être mise à jour pour :

1. Retirer la mention des trois méthodes individuelles comme "gelées"
2. Mentionner `_make_phase_move` comme point d'entrée unique
3. Maintenir la politique d'isolement : toute modification du comportement de homing requiert une tâche dédiée

---

## C4 — Supprimer le dispatch duck-type `is_synchronized_move`

**Fichier :** `motion/move_queue.py`, `winding/adaptive.py`
**Priorité : Moyenne**

Dans `_execute_move` :
```python
elif getattr(move, "is_synchronized_move", False):
    self._execute_wound_move(move)  # type: ignore[arg-type]
```

`AdaptiveWindingMove` hérite de `Move` (pas de `WoundMove`) et expose un attribut de classe `is_synchronized_move = True` pour contourner le dispatch. C'est le seul `type: ignore` restant dans `_execute_move`.

**Option A (conservatrice)** : faire hériter `AdaptiveWindingMove` de `WoundMove`. Vérifier les attributs requis par `_execute_wound_move` (`axis_ids`, `kinematics`, `spindle_cfg`, `segment_duration_s`) — `AdaptiveWindingMove` les expose déjà. Supprimer `is_synchronized_move`.

**Option B** : introduire un ABC `SynchronizedMove(Move)` dont héritent `WoundMove` et `AdaptiveWindingMove`, avec `isinstance(move, SynchronizedMove)` dans le dispatch.

Option A est préférable si `AdaptiveWindingMove` satisfait déjà l'interface de `WoundMove`. À vérifier avec `grep -n "def " winding/wound_move.py` avant d'appliquer.

---

## C5 — Supprimer la constante morte `_ENDSTOP_VERIFY_TIMEOUT_S`

**Fichier :** `motion/move_queue.py` ligne 25
**Priorité : Faible**

```python
_ENDSTOP_VERIFY_TIMEOUT_S = 0.5
```

Définie mais jamais utilisée dans le module. Vestige d'une version antérieure du protocole d'armement. Supprimer.

---

## C6 — Supprimer les shims de compatibilité `try/except TypeError`

**Fichier :** `motion/move_queue.py`
**Priorité : Faible** (après vérification que le transport est stable)

Trois méthodes contiennent des blocs `try/except TypeError` pour gérer d'anciens prototypes du transport :

| Méthode | Argument optionnel protégé |
|---|---|
| `_read_status` | `allow_stale=` |
| `_wait_for_transport_request_result` | `hint_status=` |
| `_make_streamer` | `initial_segments_dropped=` |

Si `src/rpi/transport/spi_transport.py` expose ces paramètres de façon stable, les trois blocs try/except peuvent être remplacés par des appels directs.

**Vérification avant action :**
```bash
grep -n "def get_status\|def wait_for_request_result" src/rpi/transport/spi_transport.py
grep -n "initial_segments_dropped\|allow_stale\|hint_status" src/rpi/transport/spi_transport.py
```

---

## Plan de migration Partie C

| # | Message de commit | Périmètre | Dépendances | Statut |
|---|---|---|---|---|
| C1 | `refactor(motion): add axis_id/home_position_steps to CompositeMove ABC` | `motion/move.py`, `motion/move_queue.py` | — | ✅ FAIT |
| C2 | `refactor(move_queue): simplify _compute_backoff_timeout for RampMove` | `motion/move_queue.py` | C1 | ✅ FAIT |
| C3 | `docs(motion): update freeze banner in move.py` | `motion/move.py` | — | ✅ FAIT |
| C4 | `refactor(winding): remove is_synchronized_move duck-type dispatch` | `winding/adaptive.py`, `motion/move_queue.py` | — | ✅ FAIT |
| C5 | `refactor(move_queue): remove unused _ENDSTOP_VERIFY_TIMEOUT_S` | `motion/move_queue.py` | — | ✅ FAIT |
| C6 | `refactor(move_queue): remove try/except TypeError transport shims` | `motion/move_queue.py` | transport stable | ✅ FAIT |

---

## Architecture cible après Partie C

```
CompositeMove (ABC)
  ├─ axis_id: int           (abstract property)
  ├─ home_position_steps: int (abstract property)
  └─ phases() → list[HomingPhaseDescriptor]

HomingMove(CompositeMove)
  ├─ axis_id (instance attr, satisfies ABC)
  ├─ home_position_steps (instance attr, satisfies ABC)
  └─ phases() → [approach, backoff, search]

MoveQueue._execute_composite(move: CompositeMove)
  ├─ move.axis_id        ← garanti par l'ABC
  ├─ move.home_position_steps ← garanti par l'ABC
  └─ for descriptor in move.phases(): …  (RampMove, pas Any)

MoveQueue._compute_backoff_timeout(sub_move: RampMove) → float
  └─ sub_move.axis_configs[0].ramp.total_duration * margin + guard
```

**Ce qui disparaît :**
- `move: Any` dans `_stream_homing_sub_move`, `_check_armed_phase_result`, `_clear_closed_endstop_before_homing`
- La chaîne `getattr` à 4 priorités dans `_compute_backoff_timeout`
- `getattr(move, "is_synchronized_move", False)` et le `type: ignore` associé
- `_ENDSTOP_VERIFY_TIMEOUT_S`
- Les 3 blocs `try/except TypeError` de compatibilité transport

---

# Partie D — Nettoyage résiduel post-C : shims engine, typage fort, DRY exécuteurs

## Contexte

Après les Parties A, B et C, les couches `move.py`, `wound_move.py`, `adaptive.py` sont propres. Les problèmes résiduels identifiés concernent principalement `core/engine.py` (shims toujours présents), deux quasi-doublons dans `move_queue.py`, le typage faible de `_clear_closed_endstop_before_homing`, et une opportunité d'export dans `winding/__init__.py`.

---

## D1 — Supprimer les shims de compatibilité dans `core/engine.py`

**Fichier :** `core/engine.py`
**Priorité : Haute**

Deux shims résiduels dans `WindingEngine` :

### D1a — `try/except TypeError` dans `request_stop`

```python
# Avant
try:
    self._move_queue.clear(stop_plan=effective_stop_plan)
except TypeError as exc:
    if "stop_plan" not in str(exc):
        raise
    self._move_queue.clear()
```

`MoveQueue.clear(stop_plan: MotionStopPlan | None = None)` accepte `stop_plan` depuis la Partie A. Le bloc `except` est du code mort.

**Après :**
```python
self._move_queue.clear(stop_plan=effective_stop_plan)
```

### D1b — `hasattr` guard dans `status()`

```python
# Avant
move_queue_status = (
    self._move_queue.status()
    if hasattr(self._move_queue, "status")
    else { ... dict de fallback ... }
)
```

`MoveQueue.status()` est définie (ligne 143 de `move_queue.py`) — le fallback ne peut pas être atteint. La branche `else` est du code mort.

**Après :**
```python
move_queue_status = self._move_queue.status()
```

---

## D2 — Typer `_clear_closed_endstop_before_homing` avec `CompositeMove`

**Fichier :** `motion/move_queue.py`
**Priorité : Haute** (dépend de C1, déjà fait)

`_clear_closed_endstop_before_homing(self, move: Any)` est le seul endroit dans `move_queue.py` où `move` est encore typé `Any`. La méthode accède à :
- `move.axis_id` — garanti par `CompositeMove` depuis C1 ✅
- `move._make_backoff_move()` — méthode privée de `HomingMove`, pas dans l'ABC

**Option A — Ajouter `preclear_move() -> RampMove` à `CompositeMove`** (propriété ou méthode) :
```python
class CompositeMove(BaseMove, ABC):
    ...
    @abstractmethod
    def preclear_move(self) -> RampMove:
        """Return the move to execute if the endstop is found closed at homing start."""
        ...
```
`HomingMove.preclear_move()` délègue à `_make_backoff_move()`. La méthode privée reste, l'interface publique est propre.

**Option B — Typer directement `move: HomingMove`** :
Réintroduit l'import de `HomingMove` dans `move_queue.py` (supprimé en B5). Recrée le couplage que B5 visait à briser. Non recommandé.

**Option A préférable.** Impact : `_clear_closed_endstop_before_homing(move: CompositeMove)`, signature nette, zéro `Any`.

---

## D3 — DRY : factoriser la conclusion commune de `_execute_ramp_move` et `_execute_wound_move`

**Fichier :** `motion/move_queue.py`
**Priorité : Moyenne**

Les deux exécuteurs se terminent par la même séquence :

```python
# Dans _execute_ramp_move ET _execute_wound_move :
if streamer.endstop_triggered:
    for ax_id in axis_ids:
        if ax_id in self._axis_states:
            self._axis_states[ax_id].invalidate_position()
    move.mark_aborted("endstop triggered", by_endstop=True)
    return

if self._stop_requested or streamer.has_stop_been_requested():
    stop_plan = self._active_stop_plan or self._default_stop_plan(axis_ids, "stop requested")
    self._apply_stop_plan(axis_ids, stop_plan)
    move.mark_aborted(f"{stop_plan.mode.value} requested")
    return

for ax_id in axis_ids:
    delta = move.expected_delta_steps(ax_id)
    if delta is not None and ax_id in self._axis_states:
        self._axis_states[ax_id].advance_position(delta)

move.mark_completed()
```

Ce bloc de ~15 lignes est dupliqué à l'identique. Extraire en :

```python
def _finalize_streamer_move(
    self,
    move: Move,
    streamer: MultiAxisRampStreamer,
    axis_ids: list[int],
) -> None:
    """Handle post-stream outcome: endstop, stop request, or completion."""
    if streamer.endstop_triggered:
        for ax_id in axis_ids:
            if ax_id in self._axis_states:
                self._axis_states[ax_id].invalidate_position()
        move.mark_aborted("endstop triggered", by_endstop=True)
        return
    if self._stop_requested or streamer.has_stop_been_requested():
        stop_plan = self._active_stop_plan or self._default_stop_plan(axis_ids, "stop requested")
        self._apply_stop_plan(axis_ids, stop_plan)
        move.mark_aborted(f"{stop_plan.mode.value} requested")
        return
    for ax_id in axis_ids:
        delta = move.expected_delta_steps(ax_id)
        if delta is not None and ax_id in self._axis_states:
            self._axis_states[ax_id].advance_position(delta)
    move.mark_completed()
```

Les deux exécuteurs remplacent leur conclusion par `self._finalize_streamer_move(move, streamer, axis_ids)`.

---

## D4 — Exporter `SynchronizedMove` depuis `winding/__init__.py`

**Fichier :** `winding/__init__.py`
**Priorité : Faible**

`SynchronizedMove` est maintenant la classe de base des deux moves synchronisés (`WoundMove`, `AdaptiveWindingMove`). Elle est définie dans `winding/wound_move.py` mais non exportée depuis `winding/__init__.py`. Les consommateurs qui doivent faire un `isinstance(move, SynchronizedMove)` doivent importer directement depuis le module interne.

Ajouter à `winding/__init__.py` :
```python
from winding.wound_move import SynchronizedMove, WoundMove
```

---

## D5 — Supprimer `import Move` mort dans `winding/adaptive.py`

**Fichier :** `winding/adaptive.py`
**Priorité : Faible**

Depuis C4, `AdaptiveWindingMove` hérite de `SynchronizedMove` (pas de `Move` directement). Si `from motion.move import Move` n'est utilisé nulle part ailleurs dans le module, l'import est mort.

**Vérification :** `grep -n "Move" winding/adaptive.py` — si `Move` n'apparaît que dans l'import et non dans les annotations ou le corps, supprimer.

---

## Plan de migration Partie D

| # | Message de commit | Périmètre | Dépendances | Statut |
|---|---|---|---|---|
| D1a | `refactor(engine): remove dead try/except TypeError in request_stop` | `core/engine.py` | — | ✅  |
| D1b | `refactor(engine): remove hasattr guard in status()` | `core/engine.py` | — | ✅ |
| D2 | `refactor(move_queue): add preclear_move() to CompositeMove, retype _clear_closed_endstop` | `motion/move.py`, `motion/move_queue.py` | C1 ✅ | ✅ |
| D3 | `refactor(move_queue): extract _finalize_streamer_move, eliminate duplication` | `motion/move_queue.py` | — | ✅ |
| D4 | `refactor(winding): export SynchronizedMove from winding/__init__.py` | `winding/__init__.py` | C4 ✅ | ✅ |
| D5 | `refactor(winding): remove dead Move import in adaptive.py` | `winding/adaptive.py` | C4 ✅ | ✅ |

---

## Architecture cible après Partie D

```
CompositeMove (ABC)
  ├─ axis_id: int
  ├─ home_position_steps: int
  ├─ preclear_move() → RampMove      ← nouveau (D2)
  └─ phases() → list[HomingPhaseDescriptor]

HomingMove(CompositeMove)
  └─ preclear_move() → _make_backoff_move()

MoveQueue
  ├─ _clear_closed_endstop_before_homing(move: CompositeMove)  ← plus de Any (D2)
  ├─ _execute_ramp_move(move: Move)
  │    └─ _finalize_streamer_move(...)   ← factorisé (D3)
  └─ _execute_wound_move(move: SynchronizedMove)
       └─ _finalize_streamer_move(...)   ← factorisé (D3)

winding/__init__.py
  └─ exports: AdaptiveWindingMove, SynchronizedMove, WoundMove  ← D4
```

**Ce qui disparaît :**
- `try/except TypeError` dans `engine.py` (D1a)
- `hasattr(self._move_queue, "status")` fallback dict de 10 lignes (D1b)
- `move: Any` dans `_clear_closed_endstop_before_homing` (D2)
- ~15 lignes dupliquées entre `_execute_ramp_move` et `_execute_wound_move` (D3)
- Import mort `from motion.move import Move` dans `adaptive.py` (D5)

