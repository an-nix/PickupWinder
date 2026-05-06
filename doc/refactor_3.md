# Plan de refactoring — Nettoyage post-motion-segmentation + architecture homing

> Date : 2026-05-06
> Prérequis : `refactor_2.md` entièrement appliqué
> Statut : **Partie A terminée ✅ — Partie B documentée, non implémentée**

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
| B1 | `refactor(motion): introduce HomingPhaseDescriptor, replace tuple phases()` | `motion/move.py` | — | ⬜ À faire |
| B2+B3 | `refactor(move_queue): generic _execute_composite, try/finally endstop safety` | `motion/move_queue.py` | B1 | ⬜ À faire |
| B4 | `refactor(motion): extract _make_phase_move helper in HomingMove` | `motion/move.py` | B1 (ou A3) | ⬜ À faire |
| B5 | `refactor(move_queue): remove direct HomingMove import` | `motion/move_queue.py` | B1+B2 | ⬜ À faire |

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
