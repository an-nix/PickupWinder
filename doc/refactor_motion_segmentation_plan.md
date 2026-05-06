# Plan de refactoring — Motion Segmentation

> Branche cible : `refactor/motion-segmentation`
> Date : 2026-05-06
> Statut : **Plan uniquement — aucun fichier modifié**

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
| `StepEntry` | Firmware : `ESP_ERR_NOT_SUPPORTED` pour `STEP_BLOCK` |
| `StepBlockPayload` | Idem |
| `_STEP_BLOCK_HEAD_STRUCT` | Utilisé uniquement par `StepBlockPayload` |
| `STEP_BLOCK_SIZE` | Idem |
| `MotionSegment` | Firmware : `ESP_ERR_NOT_SUPPORTED` pour `SEGMENT_BLOCK` |
| `SegmentBlockPayload` | Idem |
| `_SEGMENT_BLOCK_HEAD_STRUCT` | Utilisé uniquement par `SegmentBlockPayload` |
| `SEGMENT_BLOCK_SIZE` | Idem |
| `make_step_block()` | Aucun appelant actif |
| `make_segment_block()` | Idem |
| `SpiStepFlags` | Dernière référence dans `StepEntry.pack()` (supprimé) |

> `SpiMessageType.STEP_BLOCK` et `SpiMessageType.SEGMENT_BLOCK` sont **conservés** comme constantes de documentation du protocole fil.

### `src/rpi/transport/spi_transport.py`

Méthodes à supprimer :

```
send_step_block()
send_step_block_request()
send_step_block_with_backpressure()
send_segment_block()
send_segment_block_request()
```

Imports morts à retirer : `StepBlockPayload`, `SegmentBlockPayload`, `make_step_block`, `make_segment_block`.

### `src/rpi/transport/__init__.py`

Retirer des imports et de `__dir__` :
`StepEntry`, `MotionSegment`, `StepBlockPayload`, `SegmentBlockPayload`,
`make_step_block`, `make_segment_block`, `STEP_BLOCK_SIZE`, `SEGMENT_BLOCK_SIZE`, `SpiStepFlags`.

### `src/rpi/motion/move.py`

Supprimer la classe `JogMove` entière.

Les deux sites d'appel sont remplacés par une construction `RampMove` équivalente (voir §6, commit 5).

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
    segment.direction_mask,   # déjà un masque — plus de boucle
)
```

### 2b. `segment_generator.py` — déplacer le calcul du masque en amont

`_compute_segment()` retourne actuellement `(list[int], list[int])` (steps, directions).

**Après :** retourne `(list[int], int)` (steps, direction_mask).

```python
def _compute_segment(self, time_start, time_end):
    steps = [0] * len(self.axis_profiles)
    direction_mask = 0
    for i, profile in enumerate(self.axis_profiles):
        # ... calcul du delta de pas existant ...
        is_negative = count < 0
        if is_negative:
            count = abs(count)
        if is_negative ^ profile.reverse_direction:
            direction_mask |= (1 << i)
        steps[i] = count
    return steps, direction_mask
```

`SynchronizedSegmentGenerator` (utilisé par `WoundMove`) : même changement.

### 2c. `transport/__init__.py` — nettoyer les re-exports

### 2d. `transport/streamer.py` — retirer l'import inutile de `AxisMotionConfig`/`RampConfig`

Ces imports ne sont plus nécessaires après que `StreamAxisConfig` n'expose plus de champ `ramp` à des appelants externes.

### 2e. `motion/move.py` — documenter le statut interne de `RampMove`

Ajouter un commentaire de module :

```python
# RampMove, RampMoveConfig et AxisMotionConfig sont conservés intentionnellement
# comme infrastructure interne de HomingMove.
# Ils NE font PAS partie de l'API publique de mouvement.
```

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

**Ce qui disparaît :** `StepBlockPayload`, `SegmentBlockPayload`, `JogMove`, toutes les fonctions d'envoi legacy, le tableau `directions` par axe.

---

## 5. Plan de migration — étape par étape

### Commit 1 — Supprimer le code step-based legacy - FAIT
**Fichiers :** `transport/messages.py`, `transport/spi_transport.py`, `transport/__init__.py`

1. Supprimer `StepEntry`, `StepBlockPayload`, `_STEP_BLOCK_HEAD_STRUCT`, `STEP_BLOCK_SIZE`, `SpiStepFlags`
2. Supprimer `make_step_block()`
3. Supprimer `send_step_block`, `send_step_block_request`, `send_step_block_with_backpressure`
4. Retirer les imports morts
5. Nettoyer `transport/__init__.py`

**Vérification :** `grep -rn StepBlockPayload src/rpi/` → zéro résultat hors commentaires.

---

### Commit 2 — Supprimer l'abstraction mono-axe SEGMENT_BLOCK - FAIT
**Fichiers :** mêmes fichiers

1. Supprimer `MotionSegment`, `SegmentBlockPayload`, `_SEGMENT_BLOCK_HEAD_STRUCT`, `SEGMENT_BLOCK_SIZE`
2. Supprimer `make_segment_block()`
3. Supprimer `send_segment_block`, `send_segment_block_request`
4. Nettoyer `transport/__init__.py`

---

### Commit 3 — Remplacer `MultiAxisSegment.directions` par `direction_mask`
**Fichiers :** `transport/messages.py`, `motion/segment_generator.py`, `winding/synchronized_segment_generator.py`

1. Remplacer `directions: List[int]` par `direction_mask: int` dans `MultiAxisSegment`
2. Simplifier `MultiAxisSegmentBlockPayload.pack()` — supprimer la boucle de direction
3. Mettre à jour `BaseSegmentGenerator.__iter__` : passer `direction_mask=` au lieu de `directions=`
4. Mettre à jour `StepProfileSegmentGenerator._compute_segment()` → retourne `(list[int], int)`
5. Mettre à jour `SynchronizedSegmentGenerator._compute_segment()` de même

**Test d'invariant :** pour un axe unique en sens inverse à l'index 1, `direction_mask == 0b10 == 2`.

---

### Commit 4 — Introduire le protocole `SegmentProducer`
**Fichiers :** `motion/segment_producer.py` (nouveau), `motion/__init__.py`

1. Créer `motion/segment_producer.py` avec le protocole `SegmentProducer`
2. Ajouter `SegmentProducer` dans `motion/__init__.py __all__`
3. Annoter `MultiAxisRampStreamer.set_generator()` avec `SegmentProducer | Iterator[MultiAxisSegment]`

---

### Commit 5 — Supprimer `JogMove`, inliner aux sites d'appel
**Fichiers :** `motion/move.py`, `core/command_service.py`, `winding/service.py`

Remplacer dans `core/command_service.py` (~ligne 141) :

```python
# Avant
move = JogMove(
    name="jog",
    axis_id=axis_id,
    steps_per_rev=steps_per_rev,
    steps=steps,
    rpm=rpm,
    reverse_direction=reverse_direction,
)

# Après
total_s = (steps / float(steps_per_rev)) / (rpm / 60.0)
move = RampMove(
    name="jog",
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
                    reverse_direction=reverse_direction,
                ),
            )
        ],
    ),
)
```

Appliquer le même remplacement dans `winding/service.py:499`. Supprimer `JogMove` de `move.py`.

---

### Commit 6 — Nettoyage et documentation
**Fichiers :** `motion/move.py`, `transport/streamer.py`, `motion/__init__.py`, `README.md`, `doc/architecture.md`, `doc/stepper_engine.md`, `doc/spi_protocol.md`

1. Ajouter le commentaire de gel homing dans `move.py`
2. Retirer l'import mort `AxisMotionConfig`/`RampConfig` dans `transport/streamer.py`
3. Mettre à jour la documentation (voir §6 ci-dessous)

---

## 6. Checklist documentation

| Fichier | Changement requis |
|---|---|
| `README.md` | Mettre à jour le pipeline de mouvement ; supprimer les mentions de step-block et segment-block ; noter l'isolement du homing |
| `doc/architecture.md` | Redessiner la section pipeline mouvement ; documenter `SegmentProducer` ; ajouter l'encadré « Homing isolation » |
| `doc/spi_protocol.md` | Marquer `STEP_BLOCK (0x10)` et `SEGMENT_BLOCK (0x11)` comme **types legacy rejetés par le firmware** (`ESP_ERR_NOT_SUPPORTED`) |
| `doc/stepper_engine.md` | Documenter `MultiAxisSegment.direction_mask` remplaçant `directions` ; documenter le contrat `SegmentProducer` |
| `doc/sequencing.md` | Pas de changement requis sauf si les docs de compteur de séquence référencent le chemin legacy |
| Docstrings inline | `Move.segments()`, `MultiAxisRampStreamer.set_generator()`, `BaseSegmentGenerator.__iter__()` |

Ajouter explicitement dans `doc/architecture.md` :

> **Politique d'isolement du homing :** `HomingMove` et son infrastructure interne (`RampMove`, `RampMoveConfig`, `AxisMotionConfig`) sont intentionnellement non refactorisés. Ils sont corrects, testés et gelés. Toute modification nécessite une tâche spécifique au homing.

---

## 7. Ce qui n'est PAS touché

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

## 8. Workflow Git

```bash
git checkout -b refactor/motion-segmentation
```

| # | Message de commit | Périmètre |
|---|---|---|
| 1 | `refactor(transport): remove legacy STEP_BLOCK payload and send functions` | `messages.py`, `spi_transport.py`, `__init__.py` |
| 2 | `refactor(transport): remove mono-axis SEGMENT_BLOCK payload and send functions` | Mêmes fichiers |
| 3 | `refactor(motion): replace MultiAxisSegment.directions with direction_mask` | `messages.py`, `segment_generator.py`, `synchronized_segment_generator.py` |
| 4 | `feat(motion): introduce SegmentProducer structural protocol` | `motion/segment_producer.py` (nouveau), `motion/__init__.py` |
| 5 | `refactor(motion): delete JogMove, inline at call sites` | `move.py`, `command_service.py`, `winding/service.py` |
| 6 | `docs+cleanup: align docs with new motion pipeline, annotate homing isolation` | `README.md`, docs, docstrings |

Chaque commit doit compiler et passer `mypy --strict` avant le suivant.
