# Plan — Positionnement post-homing de l'axe latéral

> Date : 2026-05-08
> Prérequis : refactor_3.md entièrement appliqué

---

## Sémantique des deux paramètres

```
position_depart_mm = lateral_soft_limit_min_mm + lateral_axis_offset_mm
```

| Paramètre | Type | Persisté | Modifiable à chaud |
|---|---|---|---|
| `lateral_soft_limit_min_mm` | config JSON | ✅ | ❌ Nécessite re-homer |
| `lateral_axis_offset_mm` | config JSON | ✅ | ✅ Via RPC `winding.set_axis_offset` |

Les deux vivent dans `AppConfiguration` et sont sérialisés/chargés automatiquement
par `ConfigurationManager` (qui utilise `asdict()` / `AppConfiguration(**payload)`).

`lateral_axis_offset_mm` peut être modifié via RPC **sans re-homer** : le nouveau
offset est persisté en config, et un `winding.move_to_start_position()` peut être
appelé immédiatement pour se repositionner sans refaire le homing complet.

---

## Séquence complète après homing

```
[endstop contact]
      │
      ▼
mark_homed(position_steps=0)          ← MoveQueue, fin de HomingMove
      │
      ▼
LateralAxisController.home()          ← détecte COMPLETED, appelle move_to_start_position()
      │
      ▼
move_to_start_position()
  target_mm = soft_limit_min + axis_offset
  delta_steps = target_mm * steps_per_mm - 0
  → build_jog_move(delta_steps) → wait_until_idle
      │
      ▼
EventKind.HOMING_COMPLETED(position_mm=target_mm)
      │
      ▼
EngineState → IDLE
```

---

## 1. `AppConfiguration` — ajouter `lateral_axis_offset_mm`

**Fichier :** `core/config.py`

### 1a. Champ dans le dataclass

Ajouter après `lateral_homing_backoff_steps` :

```python
# Winding start position offset applied on top of lateral_soft_limit_min_mm.
# Defines where the axis parks after homing and before winding starts.
# Modifiable at runtime via RPC without re-homing.
# Can be negative (start before soft_limit_min) or positive (start after).
# Constraints:
#   soft_limit_min_mm + axis_offset_mm >= soft_limit_min_mm (or unbounded if None)
#   soft_limit_min_mm + axis_offset_mm <= soft_limit_max_mm (if set)
lateral_axis_offset_mm: float = 0.0
```

### 1b. Validation dans `__post_init__`

Ajouter après la validation de `lateral_soft_limit_min/max` :

```python
# start_position must stay within [soft_limit_min, soft_limit_max].
start_mm = (self.lateral_soft_limit_min_mm or 0.0) + self.lateral_axis_offset_mm

if (
    self.lateral_soft_limit_min_mm is not None
    and start_mm < self.lateral_soft_limit_min_mm
):
    raise ValueError(
        f"lateral_soft_limit_min_mm ({self.lateral_soft_limit_min_mm}) "
        f"+ lateral_axis_offset_mm ({self.lateral_axis_offset_mm}) "
        f"= {start_mm:.3f} mm is below lateral_soft_limit_min_mm "
        f"({self.lateral_soft_limit_min_mm})"
    )

if (
    self.lateral_soft_limit_max_mm is not None
    and start_mm > self.lateral_soft_limit_max_mm
):
    raise ValueError(
        f"lateral_soft_limit_min_mm ({self.lateral_soft_limit_min_mm}) "
        f"+ lateral_axis_offset_mm ({self.lateral_axis_offset_mm}) "
        f"= {start_mm:.3f} mm exceeds lateral_soft_limit_max_mm "
        f"({self.lateral_soft_limit_max_mm})"
    )
```

### 1c. Propriétés dérivées

```python
@property
def lateral_start_position_mm(self) -> float:
    """Winding start position = soft_limit_min_mm + axis_offset_mm."""
    return (self.lateral_soft_limit_min_mm or 0.0) + self.lateral_axis_offset_mm

@property
def lateral_start_position_steps(self) -> int:
    """Winding start position converted to steps."""
    return int(round(self.lateral_start_position_mm * self.lateral_steps_per_mm))
```

> **Persistance automatique :** `ConfigurationManager.save_configuration()` appelle
> `json.dump(asdict(config), ...)`. `lateral_axis_offset_mm` étant un champ du
> dataclass, il est sérialisé et rechargé sans aucun code supplémentaire dans
> `ConfigurationManager`.

---

## 2. `LateralAxisController` — `move_to_start_position()` et refactor de `home()`

**Fichier :** `core/lateral.py`

### 2a. Méthode `move_to_start_position()`

```python
def move_to_start_position(self) -> None:
    """Move the lateral axis to the configured winding start position.

    start_position = lateral_soft_limit_min_mm + lateral_axis_offset_mm

    Called automatically at the end of home(). Can also be called
    explicitly via RPC after set_axis_offset() to reposition without
    re-homing.

    Publishes EventKind.HOMING_COMPLETED when the move completes
    (or immediately if delta_steps == 0).

    Raises:
        RuntimeError: if the axis is not homed or the move fails.
    """
    from motion.move_builders import build_jog_move

    axis_state = self.require_homed()
    current_steps = axis_state.position_steps
    if current_steps is None:
        raise RuntimeError(
            "Lateral position unknown — cannot move to start position"
        )

    target_steps = self._config.lateral_start_position_steps
    delta_steps = target_steps - current_steps

    if delta_steps == 0:
        logger.info(
            "lateral axis already at start position (%.3f mm) — no move needed",
            self._config.lateral_start_position_mm,
        )
        self._events.publish(
            EventKind.HOMING_COMPLETED,
            axis_id=self._config.lateral_axis_id,
            position_mm=self._config.lateral_start_position_mm,
        )
        return

    steps_per_rev = (
        self._config.lateral_steps_per_revolution
        * self._config.lateral_microstepping
    )

    move = build_jog_move(
        name="post_home_goto_start_position",
        axis_id=self._config.lateral_axis_id,
        steps=abs(delta_steps),
        steps_per_rev=steps_per_rev,
        rpm=self._config.lateral_homing_search_rpm,
        reverse=(delta_steps < 0),
    )

    logger.info(
        "lateral axis: moving to start position %.3f mm "
        "(soft_limit_min=%.3f mm + offset=%.3f mm), delta=%+d steps",
        self._config.lateral_start_position_mm,
        self._config.lateral_soft_limit_min_mm or 0.0,
        self._config.lateral_axis_offset_mm,
        delta_steps,
    )

    self._move_queue.enqueue(move)
    self._move_queue.wait_until_idle(
        timeout_s=max(move.axis_configs[0].ramp.total_duration * 4.0, 10.0)
    )

    if move.state != MoveState.COMPLETED:
        raise RuntimeError(
            f"post-home move to start position failed: "
            f"{move.error or move.state.name}"
        )

    self._events.publish(
        EventKind.HOMING_COMPLETED,
        axis_id=self._config.lateral_axis_id,
        position_mm=self._config.lateral_start_position_mm,
    )
```

### 2b. Refactor de `home()`

Supprimer le `self._events.publish(EventKind.HOMING_COMPLETED, ...)` existant
dans `home()` — il est maintenant délégué à `move_to_start_position()`.

```python
def home(
    self,
    *,
    axis_id: int,
    approach_rpm: float,
    search_rpm: float,
    backoff_steps: int,
) -> tuple[bool, str | None]:
    """
    Execute lateral homing then move to winding start position.

    Sequence:
      1. HomingMove (approach → backoff → search → mark_homed(0))
      2. move_to_start_position() → soft_limit_min + axis_offset
         → publishes HOMING_COMPLETED on success

    Returns (True, None) on success, (False, reason) on failure.
    """
    self.require_axis_state(axis_id)
    steps_per_rev = (
        self._config.lateral_steps_per_revolution
        * self._config.lateral_microstepping
    )
    max_approach_steps = steps_per_rev * 20

    self._events.publish(EventKind.HOMING_STARTED, axis_id=axis_id)

    move = HomingMove(
        name=f"home_axis_{axis_id}",
        axis_id=axis_id,
        steps_per_rev=steps_per_rev,
        approach_rpm=approach_rpm,
        search_rpm=search_rpm,
        backoff_steps=backoff_steps,
        max_approach_steps=max_approach_steps,
        home_position_steps=0,
        reverse_direction=self._config.lateral_invert_direction,
    )

    self._move_queue.enqueue(move)

    try:
        self._move_queue.wait_until_idle(timeout_s=120.0)
    except TimeoutError as exc:
        self._events.publish(
            EventKind.HOMING_FAILED, axis_id=axis_id, reason=str(exc)
        )
        return False, f"homing timeout: {exc}"

    if move.state != MoveState.COMPLETED:
        reason = move.error or f"homing ended in state {move.state.name}"
        self._events.publish(EventKind.HOMING_FAILED, axis_id=axis_id, reason=reason)
        return False, reason

    # HomingMove succeeded. Move to winding start position.
    # HOMING_COMPLETED is published inside move_to_start_position().
    try:
        self.move_to_start_position()
    except Exception as exc:
        reason = f"post-home positioning failed: {exc}"
        logger.exception("lateral axis: move_to_start_position failed")
        self._events.publish(EventKind.HOMING_FAILED, axis_id=axis_id, reason=reason)
        return False, reason

    return True, None
```

---

## 3. `MotionCommandService` — exposer `move_to_start_position`

**Fichier :** `core/command_service.py`

```python
def move_to_start_position(self) -> dict[str, Any]:
    """Move the lateral axis to its current winding start position.

    start_position = lateral_soft_limit_min_mm + lateral_axis_offset_mm

    Requires the axis to be homed and the engine to be IDLE.
    Useful after calling set_axis_offset() to reposition without re-homing.
    """
    if self._state.engine_state != EngineState.IDLE:
        raise RuntimeError(
            "move_to_start_position only allowed when engine is IDLE"
        )
    self._lateral.move_to_start_position()
    return {
        "status": "completed",
        "position_mm": self._config.lateral_start_position_mm,
        "soft_limit_min_mm": self._config.lateral_soft_limit_min_mm,
        "axis_offset_mm": self._config.lateral_axis_offset_mm,
    }
```

---

## 4. `WindingRpcHandler` — `set_axis_offset` et `move_to_start_position`

**Fichier :** `jsonrpc/winding_handler.py`

`WindingRpcHandler` a accès à `ConfigurationManager` — c'est lui qui gère la
mutation + persistance de la config, pas `MotionCommandService`.

### 4a. `set_axis_offset`

```python
def set_axis_offset(self, offset_mm: float) -> dict[str, Any]:
    """Set the lateral axis start offset in mm (from soft_limit_min_mm).

    Persists immediately to the configuration file. Does NOT trigger
    motion — call winding.move_to_start_position() separately to
    reposition without re-homing.

    Args:
        offset_mm: Offset in mm (positive or negative).
                   soft_limit_min_mm + offset_mm must stay within
                   [soft_limit_min_mm, soft_limit_max_mm].

    Returns:
        {status, axis_offset_mm, start_position_mm, soft_limit_min_mm}
    """
    try:
        offset_mm = float(offset_mm)
    except (TypeError, ValueError) as exc:
        raise JsonRpcError(-32602, f"offset_mm must be a number: {exc}") from exc

    current = self._config_manager.active_configuration

    # Validate by constructing a new AppConfiguration — __post_init__
    # enforces all soft-limit constraints (start position within bounds).
    try:
        updated = AppConfiguration(
            **{**vars(current), "lateral_axis_offset_mm": offset_mm}
        )
    except ValueError as exc:
        raise JsonRpcError(-32602, str(exc)) from exc

    # Persist to disk.
    self._config_manager.save_configuration(updated)

    # Propagate to the live active_configuration instance so that all
    # objects holding a reference to it see the updated value immediately.
    # AppConfiguration is not frozen, so direct mutation is safe here.
    self._config_manager.active_configuration.lateral_axis_offset_mm = offset_mm

    return {
        "status": "ok",
        "axis_offset_mm": offset_mm,
        "start_position_mm": updated.lateral_start_position_mm,
        "soft_limit_min_mm": updated.lateral_soft_limit_min_mm,
    }
```

### 4b. `move_to_start_position`

```python
def move_to_start_position(self, _params: Any | None = None) -> dict[str, Any]:
    """Move the lateral axis to soft_limit_min_mm + lateral_axis_offset_mm.

    Requires:
      - Axis homed.
      - Engine IDLE.

    Call after set_axis_offset() to reposition without re-homing.
    """
    try:
        return self._commands.move_to_start_position()
    except RuntimeError as exc:
        raise JsonRpcError(-32000, str(exc)) from exc
```

### 4c. Enregistrement dans `register_all`

```python
handler.register("winding.set_axis_offset",        self.set_axis_offset)
handler.register("winding.move_to_start_position",  self.move_to_start_position)
```

---

## 5. Note sur la propagation de la config en mémoire

`LateralAxisController`, `MotionCommandService` et `WindingEngine` reçoivent tous
`config: AppConfiguration` à la construction et gardent `self._config = config`.
Si ce sont des **références à la même instance** (c'est le cas dans
`WinderApplication` où un seul objet `AppConfiguration` est créé), alors la ligne :

```python
self._config_manager.active_configuration.lateral_axis_offset_mm = offset_mm
```

suffit à propager la valeur à tous les objets vivants immédiatement.

`save_configuration(updated)` crée une nouvelle instance `updated` pour la
validation et la sérialisation JSON, mais **ne remplace pas** `active_configuration`
— c'est pourquoi la mutation directe est nécessaire après. Si ce comportement
doit changer, envisager de passer `ConfigurationManager` en argument de
`LateralAxisController` pour lire `_config_manager.active_configuration` à chaque
accès plutôt que de cacher la référence dans `self._config`.

---

## 6. Cas dégénéré — offset et soft_limit_min à zéro

```
target_steps = int(round(0.0 * steps_per_mm)) = 0
current_steps après mark_homed(0) = 0
delta_steps = 0
→ early return dans move_to_start_position()
→ HOMING_COMPLETED publié quand même (position_mm=0.0)
```

Aucune régression : comportement identique à l'actuel, aucun mouvement superflu.

---

## 7. Invariants et contrats

- `soft_limit_min_mm + axis_offset_mm >= soft_limit_min_mm` — validé dans `__post_init__` (si `soft_limit_min_mm` est défini).
- `soft_limit_min_mm + axis_offset_mm <= soft_limit_max_mm` — validé dans `__post_init__` (si `soft_limit_max_mm` est défini).
- `move_to_start_position()` requiert `homed = True` — délégué à `require_homed()`.
- L'offset ne bypass pas les soft-limits à l'exécution : `check_move()` dans
  `AxisState` est le gardien final lors du jog.
- `set_axis_offset()` est sans effet de bord sur la position courante.
- `HOMING_COMPLETED` est publié **une seule fois**, à la fin de `move_to_start_position()`.
- `HOMING_COMPLETED` est publié **même si delta_steps == 0** (early return).

---

## 8. Tableau des fichiers modifiés

| Fichier | Changement |
|---|---|
| `core/config.py` | Ajouter `lateral_axis_offset_mm: float = 0.0`, validation dans `__post_init__`, propriétés `lateral_start_position_mm` et `lateral_start_position_steps` |
| `core/lateral.py` | Ajouter `move_to_start_position()`; modifier `home()` pour l'appeler en fin de séquence; supprimer `HOMING_COMPLETED` dans `home()` |
| `core/command_service.py` | Ajouter `move_to_start_position()` |
| `jsonrpc/winding_handler.py` | Ajouter `set_axis_offset()`, `move_to_start_position()`; enregistrer dans `register_all()` |

`AxisState`, `MoveQueue`, `ConfigurationManager` : **aucun changement**.

---

## 9. Plan de commits

```
G1  feat(config): add lateral_axis_offset_mm with validation and start_position properties
G2  feat(lateral): add move_to_start_position(), refactor home() to call it
G3  feat(command_service): expose move_to_start_position()
G4  feat(rpc): register winding.set_axis_offset and winding.move_to_start_position
```

G1 est un prérequis de tous les autres.
G2 dépend de G1. G3 dépend de G2. G4 dépend de G3.
Chaque commit doit passer `mypy --strict` avant le suivant.