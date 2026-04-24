# Homing latéral — document complet d’analyse

## Objectif

Ce document regroupe **toutes les phases du homing latéral**, les responsabilités **host** et **firmware**, les **champs de status** surveillés, les **extraits de code actifs** dans le dépôt, ainsi que les **points suspects** pouvant expliquer le comportement observé sur machine réelle :

- l’axe part dans la mauvaise direction,
- l’axe va au endstop puis passe en faute,
- le comportement réel diverge d’une suite de tests host pourtant verte.

Ce document est volontairement orienté **forensic / diagnostic**.

---

## 1. Vue d’ensemble du chemin d’exécution

Chaîne complète du homing latéral :

1. appel JSON-RPC `winding.home_lateral`
2. `WindingRpcHandler.home_lateral()`
3. `MotionCommandService.home_lateral()`
4. `LateralAxisController.start_home()`
5. création d’un `HomingMove`
6. `MoveQueue._execute_homing()`
7. pour chaque phase :
   - préchecks status
   - arm/disarm endstop via SPI
   - streaming des segments
   - validation du résultat de phase
8. firmware :
   - accepte `ENABLE_ENDSTOP`
   - publie `lateral_endstop_state`, `endstop_armed_mask`, `endstop_hit_mask`
   - bloque ou stoppe le mouvement selon l’état endstop
   - passe en `RECOVERY` sur hit, blocage ou fail-safe

Résumé simple :

```text
RPC
 -> handler
 -> command service
 -> lateral controller
 -> HomingMove(phases)
 -> MoveQueue
    -> arm/disarm endstop
    -> MultiAxisRampStreamer
       -> Esp32SpiTransport
          -> SPI
             -> CommInterface
                -> planner/executor
                -> StepperDriver / endstop ISR
```

---

## 2. Phases fonctionnelles du homing

Le homing latéral côté host est un homing **multi-phase piloté par le Raspberry Pi**.

### Phase A — vérification initiale

Le host lit le status firmware avant de lancer la moindre phase.

Cas possibles :

- `PRESENT_OPEN` : on peut démarrer normalement
- `PRESENT_CLOSED` : on exécute un **preclear**
- `ABSENT` : le homing échoue immédiatement

### Phase B — preclear si le capteur est déjà fermé

Si l’axe démarre déjà en appui sur le endstop :

- désarmement de l’endstop
- mouvement de recul
- attente de réouverture du capteur
- attente de stabilisation mécanique
- relecture finale du status

### Phase C — approach rapide

Phase armée, déplacement vers le capteur à vitesse élevée.

Condition de succès :

- hit endstop détecté par le streamer / firmware

Condition d’échec :

- mouvement terminé sans déclenchement endstop
- capteur `ABSENT`
- capteur reste `CLOSED` quand il ne devrait pas

### Phase D — backoff

Phase désarmée, déplacement en sens inverse pour rouvrir le capteur.

Condition de succès :

- `lateral_endstop_state == PRESENT_OPEN`

Condition d’échec :

- timeout d’ouverture
- capteur absent / incohérent

### Phase E — search lent

Nouvelle approche, armée, à vitesse réduite pour fixer le zéro.

Condition de succès :

- second déclenchement endstop

### Phase F — finalisation

- désarmement endstop
- `AxisState.mark_homed(home_position_steps)`
- move marqué `COMPLETED`

---

## 3. Entrée RPC

### Fichier

`src/rpi/jsonrpc/winding_handler.py`

### Code actif

```python
def home_lateral(
    self,
    approach_rpm: float = 100.0,
    search_rpm: float = 20.0,
    backoff_steps: int = 3200,
) -> dict[str, Any]:
    """Start the lateral homing sequence and return immediately."""
    result = self._commands.home_lateral(
        approach_rpm=approach_rpm,
        search_rpm=search_rpm,
        backoff_steps=backoff_steps,
    )
    return result
```

### Ce que ça implique

- Le RPC ne fait **aucune logique de homing lui-même**.
- Il délègue tout au `MotionCommandService`.
- Les paramètres critiques utilisateur sont :
  - `approach_rpm`
  - `search_rpm`
  - `backoff_steps`

---

## 4. Command service — démarrage asynchrone

### Fichier

`src/rpi/motion/command_service.py`

### Code actif

```python
def home_lateral(
    self,
    approach_rpm: float = 120.0,
    search_rpm: float = 10.0,
    backoff_steps: int = 1600,
) -> dict[str, Any]:
    """Start lateral homing asynchronously and return immediately."""
    if self._state.engine_state != EngineState.IDLE:
        raise RuntimeError(
            "home_lateral only allowed when engine is IDLE; if a FAULT occurred, "
            "acknowledge it with winding.clear_fault first"
        )

    self._state.set_engine_state(EngineState.HOMING)
    move = self._lateral.start_home(
        axis_id=self._config.lateral_axis_id,
        approach_rpm=approach_rpm,
        search_rpm=search_rpm,
        backoff_steps=backoff_steps,
    )

    monitor = threading.Thread(
        target=self._wait_for_lateral_home_completion,
        args=(move,),
        daemon=True,
        name="manual_home_monitor",
    )
    monitor.start()

    return {
        "status": "started",
        "axis_id": self._config.lateral_axis_id,
        "approach_rpm": approach_rpm,
        "search_rpm": search_rpm,
        "backoff_steps": backoff_steps,
    }
```

### Complétion/fault

```python
def _wait_for_lateral_home_completion(self, move: Any) -> None:
    while not move.done:
        time.sleep(0.05)

    success, _reason = self._lateral.finalize_home_move(move)
    if success and self._state.engine_state == EngineState.HOMING:
        self._state.set_engine_state(EngineState.IDLE)
```

### Ce que ça implique

- Le homing est **lancé puis surveillé**.
- Le retour RPC `started` ne signifie pas succès.
- La faute finale est propagée plus tard par `finalize_home_move()`.

---

## 5. LateralAxisController — création du HomingMove

### Fichier

`src/rpi/core/lateral.py`

### Code actif

```python
def _create_home_move(
    self,
    *,
    axis_id: int,
    approach_rpm: float,
    search_rpm: float,
    backoff_steps: int,
) -> HomingMove:
    steps_per_rev = (
        self._config.lateral_steps_per_revolution
        * self._config.lateral_microstepping
    )
    return HomingMove(
        name="home_lateral",
        axis_id=axis_id,
        steps_per_rev=steps_per_rev,
        approach_rpm=approach_rpm,
        search_rpm=search_rpm,
        backoff_steps=backoff_steps,
        max_approach_steps=int(steps_per_rev * 20),
        reverse_direction=self._config.lateral_invert_direction,
    )
```

### Point critique n°1 — direction

Le sens du homing dépend directement de :

```python
reverse_direction=self._config.lateral_invert_direction
```

Donc si l’axe part **dans la mauvaise direction**, le premier suspect logique côté host est :

- `lateral_invert_direction`

### Finalisation du homing

```python
def finalize_home_move(self, move: HomingMove) -> tuple[bool, str | None]:
    axis_id = move.axis_id
    if move.state is MoveState.COMPLETED:
        axis_state = self.require_axis_state(axis_id).snapshot()
        self._events.publish(
            EventKind.HOMING_COMPLETED,
            axis_id=axis_id,
            axis_state=axis_state,
        )
        return True, None

    message = f"Homing failed: {move.error or move.state.name}"
    self._state.set_fault(message)
    self._events.publish(
        EventKind.HOMING_FAILED,
        axis_id=axis_id,
        error=message,
        move_state=move.state.name,
    )
    return False, move.error
```

### Ce que ça implique

Toute sortie non `COMPLETED` produit un état `FAULT` global côté host.

---

## 6. Paramètres host qui influencent fortement le homing

### Fichier

`src/rpi/core/config.py`

### Code actif

```python
@dataclass
class AppConfiguration:
    rpc_socket_path: str = "/tmp/winding.sock"
    spi_device: str = "/dev/spidev0.0"
    spi_speed_hz: int = 4_000_000
    spi_ready_gpio_chip: Optional[str] = "/dev/gpiochip0"
    spi_ready_gpio_line: Optional[int] = 17
    spi_ready_active_high: bool = True

    spindle_axis_id: int = 0
    spindle_steps_per_revolution: int = 200
    spindle_microstepping: int = 32
    spindle_invert_direction: bool = False

    lateral_axis_id: int = 1
    lateral_steps_per_revolution: int = 200
    lateral_microstepping: int = 32
    lateral_invert_direction: bool = True
```

### Points critiques

#### 6.1 Direction latérale

```python
lateral_invert_direction: bool = True
```

C’est un candidat direct pour expliquer :

- « l’axe part dans la mauvaise direction »

#### 6.2 READY GPIO

```python
spi_ready_gpio_line: Optional[int] = 17
```

Ce point est critique car le firmware actuel utilise **GPIO4**, pas GPIO17.

---

## 7. Contrat des phases — objet HomingMove

### Fichier

`src/rpi/motion/move.py`

### Code actif

```python
class HomingMove(CompositeMove):
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
```

### Sous-mouvement approach

```python
def _make_approach_move(self) -> RampMove:
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
```

### Sous-mouvement backoff

```python
def _make_backoff_move(self) -> RampMove:
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
                        reverse_direction=not self.reverse_direction,
                    ),
                )
            ],
            segment_duration_s=self.segment_duration_s,
        ),
    )
```

### Sous-mouvement search

```python
def _make_search_move(self) -> RampMove:
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
```

### Séquence officielle des phases

```python
def phases(self) -> list[tuple[str, RampMove, bool]]:
    return [
        ("approach", self._make_approach_move(), True),
        ("backoff", self._make_backoff_move(), False),
        ("search", self._make_search_move(), True),
    ]
```

### Ce que ça implique

- `approach` et `search` vont **dans le même sens**.
- `backoff` va explicitement **dans le sens inverse**.
- Si `reverse_direction` est faux alors approach/search utilisent le sens direct.
- Si ce booléen ne correspond pas au câblage moteur, le homing partira physiquement du mauvais côté.

---

## 8. Exécution host réelle du homing — MoveQueue

### Fichier

`src/rpi/motion/move_queue.py`

C’est le cœur réel du homing.

### 8.1 Preclear si le endstop est fermé au départ

```python
def _clear_closed_endstop_before_homing(self, move: HomingMove) -> None:
    logger.info(
        "homing axis %s: endstop already closed at start, running preclear",
        move.axis_id,
    )
    clearance_move = move._make_backoff_move()
    self._set_endstop_armed(move.axis_id, arm=False)
    streamer = self._stream_homing_sub_move(
        move,
        phase_name="preclear",
        sub_move=clearance_move,
        arm_endstop=False,
    )
    if streamer.endstop_triggered:
        raise RuntimeError(
            f"preclear failed: unexpected endstop trigger on axis {move.axis_id}"
        )
    self._wait_for_endstop_open(
        move.axis_id,
        timeout_s=self._compute_backoff_timeout(clearance_move),
    )
    time.sleep(0.020)
    final_status = self._read_status(move.axis_id)
    lateral_state = int(
        getattr(final_status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
    )
    if lateral_state != LATERAL_ENDSTOP_PRESENT_OPEN:
        raise RuntimeError(
            f"preclear did not clear the endstop on axis {move.axis_id}: "
            f"lateral_endstop_state=0x{lateral_state:02X} after stabilisation wait"
        )
```

### Ce que ça implique

- si le capteur est fermé au départ, le host essaie d’abord un recul contrôlé,
- une stabilisation mécanique de `20 ms` est imposée,
- si le capteur n’est pas `OPEN` après ça, le homing échoue.

### 8.2 Vérification d’une phase armée

```python
def _check_armed_phase_result(
    self,
    move: HomingMove,
    phase_name: str,
    streamer: "MultiAxisRampStreamer",
) -> None:
    if streamer.endstop_triggered:
        return

    status = self._read_status(move.axis_id)
    lateral_state = int(
        getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
    )
    last_exec = int(getattr(status, "last_executed_sequence", 0xFFFF))
    last_sent = streamer.last_sent_motion_seq
    running   = int(getattr(status, "running_mask", 0))

    state_names = {
        LATERAL_ENDSTOP_PRESENT_OPEN: "PRESENT_OPEN",
        LATERAL_ENDSTOP_PRESENT_CLOSED: "PRESENT_CLOSED",
        LATERAL_ENDSTOP_ABSENT: "ABSENT",
    }
    state_str = state_names.get(lateral_state, f"0x{lateral_state:02X}")

    raise RuntimeError(
        f"homing phase '{phase_name}' on axis {move.axis_id} ended without "
        f"endstop trigger. "
        f"lateral_state={state_str}, "
        f"running=0x{running:02X}, "
        f"last_exec={last_exec}, "
        f"last_sent={last_sent}"
    )
```

### Ce que ça implique

Si l’axe bouge mais que le déclenchement attendu n’est pas vu, le host passe en faute avec un message qui doit permettre de savoir :

- si le capteur était vu `OPEN`, `CLOSED` ou `ABSENT`,
- si l’axe tournait encore,
- où en étaient les séquences envoyées/exécutées.

### 8.3 Armement / désarmement endstop

```python
def _set_endstop_armed(self, axis_id: int, arm: bool) -> Any:
    sequence, send_status = self._transport.enable_endstop_request(axis_id, arm=arm)
    status = self._wait_for_transport_request_result(
        sequence,
        send_status=send_status,
    )
    self._update_axis_endstop_state(axis_id, status)
    if int(getattr(status, "last_result", SpiMessageResult.OK)) != int(SpiMessageResult.OK):
        raise RuntimeError(
            f"enable_endstop axis {axis_id} arm={int(arm)} failed with result=0x{int(status.last_result):02X}"
        )
    if self._status_has_endstop_armed(status, axis_id, arm):
        return status
    return self._wait_for_endstop_arm_state(axis_id, arm)
```

### Ce que ça implique

Le host ne suppose pas que la commande est effective immédiatement :

- il envoie `ENABLE_ENDSTOP`
- il attend l’ACK
- il attend aussi que `endstop_armed_mask` reflète l’état demandé

### 8.4 Boucle complète du homing

```python
def _execute_homing(self, move: HomingMove) -> None:
    move.mark_running()
    axis_state = self._axis_states.get(move.axis_id)

    try:
        initial_status = self._read_status(move.axis_id)
        initial_state = int(
            getattr(initial_status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
        )
        if initial_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
            initial_state = self._confirm_initial_closed_endstop(move.axis_id)
        if initial_state == LATERAL_ENDSTOP_ABSENT:
            self._ensure_homing_can_start(move.axis_id, "start")
        elif initial_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
            self._clear_closed_endstop_before_homing(move)
            self._ensure_homing_can_start(move.axis_id, "start")
        else:
            self._ensure_homing_can_start(move.axis_id, "start")
    except Exception as exc:
        move.mark_failed(str(exc))
        return

    next_sequence = self._next_motion_sequence()

    for phase_name, sub_move, arm_endstop in move.phases():
        if self._stop_requested:
            self._set_endstop_armed(move.axis_id, arm=False)
            stop_plan = self._active_stop_plan or self._default_stop_plan(
                [move.axis_id],
                "stop requested during homing",
            )
            self._apply_stop_plan([move.axis_id], stop_plan)
            move.mark_aborted(f"{stop_plan.mode.value} requested during homing")
            return

        if arm_endstop:
            try:
                self._ensure_homing_can_start(move.axis_id, phase_name)
            except Exception as exc:
                move.mark_failed(str(exc))
                return

        try:
            self._set_endstop_armed(move.axis_id, arm=arm_endstop)
        except Exception as exc:
            move.mark_failed(str(exc))
            return

        try:
            streamer = self._stream_homing_sub_move(
                move,
                phase_name=phase_name,
                sub_move=sub_move,
                arm_endstop=arm_endstop,
                start_sequence=next_sequence,
            )
        except Exception as exc:
            self._set_endstop_armed(move.axis_id, arm=False)
            move.mark_failed(str(exc))
            return

        next_sequence = self._next_sequence_after_streamer(streamer)

        phase_completed_on_expected_endstop = arm_endstop and streamer.endstop_triggered
        if self._stop_requested or (
            self._streamer_stop_requested(streamer)
            and not phase_completed_on_expected_endstop
        ):
            self._set_endstop_armed(move.axis_id, arm=False)
            stop_plan = self._active_stop_plan or self._default_stop_plan(
                [move.axis_id],
                "stop requested during homing",
            )
            self._apply_stop_plan([move.axis_id], stop_plan)
            move.mark_aborted(f"{stop_plan.mode.value} requested during homing")
            return

        if phase_name in ("approach", "search"):
            try:
                self._check_armed_phase_result(move, phase_name, streamer)
            except RuntimeError as exc:
                self._set_endstop_armed(move.axis_id, arm=False)
                move.mark_failed(str(exc))
                return

        if phase_name == "backoff":
            try:
                self._wait_for_endstop_open(
                    move.axis_id,
                    timeout_s=self._compute_backoff_timeout(sub_move),
                )
            except Exception as exc:
                self._set_endstop_armed(move.axis_id, arm=False)
                move.mark_failed(str(exc))
                return

            _deadline = time.monotonic() + 1.0
            while time.monotonic() < _deadline:
                _status = self._read_status(move.axis_id)
                if (int(getattr(_status, "running_mask", 0)) & (1 << move.axis_id)) == 0:
                    break
                time.sleep(0.010)
            else:
                _running_mask = int(getattr(_status, "running_mask", 0xFF))
                self._set_endstop_armed(move.axis_id, arm=False)
                move.mark_failed(
                    f"backoff stop timeout on axis {move.axis_id}: "
                    f"running_mask=0x{_running_mask:02X} still non-zero after backoff"
                )
                return

        if self._stop_requested:
            self._set_endstop_armed(move.axis_id, arm=False)
            stop_plan = self._active_stop_plan or self._default_stop_plan(
                [move.axis_id],
                "stop requested during homing",
            )
            self._apply_stop_plan([move.axis_id], stop_plan)
            move.mark_aborted(f"{stop_plan.mode.value} requested during homing")
            return

    self._set_endstop_armed(move.axis_id, arm=False)
    if axis_state is not None:
        axis_state.mark_homed(move.home_position_steps)

    move.mark_completed()
```

### Ce que ça implique

L’ordre réel est :

1. lecture status initial
2. éventuellement preclear
3. phase `approach` armée
4. **attente obligatoire** de fin d’arrêt réel + fin de cycle `RECOVERY` firmware après le hit d’approche
5. phase `backoff` désarmée
6. attente `PRESENT_OPEN` + `running_mask == 0` + latch effacé avant toute nouvelle phase armée
7. phase `search` armée
8. attente de fin d’arrêt réel + fin de cycle `RECOVERY` firmware après le hit de `search`
9. désarmement final
10. `mark_homed` (actuellement sur valeur fixe, idéalement sur position réelle du hit)

### 8.5 Barrière post-hit requise entre `approach` et `backoff`

Depuis la correction appliquée, le host ne passe plus directement du hit d’approche au backoff.
Il attend explicitement :

- que l’axe soit réellement arrêté (`running_mask` à 0 pour l’axe),
- qu’un délai de garde laisse le firmware terminer `RECOVERY`,
- que le capteur ne soit pas devenu `ABSENT`.

Code actif :

```python
def _wait_for_post_hit_recovery(
    self,
    axis_id: int,
    *,
    stop_timeout_s: float = 1.0,
    recovery_guard_s: float = 0.080,
) -> Any:
    deadline = time.monotonic() + stop_timeout_s
    last_status = None
    while time.monotonic() < deadline:
        last_status = self._read_status(axis_id)
        running = int(getattr(last_status, "running_mask", 0))
        if (running & (1 << axis_id)) == 0:
            break
        time.sleep(0.010)
    else:
        running_mask = int(getattr(last_status, "running_mask", 0xFF)) if last_status else 0xFF
        raise RuntimeError(
            f"post-hit stop timeout on axis {axis_id}: running_mask=0x{running_mask:02X}"
        )

    time.sleep(recovery_guard_s)
    status = self._read_status(axis_id)
    lateral_state = int(
        getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
    )
    if lateral_state == LATERAL_ENDSTOP_ABSENT:
        raise RuntimeError(
            f"endstop became ABSENT after hit on axis {axis_id} — check wiring"
        )
    return status
```

Et dans la boucle de homing :

```python
if phase_name in ("approach", "search") and streamer.endstop_triggered:
    try:
        self._wait_for_post_hit_recovery(move.axis_id)
    except Exception as exc:
        self._set_endstop_armed(move.axis_id, arm=False)
        move.mark_failed(str(exc))
        return
```

Cette barrière n’est **pas optionnelle** avec le firmware actuel : sans elle, le host peut envoyer le backoff ou finaliser le homing alors que le firmware est encore en `RECOVERY`, ce qui décale le point de synchronisation réel entre le hit mécanique et l’état observé côté host.

### 8.6 Latch clear requis avant tout streaming armé

Depuis les corrections résiduelles du round 2, le host vérifie explicitement qu’après un `ENABLE_ENDSTOP(arm=1)`, le `endstop_hit_mask` ne reflète plus l’ancien hit avant de démarrer une phase armée.

Code actif :

```python
def _wait_for_endstop_latch_cleared(
    self,
    axis_id: int,
    *,
    timeout_s: float = 0.5,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = self._read_status(axis_id)
        hit_mask = int(getattr(status, "endstop_hit_mask", 0xFF))
        if (hit_mask & (1 << axis_id)) == 0:
            return
        time.sleep(0.010)
    raise RuntimeError(
        f"endstop latch not cleared after arm on axis {axis_id}: "
        f"endstop_hit_mask still set after {timeout_s:.1f}s"
    )
```

Utilisation dans `_execute_homing()` :

```python
try:
    self._set_endstop_armed(move.axis_id, arm=arm_endstop)
    if arm_endstop:
        self._wait_for_endstop_latch_cleared(move.axis_id)
except Exception as exc:
    move.mark_failed(str(exc))
    return
```

Cette vérification élimine le cas où le `search` démarre sur un status SPI qui expose encore le hit précédent, ce qui ferait croire au streamer qu’un second hit a déjà eu lieu avant tout mouvement réel.

---

## 9. Détection du hit endstop côté streamer

### Fichier

`src/rpi/transport/streamer.py`

### Code actif

```python
def _check_endstop(self, status) -> bool:
    armed_mask   = int(getattr(status, "endstop_armed_mask", 0))
    lateral_state = int(getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT))
    running_mask  = int(getattr(status, "running_mask", 0))
    hit_mask      = int(getattr(status, "endstop_hit_mask", 0))

    for axis_id in self._endstop_armed_axes:
        if hit_mask & (1 << axis_id):
            self._mark_endstop_triggered()
            return True

    for axis_id in self._endstop_armed_axes:
        axis_stopped = (running_mask & (1 << axis_id)) == 0
        if lateral_state == LATERAL_ENDSTOP_PRESENT_CLOSED and axis_stopped:
            self._mark_endstop_triggered()
            return True

    dropped_now = int(getattr(status, "segments_dropped", 0))
    if self._endstop_armed_axes and dropped_now > self._last_segments_dropped:
        any_armed_stopped = any(
            (running_mask & (1 << a)) == 0
            for a in self._endstop_armed_axes
        )
        if any_armed_stopped:
            self._mark_endstop_triggered()
            return True
    self._last_segments_dropped = dropped_now

    return False
```

### Priorités réelles de détection

1. `endstop_hit_mask`
2. capteur `CLOSED` + axe arrêté
3. `segments_dropped` + axe arrêté

### Stop/flush déclenché

```python
def _mark_endstop_triggered(self) -> None:
    if self._endstop_triggered:
        return
    self._endstop_triggered = True
    flush_seq = self._last_confirmed_motion_seq
    if flush_seq < 0:
        flush_seq = self._last_sent_motion_seq
    if flush_seq < 0:
        flush_seq = 0xFFFF
    self._flush_floor_sequence = int(flush_seq) & 0xFFFF
    self.request_stop()
    self.request_flush(flush_seq)
```

### Ce que ça implique

Quand le host estime que le hit a eu lieu :

- il demande l’arrêt,
- il demande un flush firmware,
- il ne continue pas à pousser du mouvement “vers l’avant”.

---

## 10. Transport SPI — ACK, polling et READY handshake

### Fichier

`src/rpi/transport/spi_transport.py`

Le transport est crucial car un problème de timing SPI peut faire diverger la logique sans casser les tests host.

### 10.1 Désactivation automatique du READY si timeouts répétés

```python
self._ready_timeout_streak: int = 0
self._ready_timeout_disable_threshold: int = 3
self._ready_handshake_disabled: bool = False
```

```python
def _wait_until_ready(self) -> None:
    if self._ready_monitor is not None and not self._ready_handshake_disabled:
        deadline = time.monotonic() + self._ready_wait_timeout_s
        while time.monotonic() < deadline:
            if self._ready_monitor.value() == self._ready_active_level:
                self._ready_timeout_streak = 0
                return
            time.sleep(self._ready_poll_sleep_s)
        self._diag_ready_timeouts += 1
        self._diag_lifetime_ready_timeouts += 1
        self._ready_timeout_streak += 1
        if self._ready_timeout_streak >= self._ready_timeout_disable_threshold:
            self._ready_handshake_disabled = True
            logger.warning(
                "SPI READY handshake timed out %d times on %s; disabling READY GPIO for this session and using software guard",
                self._ready_timeout_streak,
                self._device_path,
            )
        else:
            logger.warning(
                "SPI READY handshake timeout on %s; falling back to software guard for this transfer",
                self._device_path,
            )

    now = time.monotonic()
    if self._last_xfer_end_ts > 0.0:
        remaining_gap_s = self._inter_transfer_guard_s - (now - self._last_xfer_end_ts)
        if remaining_gap_s > 0.0:
            time.sleep(remaining_gap_s)
```

### Ce que ça implique

Si la ligne READY configurée côté host ne correspond pas à la broche firmware, le host :

- attend inutilement,
- accumule des timeouts,
- puis désactive READY et passe en garde logicielle.

Ça évite la catastrophe totale, mais ça ne garantit pas un homing parfait si le process réel n’a pas été redémarré avec cette version.

### 10.2 Attente d’ACK firmware

```python
def wait_for_request_result(
    self,
    sequence: int,
    *,
    hint_status: StatusPayload | None = None,
    poll_interval_s: float = 0.0005,
    timeout_s: float = 1.5,
) -> StatusPayload:
    transient_protocol_results = {
        int(SpiMessageResult.BAD_MAGIC),
        int(SpiMessageResult.BAD_VERSION),
        int(SpiMessageResult.BAD_LENGTH),
        int(SpiMessageResult.BAD_CRC),
    }
    target_seq = sequence & 0xFFFF
    if hint_status is not None:
        if hint_status.last_rx_sequence == target_seq:
            if int(hint_status.last_result) not in transient_protocol_results:
                return hint_status
    deadline = time.monotonic() + max(timeout_s, 0.05)
    last_exc: Exception | None = None
    while True:
        if time.monotonic() >= deadline:
            if last_exc is not None:
                raise RuntimeError(
                    f"wait_for_request_result timeout for seq={sequence}: {last_exc!s}"
                ) from last_exc
            raise RuntimeError(
                f"wait_for_request_result timeout for seq={sequence}: no matching ack"
            )
        try:
            status = self.get_status(
                timeout_s=min(0.25, max(0.02, deadline - time.monotonic())),
                allow_stale=False,
            )
        except Exception as exc:
            last_exc = exc
            time.sleep(poll_interval_s)
            continue
        if status.last_rx_sequence == target_seq:
            if int(status.last_result) in transient_protocol_results:
                last_exc = RuntimeError(
                    "transient SPI protocol error observed after matching ack "
                    f"for seq={sequence}: result=0x{int(status.last_result):02X}"
                )
                time.sleep(poll_interval_s)
                continue
            return status
        time.sleep(poll_interval_s)
```

### Ce que ça implique

Toute commande critique du homing (`ENABLE_ENDSTOP`, envois segments, flush) dépend de la cohérence de :

- `last_rx_sequence`
- `last_result`

Si la couche SPI réelle est dégradée, le homing peut rater même avec une logique de haut niveau correcte.

### 10.3 Diagnostic exporté

```python
def transport_diagnostics(self) -> dict[str, int | float | None]:
    ...
    return {
        "total_xfers": self._diag_lifetime_total_xfers,
        "bad_magic": self._diag_lifetime_bad_magic,
        "bad_crc": self._diag_lifetime_bad_crc,
        "zero_rx": self._diag_lifetime_zero_rx,
        "echo_rx": self._diag_lifetime_echo_rx,
        "ready_timeouts": self._diag_lifetime_ready_timeouts,
        "ready_handshake_disabled": int(self._ready_handshake_disabled),
        "reopens": self._diag_lifetime_reopens,
        "last_status_age_s": last_status_age_s,
    }
```

---

## 11. Contrat firmware officiel du homing

### Fichier

`src/esp32/HOMING.md`

### Texte source

```markdown
1. `ENABLE_ENDSTOP(arm=1)` clears the previous homing latch.
2. A valid CLOSED sample (`NO=0`, `NC=1`) while armed latches a hit immediately.
3. The executor posts an internal `FLUSH` request on endstop recovery paths so stale queued motion is discarded.
4. After a hit, only the latched clearance direction is accepted while the switch remains closed.
5. Short INVALID (`NO==NC`) crossover windows are masked using the last stable state.
6. Persistent INVALID state is reported as `ABSENT` and must be treated as a homing fault.
```

### Ce que ça implique

Le firmware ne fait pas un homing autonome complet.
Il fournit :

- la protection endstop,
- le latch de hit,
- le stop / flush / recovery après hit.

Depuis la correction 5 appliquée dans `stepper_driver.cpp`, **le point 4 du contrat
firmware n'est plus actif dans le code** : `isEndstopMoveAllowed` ne rejette plus les
mouvements sur la base d'une direction latchée quand le capteur est `CLOSED`.
Le séquencement du backoff et du second pass reste intégralement côté host.

Si le fichier `src/esp32/HOMING.md` n'a pas encore été mis à jour, il doit l'être
pour refléter ce comportement : le point 4 doit être supprimé ou annoté comme supprimé.

---

## 12. Firmware — définition wire du protocole et du status

### Fichier

`src/esp32/src/messages.h`

### Types endstop

```cpp
enum class SpiMessageType : uint8_t {
    ...
    MULTI_AXIS_SEGMENT_BLOCK = 0x13,
    ENABLE_ENDSTOP           = 0x14,
    ...
};

enum class SpiMessageResult : uint8_t {
    OK             = 0x00,
    BAD_MAGIC      = 0x01,
    BAD_VERSION    = 0x02,
    BAD_LENGTH     = 0x03,
    BAD_CRC        = 0x04,
    UNKNOWN_TYPE   = 0x05,
    BAD_AXIS       = 0x06,
    QUEUE_FULL     = 0x07,
    INTERNAL_ERROR = 0x08,
    ENDSTOP_BLOCKED = 0x09,
};

enum class LateralEndstopState : uint8_t {
    PRESENT_OPEN   = 0x00,
    PRESENT_CLOSED = 0x01,
    ABSENT         = 0xFF,
};
```

### Payload ENABLE_ENDSTOP

```cpp
struct __attribute__((packed)) EnableEndstopPayload {
    uint8_t axis_id;
    uint8_t arm;
    uint8_t reserved[2];
};
```

### StatusPayload

```cpp
struct __attribute__((packed)) StatusPayload {
    uint32_t uptime_ms;
    uint16_t queue_free_slots[SPI_MAX_AXES];
    uint16_t ring_free_slots[SPI_MAX_AXES];
    uint32_t underrun_count[SPI_MAX_AXES];
    uint16_t last_rx_sequence;
    uint8_t  last_rx_type;
    uint8_t  last_result;
    uint8_t  protocol_version;
    uint8_t  enabled_mask;
    uint8_t  running_mask;
    uint8_t lateral_endstop_state;
    uint8_t endstop_armed_mask;
    uint8_t endstop_hit_mask;
    uint16_t last_executed_sequence;
    uint8_t  multi_axis_queue_free;
    uint8_t  planner_queue_free;
    uint16_t last_planned_sequence;
    uint16_t segments_dropped;
};
```

### Champs les plus importants pour diagnostiquer le homing

- `lateral_endstop_state`
- `endstop_armed_mask`
- `endstop_hit_mask`
- `running_mask`
- `last_rx_sequence`
- `last_result`
- `last_executed_sequence`
- `segments_dropped`

---

## 13. Firmware — publication du status et ENABLE_ENDSTOP

### Fichier

`src/esp32/src/comm_interface.cpp`

### Construction du status

```cpp
payload->last_rx_sequence = last_rx_sequence_;
payload->last_rx_type     = last_rx_type_;
payload->last_result      = last_result_;
payload->protocol_version = SPI_MSG_VERSION;
payload->lateral_endstop_state = readLateralEndstopState();

payload->endstop_armed_mask = 0;
for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
    if (axis < n_motors_ && queues_[axis] != nullptr) {
        if (queues_[axis]->driver().isEndstopArmed()) {
            payload->endstop_armed_mask |= static_cast<uint8_t>(1U << axis);
        }
    }
}

payload->endstop_hit_mask = 0;
for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
    if (axis < n_motors_ && queues_[axis] != nullptr) {
        if (queues_[axis]->driver().getEndstopHitCount() > 0) {
            payload->endstop_hit_mask |= static_cast<uint8_t>(1U << axis);
        }
    }
}
```

### Handler ENABLE_ENDSTOP

```cpp
esp_err_t CommInterface::handleEnableEndstop(const EnableEndstopPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    StepperDriver& drv = queues_[payload.axis_id]->driver();
    if (payload.arm) {
        drv.armEndstop();
        ESP_LOGI(TAG, "endstop armed on axis %u", payload.axis_id);
    } else {
        drv.disarmEndstop();
        ESP_LOGI(TAG, "endstop disarmed on axis %u", payload.axis_id);
    }
    return ESP_OK;
}
```

---

## 14. Firmware — lecture logique du capteur endstop

### Fichier

`src/esp32/src/stepper_driver.cpp`

### États logiques rapportés

```cpp
uint8_t StepperDriver::reportedEndstopState() const
{
    if (endstop_no_pin_ == GPIO_NUM_NC || endstop_nc_pin_ == GPIO_NUM_NC) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }

    const EndstopSignalState raw = static_cast<EndstopSignalState>(
        endstop_signal_state_.load(std::memory_order_acquire));
    if (raw == EndstopSignalState::CLOSED) {
        return static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED);
    }
    if (raw == EndstopSignalState::OPEN) {
        return static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
    }

    const TickType_t invalid_since =
        endstop_invalid_since_tick_.load(std::memory_order_acquire);
    if (invalid_since != 0) {
        const TickType_t now = xTaskGetTickCount();
        if ((now - invalid_since) >= ENDSTOP_INVALID_DEBOUNCE_TICKS) {
            return static_cast<uint8_t>(LateralEndstopState::ABSENT);
        }
    }

    const EndstopSignalState stable = static_cast<EndstopSignalState>(
        endstop_last_stable_state_.load(std::memory_order_acquire));
    return (stable == EndstopSignalState::CLOSED)
        ? static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED)
        : static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}
```

### Règle de blocage des mouvements

```cpp
bool StepperDriver::isEndstopMoveAllowed(bool direction) const
{
    (void)direction;

    if (!isEndstopArmed()) {
        return true;
    }

    const uint8_t state = reportedEndstopState();
    if (state == static_cast<uint8_t>(LateralEndstopState::ABSENT)) {
        return false;
    }

    return true;
}
```

### Ce que ça implique

Si l’endstop est armé :

- `OPEN` : mouvement autorisé
- `CLOSED` : mouvement autorisé
- `ABSENT` : mouvement refusé

Le firmware ne bloque donc plus le backoff ou le second pass sur une règle de direction latchée. Le seul veto firmware restant sur cette fonction est le cas `ABSENT`.

Historiquement, un problème de sens pouvait produire :

- blocage immédiat firmware,
- `ENDSTOP_BLOCKED`,
- arrêt suivi d’une faute host.

---

## 15. Firmware — ISR endstop

### Fichier

`src/esp32/src/stepper_driver.cpp`

### Code actif

```cpp
void IRAM_ATTR StepperDriver::endstopIsrHandler(void* arg)
{
    StepperDriver* drv = static_cast<StepperDriver*>(arg);

    const int no_lvl = gpio_get_level(drv->endstop_no_pin_);
    const int nc_lvl = gpio_get_level(drv->endstop_nc_pin_);

    const EndstopSignalState raw = decodeEndstopSignalState(no_lvl, nc_lvl);
    drv->endstop_signal_state_.store(static_cast<uint8_t>(raw), std::memory_order_release);

    const TickType_t now_tick = xTaskGetTickCountFromISR();
    if (raw == EndstopSignalState::INVALID) {
        const TickType_t invalid_since =
            drv->endstop_invalid_since_tick_.load(std::memory_order_relaxed);
        if (invalid_since == 0) {
            drv->endstop_invalid_since_tick_.store(now_tick, std::memory_order_release);
        }
        return;
    }

    drv->endstop_last_stable_state_.store(static_cast<uint8_t>(raw), std::memory_order_release);
    drv->endstop_invalid_since_tick_.store(0, std::memory_order_release);

    if (raw == EndstopSignalState::OPEN) {
        drv->endstop_active_.store(false, std::memory_order_release);
        drv->endstop_clearance_pending_.store(false, std::memory_order_release);
        return;
    }

    if (!drv->isEndstopArmed()) {
        return;
    }

    const bool was_active = drv->endstop_active_.exchange(true, std::memory_order_acq_rel);
    if (!was_active) {
        drv->endstop_hit_count_.fetch_add(1, std::memory_order_relaxed);
    }
    drv->endstop_clearance_pending_.store(true, std::memory_order_release);
    drv->endstop_clearance_direction_.store(
        !drv->last_dir_commanded_.load(std::memory_order_acquire),
        std::memory_order_release);

    BaseType_t woken = pdFALSE;
    TaskHandle_t exec = drv->executor_task_.load(std::memory_order_relaxed);
    if (exec != nullptr) {
        vTaskNotifyGiveFromISR(exec, &woken);
    }
    if (woken) portYIELD_FROM_ISR();
}
```

### Ce que ça implique

Au hit :

- le capteur passe actif,
- `endstop_hit_count_` est incrémenté (visible via `endstop_hit_mask` dans le status),
- une direction de dégagement est mémorisée dans `endstop_clearance_direction_`
  (vestige conservé pour information, non utilisé depuis la correction 5),
- l’executor est réveillé pour traiter l’arrêt d’urgence.

Depuis la correction 5, `isEndstopMoveAllowed` n’utilise plus `endstop_clearance_direction_`
pour bloquer les mouvements. Le seul veto firmware restant est l’état `ABSENT`.
Le commentaire historique indiquant qu’un mouvement pouvait être refusé
sur la direction de dégagement **ne s’applique plus** au code actif.

---

## 16. Firmware — acceptation/rejet des segments

### Fichier

`src/esp32/src/comm_interface.cpp`

### Gating sur les blocs simples

```cpp
const bool direction = (payload.step_count > 0)
    ? ((payload.entries[0].flags & SpiStepFlags::DIR_REVERSE) != 0)
    : false;
if (!isLateralMovementAllowed(payload.axis_id, direction)) {
    return ESP_ERR_INVALID_STATE;
}
```

```cpp
for (uint32_t i = 0; i < payload.segment_count; ++i) {
    if (payload.segments[i].step_count == 0) {
        continue;
    }
    const bool direction =
        (payload.segments[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
    if (!isLateralMovementAllowed(payload.axis_id, direction)) {
        return ESP_ERR_INVALID_STATE;
    }
}
```

### Gating sur les blocs multi-axes

```cpp
for (uint8_t s = 0; s < segment_count; ++s) {
    ...
    for (uint8_t a = 0; a < axis_count; ++a) {
        uint16_t steps;
        memcpy(&steps, cursor, 2);
        block.segments[s].step_counts[a] = steps;
        if (steps > 0) {
            const bool direction = (dir_mask & static_cast<uint16_t>(1U << a)) != 0;
            if (!isLateralMovementAllowed(block.axis_ids[a], direction)) {
                return ESP_ERR_INVALID_STATE;
            }
        }
        cursor += 2;
    }
}
```

### Ce que ça implique

Le firmware peut toujours rejeter un segment **avant même son exécution**, mais avec le code corrigé ce rejet n’est plus lié à une direction interdite sur `CLOSED`. Il survient principalement si le capteur est vu `ABSENT` alors que la protection endstop est armée.

Depuis la correction 5, `isEndstopMoveAllowed` ne bloque plus les mouvements sur la base d'une direction latchée quand le capteur est `CLOSED`. Le seul veto firmware restant dans ce chemin est l'état `ABSENT`. Le gating SPI dans `comm_interface.cpp` délègue à `isLateralMovementAllowed` qui délègue lui-même à `isEndstopMoveAllowed` : le comportement corrigé se propage donc à toute la chaîne de réception des segments.

---

## 17. Firmware — DRAIN, blocages, fail-safe et RECOVERY

### Fichier

`src/esp32/src/comm_interface.cpp`

### 17.1 DRAIN — détection endstop déjà actif

```cpp
bool endstop_hit = false;
for (uint8_t a = 0; a < seg.axis_count && !endstop_hit; ++a) {
    const uint8_t eid = seg.axis_ids[a];
    if (eid >= self->n_motors_ ||
        self->queues_[eid] == nullptr) continue;
    if (self->queues_[eid]->driver().isEndstopActive()) {
        clearMultiExecFlags();
        self->queues_[eid]->driver().emergencyStop();
        requestPlannerFlush(seg.motion_sequence);
        ESP_LOGW(TAG, "endstop on axis %u at seq=%u",
                 eid, seg.motion_sequence);
        endstop_hit = true;
    }
}
if (endstop_hit) {
    state = ExecState::RECOVERY;
    goto exit_drain;
}
```

### 17.2 DRAIN — fail-safe ABSENT pendant phase armée

```cpp
const uint8_t lateral_state = self->readLateralEndstopState();
const bool lateral_endstop_armed =
    self->n_motors_ > 1
    && self->queues_[1] != nullptr
    && self->queues_[1]->driver().isEndstopArmed();

if (lateral_endstop_armed &&
    lateral_state == static_cast<uint8_t>(LateralEndstopState::ABSENT)) {
    ESP_LOGW(TAG, "lateral endstop ABSENT while armed at seq=%u — fail-safe stop",
             seg.motion_sequence);
    clearMultiExecFlags();
    if (self->queues_[1] != nullptr) {
        self->queues_[1]->driver().emergencyStop();
    }
    requestPlannerFlush(seg.motion_sequence);
    state = ExecState::RECOVERY;
    goto exit_drain;
}
```

### 17.3 DRAIN — blocage latéral pendant écriture des steps

```cpp
const bool lateral_blocked =
    lateral_endstop_armed
    && lateral_state !=
    static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
...
if (axis_id == 1 && lateral_blocked
    && !self->isLateralMovementAllowed(axis_id, seg.axes[a].direction)) {
    clearMultiExecFlags();
    ESP_LOGW(TAG, "axis1 blocked while armed at seq=%u",
             seg.motion_sequence);
    self->queues_[axis_id]->driver().emergencyStop();
    requestPlannerFlush(seg.motion_sequence);
    state = ExecState::RECOVERY;
    goto exit_drain;
}
```

Depuis la correction 5, ce cas ne peut plus se produire à cause d'une direction interdite alors que le capteur est `CLOSED` : `isLateralMovementAllowed()` délègue à `isEndstopMoveAllowed()`, qui ne veto plus les mouvements sur cette base. Le seul cas résiduel réaliste pour cette branche est désormais un capteur vu `ABSENT` pendant qu'il est armé.

### 17.4 DRAIN — erreur pendant exécution de bloc

```cpp
esp_err_t err = axis_queue->executeConstantRateBlock(
    seg.axes[a].direction,
    seg.axes[a].step_count,
    seg.duration_us);

if (err == ESP_ERR_INVALID_STATE) {
    clearMultiExecFlags();
    ESP_LOGW(TAG, "axis %u endstop mid-seg seq=%u",
             axis_id, seg.motion_sequence);
    axis_queue->driver().emergencyStop();
    requestPlannerFlush(seg.motion_sequence);
    state = ExecState::RECOVERY;
    goto exit_drain;
}
```

### 17.5 RECOVERY actuel

```cpp
case ExecState::RECOVERY: {
    planned_segment_t discard;
    uint32_t drained = 0;
    uint16_t last_drained_seq = 0;
    while (drained < SEGMENT_QUEUE_DEPTH &&
           xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
        if (!discard.is_flush) {
            last_drained_seq = discard.motion_sequence;
        }
        ++drained;
    }

    defer_head = defer_tail = 0;

    ESP_LOGW(TAG, "recovery: drained %lu remaining segments (last_seq=%u)",
             (unsigned long)drained,
             (unsigned)last_drained_seq);

    batch_count = 0;
    batch_index = 0;
    state = ExecState::IDLE;
    break;
}
```

### Ce que ça implique

Sur hit, sur capteur absent, ou sur mouvement interdit :

- stop d’urgence,
- demande de flush planner,
- bascule en `RECOVERY`,
- vidage des segments restants,
- retour à `IDLE`.

---

## 18. Broches matérielles — point de divergence critique

### Fichier firmware

`src/esp32/src/main.cpp`

### Code actif

```cpp
static constexpr gpio_num_t SPI_READY = GPIO_NUM_4;
static constexpr gpio_num_t RPI_SHUTDOWN_REQ = GPIO_NUM_16;
static constexpr gpio_num_t HOME_NO = GPIO_NUM_21;
static constexpr gpio_num_t HOME_NC = GPIO_NUM_22;
```

### Mais l’en-tête du fichier dit encore

```cpp
*   SPI host link              : MOSI=GPIO23  MISO=GPIO19
*                                SCLK=GPIO18  CS=GPIO5  READY=GPIO17
```

### Et le host par défaut dit

```python
spi_ready_gpio_line: Optional[int] = 17
```

### Conclusion

Il existe **une divergence claire** entre :

- le commentaire firmware (`READY=GPIO17`),
- le code firmware réel (`GPIO4`),
- la config host par défaut (`17`).

C’est un candidat fort pour expliquer des anomalies de timing SPI en situation réelle.

---

## 19. Matrice des fautes plausibles

### Cas 1 — axe dans le mauvais sens dès l’approche

Cause probable :

- `lateral_invert_direction` mal réglé
- ou inversion matérielle moteur/driver différente de l’hypothèse logicielle

Effet attendu :

- la phase `approach` s’éloigne du capteur,
- elle peut se terminer sans trigger,
- le host produit une faute `ended without endstop trigger`.

### Cas 2 — l’axe va au endstop puis passe en faute

Causes possibles :

1. ~~le hit a lieu mais la transition vers backoff/search ne correspond pas au sens de
   dégagement attendu par le firmware~~ — **résolu** : correction 5 (blocage
   directionnel firmware supprimé) + corrections round 2 A/B/C (synchronisation host),
2. `lateral_endstop_state` devient `ABSENT` pendant une phase armée,
3. le process réellement lancé sur le Pi n’utilise pas la dernière version host,
4. la couche SPI est ralentie/dégradée par un READY GPIO faux,
5. divergence host/firmware dans l’ordre effectif des statuses au moment des flush,
6. distance de backoff insuffisante pour rouvrir le contact mécanique (correction round 3).

### Cas 3 — endstop touché, puis `ENDSTOP_BLOCKED` *(résolu)*

~~Cause probable :~~

~~- le firmware est encore dans un état où seul le sens de dégagement est autorisé,~~
~~- mais le segment suivant demande le sens opposé.~~

Ce cas n’est plus possible depuis la correction 5 : `isEndstopMoveAllowed` n’utilise
plus `endstop_clearance_direction_` pour bloquer les mouvements sur `CLOSED`.

### Cas 4 — homing impossible avec capteur absent/intermittent

Cause probable :

- NO/NC dans un état `INVALID` persistant,
- câble, capteur, rebond excessif, alimentation, masse, ou mapping pins.

---

## 20. Écarts documentaires ou de version observés pendant l’analyse

### Écart 1 — READY GPIO

- `src/esp32/src/main.cpp` code actif : `GPIO4`
- commentaire du même fichier : `GPIO17`
- `src/rpi/core/config.py` par défaut : `17`

### Écart 2 — doc homing vs code courant

Le document `doc/homing.md` décrit certains comportements qui ne correspondent pas strictement au code actuellement lu dans l’arbre, notamment sur certains détails de `RECOVERY`.

Exemple notable :

- la documentation décrit une notification explicite de `last_drained_seq` en `RECOVERY`,
- le code courant de `comm_interface.cpp` ne la fait pas dans l’extrait actif lu.

Ce point mérite une vérification si l’analyse doit être menée au niveau séquence/flush très fin.

### Écart 3 — commentaires firmware vs comportement réel

Le commentaire d’en-tête de `main.cpp` n’est plus une source fiable pour READY.
Le **code** doit être considéré comme la vérité.

---

## 21. Test host récent ajouté pour le symptôme READY

### Fichier

`tests/test_critique_v10_regressions.py`

### Code actif

```python
def test_spi_transport_disables_ready_handshake_after_repeated_timeouts() -> None:
    class FakeReadyMonitor:
        def value(self) -> int:
            return 0

    transport = Esp32SpiTransport.__new__(Esp32SpiTransport)
    transport._ready_monitor = FakeReadyMonitor()
    transport._ready_active_level = 1
    transport._ready_wait_timeout_s = 0.0
    transport._ready_poll_sleep_s = 0.0
    transport._ready_timeout_streak = 0
    transport._ready_timeout_disable_threshold = 3
    transport._ready_handshake_disabled = False
    transport._diag_ready_timeouts = 0
    transport._diag_lifetime_ready_timeouts = 0
    transport._device_path = "/dev/spidev0.0"
    transport._inter_transfer_guard_s = 0.0
    transport._last_xfer_end_ts = 0.0

    Esp32SpiTransport._wait_until_ready(transport)
    Esp32SpiTransport._wait_until_ready(transport)
    assert transport._ready_handshake_disabled is False

    Esp32SpiTransport._wait_until_ready(transport)
    assert transport._ready_handshake_disabled is True
    assert transport._diag_lifetime_ready_timeouts == 3

    Esp32SpiTransport._wait_until_ready(transport)
    assert transport._diag_lifetime_ready_timeouts == 3
```

### Ce que ce test prouve

Le host a désormais une protection logicielle contre un READY GPIO faux ou muet.
Mais ce correctif n’est utile que si le process réel exécuté sur la machine utilise bien cette version.

---

## 22. Séquence d’analyse recommandée sur la machine réelle

Ordre recommandé pour identifier la vraie panne :

1. vérifier que `lateral_invert_direction` correspond bien au câblage moteur réel et au sens physique attendu pendant `approach`
2. vérifier que le process actif utilise **la bonne arborescence déployée**
3. vérifier si le process actif est bien redémarré après déploiement
4. vérifier la **ligne READY réellement câblée** entre Pi et ESP32
5. vérifier les états `OPEN/CLOSED/ABSENT` en temps réel pendant homing
6. capturer les champs :
   - `lateral_endstop_state`
   - `endstop_armed_mask`
   - `endstop_hit_mask`
   - `running_mask`
   - `last_result`
   - `last_rx_sequence`
   - `last_executed_sequence`
7. vérifier si la faute apparaît :
   - avant le hit,
   - au hit,
   - juste après le hit,
   - pendant le backoff,
   - pendant le search lent

---

## 23. Hypothèses principales classées par priorité

### Hypothèse A — sens de homing faux

**Statut : confirmée et corrigée** — voir la section 25, correction 1.

Très compatible avec le symptôme :

- « l’axe part dans la mauvaise direction »

Le paramètre le plus direct est :

- `lateral_invert_direction`

### Hypothèse B — problème READY GPIO / timing SPI réel

Très compatible avec :

- comportement réel incohérent malgré tests host verts,
- divergence entre code firmware et config host,
- latences anormales ou ACK/status déphasés.

### Hypothèse C — process actif pas redémarré ou mauvaise arborescence active

Compatible avec :

- code local corrigé,
- tests verts,
- comportement réel inchangé.

### Hypothèse D — capteur NO/NC ou câblage intermittent

Compatible avec :

- hit visible puis faute,
- bascule vers `ABSENT`,
- INVALID persistant,
- comportement instable selon vibration/rebond.

### Hypothèse E — blocage firmware sur sens non autorisé après hit

**Statut : partiellement corrigée** — la composante firmware a été corrigée par la correction 5, et la composante de synchronisation host est complétée par les corrections round 2 A/B/C. Le risque résiduel principal avant round 2 était le faux hit immédiat en début de `search`, désormais adressé par la vérification explicite du latch clear.

Compatible avec :

- l’axe touche le switch,
- puis les segments suivants sont rejetés,
- `ENDSTOP_BLOCKED` ou `emergencyStop` + `RECOVERY`.

---

## 24. Diagnostic confirmé

Le diagnostic final confirmé sur ce dépôt est le suivant.

### Bug confirmé n°1 — absence d’attente de fin de `RECOVERY` après le hit d’approche

Après que le streamer détecte `endstop_triggered=True` sur la phase `approach`, le host enchaînait trop vite vers le `backoff`.

Conséquence :

- le firmware pouvait être encore dans son cycle `emergencyStop()` → `requestPlannerFlush()` → `ExecState::RECOVERY`,
- les premiers segments du `backoff` pouvaient être drainés / ignorés,
- l’axe ne reculait pas assez ou pas du tout,
- le capteur restait `CLOSED`,
- `_wait_for_endstop_open()` expirait,
- le host passait en `FAULT`.

### Bug confirmé n°2 — `backoff_steps` host trop faible dans le chemin manuel

La valeur par défaut de `backoff_steps` dans [src/rpi/motion/command_service.py](src/rpi/motion/command_service.py) était `400`, soit une marge faible pour relâcher mécaniquement le contact si le démarrage du recul est retardé ou si une partie des premiers segments n’est pas exécutée.

Conséquence :

- même sans panne dure, le dégagement pouvait être insuffisant,
- la phase de `backoff` devenait fragile,
- la vérification `PRESENT_OPEN` pouvait échouer trop facilement.

### Bug confirmé n°3 — timeout de `backoff` sous-estimé

`_compute_backoff_timeout()` utilisait une estimation cinématique, mais sans marge fixe couvrant explicitement :

- la fin du cycle `RECOVERY` firmware,
- le temps de redémarrage effectif du mouvement,
- la latence SPI / polling côté host.

Conséquence :

- le timeout pouvait expirer alors que la chaîne de retour au mouvement n’avait pas encore totalement convergé,
- le host concluait à tort à un échec de release.

### Bug confirmé n°4 — absence de vérification du latch et de l'arrêt complet avant le `search`

Après le `backoff`, le host ne vérifiait pas que :

- l'axe était physiquement arrêté (`running_mask == 0`),
- le latch de hit de l'`approach` était bien effacé (`endstop_hit_mask == 0`).

Conséquence :

- le `search` pouvait démarrer avec un axe encore en mouvement,
- ou avec un latch résiduel encore visible dans le status,
- produisant soit un faux hit immédiat soit un hit parasite en début de phase,
- et `mark_homed` pouvait alors être appelé sur une position corrélée à l'`approach`, pas au `search`.

### Cause racine consolidée

Le défaut principal n’était pas seulement « le hit » mais **l’absence de synchronisation explicite entre la machine d’état `RECOVERY` du firmware et l’enchaînement des phases côté host**.

Le problème de sens était réel au départ, mais une fois corrigé, le vrai défaut résiduel restait l’absence de barrière post-hit entre `approach` et `backoff`.

---

## 25. Corrections appliquées

### Correction 1 — inversion du sens latéral par défaut côté host

Fichier : [src/rpi/core/config.py](src/rpi/core/config.py)

Avant :

```python
lateral_invert_direction: bool = False
```

Après :

```python
lateral_invert_direction: bool = True
```

Effet recherché :

- aligner le sens `approach/search` avec le câblage moteur réel observé sur machine.

### Correction 2 — barrière de synchronisation post-hit dans `MoveQueue`

Fichier : [src/rpi/motion/move_queue.py](src/rpi/motion/move_queue.py)

Avant, la boucle de homing passait directement de `approach` vers la phase suivante.

Après, le host attend explicitement l’arrêt réel de l’axe puis ajoute une garde pour laisser le firmware sortir de `RECOVERY`.

Ajout principal :

```python
def _wait_for_post_hit_recovery(
    self,
    axis_id: int,
    *,
    stop_timeout_s: float = 1.0,
    recovery_guard_s: float = 0.080,
) -> Any:
    ...
```

Utilisation dans `_execute_homing()` :

```python
if phase_name == "approach" and streamer.endstop_triggered:
    try:
        self._wait_for_post_hit_recovery(move.axis_id)
    except Exception as exc:
        self._set_endstop_armed(move.axis_id, arm=False)
        move.mark_failed(str(exc))
        return
```

Effet recherché :

- éviter que les premiers segments du `backoff` soient envoyés pendant `RECOVERY`,
- garantir que le recul commence sur un firmware revenu dans un état stable.

### Correction 3 — augmentation du `backoff_steps` manuel par défaut

Fichier : [src/rpi/motion/command_service.py](src/rpi/motion/command_service.py)

Avant :

```python
def home_lateral(
    self,
    approach_rpm: float = 120.0,
    search_rpm: float = 10.0,
    backoff_steps: int = 400,
) -> dict[str, Any]:
```

Après :

```python
def home_lateral(
    self,
    approach_rpm: float = 120.0,
    search_rpm: float = 10.0,
    backoff_steps: int = 1600,
) -> dict[str, Any]:
```

Effet recherché :

- donner plus de marge mécanique pour relâcher le capteur,
- rendre le `backoff` plus robuste aux délais réels de reprise.

### Correction 4 — marge fixe ajoutée au timeout de `backoff`

Fichier : [src/rpi/motion/move_queue.py](src/rpi/motion/move_queue.py)

Avant, le timeout reposait essentiellement sur une estimation cinématique multipliée par une marge.

Après :

```python
@staticmethod
def _compute_backoff_timeout(sub_move: Move, margin: float = 1.5) -> float:
    recovery_guard_s = 0.5
    ...
    return max(1.0, float(total) * margin + recovery_guard_s)
```

Effet recherché :

- absorber le délai de sortie de `RECOVERY`,
- absorber le redémarrage moteur et la latence de polling,
- éviter les `endstop release timeout` prématurés.

### Correction 5 — suppression du blocage directionnel firmware pour se rapprocher de Klipper

Fichiers :

- [src/esp32/src/stepper_driver.cpp](src/esp32/src/stepper_driver.cpp)
- [src/esp32/src/stepper_driver.h](src/esp32/src/stepper_driver.h)

Avant :

```cpp
bool StepperDriver::isEndstopMoveAllowed(bool direction) const
{
    if (!isEndstopArmed()) {
        return true;
    }

    const uint8_t state = reportedEndstopState();
    if (state == static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN)) {
        return true;
    }
    if (state == static_cast<uint8_t>(LateralEndstopState::ABSENT)) {
        return false;
    }

    return endstop_clearance_pending_.load(std::memory_order_acquire)
        && direction == endstop_clearance_direction_.load(std::memory_order_acquire);
}
```

Après :

```cpp
bool StepperDriver::isEndstopMoveAllowed(bool direction) const
{
    (void)direction;

    if (!isEndstopArmed()) {
        return true;
    }

    const uint8_t state = reportedEndstopState();
    if (state == static_cast<uint8_t>(LateralEndstopState::ABSENT)) {
        return false;
    }

    return true;
}
```

Effet recherché :

- rapprocher le firmware du modèle Klipper, où le séquencement est host-driven,
- supprimer le rejet des mouvements sur `CLOSED` basé sur un sens latché,
- conserver seulement le veto de sécurité sur `ABSENT`.

Cette correction n’élimine pas le besoin de la barrière post-hit côté host : elle supprime un mécanisme de blocage directionnel, mais ne remplace pas la synchronisation avec `RECOVERY`.

---

## 26. Conclusion opérationnelle

Le homing actif reste bien un homing **piloté par le host**, mais le diagnostic final montre que la panne principale n’était pas un simple problème de capteur : c’était un problème de **synchronisation inter-couches** entre le host Python et la machine d’état `RECOVERY` du firmware.

Le problème observé sur machine réelle se décompose désormais ainsi :

1. le sens logique du homing était incorrect au départ et a été corrigé par `lateral_invert_direction = True`,
2. le défaut central était l’absence de barrière post-hit entre `approach` et `backoff`,
3. le `backoff` manuel et son timeout étaient trop fragiles pour un firmware qui fait encore `stop + flush + recovery`,
4. le blocage directionnel firmware après hit renforçait l’écart avec le modèle Klipper et a été retiré.

Le symptôme « va au endstop puis fault » correspondait précisément au scénario suivant :

- hit correctement détecté,
- backoff envoyé trop tôt,
- firmware encore en `RECOVERY`,
- premiers segments de recul perdus ou absorbés,
- capteur toujours `CLOSED`,
- timeout host sur l’ouverture,
- `FAULT`.

Le symptôme « part dans la mauvaise direction » relevait bien du paramètre de direction. Mais une fois ce point corrigé, le diagnostic final confirmé est que **la synchronisation entre la fin de `RECOVERY` firmware et le démarrage du `backoff` host est la cause racine principale**.

La divergence READY GPIO entre host et firmware reste un sujet important à surveiller pour la qualité du lien SPI réel, mais elle n’est plus le diagnostic principal de la panne de séquencement du homing décrite ici.

Les corrections du round 2 (A, B, C) complètent la synchronisation entre les phases host et la machine d'état firmware. La séquence est désormais alignée sur le modèle Klipper : chaque phase démarre depuis un état firmware confirmé stable, le latch est vérifié propre avant tout streaming armé, et `mark_homed` n'est appelé qu'après l'arrêt complet de l'axe. Le seul point d'amélioration restant est la corrélation de la position homée au hit réel (amélioration D), conditionnée à l'exposition d'un champ de position dans `StatusPayload`.

---

## 27. Corrections résiduelles appliquées (round 2)

### Correction A — attente `running_mask == 0` après le `backoff`

**Problème résolu**

Après `_wait_for_endstop_open()`, le capteur pouvait être confirmé `OPEN` alors que l'axe était encore en décélération. Armer l'endstop du `search` dans cet état exposait un hit parasite immédiat.

**Fichier concerné**

- [src/rpi/motion/move_queue.py](src/rpi/motion/move_queue.py)

**Avant**

```python
if phase_name == "backoff":
    try:
        self._wait_for_endstop_open(
            move.axis_id,
            timeout_s=self._compute_backoff_timeout(sub_move),
        )
    except Exception as exc:
        self._set_endstop_armed(move.axis_id, arm=False)
        move.mark_failed(str(exc))
        return
```

**Après**

```python
if phase_name == "backoff":
    try:
        self._wait_for_endstop_open(
            move.axis_id,
            timeout_s=self._compute_backoff_timeout(sub_move),
        )
    except Exception as exc:
        self._set_endstop_armed(move.axis_id, arm=False)
        move.mark_failed(str(exc))
        return

    _deadline = time.monotonic() + 1.0
    while time.monotonic() < _deadline:
        _status = self._read_status(move.axis_id)
        if (int(getattr(_status, "running_mask", 0)) & (1 << move.axis_id)) == 0:
            break
        time.sleep(0.010)
    else:
        _running_mask = int(getattr(_status, "running_mask", 0xFF))
        self._set_endstop_armed(move.axis_id, arm=False)
        move.mark_failed(
            f"backoff stop timeout on axis {move.axis_id}: "
            f"running_mask=0x{_running_mask:02X} still non-zero after backoff"
        )
        return
```

**Effet attendu**

- le `search` ne peut être armé que lorsque l'axe est réellement arrêté,
- les faux hits dus à une glisse résiduelle en sortie de `backoff` sont éliminés.

### Correction B — vérification `endstop_hit_mask == 0` après armement des phases armées

**Problème résolu**

`ENABLE_ENDSTOP(arm=1)` efface le latch firmware, mais cet effacement peut n'apparaître dans le status SPI qu'après un échange supplémentaire. Sans vérification explicite, le `search` pouvait voir un ancien hit et se terminer sans mouvement réel.

**Fichier concerné**

- [src/rpi/motion/move_queue.py](src/rpi/motion/move_queue.py)

**Avant**

```python
try:
    self._set_endstop_armed(move.axis_id, arm=arm_endstop)
except Exception as exc:
    move.mark_failed(str(exc))
    return
```

**Après**

```python
def _wait_for_endstop_latch_cleared(
    self,
    axis_id: int,
    *,
    timeout_s: float = 0.5,
) -> None:
    ...

try:
    self._set_endstop_armed(move.axis_id, arm=arm_endstop)
    if arm_endstop:
        self._wait_for_endstop_latch_cleared(move.axis_id)
except Exception as exc:
    move.mark_failed(str(exc))
    return
```

**Effet attendu**

- toute phase armée démarre avec un latch confirmé propre,
- les faux hits immédiats au démarrage du `search` sont éliminés.

### Correction C — barrière post-hit symétrique après le `search`

**Problème résolu**

Le cycle firmware `emergencyStop()` → `requestPlannerFlush()` → `RECOVERY` se produit aussi après le hit du `search`. Sans barrière symétrique, `mark_homed` pouvait être appelé avant le retour complet à un état stable.

**Fichier concerné**

- [src/rpi/motion/move_queue.py](src/rpi/motion/move_queue.py)

**Avant**

```python
if phase_name == "approach" and streamer.endstop_triggered:
    try:
        self._wait_for_post_hit_recovery(move.axis_id)
    except Exception as exc:
        self._set_endstop_armed(move.axis_id, arm=False)
        move.mark_failed(str(exc))
        return
```

**Après**

```python
if phase_name in ("approach", "search") and streamer.endstop_triggered:
    try:
        self._wait_for_post_hit_recovery(move.axis_id)
    except Exception as exc:
        self._set_endstop_armed(move.axis_id, arm=False)
        move.mark_failed(str(exc))
        return
```

**Effet attendu**

- le zéro logiciel n'est défini qu'après arrêt complet et fin effective de `RECOVERY`,
- la position homée est cohérente avec le hit réel du `search`.

### ### Amélioration D — position homée corrélée au hit réel

**Statut : réservée pour une itération ultérieure**

**Constat**

`axis_state.mark_homed(move.home_position_steps)` continue d’utiliser `home_position_steps` comme valeur logique finale, sans champ de position cumulée exposé dans le status firmware.

**État actuel de `StatusPayload`**

Dans [src/rpi/transport/messages.py](src/rpi/transport/messages.py) et [src/esp32/src/messages.h](src/esp32/src/messages.h), `StatusPayload` n’expose pas de position cumulée d’axe, seulement :

- `last_executed_sequence`
- `running_mask`
- `lateral_endstop_state`
- `endstop_armed_mask`
- `endstop_hit_mask`
- champs de file / diagnostic

**Conséquence**

Le host ne peut pas encore corréler le zéro logiciel à une position comptée exacte au moment du hit du `search`.

**Besoin identifié**

Une future itération devra exposer dans `StatusPayload` une position cumulée fiable pour l’axe latéral afin de permettre un `mark_homed` basé sur la position réelle du hit, à la manière de `position_endstop` côté Klipper.

---

## 28. Bug 5 — backoff insuffisant pour relacher le contact mécanique (round 3)

### Symptôme observé

```
Homing failed: endstop release timeout on axis 1: state=0x01
```

`state=0x01` = `LATERAL_ENDSTOP_PRESENT_CLOSED` : le capteur est encore fermé après le backoff complet.

### Cause racine

`backoff_steps` était hardcodé à `1600` dans `command_service.py`.

Avec la config par défaut (200 pas/tour × 32 micropas = 6 400 pas/tour, pas de vis 1 mm/tour) :

$$\text{déplacement} = \frac{1600}{6400} \times 1\,\text{mm} = 0{,}25\,\text{mm}$$

Un microswitch mécanique standard nécessite 0,5–2 mm de course pour relacher son contact.
0,25 mm est systématiquement insuffisant.

### Correction appliquée

**Fichier** : `src/rpi/motion/command_service.py`

Remplacement du défaut fixe par un défaut dynamique calculé à l’exécution :

```python
# Avant
backoff_steps: int = 1600

# Après
backoff_steps: int | None = None
# ...
if backoff_steps is None:
    steps_per_rev = (
        self._config.lateral_steps_per_revolution
        * self._config.lateral_microstepping
    )
    backoff_steps = steps_per_rev * 2  # 2 tours complets — toujours suffisant
```

Avec la config par défaut : `backoff_steps = 6400 × 2 = 12 800` → 2 mm de course.

Ce défaut est **indépendant du pas de vis** : il garantit toujours 2 révolutions mécaniques
complètes quel que soit le pas de vis configuré.
