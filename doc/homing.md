# Homing et endstop — flux complet host / firmware

Ce document décrit **tout le processus de homing latéral** dans PickupWinder, côté **Python host (Raspberry Pi)** et côté **firmware ESP32**.

Il couvre :

- le point d'entrée RPC,
- l'orchestration moteur,
- la construction du `HomingMove`,
- l'armement / désarmement de l'endstop,
- la détection du contact,
- le `FLUSH` et le `RECOVERY`,
- le `backoff`,
- la mise à jour de l'état applicatif,
- les statuts SPI utilisés,
- les actions et méthodes liées au homing et à l'endstop.

---

## 1. Vue d'ensemble

Le homing latéral est un flux en deux parties :

- **Host Python** : décide quand lancer le homing, découpe le mouvement en phases, arme/désarmer l'endstop, surveille les statuts et déclare le succès ou l'échec.
- **ESP32** : reçoit les blocs de segments via SPI, lit l'endstop physique, stoppe le moteur en temps réel si nécessaire, vide les queues et renvoie l'état courant dans `StatusPayload`.

Le homing latéral suit trois phases fonctionnelles :

1. **Approach** : avance rapide vers la butée, endstop armé.
2. **Backoff** : recul pour libérer la butée, endstop désarmé.
3. **Search** : approche lente pour définir précisément la position home, endstop armé.

Quand les trois phases réussissent, la position Python de l'axe est marquée comme homée.

---

## 2. Fichiers impliqués

## 2.1 Host Python

- `src/rpi/jsonrpc/winding_handler.py`
- `src/rpi/motion/engine.py`
- `src/rpi/motion/move.py`
- `src/rpi/motion/move_queue.py`
- `src/rpi/motion/axis_state.py`
- `src/rpi/transport/streamer.py`
- `src/rpi/transport/spi_transport.py`
- `src/rpi/transport/messages.py`

## 2.2 Firmware ESP32

- `src/esp32/src/messages.h`
- `src/esp32/src/comm_interface.cpp`
- `src/esp32/src/stepper_driver.h`
- `src/esp32/src/stepper_driver.cpp`
- `src/esp32/src/motion_planner.cpp`

---

## 3. Point d'entrée côté host

Le point d'entrée public du homing est la méthode JSON-RPC `winding.home_lateral`.

### Fichier

`src/rpi/jsonrpc/winding_handler.py`

### Méthode

```python
def home_lateral(
    self,
    approach_rpm: float = 100.0,
    search_rpm: float = 20.0,
    backoff_steps: int = 3200,
) -> dict[str, Any]:
    axis_state = self._engine.home_lateral(
        approach_rpm=approach_rpm,
        search_rpm=search_rpm,
        backoff_steps=backoff_steps,
    )
    return {
        "status": "homed",
        "axis_state": axis_state,
    }
```

### Rôle

- expose le homing latéral à l'API JSON-RPC,
- délègue tout le travail à `WindingEngine.home_lateral(...)`.

---

## 4. Orchestration moteur côté host

### Fichier

`src/rpi/motion/engine.py`

### Méthode publique

```python
def home_lateral(
    self,
    approach_rpm: float = _DEFAULT_HOME_APPROACH_RPM,
    search_rpm: float = _DEFAULT_HOME_SEARCH_RPM,
    backoff_steps: int = _DEFAULT_HOME_BACKOFF_STEPS,
) -> dict[str, Any]:
    if self._state.engine_state != EngineState.IDLE:
        raise RuntimeError(...)

    self._state.set_engine_state(EngineState.HOMING)
    success, reason = self._home_lateral_axis(
        axis_id=self._config.lateral_axis_id,
        approach_rpm=approach_rpm,
        search_rpm=search_rpm,
        backoff_steps=backoff_steps,
    )
    if not success:
        if reason:
            raise RuntimeError(f"Lateral homing failed: {reason}")
        raise RuntimeError("Lateral homing failed")

    self._state.set_engine_state(EngineState.IDLE)
    return self._require_axis_state(self._config.lateral_axis_id).snapshot()
```

### Méthode interne réellement utilisée

```python
def _home_lateral_axis(
    self,
    *,
    axis_id: int,
    approach_rpm: float,
    search_rpm: float,
    backoff_steps: int,
) -> tuple[bool, str | None]:
    self._events.publish(EventKind.HOMING_STARTED, axis_id=axis_id)
    steps_per_rev = (
        self._config.lateral_steps_per_revolution
        * self._config.lateral_microstepping
    )
    move = HomingMove(
        name="home_lateral",
        axis_id=axis_id,
        steps_per_rev=steps_per_rev,
        approach_rpm=approach_rpm,
        search_rpm=search_rpm,
        backoff_steps=backoff_steps,
        max_approach_steps=int(steps_per_rev * 20),
        reverse_direction=self._config.lateral_invert_direction,
    )
    self._move_queue.enqueue(move)
    self._wait_for_move_queue()

    if move.state.name == "COMPLETED":
        self._events.publish(EventKind.HOMING_COMPLETED, axis_id=axis_id)
        return True, None
    else:
        msg = f"Homing failed: {move.error}"
        self._state.set_fault(msg)
        self._events.publish(...)
        return False, move.error
```

### Rôle

- construit le `HomingMove`,
- l'enfile dans `MoveQueue`,
- attend la fin,
- convertit le résultat en succès/échec de haut niveau,
- met le contrôleur en `FAULT` si le homing échoue.

---

## 5. Définition des phases de homing

### Fichier

`src/rpi/motion/move.py`

### Classe concernée

`HomingMove`

### Phases

```python
def phases(self) -> list[tuple[str, RampMove, bool]]:
    return [
        ("approach", self._make_approach_move(), True),
        ("backoff", self._make_backoff_move(), False),
        ("search", self._make_search_move(), True),
    ]
```

### Méthodes de génération des sous-mouvements

- `_make_approach_move()`
- `_make_backoff_move()`
- `_make_search_move()`

### Rôle

- `approach` : va vers la butée à `approach_rpm`, endstop armé,
- `backoff` : repart en sens inverse à `search_rpm`, endstop désarmé,
- `search` : revient lentement vers la butée à `search_rpm`, endstop armé.

Le booléen du tuple indique explicitement à `MoveQueue` s'il faut armer ou non l'endstop pour la phase.

---

## 6. Exécution réelle du homing côté host

### Fichier

`src/rpi/motion/move_queue.py`

C'est le fichier central du homing côté Python.

## 6.1 Vérification préalable du capteur

### Méthode

```python
def _ensure_homing_can_start(self, axis_id: int, phase_name: str) -> None:
    status = self._read_status(axis_id)
    lateral_state = int(getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT))
    if lateral_state != LATERAL_ENDSTOP_PRESENT_OPEN:
        state_name = (
            "absent" if lateral_state == LATERAL_ENDSTOP_ABSENT else f"0x{lateral_state:02X}"
        )
        raise RuntimeError(
            f"homing {phase_name} cannot start on axis {axis_id}: lateral_endstop_state={state_name}"
        )
```

### Rôle

Avant le homing, et avant chaque phase armée (`approach`, `search`), le host exige :

- `lateral_endstop_state == PRESENT_OPEN`

Sinon le homing est refusé proprement.

---

## 6.2 Armement / désarmement avec vérification

### Méthodes

```python
def _set_endstop_armed(self, axis_id: int, arm: bool) -> Any:
    sequence, _ = self._transport.enable_endstop_request(axis_id, arm=arm)
    status = self._transport.wait_for_request_result(
        sequence,
        poll_interval_s=self._poll_interval_s,
    )
    self._update_axis_endstop_state(axis_id, status)
    if int(getattr(status, "last_result", SpiMessageResult.OK)) != int(SpiMessageResult.OK):
        raise RuntimeError(...)
    return self._wait_for_endstop_arm_state(axis_id, arm)
```

```python
def _wait_for_endstop_arm_state(self, axis_id: int, arm: bool, timeout_s: float = 0.5) -> Any:
    deadline = time.monotonic() + timeout_s
    last_status = None
    while time.monotonic() < deadline:
        last_status = self._read_status(axis_id)
        if self._status_has_endstop_armed(last_status, axis_id, arm):
            return last_status
        time.sleep(self._poll_interval_s)
    raise RuntimeError(...)
```

### Rôle

Le host ne se contente pas d'envoyer `ENABLE_ENDSTOP`.

Il fait trois choses :

1. envoie `ENABLE_ENDSTOP arm=0|1`,
2. attend la confirmation via `wait_for_request_result()`,
3. vérifie ensuite que `endstop_armed_mask` reflète bien l'état demandé.

---

## 6.3 Attente de libération pendant le backoff

### Méthode

```python
def _wait_for_endstop_open(self, axis_id: int, timeout_s: float = 1.5) -> Any:
    deadline = time.monotonic() + timeout_s
    last_status = None
    while time.monotonic() < deadline:
        last_status = self._read_status(axis_id)
        if int(getattr(last_status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)) == LATERAL_ENDSTOP_PRESENT_OPEN:
            return last_status
        time.sleep(self._poll_interval_s)
    raise RuntimeError(
        f"endstop release timeout on axis {axis_id}: state=0x{int(getattr(last_status, 'lateral_endstop_state', 0xFF)):02X}"
    )
```

### Rôle

Après le `backoff`, le host attend que le capteur repasse en `PRESENT_OPEN` avant de considérer la phase comme réussie.

---

## 6.4 Boucle d'exécution du homing

### Méthode principale

```python
def _execute_homing(self, move: HomingMove) -> None:
    move.mark_running()
    axis_state = self._axis_states.get(move.axis_id)

    self._ensure_homing_can_start(move.axis_id, "start")

    for phase_name, sub_move, arm_endstop in move.phases():
        ...
        if arm_endstop:
            self._ensure_homing_can_start(move.axis_id, phase_name)

        self._set_endstop_armed(move.axis_id, arm=arm_endstop)

        streamer = self._make_streamer(
            sub_move.axis_configs,
            keep_enabled_axes={move.axis_id},
        )
        streamer.note_endstop_armed(move.axis_id, arm_endstop)
        streamer.set_generator(
            self._wrap_segment_sequence(
                sub_move.segments(),
                self._next_motion_sequence(),
            )
        )
        streamer.stream_all()

        if phase_name in ("approach", "search") and not streamer.endstop_triggered:
            ...

        if phase_name == "backoff":
            self._wait_for_endstop_open(move.axis_id)

    self._set_endstop_armed(move.axis_id, arm=False)
    if axis_state is not None:
        axis_state.mark_homed(move.home_position_steps)
    move.mark_completed()
```

### Ce que fait vraiment cette méthode

Pour chaque phase :

1. vérifie que le homing armé peut commencer,
2. arme ou désarme l'endstop selon la phase,
3. crée un `MultiAxisRampStreamer`,
4. injecte localement l'information `note_endstop_armed(...)`,
5. envoie les segments de mouvement,
6. vérifie que `approach` et `search` ont bien été arrêtés par l'endstop,
7. pour `backoff`, attend la réouverture du contact,
8. à la fin, désarme définitivement et marque l'axe comme homé.

---

## 7. Suivi de l'état logiciel de l'axe

### Fichier

`src/rpi/motion/axis_state.py`

### Méthodes liées au homing/endstop

```python
def update_endstop_state(self, state: int) -> None:
    self._endstop_state = state
```

```python
@property
def endstop_triggered(self) -> bool:
    return self._endstop_state == LATERAL_ENDSTOP_PRESENT_CLOSED
```

```python
def mark_homed(self, position_steps: int = 0) -> None:
    self._position_steps = position_steps
    self._homed = True
```

```python
def invalidate_position(self) -> None:
    self._position_steps = None
    self._homed = False
```

### Rôle

`AxisState` est le miroir applicatif Python de l'état latéral :

- position connue ou inconnue,
- axe homé ou non,
- état du capteur,
- détection logicielle `endstop_triggered`.

---

## 8. Transport SPI côté host

### Fichier

`src/rpi/transport/spi_transport.py`

## 8.1 Actions liées à l'endstop

```python
def arm_endstop(self, axis_id: int) -> StatusPayload:
    return self.transfer_frame(
        make_enable_endstop(EnableEndstopPayload(axis_id=axis_id, arm=True), self._next_sequence())
    )
```

```python
def disarm_endstop(self, axis_id: int) -> StatusPayload:
    return self.transfer_frame(
        make_enable_endstop(EnableEndstopPayload(axis_id=axis_id, arm=False), self._next_sequence())
    )
```

```python
def enable_endstop_request(self, axis_id: int, arm: bool) -> tuple[int, StatusPayload]:
    return self.transfer_request(
        make_enable_endstop(EnableEndstopPayload(axis_id=axis_id, arm=arm), self._next_sequence())
    )
```

### Rôle

- construit les trames SPI pour `ENABLE_ENDSTOP`,
- permet soit un envoi simple, soit un envoi avec confirmation différée.

## 8.2 Action liée au `FLUSH`

```python
def flush_until(self, sequence: int) -> StatusPayload:
    transport_sequence, _ = self.transfer_request(
        make_flush(FlushPayload(flush_sequence=sequence), self._next_sequence())
    )
    return self.wait_for_request_result(transport_sequence)
```

### Rôle

Le host peut demander au firmware de purger les segments encore présents dans les queues au-delà d'une séquence donnée.

---

## 9. Streamer host : détection du contact et arrêt

### Fichier

`src/rpi/transport/streamer.py`

Le streamer est la couche qui pousse les blocs multi-axes vers l'ESP32 et décide qu'un mouvement doit s'arrêter.

## 9.1 Marquage local d'une phase armée

```python
def note_endstop_armed(self, axis_id: int, arm: bool) -> None:
    if arm:
        self._endstop_armed_axes.add(axis_id)
    else:
        self._endstop_armed_axes.discard(axis_id)
```

### Rôle

Le `MoveQueue` informe le streamer que la phase courante attend un déclenchement endstop, même si le `status` firmware est en retard d'un transfert SPI.

## 9.2 Détection du déclenchement

```python
def _check_endstop(self, status) -> bool:
    armed_mask = int(getattr(status, "endstop_armed_mask", 0))
    lateral_state = int(getattr(status, "lateral_endstop_state", 0xFF))
    endstop_expected = armed_mask != 0 or bool(self._endstop_armed_axes)
    running_mask = int(getattr(status, "running_mask", 0))
    any_armed_axis_stopped = any(
        (running_mask & (1 << axis_id)) == 0
        for axis_id in self._endstop_armed_axes
    )
    if (
        lateral_state == LATERAL_ENDSTOP_PRESENT_CLOSED
        and endstop_expected
        and (any_armed_axis_stopped or armed_mask == 0)
    ):
        self._mark_endstop_triggered()
        return True
    return False
```

### Signaux utilisés

Le host considère qu'un endstop s'est déclenché quand :

- `lateral_endstop_state == PRESENT_CLOSED`,
- l'endstop était attendu (`endstop_armed_mask != 0` ou tracking local),
- et soit :
  - l'axe armé n'est plus `running`,
  - soit le firmware a déjà auto-nettoyé l'armement.

## 9.3 Que fait le streamer quand il détecte le contact ?

```python
def _mark_endstop_triggered(self) -> None:
    if self._endstop_triggered:
        return
    self._endstop_triggered = True
    flush_seq = self._last_sent_motion_seq
    self.request_stop()
    self.request_flush(flush_seq)
```

### Rôle

Le streamer :

- marque l'événement,
- arrête la boucle d'envoi,
- demande un `FLUSH` jusqu'à la dernière séquence envoyée.

## 9.4 Détection indirecte par `segments_dropped`

```python
if self._endstop_armed_axes:
    logger.info(
        "planner dropped segments while endstop is armed; treating as endstop-triggered recovery"
    )
    self._mark_endstop_triggered()
```

### Rôle

Si le firmware entre en `RECOVERY` et vide sa queue, `segments_dropped` peut augmenter.
Le host interprète cette situation comme un déclenchement d'endstop lorsqu'une phase armée était en cours.

## 9.5 Boucle principale de streaming

```python
def stream_all(self) -> int:
    status = self._transport.get_status()
    ...
    self._enable_axes()
    status = self._transport.get_status()

    self._check_endstop(status)
    if not self._stop_requested:
        total_segments, status = self._prefill(status)

    while True:
        if self._stop_requested:
            if self._flush_sequence_requested is not None:
                self.flush_until(self._flush_sequence_requested)
            break

        self._remove_confirmed_segments(status)
        self._log_runtime_diagnostics(status)
        if self._stop_requested:
            break
        self._check_premature_completion(status)
        if self._check_stall(status):
            break

        if self._check_endstop(status):
            break
        ...
```

### Rôle

- fait un pré-check endstop avant le `prefill`,
- surveille les signaux de contact pendant le mouvement,
- déclenche le `FLUSH` si un contact est détecté.

---

## 10. Miroir de protocole Python

### Fichier

`src/rpi/transport/messages.py`

### Constantes clés pour le homing

```python
LATERAL_ENDSTOP_PRESENT_OPEN = 0x00
LATERAL_ENDSTOP_PRESENT_CLOSED = 0x01
LATERAL_ENDSTOP_ABSENT = 0xFF
```

```python
class SpiMessageType(IntEnum):
    FLUSH = 0x12
    MULTI_AXIS_SEGMENT_BLOCK = 0x13
    ENABLE_ENDSTOP = 0x14
    STATUS = 0x80
```

```python
class SpiMessageResult(IntEnum):
    OK = 0x00
    ...
    ENDSTOP_BLOCKED = 0x09
```

### Rôle

Le host utilise ces constantes pour interpréter exactement les mêmes valeurs binaires que le firmware.

---

## 11. Protocole firmware : messages et status

### Fichier

`src/esp32/src/messages.h`

## 11.1 Message d'armement endstop

```cpp
struct __attribute__((packed)) EnableEndstopPayload {
    uint8_t axis_id;
    uint8_t arm;
    uint8_t reserved[2];
};
```

## 11.2 Type de message

```cpp
enum class SpiMessageType : uint8_t {
    ...
    ENABLE_ENDSTOP           = 0x14,
    ...
};
```

## 11.3 Champs de status utilisés pendant le homing

```cpp
struct __attribute__((packed)) StatusPayload {
    ...
    uint8_t  enabled_mask;
    uint8_t  running_mask;
    uint8_t  lateral_endstop_state;
    uint8_t  endstop_armed_mask;
    uint16_t last_executed_sequence;
    uint8_t  multi_axis_queue_free;
    uint8_t  planner_queue_free;
    uint16_t last_planned_sequence;
    uint16_t segments_dropped;
};
```

### Signification pour le homing

- `lateral_endstop_state` : état physique du capteur,
- `endstop_armed_mask` : axes dont la protection endstop est armée,
- `running_mask` : axes dont le RMT est actif,
- `last_executed_sequence` : dernière séquence réellement exécutée,
- `segments_dropped` : segments perdus / purgés côté planner.

---

## 12. Firmware : construction du status SPI

### Fichier

`src/esp32/src/comm_interface.cpp`

### Méthode

`CommInterface::buildStatusFrame(...)`

### Extrait

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

payload->last_executed_sequence = last_executed_sequence_.load(...);
payload->planner_queue_free = ...;
payload->last_planned_sequence = planner_.lastPlannedMotionSequence();
payload->segments_dropped = ...;
```

### Rôle

À chaque transfert SPI, l'ESP32 renvoie au host une photo de l'état runtime du moteur et du capteur.

---

## 13. Firmware : armement / désarmement de l'endstop

### Fichier

`src/esp32/src/comm_interface.cpp`

### Méthode

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

### Rôle

C'est le point de passage firmware pour `ENABLE_ENDSTOP`.

---

## 14. Firmware : lecture de l'endstop physique

### Fichier

`src/esp32/src/comm_interface.cpp`

### Méthode

```cpp
uint8_t CommInterface::readLateralEndstopState() const
{
    if (pins_.home_pin_no == GPIO_NUM_NC || pins_.home_pin_nc == GPIO_NUM_NC) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }

    const int no_state = gpio_get_level(pins_.home_pin_no);
    const int nc_state = gpio_get_level(pins_.home_pin_nc);

    if (no_state == nc_state) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }
    if (no_state == 0 && nc_state == 1) {
        return static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED);
    }
    return static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}
```

### Rôle

Le capteur est câblé en **double contact NO/NC**.
Le firmware renvoie :

- `PRESENT_OPEN` si le capteur est ouvert,
- `PRESENT_CLOSED` si la butée est contactée,
- `ABSENT` si le câblage est incohérent ou absent.

---

## 15. Firmware : garde passive de mouvement latéral

### Fichier

`src/esp32/src/comm_interface.cpp`

### Méthode

```cpp
bool CommInterface::isLateralMovementAllowed(uint8_t axis_id) const
{
    if (axis_id != 1) {
        return true;
    }
    if (axis_id >= n_motors_ || queues_[axis_id] == nullptr) {
        return false;
    }
    if (!queues_[axis_id]->driver().isEndstopArmed()) {
        return true;
    }
    return readLateralEndstopState() == static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}
```

### Rôle

Cette garde s'applique aux chemins `STEP_BLOCK` et `SEGMENT_BLOCK`, et plus largement au filtrage passif du latéral :

- si l'endstop est **armé**, un mouvement vers un capteur fermé est bloqué,
- si l'endstop est **désarmé**, le `backoff` est autorisé même si le capteur est encore physiquement fermé.

C'est précisément ce qui permet la phase `backoff` après un contact homing.

---

## 16. Firmware : ISR et arrêt temps réel

### Fichiers

- `src/esp32/src/stepper_driver.h`
- `src/esp32/src/stepper_driver.cpp`

## 16.1 Armement / désarmement dans le driver

```cpp
void armEndstop() {
    endstop_active_.store(false, std::memory_order_release);
    endstop_armed_.store(true, std::memory_order_release);
}

void disarmEndstop() {
    endstop_armed_.store(false, std::memory_order_release);
    endstop_active_.store(false, std::memory_order_release);
}
```

### Rôle

- `armEndstop()` arme la protection,
- `disarmEndstop()` retire la protection et efface aussi `endstop_active_`,
- ce reset explicite est indispensable pour que le `backoff` puisse repartir proprement.

## 16.2 ISR GPIO du capteur

```cpp
void IRAM_ATTR StepperDriver::endstopIsrHandler(void* arg)
{
    StepperDriver* drv = static_cast<StepperDriver*>(arg);

    if (!drv->isEndstopArmed()) {
        return;
    }

    const int no_lvl = gpio_get_level(drv->endstop_no_pin_);
    const int nc_lvl = gpio_get_level(drv->endstop_nc_pin_);

    const bool triggered = (no_lvl == 0 && nc_lvl == 1) || (no_lvl == nc_lvl);

    if (triggered) {
        drv->endstop_active_.store(true, std::memory_order_release);
        ... wake executor task ...
    } else {
        drv->endstop_active_.store(false, std::memory_order_release);
    }
}
```

### Rôle

Quand le capteur change d'état pendant une phase armée :

- l'ISR met `endstop_active_ = true`,
- réveille l'exécuteur,
- l'arrêt RMT peut alors se faire immédiatement côté temps réel.

## 16.3 Arrêt RMT dans l'encodeur ISR

```cpp
if (drv->endstop_active_.load(std::memory_order_relaxed)) {
    drv->rmt_stopped_.store(true, std::memory_order_relaxed);
    *done = true;
    return 0;
}
```

### Rôle

Le flux RMT s'arrête immédiatement si `endstop_active_` est levé.

---

## 17. Firmware : exécution multi-axis et RECOVERY

### Fichier

`src/esp32/src/comm_interface.cpp`

## 17.1 Détection dans la boucle DRAIN

```cpp
for (uint8_t a = 0; a < seg.axis_count && !endstop_hit; ++a) {
    const uint8_t eid = seg.axis_ids[a];
    ...
    if (self->queues_[eid]->driver().isEndstopActive()) {
        self->queues_[eid]->driver().emergencyStop();
        self->notifySegmentExecuted(seg.motion_sequence);
        ESP_LOGW(TAG, "endstop on axis %u at seq=%u", eid, seg.motion_sequence);
        endstop_hit = true;
    }
}
if (endstop_hit) {
    state = ExecState::RECOVERY;
    goto exit_drain;
}
```

### Rôle

Quand l'exécuteur voit `endstop_active_` sur un segment :

- il déclenche `emergencyStop()`,
- il notifie la dernière séquence exécutée,
- il bascule en `RECOVERY`.

## 17.2 Garde passive du latéral dans le multi-axis

```cpp
const bool lateral_endstop_armed =
    self->n_motors_ > 1
    && self->queues_[1] != nullptr
    && self->queues_[1]->driver().isEndstopArmed();
const bool lateral_blocked =
    lateral_endstop_armed
    && lateral_state !=
    static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
```

Puis :

```cpp
if (axis_id == 1 && lateral_blocked) {
    ESP_LOGD(TAG, "axis1 blocked, skip %u steps", seg.axes[a].step_count);
    continue;
}
```

### Rôle

- si l'endstop latéral est armé et que le capteur n'est pas `OPEN`, les steps latéraux sont ignorés,
- si l'endstop est désarmé, le `backoff` n'est plus filtré.

## 17.3 États `FLUSH` et `RECOVERY`

```cpp
case ExecState::FLUSH: {
    const planned_segment_t& flush_seg = batch[batch_index];
    defer_head = defer_tail = 0;
    self->notifySegmentExecuted(flush_seg.flush_sequence);
    ...
    state = ExecState::IDLE;
    break;
}
```

```cpp
case ExecState::RECOVERY: {
    planned_segment_t discard;
    uint32_t drained = 0;
    while (drained < SEGMENT_QUEUE_DEPTH &&
           xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
        ++drained;
    }
    defer_head = defer_tail = 0;
    ESP_LOGW(TAG, "recovery: drained %lu remaining segments", ...);
    state = ExecState::IDLE;
    break;
}
```

### Rôle

- `FLUSH` : purge demandée explicitement par le host,
- `RECOVERY` : purge déclenchée localement après endstop / erreur.

C'est ce comportement qui explique les `segments_dropped` vus côté host.

---

## 18. Firmware : planner et purge

### Fichier

`src/esp32/src/motion_planner.cpp`

### Méthode liée au `FLUSH`

```cpp
void MotionPlanner::handleFlush(const flush_request_t& req)
{
    has_pending_block_ = false;
    pending_segment_idx_ = 0;
    timeline_us_ = esp_timer_get_time();
    last_planned_motion_seq_ = req.flush_sequence;

    while (...) {
        ... drop cmd queue ...
    }
    while (...) {
        ... drop segment queue ...
    }

    planned_segment_t flush_seg {};
    flush_seg.is_flush = true;
    flush_seg.flush_sequence = req.flush_sequence;
    ...
}
```

### Rôle

Le planner :

- abandonne le bloc en cours,
- purge les queues,
- injecte un segment sentinelle `flush` pour que l'exécuteur accuse réception du `flush_sequence`.

---

## 19. Mapping des résultats SPI liés au homing

### Fichier

`src/esp32/src/comm_interface.cpp`

### Mapping dans la boucle SPI

```cpp
if (err == ESP_OK) {
    self->last_result_ = static_cast<uint8_t>(SpiMessageResult::OK);
} else if (err == ESP_ERR_TIMEOUT) {
    self->last_result_ = static_cast<uint8_t>(SpiMessageResult::QUEUE_FULL);
} else if (err == ESP_ERR_INVALID_STATE) {
    self->last_result_ = static_cast<uint8_t>(SpiMessageResult::ENDSTOP_BLOCKED);
} else {
    ...
}
```

### Rôle

Quand un mouvement est bloqué par la garde d'endstop, le firmware renvoie `ENDSTOP_BLOCKED` au host.

---

## 20. Actions / méthodes liées au homing et à l'endstop

## 20.1 Host Python

### API RPC

- `winding.home_lateral(...)`
- `winding.arm_endstop(axis_id)`
- `winding.disarm_endstop(axis_id)`
- `winding.flush_until(sequence)`

### `WindingEngine`

- `home_lateral(...)`
- `_home_lateral_axis(...)`
- `arm_endstop(axis_id)`
- `disarm_endstop(axis_id)`
- `flush_until(sequence)`

### `MoveQueue`

- `_execute_homing(...)`
- `_ensure_homing_can_start(...)`
- `_set_endstop_armed(...)`
- `_wait_for_endstop_arm_state(...)`
- `_wait_for_endstop_open(...)`
- `_update_axis_endstop_state(...)`
- `_next_motion_sequence()`
- `_wrap_segment_sequence(...)`

### `AxisState`

- `update_endstop_state(...)`
- `endstop_triggered`
- `mark_homed(...)`
- `invalidate_position()`

### `MultiAxisRampStreamer`

- `stream_all()`
- `_check_endstop(...)`
- `_mark_endstop_triggered()`
- `note_endstop_armed(...)`
- `request_flush(...)`
- `flush_until(...)`
- `arm_endstop(...)`
- `disarm_endstop(...)`

### `Esp32SpiTransport`

- `enable_endstop_request(...)`
- `arm_endstop(...)`
- `disarm_endstop(...)`
- `flush_until(...)`
- `wait_for_request_result(...)`
- `get_status()`

## 20.2 Firmware ESP32

### `CommInterface`

- `handleEnableEndstop(...)`
- `readLateralEndstopState()`
- `isLateralMovementAllowed(...)`
- `buildStatusFrame(...)`
- `handleFlush(...)`
- `handleFrame(...)`
- `notifySegmentExecuted(...)`
- `multiAxisExecutorTask(...)`

### `StepperDriver`

- `armEndstop()`
- `disarmEndstop()`
- `isEndstopArmed()`
- `isEndstopActive()`
- `initEndstopIsr(...)`
- `endstopIsrHandler(...)`
- `encode_steps(...)`
- `emergencyStop()`

### `MotionPlanner`

- `handleFlush(...)`
- `segmentQueueFree()`
- `resetStats()`

---

## 21. Séquence complète résumée

## 21.1 Homing réussi

1. Le client appelle `winding.home_lateral`.
2. `WindingEngine.home_lateral()` vérifie que l'engine est `IDLE`.
3. `WindingEngine._home_lateral_axis()` construit un `HomingMove` et l'enfile.
4. `MoveQueue._execute_homing()` vérifie que l'endstop est `PRESENT_OPEN`.
5. `MoveQueue` arme l'endstop avec `ENABLE_ENDSTOP arm=1`.
6. Le host vérifie `endstop_armed_mask`.
7. Le streamer envoie les `MULTI_AXIS_SEGMENT_BLOCK` de la phase `approach`.
8. Le firmware reçoit les blocs, les planifie, puis l'exécuteur les pousse dans le ring RMT.
9. L'ISR endstop détecte le contact et met `endstop_active_=true`.
10. L'exécuteur stoppe le moteur, notifie la séquence exécutée et passe en `RECOVERY`.
11. Le host voit le contact via `lateral_endstop_state`, `running_mask`, `segments_dropped` ou `ENDSTOP_BLOCKED`.
12. Le streamer déclenche `request_stop()` et `request_flush()`.
13. `MoveQueue` valide que `approach` s'est bien terminé par endstop.
14. `MoveQueue` désarme l'endstop.
15. La phase `backoff` recule jusqu'à ce que `lateral_endstop_state == PRESENT_OPEN`.
16. `MoveQueue` réarme l'endstop.
17. La phase `search` recommence plus lentement et retouche la butée.
18. À la fin, `AxisState.mark_homed(...)` fixe la position home.
19. Le moteur repasse en état `IDLE`.

## 21.2 Cas d'échec typiques

- le capteur est déjà fermé au départ,
- le capteur est absent,
- le bit `endstop_armed_mask` ne reflète pas l'armement demandé,
- la phase `approach` ou `search` se termine sans `endstop_triggered`,
- le `backoff` ne ré-ouvre jamais le capteur,
- le firmware entre en `RECOVERY` et le host finit par timeout ou par `FAULT`.

---

## 22. Points importants à retenir

- Le **host pilote la sémantique du homing**.
- Le **firmware pilote l'arrêt temps réel**.
- `ENABLE_ENDSTOP` ne sert pas seulement à lire le capteur : il active la protection temps réel.
- Le `backoff` doit se faire **endstop désarmé**.
- `lateral_endstop_state` décrit le capteur physique, alors que `endstop_armed_mask` décrit la protection firmware.
- `running_mask`, `last_executed_sequence`, `last_planned_sequence` et `segments_dropped` sont essentiels pour diagnostiquer un homing qui touche la butée mais que le host ne comprend pas correctement.
- Le `FLUSH` côté host et le `RECOVERY` côté firmware sont deux mécanismes différents mais complémentaires.

---

## 23. Fichiers à relire en priorité pour déboguer un homing

### Côté host

1. `src/rpi/motion/move_queue.py`
2. `src/rpi/transport/streamer.py`
3. `src/rpi/motion/engine.py`
4. `src/rpi/transport/spi_transport.py`
5. `src/rpi/transport/messages.py`

### Côté firmware

1. `src/esp32/src/comm_interface.cpp`
2. `src/esp32/src/stepper_driver.cpp`
3. `src/esp32/src/stepper_driver.h`
4. `src/esp32/src/messages.h`
5. `src/esp32/src/motion_planner.cpp`
