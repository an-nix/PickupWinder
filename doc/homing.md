# Homing latéral — référence technique v3

> Généré depuis les sources après application des correctifs R1–R10 (homing_analyze.md).
> Dernière mise à jour : 2026-04-20.

---

## Table des matières

1. [Vue d'ensemble](#1-vue-densemble)
2. [Point d'entrée JSON-RPC](#2-point-dentrée-json-rpc)
3. [Orchestration engine.py](#3-orchestration-enginepy)
4. [HomingMove — phases et paramètres](#4-homingmove--phases-et-paramètres)
5. [MoveQueue — exécution phase par phase](#5-movequeue--exécution-phase-par-phase)
6. [Préconditions : `_ensure_homing_can_start`](#6-préconditions--_ensure_homing_can_start)
7. [Pré-dégagement : `_clear_closed_endstop_before_homing`](#7-pré-dégagement--_clear_closed_endstop_before_homing)
8. [Streamer — détection endstop](#8-streamer--détection-endstop)
9. [Firmware — StatusPayload et endstop_hit_mask](#9-firmware--statuspayload-et-endstop_hit_mask)
10. [Firmware — DRAIN : gate latérale et fail-safe ABSENT](#10-firmware--drain--gate-latérale-et-fail-safe-absent)
11. [Firmware — RECOVERY](#11-firmware--recovery)
12. [Matrice de comportement](#12-matrice-de-comportement)
13. [Sémantique des états de faute](#13-sémantique-des-états-de-faute)
14. [Catalogue des méthodes](#14-catalogue-des-méthodes)
15. [Note de déploiement](#15-note-de-déploiement)

---

## 1. Vue d'ensemble

Le homing latéral est une séquence **host-driven** : le Raspberry Pi orchestre toutes les phases ; l'ESP32 exécute les pas et signale les événements via le status SPI.

```
RPC home_lateral_axis
  └─ WindingEngine._home_lateral_axis()
       └─ MoveQueue._execute_homing(HomingMove)
            ├─ [initial_state == CLOSED] _clear_closed_endstop_before_homing()
            ├─ approach  (armé)    → endstop_triggered → RECOVERY firmware
            ├─ backoff   (désarmé) → _wait_for_endstop_open()
            └─ search    (armé)    → endstop_triggered → position home
```

**Invariants clés :**
- L'axe ne doit jamais se déplacer quand `lateral_endstop_state == ABSENT` et l'endstop est armé : le firmware déclenche un arrêt fail-safe (R9).
- `endstop_hit_mask` dans le status est remis à zéro dès le désarmement (R1), ce qui évite les faux déclenchements sur la phase suivante.
- Le host filtre `endstop_hit_mask` par axe armé localement (R2), évitant les faux positifs dus à des bits résiduels d'un autre axe.

---

## 2. Point d'entrée JSON-RPC

**Fichier :** `src/rpi/jsonrpc/winding_handler.py`

```python
@rpc_method("winding.home_lateral_axis")
def home_lateral_axis(self, params):
    approach_rpm = float(params.get("approach_rpm", 100.0))
    search_rpm   = float(params.get("search_rpm",   20.0))
    backoff_steps = int(params.get("backoff_steps", 3200))
    self._engine.home_lateral_axis(
        approach_rpm=approach_rpm,
        search_rpm=search_rpm,
        backoff_steps=backoff_steps,
    )
```

---

## 3. Orchestration engine.py

**Fichier :** `src/rpi/motion/engine.py`  
**Méthode :** `WindingEngine._home_lateral_axis`

```python
def _home_lateral_axis(
    self, *, axis_id, approach_rpm, search_rpm, backoff_steps
) -> tuple[bool, str | None]:
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
    ...
```

**Direction du homing :** `reverse_direction=self._config.lateral_invert_direction`.  
`lateral_invert_direction` (défaut `False`) inverse la direction d'approche ET de recherche.  
Le backoff utilise automatiquement `not reverse_direction`.

---

## 4. HomingMove — phases et paramètres

**Fichier :** `src/rpi/motion/move.py`

| Phase | Armement endstop | Direction | Critère de fin |
|-------|:---:|---|---|
| `approach` | ✅ armé | `reverse_direction` | `endstop_triggered` |
| `backoff`  | ❌ désarmé | `not reverse_direction` | durée (`backoff_steps`) |
| `search`   | ✅ armé | `reverse_direction` | `endstop_triggered` |

- `max_approach_steps` : butée dure (20 tours par défaut) — si l'endstop n'a pas tiré, homing échoue avec diagnostic complet (R7).
- `home_position_steps` : position absolue enregistrée dans `AxisState` après la phase search.

---

## 5. MoveQueue — exécution phase par phase

**Fichier :** `src/rpi/motion/move_queue.py`  
**Méthode :** `MoveQueue._execute_homing`

```
1. Lire initial_status → lateral_endstop_state
2. Si ABSENT  → _ensure_homing_can_start() → RuntimeError → mark_failed
3. Si CLOSED  → _clear_closed_endstop_before_homing() puis _ensure_homing_can_start()
4. Si OPEN    → _ensure_homing_can_start() (confirmation)

Pour chaque phase (approach, backoff, search) :
  a. _ensure_homing_can_start() si la phase est armée
  b. _set_endstop_armed(arm)  ← envoie ENABLE_ENDSTOP SPI, attend endstop_armed_mask
  c. _stream_homing_sub_move()  ← crée streamer, stream_all()
  d. Si approach ou search : _check_armed_phase_result()  ← diagnostics sur échec
  e. Si backoff : _wait_for_endstop_open()

5. _set_endstop_armed(arm=False)
6. axis_state.mark_homed(home_position_steps)
7. move.mark_completed()
```

### 5.1 Confirmation du désarmement : `_set_endstop_armed`

Après envoi du SPI `ENABLE_ENDSTOP`, le host attend que `endstop_armed_mask` reflète
l'état demandé via `_wait_for_endstop_arm_state` (R8 — timeout adaptatif : `max(0.5, 20 × poll_interval_s)`).

---

## 6. Préconditions : `_ensure_homing_can_start`

```python
def _ensure_homing_can_start(self, axis_id, phase_name):
    status = self._read_status(axis_id)
    lateral_state = int(getattr(status, "lateral_endstop_state", ABSENT))
    if lateral_state == ABSENT:
        raise RuntimeError("lateral endstop sensor is ABSENT (cable disconnected)")
    if lateral_state != PRESENT_OPEN:
        raise RuntimeError(f"lateral_endstop_state=0x{lateral_state:02X} (expected PRESENT_OPEN)")
```

Appelée avant chaque phase armée. Refuse explicitement `ABSENT` (capteur débranché).

---

## 7. Pré-dégagement : `_clear_closed_endstop_before_homing`

Déclenchée si `lateral_endstop_state == PRESENT_CLOSED` au démarrage du homing (l'axe
est déjà en contact avec la butée).

```
1. _set_endstop_armed(arm=False)         ← désarmer avant de bouger
2. _stream_homing_sub_move(phase="preclear", arm_endstop=False)
   └─ mouvement backoff sans armement : recule loin de l'endstop
3. _wait_for_endstop_open(timeout=_compute_backoff_timeout)
4. time.sleep(0.020)                     ← R5: debounce mécanique (2 cycles SPI)
5. _read_status() → vérifier lateral_endstop_state == PRESENT_OPEN
   └─ sinon RuntimeError "preclear did not clear the endstop"
```

**R5 — debounce mécanique :** après que `_wait_for_endstop_open` confirme l'ouverture,
une attente de 20 ms est insérée pour stabiliser le GPIO avant de relire l'état. Sans
cette attente, un rebond mécanique peut repasser brièvement à CLOSED et faire échouer
`_ensure_homing_can_start` avec un message trompeur.

---

## 8. Streamer — détection endstop

**Fichier :** `src/rpi/transport/streamer.py`  
**Méthode :** `MultiAxisRampStreamer._check_endstop`

### 8.1 Priorité de détection (R2)

```python
# 1. endstop_hit_mask filtré sur les axes armés localement
for axis_id in self._endstop_armed_axes:
    if hit_mask & (1 << axis_id):
        self._mark_endstop_triggered()
        return True

# 2. Fallback : capteur CLOSED + axe armé arrêté
for axis_id in self._endstop_armed_axes:
    axis_stopped = (running_mask & (1 << axis_id)) == 0
    if lateral_state == PRESENT_CLOSED and axis_stopped:
        self._mark_endstop_triggered()
        return True

# 3. segments_dropped + axe armé arrêté (file planner vide ≠ endstop seul)
```

**R2 — filtrage par axe :** avant ce correctif, tout bit non nul dans `endstop_hit_mask`
déclenchait le flag, même si le bit correspondait à un axe non armé (e.g. axe 0 avec
un `endstop_hit_count_` résiduel). Le nouveau code teste uniquement les bits
correspondant aux axes dans `_endstop_armed_axes`.

**R3 — `_last_segments_dropped` initialisé à `0` :** l'ancienne valeur `None` obligeait
un guard spécial au premier appel. Maintenant initialisé à `0` dans `__init__`.

### 8.2 `_mark_endstop_triggered`

```python
def _mark_endstop_triggered(self) -> None:
    if self._endstop_triggered:
        return
    self._endstop_triggered = True
    if self._last_confirmed_motion_seq >= 0:
        flush_seq = self._last_confirmed_motion_seq   # confirmé par le firmware
    elif self._last_sent_motion_seq >= 0:
        flush_seq = self._last_sent_motion_seq        # fallback : dernier envoyé
    else:
        flush_seq = 0xFFFF                            # flush total
    self.request_stop()
    self.request_flush(flush_seq)
```

`_last_confirmed_motion_seq` est initialisé à `-1` ; la valeur `0xFFFF` firmware (rien
exécuté) est ignorée dans `_update_confirmed_motion_sequence`.

---

## 9. Firmware — StatusPayload et endstop_hit_mask

**Fichiers :** `src/esp32/src/messages.h`, `src/rpi/transport/messages.py`

```c
struct StatusPayload {          // 54 bytes
    ...
    uint8_t  lateral_endstop_state;  // LateralEndstopState enum
    uint8_t  endstop_armed_mask;     // bit i = axe i armé
    uint8_t  endstop_hit_mask;       // bit i = endstop_hit_count_[i] > 0
    ...
};
```

### `endstop_hit_mask` côté firmware (R1)

Le bit `i` dans `endstop_hit_mask` est à `1` si `endstop_hit_count_[i] > 0`.

`armEndstop()` remet `endstop_hit_count_` à zéro.  
`disarmEndstop()` remet **aussi** `endstop_hit_count_` à zéro (R1 — correctif).

**Pourquoi R1 est critique :**  
Sans ce correctif, `disarmEndstop()` laissait `endstop_hit_count_` non nul après une
phase approach. Le premier status SPI émis pendant la phase backoff (désarmée) portait
encore `endstop_hit_mask=1`. Sans le filtre R2, le streamer aurait déclenché un faux
endstop sur la phase backoff.

---

## 10. Firmware — DRAIN : gate latérale et fail-safe ABSENT

**Fichier :** `src/esp32/src/comm_interface.cpp`  
**État machine :** `ExecState::DRAIN`

### Ordre d'exécution dans DRAIN (restructuré R9+R10)

```cpp
// 1. Déclaration anticipée (R10)
uint8_t guarded_axis_ids[MULTI_AXIS_MAX_AXES] = {};
uint8_t guarded_axis_count = 0;
auto clearMultiExecFlags = [&]() { /* setMultiExecActive(false) pour tous */ };

// 2. Vérification endstop ISR (avant setMultiExecActive)
bool endstop_hit = false;
for (axis in seg.axis_ids) {
    if (driver.isEndstopActive()) { emergencyStop(); notifySegmentExecuted(); endstop_hit=true; }
}
if (endstop_hit) {
    clearMultiExecFlags();   // R10 : libérer avant RECOVERY
    state = RECOVERY; goto exit_drain;
}

// 3. Lecture état capteur
const uint8_t lateral_state = readLateralEndstopState();
const bool lateral_endstop_armed = (n_motors_>1 && queues_[1]->driver().isEndstopArmed());

// 4. Fail-safe ABSENT (R9)
if (lateral_endstop_armed && lateral_state == ABSENT) {
    ESP_LOGW(TAG, "lateral endstop ABSENT while armed — fail-safe stop");
    queues_[1]->driver().emergencyStop();
    clearMultiExecFlags();
    notifySegmentExecuted(seg.motion_sequence);
    state = RECOVERY; goto exit_drain;
}

// 5. Gate latérale normale
const bool lateral_blocked = lateral_endstop_armed
 10 — `clearMultiExecFlags` avant RECOVERY :** la lambda est maintenant déclarée
*avant* le premier check endstop. À ce point `guarded_axis_count=0` (pas encore de
`setMultiExecActive`), donc l'appel est un no-op sur le chemin early-endstop, mais il
sera utile si du code futur appelle `setMultiExecActive` plus tôt.

**R9 — fail-safe ABSENT :** si le capteur est absent (câble coupé) pendant qu'un
homing est armé, le firmware déclenche un arrêt d'urgence. L'ISR GPIO ne peut pas
détecter ABSENT (elle lit les pins NO/NC individuellement) ; seule la lecture conjointe
`NO==NC` dans le task context peut l'identifier.

---

## 11. Firmware — RECOVERY

**Fichier :** `src/esp32/src/comm_interface.cpp`  
**État machine :** `ExecState::RECOVERY`

```cpp
case ExecState::RECOVERY: {
    planned_segment_t discard;
    uint32_t drained = 0;
    uint16_t last_drained_seq = 0;
    bool has_seq = false;
    while (drained < SEGMENT_QUEUE_DEPTH &&
           xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
        if (!discard.is_flush) {
            last_drained_seq = discard.motion_sequence;
            has_seq = true;
        }
        ++drained;
    }
    if (has_seq) {
        notifySegmentExecuted(last_drained_seq);  // notifie la dernière seq drainée
    }
    defer_head = defer_tail = 0;
    batch_count = batch_index = 0;
    state = ExecState::IDLE;
}
```

La notification de `last_drained_seq` permet au host de calculer le `flush_sequence`
correct dans `_mark_endstop_triggered`.

---

## 12. Matrice de comportement

| `lateral_endstop_state` au démarrage | Endstop armé firmware | Action host |
|---|:---:|---|
| `PRESENT_OPEN` | non | démarrage normal |
| `PRESENT_CLOSED` | non | `_clear_closed_endstop_before_homing` + debounce |
| `ABSENT` | non | `RuntimeError` "ABSENT (cable disconnected)" |
| `PRESENT_OPEN` | oui | démarrage normal (endstop armé par `_set_endstop_armed`) |
| `PRESENT_CLOSED` | oui | `isLateralMovementAllowed` refuse → `ENDSTOP_BLOCKED` |
| `ABSENT` | oui (pendant DRAIN) | fail-safe firmware : `emergencyStop` → RECOVERY |

---

## 13. Sémantique des états de faute

- **`FAULT`** : homing échoué (ABSENT, timeout, approche sans trigger).  
  → `winding.clear_fault` requis avant tout mouvement latéral.
- **`move_lateral_to_mm` refusé tant que `homed=False`** : normal, pas un bug.
- **Message diagnostique (R7) :** si approach ou search se terminent sans trigger,
  l'erreur inclut `lateral_state`, `running_mask`, `last_exec`, `last_sent`.

---

## 14. Catalogue des méthodes

### Host — MoveQueue (`src/rpi/motion/move_queue.py`)

| Méthode | Rôle | Correctif |
|---|---|:---:|
| `_execute_homing` | Dispatch des phases | — |
| `_ensure_homing_can_start` | Refuse ABSENT/CLOSED avant phase | — |
| `_clear_closed_endstop_before_homing` | Pré-dégagement + debounce | R5 |
| `_stream_homing_sub_move` | Crée streamer + stream_all | — |
| `_set_endstop_armed` | ENABLE_ENDSTOP SPI + attente mask | — |
| `_wait_for_endstop_arm_state` | Timeout adaptatif `max(0.5, 20×poll)` | R8 |
| `_wait_for_endstop_open` | Attend PRESENT_OPEN | — |
| `_compute_backoff_timeout` | Fallback chain : estimated→ramp→steps/hz | R4 |
| `_check_armed_phase_result` | Diagnostics si approach/search sans trigger | R7 |

### Host — MultiAxisRampStreamer (`src/rpi/transport/streamer.py`)

| Méthode/attribut | Rôle | Correctif |
|---|---|:---:|
| `_check_endstop` | Détection par axe armé (filtre hit_mask) | R2 |
| `_mark_endstop_triggered` | flush_seq depuis confirmed→sent→0xFFFF | R3 |
| `_last_confirmed_motion_seq` | Initialisé à `-1` (sentinel) | R3 |
| `_last_segments_dropped` | Initialisé à `0` (pas `None`) | R3 |
| `last_sent_motion_seq` | Property exposée pour diagnostics | R7 |

### Firmware — StepperDriver (`src/esp32/src/stepper_driver.h`)

| Méthode | Rôle | Correctif |
|---|---|:---:|
| `armEndstop()` | arm=true + reset hit_count | — |
| `disarmEndstop()` | arm=false + reset hit_count | R1 |
| `getEndstopHitCount()` | Lu par buildStatusFrame → hit_mask | — |

### Firmware — CommInterface (`src/esp32/src/comm_interface.cpp`)

| Lieu | Rôle | Correctif |
|---|---|:---:|
| DRAIN — endstop ISR check | clearMultiExecFlags avant RECOVERY | R10 |
| DRAIN — gate latérale | ABSENT+armé → fail-safe emergencyStop | R9 |
| RECOVERY | Drainer + notifier last_drained_seq | — |

---

## 15. Note de déploiement

**Host Python** (Pi) :

```bash
scp -r src/rpi/* pi@192.168.74.89:/home/pi/winder/
```

**Firmware ESP32** (PlatformIO) : après modification de `stepper_driver.h` ou
`comm_interface.cpp`, reconstruire et flasher via PlatformIO.

Les changements R1, R9, R10 (firmware) et R2, R3, R4, R5, R7, R8 (host Python)
sont indépendants — le firmware peut être flashé séparément du déploiement Python.
