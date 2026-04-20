# PickupWinder — Correctifs homing v2 : analyse des modifications et optimisations restantes

> Document de spécification à destination de GitHub Copilot.
> Ce document part de l'état **post-correctifs** décrit dans le document de référence v2
> et identifie ce qui reste incorrect, incomplet ou sous-optimal.
> Chaque section suit le même format : fichier, méthode, problème, comportement attendu, code cible.

---

## Résumé des correctifs déjà appliqués (ne pas re-appliquer)

Les points suivants sont confirmés comme implémentés dans la v2 :

- `isLateralMovementAllowed` conditionne le blocage à `isEndstopArmed()` ✓
- `endstop_hit_mask` dans `StatusPayload` firmware et miroir Python ✓
- `endstop_hit_count_` dans `StepperDriver`, remis à zéro dans `armEndstop()` ✓
- `RECOVERY` notifie la dernière séquence drainée ✓
- `_mark_endstop_triggered` utilise `_last_confirmed_motion_seq` ✓
- `_check_endstop` utilise `endstop_hit_mask` en priorité ✓
- `_ensure_homing_can_start` refuse explicitement `ABSENT` ✓
- Pré-dégagement `preclear` si capteur déjà fermé au départ ✓
- Timeout de backoff dynamique `_compute_backoff_timeout` ✓

---

## Problèmes résiduels identifiés

---

## R1 — `disarmEndstop()` ne remet pas `endstop_hit_count_` à zéro

### Fichier
`src/esp32/src/stepper_driver.h`

### Méthode
`StepperDriver::disarmEndstop`

### Problème
Le document v2 confirme que `armEndstop()` remet `endstop_hit_count_` à zéro.
Mais `disarmEndstop()` ne le fait pas.

Si le host désarme entre deux phases (fin `approach`, début `backoff`), puis réarme pour
`search`, le `endstop_hit_count_` de la phase `approach` est toujours non nul au moment
de l'armement `search`. `armEndstop()` le remet à zéro à ce moment-là, donc ça fonctionne.

**Mais** si le host appelle `disarm → mouvements divers → arm`, le `endstop_hit_count_`
accumulé pendant les mouvements désarmés (ISR inactive, donc impossible) ne pose pas
de problème. Ce n'est pas un bug actif.

**Le vrai problème** : si le host appelle `disarm` puis lit le status avant le prochain
`arm`, `endstop_hit_mask` peut encore être à 1 (le bit est mis depuis `endstop_hit_count_ > 0`,
et `disarm` ne remet pas le compteur à zéro). Le streamer du `backoff` pourrait donc
voir `endstop_hit_mask` non nul au début du `backoff` et déclencher un faux endstop
sur la phase désarmée.

### Comportement attendu
`disarmEndstop()` doit aussi remettre `endstop_hit_count_` à zéro pour que le status
suivant reflète un état propre dès le désarmement.

### Code cible

```cpp
void disarmEndstop() {
    endstop_armed_.store(false, std::memory_order_release);
    endstop_active_.store(false, std::memory_order_release);
    endstop_hit_count_.store(0, std::memory_order_relaxed);  // ← AJOUT
}
```

---

## R2 — `_check_endstop` utilise `endstop_hit_mask` sans filtre d'axe

### Fichier
`src/rpi/transport/streamer.py`

### Méthode
`MultiAxisRampStreamer._check_endstop`

### Problème
Le document v2 montre :

```python
hit_mask = int(getattr(status, "endstop_hit_mask", 0))
if hit_mask != 0 and (armed_mask != 0 or bool(self._endstop_armed_axes)):
    self._mark_endstop_triggered()
    return True
```

Cette logique déclenche le flag endstop si **n'importe quel** bit de `endstop_hit_mask`
est non nul, même si ce bit correspond à un axe qui n'est pas dans `self._endstop_armed_axes`.

Exemple : le mouvement homing arme l'axe 1. Si l'axe 0 (bobine) avait un `endstop_hit_count_`
résiduel non nul (bug R1 ci-dessus, ou test précédent), `hit_mask` serait `0x01`, et le
homing latéral serait déclenché à tort.

### Comportement attendu
Ne déclencher que si le bit `endstop_hit_mask` correspond à un axe dans
`self._endstop_armed_axes`.

### Code cible

```python
def _check_endstop(self, status) -> bool:
    armed_mask   = int(getattr(status, "endstop_armed_mask", 0))
    lateral_state = int(getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT))
    running_mask  = int(getattr(status, "running_mask", 0))
    hit_mask      = int(getattr(status, "endstop_hit_mask", 0))

    # Signal canonique : endstop_hit_mask, filtré sur les axes réellement armés
    for axis_id in self._endstop_armed_axes:
        if hit_mask & (1 << axis_id):
            self._mark_endstop_triggered()
            return True

    # Fallback : capteur fermé + axe armé arrêté
    for axis_id in self._endstop_armed_axes:
        axis_stopped = (running_mask & (1 << axis_id)) == 0
        if lateral_state == LATERAL_ENDSTOP_PRESENT_CLOSED and axis_stopped:
            self._mark_endstop_triggered()
            return True

    # segments_dropped uniquement si un axe armé est aussi arrêté
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

> `self._last_segments_dropped: int = 0` doit être initialisé dans `__init__`
> et mis à jour à chaque poll même hors déclenchement.

---

## R3 — `_last_confirmed_motion_seq` initialisé à 0 peut provoquer un FLUSH à seq=0

### Fichier
`src/rpi/transport/streamer.py`

### Méthode
`MultiAxisRampStreamer._mark_endstop_triggered` et `__init__`

### Problème
Le document v2 montre :

```python
flush_seq = self._last_confirmed_motion_seq
if flush_seq < 0:
    flush_seq = self._last_sent_motion_seq
if flush_seq < 0:
    flush_seq = 0xFFFF
```

Si `_last_confirmed_motion_seq` est initialisé à `0` et que l'endstop se déclenche
avant le premier status confirmé (premier segment très court, capteur déjà en contact),
le FLUSH sera envoyé avec `flush_sequence=0`.

Le planner firmware va alors filtrer tous les segments avec `motion_sequence > 0`, ce qui
est potentiellement tous les segments, alors que l'intention est de flusher jusqu'à la
dernière séquence connue.

### Comportement attendu
Initialiser `_last_confirmed_motion_seq` à `-1` (sentinel "jamais lu") et utiliser
`_last_sent_motion_seq` comme fallback si la valeur confirmée n'a pas encore été reçue.

### Code cible

Dans `__init__` :
```python
self._last_confirmed_motion_seq: int = -1
self._last_segments_dropped: int = 0
```

Dans la boucle de poll du status (à chaque réception d'un status firmware) :
```python
confirmed = int(getattr(status, "last_executed_sequence", 0xFFFF))
# 0xFFFF est la valeur initiale firmware "rien exécuté" — ne pas la stocker
if confirmed != 0xFFFF:
    self._last_confirmed_motion_seq = confirmed
```

Dans `_mark_endstop_triggered` :
```python
def _mark_endstop_triggered(self) -> None:
    if self._endstop_triggered:
        return
    self._endstop_triggered = True
    # Priorité : séquence confirmée par le firmware
    # Fallback : séquence envoyée (garantit cohérence si rien confirmé)
    # Dernier recours : 0xFFFF (flush total)
    if self._last_confirmed_motion_seq >= 0:
        flush_seq = self._last_confirmed_motion_seq
    elif self._last_sent_motion_seq >= 0:
        flush_seq = self._last_sent_motion_seq
    else:
        flush_seq = 0xFFFF
    self.request_stop()
    self.request_flush(flush_seq)
```

---

## R4 — `_compute_backoff_timeout` : le document v2 ne montre pas l'implémentation réelle

### Fichier
`src/rpi/motion/move_queue.py`

### Méthode
`MoveQueue._compute_backoff_timeout`

### Problème
Le document v2 mentionne la méthode mais ne montre pas son code. L'implémentation
doit couvrir les cas où `RampMove` n'a pas d'attribut `estimated_duration_s`.

La plupart des `RampMove` exposent `ramp.total_duration` (en secondes) via le
planificateur de trajectoire, mais ce nom d'attribut n'est pas documenté.

### Comportement attendu
Chercher les attributs dans l'ordre de fiabilité décroissante, avec fallback explicite.

### Code cible

```python
@staticmethod
def _compute_backoff_timeout(sub_move: "Move", margin: float = 1.5) -> float:
    """Calcule un timeout basé sur la durée estimée du sous-mouvement.

    Cherche les attributs dans l'ordre de priorité suivant :
      1. sub_move.estimated_duration_s  (attribut ajouté par HomingMove)
      2. sub_move.ramp.total_duration   (RampMove standard)
      3. steps / target_hz              (estimation depuis les paramètres bruts)
      4. 2.0 s                          (fallback de sécurité)

    Args:
        sub_move: Le Move correspondant au backoff ou preclear.
        margin:   Facteur multiplicatif de sécurité (défaut 1.5×).

    Returns:
        Timeout en secondes, minimum 1.0 s.
    """
    # Priorité 1 : attribut explicite
    estimated = getattr(sub_move, "estimated_duration_s", None)
    if estimated is not None and estimated > 0:
        return max(1.0, float(estimated) * margin)

    # Priorité 2 : RampMove.ramp.total_duration
    ramp = getattr(sub_move, "ramp", None)
    if ramp is not None:
        total = getattr(ramp, "total_duration", None)
        if total is not None and total > 0:
            return max(1.0, float(total) * margin)

    # Priorité 3 : estimation brute
    steps = getattr(sub_move, "total_steps", None) or getattr(sub_move, "step_count", None)
    hz    = getattr(sub_move, "target_hz", None) or getattr(sub_move, "cruise_steps_per_s", None)
    if steps and hz and steps > 0 and hz > 0:
        return max(1.0, (float(steps) / float(hz)) * margin)

    # Fallback
    return 2.0
```

---

## R5 — Pas de garde contre un re-arm immédiat si le preclear échoue silencieusement

### Fichier
`src/rpi/motion/move_queue.py`

### Méthode
`MoveQueue._clear_closed_endstop_before_homing`

### Problème
Le document v2 montre :

```python
def _clear_closed_endstop_before_homing(self, move: HomingMove) -> None:
    clearance_move = move._make_backoff_move()
    self._set_endstop_armed(move.axis_id, arm=False)
    streamer = self._stream_homing_sub_move(
        move,
        phase_name="preclear",
        sub_move=clearance_move,
        arm_endstop=False,
    )
    if streamer.endstop_triggered:
        raise RuntimeError(...)
    self._wait_for_endstop_open(
        move.axis_id,
        timeout_s=self._compute_backoff_timeout(clearance_move),
    )
```

La vérification `if streamer.endstop_triggered` contrôle que le streamer n'a pas
interprété autre chose comme un déclenchement.

**Mais** si `_wait_for_endstop_open` réussit, il n'y a pas de nouvelle lecture
du status pour confirmer que le capteur est stable avant de reprendre.
Si le moteur a un rebond mécanique et que le capteur repasse brièvement à
`CLOSED` juste après l'ouverture, `_ensure_homing_can_start` (appelé ensuite
pour la phase `approach`) rejette le homing.

Ce n'est pas un bug critique mais produit une erreur peu lisible :
`"homing approach cannot start: lateral_endstop_state=0x01"` alors que le
préclear avait réussi.

### Comportement attendu
Après `_wait_for_endstop_open`, ajouter une courte attente de stabilisation
et relire le status une fois de plus avant de continuer.

### Code cible

```python
def _clear_closed_endstop_before_homing(self, move: HomingMove) -> None:
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

    # Attente de stabilisation mécanique (debounce)
    # Durée : 2 cycles SPI minimum pour que le GPIO se stabilise
    time.sleep(0.020)

    # Relecture finale pour confirmer l'état avant d'armer
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

---

## R6 — `endstop_hit_mask` n'est pas remis à zéro côté firmware entre les phases

### Fichier
`src/esp32/src/stepper_driver.h` et `src/esp32/src/stepper_driver.cpp`

### Méthode
`StepperDriver::armEndstop` et `StepperDriver::disarmEndstop`

### Problème
Le document v2 confirme que `armEndstop()` remet `endstop_hit_count_` à zéro.
Cela signifie que le bit `endstop_hit_mask` dans le status ne s'efface que quand
le host envoie un nouvel `ENABLE_ENDSTOP arm=1`.

Mais entre `approach` (armé) et `backoff` (désarmé), le host envoie `arm=0`.
Avec R1 corrigé, `disarm` remet `endstop_hit_count_` à zéro → le status suivant
montre `endstop_hit_mask=0` dès le désarmement. C'est le comportement voulu.

**Vérification à faire** : s'assurer que `buildStatusFrame` lit `endstop_hit_count_`
après l'opération atomique de désarmement, pas avant. Comme `disarmEndstop()` est
appelé depuis le task SPI et que `buildStatusFrame` est aussi dans le task SPI,
l'ordre d'exécution est séquentiel. Pas de race condition ici.

> Ce point est une vérification, pas une modification.
> S'assurer que `handleEnableEndstop` appelle `disarmEndstop()` **avant** que
> `buildStatusFrame` ne soit appelé pour la réponse du même transfert SPI.
> L'ordre actuel dans `spiTask` est : `handleFrame(...)` → `buildStatusFrame(...)`.
> C'est correct — le désarmement est appliqué avant la construction du status.

### Action requise
Aucune modification nécessaire si R1 est appliqué. Ce point est documenté
pour clarifier l'ordre d'exécution dans `spiTask`.

---

## R7 — La validation "approach sans endstop_triggered" ne distingue pas les causes d'échec

### Fichier
`src/rpi/motion/move_queue.py`

### Méthode
`MoveQueue._execute_homing` (vérification post-phase)

### Problème
Le document v2 indique (section 6.4 ancienne version) :

```python
if phase_name in ("approach", "search") and not streamer.endstop_triggered:
    ...  # échec
```

Le message d'erreur générique ne distingue pas :
- le mouvement s'est terminé normalement sans toucher la butée (max steps atteint),
- le mouvement a été interrompu par un stall détecté,
- le firmware a retourné `QUEUE_FULL` sur tous les segments,
- le streamer a été stoppé par un autre signal.

### Comportement attendu
Inclure dans le message d'erreur la cause réelle observée : état final du capteur,
`running_mask`, `last_executed_sequence` vs séquences envoyées.

### Code cible

```python
def _check_armed_phase_result(
    self,
    move: "HomingMove",
    phase_name: str,
    streamer: "MultiAxisRampStreamer",
) -> None:
    """Vérifie qu'une phase armée s'est bien terminée par un déclenchement endstop.

    Lève RuntimeError avec un message diagnostique si ce n'est pas le cas.
    """
    if streamer.endstop_triggered:
        return  # succès nominal

    # Lire le status pour diagnostiquer
    status = self._read_status(move.axis_id)
    lateral_state = int(
        getattr(status, "lateral_endstop_state", LATERAL_ENDSTOP_ABSENT)
    )
    last_exec = int(getattr(status, "last_executed_sequence", 0xFFFF))
    last_sent = streamer.last_sent_motion_seq  # exposer cet attribut si nécessaire
    running   = int(getattr(status, "running_mask", 0))

    state_names = {0x00: "PRESENT_OPEN", 0x01: "PRESENT_CLOSED", 0xFF: "ABSENT"}
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

Appel dans `_execute_homing` :

```python
if phase_name in ("approach", "search"):
    self._check_armed_phase_result(move, phase_name, streamer)
```

---

## R8 — `_wait_for_endstop_arm_state` : timeout de 0.5s non justifié

### Fichier
`src/rpi/motion/move_queue.py`

### Méthode
`MoveQueue._wait_for_endstop_arm_state`

### Problème
Le timeout de 0.5 s pour attendre la confirmation de `endstop_armed_mask` est
arbitraire. Sur un Pi chargé avec un SPI à 1 MHz, un aller-retour SPI prend
~512 µs + overhead. À `poll_interval_s=0.005` (5 ms), 100 polls = 500 ms.

Le timeout est suffisant dans la pratique, mais si le Pi est très chargé et que
le transfert SPI prend plus de temps, la confirmation peut arriver juste après
l'expiration du timeout, causant un échec spurieux.

### Comportement attendu
Le timeout devrait être au moins `max(0.5, 10 * poll_interval_s * expected_spi_cycles)`.
En pratique, 3 cycles SPI suffisent (un pour envoyer, un pour que l'ESP32 traite,
un pour lire le résultat). Le timeout minimum raisonnable est `20 * poll_interval_s`.

### Code cible

```python
def _wait_for_endstop_arm_state(
    self,
    axis_id: int,
    arm: bool,
    timeout_s: float | None = None,
) -> Any:
    # Timeout par défaut : 20 cycles de poll, minimum 0.5s
    if timeout_s is None:
        timeout_s = max(0.5, 20 * self._poll_interval_s)

    deadline = time.monotonic() + timeout_s
    last_status = None
    while time.monotonic() < deadline:
        last_status = self._read_status(axis_id)
        if self._status_has_endstop_armed(last_status, axis_id, arm):
            return last_status
        time.sleep(self._poll_interval_s)

    # Message d'erreur avec état observé
    armed_mask = int(getattr(last_status, "endstop_armed_mask", 0)) if last_status else -1
    raise RuntimeError(
        f"endstop arm state timeout on axis {axis_id}: "
        f"wanted arm={arm}, "
        f"endstop_armed_mask=0x{armed_mask:02X} after {timeout_s:.2f}s"
    )
```

---

## R9 — `lateral_blocked` dans DRAIN ne tient pas compte du cas `ABSENT`

### Fichier
`src/esp32/src/comm_interface.cpp`

### Méthode
`CommInterface::multiAxisExecutorTask` — case `ExecState::DRAIN`

### Problème
Le document v2 (section 17.2) montre :

```cpp
const bool lateral_blocked =
    lateral_endstop_armed
    && lateral_state != static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
```

Si `lateral_state == ABSENT` (câble coupé) et que l'endstop est armé, cette condition
est vraie et les steps de l'axe 1 sont silencieusement ignorés.

Le comportement est discutable : si le capteur est absent pendant un homing armé,
le firmware devrait déclencher un arrêt d'urgence et passer en RECOVERY, pas
continuer silencieusement en ignorant les steps.

**Note** : l'ISR ne peut pas détecter `ABSENT` car elle lit les GPIO individuellement.
La détection `ABSENT` (NO==NC) n'est possible que via `readLateralEndstopState()`
dans le task context.

### Comportement attendu
Si l'endstop est armé et que `readLateralEndstopState()` retourne `ABSENT`,
déclencher un arrêt d'urgence comme si l'endstop avait été touché.

### Code cible

Dans `ExecState::DRAIN`, remplacer le bloc `lateral_blocked` par :

```cpp
const uint8_t lateral_state = self->readLateralEndstopState();

// Si l'endstop latéral est armé et que le capteur est absent (câble coupé),
// traiter comme un déclenchement pour sécurité fail-safe.
const bool lateral_endstop_armed =
    self->n_motors_ > 1
    && self->queues_[1] != nullptr
    && self->queues_[1]->driver().isEndstopArmed();

if (lateral_endstop_armed &&
    lateral_state == static_cast<uint8_t>(LateralEndstopState::ABSENT)) {
    // Capteur absent + armé = fail-safe : traiter comme hit
    ESP_LOGW(TAG, "lateral endstop ABSENT while armed at seq=%u — fail-safe stop",
             seg.motion_sequence);
    if (self->queues_[1] != nullptr) {
        self->queues_[1]->driver().emergencyStop();
    }
    self->notifySegmentExecuted(seg.motion_sequence);
    state = ExecState::RECOVERY;
    goto exit_drain;
}

const bool lateral_blocked =
    lateral_endstop_armed
    && lateral_state != static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
```

---

## R10 — `RECOVERY` dans DRAIN ne nettoie pas les `multi_exec_active_` flags

### Fichier
`src/esp32/src/comm_interface.cpp`

### Méthode
`CommInterface::multiAxisExecutorTask` — case `ExecState::DRAIN`

### Problème
Dans DRAIN, quand un endstop est détecté, le code fait :

```cpp
if (endstop_hit) {
    state = ExecState::RECOVERY;
    goto exit_drain;
}
```

Mais `clearMultiExecFlags()` (le lambda local qui appelle `setMultiExecActive(false)`) n'a
pas encore été appelé pour ce segment car il est appelé **après** la boucle
`executeConstantRateBlock`. Le `goto exit_drain` saute par-dessus cet appel.

En conséquence, les axes dont le flag `multi_exec_active_` est encore à `true` vont
bloquer dans `executorTask` sur `ulTaskNotifyTake` (section de `executorTask` dans
`stepper_queue.cpp`) jusqu'à la fin de RECOVERY.

### Comportement attendu
Appeler `clearMultiExecFlags()` avant tout `goto exit_drain` dans le path endstop.

### Code cible

Dans DRAIN, dans le bloc de détection endstop par ISR :

```cpp
bool endstop_hit = false;
for (uint8_t a = 0; a < seg.axis_count && !endstop_hit; ++a) {
    const uint8_t eid = seg.axis_ids[a];
    if (eid >= self->n_motors_ || self->queues_[eid] == nullptr) continue;
    if (self->queues_[eid]->driver().isEndstopActive()) {
        self->queues_[eid]->driver().emergencyStop();
        self->notifySegmentExecuted(seg.motion_sequence);
        ESP_LOGW(TAG, "endstop on axis %u at seq=%u", eid, seg.motion_sequence);
        endstop_hit = true;
    }
}
if (endstop_hit) {
    clearMultiExecFlags();  // ← AJOUT : libérer les axes avant RECOVERY
    state = ExecState::RECOVERY;
    goto exit_drain;
}
```

De même dans le bloc `ESP_ERR_INVALID_STATE` de `executeConstantRateBlock` :

```cpp
if (err == ESP_ERR_INVALID_STATE) {
    clearMultiExecFlags();  // ← déjà présent dans certaines versions, vérifier
    ESP_LOGW(TAG, "axis %u endstop mid-seg seq=%u", axis_id, seg.motion_sequence);
    axis_queue->driver().emergencyStop();
    self->notifySegmentExecuted(seg.motion_sequence);
    state = ExecState::RECOVERY;
    goto exit_drain;
}
```

---

## Résumé des modifications par fichier

| Fichier | Priorité | Points | Description |
|---|---|---|---|
| `src/esp32/src/stepper_driver.h` | 🔴 Critique | R1 | `disarmEndstop` remet `endstop_hit_count_` à zéro |
| `src/esp32/src/comm_interface.cpp` | 🔴 Critique | R9, R10 | `ABSENT` = fail-safe en DRAIN, `clearMultiExecFlags` avant RECOVERY |
| `src/rpi/transport/streamer.py` | 🔴 Critique | R2, R3 | `_check_endstop` filtre par axe, `_last_confirmed_motion_seq` initialisé à -1 |
| `src/rpi/motion/move_queue.py` | 🟠 Important | R4, R5, R7, R8 | `_compute_backoff_timeout` robuste, debounce preclear, diagnostics d'échec, timeout arm |

---

## Ordre d'application recommandé

1. **R1** + **R2** ensemble : R1 empêche les faux bits dans `endstop_hit_mask`, R2 filtre
   les bits résiduels. Les deux se complètent.

2. **R10** : libération des flags `multi_exec_active_` avant RECOVERY. Risque de deadlock
   entre le per-axis `executorTask` et `multiAxisExecutorTask` si non corrigé.

3. **R9** : fail-safe capteur absent. Correction de sécurité indépendante.

4. **R3** : sentinel `-1` pour `_last_confirmed_motion_seq`. Évite le FLUSH à seq=0.

5. **R5** : debounce dans `_clear_closed_endstop_before_homing`. Robustesse mécanique.

6. **R7** : message d'erreur diagnostique post-phase. Améliore debuggabilité.

7. **R4** + **R8** : `_compute_backoff_timeout` robuste et timeout arm adaptatif.
   Peuvent être appliqués ensemble.