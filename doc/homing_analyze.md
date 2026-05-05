# PickupWinder — Correctifs homing v3 : analyse des modifications et optimisations restantes

> Document de spécification à destination de GitHub Copilot.
> Ce document part de l'état **post-correctifs v3** (R1–R10 appliqués).
> Tous les points ci-dessous sont **nouveaux** — ne pas re-appliquer R1–R10.

---

## Résumé des correctifs confirmés dans v3 (ne pas re-appliquer)

| Ref | Description | Statut |
|-----|-------------|:------:|
| R1  | `disarmEndstop()` remet `endstop_hit_count_` à zéro | ✓ |
| R2  | `_check_endstop` filtre `endstop_hit_mask` par axe armé | ✓ |
| R3  | `_last_confirmed_motion_seq` initialisé à `-1`, flush fallback chain | ✓ |
| R4  | `_compute_backoff_timeout` chaîne de fallback robuste | ✓ |
| R5  | Debounce 20 ms + relecture finale dans `_clear_closed_endstop_before_homing` | ✓ |
| R7  | `_check_armed_phase_result` avec diagnostics complets | ✓ |
| R8  | Timeout `_wait_for_endstop_arm_state` adaptatif `max(0.5, 20×poll)` | ✓ |
| R9  | Fail-safe ABSENT dans DRAIN : arrêt d'urgence si capteur absent + armé | ✓ |
| R10 | `clearMultiExecFlags()` appelé avant tout `goto exit_drain` vers RECOVERY | ✓ |

---

## Problèmes résiduels identifiés dans v3

---

## N1 — DRAIN : le fail-safe ABSENT (R9) n'appelle pas `clearMultiExecFlags` avant `emergencyStop`

### Fichier
`src/esp32/src/comm_interface.cpp`

### État machine
`ExecState::DRAIN`

### Problème
Le document v3 section 10 montre l'ordre suivant pour le fail-safe ABSENT :

```cpp
if (lateral_endstop_armed && lateral_state == ABSENT) {
    queues_[1]->driver().emergencyStop();
    clearMultiExecFlags();          // ← appelé APRÈS emergencyStop
    notifySegmentExecuted(seg.motion_sequence);
    state = RECOVERY; goto exit_drain;
}
```

`clearMultiExecFlags()` doit être appelé **avant** `emergencyStop()`, comme R10 le
spécifie pour le chemin endstop ISR normal. `emergencyStop()` reset le ring RMT et
peut déclencher l'ISR `on_trans_done` qui réveille les per-axis `executorTask`. Si ces
tâches commencent à s'exécuter alors que `multi_exec_active_` est encore à `true`,
elles entrent dans `ulTaskNotifyTake` au lieu de traiter leur queue — elles attendent
une notification qui ne viendra pas de l'ISR endstop.

### Comportement attendu
Ordre strict : `clearMultiExecFlags()` → `emergencyStop()` → `notifySegmentExecuted()`
→ `state = RECOVERY`.

### Code cible

```cpp
if (lateral_endstop_armed &&
    lateral_state == static_cast<uint8_t>(LateralEndstopState::ABSENT)) {
    ESP_LOGW(TAG, "lateral endstop ABSENT while armed at seq=%u — fail-safe stop",
             seg.motion_sequence);
    clearMultiExecFlags();                          // ← EN PREMIER
    if (self->queues_[1] != nullptr) {
        self->queues_[1]->driver().emergencyStop();
    }
    self->notifySegmentExecuted(seg.motion_sequence);
    state = ExecState::RECOVERY;
    goto exit_drain;
}
```

---

## N2 — DRAIN : la gate `lateral_blocked` est lue après `setMultiExecActive(true)` mais avant `clearMultiExecFlags`

### Fichier
`src/esp32/src/comm_interface.cpp`

### État machine
`ExecState::DRAIN`

### Problème
Le document v3 section 10 montre la structure suivante (résumée) :

```
1. Check endstop ISR       → clearMultiExecFlags + RECOVERY si hit
2. Lire lateral_state
3. Fail-safe ABSENT        → clearMultiExecFlags + RECOVERY si absent+armé
4. setMultiExecActive(true) pour chaque axe du segment  ← DÉBUT de la zone gardée
5. kickStart si RMT arrêté
6. executeConstantRateBlock pour chaque axe
   └─ si ESP_ERR_INVALID_STATE → clearMultiExecFlags + RECOVERY
7. clearMultiExecFlags()  ← FIN de la zone gardée
```

Entre les étapes 4 et 6, si `executeConstantRateBlock` retourne une erreur autre que
`ESP_ERR_INVALID_STATE` (par exemple `ESP_ERR_TIMEOUT` si le ring est plein après 20
tentatives), le code actuel log un warning (`ESP_LOGW`) et **continue** sans aller en
RECOVERY. Le flag `multi_exec_active_` reste `true` pour l'axe concerné jusqu'à
`clearMultiExecFlags()` à l'étape 7.

Ce n'est pas un deadlock (l'étape 7 est atteinte dans tous les chemins non-RECOVERY),
mais un `ESP_ERR_TIMEOUT` dans `pushBlock` indique que le ring est plein et que le RMT
ne consomme plus. Dans ce cas, continuer les autres axes et appeler `kickStart` en
fin de batch peut aggraver l'état au lieu de le corriger.

### Comportement attendu
Sur `ESP_ERR_TIMEOUT` (ring plein + RMT bloqué) dans `executeConstantRateBlock`,
déclencher RECOVERY explicitement au lieu de continuer.

### Code cible

```cpp
esp_err_t err = axis_queue->executeConstantRateBlock(
    seg.axes[a].direction,
    seg.axes[a].step_count,
    seg.duration_us);

if (err == ESP_ERR_INVALID_STATE) {
    // Endstop déclenché pendant le remplissage du ring
    clearMultiExecFlags();
    ESP_LOGW(TAG, "axis %u endstop mid-seg seq=%u",
             axis_id, seg.motion_sequence);
    axis_queue->driver().emergencyStop();
    self->notifySegmentExecuted(seg.motion_sequence);
    state = ExecState::RECOVERY;
    goto exit_drain;
} else if (err == ESP_ERR_TIMEOUT) {
    // Ring plein + RMT bloqué : impossible de continuer proprement
    clearMultiExecFlags();
    ESP_LOGE(TAG, "axis %u ring timeout at seq=%u — forcing RECOVERY",
             axis_id, seg.motion_sequence);
    self->notifySegmentExecuted(seg.motion_sequence);
    state = ExecState::RECOVERY;
    goto exit_drain;
} else if (err != ESP_OK) {
    // Autre erreur non fatale : logger et continuer
    ESP_LOGW(TAG, "axis %u seg %u: %s",
             axis_id, seg.motion_sequence, esp_err_to_name(err));
}
```

---

## N3 — `_check_armed_phase_result` expose `last_sent_motion_seq` mais l'attribut n'est pas défini

### Fichier
`src/rpi/transport/streamer.py`

### Problème
Le document v3 section 14 indique :

> `last_sent_motion_seq` — Property exposée pour diagnostics (R7)

Mais le document ne montre pas l'implémentation de cette property. Si elle n'est pas
définie, `_check_armed_phase_result` lèvera `AttributeError` en tentant de lire
`streamer.last_sent_motion_seq` pour construire le message diagnostique.

### Comportement attendu
Exposer `_last_sent_motion_seq` comme property publique en lecture seule.

### Code cible

Dans `MultiAxisRampStreamer` :

```python
@property
def last_sent_motion_seq(self) -> int:
    """Dernière motion_sequence envoyée au firmware (ou -1 si rien envoyé)."""
    return self._last_sent_motion_seq
```

Et s'assurer que `_last_sent_motion_seq` est initialisé dans `__init__` :

```python
self._last_sent_motion_seq: int = -1
```

Mis à jour à chaque envoi d'un bloc `MULTI_AXIS_SEGMENT_BLOCK` :

```python
# Dans la boucle d'envoi des segments, après chaque send réussi :
self._last_sent_motion_seq = current_segment.motion_sequence
```

---

## N4 — `_wait_for_endstop_open` ne met pas à jour `_last_segments_dropped`

### Fichier
`src/rpi/motion/move_queue.py`

### Méthode
`MoveQueue._wait_for_endstop_open`

### Problème
Pendant la phase `backoff`, le streamer est terminé et `_wait_for_endstop_open` poll
le status SPI via `_read_status`. Chaque appel lit un nouveau `segments_dropped`
depuis le firmware.

Mais le streamer de la phase `backoff` a déjà terminé (`stream_all()` est revenu).
Le prochain streamer créé pour la phase `search` initialisera `_last_segments_dropped`
à `0`. Si le firmware a eu des segments drainés pendant le RECOVERY de la phase
`approach` (normal), et que ces drops ont été comptabilisés pendant le backoff, le
nouveau streamer `search` verra une valeur `segments_dropped` > 0 dès son premier
status, et pourrait le traiter comme un incrément (si le compteur firmware ne s'est
pas remis à zéro entre-temps).

**Précision :** le compteur `segments_dropped` du firmware (`planner_.segmentsDropped()`)
est cumulatif et ne se remet à zéro que sur `RESET_STATS`. Le streamer `search`
commencera avec `_last_segments_dropped=0` et verra une valeur potentiellement élevée
dès le premier status. Si cette valeur dépasse `0`, et qu'un axe armé est arrêté
(ce qui est le cas juste après l'armement, avant que le premier segment soit exécuté),
R2-corrigé `_check_endstop` déclenchera un faux endstop immédiatement.

### Comportement attendu
Le streamer de chaque phase doit s'initialiser avec la valeur courante de
`segments_dropped` (baseline), pas avec `0`. Ainsi seuls les incréments survenus
pendant la phase sont détectés.

### Code cible — option A : passer la baseline au constructeur du streamer

Dans `MoveQueue._stream_homing_sub_move` :

```python
def _stream_homing_sub_move(self, move, phase_name, sub_move, arm_endstop):
    # Lire la baseline segments_dropped avant de créer le streamer
    baseline_status = self._read_status(move.axis_id)
    baseline_dropped = int(
        getattr(baseline_status, "segments_dropped", 0)
    )

    streamer = self._make_streamer(
        sub_move.axis_configs,
        keep_enabled_axes={move.axis_id},
        initial_segments_dropped=baseline_dropped,   # ← nouveau paramètre
    )
    ...
```

Dans `MultiAxisRampStreamer.__init__` :

```python
def __init__(self, ..., initial_segments_dropped: int = 0):
    ...
    self._last_segments_dropped: int = initial_segments_dropped
```

### Code cible — option B (plus simple) : initialiser dans `note_endstop_armed`

```python
def note_endstop_armed(self, axis_id: int, arm: bool) -> None:
    if arm:
        self._endstop_armed_axes.add(axis_id)
    else:
        self._endstop_armed_axes.discard(axis_id)
    # Relever la baseline segments_dropped à chaque changement d'armement
    # pour éviter les faux positifs dus au compteur cumulatif firmware.
    # Note : nécessite un accès au transport depuis le streamer.
    # Préférer l'option A si le streamer n'a pas accès direct au transport.
```

> **Recommandation : option A.** Elle est explicite, testable et ne couple pas le
> streamer à un appel SPI supplémentaire dans `note_endstop_armed`.

---

## N5 — `RECOVERY` notifie `last_drained_seq` mais pas les segments déjà dans `defer_*`

### Fichier
`src/esp32/src/comm_interface.cpp`

### État machine
`ExecState::RECOVERY`

### Problème
RECOVERY réinitialise `defer_head = defer_tail = 0` et notifie `last_drained_seq`
(séquence maximale drainée de `seg_queue_`).

Mais entre le moment où un segment est consommé de `seg_queue_` dans DRAIN et le
moment où sa notification différée est émise (dans `fireDeferred()` au début du
prochain tour de boucle), des segments peuvent être présents dans le ring `defer_*`
avec un `defer_fire_us` déjà dépassé.

En effaçant `defer_head/tail` sans les notifier, ces séquences ne sont jamais
transmises au host via `notifySegmentExecuted`. Le host voit donc :

- `last_executed_sequence` avancé jusqu'à la séquence du segment qui a déclenché
  l'endstop (notifié dans DRAIN),
- puis un saut jusqu'à `last_drained_seq` (notifié dans RECOVERY),
- avec un "trou" correspondant aux segments déjà dans `defer_*` au moment du reset.

En pratique, ce trou est comblé par le FLUSH que le host envoie, car le FLUSH fait
avancer `last_executed_sequence` jusqu'à `flush_sequence`. Mais si le host utilise
`last_executed_sequence` pour calculer le nombre de segments encore en transit
(lookahead = `last_planned - last_executed`), ce trou peut provoquer un calcul de
lookahead incorrect immédiatement après l'endstop, avant la réception du status
post-FLUSH.

### Comportement attendu
Avant d'effacer `defer_head/tail` dans RECOVERY, notifier toutes les entrées dont
`defer_fire_us` est déjà passé.

### Code cible

```cpp
case ExecState::RECOVERY: {
    // Notifier les entrées différées déjà échues avant de vider le ring
    {
        const int64_t now = esp_timer_get_time();
        while (defer_head != defer_tail) {
            const int idx = defer_head & (DEFER_DEPTH - 1);
            if (now >= defer_fire_us[idx]) {
                self->notifySegmentExecuted(
                    static_cast<uint16_t>(defer_seqs[idx]));
                ++defer_head;
            } else {
                break;  // Les entrées suivantes ne sont pas encore échues
            }
        }
        // Vider le reste (non échus) sans notifier
        defer_head = defer_tail = 0;
    }

    // Drainer seg_queue_ et notifier last_drained_seq (inchangé)
    planned_segment_t discard;
    uint32_t  drained          = 0;
    uint16_t  last_drained_seq = 0;
    bool      has_seq          = false;

    while (drained < SEGMENT_QUEUE_DEPTH &&
           xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
        if (!discard.is_flush) {
            last_drained_seq = discard.motion_sequence;
            has_seq = true;
        }
        ++drained;
    }

    if (has_seq) {
        self->notifySegmentExecuted(last_drained_seq);
    }

    ESP_LOGW(TAG,
             "recovery: drained %lu segments, notified deferred=%d queued=%d",
             static_cast<unsigned long>(drained),
             static_cast<int>(defer_head - defer_head),  // 0 après reset
             static_cast<int>(has_seq));

    batch_count = 0;
    batch_index = 0;
    state = ExecState::IDLE;
    break;
}
```

---

## N6 — `_compute_backoff_timeout` : la chaîne de fallback lit `ramp.total_duration` mais `RampMove` expose `duration_s`

### Fichier
`src/rpi/motion/move_queue.py`

### Méthode
`MoveQueue._compute_backoff_timeout`

### Problème
Le document v3 (section 14) documente la chaîne de fallback :
`estimated_duration_s` → `ramp.total_duration` → `steps/hz` → `2.0`.

Mais `RampMove` (dans `src/rpi/motion/move.py`) expose très probablement la durée
totale via `ramp.duration_s` ou directement `move.duration_s`, pas `ramp.total_duration`.
Le nom exact n'est pas visible dans les documents fournis.

Si l'attribut cherché n'existe pas, la chaîne tombe silencieusement sur le fallback
suivant sans log, ce qui produit des timeouts incorrects sans indication.

### Comportement attendu
Ajouter un log DEBUG explicite à chaque étape de la chaîne pour faciliter le diagnostic,
et vérifier le nom réel de l'attribut de durée dans `RampMove`.

### Code cible

```python
@staticmethod
def _compute_backoff_timeout(sub_move: "Move", margin: float = 1.5) -> float:
    import logging
    _log = logging.getLogger(__name__)

    # Priorité 1 : attribut explicite
    estimated = getattr(sub_move, "estimated_duration_s", None)
    if estimated is not None and float(estimated) > 0:
        t = max(1.0, float(estimated) * margin)
        _log.debug("backoff timeout from estimated_duration_s: %.2fs", t)
        return t

    # Priorité 2 : RampMove.duration_s (nom le plus courant dans le code)
    for attr in ("duration_s", "total_duration_s", "total_duration"):
        ramp = getattr(sub_move, "ramp", sub_move)  # sub_move peut être le ramp lui-même
        total = getattr(ramp, attr, None)
        if total is not None and float(total) > 0:
            t = max(1.0, float(total) * margin)
            _log.debug("backoff timeout from ramp.%s: %.2fs", attr, t)
            return t

    # Priorité 3 : estimation brute steps / hz
    for steps_attr in ("total_steps", "step_count", "num_steps"):
        steps = getattr(sub_move, steps_attr, None)
        if steps is not None and int(steps) > 0:
            for hz_attr in ("target_hz", "cruise_steps_per_s", "max_steps_per_s"):
                hz = getattr(sub_move, hz_attr, None)
                if hz is not None and float(hz) > 0:
                    t = max(1.0, (float(steps) / float(hz)) * margin)
                    _log.debug(
                        "backoff timeout from %s/%s: %.2fs", steps_attr, hz_attr, t
                    )
                    return t

    _log.warning(
        "backoff timeout: no duration attribute found on %s, using 2.0s fallback",
        type(sub_move).__name__,
    )
    return 2.0
```

---

## N7 — Pas de limite haute sur `max_approach_steps` en cas de misconfiguration

### Fichier
`src/rpi/motion/engine.py`

### Méthode
`WindingEngine._home_lateral_axis`

### Problème
```python
max_approach_steps=int(steps_per_rev * 20),
```

`steps_per_rev = lateral_steps_per_revolution * lateral_microstepping`.
Si `lateral_microstepping` est mal configuré (ex. 256 au lieu de 32), `max_approach_steps`
peut valoir `200 * 256 * 20 = 1 024 000 steps`. À 100 RPM et 32 µstep cela représente
~6 400 µs par step → 6.5 secondes de mouvement. À 256 µstep cela représente **52 secondes**
de mouvement non stoppé avant de détecter l'absence d'endstop.

Le moteur heurte la butée mécanique bien avant, mais sans capteur, aucun arrêt matériel
ne se produit. Le firmware continue d'envoyer des steps contre la butée mécanique pendant
toute la durée de `max_approach_steps`.

### Comportement attendu
Plafonner `max_approach_steps` à une valeur absolue raisonnable indépendante du
microstepping, par exemple 5 secondes de mouvement à la vitesse d'approche.

### Code cible

```python
def _home_lateral_axis(
    self, *, axis_id, approach_rpm, search_rpm, backoff_steps
) -> tuple[bool, str | None]:
    steps_per_rev = (
        self._config.lateral_steps_per_revolution
        * self._config.lateral_microstepping
    )
    # Durée max d'approche : 5 secondes à la vitesse demandée
    # Indépendant du microstepping pour éviter les dérives de config
    approach_steps_per_s = (approach_rpm / 60.0) * steps_per_rev
    max_approach_steps_from_time = int(approach_steps_per_s * 5.0)

    # Plafond absolu : 20 tours (comportement actuel), mais aussi plafonné
    # par la durée pour protéger contre un microstepping mal configuré
    max_approach_steps = min(
        int(steps_per_rev * 20),
        max_approach_steps_from_time,
    )
    # Minimum garanti : au moins 2 tours pour que l'axe ait une chance d'atteindre la butée
    max_approach_steps = max(max_approach_steps, int(steps_per_rev * 2))

    move = HomingMove(
        ...
        max_approach_steps=max_approach_steps,
        ...
    )
```

---

## N8 — `notifySegmentExecuted` n'est pas thread-safe avec `sequence_is_newer_u16` pour RECOVERY

### Fichier
`src/esp32/src/comm_interface.cpp`

### Méthode
`CommInterface::notifySegmentExecuted`

### Problème
```cpp
void CommInterface::notifySegmentExecuted(uint16_t motion_seq)
{
    uint16_t current = last_executed_sequence_.load(std::memory_order_relaxed);
    if (sequence_is_newer_u16(motion_seq, current)) {
        last_executed_sequence_.store(motion_seq, std::memory_order_release);
    }
}
```

Cette méthode est appelée depuis `multiAxisExecutorTask` (Core 1). Dans RECOVERY,
elle peut être appelée deux fois en succession rapide :

1. Depuis DRAIN (pour la séquence qui a déclenché l'endstop)
2. Depuis RECOVERY (pour `last_drained_seq`)

Le load/compare/store n'est pas atomique. Si Core 0 lit `last_executed_sequence_`
entre ces deux stores (dans `buildStatusFrame`), il peut lire une valeur intermédiaire.

**En pratique** : comme `multiAxisExecutorTask` est sur Core 1 et `buildStatusFrame`
sur Core 0, et que `std::memory_order_release` garantit que Core 0 voit les stores
dans l'ordre, ce problème est théoriquement possible mais improbable sur ESP32 avec
le modèle mémoire ARM.

**Le vrai problème** est que le compare-and-store non atomique peut produire une
régression si `notifySegmentExecuted` est jamais appelé depuis plusieurs tâches.
Actuellement ce n'est pas le cas, mais le code n'a pas de protection explicite.

### Comportement attendu
Utiliser `compare_exchange_weak` pour rendre l'opération atomique.

### Code cible

```cpp
void CommInterface::notifySegmentExecuted(uint16_t motion_seq)
{
    uint16_t current = last_executed_sequence_.load(std::memory_order_relaxed);
    // Boucle CAS : avance last_executed_sequence_ seulement si motion_seq est plus récent
    while (sequence_is_newer_u16(motion_seq, current)) {
        if (last_executed_sequence_.compare_exchange_weak(
                current, motion_seq,
                std::memory_order_release,
                std::memory_order_relaxed)) {
            break;  // Store réussi
        }
        // current a été mis à jour par compare_exchange_weak en cas d'échec
        // → re-tester la condition de la boucle avec la nouvelle valeur
    }
}
```

---

## N9 — Pas de guard contre un double-appel à `home_lateral_axis` concurrent

### Fichier
`src/rpi/motion/engine.py`

### Méthode
`WindingEngine._home_lateral_axis`

### Problème
`home_lateral` vérifie `engine_state != IDLE` en entrée et passe en `HOMING`.
Mais `_home_lateral_axis` est appelé directement depuis les tests unitaires sans
passer par cette garde.

Si un test appelle `_home_lateral_axis` pendant qu'un homing est en cours (ex.
un thread de monitoring qui lit l'état), la `MoveQueue` peut recevoir deux `HomingMove`
simultanément. Le second sera exécuté après le premier, remettant le homing en état
inconnu avec une position home écrasée.

### Comportement attendu
`_home_lateral_axis` doit vérifier que l'engine est bien en `HOMING` (et non `IDLE`
ou autre) avant de continuer, pour documenter explicitement la précondition.

### Code cible

```python
def _home_lateral_axis(
    self, *, axis_id, approach_rpm, search_rpm, backoff_steps
) -> tuple[bool, str | None]:
    # Précondition : l'appelant doit avoir déjà mis l'engine en HOMING
    assert self._state.engine_state == EngineState.HOMING, (
        f"_home_lateral_axis called with engine_state="
        f"{self._state.engine_state!r}, expected HOMING"
    )
    ...
```

---

## Résumé des modifications par fichier

| Fichier | Priorité | Points | Description |
|---|---|---|---|
| `src/esp32/src/comm_interface.cpp` | 🔴 Critique | N1, N2, N5 | Ordre clearFlags dans ABSENT fail-safe, RECOVERY sur timeout, defer ring avant reset |
| `src/rpi/transport/streamer.py` | 🔴 Critique | N3, N4 | Property `last_sent_motion_seq`, baseline `segments_dropped` par phase |
| `src/rpi/motion/move_queue.py` | 🟠 Important | N4, N6 | Baseline streamer, `_compute_backoff_timeout` robuste avec logs |
| `src/esp32/src/comm_interface.cpp` | 🟠 Important | N8 | `notifySegmentExecuted` CAS atomique |
| `src/rpi/motion/engine.py` | 🟡 Robustesse | N7, N9 | Plafond `max_approach_steps`, assert précondition HOMING |

---

## Ordre d'application recommandé

1. **N1** : une ligne, risque zero, corrige un ordre d'opération dans le fail-safe ABSENT.

2. **N3** : expose `last_sent_motion_seq` — requis pour que R7 (`_check_armed_phase_result`)
   ne lève pas `AttributeError` en production.

3. **N4** : baseline `segments_dropped` — corrige les faux positifs au démarrage de la
   phase `search`. À appliquer avec la modification du constructeur `MultiAxisRampStreamer`
   et de `_stream_homing_sub_move`.

4. **N2** : traiter `ESP_ERR_TIMEOUT` dans DRAIN comme une transition vers RECOVERY.
   Indépendant des autres.

5. **N5** : notifier les entrées `defer_*` échues avant de vider le ring dans RECOVERY.
   Améliore la cohérence de `last_executed_sequence` vue par le host.

6. **N8** : CAS atomique dans `notifySegmentExecuted`. Correction de sécurité bas risque.

7. **N6** : `_compute_backoff_timeout` avec logs et recherche multi-attributs.
   Appliqué en même temps que N4 si possible.

8. **N7** + **N9** : plafond `max_approach_steps` et assert précondition.
   Peuvent être appliqués indépendamment, faible risque de régression.