# CRITIQUE_V9 — Analyse critique firmware src/esp32/ + StatusPayload

**Date** : 2026-04-20  
**Base** : document index 6 (sources complètes V9)

---

## Tableau de synthèse

| ID | Sévérité | Fichier | Problème |
|----|----------|---------|----------|
| **A1** | **CRITIQUE (FAIT + VÉRIFIÉ)** | `comm_interface.cpp` | `setMultiExecActive` posé dans la boucle axes (B1 jamais appliqué) |
| **A2** | **SÉVÈRE (FAIT + VÉRIFIÉ)** | `step_types.h` | `STEP_STREAM_START_FILL=128` cause à-coups de rampe (V4 non appliqué) |
| **A3** | **SÉVÈRE (FAIT + VÉRIFIÉ)** | `motion_planner.h` | `EXEC_TIME_BUDGET_US=3000µs` < `segment_duration=4000µs` |
| **A4** | **SÉVÈRE (FAIT + VÉRIFIÉ)** | `comm_interface.cpp` | `pushBlock()` bloquant dans DRAIN sans pré-démarrage RMT |
| **A5** | **MODÉRÉ (FAIT + VÉRIFIÉ)** | `motion_planner.cpp` | `handleFlush()` draine deux queues sans borne |
| **A6** | **MODÉRÉ (FAIT + VÉRIFIÉ)** | **`comm_interface.cpp`** | **`last_accepted_block_seq_` écrasé avec `flush_sequence` (espaces différents)** |
| **S1** | **SÉVÈRE (FAIT + VÉRIFIÉ)** | **`comm_interface.cpp`** | **`defer_fire_us` basé sur `scheduled_time_us` (passé), pas le temps réel** |
| **A7** | **MODÉRÉ (FAIT + VÉRIFIÉ)** | `comm_interface.cpp` | Guard stale-exec : faux positif possible en début de mouvement |
| **A8** | **MODÉRÉ (FAIT + VÉRIFIÉ)** | `stepper_queue.cpp` | `loop_start_us` recalculé à chaque itération, guard 5000µs inopérant |
| **A9** | **FAIBLE (FAIT + VÉRIFIÉ)** | `comm_interface.cpp` | Cache doublon SPI limité à 1 entrée |
| **A10** | **FAIBLE (FAIT + VÉRIFIÉ)** | `motion_planner.h` | `SEGMENT_PREFILL_THRESHOLD=16` déclaré mais jamais utilisé (code mort) |
| **A11** | **FAIBLE (FAIT + VÉRIFIÉ)** | `comm_interface.cpp` | `defer_fire_us`/`defer_seqs` `static` → non-réentrable, non documenté |

---

## A1 — CRITIQUE : `setMultiExecActive` dans la boucle axes

La correction B1 (PERF_ANALYSIS_V3) n'a jamais été appliquée. La structure actuelle dans DRAIN :

```cpp
// CODE ACTUEL — INCORRECT
for (uint8_t a = 0; a < seg.axis_count; ++a) {
    const uint8_t axis_id = seg.axis_ids[a];
    if (...) {
        self->queues_[axis_id]->setMultiExecActive(true);  // ← DANS la boucle
        guarded_axis_ids[guarded_axis_count++] = axis_id;
    }
}
// ... executeConstantRateBlock ...
clearMultiExecFlags();
```

Le per-axis executor (prio 24 > multi_exec prio 20) peut préempter entre l'itération axis 0 et axis 1. À ce moment, `multi_exec_active_` pour l'axe 1 est encore `false` → le per-axis executor s'y insère et appelle `pushExpandedBlock` simultanément → corruption possible du ring.

**Correction** : deux passes séparées, identique à PERF_ANALYSIS_V3 B1.

```cpp
// PASSE 1 : lever TOUS les flags avant toute écriture
uint8_t guarded_axis_ids[MULTI_AXIS_MAX_AXES] = {};
uint8_t guarded_axis_count = 0;
for (uint8_t a = 0; a < seg.axis_count; ++a) {
    const uint8_t axis_id = seg.axis_ids[a];
    if (axis_id < self->n_motors_ && self->queues_[axis_id]) {
        self->queues_[axis_id]->setMultiExecActive(true);
        guarded_axis_ids[guarded_axis_count++] = axis_id;
    }
}
// PASSE 2 : écrire les steps
for (uint8_t a = 0; a < seg.axis_count; ++a) { ... executeConstantRateBlock ... }
// PASSE 3 : baisser APRÈS toutes les axes
clearMultiExecFlags();
```

> Correction appliquée et vérifiée dans `src/esp32/src/comm_interface.cpp` : les flags `multi_exec_active` sont désormais levés avant d’écrire les étapes dans le ring.

---

## A2 — SÉVÈRE : `STEP_STREAM_START_FILL=128`

`step_types.h` maintient la valeur 128. Elle retarde le démarrage RMT de ~16 segments en début de rampe : les premiers segments (1, 2, 3… steps) s'accumulent dans le ring jusqu'à atteindre 128 entries, puis sont consommés en rafale, produisant exactement les à-coups observés.

**Correction** (PERF_ANALYSIS_V4 R1) :

```c
#define STEP_STREAM_START_FILL    16U   // = 2 × PART_SIZE
#define STEP_STREAM_RESTART_FILL  PART_SIZE  // = 8
```

Le coast-mode maintient le RMT actif entre les segments — `STEP_STREAM_START_FILL` n'intervient qu'au premier démarrage. Avec 16, les deux premiers callbacks ISR (8+8 steps) trouvent des données disponibles, ce qui est la garantie minimale requise.

> Correction appliquée et vérifiée dans `src/esp32/src/step_types.h` : `STEP_STREAM_START_FILL=16U` et `STEP_STREAM_RESTART_FILL=PART_SIZE`.

---

## A3 — SÉVÈRE : `EXEC_TIME_BUDGET_US=3000µs`

`EXEC_TIME_BUDGET_US` doit être supérieur à `segment_duration_us=4000µs`. Si un segment prend plus de 3ms à traiter (pushBlock bloquant sur ring plein), le budget est dépassé dans DRAIN → `goto exit_drain` → `taskYIELD` 1ms → le segment suivant est retardé → décalage en cascade sur la rampe.

**Correction** (PERF_ANALYSIS_V4 R3) :

```cpp
// Dans motion_planner.h
static constexpr int64_t  EXEC_TIME_BUDGET_US = 8000;  // 3000 → 8000
```

> Correction appliquée et vérifiée dans `src/esp32/src/motion_planner.h` : `EXEC_TIME_BUDGET_US` vaut maintenant `8000`.

---

## A4 — SÉVÈRE : `pushBlock()` bloquant dans DRAIN

`executeConstantRateBlock` → `pushExpandedBlock` → `pushBlock` → `ulTaskNotifyTake(5ms)` si ring plein. Cet appel bloquant se produit à l'intérieur du `case ExecState::DRAIN`, paralysant le state machine entier.

**Correction** (PERF_ANALYSIS_V4 R2) : pré-démarrer le RMT pour chaque axe **avant** d'appeler `executeConstantRateBlock`, juste après les checks endstop et lateral :

```cpp
// AVANT executeConstantRateBlock, après clearMultiExecFlags setup :
for (uint8_t a = 0; a < seg.axis_count; ++a) {
    const uint8_t axis_id = seg.axis_ids[a];
    if (axis_id < self->n_motors_ && self->queues_[axis_id] &&
        !self->queues_[axis_id]->driver().isStreaming()) {
        self->queues_[axis_id]->kickStart();
    }
}
```

En coast-mode, `isStreaming()=true` → `kickStart` ne fait rien → zéro overhead en croisière. Le pré-démarrage n'intervient qu'au premier segment ou après `emergencyStop`.

> Correction appliquée et vérifiée dans `src/esp32/src/comm_interface.cpp` : les axes non-streaming sont pré-démarrés avant `executeConstantRateBlock()`.

---

## A5 — MODÉRÉ : `handleFlush()` sans borne de drain

La nouvelle `handleFlush()` draine les deux queues en boucle while non bornée :

```cpp
while (xQueueReceive(cmd_queue_, &dropped_block, 0) == pdTRUE) { ++dropped_cmd; }
while (xQueueReceive(segment_queue_, &dropped_seg, 0) == pdTRUE) { ++dropped_planned; }
```

Si `segment_queue_` contient 128 éléments et `cmd_queue_` 64, cette boucle peut prendre plusieurs millisecondes et déclencher le watchdog du planner.

**Correction** : borner les deux drains :

```cpp
uint32_t dropped_cmd = 0;
while (dropped_cmd < MULTI_AXIS_QUEUE_DEPTH &&
       xQueueReceive(cmd_queue_, &dropped_block, 0) == pdTRUE) {
    ++dropped_cmd;
}
uint32_t dropped_planned = 0;
while (dropped_planned < SEGMENT_QUEUE_DEPTH &&
       xQueueReceive(segment_queue_, &dropped_seg, 0) == pdTRUE) {
    ++dropped_planned;
}
```

> Correction appliquée et vérifiée dans `src/esp32/src/motion_planner.cpp` : les drains de flush sont maintenant bornés par `MULTI_AXIS_QUEUE_DEPTH` et `SEGMENT_QUEUE_DEPTH`.

---

## A6 — MODÉRÉ : confusion `block_seq` vs `flush_sequence`

Dans `handleFlush()` :

```cpp
last_accepted_block_seq_ = flush_payload.flush_sequence;
```

`flush_sequence` est un `motion_sequence` (espace des segments, typiquement dans les milliers). `last_accepted_block_seq_` est un `block_seq` (espace des blocs SPI, typiquement dans les dizaines). Après un `flush(flush_seq=500)`, le guard dans `handleMultiAxisSegmentBlock()` :

```cpp
if (sequence_is_stale_or_equal_u16(hdr_val.block_seq, last_accepted_block_seq_))
    return ESP_OK;  // DROP
```

Un bloc avec `block_seq=50` serait dropé car `int16_t(50 - 500) = -450 ≤ 0` → **blocage total de la reprise après flush** si `block_seq < flush_sequence`.

**Correction** : ne pas toucher `last_accepted_block_seq_` dans `handleFlush()`. Réinitialiser à `0xFFFF` à la place pour forcer l'acceptation du prochain bloc :

```cpp
// Dans handleFlush()
last_accepted_block_seq_ = 0xFFFFu;  // reset pour accepter le prochain bloc
// NE PAS faire : last_accepted_block_seq_ = flush_payload.flush_sequence;
```

> Correction appliquée et vérifiée dans `src/esp32/src/comm_interface.cpp` : `last_accepted_block_seq_` est désormais réinitialisé à `0xFFFFu` dans `handleFlush()`.

---

## A7 — MODÉRÉ : guard stale-exec potentiellement prématuré

Dans DRAIN, avant d'exécuter un segment :

```cpp
const uint16_t last_exec = self->last_executed_sequence_.load(std::memory_order_relaxed);
if (sequence_is_stale_or_equal_u16(seg.motion_sequence, last_exec)) {
    ++batch_index;
    continue;  // skip
}
```

`last_executed_sequence_` est mis à jour par `fireDeferred()` qui respecte `defer_fire_us[idx]` (timestamp futur). Au début d'un mouvement, si `fireDeferred()` n'a pas encore tourné pour les premiers segments, `last_exec` peut valoir la séquence du dernier segment du mouvement **précédent**. Si le nouveau mouvement recommence la numérotation à une valeur inférieure (ex: `motion_seq` repart de 1 après un flush), le guard skipperait les premiers segments légitimes.

Ce risque est atténué par le reset de `last_planned_motion_seq_` dans `handleFlush()`, mais `last_executed_sequence_` lui n'est jamais reset à 0. Il reste à 0xFFFF au démarrage, et après un flush reste à la dernière valeur notifiée.

**Correction appliquée** : le guard stale-exec a été supprimé du `DRAIN` de `multiAxisExecutorTask()`. Le filtrage monotone reste du ressort du planner (`last_planned_motion_seq_` + flush sentinel), ce qui évite les faux positifs au redémarrage d'une nouvelle séquence.

> Correction appliquée et vérifiée dans `src/esp32/src/comm_interface.cpp` : l'executor ne skippe plus un segment uniquement parce que `last_executed_sequence_` provient d'un mouvement précédent.

---

## A8 — MODÉRÉ : `loop_start_us` recalculé à chaque itération

Dans `stepper_queue.cpp`, `executorTask()` :

```cpp
do {
    const int64_t loop_start_us = esp_timer_get_time();  // ← recalculé ici
    ...
    if (work_done >= WORK_BUDGET) {
        if ((esp_timer_get_time() - loop_start_us) > 5000) taskYIELD();
    }
} while (xQueueReceive(self->queue_, &block, 0) == pdTRUE);
```

`loop_start_us` est déclaré **à l'intérieur** du do-while, donc réinitialisé à chaque bloc. Il mesure le temps d'un seul bloc (toujours < 1ms), jamais le temps total du batch. Le guard `> 5000µs` ne déclenchera jamais.

**Correction** :

```cpp
const int64_t batch_start_us = esp_timer_get_time();  // AVANT le do-while
do {
    ...
    if (work_done >= WORK_BUDGET) {
        work_done = 0;
        const uint32_t ring_free = driver.ringFreeSlots();
        if (ring_free > (STEP_RING_SIZE / 2U)) {
            taskYIELD();
        } else if ((esp_timer_get_time() - batch_start_us) > 5000) {
            taskYIELD();
        }
    }
} while (xQueueReceive(self->queue_, &block, 0) == pdTRUE);
```

> Correction appliquée et vérifiée dans `src/esp32/src/stepper_queue.cpp` : le guard temporel utilise désormais `batch_start_us` hors du `do-while`.

---

## A10 — FAIBLE : `SEGMENT_PREFILL_THRESHOLD` code mort

```cpp
static constexpr uint32_t SEGMENT_PREFILL_THRESHOLD = 16;  // jamais utilisé
```

À supprimer ou à implémenter : bloquer le premier `kickStart` jusqu'à ce que `segment_queue_` ait ≥ 16 segments planifiés, pour garantir que le ring est rempli avant le démarrage.

> Correction appliquée et vérifiée dans `src/esp32/src/motion_planner.h` : `SEGMENT_PREFILL_THRESHOLD` a été supprimé car inutilisé.

---

## A9 — FAIBLE : cache doublon SPI limité à 1 entrée

Le cache de déduplication SPI conservait uniquement la dernière requête traitée. Une retransmission exacte plus ancienne qu'une seule transaction pouvait donc être rejouée inutilement.

**Correction appliquée** : la déduplication SPI s'appuie maintenant sur un petit cache circulaire des quatre dernières signatures exactes (`sequence`, `msg_type`, `payload_length`, `crc`).

> Correction appliquée et vérifiée dans `src/esp32/src/comm_interface.h` et `src/esp32/src/comm_interface.cpp` : la déduplication couvre désormais plusieurs retries récents, sans réexécuter les effets de bord.

---

## A11 — FAIBLE : buffers différés `static`

Les buffers `defer_fire_us` et `defer_seqs` étaient déclarés `static` à l'intérieur de `multiAxisExecutorTask()`. Cela introduisait un état caché partagé entre d'éventuelles instances futures de la tâche.

**Correction appliquée** : ces buffers sont désormais alloués localement à la tâche, avec une initialisation explicite à zéro au démarrage du thread d'exécution.

> Correction appliquée et vérifiée dans `src/esp32/src/comm_interface.cpp` : l'état différé est maintenant local à la tâche, sans stockage `static` partagé.

---

## S1 — SÉVÈRE : `fireDeferred()` notifie avant l'exécution réelle

C'est le problème le plus subtil du pipeline de status.

`defer_fire_us[idx]` est calculé ainsi dans DRAIN :

```cpp
defer_fire_us[idx] = seg.scheduled_time_us + static_cast<int64_t>(seg.duration_us);
```

`scheduled_time_us` est le timestamp **de planification** (valeur de `timeline_us_` au moment où le planner a enqueué le segment). Si le pipeline a accumulé du retard (backpressure, ring plein, yields), `scheduled_time_us` peut être plusieurs secondes dans le **passé** par rapport à `esp_timer_get_time()`.

Dans ce cas, `defer_fire_us[idx] < now` **dès l'écriture** → `fireDeferred()` au prochain passage au top de boucle notifie `last_executed_sequence` immédiatement, **avant que les steps aient eu le temps d'être consommés par l'ISR**.

Le host reçoit une séquence avancée, croit que le MCU a de la marge, envoie plus de blocs, aggrave le backpressure.

**Correction** :

```cpp
// Dans DRAIN, remplacer :
defer_fire_us[idx] = seg.scheduled_time_us + static_cast<int64_t>(seg.duration_us);

// Par :
defer_fire_us[idx] = esp_timer_get_time() + static_cast<int64_t>(seg.duration_us);
```

Cela garantit que la notification est envoyée `duration_us` après l'écriture dans le ring, ce qui correspond approximativement à quand les steps seront consommés par l'ISR à vitesse nominale.
> Correction appliquée et vérifiée dans `src/esp32/src/comm_interface.cpp` : `defer_fire_us` est maintenant basé sur `esp_timer_get_time()`.
---

## Analyse du StatusPayload

### Ce qui fonctionne bien

- `last_executed_sequence_` est `atomic<uint16_t>` avec `memory_order_acquire` → lecture cross-core correcte.
- `ring_write_`/`ring_read_` sont `atomic<uint32_t>` avec ordering correct → `ringFreeSlots()` safe.
- `uxQueueSpacesAvailable()` est safe depuis n'importe quel contexte task.
- Le snapshot n'est pas atomique globalement (acceptable pour un contrôleur temps-réel).
- La fréquence d'appel (~250/s) représente ~0.05% du CPU Core 0. Négligeable.

### Problèmes identifiés

**P1** — Les tableaux `[SPI_MAX_AXES]` = `[4]` pour un système à 2 axes gaspillent 16 octets dans le payload (les entrées 2 et 3 sont toujours à zéro). Plus important : le host doit savoir qu'elles sont vides.

**P2** — `planner_queue_free` indique les slots libres, mais pas le nombre total (`SEGMENT_QUEUE_DEPTH=128`). Le host doit connaître la constante pour calculer le pourcentage de remplissage. `segment_queue_depth` devrait aussi être exposé (ou définir `SEGMENT_QUEUE_DEPTH` côté host également).

**P3** — Aucun compteur de segments droppés n'est exposé. Si `segments_dropped_` du planner monte, le host ne le voit pas. C'est le seul indicateur fiable d'un problème de séquençage.

**P4** — `last_planned_motion_seq_` n'est pas exposé. La différence `last_planned_motion_seq_ - last_executed_sequence_` donne la profondeur réelle du buffer pipeline (segments planifiés mais pas encore exécutés). C'est plus précis que `planner_queue_free` seul car inclut les segments dans `segment_queue_` ET dans le ring.

### StatusPayload optimal recommandé

Il reste **452 octets** disponibles dans le payload SPI. Voici les ajouts à haute valeur diagnostique, **sans casser la compatibilité binaire** (ajouter à la fin, bumper `SPI_MSG_VERSION` à 2) :

```c
struct __attribute__((packed)) StatusPayload {
    // ── Champs existants (48 octets, inchangés) ──────────────────────────
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
    uint8_t  lateral_endstop_state;
    uint8_t  endstop_armed_mask;
    uint16_t last_executed_sequence;
    uint8_t  planner_queue_free;

    // ── Nouveaux champs diagnostiques (10 octets) ────────────────────────
    // VERSION 2 : ajouter à la fin pour backward compat

    /** Dernier motion_sequence planifié par le planner.
     *  (last_planned_motion_seq_ - last_executed_sequence) = profondeur pipeline. */
    uint16_t last_planned_sequence;

    /** Nombre de segments rejetés (hors-ordre ou doublons) depuis RESET_STATS. */
    uint16_t segments_dropped;

    /** Nombre total de slots dans segment_queue_ (constant = SEGMENT_QUEUE_DEPTH).
     *  Permet au host de calculer le % de remplissage : 1 - planner_queue_free/segment_queue_depth. */
    uint8_t  segment_queue_depth;

    /** Bitfield d'état diagnostique rapide.
     *  Bit 0 : planner_starved (CMD_POLL_TIMEOUT expiré sans données)
     *  Bit 1 : ring_stall (pushBlock a dû retenter > 1 fois sur au moins 1 axe)
     *  Bit 2 : coast_active (au moins 1 axe en coast-mode actuellement)
     *  Bit 3 : stale_drops (au moins 1 segment stale skipé dans DRAIN depuis reset)
     *  Bits 4-7 : réservés */
    uint8_t  diag_flags;

    uint8_t  reserved[2];  // padding pour alignement futur
};
static_assert(sizeof(StatusPayload) == 58, "StatusPayload V2 must be 58 bytes");
```

Pour exposer ces champs, il faut :
1. Ajouter `last_planned_motion_seq_` comme méthode publique dans `MotionPlanner`.
2. Ajouter `segments_dropped_u16_` (tronqué à 16 bits, reset sur `handleResetStats()`) dans `MotionPlanner`.
3. Ajouter `diag_flags_` comme `std::atomic<uint8_t>` dans `CommInterface`, mis à jour par le planner et l'executor.

> Correctif P3+P4 appliqué et vérifié : `last_planned_sequence` et `segments_dropped` sont maintenant exposés dans `StatusPayload` côté firmware et côté host Python, avec `SPI_MSG_VERSION=2` et documentation protocole mise à jour. La variante 58 octets ci-dessus reste une recommandation plus large, non entièrement implémentée dans ce lot.

---

## Récapitulatif priorité d'intervention

| Priorité | ID | Action | Impact | Statut |
|----------|-----|--------|--------|--------|
| 1 | A6 | Corriger `last_accepted_block_seq_ = flush_sequence` → `= 0xFFFF` | Bloque la reprise après flush | FAIT + VÉRIFIÉ |
| 2 | S1 | `defer_fire_us` : utiliser `esp_timer_get_time()` au lieu de `scheduled_time_us` | Notifications prématurées | FAIT + VÉRIFIÉ |
| 3 | A1 | Deux passes séparées pour `setMultiExecActive` | Race condition per-axis/multi-exec | FAIT + VÉRIFIÉ |
| 4 | A2 | `STEP_STREAM_START_FILL` 128 → 16 | À-coups de rampe | FAIT + VÉRIFIÉ |
| 5 | A3 | `EXEC_TIME_BUDGET_US` 3000 → 8000 | Décalage segment > budget | FAIT + VÉRIFIÉ |
| 6 | A4 | Pré-kick RMT avant `executeConstantRateBlock` | pushBlock bloquant dans DRAIN | FAIT + VÉRIFIÉ |
| 7 | A5 | Borner drain dans `handleFlush()` | Watchdog planner | FAIT + VÉRIFIÉ |
| 8 | A8 | `loop_start_us` hors du do-while | Guard temporel inopérant | FAIT + VÉRIFIÉ |
| 9 | A7 | Supprimer le stale-exec guard de l'executor | Faux positifs après reprise | FAIT + VÉRIFIÉ |
| 10 | A9 | Étendre le cache de déduplication SPI | Retries exacts plus robustes | FAIT + VÉRIFIÉ |
| 11 | StatusPayload P3+P4 | Exposer `last_planned_sequence` + `segments_dropped` | Diagnostic backpressure | FAIT + VÉRIFIÉ |
| 12 | A10 | Supprimer `SEGMENT_PREFILL_THRESHOLD` (code mort) | Nettoyage | FAIT + VÉRIFIÉ |
| 13 | A11 | Rendre les buffers différés locaux à la tâche | Réentrance et clarté d'état | FAIT + VÉRIFIÉ |