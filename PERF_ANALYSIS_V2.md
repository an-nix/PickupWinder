# PERF_ANALYSIS_V2.md

## 1) Changelog PERF_ANALYSIS V1 → V2

| Fichier | Correction | Impact attendu |
|---|---|---|
| src/esp32/src/stepper_queue.cpp | Remplacement de `vTaskDelay(1)` par `taskYIELD()` adaptatif (yield uniquement si `ring_free > STEP_RING_SIZE/2`) | Évite un sommeil de 10 ms tous les 8 blocs en charge haute vitesse, baisse forte du risque d’underrun |
| src/esp32/src/step_types.h | `STEP_STREAM_START_FILL` passé de `64` à `(4U * PART_SIZE)` (128) | Double le buffer de démarrage (0.4 ms → 0.8 ms à 1500 RPM), meilleure absorption de latence host→ESP32 |
| src/esp32/src/stepper_queue.h | Ajout du flag atomique `multi_exec_active_` + accès `setMultiExecActive()` / `isMultiExecActive()` | Exclusion simple entre exécution per-axis et multi-axis sur un même driver |
| src/esp32/src/comm_interface.cpp | Multi-axis exécuteur marque le driver actif autour de `executeConstantRateBlock()` | Supprime la race de concurrence `pushBlock()/maybeStartDriver()` entre deux tâches distinctes |
| src/rpi/transport/streamer.py | Docstring `_prefill()` alignée avec l’implémentation (`_initial_steps_per_segment`) | Documentation cohérente, évite les ambiguïtés de maintenance |

**Choix pour la correction 3:** Option A (flag atomique), retenue pour simplicité, coût faible, et pas de suspension/reprise de tâche (latence de fallback single-axis préservée).

---

## 2) Sources complètes modifiées

Les fichiers modifiés en V2 (version complète actuelle dans le workspace):

- src/esp32/src/stepper_queue.cpp
- src/esp32/src/stepper_queue.h
- src/esp32/src/step_types.h
- src/rpi/transport/streamer.py
- src/esp32/src/comm_interface.cpp

### Extrait correction 1 (stepper_queue.cpp)

```cpp
if (work_done >= WORK_BUDGET) {
    work_done = 0;
    const uint32_t ring_free = driver.ringFreeSlots();
    if (ring_free > (STEP_RING_SIZE / 2U)) {
        taskYIELD();
    }
}
```

### Extrait correction 3 (stepper_queue.h)

```cpp
std::atomic<bool> multi_exec_active_ {false};

void setMultiExecActive(bool active) {
    multi_exec_active_.store(active, std::memory_order_release);
}

bool isMultiExecActive() const {
    return multi_exec_active_.load(std::memory_order_acquire);
}
```

### Extrait correction 3 (comm_interface.cpp)

```cpp
StepperQueue* axis_queue = self->queues_[axis_id];
axis_queue->setMultiExecActive(true);
esp_err_t err = axis_queue->executeConstantRateBlock(
    seg.axes[a].direction,
    seg.axes[a].step_count,
    seg.duration_us);
axis_queue->setMultiExecActive(false);
```

### Extrait correction 2 (step_types.h)

```cpp
#define STEP_STREAM_START_FILL  (4U * PART_SIZE)
```

### Extrait correction 4 (streamer.py)

```python
"""Pre-send segments to seed the ESP32 planner queue before RMT starts.

The prefill target is computed from the startup speed estimate
(_initial_steps_per_segment), not the live segment state, to avoid
low-speed misclassification at startup.

Speed tiers (steps_per_segment):
  - < 10: 64 segments (low speed, conservative fill)
  - >= 10: required_lookahead(initial_steps) (speed-appropriate fill)
..."""
```

---

## 3) Sources complètes pour analyse future

Fichiers complets de référence dans cette branche:

- src/esp32/src/comm_interface.h
- src/esp32/src/comm_interface.cpp

Paramètres timing FreeRTOS/CPU (sdkconfig):

- src/esp32/sdkconfig.esp32
  - `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=160`
  - `CONFIG_FREERTOS_HZ=100` (tick = 10 ms)
  - `CONFIG_FREERTOS_TICK_SUPPORT_CORETIMER=y`
  - `CONFIG_FREERTOS_CORETIMER_0=y`
  - `CONFIG_FREERTOS_SYSTICK_USES_CCOUNT=y`

Note: `CONFIG_FREERTOS_MAX_PRIORITIES` n’est pas explicitement fixé dans le sdkconfig actuel (valeur par défaut ESP-IDF).

CMake ESP-IDF:

- src/esp32/CMakeLists.txt

---

## 4) Vérification numérique V2

Hypothèses: `steps/rev = 6400`, bloc = `8 × 64 = 512 steps`.

### Correction 1 — temps de drain 8 blocs (avant/après)

Formules:

- $f_{step} = \frac{RPM}{60} \times 6400$
- $t_{drain} = \frac{512}{f_{step}}$

| RPM | $f_{step}$ (steps/s) | $t_{drain}$ pur | V1 (avec `vTaskDelay(1)`=10ms) | V2 (yield adaptatif) |
|---:|---:|---:|---:|---:|
| 250 | 26,666.67 | 19.2 ms | ~29.2 ms | ~19.2 ms |
| 660 | 70,400 | 7.273 ms | ~17.273 ms | ~7.273 ms |
| 1500 | 160,000 | 3.2 ms | ~13.2 ms | ~3.2 ms |

Conclusion: à 1500 RPM, la pause de 10 ms dominait le cycle (≈76% du temps).

### Correction 2 — buffer ring `STEP_STREAM_START_FILL`

À 1500 RPM:

- $f_{step}=160000$ steps/s
- 64 steps: $64/160000 = 0.0004$ s = **0.4 ms**
- 128 steps: $128/160000 = 0.0008$ s = **0.8 ms**

Donc V2 double le buffer de démarrage.

### Correction 3 — scénario race exact et élimination

Scénario V1:

1. `multiAxisExecutorTask` appelle `executeConstantRateBlock()` sur un axe.
2. En parallèle, `executorTask` per-axis du même axe traite sa queue.
3. Les deux chemins peuvent atteindre `pushBlock()` / `maybeStartDriver()` sans exclusion.
4. Effets possibles: ordre non déterministe des écritures ring/start/stop, blocages intermittents, underruns.

Mitigation V2 (Option A):

- Le multi-axis pose `multi_exec_active_ = true` autour de son appel.
- Le per-axis `executorTask` détecte ce flag et ne pousse pas de bloc pendant cette fenêtre.
- Le partage du driver devient mutuellement exclusif au niveau logique.

---

## 5) Points d’attention restants

1. Le flag `multi_exec_active_` est posé par segment/axe; un guard RAII renforcerait la sûreté en cas de futur refactoring.
2. Le scheduler est à `CONFIG_FREERTOS_HZ=100` (tick 10 ms) : éviter toute réintroduction de `vTaskDelay(1)` en chemin critique.
3. `STEP_STREAM_START_FILL=128` améliore le startup, mais reste inférieur à une latence host de 3–5 ms à 160 kHz; la robustesse dépend encore du préremplissage multi-segments côté host.
4. Reste à instrumenter terrain: corréler `ring_free`, `planner_queue_free`, `underrun_count`, `last_executed_sequence` sur 250/660/1500 RPM.
5. L’exposition dynamique `STEP_RING_SIZE` dans `StatusPayload` (demande 3.c initiale) n’est toujours pas implémentée.
