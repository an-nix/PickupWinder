# Analyse du bug de Homing — PickupWinder

## Configuration mécanique de référence

| Paramètre | Valeur |
|-----------|--------|
| Tige filetée | M6 (pas = 1 mm/tour) |
| Steps/rev moteur | 64 |
| Microstepping | 32 |
| **Steps/rev total** | **2048** |
| **Steps/mm** | **2048** |
| 1 tour moteur | 1 mm de déplacement |
| **Vitesse d'approche homing** | **15 RPM** |

Quelques valeurs utiles à 15 RPM :

| Grandeur | Calcul | Résultat |
|----------|--------|---------|
| Fréquence steps | 15/60 × 2048 | **512 steps/s** |
| Déplacement/s | 15/60 × 1 mm | **0,25 mm/s** |
| Surtravel par ms de retard | 0,25 mm/s × 0,001 s | **0,25 mm/ms** |
| Surtravel pour 5 ms de rebond | 0,25 × 5 | **1,25 mm** |
| Surtravel pour 10 ms de rebond | 0,25 × 10 | **2,5 mm** |

---

## Symptômes observés

- L'axe approche l'endstop et s'arrête brièvement au contact
- Il repart quelques mm supplémentaires avant de s'arrêter vraiment
- La phase backoff démarre mais se termine sur un timeout
- Le comportement est **intermittent** (parfois ça fonctionne)

---

## Causes identifiées

### Cause #1 — Race condition ISR endstop (PRINCIPALE)

**Fichier :** `esp32/src/motion/stepper_driver.cpp` — `endstopIsrHandler()`

Le capteur est de type **NO/NC à deux contacts**. Lors d'un actionnement, deux fronts GPIO arrivent quasi-simultanément (quelques µs d'écart hardware). L'ISR est configurée sur `GPIO_INTR_ANYEDGE` sur les deux pins.

**Le bug :** entre les deux fronts, l'état transitoire des deux pins peut produire `EndstopSignalState::INVALID` (NO et NC simultanément actifs). Le code actuel remet le compteur de confirmation à zéro sur INVALID :

```cpp
// stepper_driver.cpp — comportement ACTUEL (buggé)
if (raw == EndstopSignalState::INVALID) {
    drv->endstop_closed_confirmations_ = 0;  // ← reset intempestif !
    ...
    return;
}
```

Résultat : si le front INVALID arrive entre les deux fronts CLOSED, le compteur est remis à zéro et `endstop_active_` n'est jamais latché lors du premier passage. Le moteur continue donc de tourner jusqu'au prochain rebond mécanique qui valide enfin le compteur.

**Conséquence mécanique :** à **15 RPM** sur tige M6 (1 mm/tour), le déplacement est de **0,25 mm/s**. Chaque ms de retard à valider le latch représente **0,25 mm** de surtravel. Un cycle de rebond mécanique typique de 5 à 10 ms produit donc **1,25 à 2,5 mm** de surtravel — ce qui correspond exactement aux quelques mm observés.

---

### Cause #2 — Latence entre latch endstop et arrêt RMT

**Fichier :** `esp32/src/motion/stepper_driver.cpp` — `encode_steps()`

```cpp
// Vérification au début du callback, toutes les PART_SIZE = 4 steps
if (drv->endstop_active_.load(std::memory_order_relaxed)) {
    drv->rmt_stopped_.store(true, std::memory_order_relaxed);
    *done = true;
    return 0;
}
```

Même quand `endstop_active_` est correctement latché, l'arrêt ne prend effet qu'au prochain appel du callback RMT. À **15 RPM** avec 2048 steps/rev :
- Fréquence de steps = 15/60 × 2048 = **512 steps/s**
- Durée de 4 steps (PART_SIZE) = 4 / 512 = **7,8 ms**
- Surtravel résiduel = 0,25 mm/s × 7,8 ms ≈ **0,002 mm** (totalement négligeable)

À 15 RPM, la latence RMT est donc sans impact pratique. La cause #1 est quasi-seule responsable du surtravel observé.

---

### Cause #3 — `backoff_steps` trop petit (Python)

**Fichier :** `rpi/src/motion/move.py` — `HomingMove`

La valeur par défaut dans le code est `backoff_steps = 3200`. Avec votre mécanique :

| Valeur | En steps | En mm |
|--------|----------|-------|
| Défaut code (`3200`) | 3200 | **1,56 mm** |
| Minimum recommandé | 4096 | 2 mm |
| Recommandé | 6144 | 3 mm |

Un backoff de 1,56 mm est insuffisant : si le surtravel a déjà consommé 1–2 mm au-delà du point de contact, le moteur peut ne pas réussir à libérer l'endstop avant que `_wait_for_endstop_open()` expire (timeout de 1,5 s).

---

### Cause #4 — `recovery_guard_s` trop court (Python)

**Fichier :** `rpi/src/motion/move_queue.py` — `_wait_for_post_hit_recovery()`

```python
def _wait_for_post_hit_recovery(self, ..., recovery_guard_s: float = 0.080):
```

Le délai de 80 ms entre l'arrêt détecté (`running_mask == 0`) et le lancement du backoff peut être insuffisant. Entre le moment où le firmware positionne son cycle RECOVERY et le moment où le status SPI reflète `running_mask = 0`, il peut y avoir 1–3 cycles SPI (le polling est à 5–15 ms). Dans les cas limites, le backoff démarre avant que le firmware ait fini son flush interne.

---

## Modifications à apporter

### MOD-1 — ESP32 : corriger l'anti-rebond ISR (CRITIQUE)

**Fichier :** `esp32/src/motion/stepper_driver.cpp`

Localiser la fonction `endstopIsrHandler()`. Remplacer le bloc `INVALID` et déplacer le reset du compteur :

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
        // ── MODIFICATION : NE PAS réinitialiser endstop_closed_confirmations_ ──
        // L'état INVALID est un transitoire hardware entre deux fronts NO/NC.
        // Remettre le compteur à zéro ici empêche la détection du vrai hit.
        // On enregistre juste le timestamp du transitoire pour diagnostic.
        const TickType_t invalid_since =
            drv->endstop_invalid_since_tick_.load(std::memory_order_relaxed);
        if (invalid_since == 0) {
            drv->endstop_invalid_since_tick_.store(now_tick, std::memory_order_release);
        }
        return;  // sortir sans toucher au compteur ni à last_stable_state
    }

    // État stable (OPEN ou CLOSED) : mettre à jour last_stable et réinitialiser le timer INVALID
    drv->endstop_last_stable_state_.store(static_cast<uint8_t>(raw), std::memory_order_release);
    drv->endstop_invalid_since_tick_.store(0, std::memory_order_release);

    if (raw == EndstopSignalState::OPEN) {
        // ── MODIFICATION : reset du compteur UNIQUEMENT sur OPEN confirmé ──
        drv->endstop_closed_confirmations_ = 0;
        drv->endstop_active_.store(false, std::memory_order_release);
        drv->endstop_clearance_pending_.store(false, std::memory_order_release);
        return;
    }

    // État CLOSED : logique de confirmation inchangée
    if (!drv->isEndstopArmed()) {
        return;
    }

    const uint8_t confirmations = drv->endstop_closed_confirmations_;
    if (confirmations < ENDSTOP_CLOSED_CONFIRM_COUNT) {
        drv->endstop_closed_confirmations_ = static_cast<uint8_t>(confirmations + 1);
    }
    if (drv->endstop_closed_confirmations_ < ENDSTOP_CLOSED_CONFIRM_COUNT) {
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

---

### MOD-2 — ESP32 : augmenter `ENDSTOP_CLOSED_CONFIRM_COUNT` à 3

**Fichier :** `esp32/src/motion/stepper_driver.h`

```cpp
// Avant :
static constexpr uint8_t ENDSTOP_CLOSED_CONFIRM_COUNT = 2;

// Après :
static constexpr uint8_t ENDSTOP_CLOSED_CONFIRM_COUNT = 3;
```

**Justification :** avec un capteur NO/NC, un vrai actionnement produit la séquence : front OPEN→CLOSED sur NO, puis front CLOSED→OPEN sur NC (ou l'inverse selon le câblage). Cela génère au minimum 2 fronts valides sur des pins différentes, soit 2 appels ISR avec état CLOSED. Passer à 3 confirmations ajoute une marge pour absorber un rebond sans retarder la détection d'un vrai hit (les rebonds mécaniques sur un endstop standard durent plusieurs ms, donc plusieurs ISR supplémentaires arrivent rapidement).

---

### MOD-3 — Python : corriger `backoff_steps` selon la mécanique réelle

**Calcul pour votre configuration (M6, 2048 steps/mm) :**

| Distance | Steps nécessaires |
|----------|-------------------|
| 2 mm (minimum) | 4096 |
| **3 mm (recommandé)** | **6144** |
| 4 mm (conservateur) | 8192 |

**Fichier :** `rpi/core/config.py`

```python
@dataclass
class AppConfiguration:
    # ...
    # Avant :
    # lateral_homing_backoff_steps: Optional[int] = None  (défaut = 2 tours = 4096)

    # Modifier la valeur par défaut si le champ existe,
    # ou forcer la valeur dans votre fichier de configuration JSON :
    lateral_homing_backoff_steps: Optional[int] = 6144   # 3 mm sur M6 avec 2048 steps/mm
```

Si vous passez la valeur via JSON ou via l'appel RPC `winding.home_lateral`, utiliser :

```json
{
  "backoff_steps": 6144
}
```

**Note :** vérifier aussi `_DEFAULT_HOME_BACKOFF_STEPS` dans `engine.py` :

```python
# Fichier : rpi/core/engine.py
# Avant :
_DEFAULT_HOME_BACKOFF_STEPS = 3200

# Après :
_DEFAULT_HOME_BACKOFF_STEPS = 6144  # 3 mm sur M6 avec 2048 steps/mm
```

Et dans `move.py`, `HomingMove._make_backoff_move()` calcule sa durée à partir de `backoff_steps` et `backoff_rpm` — aucun changement de logique nécessaire, la valeur correcte sera propagée automatiquement.

---

### MOD-4 — Python : augmenter `recovery_guard_s`

**Fichier :** `rpi/motion/move_queue.py`

```python
def _wait_for_post_hit_recovery(
    self,
    axis_id: int,
    *,
    stop_timeout_s: float = 1.0,
    recovery_guard_s: float = 0.150,   # ← 80 ms → 150 ms
) -> Any:
```

---

### MOD-5 — Python : augmenter `_ENDSTOP_RELEASE_TIMEOUT_S`

**Fichier :** `rpi/motion/move_queue.py`

```python
# Avant :
_ENDSTOP_RELEASE_TIMEOUT_S = 1.5

# Après (pour laisser le temps au backoff de 3 mm de se terminer) :
_ENDSTOP_RELEASE_TIMEOUT_S = 3.0
```

**Justification :** à `backoff_rpm` calculé automatiquement dans `_make_backoff_move()` — `max(search_rpm, approach_rpm × 0.5)` — avec `approach_rpm = 15 RPM` et `search_rpm = 5 RPM` (valeurs typiques), le backoff tourne à **7,5 RPM**. Le temps de parcours de 3 mm (6144 steps) à 7,5 RPM sur M6 est :

```
durée = (6144 steps / 2048 steps/mm) / (7,5 RPM × 1 mm/rev / 60)
      = 3 mm / 0,125 mm/s = 24 secondes
```

La constante `_ENDSTOP_RELEASE_TIMEOUT_S = 1.5 s` est donc **massivement** insuffisante pour cette mécanique. Heureusement, `_execute_homing()` utilise `_compute_backoff_timeout(sub_move)` qui calcule un timeout dynamique basé sur la durée réelle — cette constante n'est qu'un fallback.

> **Vérification :** dans `_execute_homing()`, la ligne qui attend la libération de l'endstop est :
> ```python
> self._wait_for_endstop_open(
>     move.axis_id,
>     timeout_s=self._compute_backoff_timeout(sub_move),  # ← dynamique ✓
> )
> ```
> `_compute_backoff_timeout()` calcule bien un timeout basé sur la durée réelle du mouvement. La constante `_ENDSTOP_RELEASE_TIMEOUT_S` n'est utilisée que comme fallback si le calcul échoue — la MOD-5 n'est donc qu'une précaution.

---

## Résumé des modifications par priorité

| Priorité | Fichier | Modification | Impact |
|----------|---------|--------------|--------|
| 🔴 CRITIQUE | `stepper_driver.cpp` | Ne pas reset `endstop_closed_confirmations_` sur INVALID | Corrige l'overtravel principal |
| 🔴 CRITIQUE | `stepper_driver.h` | `ENDSTOP_CLOSED_CONFIRM_COUNT = 3` | Élimine les faux positifs sur rebonds |
| 🟠 IMPORTANT | `engine.py` + `config.py` | `backoff_steps = 6144` (3 mm) | Garantit la libération de l'endstop |
| 🟡 UTILE | `move_queue.py` | `recovery_guard_s = 0.150` | Réduit les timeouts intermittents |
| 🟡 UTILE | `move_queue.py` | `_ENDSTOP_RELEASE_TIMEOUT_S = 3.0` | Fallback timeout plus robuste |

---

## Validation après modification

Après avoir appliqué les modifications, vérifier dans les logs ESP32 :

1. **Absence de `endstop_closed_confirmations_` reset intempestif** : l'endstop doit se latcher en 1–3 appels ISR maximum.
2. **`endstop_hit_count` = 1** après chaque phase approach/search (pas de double hit).
3. **`lateral_endstop_state` = `PRESENT_CLOSED` (0x01)** immédiatement après le contact, sans délai de plusieurs ms.
4. Sur le côté Python, vérifier que `_wait_for_endstop_open()` se résout bien avant l'expiration du timeout lors du backoff.

Pour activer les logs de diagnostic niveau DEBUG sur l'ESP32 :

```cpp
// Ajouter temporairement dans endstopIsrHandler() pour diagnostic
// (attention : ESP_LOG depuis ISR nécessite ESP_EARLY_LOGE ou log dans une tâche)
```

Le diagnostic le plus fiable reste de surveiller `lateral_endstop_state` en continu via le script `diag/read_last_executed.py` pendant un homing.