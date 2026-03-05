#pragma once

// ── Moteur pas à pas ─────────────────────────────────────
#define STEP_PIN            32
#define DIR_PIN             33
#define ENABLE_PIN          25      // LOW = driver actif
#define LED_PIN             23      // LED guide aller-retour (toggle à chaque passage)

#define MICROSTEPPING       32      // Microstepping configured on driver (M0/M1/M2)
#define STEPS_PER_REV       (200 * MICROSTEPPING)   // 6400 steps/rev at 1/32
// Ratio between step frequency and audible frequency produced by the motor.
// 200-step motor has 50 pole pairs → audible freq = step_freq / (MICROSTEPPING × 4).
// Verified: 42000 Hz step rate ≈ E4 (329.6 Hz) → 42000/329.6 ≈ 128.
#define MOTOR_NOTE_MULT     (MICROSTEPPING * 4)      // 128 at 1/32 microstepping
#define ACCELERATION        150000  // steps/s² — ~1400 RPM/s, réponse vive au pot (le doux démarrage est géré par SPEED_HZ_START)
#define SPEED_HZ_MIN        9600/9    // ~90 RPM  (90 × 6400 / 60)
#define SPEED_HZ_MAX        160000  // ~1500 RPM (1500 × 6400 / 60)
// Vitesse initiale de départ — le moteur commence à cette vitesse basse
// et accélère via la rampe jusqu'à la vitesse cible du pot.
// Cela évite le choc violent au démarrage (sinon le moteur partirait
// directement à SPEED_HZ_MIN même si le pot est à mi-course).
#define SPEED_HZ_START      800     // ~7.5 RPM — quasi imperceptible, rampe ensuite

// ── Potentiomètre ────────────────────────────────────────
// ⚠ ADC1 uniquement (GPIO 32-39) — ADC2 est incompatible avec le WiFi
#define POT_PIN             34
#define POT_INVERTED        true    // true = pot wired in reverse (swap min/max)
#define POT_FILTER_SIZE     32      // 32 × 20ms = ~640ms de lissage (lisse le bruit ADC et les à-coups de vitesse)
#define POT_READ_INTERVAL   20      // ms entre deux lectures
#define POT_HYSTERESIS_HZ   100    // La vitesse ne change que si l'écart dépasse ce seuil
// Détection zéro sur counts ADC bruts (après inversion).
// Augmenter si le moteur ne s'arrête pas quand le pot est en butée basse.
#define POT_ADC_ZERO_BAND   150    // ≈ 11° sur un pot 300° (150/4095 ≈ 3.7%)
// Courbe exponentielle sur toute la course du pot.
// Formule : hz = SPEED_HZ_MAX × (e^(k·t) − 1) / (e^k − 1)  avec t ∈ [0..1]
// Plus k est grand, plus la progression est lente au début et rapide en fin de course.
// k=4.5 → ~50 RPM à 30% de la course, ~375 RPM à 70%, 1500 RPM à 100%.
// Augmenter k pour plus d'exponentialité (ex. 6.0), diminuer pour moins (ex. 3.0).
#define POT_EXP_K           4.5f

// ── WiFi & interface web ─────────────────────────────────
#define WIFI_SSID           "meba"
#define WIFI_PASSWORD       "welcome@th0me4.0"
#define WEB_PORT            80
#define WS_UPDATE_MS        200     // Intervalle d'envoi WebSocket (ms)

// ── Bobinage ─────────────────────────────────────────────
#define DEFAULT_TARGET_TURNS  8000

// Number of turns before the target where the motor starts slowing down.
// Speed is linearly reduced from the pot value down to SPEED_HZ_MIN over
// this window so the final stop is gentle on the wire.
#define LED_FLASH_MS        400   // LED flash duration on pass change (ms)
#define APPROACH_TURNS      80    // Nombre de tours avant la cible où la décélération commence
// Vitesse plancher en zone d'approche : le moteur ne descend jamais en-dessous
// de cette valeur (même si le pot est plus haut), puis s'arrête brusquement à la cible.
#define APPROACH_SPEED_HZ_FLOOR  (200UL * STEPS_PER_REV / 60)  // 200 RPM ≈ 21333 Hz
