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
#define SPEED_HZ_MIN        9600    // ~90 RPM  (90 × 6400 / 60)
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
#define POT_HYSTERESIS_HZ   200     // La vitesse ne change que si l'écart dépasse ce seuil
#define POT_DEADZONE_HZ     300     // Zone morte démarrage : pot doit dépasser ce seuil (en Hz)
#define POT_DEADZONE_STOP   150     // Zone morte arrêt : hystérésis (seuil inférieur)

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
#define APPROACH_TURNS        150
