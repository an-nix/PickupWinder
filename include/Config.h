#pragma once

// ── Moteur pas à pas ─────────────────────────────────────
#define STEP_PIN            32
#define DIR_PIN             33
#define ENABLE_PIN          25      // LOW = driver actif
#define LED_PIN             23      // LED guide aller-retour (toggle à chaque passage)

#define MICROSTEPPING       32      // Microstepping configured on driver (M0/M1/M2)
#define STEPS_PER_REV       (200 * MICROSTEPPING)   // 6400 steps/rev at 1/32
#define ACCELERATION        200000  // steps/s² — equivalent to 50000 at 1/8 (~1875 RPM/s)
#define SPEED_HZ_MIN        9600    // ~90 RPM  (90 × 6400 / 60)
#define SPEED_HZ_MAX        160000  // ~1500 RPM (1500 × 6400 / 60)

// ── Potentiomètre ────────────────────────────────────────
// ⚠ ADC1 uniquement (GPIO 32-39) — ADC2 est incompatible avec le WiFi
#define POT_PIN             34
#define POT_INVERTED        true    // true = pot wired in reverse (swap min/max)
#define POT_FILTER_SIZE     16      // 16 × 20ms = ~320ms de lissage (lisse le bruit ADC)
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
