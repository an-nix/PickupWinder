#pragma once

// ── Moteur pas à pas ─────────────────────────────────────
#define STEP_PIN            32
#define DIR_PIN             33
#define ENABLE_PIN          25      // LOW = driver actif
#define LED_PIN             23      // LED guide aller-retour (toggle à chaque passage)

#define MICROSTEPPING       32      // Microstepping configuré sur le driver (M0/M1/M2)
#define STEPS_PER_REV       (200 * MICROSTEPPING)   // 6400 pas/tour en 1/32 step
#define ACCELERATION        50000   // pas/s²
#define SPEED_HZ_MIN        2400    // ~90 RPM  (90 × 1600 / 60)
#define SPEED_HZ_MAX        40000   // ~1500 RPM (1500 × 1600 / 60)

// ── Potentiomètre ────────────────────────────────────────
// ⚠ ADC1 uniquement (GPIO 32-39) — ADC2 est incompatible avec le WiFi
#define POT_PIN             34
#define POT_FILTER_SIZE     6       // Filtre léger : 6 × 20ms = ~60ms de lissage
#define POT_READ_INTERVAL   20      // ms entre deux lectures
#define POT_DEADZONE_HZ     300     // Zone morte démarrage : pot doit dépasser ce seuil (en Hz)
#define POT_DEADZONE_STOP   150     // Zone morte arrêt : hystérésis (seuil inférieur)

// ── WiFi & interface web ─────────────────────────────────
#define WIFI_SSID "<redacted>"
#define WIFI_PASSWORD "<redacted>"
#define WEB_PORT            80
#define WS_UPDATE_MS        200     // Intervalle d'envoi WebSocket (ms)

// ── Bobinage ─────────────────────────────────────────────
#define DEFAULT_TARGET_TURNS  8000
