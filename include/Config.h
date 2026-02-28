#pragma once

// ── Moteur pas à pas ─────────────────────────────────────
#define STEP_PIN            32
#define DIR_PIN             33
#define ENABLE_PIN          25      // LOW = driver actif
#define LED_PIN             26      // LED guide aller-retour (toggle à chaque passage)

#define MICROSTEPPING       8       // Microstepping configuré sur le driver (MS1/MS2/MS3)
#define STEPS_PER_REV       (200 * MICROSTEPPING)   // 1600 pas/tour en 1/8 step
#define ACCELERATION        120000  // pas/s²
#define SPEED_HZ_MIN        2400    // ~90 RPM  (300 Hz × 8)
#define SPEED_HZ_MAX        80000   // ~3000 RPM (10000 Hz × 8)

// ── Potentiomètre ────────────────────────────────────────
// ⚠ ADC1 uniquement (GPIO 32-39) — ADC2 est incompatible avec le WiFi
#define POT_PIN             34
#define POT_FILTER_SIZE     16      // Taille filtre moyenne glissante
#define POT_READ_INTERVAL   50      // ms entre deux lectures
#define POT_DEADZONE_HZ     300     // Zone morte basse du pot (en Hz) pour éviter les démarrages intempestifs

// ── WiFi & interface web ─────────────────────────────────
#define WIFI_SSID           "meba"
#define WIFI_PASSWORD       "welcome@th0me4.0"
#define WEB_PORT            80
#define WS_UPDATE_MS        200     // Intervalle d'envoi WebSocket (ms)

// ── Bobinage ─────────────────────────────────────────────
#define DEFAULT_TARGET_TURNS  8000
