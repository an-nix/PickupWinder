#pragma once

// ── Stepper bobine (axe principal) ───────────────────────
#define STEP_PIN            26
#define DIR_PIN             27
#define ENABLE_PIN          14      // LOW = driver actif
#define WINDING_MOTOR_INVERTED  true   // true = brochage moteur bobinage inversé (inverse DIR)

// ── Stepper latéral (guide fil) ──────────────────────────
#define STEP_PIN_LAT        32
#define DIR_PIN_LAT         33
#define ENABLE_PIN_LAT      25      // LOW = driver actif

#define LED_PIN             2       // LED guide aller-retour — déplacée sur GPIO 2 (LED onboard)
                                    // GPIO 23 libéré pour l'encodeur

// ── Encodeur rotatif ─────────────────────────────────────
// Interrupt-capable, pull-up interne dispo (INPUT_PULLUP).
// Déplacé sur 18/19 — 22/23 utilisés par le capteur de position latérale.
#define ENC1_CLK            19      // Signal A
#define ENC1_DT             18      // Signal B
// Durée minimale (µs) entre deux fronts d'encodeur acceptés dans l'ISR.
// Filtre le bruit EMI généré par les impulsions step du moteur ;
// les transitions humaines légitimes sont espacées de ≥ 3 000 µs même
// en tournant vite → 1 000 µs est un choix confortable.
#define ENC_DEBOUNCE_US     1000// Déplacement de la butée par cran d'encodeur (mm) pendant la phase de vérification.
#define ENC_STEP_MM         0.05f
// Multiplicateur de pas appliqué au premier passage en mode MANUAL (avant qu'une bute soit atteinte).
// Permet de traverser rapidement la fenêtre en début de bobinage.
#define MANUAL_FAST_STEP_MULT  10
// ── Capteur position initiale axe latéral ────────────────
// Le capteur est connecté à la masse, entrées en INPUT_PULLUP.
// Contact fermé (pin à GND) = LOW. Contact ouvert (pull-up) = HIGH.
//   Hors home : NO ouvert (HIGH)  | NC fermé (LOW)
//   En home   : NO fermé (LOW)   | NC ouvert (HIGH)
//   Défaut    : NO et NC identiques (les deux HIGH = capteur absent)
#define HOME_PIN_NO         23      // Normally Open  — LOW quand en home
#define HOME_PIN_NC         22      // Normally Closed — HIGH quand en home

// Paramètres du homing de l'axe latéral
// true = runForward() va vers le home, false = runBackward(). À ajuster selon le câblage.
#define LAT_HOME_DIR        false   // false = runBackward() vers le capteur (gauche)
#define LAT_ACCEL           40000   // steps/s² — accélération axe latéral
#define LAT_HOME_SPEED_HZ   4800    // Vitesse de homing (≈45 RPM à 1/32)
// Offset entre le hard-stop physique (capteur) et la position 0 réelle.
// Valeur par défaut à la compilation — peut être modifié depuis l'interface web
// et stocké en mémoire NVS (sans recompilation).
#define LAT_HOME_OFFSET_DEFAULT_MM  15.0f // mm — 15 mm entre le hard-stop et la position 0 réelle
// Cinématique axe latéral
// Moteur 96 pas, 1/32 microstep, tige M6 (1 mm/tr)
// steps/mm = 96 × 32 / 1 = 3072
#define LAT_MOTOR_STEPS     96      // Pas/tr du moteur latéral
#define LAT_STEPS_PER_MM    (LAT_MOTOR_STEPS * MICROSTEPPING)  // 3072 steps/mm
#define LAT_TRAVERSE_MM     100     // 10 cm — vérification calibration
#define LAT_TRAVERSE_SPEED_HZ 4800  // Vitesse de traversée ≈ 0.75 mm/s (M6 1/32)
// Vitesse fallback pour la phase de positionnement final (endPos) si le
// moteur ne tourne pas encore — ≈ 1/6 de la vitesse normale.
#define LAT_ENDPOS_FALLBACK_HZ  (LAT_TRAVERSE_SPEED_HZ / 6)
// Vitesse pendant le rodage : plus rapide que la traversée normale.
// 12 000 Hz ≈ 3.9 mm/s (2.5× la traversée), confortable pour le rodage mécanique.
#define LAT_RODAGE_SPEED_HZ  12000
// Facteur de ralentissement du moteur de bobinage pendant le demi-tour latéral.
// La durée effective est calculée dynamiquement : 2 × (v_lat / LAT_ACCEL) secondes.
// Réduire si les extrémités s'accumulent ; augmenter si les variations de vitesse gênent.
#define LAT_REVERSAL_SLOWDOWN  0.5f  // 0 < x ≤ 1.0 — vitesse bobinage × ce facteur au demi-tour

// Choisir UNE option selon le protocole retenu :
//
// Option A — UART2  (simple, 2 fils, longue distance)
//   TX → GPIO 17  |  RX → GPIO 16
//   Activer dans platformio.ini : build_flags = -Dil COMM_UART
//
// Option B — SPI esclave  (rapide, 4 fils)
//   MOSI → GPIO 13  |  MISO → GPIO 15
//   CLK  → GPIO 18  |  CS   → GPIO 19
//   Activer dans platformio.ini : build_flags = -DCOMM_SPI
//
// GPIO 21/22 réservés I2C SDA/SCL — ne pas utiliser ici.

// ── UART2 liaison série ESP ↔ ESP ────────────────────────
#define LINK_UART           Serial2
#define LINK_BAUD           115200
#define LINK_TX_PIN         17      // Winder TX → Écran RX
#define LINK_RX_PIN         16      // Winder RX ← Écran TX
#define LINK_UPDATE_MS      100     // Intervalle d'envoi du statut vers l'écran (ms)

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
#define POT_INVERTED        false   // true = pot wired in reverse (swap min/max)
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

#define WIFI_SSID           "meba"
#define WIFI_PASSWORD       "welcome@th0me4.0"
#define WEB_PORT            80
#define WS_UPDATE_MS        200     // Intervalle d'envoi WebSocket (ms)

// ── Bobinage ─────────────────────────────────────────────
#define DEFAULT_TARGET_TURNS  8000
#define WINDING_DEFAULT_SEED   424242UL
#define WINDING_LAYER_JITTER_DEFAULT          0.10f
#define WINDING_LAYER_SPEED_JITTER_DEFAULT    0.08f
#define WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT 0.16f
#define WINDING_HUMAN_SPEED_JITTER_DEFAULT    0.12f

// Number of turns before the target where the motor starts slowing down.
// Speed is linearly reduced from the pot value down to SPEED_HZ_MIN over
// this window so the final stop is gentle on the wire.
#define LED_FLASH_MS        400   // LED flash duration on pass change (ms)
#define APPROACH_TURNS      80    // Nombre de tours avant la cible où la décélération commence
// Vitesse plancher en zone d'approche : le moteur ne descend jamais en-dessous
// de cette valeur (même si le pot est plus haut), puis s'arrête brusquement à la cible.
#define APPROACH_SPEED_HZ_FLOOR  (200UL * STEPS_PER_REV / 60)  // 200 RPM ≈ 21333 Hz
// Vitesse max pendant la passe de vérification de la première inversion.
// Le moteur est limité à cette vitesse pour que l'arrêt soit quasi-synchrone
// avec le chariot latéral (décel rapide depuis une vitesse faible).
#define VERIFY_SPEED_HZ_MAX      (600UL * STEPS_PER_REV / 60)  // ~100 RPM
