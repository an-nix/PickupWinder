#pragma once

// ── Main spindle stepper ──────────────────────────────────
#define STEP_PIN            26
#define DIR_PIN             27
#define ENABLE_PIN          14      // LOW = driver enabled
#define WINDING_MOTOR_INVERTED  true   // true when motor wiring requires inverted direction logic

// ── Lateral carriage stepper ─────────────────────────────
#define STEP_PIN_LAT        32
#define DIR_PIN_LAT         33
#define ENABLE_PIN_LAT      25      // LOW = driver enabled

#define LED_PIN             2       // Pass indicator LED, mapped to the on-board LED on GPIO 2

// ── Rotary encoder ───────────────────────────────────────
// Both pins support interrupts and internal pull-ups.
// GPIO 18/19 are dedicated to the encoder; 22/23 are used by the lateral home sensor.
#define ENC1_CLK            19      // Encoder signal A
#define ENC1_DT             18      // Encoder signal B
// Minimum time between accepted encoder edges inside the ISR.
// This rejects EMI created by step pulses while remaining comfortably below
// legitimate human rotation timing.
#define ENC_DEBOUNCE_US     1000
// Encoder trim step used during verification and paused-bound adjustment.
#define ENC_STEP_MM         0.05f

// ── Lateral axis home sensor ─────────────────────────────
// The sensor is wired to ground and read with INPUT_PULLUP.
// Closed contact to GND = LOW. Open contact with pull-up = HIGH.
//   Away from home: NO open (HIGH), NC closed (LOW)
//   At home:        NO closed (LOW), NC open (HIGH)
//   Fault:          NO and NC identical, usually both HIGH when disconnected
#define HOME_PIN_NO         23      // Normally-open contact, LOW when at home
#define HOME_PIN_NC         22      // Normally-closed contact, HIGH when at home

// Lateral-axis homing parameters.
// true means runForward() moves toward home; false means runBackward().
#define LAT_HOME_DIR        false   // false means home lies in the backward direction
#define LAT_ACCEL           40000   // steps/s² for the lateral axis
#define LAT_HOME_SPEED_HZ   4800    // Homing speed (~45 RPM at 1/32 microstepping)
// Distance between the physical sensor trip point and the logical zero position.
#define LAT_HOME_OFFSET_DEFAULT_MM  15.0f // mm from hard-stop trigger to logical zero
// Lateral axis kinematics:
// 96-step motor, 1/32 microstepping, M6 lead screw with 1 mm pitch.
#define LAT_MOTOR_STEPS     96      // Full steps per revolution on the lateral motor
#define LAT_STEPS_PER_MM    (LAT_MOTOR_STEPS * MICROSTEPPING)  // 3072 steps/mm
#define LAT_TRAVERSE_MM     100     // 100 mm traversal used for calibration and checks
#define LAT_TRAVERSE_SPEED_HZ 4800  // Traverse speed ~= 0.75 mm/s with the current mechanics
// Fallback speed for the final end-position hold move if the carriage is idle.
#define LAT_ENDPOS_FALLBACK_HZ  (LAT_TRAVERSE_SPEED_HZ / 6)
// Break-in shuttle speed, intentionally faster than normal traversal.
#define LAT_RODAGE_SPEED_HZ  12000
// Spindle slowdown factor applied during lateral reversals.
// Effective duration is computed dynamically from lateral speed and acceleration.
#define LAT_REVERSAL_SLOWDOWN  0.5f  // Spindle speed multiplier while reversing

// Select ONE transport option if the project ever needs an alternate external link:
//
// Option A - UART2 (simple two-wire link, good over distance)
//   TX -> GPIO 17  |  RX -> GPIO 16
//
// Option B - SPI slave (faster, four-wire link)
//   MOSI -> GPIO 13  |  MISO -> GPIO 15
//   CLK  -> GPIO 18  |  CS   -> GPIO 19
//
// GPIO 21/22 are commonly reserved for I2C and should be avoided here.

// ── UART2 ESP-to-ESP link ────────────────────────────────
#define LINK_UART           Serial2
#define LINK_BAUD           115200
#define LINK_TX_PIN         17      // Winder TX -> display RX
#define LINK_RX_PIN         16      // Winder RX <- display TX
#define LINK_UPDATE_MS      100     // Status transmission interval to the display, in ms

#define MICROSTEPPING       32      // Microstepping configured on driver (M0/M1/M2)
#define STEPS_PER_REV       (200 * MICROSTEPPING)   // 6400 steps/rev at 1/32
// Ratio between step frequency and audible frequency produced by the motor.
// 200-step motor has 50 pole pairs → audible freq = step_freq / (MICROSTEPPING × 4).
// Verified: 42000 Hz step rate ≈ E4 (329.6 Hz) → 42000/329.6 ≈ 128.
#define MOTOR_NOTE_MULT     (MICROSTEPPING * 4)      // 128 at 1/32 microstepping
#define ACCELERATION        150000  // steps/s², tuned for responsive spindle acceleration
#define SPEED_HZ_MIN        9600/9    // ~90 RPM  (90 × 6400 / 60)
#define SPEED_HZ_MAX        160000  // ~1500 RPM (1500 × 6400 / 60)
// Historical soft-start speed kept as a reference tuning constant.
#define SPEED_HZ_START      800     // ~7.5 RPM

// ── Potentiometer ────────────────────────────────────────
// Use ADC1 only (GPIO 32-39); ADC2 conflicts with Wi-Fi on ESP32.
#define POT_PIN             34
#define POT_INVERTED        false   // true = pot wired in reverse (swap min/max)
#define POT_FILTER_SIZE     32      // 32 samples at 20 ms -> ~640 ms smoothing window
#define POT_READ_INTERVAL   20      // Time between ADC reads, in ms
#define POT_HYSTERESIS_HZ   100     // Speed output changes only past this delta
// Raw ADC zero band after inversion.
#define POT_ADC_ZERO_BAND   150     // ~= 3.7% of full-scale ADC range
// Raw ADC full-scale clamp for imperfect pots that never reach 4095.
#define POT_ADC_FULL_BAND   3700
// Exponential transfer curve over the full pot travel.
// hz = SPEED_HZ_MAX * (exp(k*t) - 1) / (exp(k) - 1), with t in [0..1].
#define POT_EXP_K           4.5f

// ── Footswitch ───────────────────────────────────────────
// Recommended wiring: normally-open footswitch between GPIO 13 and GND.
// Change the pin here if you prefer a different GPIO.
//
// The macro `FOOTSWITCH_ACTIVE_LOW` selects the electrical polarity the
// firmware expects at the sampled GPIO level:
//  - true  => pressed = LOW (use INPUT_PULLUP, common for NO to GND wiring)
//  - false => pressed = HIGH (use INPUT_PULLUP but interpret logic inverted)
// This lets you compile for either NO (normally-open) or NC (normally-closed)
// wiring without changing code elsewhere.
#define FOOTSWITCH_PIN      13
// Invert footswitch polarity to match wiring: your switch is CLOSED when
// released and OPEN when pressed (NC wiring). Set to `false` so `pressed` maps
// to HIGH when using `INPUT_PULLUP` sampling logic.
#define FOOTSWITCH_ACTIVE_LOW false
// Debounce time for mechanical contact stabilization.
#define FOOTSWITCH_DEBOUNCE_MS 20

#define WIFI_SSID           "meba"
#define WIFI_PASSWORD       "welcome@th0me4.0"
#define WEB_PORT            80
#define WS_UPDATE_MS        200     // WebSocket status update period in ms

// ── Winding defaults ─────────────────────────────────────
#define DEFAULT_TARGET_TURNS  8000
#define DEFAULT_REWIND_BATCH_TURNS 200
#define DEFAULT_REWIND_BATCH_RPM   350
#define WINDING_DEFAULT_SEED   424242UL
#define WINDING_LAYER_JITTER_DEFAULT          0.10f
#define WINDING_LAYER_SPEED_JITTER_DEFAULT    0.08f
#define WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT 0.16f
#define WINDING_HUMAN_SPEED_JITTER_DEFAULT    0.12f

// Number of turns before the target where the motor starts slowing down.
// Speed is linearly reduced from the pot value down to SPEED_HZ_MIN over
// this window so the final stop is gentle on the wire.
#define LED_FLASH_MS        400   // LED flash duration on pass change (ms)
#define APPROACH_TURNS      80    // Number of turns before target where slowdown begins
// Minimum speed allowed in the target approach zone.
#define APPROACH_SPEED_HZ_FLOOR  (200UL * STEPS_PER_REV / 60)  // 200 RPM ≈ 21333 Hz
// Maximum spindle speed during the first verification pass near the reversal.
#define VERIFY_SPEED_HZ_MAX      (600UL * STEPS_PER_REV / 60)  // ~100 RPM
