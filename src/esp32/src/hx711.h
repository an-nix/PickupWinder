/* hx711.h — HX711 24-bit ADC driver.
 *
 * Non-blocking poll model: hx711_tick() is called from sensor_task
 * every 1 ms and returns immediately if no new conversion is ready.
 * When DOUT goes LOW (~80 Hz at RATE=HIGH), the bitbang read takes ~50 µs.
 *
 * PID control is NOT done on the ESP32.  Readings are forwarded to the
 * Raspberry Pi via the StatusFrame; the Pi runs PID and sends setpoint
 * commands back as SET_SPEED on the tensioner axis.
 *
 * Two sensors:
 *   [0] — Wire tension load cell: reading forwarded to RPi.
 *   [1] — Auxiliary load cell:    reading forwarded to RPi.
 *
 * HX711 wiring:
 *   SCK  → any output GPIO
 *   DOUT → any input GPIO (GPIO 34–39 recommended: input-only, 5 V tolerant)
 *
 * Output units: 0.1 g (decigrams).  Range: ±3276.7 g in int16_t.
 */

#pragma once

#include <cstdint>

static constexpr uint8_t NUM_HX711 = 2;

// ── Per-sensor configuration ──────────────────────────────────────────────────
struct Hx711Cfg {
    int      sck_pin;   // SCK output GPIO
    int      dout_pin;  // DOUT input GPIO (GPIO 34–39 recommended)
    int32_t  zero_raw;  // Tare offset (raw counts at zero load)
    float    scale;     // Raw counts per gram (calibration factor)
};

// ── Shared state (written by sensor_task Core 0, read by stepper_task Core 1) ─
// Protected by g_sensor.mux in sensor_task; readings are copied to g_sensor
// under the spinlock.  Fields are volatile for visibility across cores.
struct Hx711State {
    volatile int16_t reading_dg[NUM_HX711];  // Converted readings in 0.1 g
    volatile bool    fresh[NUM_HX711];        // true = new data since last tick
};

extern Hx711State g_hx711;

// ── API ───────────────────────────────────────────────────────────────────────

/// Initialize HX711 GPIO pins.  Must be called from sensor_task_init().
void hx711_init(const Hx711Cfg cfgs[NUM_HX711]);

/// Non-blocking tick — call from sensor_task every 1 ms.
/// Returns true when at least one sensor produced a fresh reading.
bool hx711_tick();

/// Tare (zero) a sensor.  Blocking (~100 ms, averages 8 readings).
/// Call only from sensor_task when the machine is idle.
void hx711_tare(uint8_t sensor_id);
