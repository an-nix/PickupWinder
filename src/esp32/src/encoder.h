/* encoder.h — Quadrature encoder via ESP32 PCNT hardware peripheral.
 *
 * Uses the IDF legacy PCNT driver (driver/pcnt.h) for full 4X quadrature
 * decoding on PCNT_UNIT_0 with hardware glitch filtering.
 *
 * Pin assignment (UI manual encoder):
 *   A = GPIO 0  (strap pin)
 *   B = GPIO 15 (strap pin)
 *
 * Note: GPIO0 and GPIO15 are strapping/boot pins on many ESP32 modules.
 * Driving them can affect boot mode. Add proper pull resistors or ensure
 * the encoder is idle during reset/flash to avoid interfering with boot.
 *
 * PCNT hardware handles edge counting without ISR overhead.
 * The 16-bit counter wraps at ±32767.
 * Typical usage: sensor_task reads the count every 1 ms and publishes to
 * g_sensor.encoder_manual (int16_t, wrapping absolute position).
 */

#pragma once

#include <cstdint>

struct EncoderCfg {
    int pin_a;   // Encoder A (pulse input)
    int pin_b;   // Encoder B (control / quadrature)
};

/// Initialize PCNT unit for 4X quadrature decoding.
/// Must be called from app_main() before sensor_task_start().
void encoder_init(const EncoderCfg& cfg);

/// Return current 16-bit PCNT counter value (wrapping ±32767).
int16_t encoder_get_count();

/// Return delta since last call (clears internal reference).
/// Useful for incremental position tracking.
int16_t encoder_get_and_clear_delta();
