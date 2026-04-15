/* pot.h — Potentiometer ADC driver (ADC1_CHANNEL_0 = GPIO 36).
 *
 * GPIO 36 (VP, input-only) is the only free ADC1 pin in the current design.
 * The old implementation used GPIO 34 (now occupied by HX711[0] DOUT).
 *
 * Configuration:
 *   Channel:     ADC1_CHANNEL_0 (GPIO 36, VP, input-only, no pull-up)
 *   Attenuation: ADC_ATTEN_DB_11 (approx. 0–3.9 V full range)
 *   Resolution:  12-bit (0–4095)
 *   Filter:      32-sample moving average for smooth readings
 *   Read rate:   ~50 Hz (20 ms period, called from sensor_task)
 *
 * ADC2 is avoided because it conflicts with Wi-Fi on ESP32.
 * All ADC1 channels other than GPIO 36 are occupied in this design.
 */

#pragma once

#include <cstdint>
#include <driver/adc.h>

static constexpr size_t POT_FILTER_SIZE = 32;

/// Initialize ADC1 channel and pre-fill the filter buffer.
/// Must be called from app_main() before sensor_task_start().
void pot_init();

/// Acquire one ADC sample, update moving average, return smoothed 12-bit value.
/// Call from sensor_task at 20 ms intervals.
uint16_t pot_read();
