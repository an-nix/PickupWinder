/* pot.cpp — Potentiometer ADC driver implementation.
 *
 * Uses the IDF legacy ADC1 API (driver/adc.h).
 * A 32-sample ring-buffer moving average reduces quantisation noise.
 */

#include "pot.h"
#include <esp_log.h>

static const char* TAG = "pot";

// ADC1 channel 0 = GPIO 36 (VP, input-only).
static constexpr adc1_channel_t  POT_CHANNEL = ADC1_CHANNEL_0;
static constexpr adc_atten_t     POT_ATTEN   = ADC_ATTEN_DB_12;  // ~0–3.9 V (was DB_11, same value)
static constexpr adc_bits_width_t POT_WIDTH  = ADC_WIDTH_BIT_12;  // 0–4095

static uint32_t s_samples[POT_FILTER_SIZE] = {};
static uint8_t  s_idx         = 0;
static bool     s_initialized = false;

void pot_init() {
    adc1_config_width(POT_WIDTH);
    adc1_config_channel_atten(POT_CHANNEL, POT_ATTEN);

    // Pre-fill the filter buffer with the current ADC reading to avoid
    // a startup ramp-up transient.
    int seed = adc1_get_raw(POT_CHANNEL);
    for (size_t i = 0; i < POT_FILTER_SIZE; ++i) {
        s_samples[i] = static_cast<uint32_t>(seed);
    }
    s_idx         = 0;
    s_initialized = true;

    ESP_LOGI(TAG, "Pot: ADC1_CH0 (GPIO 36), 11 dB atten, 12-bit, %u-sample MA filter",
             POT_FILTER_SIZE);
}

uint16_t pot_read() {
    if (!s_initialized) return 0;

    // Insert new sample into ring buffer.
    s_samples[s_idx] = static_cast<uint32_t>(adc1_get_raw(POT_CHANNEL));
    s_idx = (s_idx + 1) % POT_FILTER_SIZE;

    // Compute moving average.
    uint32_t sum = 0;
    for (size_t i = 0; i < POT_FILTER_SIZE; ++i) {
        sum += s_samples[i];
    }
    return static_cast<uint16_t>(sum / POT_FILTER_SIZE);
}
