#include "SpeedInput.h"

/**
 * @brief Initialize ADC smoothing history.
 */
void SpeedInput::begin() {
    // Pre-fill the entire filter buffer with the current ADC reading.
    // Without this, the average would start near zero and ramp up over
    // POT_FILTER_SIZE samples, making the motor appear to accelerate on its own.
    for (uint8_t i = 0; i < POT_FILTER_SIZE; i++) {
        _samples[i] = analogRead(POT_PIN);
    }
    Serial.println("[SpeedInput] OK");
}

/**
 * @brief Read filtered potentiometer speed command.
 * @return Requested speed in Hz (0 when in zero-band).
 */
uint32_t SpeedInput::readHz() {
    // Write the new sample into the circular buffer at the current index.
    _samples[_idx] = analogRead(POT_PIN);
    _idx = (_idx + 1) % POT_FILTER_SIZE;  // Advance index, wrapping around

    // Compute the mean of all samples in the buffer.
    uint32_t sum = 0;
    for (uint8_t i = 0; i < POT_FILTER_SIZE; i++) sum += _samples[i];
    uint32_t avg = sum / POT_FILTER_SIZE;

    // Invert ADC reading if the pot is wired in reverse.
    #if POT_INVERTED
        avg = 4095 - avg;
    #endif

    // ── Zero band ─────────────────────────────────────────────────────────
    // When the pot is physically at the low stop, return zero immediately.
    // Raw ADC counts are used here because that remains robust even when the
    // pot does not quite reach the electrical rail.
    if (avg <= POT_ADC_ZERO_BAND) {
        _lastHz = 0;
        return 0;
    }

    uint32_t hz;
    if (avg >= POT_ADC_FULL_BAND) {
        hz = SPEED_HZ_MAX;
    } else {
        // Exponential response curve: gentle progression at the bottom end and
        // faster growth near the top end, which gives finer low-speed control.
        // Formula: hz = SPEED_HZ_MAX * (e^(k*t) - 1) / (e^k - 1)
        //   t = 0 → hz = 0   |   t = 1 → hz = SPEED_HZ_MAX
        // With k=4.5: about 50 RPM at 30% travel, 375 RPM at 70%, and 1500 RPM at 100%.
        float t = (float)(avg - POT_ADC_ZERO_BAND)
                / (float)(POT_ADC_FULL_BAND - POT_ADC_ZERO_BAND);
        t = constrain(t, 0.0f, 1.0f);
        // The denominator is constant for a fixed k and can be cached once.
        static const float invDenom = 1.0f / (expf(POT_EXP_K) - 1.0f);
        hz = (uint32_t)((expf(POT_EXP_K * t) - 1.0f) * invDenom * (float)SPEED_HZ_MAX);
    }

    // Hysteresis: only update the output if the delta crosses the configured threshold.
    // Exception: stay responsive near zero and near the maximum command.
    uint32_t delta = (hz > _lastHz) ? (hz - _lastHz) : (_lastHz - hz);
    if (delta >= POT_HYSTERESIS_HZ
        || hz < 3000                                   // Low-speed region stays responsive.
        || hz >= SPEED_HZ_MAX - POT_HYSTERESIS_HZ) {  // High-speed region also stays responsive.
        _lastHz = hz;
    }

    return _lastHz;
}
