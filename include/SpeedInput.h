#pragma once
#include <Arduino.h>
#include "Config.h"

// ── SpeedInput ────────────────────────────────────────────────────────────────
// Reads the speed potentiometer (ADC1 pin, 12-bit) and applies a sliding-window
// average filter to smooth out ADC noise before mapping to a Hz value.
// ADC1 must be used (GPIO 32-39) because ADC2 is disabled when WiFi is active.
class SpeedInput {
public:
    // Pre-fill the filter buffer with the current ADC value to avoid
    // a false ramp-up at startup.
    void     begin();

    // Read one ADC sample, update the circular buffer, compute the average
    // and map the result to [SPEED_HZ_MIN, SPEED_HZ_MAX].
    uint32_t readHz();

private:
    uint32_t _samples[POT_FILTER_SIZE] = {};  // Circular buffer of raw ADC readings
    uint8_t  _idx    = 0;                     // Write index into the circular buffer
    uint32_t _lastHz = 0;                     // Last output Hz (for hysteresis)
};
