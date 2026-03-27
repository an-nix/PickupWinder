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
