#include "SpeedInput.h"

void SpeedInput::begin() {
    // Pre-fill the entire filter buffer with the current ADC reading.
    // Without this, the average would start near zero and ramp up over
    // POT_FILTER_SIZE samples, making the motor appear to accelerate on its own.
    for (uint8_t i = 0; i < POT_FILTER_SIZE; i++) {
        _samples[i] = analogRead(POT_PIN);
    }
    Serial.println("[SpeedInput] OK");
}

uint32_t SpeedInput::readHz() {
    // Write the new sample into the circular buffer at the current index.
    _samples[_idx] = analogRead(POT_PIN);
    _idx = (_idx + 1) % POT_FILTER_SIZE;  // Advance index, wrapping around

    // Compute the mean of all samples in the buffer.
    uint32_t sum = 0;
    for (uint8_t i = 0; i < POT_FILTER_SIZE; i++) sum += _samples[i];
    uint32_t avg = sum / POT_FILTER_SIZE;

    // Map the 12-bit ADC range (0–4095) to the configured speed range in Hz.
    // Arduino's map() performs integer linear interpolation.
    return (uint32_t)map(avg, 0, 4095, SPEED_HZ_MIN, SPEED_HZ_MAX);
}
