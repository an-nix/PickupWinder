#include "SpeedInput.h"

void SpeedInput::begin() {
    // Pré-remplir le filtre pour éviter une rampe parasite au démarrage
    for (uint8_t i = 0; i < POT_FILTER_SIZE; i++) {
        _samples[i] = analogRead(POT_PIN);
    }
    Serial.println("[SpeedInput] OK");
}

uint32_t SpeedInput::readHz() {
    _samples[_idx] = analogRead(POT_PIN);
    _idx = (_idx + 1) % POT_FILTER_SIZE;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < POT_FILTER_SIZE; i++) sum += _samples[i];
    uint32_t avg = sum / POT_FILTER_SIZE;

    // Map 0–4095 (ADC 12 bits) → SPEED_HZ_MIN–SPEED_HZ_MAX
    return (uint32_t)map(avg, 0, 4095, SPEED_HZ_MIN, SPEED_HZ_MAX);
}
