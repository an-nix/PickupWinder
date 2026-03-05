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

    // Invert ADC reading if the pot is wired in reverse.
    #if POT_INVERTED
        avg = 4095 - avg;
    #endif

    // ── Zone zéro ────────────────────────────────────────────────────────
    // Pot en butée physique basse : retourner 0 directement.
    // Détection sur counts ADC bruts (robuste même si le pot ne touche pas
    // exactement le rail électrique en butée).
    if (avg <= POT_ADC_ZERO_BAND) {
        _lastHz = 0;
        return 0;
    }

    uint32_t hz;
    {
        // Courbe exponentielle : progression lente en début de course,
        // rapide en fin — idéal pour contrôler finement les basses vitesses.
        // Formule : hz = SPEED_HZ_MAX × (e^(k·t) − 1) / (e^k − 1)
        //   t = 0 → hz = 0   |   t = 1 → hz = SPEED_HZ_MAX
        // Avec k=4.5 : ~50 RPM à 30% de course, ~375 RPM à 70%, 1500 RPM à 100%.
        float t = (float)(avg - POT_ADC_ZERO_BAND)
                / (float)(4095 - POT_ADC_ZERO_BAND);
        // Le dénominateur est constant (k fixe) : calculé une seule fois.
        static const float invDenom = 1.0f / (expf(POT_EXP_K) - 1.0f);
        hz = (uint32_t)((expf(POT_EXP_K * t) - 1.0f) * invDenom * (float)SPEED_HZ_MAX);
    }

    // Hystérésis : ne mettre à jour _lastHz que si la variation dépasse le seuil.
    // Exception : toujours réactif en zone basse et proche du maximum.
    uint32_t delta = (hz > _lastHz) ? (hz - _lastHz) : (_lastHz - hz);
    if (delta >= POT_HYSTERESIS_HZ
        || hz < 3000                                   // zone basse : toujours réactif
        || hz >= SPEED_HZ_MAX - POT_HYSTERESIS_HZ) {  // proche du max : toujours réactif
        _lastHz = hz;
    }

    return _lastHz;
}
