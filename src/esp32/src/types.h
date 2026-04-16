#pragma once

typedef struct {
    uint32_t duration_us; // délai avant le prochain step (µs)
    uint8_t  dir;         // 0 ou 1
    uint8_t  flags;       // bit0 = force enable, bit1 = force disable (optionnel)
} StepEvent;