#pragma once
#include <Arduino.h>
#include "Config.h"

class SpeedInput {
public:
    void     begin();
    uint32_t readHz();  // Retourne la vitesse en Hz avec filtre moyenne glissante

private:
    uint32_t _samples[POT_FILTER_SIZE] = {};
    uint8_t  _idx = 0;
};
