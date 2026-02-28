#pragma once
#include <Arduino.h>
#include "Config.h"

class LEDController {
public:
    void begin();

    // À appeler à chaque tick du loop.
    // Toggle la LED à chaque nouveau passage (aller ou retour).
    // Retourne le numéro de passage courant.
    int  update(long currentTurns, long turnsPerPass, bool motorRunning);

    void reset();
    int  getCurrentPass() const { return _currentPass; }

private:
    int  _currentPass = -1;
    bool _ledState    = false;
};
