#pragma once
#include <FastAccelStepper.h>
#include "Config.h"

class StepperController {
public:
    void     begin();

    void     setSpeedHz(uint32_t hz);
    void     start(bool forward = true);  // true = CW, false = CCW
    void     stop();           // Décélération contrôlée (ne désactive pas le driver)
    void     forceStop();      // Arrêt immédiat (par pot ou urgence)
    void     disableDriver();  // Désactiver le driver (après arrêt complet)

    bool     isRunning()  const;
    long     getTurns()   const;   // Basé sur getCurrentPosition()
    void     resetTurns();

    float    getRPM()     const;   // Vitesse instantanée réelle
    uint32_t getSpeedHz() const { return _speedHz; }

private:
    FastAccelStepperEngine _engine;
    FastAccelStepper*      _stepper = nullptr;
    uint32_t               _speedHz = SPEED_HZ_MIN;
};
