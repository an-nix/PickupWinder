#include "StepperController.h"
#include <Arduino.h>

void StepperController::begin() {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);  // Activer le driver (LOW = actif)

    _engine.init();
    _stepper = _engine.stepperConnectToPin(STEP_PIN);
    if (!_stepper) {
        Serial.println("[Stepper] ERREUR : impossible d'initialiser !");
        while (true);
    }

    _stepper->setDirectionPin(DIR_PIN);
    _stepper->setAcceleration(ACCELERATION);
    _stepper->setSpeedInHz(_speedHz);

    Serial.println("[Stepper] OK");
}

void StepperController::setSpeedHz(uint32_t hz) {
    uint32_t clamped = constrain(hz, SPEED_HZ_MIN, SPEED_HZ_MAX);
    if (clamped == _speedHz) return;
    _speedHz = clamped;
    _stepper->setSpeedInHz(_speedHz);
    // La vitesse est prise en compte immédiatement par FastAccelStepper,
    // y compris pendant la rotation (transition douce via l'accélération)
}

void StepperController::start(bool forward) {
    if (!_stepper) return;
    _stepper->setSpeedInHz(_speedHz);
    if (forward)
        _stepper->runForward();
    else
        _stepper->runBackward();
}

void StepperController::stop() {
    if (_stepper) _stepper->stopMove();
}

bool StepperController::isRunning() const {
    return _stepper && _stepper->isRunning();
}

long StepperController::getTurns() const {
    if (!_stepper) return 0;
    return abs(_stepper->getCurrentPosition()) / STEPS_PER_REV;
}

void StepperController::resetTurns() {
    if (_stepper) _stepper->setCurrentPosition(0);
}

float StepperController::getRPM() const {
    if (!_stepper) return 0.0f;
    // getCurrentSpeedInMilliHz retourne la vitesse instantanée réelle (0 pendant l'arrêt)
    int32_t mhz = _stepper->getCurrentSpeedInMilliHz();
    return abs((float)mhz) / 1000.0f * 60.0f / STEPS_PER_REV;
}
