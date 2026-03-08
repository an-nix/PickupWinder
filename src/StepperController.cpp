#include "StepperController.h"
#include <Arduino.h>

void StepperController::begin(FastAccelStepperEngine& engine) {
    // Keep the driver disabled at startup so the motor coils are not energised
    // until the user explicitly starts winding.
    // IMPORTANT: write HIGH to the output register BEFORE enabling the output driver.
    // On ESP32, pinMode(OUTPUT) defaults to LOW, causing a brief LOW glitch that
    // would energise the coils and make a clunk on every power cycle.
    digitalWrite(ENABLE_PIN, HIGH);  // Pre-set output register HIGH
    pinMode(ENABLE_PIN, OUTPUT);     // Enable output driver — starts HIGH, no glitch
    _driverEnabled = false;

    _stepper = engine.stepperConnectToPin(STEP_PIN);
    if (!_stepper) {
        // Fatal error: cannot allocate a hardware timer channel for this pin.
        Serial.println("[Stepper] ERROR: could not initialise stepper!");
        while (true);  // Halt — nothing can run without a stepper
    }

    _stepper->setDirectionPin(DIR_PIN);
    _stepper->setAcceleration(ACCELERATION);
    _stepper->setSpeedInHz(_speedHz);  // Set the initial speed target

    Serial.println("[Stepper] OK");
}

void StepperController::setSpeedHz(uint32_t hz) {
    // Clamp the requested speed to the configured min/max range.
    uint32_t clamped = constrain(hz, SPEED_HZ_MIN, SPEED_HZ_MAX);
    // Skip update if speed has not changed (avoids unnecessary I2C-like overhead).
    if (clamped == _speedHz) return;
    _speedHz = clamped;
    // Only commit the speed change to FastAccelStepper if the motor is already
    // running.  When stopped, applySpeedAcceleration() would "commit" the target
    // speed inside the library so that the next runForward() starts directly at
    // that speed — bypassing the SPEED_HZ_START soft-start in start() and
    // causing a violent jerk on every restart.
    // When the motor is stopped, _speedHz is updated here and start() will use
    // it to set the final ramp target after launching from SPEED_HZ_START.
    if (_stepper->isRunning()) {
        _stepper->setSpeedInHz(_speedHz);
        _stepper->applySpeedAcceleration();
    }
}

void StepperController::start(bool forward) {
    if (!_stepper) return;
    // Si le driver était éteint, l'activer et laisser le rotor se caler
    // sur la position de pas la plus proche avant d'émettre des impulsions.
    if (!_driverEnabled) {
        digitalWrite(ENABLE_PIN, LOW);
        _driverEnabled = true;
        delay(30);  // 30 ms — calement silencieux du rotor
    }
    // Démarrer directement à la vitesse cible : FastAccelStepper accélère de 0
    // à _speedHz en utilisant setAcceleration(). SPEED_HZ_START était un contournement
    // de l'ancien bug où setSpeedHz() pré-committait la vitesse via applySpeedAcceleration()
    // même quand le moteur était arrêté. Ce bug est corrigé : setSpeedHz() n'appelle
    // plus applySpeedAcceleration() quand isRunning()==false, donc la bibliothèque
    // reçoit toujours un état propre (vitesse=0) à l'appel de runForward/runBackward.
    _stepper->setSpeedInHz(_speedHz);
    if (forward)
        _stepper->runForward();
    else
        _stepper->runBackward();
}

void StepperController::stop() {
    if (_stepper) {
        // Ramp speed down to zero following the configured ACCELERATION profile.
        // The driver remains enabled during deceleration so no steps are lost.
        // WinderApp calls disableDriver() once isRunning() returns false.
        _stepper->stopMove();
    }
}

void StepperController::disableDriver() {
    // Only call this after the motor has fully stopped.
    digitalWrite(ENABLE_PIN, HIGH);
    _driverEnabled = false;
}

void StepperController::forceStop() {
    if (_stepper) {
        // Freeze position immediately (no deceleration ramp).
        // forceStopAndNewPosition() resets the internal position target so
        // the library does not try to continue a previous move command.
        _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
        // Disable driver immediately — no controlled stop needed here.
        digitalWrite(ENABLE_PIN, HIGH);
        _driverEnabled = false;
    }
}

bool StepperController::isRunning() const {
    // Returns true while steps are still being generated (including ramp-down).
    return _stepper && _stepper->isRunning();
}

long StepperController::getTurns() const {
    if (!_stepper) return 0;
    // Convert absolute step position to full revolutions.
    // abs() handles both CW and CCW directions.
    return abs(_stepper->getCurrentPosition()) / STEPS_PER_REV;
}

void StepperController::resetTurns() {
    // Reset the internal step counter to 0 — turn count starts fresh.
    if (_stepper) _stepper->setCurrentPosition(0);
}

float StepperController::getRPM() const {
    if (!_stepper) return 0.0f;
    // getCurrentSpeedInMilliHz returns the real-time speed in mHz
    // (0 when stopped, reflects actual ramp state during accel/decel).
    int32_t mhz = _stepper->getCurrentSpeedInMilliHz();
    return abs((float)mhz) / 1000.0f * 60.0f / STEPS_PER_REV;
}

void StepperController::playNote(uint16_t freqHz) {
    if (!_stepper) return;
    if (freqHz == 0) {
        // Rest: decelerate smoothly (no violent forceStop).
        _stepper->stopMove();
        return;
    }
    // Convert musical note frequency to step frequency.
    // The motor's audible vibration = step_freq / MOTOR_NOTE_MULT.
    uint32_t stepHz = (uint32_t)freqHz * MOTOR_NOTE_MULT;
    // Acceleration of 350k gives ~100ms ramp to any note — smooth enough
    // to avoid a startup clunk while still tracking note changes cleanly.
    if (!_driverEnabled) {
        digitalWrite(ENABLE_PIN, LOW);
        _driverEnabled = true;
        delay(20);  // Coil settle before first note
    }
    _stepper->setAcceleration(350000);
    _stepper->setSpeedInHz(stepHz);
    if (!_stepper->isRunning()) {
        _stepper->runForward();
    } else {
        _stepper->applySpeedAcceleration();
    }
}

void StepperController::stopNote() {
    if (!_stepper) return;
    _stepper->setAcceleration(350000);
    _stepper->stopMove();
    // Restore normal winding acceleration.
    _stepper->setAcceleration(ACCELERATION);
    digitalWrite(ENABLE_PIN, HIGH);
}
