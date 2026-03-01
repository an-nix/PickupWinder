#include "StepperController.h"
#include <Arduino.h>

void StepperController::begin() {
    // Keep the driver disabled at startup so the motor coils are not energised
    // until the user explicitly starts winding.
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);  // HIGH = driver disabled (active-LOW enable)

    // Initialise the FastAccelStepper hardware timer engine.
    _engine.init();
    _stepper = _engine.stepperConnectToPin(STEP_PIN);
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
    _stepper->setSpeedInHz(_speedHz);
    // applySpeedAcceleration() is required to make the new speed take effect
    // immediately while the motor is already running; without it the change
    // would only apply on the next start() call.
    _stepper->applySpeedAcceleration();
}

void StepperController::start(bool forward) {
    if (!_stepper) return;
    // Enable the driver (active-LOW) before issuing move commands.
    digitalWrite(ENABLE_PIN, LOW);
    _stepper->setSpeedInHz(_speedHz);
    // Run indefinitely in the requested direction.
    if (forward)
        _stepper->runForward();   // CW
    else
        _stepper->runBackward();  // CCW
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
    // De-energise the motor coils to save power and reduce heat.
    // Only call this after the motor has fully stopped.
    digitalWrite(ENABLE_PIN, HIGH);
}

void StepperController::forceStop() {
    if (_stepper) {
        // Freeze position immediately (no deceleration ramp).
        // forceStopAndNewPosition() resets the internal position target so
        // the library does not try to continue a previous move command.
        _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
        // Disable driver immediately — no controlled stop needed here.
        digitalWrite(ENABLE_PIN, HIGH);
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
