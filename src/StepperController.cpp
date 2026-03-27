#include "StepperController.h"
#include <Arduino.h>
#include "Diag.h"

/**
 * @brief Initialize spindle stepper channel and safe startup state.
 */
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
        Diag::error("[Stepper] ERROR: could not initialise stepper!");
        while (true);  // Halt — nothing can run without a stepper
    }

    _stepper->setDirectionPin(DIR_PIN);
    _stepper->setAcceleration(ACCELERATION);
    _stepper->setSpeedInHz(_speedHz);  // Set the initial speed target

    Diag::info("[Stepper] OK");
}

/**
 * @brief Update spindle speed setpoint with runtime-safe behavior.
 */
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

/**
 * @brief Start spindle rotation in selected direction.
 */
void StepperController::start(bool forward) {
    if (!_stepper) return;
    // If the driver was disabled, enable it first and let the rotor settle on
    // the nearest electrical step before emitting pulses.
    if (!_driverEnabled) {
        digitalWrite(ENABLE_PIN, LOW);
        _driverEnabled = true;
        delay(30);  // Allow a short silent rotor settle before motion.
    }
    // Start directly with the requested target speed. FastAccelStepper will ramp
    // from zero using the configured acceleration profile. Earlier revisions used
    // SPEED_HZ_START as a workaround for a pre-commit bug in setSpeedHz(); that
    // workaround is no longer needed because setSpeedHz() now avoids applying a
    // live speed change when the motor is stopped.
    _stepper->setSpeedInHz(_speedHz);
    if (forward)
        _stepper->runForward();
    else
        _stepper->runBackward();
}

/** @brief Request controlled stop (deceleration ramp). */
void StepperController::stop() {
    if (_stepper) {
        // Ramp speed down to zero following the configured ACCELERATION profile.
        // The driver remains enabled during deceleration so no steps are lost.
        // WinderApp calls disableDriver() once isRunning() returns false.
        _stepper->stopMove();
    }
}

/** @brief Disable spindle driver output. */
void StepperController::disableDriver() {
    // Only call this after the motor has fully stopped.
    digitalWrite(ENABLE_PIN, HIGH);
    _driverEnabled = false;
}

/** @brief Immediate stop without ramp, then disable driver. */
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

/** @brief Check if spindle is currently moving. */
bool StepperController::isRunning() const {
    // Returns true while steps are still being generated (including ramp-down).
    return _stepper && _stepper->isRunning();
}

/** @brief Get completed full turns from absolute step position. */
long StepperController::getTurns() const {
    if (!_stepper) return 0;
    // Convert absolute step position to full revolutions.
    // abs() handles both CW and CCW directions.
    return abs(_stepper->getCurrentPosition()) / STEPS_PER_REV;
}

/** @brief Reset spindle turns counter to zero. */
void StepperController::resetTurns() {
    // Reset the internal step counter to 0 — turn count starts fresh.
    if (_stepper) _stepper->setCurrentPosition(0);
}

/** @brief Get instantaneous spindle speed in RPM. */
float StepperController::getRPM() const {
    if (!_stepper) return 0.0f;
    // getCurrentSpeedInMilliHz returns the real-time speed in mHz
    // (0 when stopped, reflects actual ramp state during accel/decel).
    int32_t mhz = _stepper->getCurrentSpeedInMilliHz();
    return abs((float)mhz) / 1000.0f * 60.0f / STEPS_PER_REV;
}
