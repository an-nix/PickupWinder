#include "StepperController.h"
#include <Arduino.h>
#include "Diag.h"

void StepperController::_servicePendingStart() {
    if (!_stepper || !_startPending) return;
    const uint32_t now = millis();
    if ((int32_t)(now - _startReadyAtMs) < 0) return;

    _stepper->setSpeedInHz(_speedHz);
    _stepper->applySpeedAcceleration();
    if (_startForward) _stepper->runForward();
    else               _stepper->runBackward();
    _startPending = false;

#if DIAG_VERBOSE
    Diag::infof("[Stepper] start committed: targetHz=%u", _speedHz);
#endif
}

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
    _servicePendingStart();

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
    _startForward = forward;

    // If already pending, only update direction and target speed.
    if (_startPending) return;

    // If the driver was disabled, enable it first and let the rotor settle on
    // the nearest electrical step before emitting pulses.
    if (!_driverEnabled) {
        digitalWrite(ENABLE_PIN, LOW);
        _driverEnabled = true;
        _startPending = true;
        _startReadyAtMs = millis() + 30;
#if DIAG_VERBOSE
        Diag::infof("[Stepper] start deferred: targetHz=%u", _speedHz);
#endif
        return;
    }
    // Start directly with the requested target speed. FastAccelStepper will ramp
    // from zero using the configured acceleration profile. Earlier revisions used
    // SPEED_HZ_START as a workaround for a pre-commit bug in setSpeedHz(); that
    // workaround is no longer needed because setSpeedHz() now avoids applying a
    // live speed change when the motor is stopped.
    _stepper->setSpeedInHz(_speedHz);
#if DIAG_VERBOSE
    Diag::infof("[Stepper] start immediate: targetHz=%u", _speedHz);
#endif
    _stepper->applySpeedAcceleration();
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
    const_cast<StepperController*>(this)->_servicePendingStart();
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

/**
 * @brief Calculate spindle speed compensated for carriage velocity to maintain constant turns-per-mm.
 * 
 * Ensures uniform wire density by adjusting spindle speed proportionally when carriage
 * velocity changes (e.g., during reversals where lateral stepper ramping occurs).
 * 
 * Compensation formula: spindle_hz_comp = spindle_hz_nom × (lat_hz_actual / lat_hz_nominal)
 * 
 * This maintains the critical ratio: Turns_per_mm = Spindle_RPM / Carriage_velocity_mm_s = CONSTANT
 */
uint32_t StepperController::calculateCompensatedSpindleHz(
    uint32_t nominalSpindleHz,
    uint32_t currentLatHz,
    uint32_t nominalLatHz
) {
    // Protect against division by zero and invalid inputs
    if (nominalLatHz == 0 || nominalSpindleHz == 0) {
        return nominalSpindleHz;
    }
    
    // Minimum carriage velocity threshold to avoid division issues when velocity is near-zero
    const uint32_t MIN_LAT_HZ_THRESHOLD = 100;  // ~0.03 mm/s at LAT_STEPS_PER_MM = 3072
    
    if (currentLatHz < MIN_LAT_HZ_THRESHOLD) {
        // During complete stop, maintain nominal spindle speed rather than stopping spindle.
        // This creates a brief edge artifact (a few extra turns at reversal point)
        // which is acceptable if duration is very short (100-600ms).
        return nominalSpindleHz;
    }
    
    // Compute compensation: (current / nominal) ratio applied to spindle speed
    float ratio = (float)currentLatHz / (float)nominalLatHz;
    ratio = constrain(ratio, 0.3f, 1.5f);  // Clamp to reasonable bounds to prevent instability
    
    uint32_t compensated = (uint32_t)((float)nominalSpindleHz * ratio);
    
    // Ensure the result stays within configured spindle speed limits
    return constrain(compensated, SPEED_HZ_MIN, SPEED_HZ_MAX);
}
