#pragma once
#include <FastAccelStepper.h>
#include "Config.h"

// ── StepperController ────────────────────────────────────────────────────────
// Wraps FastAccelStepper to provide a simple motor control interface.
// Handles: initialisation, speed changes, controlled/immediate stop,
// turn counting (via step position), and driver enable/disable via ENABLE_PIN.
class StepperController {
public:
    // Initialise GPIO pins, FastAccelStepper engine and set default speed.
    void     begin();

    // Update target speed (Hz). Clamped to [SPEED_HZ_MIN, SPEED_HZ_MAX].
    // applySpeedAcceleration() is called so the change takes effect immediately
    // even while the motor is already running.
    void     setSpeedHz(uint32_t hz);

    // Enable the driver and start continuous rotation.
    // forward=true → CW, forward=false → CCW.
    void     start(bool forward = true);

    // Initiate a controlled deceleration to zero (ramp-down).
    // The driver stays enabled during the deceleration phase;
    // call disableDriver() once isRunning() returns false.
    void     stop();

    // Immediately freeze the motor at its current position (no ramp-down).
    // Also disables the driver right away. Used for pot-to-zero and
    // direction changes to avoid coasting through the resonance zone.
    void     forceStop();

    // Cut power to the motor driver (ENABLE_PIN HIGH).
    // Should be called only after the motor has fully stopped (isRunning()==false)
    // to avoid losing steps or making the motor jerk.
    void     disableDriver();

    // Returns true while the motor is still moving (including deceleration ramp).
    bool     isRunning()  const;

    // Returns the absolute number of full revolutions completed since the last
    // resetTurns() call. Derived from FastAccelStepper::getCurrentPosition().
    long     getTurns()   const;

    // Reset the step position counter to 0 (turn counter back to zero).
    void     resetTurns();

    // Returns the actual instantaneous speed in RPM using getCurrentSpeedInMilliHz().
    float    getRPM()     const;

    // Returns the last speed setpoint in Hz (may differ from actual speed
    // while accelerating or decelerating).
    uint32_t getSpeedHz() const { return _speedHz; }

private:
    FastAccelStepperEngine _engine;             // FastAccelStepper engine (manages hardware timers)
    FastAccelStepper*      _stepper = nullptr;  // Pointer to the stepper instance
    uint32_t               _speedHz = SPEED_HZ_MIN;  // Current speed setpoint in Hz
};
