#pragma once
#include <FastAccelStepper.h>
#include "Config.h"

// ── StepperController ────────────────────────────────────────────────────────
// Wraps FastAccelStepper to provide a simple motor control interface.
// Handles: initialisation, speed changes, controlled/immediate stop,
// turn counting (via step position), and driver enable/disable via ENABLE_PIN.
class StepperController {
public:
    /**
     * @brief Initialize spindle stepper driver and attach shared engine.
     * @param engine Shared FastAccelStepper engine.
     */
    void     begin(FastAccelStepperEngine& engine);

    /**
     * @brief Update target speed setpoint.
     * @param hz Requested speed in Hz.
     * @note Value is clamped to configured min/max.
     */
    void     setSpeedHz(uint32_t hz);

    /**
     * @brief Start continuous spindle rotation.
     * @param forward true for CW, false for CCW.
     */
    void     start(bool forward = true);

    /**
     * @brief Request controlled deceleration stop.
     */
    void     stop();

    /**
     * @brief Immediately stop spindle and disable driver.
     */
    void     forceStop();

    /**
     * @brief Disable stepper driver output stage.
     */
    void     disableDriver();

    /** @brief Check whether motor is currently moving. */
    bool     isRunning()  const;

    /** @brief Get full-turn counter since last reset. */
    long     getTurns()   const;

    /** @brief Reset turns counter to zero. */
    void     resetTurns();

    /** @brief Get measured instantaneous speed in RPM. */
    float    getRPM()     const;

    /** @brief Get last commanded speed setpoint in Hz. */
    uint32_t getSpeedHz() const { return _speedHz; }

private:
    FastAccelStepper*      _stepper = nullptr;  // Pointer to the stepper instance
    uint32_t               _speedHz = SPEED_HZ_MIN;  // Current speed setpoint in Hz
    bool                   _driverEnabled = false;   // True when ENABLE_PIN is LOW (driver on)
};
