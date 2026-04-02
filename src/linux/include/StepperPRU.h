/* StepperPRU.h — Spindle stepper API for BeagleBone Black.
 *
 * Drop-in replacement for the original StepperController (ESP32/FastAccelStepper).
 * Translates the same public API into IPC commands sent to PRU0.
 *
 * All methods are safe to call when IpcChannel is not open (they return
 * silently so the Linux application does not crash during hardware-free tests).
 */

#pragma once
#include <cstdint>
#include "IpcChannel.h"

class StepperPRU {
public:
    explicit StepperPRU(IpcChannel &channel);

    /** Attach to an opened IpcChannel and set initial acceleration.
     *  Mirrors: StepperController::begin(engine) */
    void begin(uint32_t accelHzPerMs = 1000u);

    /** Update spindle target speed. Clamped to [SPEED_HZ_MIN, SPEED_HZ_MAX].
     *  Mirrors: StepperController::setSpeedHz(hz) */
    void setSpeedHz(uint32_t hz);

    /** Start continuous rotation.
     *  Mirrors: StepperController::start(forward) */
    void start(bool forward = true);

    /** Controlled deceleration stop.
     *  Mirrors: StepperController::stop() */
    void stop();

    /** Immediate stop (no ramp) + disable driver.
     *  Mirrors: StepperController::forceStop() */
    void forceStop();

    /** Disable driver output.
     *  Mirrors: StepperController::disableDriver() */
    void disableDriver();

    /** True if motor is generating steps (running OR decelerating).
     *  Mirrors: StepperController::isRunning() */
    bool isRunning() const;

    /** Complete turns since last resetTurns().
     *  Mirrors: StepperController::getTurns() */
    long getTurns() const;

    /** Reset turn counter to 0.
     *  Mirrors: StepperController::resetTurns() */
    void resetTurns();

    /** Instantaneous speed in RPM.
     *  Mirrors: StepperController::getRPM() */
    float getRPM() const;

    /** Last commanded speed in Hz. */
    uint32_t getSpeedHz() const { return _speedHz; }

    /** Compute compensation for constant turns-per-mm ratio.
     *  Mirrors: StepperController::calculateCompensatedSpindleHz()
     *  NOTE: On BBB this is also executed in PRU0. This host-side version
     *        is kept for unit-testing and fallback. */
    static uint32_t calculateCompensatedSpindleHz(
        uint32_t nominalSpindleHz,
        uint32_t currentLatHz,
        uint32_t nominalLatHz);

private:
    IpcChannel &_channel;
    uint32_t    _speedHz   = 0u;
    uint32_t    _accelHzMs = 1000u;

    /* Cached telemetry (refreshed on each isRunning / getTurns call) */
    mutable pru_axis_telem_t _telem = {};
    void _refreshTelem() const;
};
