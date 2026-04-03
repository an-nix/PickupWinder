/* StepperPRU.h — Spindle stepper API for BeagleBone Black.
 *
 * Drop-in replacement for the original StepperController (ESP32/FastAccelStepper).
 * Translates the same public API into Klipper-style move pushes to PRU0.
 *
 * Public API is identical to the original — no application-layer changes needed.
 * Internal change: all step generation is via MoveQueue (pre-computed on host).
 * The PRU executes move rings using the 200 MHz IEP counter.
 *
 * IMPORTANT: call tick() from the control loop (≤ 10 ms) to refill the move
 * ring and apply speed transitions. Without tick() the motor will stop after
 * the initially pushed moves are exhausted.
 */

#pragma once
#include <cstdint>
#include "IpcChannel.h"
#include "MoveQueue.h"

class StepperPRU {
public:
    explicit StepperPRU(IpcChannel &channel);

    /** Attach to an opened IpcChannel and set initial acceleration.
     *  Mirrors: StepperController::begin(engine) */
    void begin(uint32_t accelHzPerMs = 1000u);

    /** Update spindle target speed. Clamped to [SPEED_HZ_MIN, SPEED_HZ_MAX].
     *  Speed transition pushed to PRU on next tick().
     *  Mirrors: StepperController::setSpeedHz(hz) */
    void setSpeedHz(uint32_t hz);

    /** Start continuous rotation. Pushes accel segment + initial constant block.
     *  Mirrors: StepperController::start(forward) */
    void start(bool forward = true);

    /** Controlled deceleration stop. Pushes decel segment, sets streaming=false.
     *  Mirrors: StepperController::stop() */
    void stop();

    /** Immediate stop (flush ring + disable driver).
     *  Mirrors: StepperController::forceStop() */
    void forceStop();

    /** Disable driver output.
     *  Mirrors: StepperController::disableDriver() */
    void disableDriver();

    /** True if motor is generating steps.
     *  Mirrors: StepperController::isRunning() */
    bool isRunning() const;

    /** Complete turns since last resetTurns().
     *  Mirrors: StepperController::getTurns() */
    long getTurns() const;

    /** Reset turn counter to 0.
     *  Mirrors: StepperController::resetTurns() */
    void resetTurns();

    /** Instantaneous speed in RPM (derived from PRU telemetry).
     *  Mirrors: StepperController::getRPM() */
    float getRPM() const;

    /** Last committed speed in Hz. */
    uint32_t getSpeedHz() const { return _speedHz; }

    /** Refill the move ring and apply pending speed changes.
     *  MUST be called from the control loop every ≤ 10 ms while running.
     *  New method (no equivalent in original API). */
    void tick();

    /** Host-side spindle-lateral ratio compensation.
     *  Mirrors: StepperController::calculateCompensatedSpindleHz() */
    static uint32_t calculateCompensatedSpindleHz(uint32_t nominalSpindleHz,
                                                   uint32_t currentLatHz,
                                                   uint32_t nominalLatHz);

private:
    IpcChannel &_channel;

    uint32_t  _speedHz        = 0u;
    uint32_t  _pushedHz       = 0u;   /* last Hz pushed into ring */
    uint32_t  _accelHzPerS    = 0u;   /* converted from accelHzMs at begin() */
    bool      _forward        = true;
    bool      _streaming      = false; /* true while continuously winding */
    bool      _speedChanged   = false; /* setSpeedHz() painted a new target */

    /* Cached telemetry, refreshed by const accessors */
    mutable pru_axis_telem_t _telem = {};
    void _refreshTelem() const;

    static constexpr uint32_t STEPS_PER_REV = 6400u;
    static constexpr uint32_t SPEED_HZ_MIN  = 1067u;
    static constexpr uint32_t SPEED_HZ_MAX  = 160000u;
    /* Constant-speed refill chunk: 10 ms worth of edges per push */
    static constexpr uint32_t REFILL_MS     = 10u;
    /* Keep at least this many move-ring slots occupied while streaming */
    static constexpr uint32_t KEEP_SLOTS    = 4u;
};

