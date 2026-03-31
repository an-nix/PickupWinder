#pragma once
#include <FastAccelStepper.h>
#include "Config.h"

// States used by the lateral-axis homing and traversal state machine.
enum class LatState {
    FAULT,          // Sensor missing or invalid, all lateral motion blocked.
    BACKOFF,        // Sensor already active at boot, moving away until released.
    HOMING,         // Moving toward the home switch.
    HOMING_DECEL,   // Home reached, waiting for normal deceleration stop.
    HOMING_ALIGN,   // Aligning to a clean full-step position to avoid buzzing.
    HOMING_OFFSET,  // Applying switch-to-zero offset travel.
    HOMED,          // Logical zero established, driver kept enabled.
    POSITIONING,    // Moving to the requested winding start position.
    WINDING_FWD,    // Forward synchronized traversal (0 -> end bound).
    WINDING_BWD,    // Backward synchronized traversal (end bound -> 0).
};

// Controls the lateral wire guide stepper with automatic homing.
//
// Startup sequence:
//   1. Validate the NO/NC sensor pair.
//   2. If already on the switch, back off until the sensor releases.
//   3. Run toward home at LAT_HOME_SPEED_HZ.
//   4. Stop on sensor hit, align, apply the configured home offset, then enter HOMED.
//
// Sensor protocol (INPUT_PULLUP, sensor closes to GND):
//   Away from home: NO=HIGH, NC=LOW
//   At home:        NO=LOW,  NC=HIGH
//   Fault:          NO=HIGH, NC=HIGH (typically disconnected sensor)
class LateralController {
public:
    /**
     * @brief Initialize lateral axis and start homing sequence.
     * @param engine Shared FastAccelStepper engine.
        * @param homeOffset_mm Offset from switch trigger to logical position zero.
        */
        void begin(FastAccelStepperEngine& engine, float homeOffset_mm = LAT_HOME_OFFSET_DEFAULT_MM);

    /**
     * @brief Non-blocking state-machine update.
     * @par Usage
     * Call on every main-loop iteration.
     */
    void update();

    /** @brief Get current lateral-axis state. */
    LatState    getState()  const { return _state; }
    /** @brief Check if axis is homed or currently traversing. */
    bool        isHomed()   const {
        return _state == LatState::HOMED
            || _state == LatState::WINDING_FWD
            || _state == LatState::WINDING_BWD;
    }
    /** @brief Check whether axis is in fault state. */
    bool        isFault()   const { return _state == LatState::FAULT;  }
    /** @brief Human-readable state string. */
    const char* stateStr()  const;

    /**
     * @brief Restart homing sequence manually.
     * @par Usage
     * Used by service UI and recovery workflows.
     */
    void rehome();

    /**
     * @brief Move carriage to requested start position.
        * @param start_mm Target position in mm from bobbin base.
        * @param speedHz Positioning speed in step-Hz.
        */
        void prepareStartPosition(float start_mm, uint32_t speedHz = LAT_TRAVERSE_SPEED_HZ);
    /** @brief True when axis is homed exactly at configured start position. */
    bool isPositionedForStart() const { return _state == LatState::HOMED && _isAtStartPosition(); }
    /** @brief Convenience helper parking carriage at 0 mm. */
    void parkAtZero() { prepareStartPosition(0.0f); }
    /** @brief Check if carriage is at physical/electrical zero (home). */
    bool isAtZero() const { return _state == LatState::HOMED && _stepper && abs(_stepper->getCurrentPosition()) <= MICROSTEPPING; }
    /** @brief True when lateral motor currently runs. */
    bool isBusy() const { return _stepper && _stepper->isRunning(); }

    /**
     * @brief Relative jog move from current/target position.
        * @param delta_mm Relative displacement in mm.
        */
        void jog(float delta_mm);
    /** @brief Arm pause on next natural reversal. */
    void armPauseOnNextReversal() { _pauseOnNextReversal = true; _pausedAtReversal = false; }
    /** @brief Arm one-shot stop at next high bound. */
    void armStopAtNextHigh() { _stopOnNextHigh = true; _pausedAtReversal = false; }
    /** @brief Arm one-shot stop at next low bound. */
    void armStopAtNextLow()  { _stopOnNextLow  = true; _pausedAtReversal = false; }
    /** @brief Check whether stop-on-high one-shot is armed. */
    bool isStopOnNextHighArmed() const { return _stopOnNextHigh; }
    /** @brief Check whether stop-on-low one-shot is armed. */
    bool isStopOnNextLowArmed()  const { return _stopOnNextLow; }
    /** @brief Clear all one-shot stop/pause flags. */
    void clearOneShotStops();
    /** @brief True if any stop-at-next-bound or pause-at-reversal flag is armed. */
    bool hasStopAtNextBoundArmed() const { return _stopOnNextHigh || _stopOnNextLow || _pauseOnNextReversal; }
    /**
     * @brief Consume and reset paused-at-reversal event latch.
     * @return true if a reversal pause event was pending.
     */
    bool consumePausedAtReversal() {
        bool v = _pausedAtReversal;
        _pausedAtReversal = false;
        return v;
    }

    /**
     * @brief Start synchronized lateral traversal for winding.
     * @param windingHz Spindle step frequency.
     * @param tpp Active turns-per-pass.
    * @param start_mm Low bound in mm.
    * @param end_mm High bound in mm.
     * @param speedScale Traverse speed multiplier.
     */
    void startWinding(uint32_t windingHz, float tpp, float start_mm, float end_mm, float speedScale = 1.0f);

    /**
     * @brief Update synchronized traversal parameters while running.
     */
    void updateWinding(uint32_t windingHz, float tpp, float start_mm, float end_mm, float speedScale = 1.0f);

    /** @brief Stop synchronized traversal and return to HOMED state. */
    void stopWinding();

    /** @brief True while lateral axis is inside reversal transition window. */
    bool isReversing() const { return millis() < _reversingUntilMs; }
    /** @brief True while actively traversing forward/backward. */
    bool isTraversing() const {
        return _state == LatState::WINDING_FWD || _state == LatState::WINDING_BWD;
    }
    /** @brief Number of completed half-passes since traversal start. */
    uint32_t getPassCount() const { return _passCount; }
    /** @brief Normalized progress (0..1) in current traversal direction. */
    float    getTraversalProgress() const;

    /**
     * @brief Set home offset between switch trigger and logical zero.
     * @param mm Offset in mm.
     */
    void  setHomeOffset(float mm);
    /** @brief Get configured home offset in mm. */
    float getHomeOffset() const { return _homeOffset_mm; }
    /** @brief Get current carriage position in mm. */
    float getCurrentPositionMm() const;
    /**
     * @brief Get current target position in mm.
     *
     * In `POSITIONING`, returns in-flight target. In `HOMED`, returns current
     * physical position.
     */
    float getTargetPositionMm() const;

private:
    FastAccelStepper*      _stepper         = nullptr;
    LatState               _state           = LatState::FAULT;
    uint32_t               _lastCheckMs     = 0;
    float                  _homeOffset_mm    = LAT_HOME_OFFSET_DEFAULT_MM;  // Switch-to-zero offset loaded from NVS
    volatile bool          _homeFlag        = false;

    // Active synchronized traversal state during winding.
    uint32_t               _latHz           = 0;    // Current lateral speed in steps/s
    int32_t                _latStartSteps   = 0;    // Effective low bound in steps
    int32_t                _latEndSteps     = 0;    // Effective high bound in steps
    uint32_t               _reversingUntilMs = 0;   // End of the spindle slowdown window
    uint32_t               _passCount       = 0;    // Completed half-passes
    bool                   _pauseOnNextReversal = false;
    bool                   _stopOnNextHigh      = false;
    bool                   _stopOnNextLow       = false;
    bool                   _pausedAtReversal    = false;
    bool                   _lastDirFwd          = true;   // Direction remembered across pauses

    // Compute synchronized lateral speed in steps/s from spindle speed and geometry.
    // lat_Hz = effWidth_steps * windingHz / (tpp * STEPS_PER_REV)
    uint32_t _calcLatHz(uint32_t windingHz, float tpp, float width_mm, float speedScale) const;
    bool     _isAtStartPosition() const;
    void     _setTraverseBounds(float start_mm, float end_mm);

    // Start the spindle slowdown window that covers a lateral reversal.
    void _onReversal();

    // ── Sensor reads (INPUT_PULLUP, closed contact to GND = LOW) ───────────
    // NO active means the normally-open contact is closed at home.
    bool _noActive()       const { return digitalRead(HOME_PIN_NO) == LOW; }
    // NC active means the normally-closed contact is closed away from home.
    bool _ncActive()       const { return digitalRead(HOME_PIN_NC) == LOW; }
    // A valid sensor presents complementary NO/NC levels.
    bool _sensorPresent()  const { return _noActive() != _ncActive(); }
    // At home the NO contact is closed and the NC contact is open.
    bool _atHome()         const { return _noActive() && !_ncActive(); }

    // ── GPIO ISR used for low-latency home detection ───────────────────────
    // Attached to HOME_PIN_NO FALLING only while HOMING is active.
    // attachInterruptArg() is used to reach the instance without a global singleton.
    static void IRAM_ATTR _homePinISR(void* arg);
    void _attachHomeISR();
    void _detachHomeISR();

    void _startHoming();
    void _startBackoff();
    void _applyOffsetOrNext(); // After alignment: apply offset or enter HOMED directly.
    void _gotoHomed();         // Final transition: axis ready for winding.
    void _enableDriver();
    void _disableDriver();
};
