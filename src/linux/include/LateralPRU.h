/* LateralPRU.h — Lateral carriage stepper + home sensor API for BeagleBone Black.
 *
 * Drop-in replacement for the original LateralController (ESP32/FastAccelStepper).
 * Translates the same public API into IPC commands sent to PRU1.
 *
 * All methods guard `if (!_channel.isOpen()) return;` to survive boot failures.
 */

#pragma once
#include <cstdint>
#include "IpcChannel.h"

/* Mirror of original LatState enum for full API compatibility */
enum class LatState {
    FAULT,
    BACKOFF,
    HOMING,
    HOMING_DECEL,
    HOMING_ALIGN,
    HOMING_OFFSET,
    HOMED,
    POSITIONING,
    WINDING_FWD,
    WINDING_BWD,
};

class LateralPRU {
public:
    explicit LateralPRU(IpcChannel &channel);

    /** Attach to IpcChannel and start homing sequence.
     *  Mirrors: LateralController::begin(engine, homeOffsetMm) */
    void begin(float homeOffsetMm = 15.0f);

    /** Non-blocking state machine tick. Call every control loop iteration.
     *  Mirrors: LateralController::update() */
    void update();

    /* ── State queries ──────────────────────────────────────────────────────*/
    LatState    getState()    const { return _state; }
    bool        isHomed()     const;
    bool        isFault()     const { return _state == LatState::FAULT; }
    bool        isBusy()      const;
    const char *stateStr()    const;
    bool        isAtZero()    const;
    bool        isPositionedForStart() const;

    /* ── Motion commands ────────────────────────────────────────────────────*/
    /** Restart homing. Mirrors: LateralController::rehome() */
    void rehome();

    /** Move to start position before winding.
     *  Mirrors: LateralController::prepareStartPosition(mm, speedHz) */
    void prepareStartPosition(float startMm,
                               uint32_t speedHz = 4800u);

    /** Park at 0 mm. Mirrors: LateralController::parkAtZero() */
    void parkAtZero() { prepareStartPosition(0.0f); }

    /** Relative jog. Mirrors: LateralController::jog(deltaMm) */
    void jog(float deltaMm);

    /* ── Synchronized winding traversal ────────────────────────────────────*/
    /** Start winding traversal.
     *  Mirrors: LateralController::startWinding(...) */
    void startWinding(uint32_t windingHz, long tpp,
                      float startMm, float endMm,
                      float speedScale = 1.0f);

    /** Update winding traversal (call every control loop tick).
     *  Mirrors: LateralController::updateWinding(...) */
    void updateWinding(uint32_t windingHz, long tpp, float speedScale);

    /* ── One-shot stop flags ────────────────────────────────────────────────*/
    void armPauseOnNextReversal()  { _pauseOnNextReversal = true; }
    void armStopAtNextHigh()       { _stopOnNextHigh = true; }
    void armStopAtNextLow()        { _stopOnNextLow  = true; }
    void clearOneShotStops();
    bool isStopOnNextHighArmed()   const { return _stopOnNextHigh; }
    bool isStopOnNextLowArmed()    const { return _stopOnNextLow;  }
    bool hasStopAtNextBoundArmed() const;
    bool consumePausedAtReversal();

    /* ── Position ───────────────────────────────────────────────────────────*/
    /** Current carriage position in mm from home. */
    float getPositionMm() const;

    /** Current lateral traverse frequency (Hz). Used for compensation. */
    uint32_t getCurrentLateralHz() const;

private:
    IpcChannel &_channel;
    LatState    _state = LatState::FAULT;

    float    _homeOffsetMm  = 15.0f;
    float    _startMm       = 0.0f;
    float    _endMm         = 0.0f;
    float    _startPosMm    = 0.0f;
    bool     _stopOnNextHigh       = false;
    bool     _stopOnNextLow        = false;
    bool     _pauseOnNextReversal  = false;
    bool     _pausedAtReversal     = false;

    mutable pru_axis_telem_t _telem = {};
    void _refreshTelem() const;

    uint32_t _mmToSteps(float mm) const;
    float    _stepsToMm(int32_t steps) const;
};
