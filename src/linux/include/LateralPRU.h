/* LateralPRU.h — Lateral carriage stepper + home sensor API for BeagleBone Black.
 *
 * Drop-in replacement for the original LateralController (ESP32/FastAccelStepper).
 * Translates the same public API into Klipper-style move pushes to PRU1.
 *
 * Internal change: step generation via MoveQueue (pre-computed on host).
 * PRU1 executes move rings using the 200 MHz IEP counter.
 * Homing uses CMD_HOME_START: PRU1 auto-stops + resets position on HOME_HIT.
 *
 * Public API is identical to the original — no application-layer changes needed.
 * New MUST-call: tick() from the control loop for winding traversal management.
 */

#pragma once
#include <cstdint>
#include "IpcChannel.h"
#include "MoveQueue.h"

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

    /** Attach to IpcChannel and start homing sequence. */
    void begin(float homeOffsetMm = 15.0f);

    /** Non-blocking state machine tick. Call every control loop iteration. */
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
    void rehome();
    void prepareStartPosition(float startMm, uint32_t speedHz = 4800u);
    void parkAtZero() { prepareStartPosition(0.0f); }
    void jog(float deltaMm);

    /* ── Synchronized winding traversal ────────────────────────────────────*/
    void startWinding(uint32_t windingHz, long tpp,
                      float startMm, float endMm,
                      float speedScale = 1.0f);
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
    float    getPositionMm()       const;
    uint32_t getCurrentLateralHz() const;

private:
    IpcChannel &_channel;
    LatState    _state = LatState::FAULT;

    float    _homeOffsetMm       = 15.0f;
    float    _startMm            = 0.0f;
    float    _endMm              = 0.0f;
    float    _startPosMm         = 0.0f;
    uint32_t _windingLatHz       = 0u;    /* last computed lateral winding Hz */
    bool     _stopOnNextHigh     = false;
    bool     _stopOnNextLow      = false;
    bool     _pauseOnNextReversal= false;
    bool     _pausedAtReversal   = false;

    mutable pru_axis_telem_t _telem = {};
    void _refreshTelem() const;

    static uint32_t _mmToSteps(float mm);
    static float    _stepsToMm(int32_t steps);

    /* Push a constant-speed traverse from A to B using MoveQueue. */
    bool _pushTraverse(float fromMm, float toMm, uint32_t latHz);

    static constexpr uint32_t LAT_STEPS_PER_MM = 3072u;
    static constexpr uint32_t LAT_HOME_HZ      = 4800u;
    static constexpr uint32_t LAT_ACCEL_HZ_S   = 48000u; /* 48 kHz/s */
};
