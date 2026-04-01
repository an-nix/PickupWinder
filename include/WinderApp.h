#pragma once
#include <FastAccelStepper.h>
#include "StepperController.h"
#include "WindingGeometry.h"
#include "WindingPattern.h"
#include "WindingRecipeStore.h"
#include "LateralController.h"
#include "Types.h"
#include "Config.h"

enum class Direction { CW, CCW };

/**
 * @brief Main winding domain controller.
 *
 * `WinderApp` owns the spindle, lateral axis, recipe state and the winding
 * state machine. It does not read raw hardware inputs directly; higher layers
 * convert user intent into commands and speed setpoints.
 */
class WinderApp {
public:
    /** @brief Initialize motors, recipe storage and derived runtime state. */
    void begin();

    /** @brief Advance the winding state machine by one deterministic step. */
    void tick();

    /** @brief Set the live spindle speed command in step-Hz. */
    void setControlHz(uint32_t hz) { _inputHz = hz; }

    /** @brief Set the requested target turns count. */
    void setTargetTurns(long t) { _targetTurns = t; }

    /** @brief Enable or disable freerun mode. */
    void setFreerun(bool f) { _freerun = f; }

    /** @brief Set spindle direction. */
    void setDirection(Direction d) { _direction = d; }

    /** @brief Set the maximum spindle speed in RPM. */
    void setMaxRpm(uint16_t rpm) { _maxSpeedHz = (uint32_t)rpm * (uint32_t)STEPS_PER_REV / 60UL; }

    /** @brief Return the current configured spindle speed ceiling in step-Hz. */
    uint32_t getMaxSpeedHz() const { return _maxSpeedHz; }

    /** @brief Request transition to paused state. */
    void pauseWinding() { _toPaused(); }

    /** @brief Request transition to idle state. */
    void stopWinding() { _toIdle(); }

    /** @brief Dispatch a user or transport command into the winding domain. */
    void handleCommand(const char* cmd, const char* value);

    /** @brief Apply one encoder delta to the interactive trim logic. */
    void handleEncoderDelta(int32_t delta);

    /** @brief Build a snapshot of all UI-facing status fields. */
    WinderStatus getStatus() const;

    /** @brief Serialize the active recipe to JSON into caller buffer. */
    void recipeJson(char* buf, size_t len) const;

private:
    // ── Hardware subsystems ──
    FastAccelStepperEngine _engine;
    StepperController      _stepper;
    WindingGeometry        _geom;
    WindingPatternPlanner  _planner;
    WindingRecipeStore     _recipeStore;
    LateralController      _lateral;
    WindingRecipe          _recipe;
    TraversePlan           _activePlan;

    // ── State machine ──
    WindingState _state        = WindingState::IDLE;
    Direction    _direction    = Direction::CW;
    long         _targetTurns  = DEFAULT_TARGET_TURNS;
    uint32_t     _maxSpeedHz   = SPEED_HZ_MAX;
    uint32_t     _inputHz      = 0;
    bool         _freerun      = false;

    // ── Control flags ──
    bool _pendingDisable = false;   // Deferred driver disable after stop
    bool _pauseRequested = false; // Consumed in the control task tick.

    // Burst mode removed: simpler single-mode winding

    // ── Verify flags (modify startup, NOT separate states) ──
    bool _verifyLowPending  = false;  // Waiting for low-bound positioning
    bool _verifyHighPending = false;  // Winding will stop at next high bound
    bool _positioningToLow  = false;  // Currently positioning to low bound

    // ── End position ──
    bool _endPosArmed = false;

    // ── State transitions ──
    void _toIdle();
    void _toWinding();
    void _toPaused();
    void _toTargetReached();
    void _toRodage();

    // ── Tick helpers ──
    float _windingStartMm() const { return _geom.windingStartMm(); }
    float _windingEndMm()   const { return _geom.windingEndMm(); }
    void  _handleLateralEvents();
    void  _processInputHz(uint32_t hz); // NEW: input commands (from session controller)
    void  _runWindingAtHz(uint32_t hz);
    void  _checkAutoStop();
    void  _applyDeferredDisable();
    TraversePlan _buildTraversePlan(uint32_t hz) const;
    bool  _readyForSpin() const;

    // ── Command handlers ──
    bool _parametersLocked() const { return _state != WindingState::IDLE; }
    bool _handleImmediateCommand(const char* cmd, const char* value);
    bool _handleGeometryCommand(const char* cmd, const char* value);
    bool _handlePatternCommand(const char* cmd, const char* value);
    void _refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged);

    // ── Recipe helpers ──
    void          _applyRecipe(const WindingRecipe& recipe, bool persist);
    WindingRecipe _captureRecipe() const;
    void          _saveRecipe();

    // ── Rodage ──
    int    _rodagePasses   = 10;
    float  _rodageDistMm   = 80.0f;
    int    _rodagePassDone = 0;
    bool   _rodageFwd      = true;
};
