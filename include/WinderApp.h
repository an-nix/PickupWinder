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

// ── WinderApp ─────────────────────────────────────────────────────────────────
// Simplified winding controller.
// States: IDLE, WINDING, PAUSED, TARGET_REACHED, MANUAL, RODAGE.
// Verify is NOT a separate state — just flags during normal PAUSED→WINDING flow.
// Start button = pot at max. Pause button = pot at zero.
class WinderApp {
public:
    void begin();
    void tick(uint32_t potHz);
    void handleCommand(const String& cmd, const String& value);
    void handleEncoderDelta(int32_t delta);
    WinderStatus getStatus() const;
    String recipeJson() const;
    bool isCaptureActive() const { return _captureActive; }
    bool getCapturePoint(float& posMm, long& turns) {
        if (!_captureActive) return false;
        float cur = _lateral.getCurrentPositionMm();
        if (fabsf(cur - _captureLastPosMm) < 0.05f) return false;
        _captureLastPosMm = cur;
        posMm = cur;
        turns = _stepper.getTurns();
        return true;
    }

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
    volatile WindingState _state        = WindingState::IDLE;
    volatile Direction    _direction    = Direction::CW;
    volatile long         _targetTurns  = DEFAULT_TARGET_TURNS;
    volatile uint32_t     _maxSpeedHz   = SPEED_HZ_MAX;
    volatile bool         _freerun      = false;

    // ── Control flags ──
    bool _canStart       = false;   // Arm: pot must return to 0 before (re)start
    bool _pendingDisable = false;   // Deferred driver disable after stop
    bool _startButtonMax = false;   // Start button latch: drive at _maxSpeedHz
    volatile bool _pauseRequested = false; // Cross-task pause request, consumed in tick()

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
    void _toManual();
    void _toRodage();

    // ── Tick helpers ──
    float _windingStartMm() const { return _geom.windingStartMm(); }
    float _windingEndMm()   const { return _geom.windingEndMm(); }
    void  _handleLateralEvents();
    void  _handlePotCycle(uint32_t hz);
    void  _runWindingAtHz(uint32_t hz);
    void  _checkAutoStop();
    void  _applyDeferredDisable();
    TraversePlan _buildTraversePlan(uint32_t hz) const;
    bool  _readyForSpin() const;

    // ── Command handlers ──
    bool _parametersLocked() const { return _state != WindingState::IDLE; }
    bool _handleImmediateCommand(const String& cmd, const String& value);
    bool _handleGeometryCommand(const String& cmd, const String& value);
    bool _handlePatternCommand(const String& cmd, const String& value);
    void _refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged);

    // ── Recipe helpers ──
    void          _applyRecipe(const WindingRecipe& recipe, bool persist);
    WindingRecipe _captureRecipe() const;
    void          _saveRecipe();

    // ── Manual mode ──
    float  _manualJogStepMm  = 0.05f;
    bool   _captureActive    = false;
    float  _captureLastPosMm = -999.0f;
    bool   _pendingManual    = false;
    bool   _manualFirstPass  = true;

    // ── Rodage ──
    int    _rodagePasses   = 10;
    float  _rodageDistMm   = 80.0f;
    int    _rodagePassDone = 0;
    bool   _rodageFwd      = true;
};
