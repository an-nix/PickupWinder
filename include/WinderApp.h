#pragma once
#include <FastAccelStepper.h>
#include "StepperController.h"
#include "WindingGeometry.h"
#include "WindingPattern.h"
#include "WindingRecipeStore.h"
#include "LEDController.h"
#include "LateralController.h"
#include "Types.h"
#include "Config.h"

// Winding direction seen from the front of the bobbin.
enum class Direction { CW, CCW };

// ── WinderApp ─────────────────────────────────────────────────────────────────
// Winding controller. Owns the bobbin motor, traverse axis, LED guide and
// recipe persistence. Infrastructure (Serial, WiFi, LinkSerial, SpeedInput) is
// managed by main and injected via tick(potHz) and handleCommand().
//
// The session lifecycle is modelled as an explicit state machine (WindingState).
// All state transitions go through a dedicated _toXxx() method so the full
// set of state changes is visible in one place.
class WinderApp {
public:
    // Initialise winding subsystems. Serial must be open before calling begin().
    void begin();

    // Winding loop body — call from the main loop as fast as possible.
    // potHz : filtered speed value from SpeedInput (0 = pot at zero).
    void tick(uint32_t potHz);

    // Dispatch a command (from WebSocket or LinkSerial) into the winding logic.
    void handleCommand(const String& cmd, const String& value);

    // Called from the encoder ISR consumer (main loop) with the tick delta
    // since the last call.
    // VERIFY_LOW  → adjusts windingStartTrim_mm, repositions carriage
    // VERIFY_HIGH → adjusts windingEndTrim_mm,   repositions carriage
    // MANUAL      → jog direct du chariot (pas de trim, pas de limite de fenêtre)
    void handleEncoderDelta(int32_t delta);

    // Build a full status snapshot for pushing to WebSocket / LinkSerial.
    WinderStatus getStatus() const;

    // Return the current recipe serialised as JSON (for UI download).
    String recipeJson() const;

    // Retourne true si la capture de position est active (mode MANUAL).
    bool   isCaptureActive() const { return _captureActive; }

    // Retourne un point de capture à streamer si la position a changé de plus de
    // CAPTURE_MIN_DELTA_MM depuis le dernier envoi. Retourne false sinon.
    bool   getCapturePoint(float& posMm, long& turns) {
        if (!_captureActive) return false;
        float cur = _lateral.getCurrentPositionMm();
        if (fabsf(cur - _captureLastPosMm) < 0.05f) return false;
        _captureLastPosMm = cur;
        posMm = cur;
        turns = _stepper.getTurns();
        return true;
    }

private:
    // ── Hardware subsystems ───────────────────────────────────────────────────
    FastAccelStepperEngine _engine;
    StepperController      _stepper;
    LEDController          _led;
    WindingGeometry        _geom;
    WindingPatternPlanner  _planner;
    WindingRecipeStore     _recipeStore;
    LateralController      _lateral;
    WindingRecipe          _recipe;
    TraversePlan           _activePlan;

    // ── State machine ─────────────────────────────────────────────────────────
    // _state is the single source of truth for the winding lifecycle.
    // Marked volatile: handleCommand() runs on FreeRTOS Core 0 (WebSocket task)
    // while tick() runs on Core 1 (Arduino loop task).
    volatile WindingState _state        = WindingState::IDLE;

    // Recipe scalars that can be changed at runtime from the UI (Core 0).
    volatile Direction    _direction    = Direction::CW;
    volatile long         _targetTurns  = DEFAULT_TARGET_TURNS;
    volatile bool         _freerun      = false;

    // ── Transient hardware flags ──────────────────────────────────────────────
    // These cut across states and are not part of the session lifecycle.

    // Start/resume arm flag: must be true before the motor (re)starts.
    // Set by the pot returning to zero, or by a UI "resume" command.
    // Cleared on every state transition so each stop requires an explicit re-arm.
    bool _canStart = false;

    // Deferred driver disable: set when stopping so the driver is cut only
    // after the deceleration ramp finishes (checked in _applyDeferredDisable).
    bool _pendingDisable = false;

    // Verification tracking: set to true when the operator confirms each bound.
    // Both must be true before WINDING starts. Reset on _toIdle().
    bool _lowVerified  = false;
    bool _highVerified = false;

    // ── State transitions ─────────────────────────────────────────────────────
    void _toIdle();
    void _toVerifyLow();
    void _toVerifyHigh();
    void _toManual();         // Mode manuel : encodeur = jog libre, pot = moteur, capture WS
    void _toRodage();         // Rodage axe latéral : aller-retour N passes
    void _toWinding();        // called from confirm once both bounds verified
    void _toPaused();
    void _toTargetReached();

    // ── Tick helpers ──────────────────────────────────────────────────────────
    float _windingStartMm() const { return _geom.windingStartMm(); }
    float _windingEndMm()   const { return _geom.windingEndMm(); }
    void  _handleLateralEvents();
    void  _handlePotCycle(uint32_t hz);
    bool  _readyForSpin() const;
    void  _runWindingAtHz(uint32_t hz);
    void  _checkAutoStop();
    void  _applyDeferredDisable();
    TraversePlan _buildTraversePlan(uint32_t hz) const;

    // ── Command handlers ──────────────────────────────────────────────────────
    bool _parametersLocked() const { return _state != WindingState::IDLE; }
    bool _handleImmediateCommand(const String& cmd, const String& value);
    bool _handleGeometryCommand(const String& cmd, const String& value);
    bool _handlePatternCommand(const String& cmd, const String& value);
    void _refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged);

    // ── Recipe helpers ────────────────────────────────────────────────────────
    void          _applyRecipe(const WindingRecipe& recipe, bool persist);
    WindingRecipe _captureRecipe() const;
    void          _saveRecipe();
    // ── Mode manuel ─────────────────────────────────────────────────
    // Pas encodeur en mode manuel (mm) — réglable via commande 'manual_step'.
    float  _manualJogStepMm  = 0.05f;
    // Indique si la capture de la position est active (streaming WS).
    bool   _captureActive    = false;
    // Dernière position streamée (pour ne pas répéter si pas de mouvement).
    float  _captureLastPosMm = -999.0f;
    // Vrai si la commande 'manual' a été demandée depuis IDLE : après la
    // vérification des deux butées, on bascule en MANUAL au lieu de WINDING.
    bool   _pendingManual    = false;
    // Vrai jusqu'à ce que le chariot atteigne une butée en mode MANUAL.
    // Quand vrai, le pas encodeur est multiplié par MANUAL_FAST_STEP_MULT.
    bool   _manualFirstPass  = true;
    // ── Rodage axe latéral ──────────────────────────────────────
    // Nombre d'allers-retours à effectuer (paramétrable depuis l'UI).
    int    _rodagePasses   = 10;
    // Distance depuis la butee home jusqu'à la position max (mm).
    float  _rodageDistMm   = 80.0f;
    // Passes complétées depuis le début du rodage.
    int    _rodagePassDone = 0;
    // true = chariot en train d'aller vers _rodageDistMm
    // false = chariot en train de revenir vers 0
    bool   _rodageFwd      = true;};

