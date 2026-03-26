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
enum class PendingVerifyBound : uint8_t { NONE = 0, START = 1, END = 2 };

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
    /**
     * @brief Initialise the winding application and all subsystems.
     *
     * Restores recipe data, applies runtime configuration, initialises
     * steppers/LED/traverse axis and computes an initial traverse plan.
     *
     * @par Usage
     * Called once at startup from `main` before entering the main loop.
     */
    void begin();

    /**
     * @brief Periodic application tick.
     *
     * Runs one state-machine iteration: lateral events, pot-driven behavior,
     * auto-stop checks, and deferred driver disable handling.
     *
     * @param potHz Filtered speed input in Hz (0 means potentiometer at zero).
     * @par Usage
     * Called continuously from the main loop.
     */
    void tick(uint32_t potHz);

    /**
     * @brief Dispatch a textual command into the winding logic.
     *
     * Handles lifecycle, geometry, pattern and recipe commands from UI/serial.
     *
     * @param cmd Command key.
     * @param value Command value as string payload.
     * @par Usage
     * Invoked by web and serial bridge layers to control the machine.
     */
    void handleCommand(const String& cmd, const String& value);

    /**
     * @brief Apply a rotary-encoder delta to the active mode.
     *
     * In verify modes, updates low/high trim bounds and repositions carriage.
     * In manual mode, jogs the carriage within the active winding window.
     *
     * @param delta Encoder tick delta since the last call.
     * @par Usage
     * Called by the main-loop consumer of encoder events.
     */
    void handleEncoderDelta(int32_t delta);

    /**
     * @brief Build a full runtime status snapshot.
     *
     * @return Structured status used by WebSocket and serial telemetry.
     * @par Usage
     * Queried periodically by transport layers to publish live machine state.
     */
    WinderStatus getStatus() const;

    /**
     * @brief Export the current recipe as JSON.
     * @return JSON string representing the current recipe.
     * @par Usage
     * Used by the UI for recipe export/download.
     */
    String recipeJson() const;

    /**
     * @brief Check whether manual capture streaming is active.
     * @return true if capture mode is enabled, false otherwise.
     * @par Usage
     * Polled by streaming code before requesting capture points.
     */
    bool   isCaptureActive() const { return _captureActive; }

    /**
     * @brief Get one capture sample for streaming (manual mode).
     *
     * Emits a sample only when carriage position changed enough since the
     * previous emitted point.
     *
     * @param posMm Output carriage position in millimeters.
     * @param turns Output current turns count.
     * @return true when a new point is produced, false otherwise.
     * @par Usage
     * Called by WebSocket capture streaming path while manual mode is active.
     */
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
    volatile bool         _rewindMode   = false;
    volatile long         _rewindBatchTurns = DEFAULT_REWIND_BATCH_TURNS;
    volatile uint16_t     _rewindBatchRpm = DEFAULT_REWIND_BATCH_RPM;
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
    /**
     * @brief Transition to IDLE and reset session state.
     * @par Usage
     * Called by stop/reset flows and when a run completes.
     */
    void _toIdle();

    /**
     * @brief Transition to VERIFY_LOW state.
     * @par Usage
     * Triggered by startup flow, confirm flow, or `verify_low` command.
     */
    void _toVerifyLow();

    /**
     * @brief Transition to VERIFY_HIGH state.
     * @par Usage
     * Triggered by startup flow, confirm flow, or `verify_high` command.
     */
    void _toVerifyHigh();

    /**
     * @brief Transition to manual mode.
     * @par Usage
     * Triggered by `manual` command and post-verify manual entry flow.
     */
    void _toManual();         // Mode manuel : encodeur = jog libre, pot = moteur, capture WS

    /**
     * @brief Transition to lateral run-in (rodage) mode.
     * @par Usage
     * Triggered by `rodage` command when machine is idle.
     */
    void _toRodage();         // Rodage axe latéral : aller-retour N passes

    /**
     * @brief Transition to WINDING state.
     * @par Usage
     * Entered once both bounds are verified and run can start/resume.
     */
    void _toWinding();        // called from confirm once both bounds verified

    /**
     * @brief Transition to PAUSED state.
     * @par Usage
     * Used when pot drops to zero or when stopping at a bound.
     */
    void _toPaused();

    /**
     * @brief Transition to TARGET_REACHED state.
     * @par Usage
     * Called when turns count reaches configured target.
     */
    void _toTargetReached();

    // ── Tick helpers ──────────────────────────────────────────────────────────
    float _windingStartMm() const { return _geom.windingStartMm(); }
    float _windingEndMm()   const { return _geom.windingEndMm(); }

    /**
     * @brief Process lateral-axis events and state-dependent transitions.
     * @par Usage
     * Called by `tick()` every loop iteration.
     */
    void  _handleLateralEvents();

    /**
     * @brief Process potentiometer-driven behavior for current state.
     * @param hz Current requested motor speed in Hz.
     * @par Usage
     * Called by `tick()` every loop iteration.
     */
    void  _handlePotCycle(uint32_t hz);

    /**
     * @brief Check if lateral axis is in a safe state to spin.
     * @return true if axis is homed and not busy.
     */
    bool  _readyForSpin() const;

    /**
     * @brief Run/update winding at a given spindle speed.
     * @param hz Requested spindle speed in Hz.
     * @par Usage
     * Called from pot-cycle logic while machine is running or resuming.
     */
    void  _runWindingAtHz(uint32_t hz);

    /**
     * @brief Apply automatic stop and end-position logic.
     * @par Usage
     * Called by `tick()` during winding sessions.
     */
    void  _checkAutoStop();

    /**
     * @brief Disable stepper driver after motion fully stops.
     * @par Usage
     * Called by `tick()` to perform safe deferred power cut.
     */
    void  _applyDeferredDisable();

    /**
     * @brief Build the current traverse plan from planner + runtime state.
     * @param hz Current spindle speed in Hz.
     * @return Planner output used to drive lateral traversal.
     */
    TraversePlan _buildTraversePlan(uint32_t hz) const;

    // ── Command handlers ──────────────────────────────────────────────────────
    bool _parametersLocked() const { return _state != WindingState::IDLE; }

    /**
     * @brief Handle immediate lifecycle commands.
     * @param cmd Command key.
     * @param value Command payload.
     * @return true if command was handled.
     * @par Usage
     * First stage of `handleCommand()` dispatch.
     */
    bool _handleImmediateCommand(const String& cmd, const String& value);

    /**
     * @brief Handle geometry-related commands.
     * @param cmd Command key.
     * @param value Command payload.
     * @return true if command was handled.
     * @par Usage
     * Second stage of `handleCommand()` dispatch.
     */
    bool _handleGeometryCommand(const String& cmd, const String& value);

    /**
     * @brief Handle winding-pattern commands.
     * @param cmd Command key.
     * @param value Command payload.
     * @return true if command was handled.
     * @par Usage
     * Called by `handleCommand()` for style/seed/jitter parameters.
     */
    bool _handlePatternCommand(const String& cmd, const String& value);

    /**
     * @brief Reposition carriage when geometry bounds changed at rest.
     * @param startBoundChanged true if start bound changed.
     * @param endBoundChanged true if end bound changed.
     * @par Usage
     * Called by geometry command handlers to provide live operator feedback.
     */
    void _refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged);

    // ── Recipe helpers ────────────────────────────────────────────────────────
    /**
     * @brief Apply a recipe to all runtime subsystems.
     * @param recipe Recipe values to apply.
     * @param persist true to save recipe to NVS after applying.
     * @par Usage
     * Used on startup, import, and command-driven recipe changes.
     */
    void          _applyRecipe(const WindingRecipe& recipe, bool persist);

    /**
     * @brief Capture current live settings into a recipe snapshot.
     * @return Recipe snapshot reflecting mutable runtime fields.
     */
    WindingRecipe _captureRecipe() const;

    /**
     * @brief Persist current recipe snapshot and sync planner state.
     * @par Usage
     * Called after mutating recipe-related settings.
     */
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
    PendingVerifyBound _pendingVerify = PendingVerifyBound::NONE;
    // ── Position finale de bobinage ──────────────────────────────────────
    // Vrai dès que le hook de fin de bobinage a repositionné le chariot.
    // Remis à false à chaque _toWinding() / _toIdle().
    bool   _endPosArmed      = false;
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

