#pragma once
#include <FastAccelStepper.h>
#include "StepperController.h"
#include "SpeedInput.h"
#include "WebInterface.h"
#include "WindingGeometry.h"
#include "WindingPattern.h"
#include "WindingRecipeStore.h"
#include "LEDController.h"
#include "LinkSerial.h"
#include "LateralController.h"

// Winding direction seen from the front of the bobbin.
enum class Direction  { CW, CCW };

// ── WinderApp ─────────────────────────────────────────────────────────────────
// Top-level application class. Owns all subsystems and orchestrates the main
// control loop: pot reading, motor control, auto-stop, LED guide and WebSocket.
class WinderApp {
public:
    // Initialise all subsystems (serial, stepper, pot, LED, web) and
    // register the WebSocket command callback.
    void begin();

    // Main loop body — must be called from Arduino loop() as fast as possible.
    // Handles: pot sampling, motor start/stop, auto-stop on target,
    // deferred driver disable, and periodic WebSocket status push.
    void run();

private:
    FastAccelStepperEngine _engine;    // Engine FastAccelStepper unique — partagée entre tous les steppers
    StepperController  _stepper;  // Stepper motor abstraction
    SpeedInput         _pot;      // Potentiometer with sliding-window filter
    WebInterface       _web;      // WiFi + HTTP + WebSocket server
    LEDController      _led;      // Traverse guide LED (toggles each pass)
    WindingGeometry    _geom;     // Bobbin geometry and turns-per-pass calculation
    WindingPatternPlanner _planner; // Profil de bobinage déterministe (droit/scatter/humain)
    WindingRecipeStore _recipeStore; // Persistance NVS + export JSON de la recette
    LinkSerial         _link;     // Liaison UART2 vers l'ESP écran
    LateralController  _lateral;  // Axe latéral avec homing automatique
    WindingRecipe      _recipe;   // Recette courante — source de vérité des paramètres
    TraversePlan       _activePlan; // Dernier plan latéral appliqué

    // These fields are marked volatile because they can be written from the
    // WebSocket callback (running on FreeRTOS Core 0) and read from loop()
    // (running on Core 1). Without volatile the compiler may cache stale values.
    volatile Direction  _direction    = Direction::CW;
    volatile long       _targetTurns  = DEFAULT_TARGET_TURNS;
    volatile bool       _freerun      = false;  // true = no auto-stop
    volatile bool       _motorEnabled = false;  // false = blocked until pot returns to 0
    volatile bool       _startRequested = false; // true after pressing Start in the UI
    bool                _pauseOnFirstReversal  = false;
    bool                _pausedForVerification = false;
    bool                _midWindingPaused      = false; // true : pause mid-bobinage, reprend sans Start
    bool                _inVerificationRun     = false; // true : passe initiale lente (cap vitesse)
    bool                _resumeFromCurrentPos  = false; // pause en passe initiale : reprise à la position courante
    // Safety interlock: pot must return to zero (below POT_ADC_ZERO_BAND) before the
    // motor can start — including the very first start after power-on.
    bool     _potWasZero     = false;

    // When true, the driver will be disabled as soon as isRunning() goes false.
    // Set by _stop() so the motor decelerates smoothly before power is cut.
    bool     _pendingDisable = false;

    // Set to true when the winding target is reached. Blocks any restart via the
    // potentiometer until the user explicitly presses Reset in the web UI.
    bool     _targetReached  = false;

    uint32_t _lastWsMs   = 0;  // Timestamp of last WebSocket status push
    uint32_t _lastPotMs  = 0;  // Timestamp of last pot reading
    uint32_t _lastLinkMs = 0;  // Timestamp of last UART status push to display ESP

    TraversePlan _buildTraversePlan(uint32_t windingHz) const;
    float _windingStartMm() const { return _geom.windingStartMm(); }
    float _windingEndMm()   const { return _geom.windingEndMm(); }
    void _handleLateralEvents();
    void _handlePotCycle();
    bool _readyForSpin() const;
    void _runWindingAtHz(uint32_t hz);
    void _checkAutoStop();
    void _applyDeferredDisable();
    void _pollSerialLink(uint32_t now);
    void _pushWebStatus(uint32_t now);

    void _applyRecipe(const WindingRecipe& recipe, bool persist);
    WindingRecipe _captureRecipe() const;
    void _saveRecipe();
    bool _parametersLocked() const;

    void _refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged);
    bool _handleImmediateCommand(const String& cmd, const String& value);
    bool _handleGeometryCommand(const String& cmd, const String& value);
    bool _handlePatternCommand(const String& cmd, const String& value);

    // Initiate a controlled stop: disable motor, set _pendingDisable, reset LED.
    void _stop();
    // Pause mid-winding (pot→0): keeps _startRequested so pot-up resumes without pressing Start.
    void _pause();

    // Dispatch a WebSocket command (cmd/value pair) to the appropriate handler.
    void _handleCommand(const String& cmd, const String& value);
};
