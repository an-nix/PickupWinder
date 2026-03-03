#pragma once
#include "StepperController.h"
#include "SpeedInput.h"
#include "WebInterface.h"
#include "WindingGeometry.h"
#include "LEDController.h"
#include "StepperMusic.h"

// Operating mode: MANUAL = potentiometer controls speed directly.
// AUTO = future mode where a second stepper drives the wire guide axis.
enum class WinderMode { MANUAL, AUTO };

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
    StepperController _stepper;  // Stepper motor abstraction
    SpeedInput        _pot;      // Potentiometer with sliding-window filter
    WebInterface      _web;      // WiFi + HTTP + WebSocket server
    LEDController     _led;      // Traverse guide LED (toggles each pass)
    WindingGeometry   _geom;     // Bobbin geometry and turns-per-pass calculation

    // These fields are marked volatile because they can be written from the
    // WebSocket callback (running on FreeRTOS Core 0) and read from loop()
    // (running on Core 1). Without volatile the compiler may cache stale values.
    volatile WinderMode _mode         = WinderMode::MANUAL;
    volatile Direction  _direction    = Direction::CW;
    volatile long       _targetTurns  = DEFAULT_TARGET_TURNS;
    volatile bool       _freerun      = false;  // true = no auto-stop
    volatile bool       _motorEnabled = true;   // false = blocked until pot returns to 0

    // Safety interlock: the motor will only restart after the pot has returned
    // to zero (below POT_DEADZONE_STOP). Prevents accidental re-start after stop.
    bool     _potWasZero     = true;

    // When true, the driver will be disabled as soon as isRunning() goes false.
    // Set by _stop() so the motor decelerates smoothly before power is cut.
    bool     _pendingDisable = false;

    // Set to true when the winding target is reached. Blocks any restart via the
    // potentiometer until the user explicitly presses Reset in the web UI.
    bool     _targetReached  = false;

    // ── Music mode ────────────────────────────────────────────────────────
    volatile bool _playingMusic  = false;  // True while a melody is playing
    size_t        _musicIdx      = 0;      // Current note index in melody
    uint32_t      _musicNoteMs   = 0;      // millis() when current note started

    uint32_t _lastWsMs  = 0;  // Timestamp of last WebSocket status push
    uint32_t _lastPotMs = 0;  // Timestamp of last pot reading

    // Initiate a controlled stop: disable motor, set _pendingDisable, reset LED.
    void _stop();

    // Dispatch a WebSocket command (cmd/value pair) to the appropriate handler.
    void _handleCommand(const String& cmd, const String& value);
};
