#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <functional>
#include "Config.h"

// Callback type invoked by WebInterface when a command arrives over WebSocket.
// Parameters: cmd (command name), val (optional value string).
using CommandCallback = std::function<void(const String&, const String&)>;

// Full machine state snapshot sent to all connected WebSocket clients
// every WS_UPDATE_MS milliseconds.
struct WinderStatus {
    float    rpm;              // Actual instantaneous speed in RPM
    uint32_t speedHz;         // Current speed setpoint in Hz
    long     turns;           // Total turns since last reset
    long     targetTurns;     // Target turn count for auto-stop
    bool     running;         // True while the motor is moving
    bool     motorEnabled;    // False after stop — pot must return to 0 to re-enable
    bool     freerun;         // True = no auto-stop (free-run mode)
    bool     directionCW;     // True = clockwise, false = counter-clockwise
    bool     autoMode;        // True = auto traverse mode (future)
    // Winding geometry and pass guide (used by the LED traverse indicator)
    long     turnsPerPass;      // Effective turns per pass (calc + offset)
    long     turnsPerPassCalc;  // Auto-calculated turns per pass (geometry only)
    long     turnsPerPassOffset; // Offset applied to auto-calc
    int      currentPass;     // Current pass number (0-based)
    float    effectiveWidth_mm; // Usable winding width after flanges and margins
    float    geomTotal, geomBottom, geomTop, geomMargin, geomWire; // Raw geometry values
};

// ── WebInterface ─────────────────────────────────────────────────────────────
// Manages WiFi connection, HTTP server (serves embedded HTML) and WebSocket.
// The HTML file (data/index.html) is embedded at compile time via
// board_build.embed_txtfiles in platformio.ini and served as a binary symbol.
class WebInterface {
public:
    WebInterface();

    // Connect to WiFi, register WebSocket handler, start HTTP server.
    void   begin();

    // Serialise a WinderStatus snapshot to JSON and broadcast to all WS clients.
    void   sendUpdate(const WinderStatus& status);

    // Register the callback that will be invoked for each incoming WebSocket command.
    void   setCommandCallback(CommandCallback cb);

    // Returns the ESP32 local IP as a string (or "N/A" if not connected).
    String getIP()         const;

    // Returns true if WiFi and the web server are up and running.
    bool   isConnected()   const { return _wifiOk; }

private:
    AsyncWebServer  _server;    // ESPAsyncWebServer instance on WEB_PORT
    AsyncWebSocket  _ws;        // WebSocket handler mounted at /ws
    CommandCallback _callback;  // Registered command handler (set by WinderApp)
    bool            _wifiOk = false;  // Set to true once WiFi is connected

    // Internal WebSocket event dispatcher; routes DATA frames to _callback.
    void _onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len);
};
