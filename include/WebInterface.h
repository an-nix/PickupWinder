#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <functional>
#include "Config.h"
#include "Types.h"

// WinderStatus and RecipeJsonProvider are defined in Types.h.
// CommandCallback is defined in Types.h.

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

    // Envoie un point de capture de position au format compact vers tous les clients WS.
    // Appelé uniquement en mode MANUAL quand la capture est active.
    // Format : {"type":"cap","t":1234,"pos":12.34,"turns":1500}
    void   sendCapture(uint32_t timestampMs, float posMm, long turns);

    // Register the callback that will be invoked for each incoming WebSocket command.
    void   setCommandCallback(CommandCallback cb);
    void   setRecipeProvider(RecipeJsonProvider cb);

    // Returns the ESP32 local IP as a string (or "N/A" if not connected).
    String getIP()         const;

    // Returns true if WiFi and the web server are up and running.
    bool   isConnected()   const { return _wifiOk; }

private:
    AsyncWebServer  _server;    // ESPAsyncWebServer instance on WEB_PORT
    AsyncWebSocket  _ws;        // WebSocket handler mounted at /ws
    CommandCallback _callback;  // Registered command handler (set by WinderApp)
    RecipeJsonProvider _recipeProvider; // Download current recipe as JSON
    bool            _wifiOk = false;  // Set to true once WiFi is connected

    // Internal WebSocket event dispatcher; routes DATA frames to _callback.
    void _onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len);
};
