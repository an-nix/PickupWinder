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
	/**
	 * @brief Construct web interface objects.
	 */
    WebInterface();

    /**
     * @brief Start WiFi, HTTP server and WebSocket endpoint.
     * @par Usage
     * Call once at startup after command callbacks are configured.
     */
    void   begin();

    /**
     * @brief Broadcast machine status to all connected WebSocket clients.
     * @param status Full status snapshot to serialize and send.
     */
    void   sendUpdate(const WinderStatus& status);

    /**
     * @brief Broadcast one manual-capture sample.
     * @param timestampMs Timestamp in milliseconds.
     * @param posMm Carriage position in millimeters.
     * @param turns Turns counter at sample time.
     */
    void   sendCapture(uint32_t timestampMs, float posMm, long turns);

    /**
     * @brief Register command dispatch callback for incoming WS commands.
     * @param cb Callback receiving `(cmd, value)`.
     */
    void   setCommandCallback(CommandCallback cb);

    /**
     * @brief Register JSON recipe provider used by export endpoint.
     * @param cb Callback returning serialized recipe JSON.
     */
    void   setRecipeProvider(RecipeJsonProvider cb);

    /**
     * @brief Get local WiFi IP string.
     * @return Local IP, or "N/A" when disconnected.
     */
    String getIP()         const;

    /**
     * @brief Check whether web interface is online.
     * @return true if WiFi + server are operational.
     */
    bool   isConnected()   const { return _wifiOk; }

private:
    AsyncWebServer  _server;    // ESPAsyncWebServer instance on WEB_PORT
    AsyncWebSocket  _ws;        // WebSocket handler mounted at /ws
    CommandCallback _callback;  // Registered command handler (set by WinderApp)
    RecipeJsonProvider _recipeProvider; // Download current recipe as JSON
    bool            _wifiOk = false;  // Set to true once WiFi is connected

    /**
     * @brief Internal WS event dispatcher.
     *
     * Routes text data frames to the registered command callback.
     */
    void _onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len);
};
