#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <functional>
#include "Config.h"

using CommandCallback = std::function<void(const String&, const String&)>;

struct WinderStatus {
    float    rpm;
    uint32_t speedHz;
    long     turns;
    long     targetTurns;
    bool     running;
    bool     motorEnabled;
    bool     freerun;
    bool     directionCW;
    bool     autoMode;
    // Géométrie / LED
    long     turnsPerPass;
    int      currentPass;
    float    effectiveWidth_mm;
    float    geomTotal, geomBottom, geomTop, geomMargin, geomWire;
};

class WebInterface {
public:
    WebInterface();

    void   begin();
    void   sendUpdate(const WinderStatus& status);
    void   setCommandCallback(CommandCallback cb);

    String getIP()         const;
    bool   isConnected()   const { return _wifiOk; }

private:
    AsyncWebServer  _server;
    AsyncWebSocket  _ws;
    CommandCallback _callback;
    bool            _wifiOk = false;

    void _onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len);
};
