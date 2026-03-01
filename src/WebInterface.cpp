#include "WebInterface.h"
#include <WiFi.h>

// The HTML file is embedded into the firmware binary at compile time.
// platformio.ini: board_build.embed_txtfiles = data/index.html
// The linker exposes the file content via these two symbols.
extern const uint8_t index_html_start[] asm("_binary_data_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_data_index_html_end");

WebInterface::WebInterface() : _server(WEB_PORT), _ws("/ws") {}

void WebInterface::begin() {
    // Attempt WiFi connection with up to 20 retries (10 seconds total).
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Connecting");

    uint8_t tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500);
        Serial.print(".");
        tries++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        // WiFi failure is non-fatal: the motor still works via the potentiometer.
        Serial.println("\n[WiFi] Failed — web interface disabled. Motor still works.");
        return;
    }

    _wifiOk = true;
    Serial.printf("\n[WiFi] Connected — IP: %s\n", WiFi.localIP().toString().c_str());

    // Register the WebSocket event handler using a lambda to forward calls
    // to the private _onWsEvent method (captures `this`).
    _ws.onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c,
                        AwsEventType t, void* a, uint8_t* d, size_t l) {
        _onWsEvent(s, c, t, a, d, l);
    });
    _server.addHandler(&_ws);

    // Serve the embedded HTML file on GET /
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        size_t len = index_html_end - index_html_start;
        req->send(200, "text/html", index_html_start, len);
    });

    _server.begin();
    Serial.println("[Web] Server started");
}

void WebInterface::sendUpdate(const WinderStatus& s) {
    // Skip if WiFi is down or no client is connected.
    if (!_wifiOk || _ws.count() == 0) return;

    // Serialise the full machine state to a compact JSON string.
    // The buffer size (320 bytes) is sized to fit all fields with margin.
    char buf[320];
    snprintf(buf, sizeof(buf),
        "{\"rpm\":%.0f,\"hz\":%u,\"turns\":%ld,\"target\":%ld,"
        "\"running\":%s,\"enabled\":%s,\"freerun\":%s,\"cw\":%s,\"auto\":%s,"
        "\"tpp\":%ld,\"pass\":%d,\"eff\":%.2f,"
        "\"gt\":%.2f,\"gb\":%.2f,\"gtp\":%.2f,\"gm\":%.2f,\"gw\":%.4f}",
        s.rpm, s.speedHz, s.turns, s.targetTurns,
        s.running      ? "true" : "false",
        s.motorEnabled ? "true" : "false",
        s.freerun      ? "true" : "false",
        s.directionCW  ? "true" : "false",
        s.autoMode     ? "true" : "false",
        s.turnsPerPass, s.currentPass, s.effectiveWidth_mm,
        s.geomTotal, s.geomBottom, s.geomTop, s.geomMargin, s.geomWire);
    // Broadcast to all connected WebSocket clients.
    _ws.textAll(buf);
}

void WebInterface::setCommandCallback(CommandCallback cb) {
    _callback = cb;
}

String WebInterface::getIP() const {
    return _wifiOk ? WiFi.localIP().toString() : "N/A";
}

void WebInterface::_onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                               AwsEventType type, void* arg, uint8_t* data, size_t len) {
    // We only care about data frames; ignore connect/disconnect/error events.
    if (type != WS_EVT_DATA) return;

    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    // Accept only complete, single-frame TEXT messages.
    // Multi-frame or binary messages are ignored for simplicity.
    if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) return;

    String msg((char*)data, len);

    // Minimal JSON field extractor: finds the value for a given string key.
    // Expects the pattern "key":"value" inside the message.
    auto extract = [&](const char* key) -> String {
        String k = String("\"") + key + "\":\"";
        int i = msg.indexOf(k);
        if (i < 0) return "";
        int s = i + k.length();
        int e = msg.indexOf("\"", s);
        return (e > s) ? msg.substring(s, e) : "";
    };

    String cmd = extract("cmd");
    String val = extract("val");

    // Forward to the registered command handler (WinderApp::_handleCommand).
    if (cmd.length() > 0 && _callback) {
        _callback(cmd, val);
    }
}
