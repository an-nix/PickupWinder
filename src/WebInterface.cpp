#include "WebInterface.h"
#include <WiFi.h>

// Fichier embarqué à la compilation via board_build.embed_txtfiles dans platformio.ini
extern const uint8_t index_html_start[] asm("_binary_data_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_data_index_html_end");

// ── Implémentation ────────────────────────────────────────────────────────────

WebInterface::WebInterface() : _server(WEB_PORT), _ws("/ws") {}

void WebInterface::begin() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Connexion");

    uint8_t tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500);
        Serial.print(".");
        tries++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] Échec — interface web désactivée. Le moteur fonctionne quand même.");
        return;
    }

    _wifiOk = true;
    Serial.printf("\n[WiFi] Connecté — IP : %s\n", WiFi.localIP().toString().c_str());

    // Enregistrer le handler WebSocket
    _ws.onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c,
                        AwsEventType t, void* a, uint8_t* d, size_t l) {
        _onWsEvent(s, c, t, a, d, l);
    });
    _server.addHandler(&_ws);

    // Route principale → page HTML
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        size_t len = index_html_end - index_html_start;
        req->send(200, "text/html", index_html_start, len);
    });

    _server.begin();
    Serial.println("[Web] Serveur démarré");
}

void WebInterface::sendUpdate(const WinderStatus& s) {
    if (!_wifiOk || _ws.count() == 0) return;

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
    if (type != WS_EVT_DATA) return;

    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    // Traiter uniquement les frames TEXT complètes
    if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) return;

    String msg((char*)data, len);

    // Extraction simple de {"cmd":"xxx","val":"yyy"}
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

    if (cmd.length() > 0 && _callback) {
        _callback(cmd, val);
    }
}
