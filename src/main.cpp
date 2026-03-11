// ── main.cpp ──────────────────────────────────────────────────────────────────
// Entry point for the Pickup Winder firmware.
//
// Infrastructure (Serial, WiFi, UART link to display) is owned here.
// WinderApp owns only the winding logic (motor, traverse, pot, LED, recipe).
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include "Config.h"
#include "SpeedInput.h"
#include "WebInterface.h"
#include "LinkSerial.h"
#include "WinderApp.h"
#include "Diag.h"

// ── Subsystem instances ───────────────────────────────────────────────────────
static SpeedInput   pot;
static WebInterface web;
static LinkSerial   link;
static WinderApp    winder;

// ── Timing ────────────────────────────────────────────────────────────────────
static uint32_t lastWsMs   = 0;
static uint32_t lastLinkMs = 0;
static uint32_t lastPotMs  = 0;

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Diag::info("\n=== Pickup Winder ===");

    pot.begin();   // Pre-fill ADC filter buffer
    web.begin();   // WiFi + WebSocket (blocks ~2-5 s during association)
    link.begin();  // UART2 liaison vers ESP écran
    winder.begin();

    // Both WebSocket and LinkSerial share the same command dispatcher.
    web.setCommandCallback([](const String& cmd, const String& val) {
        winder.handleCommand(cmd, val);
    });
    web.setRecipeProvider([]() {
        return winder.recipeJson();
    });

    if (web.isConnected()) {
        Diag::infof("→ Web interface: http://%s",
            web.getIP().c_str());
    }
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    const uint32_t now = millis();

    // Potentiometer: read at POT_READ_INTERVAL, pass filtered Hz to winder.
    // lastPotHz is retained between cycles so winder always sees the current value.
    static uint32_t lastPotHz = 0;
    if (now - lastPotMs >= POT_READ_INTERVAL) {
        lastPotMs = now;
        lastPotHz = pot.readHz();
    }

    // Winding logic (motor, traverse, auto-stop).
    winder.tick(lastPotHz);

    // UART link: receive commands from display, push status periodically.
    link.poll([](const String& cmd, const String& val) {
        winder.handleCommand(cmd, val);
    });
    if (now - lastLinkMs >= LINK_UPDATE_MS) {
        lastLinkMs = now;
        const WinderStatus s = winder.getStatus();
        link.sendStatus(s.rpm, s.speedHz, s.turns, s.targetTurns,
                        s.running, s.motorEnabled, s.freerun, s.directionCW);
    }

    // WebSocket: push full status snapshot periodically.
    if (now - lastWsMs >= WS_UPDATE_MS) {
        lastWsMs = now;
        web.sendUpdate(winder.getStatus());
    }
}
