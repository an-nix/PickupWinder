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
static LinkSerial   serialLink;
static WinderApp    winder;

// ── Encodeur rotatif — décodage quadrature complet (A+B CHANGE) ───────────────────
// Table d'état quadrature : index = (prevAB << 2) | currAB
// +1 sens horaire, -1 anti-horaire, 0 bruit/rebond
static const int8_t ENC_QEM[16] = { 0,-1, 1, 0,
                                     1, 0, 0,-1,
                                    -1, 0, 0, 1,
                                     0, 1,-1, 0 };
static volatile int32_t encCount   = 0;
static volatile uint8_t encLastAB  = 0;

void IRAM_ATTR encISR() {
    uint8_t a = digitalRead(ENC1_CLK);
    uint8_t b = digitalRead(ENC1_DT);
    uint8_t ab = (a << 1) | b;
    encCount += ENC_QEM[(encLastAB << 2) | ab];
    encLastAB = ab;
}

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
    serialLink.begin();  // UART2 liaison vers ESP écran
    winder.begin();

    // Encodeur rotatif
    pinMode(ENC1_CLK, INPUT_PULLUP);
    pinMode(ENC1_DT,  INPUT_PULLUP);
    encLastAB = ((uint8_t)digitalRead(ENC1_CLK) << 1) | (uint8_t)digitalRead(ENC1_DT);
    attachInterrupt(digitalPinToInterrupt(ENC1_CLK), encISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_DT),  encISR, CHANGE);
    Diag::info("[Encoder] ISR quadrature attachée (GPIO " __XSTRING(ENC1_CLK) " + " __XSTRING(ENC1_DT) ")");

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
    // Encodeur rotatif — affichage périodique + réglage des butées en mode verify
    static uint32_t lastEncMs  = 0;
    static int32_t  lastPrinted = 1;
    static int32_t  lastEncConsumed = 0;
    {
        // Snapshot atomique du compteur (lecture 32-bit alignée = atomique sur ESP32)
        int32_t cur = encCount;
        int32_t delta = cur - lastEncConsumed;
        if (delta != 0) {
            lastEncConsumed = cur;
            winder.handleEncoderDelta(delta);
        }
        if (now - lastEncMs >= 50) {
            lastEncMs = now;
            if (cur != lastPrinted) {
                lastPrinted = cur;
                Serial.printf("[Encoder] count = %ld\n", (long)cur);
            }
        }
    }
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
    serialLink.poll([](const String& cmd, const String& val) {
        winder.handleCommand(cmd, val);
    });
    if (now - lastLinkMs >= LINK_UPDATE_MS) {
        lastLinkMs = now;
        const WinderStatus s = winder.getStatus();
        serialLink.sendStatus(s.rpm, s.speedHz, s.turns, s.targetTurns,
                        s.running, s.motorEnabled, s.freerun, s.directionCW);
    }

    // WebSocket: push full status snapshot periodically.
    if (now - lastWsMs >= WS_UPDATE_MS) {
        lastWsMs = now;
        web.sendUpdate(winder.getStatus());
    }
}
