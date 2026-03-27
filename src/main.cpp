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
#include "SessionController.h"
#include "Diag.h"
#include <esp_system.h>

// ── Subsystem instances ───────────────────────────────────────────────────────
static SpeedInput      pot;
static WebInterface    web;
static LinkSerial      serialLink;
static WinderApp       winder;
static SessionController session(winder);

RTC_DATA_ATTR static uint32_t s_bootCount = 0;

static const char* resetReasonStr(esp_reset_reason_t r) {
    switch (r) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
    }
}

// ── Encodeur rotatif — décodage quadrature complet (A+B CHANGE) ───────────────────
// Table d'état quadrature : index = (prevAB << 2) | currAB
// +1 sens horaire, -1 anti-horaire, 0 bruit/rebond
static const int8_t ENC_QEM[16] = { 0,-1, 1, 0,
                                     1, 0, 0,-1,
                                    -1, 0, 0, 1,
                                     0, 1,-1, 0 };
static volatile int32_t encCount   = 0;
static volatile uint8_t encLastAB  = 0;
// Horodatage (µs) de la dernière ISR acceptée — filtre le bruit EMI du moteur.
// Les impulsions step du moteur génèrent du bruit <10 µs ; les transitions
// humaines légitimes sont espacées de ≥ 3 ms (tourne rapide) → seuil : 1000 µs.
static volatile uint32_t encLastUs = 0;

/**
 * @brief Quadrature encoder ISR with debounce and QEM decoding.
 */
void IRAM_ATTR encISR() {
    uint32_t now = micros();
    // Anti-rebond : ignore les transitions plus rapides que ENC_DEBOUNCE_US.
    // Protège contre le bruit EMI des impulsions step du moteur principal.
    if (now - encLastUs < ENC_DEBOUNCE_US) return;
    encLastUs = now;
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
/**
 * @brief Firmware setup entry point.
 */
void setup() {
    Serial.begin(115200);
    ++s_bootCount;
    const esp_reset_reason_t rr = esp_reset_reason();
    Diag::infof("[BOOT] count=%lu reset=%s(%d)",
        (unsigned long)s_bootCount, resetReasonStr(rr), (int)rr);
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
        session.handleCommand(cmd, val);
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
/**
 * @brief Main firmware loop.
 */
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
            // Plafonner le delta par tick pour limiter l'effet d'un burst de
            // bruit résiduel (EMI) qui aurait quand même franchi l'anti-rebond.
            // Un humain tourne rarement plus de 3-4 crans entre deux passages
            // dans le loop (< 1 ms). Un delta > MAX_ENC_DELTA signale du bruit.
            constexpr int32_t MAX_ENC_DELTA = 4;
            if (delta >  MAX_ENC_DELTA) delta =  MAX_ENC_DELTA;
            if (delta < -MAX_ENC_DELTA) delta = -MAX_ENC_DELTA;
            lastEncConsumed = cur;  // consomme TOUT (évite accumulation)
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

    // Winding session controller (source=potentiel, start/pause, etc.).
    float level = (float)lastPotHz / (float)SPEED_HZ_MAX;
    session.setPotLevel(level);
    session.tick();

    // Mode manuel : streaming WebSocket de la position toutes les 50 ms.
    static uint32_t lastCapMs = 0;
    if (winder.isCaptureActive() && now - lastCapMs >= 50) {
        lastCapMs = now;
        float posMm; long capTurns;
        if (winder.getCapturePoint(posMm, capTurns))
            web.sendCapture(now, posMm, capTurns);
    }

    // UART link: receive commands from display, push status periodically.
    serialLink.poll([](const String& cmd, const String& val) {
        session.handleCommand(cmd, val);
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
