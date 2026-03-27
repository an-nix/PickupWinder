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
#include "CommandController.h"
#include "Diag.h"
#include "ControlHardware.h"
#include <esp_system.h>

// ── Subsystem instances ───────────────────────────────────────────────────────
static WebInterface    web;
static LinkSerial      serialLink;
static WinderApp       winder;
static SessionController session(winder);
static CommandController cmdController(serialLink, web);
static ControlHardware control(session, winder);

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

// Encoder handled inside `ControlHardware`.

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

    // Initialize control peripherals (potentiometer + encoder)
    control.begin();
    web.begin();   // WiFi + WebSocket (blocks ~2-5 s during association)
    serialLink.begin();  // UART2 liaison vers ESP écran
    winder.begin();

    // Command handling is drained into SessionController::TickInput; listeners removed.
    // Start command dispatcher callbacks
    cmdController.begin();
    web.setRecipeProvider([]() {
        return winder.recipeJson();
    });

    if (web.isConnected()) {
        Diag::infof("→ Web interface: http://%s",
            web.getIP().c_str());
    }
    // Transport handled in main loop (no FreeRTOS task)
}

// ── loop ──────────────────────────────────────────────────────────────────────
/**
 * @brief Main firmware loop.
 */
void loop() {
    const uint32_t now = millis();
 
        // Let ControlHardware handle encoder + pot periodic work and fill TickInput
         SessionController::TickInput ti;
         control.tick(now, ti);

         // UART link: receive commands from display (pushes into CommandController buffer)
         serialLink.poll();

         // Retrieve current commands from CommandController buffer and put into TickInput for SessionController
         cmdController.drain(ti);

         // Run the session controller.
         session.tick(ti);

    // Periodic status publishing: retrieve snapshot from SessionController, send in main
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

