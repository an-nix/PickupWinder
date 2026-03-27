// ── main.cpp ──────────────────────────────────────────────────────────────────
// Entry point for the Pickup Winder firmware.
//
// FreeRTOS task architecture:
//   controlTask (Core 1, prio 5): sensor read + command drain + session tick
//   commsTask   (Core 0, prio 2): UART poll + status publishing (UART + WS)
//
// Inter-task communication:
//   Commands:  FreeRTOS queue (comms → control) inside CommandController
//   Status:    mutex-protected WinderStatus snapshot (control → comms)
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ── Subsystem instances ───────────────────────────────────────────────────────
static WebInterface    web;
static LinkSerial      serialLink;
static WinderApp       winder;
static SessionController session(winder);
static CommandController cmdController(serialLink, web);
static ControlHardware control(winder);

// ── Shared status (control → comms) ──────────────────────────────────────────
static SemaphoreHandle_t statusMutex = nullptr;
static WinderStatus      sharedStatus = {};

// ── Task handles ─────────────────────────────────────────────────────────────
static TaskHandle_t controlTaskHandle = nullptr;
static TaskHandle_t commsTaskHandle   = nullptr;

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

// ═════════════════════════════════════════════════════════════════════════════
// controlTask — Core 1, high priority
//   Deterministic 10 ms period: read sensors, drain commands, run session/motor.
// ═════════════════════════════════════════════════════════════════════════════
static void controlTask(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        const uint32_t now = millis();

        // 1. Read sensors (pot + encoder) and fill TickInput
        SessionController::TickInput ti;
        control.tick(now, ti);

        // 2. Drain commands from FreeRTOS queue into TickInput
        cmdController.drain(ti);

        // 3. Run session state machine + motor control
        session.tick(ti);

        // 4. Snapshot status for comms task
        WinderStatus snap = winder.getStatus();
        if (xSemaphoreTake(statusMutex, 0) == pdTRUE) {
            sharedStatus = snap;
            xSemaphoreGive(statusMutex);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// commsTask — Core 0, low priority
//   UART polling, UART status, WebSocket status.
// ═════════════════════════════════════════════════════════════════════════════
static void commsTask(void*) {
    uint32_t lastLinkMs = 0;
    uint32_t lastWsMs   = 0;
    for (;;) {
        const uint32_t now = millis();

        // Poll UART for incoming commands (pushed to queue via callback)
        serialLink.poll();

        // Periodic UART status
        if (now - lastLinkMs >= LINK_UPDATE_MS) {
            lastLinkMs = now;
            WinderStatus s;
            if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                s = sharedStatus;
                xSemaphoreGive(statusMutex);
            }
            serialLink.sendStatus(s.rpm, s.speedHz, s.turns, s.targetTurns,
                                  s.running, s.motorEnabled, s.freerun, s.directionCW);
        }

        // Periodic WebSocket status
        if (now - lastWsMs >= WS_UPDATE_MS) {
            lastWsMs = now;
            WinderStatus s;
            if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                s = sharedStatus;
                xSemaphoreGive(statusMutex);
            }
            web.sendUpdate(s);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    ++s_bootCount;
    const esp_reset_reason_t rr = esp_reset_reason();
    Diag::infof("[BOOT] count=%lu reset=%s(%d)",
        (unsigned long)s_bootCount, resetReasonStr(rr), (int)rr);
    Diag::info("\n=== Pickup Winder (FreeRTOS) ===");

    // Initialize subsystems (all on Arduino core before tasks start)
    control.begin();
    web.begin();
    serialLink.begin();
    winder.begin();
    cmdController.begin();
    web.setRecipeProvider([]() {
        return winder.recipeJson();
    });

    if (web.isConnected()) {
        Diag::infof("→ Web interface: http://%s", web.getIP().c_str());
    }

    // Create inter-task synchronization
    statusMutex = xSemaphoreCreateMutex();

    // Create tasks pinned to specific cores
    xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, 5, &controlTaskHandle, 1);
    xTaskCreatePinnedToCore(commsTask,   "comms",   8192, nullptr, 2, &commsTaskHandle,   0);

    Diag::info("[RTOS] Tasks started — control@core1 comms@core0");
}

// ── loop (idle — all work done in tasks) ──────────────────────────────────────
void loop() {
    vTaskDelay(portMAX_DELAY);
}

