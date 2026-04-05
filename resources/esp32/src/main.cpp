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

#include "WebInterface.h"
#include "LinkSerial.h"
#include "WinderApp.h"
#include "SessionController.h"
#include "CommandController.h"
#include "Diag.h"
#include "ControlHardware.h"
#include "Protocol.h"
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>

// ── Subsystem instances ───────────────────────────────────────────────────────
static WifiManager      wifiManager;
static WebInterface    web;
static LinkSerial      serialLink;
static WinderApp       winder;
static SessionController session(winder);
static CommandController cmdController(serialLink, web);
static ControlHardware control;

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
    uint32_t worstLoopUs = 0;
    uint32_t lastMetricsMs = 0;
    for (;;) {
        const uint32_t loopStartUs = micros();
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

        const uint32_t loopUs = micros() - loopStartUs;
        if (loopUs > worstLoopUs) worstLoopUs = loopUs;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));

        if (now - lastMetricsMs >= 5000) {
            lastMetricsMs = now;
            Diag::infof("[RTOS] control_worst_loop_us=%u", (unsigned)worstLoopUs);
            worstLoopUs = 0;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// commsTask — Core 0, low priority
//   UART polling, UART status, WebSocket status.
// ═════════════════════════════════════════════════════════════════════════════
static void commsTask(void*) {
    uint32_t lastLinkMs = 0;
    uint32_t lastWsMs   = 0;
    uint32_t lastWsCleanMs = 0;
    uint32_t lastMetricsMs = 0;
    uint32_t worstLoopUs = 0;
    for (;;) {
        const uint32_t loopStartUs = micros();
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

        // Periodic WebSocket client cleanup (prevents memory leak).
        if (now - lastWsCleanMs >= 2000) {
            lastWsCleanMs = now;
            web.cleanupClients();
        }

        // Periodic runtime telemetry to validate memory and task headroom.
        if (now - lastMetricsMs >= 5000) {
            lastMetricsMs = now;
            const uint32_t free8 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
            const uint32_t min8  = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
            const UBaseType_t ctrlHwmWords = uxTaskGetStackHighWaterMark(controlTaskHandle);
            const UBaseType_t commHwmWords = uxTaskGetStackHighWaterMark(nullptr);
            const CommandController::Stats cs = cmdController.getStats();
            Diag::infof("[RTOS] heap_free=%u heap_min=%u ctrl_hwm=%uB comm_hwm=%uB comms_worst_loop_us=%u cmd_enq=%u cmd_drop_q=%u cmd_rej_len=%u cmd_rej_schema=%u",
                        (unsigned)free8,
                        (unsigned)min8,
                        (unsigned)(ctrlHwmWords * sizeof(StackType_t)),
                        (unsigned)(commHwmWords * sizeof(StackType_t)),
                        (unsigned)worstLoopUs,
                        (unsigned)cs.enqueued,
                        (unsigned)cs.droppedQueueFull,
                        (unsigned)cs.rejectedOversize,
                        (unsigned)cs.rejectedSchema);
            worstLoopUs = 0;
        }

        const uint32_t loopUs = micros() - loopStartUs;
        if (loopUs > worstLoopUs) worstLoopUs = loopUs;

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
    Diag::infof("[BOOT] protocol ws=%s uart=%s recipe=%u",
        PICKUP_WS_PROTOCOL_VERSION,
        PICKUP_UART_PROTOCOL_VERSION,
        (unsigned)PICKUP_RECIPE_FORMAT_VERSION);
    Diag::info("\n=== Pickup Winder (FreeRTOS) ===");

    // Initialize subsystems (all on Arduino core before tasks start)
    control.begin();
    web.setWifiManager(&wifiManager);
    web.begin();
    serialLink.begin();
    winder.begin();
    cmdController.begin();
    web.setRecipeProvider([](char* buf, size_t len) {
        winder.recipeJson(buf, len);
    });

    if (web.isConnected()) {
        char ipBuf[32];
        web.getIP(ipBuf, sizeof(ipBuf));
        Diag::infof("→ Web interface: http://%s", ipBuf);
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

