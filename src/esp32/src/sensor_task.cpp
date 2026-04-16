/* sensor_task.cpp — Sensor acquisition task implementation.
 *
 * Core 0, priority 5 (below SPI task at 10 and stepper task at 24).
 * 1 ms tick via vTaskDelayUntil.
 *
 * Loop timing:
 *   Every tick (1 ms)  → HX711 non-blocking check, encoder read
 *   Every 20 ticks     → potentiometer ADC read (~50 Hz)
 *   On pending request → hx711_tare() (blocking, only during machine idle)
 */

#include "sensor_task.h"
#include "hx711.h"
#include "encoder.h"
#include "pot.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

static const char* TAG = "sensor";

// ── Pin configuration ────────────────────────────────────────────────────────
// Defined here so sensor_task.cpp is the single source of truth for
// sensor hardware pins.  main.cpp delegates sensor init to sensor_task_init().

static constexpr Hx711Cfg HX711_CFGS[NUM_HX711] = {
    // [0] Wire tension: SCK=GPIO13, DOUT=GPIO34
    { .sck_pin = 13, .dout_pin = 34, .zero_raw = 0, .scale = 1000.0f },
    // [1] Auxiliary:    SCK=GPIO12, DOUT=GPIO39
    { .sck_pin = 12, .dout_pin = 39, .zero_raw = 0, .scale = 1000.0f },
};

static constexpr EncoderCfg ENC_CFG = {
    .pin_a = 0,   // GPIO0 — strap pin, take care at boot
    .pin_b = 15,  // GPIO15 — strap pin, take care at boot
};

// ── Global sensor state ──────────────────────────────────────────────────────

SensorState g_sensor = {
    .tension_dg       = {0, 0},
    .tension_setpoint = 0,
    .pot_raw          = 0,
    .encoder_manual   = 0,
    .mux              = portMUX_INITIALIZER_UNLOCKED,
};

// ── Async request flags ──────────────────────────────────────────────────────
// Written by any task (atomic uint8_t), consumed by sensor_task.
static constexpr uint8_t NO_TARE = 0xFF;
static volatile uint8_t  s_pending_tare = NO_TARE;  // 0 or 1 = sensor index

// ── Pot read interval counter ────────────────────────────────────────────────
static constexpr uint32_t POT_INTERVAL_TICKS = 20;  // 20 ms at 1 kHz

// ── Init ─────────────────────────────────────────────────────────────────────

void sensor_task_init() {
    hx711_init(HX711_CFGS);
    pot_init();
    encoder_init(ENC_CFG);
    ESP_LOGI(TAG, "Sensor hardware: HX711×2 (GPIO13/34, GPIO12/39), "
             "pot GPIO36, encoder GPIO0/15 (strap pins — ensure safe state at boot)");
}

// ── FreeRTOS task ────────────────────────────────────────────────────────────

static void sensor_task_fn(void* /*param*/) {
    TickType_t last_wake        = xTaskGetTickCount();
    uint32_t   pot_tick_counter = 0;

    ESP_LOGI(TAG, "Sensor task running on Core %d", xPortGetCoreID());

    for (;;) {
        // 1. HX711 non-blocking poll (data arrives at ~80 Hz).
        //    hx711_tick() returns true only when at least one sensor had
        //    fresh data.  We always copy the current readings to g_sensor.
        hx711_tick();

        portENTER_CRITICAL(&g_sensor.mux);
        g_sensor.tension_dg[0] = g_hx711.reading_dg[0];
        g_sensor.tension_dg[1] = g_hx711.reading_dg[1];
        portEXIT_CRITICAL(&g_sensor.mux);

        // 2. Potentiometer ADC — read every POT_INTERVAL_TICKS ms.
        if (++pot_tick_counter >= POT_INTERVAL_TICKS) {
            pot_tick_counter = 0;
            uint16_t raw = pot_read();
            portENTER_CRITICAL(&g_sensor.mux);
            g_sensor.pot_raw = static_cast<int16_t>(raw);
            portEXIT_CRITICAL(&g_sensor.mux);
        }

        // 3. Manual encoder PCNT count — read every tick.
        int16_t enc = encoder_get_count();
        portENTER_CRITICAL(&g_sensor.mux);
        g_sensor.encoder_manual = enc;
        portEXIT_CRITICAL(&g_sensor.mux);

        // 4. Pending tare request (set by stepper_task on TARE_HX711 command).
        //    hx711_tare() is blocking (~100 ms) — only safe when machine is idle.
        uint8_t tare = s_pending_tare;
        if (tare < NUM_HX711) {
            s_pending_tare = NO_TARE;
            hx711_tare(tare);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

void sensor_task_start() {
    xTaskCreatePinnedToCore(
        sensor_task_fn,
        "sensor",
        3072,
        nullptr,
        5,       // lower priority than SPI (10) and stepper (24)
        nullptr,
        0        // Core 0
    );
    ESP_LOGI(TAG, "Sensor task created on Core 0, priority 5");
}

// ── Command callbacks ────────────────────────────────────────────────────────

void sensor_set_tension_setpoint(int16_t dg) {
    portENTER_CRITICAL(&g_sensor.mux);
    g_sensor.tension_setpoint = dg;
    portEXIT_CRITICAL(&g_sensor.mux);
}

void sensor_request_tare(uint8_t sensor_id) {
    if (sensor_id < NUM_HX711) {
        s_pending_tare = sensor_id;   // atomic uint8_t write
    }
}
