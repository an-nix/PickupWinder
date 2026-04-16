/* main.cpp — ESP32 PickupWinder stepper controller entry point (Arduino/ESP-IDF hybrid).
 *
 * Framework: Arduino via initArduino() + app_main().
 *
 * Architecture:
 *   Core 0: spi_task    (priority 10) — SPI slave, CmdFrame/StatusFrame
 *   Core 0: sensor_task (priority  5) — HX711 poll, pot ADC, encoder read
 *   Core 1: stepper_task(priority 24) — motor control, endstop poll, timers
 *
 * Pin assignments:
 *
 *   Axis 0 (Bobbin):     STEP=GPIO26,  DIR=GPIO27,  EN=GPIO14
 *   Axis 1 (Lateral):    STEP=GPIO32,  DIR=GPIO33,  EN=GPIO25,
 *                        HOME NO=GPIO21, HOME NC=GPIO22 (2-contact, pull-up)
 *   Axis 2 (Tensioner):  STEP=GPIO16,  DIR=GPIO17,  EN=GPIO4,
 *                        ENDSTOP=GPIO35
 *   SPI:                 MOSI=GPIO23,  MISO=GPIO19,  SCLK=GPIO18, CS=GPIO5
 *   HX711 tension:       SCK=GPIO13,   DOUT=GPIO34 (input-only)
 *   HX711 auxiliary:     SCK=GPIO12,   DOUT=GPIO39 (input-only)
 *   Potentiometer:       GPIO36 (ADC1_CH0, VP, input-only)
 *   Manual encoder A:    GPIO0  (strap pin)
 *   Manual encoder B:    GPIO15 (strap pin)
 *
 * Note: GPIO0 et GPIO15 sont des strapping pins. S'assurer que l'encodeur
 *       est au repos au boot ou ajouter des résistances de pull pour forcer
 *       des niveaux de boot sûrs.
 *
 * NOTE: GPIO23 (ancien HOME_PIN_NO) est maintenant SPI MOSI.
 *       Le contact NO du capteur home a été déplacé sur GPIO21.
 */

#include <Arduino.h>
#include <esp_chip_info.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>

#include "protocol.h"
#include "command_queue.h"
#include "spi_slave.h"

#include "stepper_engine.h"
#include "endstop.h"
#include "sensor_task.h"

static const char* TAG = "main";

// Forward declare ISR counters for debug
extern uint32_t isr_count;
extern uint32_t thr_count;
extern uint32_t end_count;



// ── Global command queue (Core 0 → Core 1) ────────────────────────────────────
static CmdQueue g_cmd_queue;

// ── Entry point ───────────────────────────────────────────────────────────────

extern "C" void app_main(void) {
    initArduino();

    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint32_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

    Serial.begin(115200);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32 Controller");
    ESP_LOGI(TAG, "  3-axis, SPI slave, FreeRTOS dual-core");
    ESP_LOGI(TAG, "  Framework: Arduino/ESP-IDF hybrid");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CPU cores: %d  Free heap: %lu bytes", chip.cores, (unsigned long)free_heap);

    // 1. Initialiser le stepper engine (FastAccelStepper)
    ESP_LOGI(TAG, "Initializing FastAccelStepper engine...");
    g_stepper_engine.init(AXIS_PINS);

    // 2. Initialiser le hardware capteurs (HX711, pot, encodeur)
    ESP_LOGI(TAG, "Initializing sensor hardware...");
    sensor_task_init();

    // 3. Initialiser les interruptions GPIO des endstops
    ESP_LOGI(TAG, "Initializing endstop ISRs...");
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    endstop_init(AXIS_PINS, RMT_NUM_AXES);

    // 4. Initialiser le périphérique SPI slave
    ESP_LOGI(TAG, "Initializing SPI slave...");
    spi_slave_init(SPI_PINS);

    // 5. Démarrer la tâche stepper sur le Core 1
    ESP_LOGI(TAG, "Starting FastAccelStepper task on Core 1...");
    g_stepper_engine.start(g_cmd_queue);

    // 6. Démarrer la tâche SPI slave sur le Core 0
    ESP_LOGI(TAG, "Starting SPI slave task on Core 0...");
    spi_slave_start(g_cmd_queue);

    // 7. Démarrer la tâche d'acquisition capteurs sur le Core 0
    ESP_LOGI(TAG, "Starting sensor task on Core 0...");
    sensor_task_start();

    ESP_LOGI(TAG, "Startup complete — waiting for SPI commands from RPi");

    // Periodic status log (every 5 seconds) — remplace Arduino loop().
    uint32_t last_isr_count = 0, last_thr_count = 0, last_end_count = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        StatusFrame sf = g_stepper_engine.get_status();
        SpiStats    ss = spi_get_stats();

        uint32_t isr_delta = isr_count - last_isr_count;
        uint32_t thr_delta = thr_count - last_thr_count;
        uint32_t end_delta = end_count - last_end_count;
        last_isr_count = isr_count;
        last_thr_count = thr_count;
        last_end_count = end_count;

        ESP_LOGI(TAG,
                 "uptime=%lus  SPI rx=%lu err=%lu  ISR_total=%lu(+%lu) THR=%lu(+%lu) END=%lu(+%lu)  "
                 "ax0=%ldpos/%uHz  ax1=%ldpos/%uHz  ax2=%ldpos/%uHz  "
                 "pot=%d  enc=%d",
                 (unsigned long)(sf.uptime_ms / 1000),
                 (unsigned long)ss.rx_frames,
                 (unsigned long)ss.rx_crc_errors,
                 (unsigned long)isr_count, (unsigned long)isr_delta,
                 (unsigned long)thr_count, (unsigned long)thr_delta,
                 (unsigned long)end_count, (unsigned long)end_delta,
                 (long)sf.axis[0].position, sf.axis[0].current_hz,
                 (long)sf.axis[1].position, sf.axis[1].current_hz,
                 (long)sf.axis[2].position, sf.axis[2].current_hz,
                 sf.pot_raw, sf.encoder_manual);
    }
}