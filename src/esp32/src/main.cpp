/* main.cpp — ESP32 PickupWinder stepper controller entry point (ESP-IDF).
 *
 * Framework: ESP-IDF (no Arduino).  Entry point: app_main().
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
 *                        Replaces the old UI encoder; GPIO34 was used in the
 *                        legacy implementation (now occupied by HX711[0] DOUT).
 *   Manual encoder A:    GPIO0  (strap pin)
 *   Manual encoder B:    GPIO15 (strap pin)
 *
 * Note: GPIO0 and GPIO15 are strapping pins on many boards. Driving the
 * encoder during reset/flash can change boot behavior. Ensure encoder is
 * idle at boot or add hardware pull resistors to force safe boot levels.
 *
 * NOTE: GPIO23 (old HOME_PIN_NO in resources/esp32/Config.h) is now SPI MOSI.
 *       Home sensor NO contact moved to GPIO21.
 * NOTE: UART0 (GPIO1/3) is used for the manual encoder.
 *       The serial console may remain enabled; if so, avoid driving the
 *       encoder during flashing or when the serial bootloader is active.
 */

#include <esp_log.h>
#include <esp_chip_info.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "protocol.h"
#include "command_queue.h"
#include "axis.h"
#include "stepper_engine.h"
#include "spi_slave.h"
#include "endstop.h"
#include "sensor_task.h"

static const char* TAG = "main";

// ── Pin configuration ─────────────────────────────────────────────────────────

static constexpr AxisPins AXIS_PINS[NUM_AXES] = {
    // Axis 0 — Bobbin rotation
    { .step = 26, .dir = 27, .enable = 14, .endstop_no = -1, .endstop_nc = -1 },
    // Axis 1 — Lateral carriage (2-contact home sensor)
    { .step = 32, .dir = 33, .enable = 25, .endstop_no = 21, .endstop_nc = 22 },
    // Axis 2 — Wire tensioner (no dedicated endstop)
    { .step = 16, .dir = 17, .enable =  4, .endstop_no = -1, .endstop_nc = -1 },
};

static constexpr SpiPins SPI_PINS = {
    .mosi = 23, .miso = 19, .sclk = 18, .cs = 5
};

// ── Global command queue (Core 0 → Core 1) ────────────────────────────────────
static CmdQueue g_cmd_queue;

// ── Entry point ───────────────────────────────────────────────────────────────

extern "C" void app_main(void) {
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint32_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32 Controller");
    ESP_LOGI(TAG, "  3-axis, SPI slave, FreeRTOS dual-core");
    ESP_LOGI(TAG, "  Framework: ESP-IDF");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CPU cores: %d  Free heap: %lu bytes", chip.cores, (unsigned long)free_heap);

    // 1. Initialize stepper engine (axes + hardware timers)
    ESP_LOGI(TAG, "Initializing stepper engine...");
    g_engine.init(AXIS_PINS);

    // 2. Initialize endstop GPIO interrupts
    ESP_LOGI(TAG, "Initializing endstop ISRs...");
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    endstop_init(AXIS_PINS, NUM_AXES);

    // 4. Initialize SPI slave
    ESP_LOGI(TAG, "Initializing SPI slave...");
    spi_slave_init(SPI_PINS);

    // 5. Start stepper task on Core 1
    ESP_LOGI(TAG, "Starting stepper task on Core 1...");
    g_engine.start(g_cmd_queue);

    // 6. Start SPI slave task on Core 0
    ESP_LOGI(TAG, "Starting SPI slave task on Core 0...");
    spi_slave_start(g_cmd_queue);


    
    ESP_LOGI(TAG, "Startup complete — waiting for SPI commands from RPi");

    // Periodic status log (every 5 seconds) — replaces Arduino loop().
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        StatusFrame sf = g_engine.get_status();
        SpiStats    ss = spi_get_stats();

        ESP_LOGI(TAG,
                 "uptime=%lus  SPI rx=%lu err=%lu  "
                 "ax0=%ldpos/%uHz  ax1=%ldpos/%uHz  ax2=%ldpos/%uHz  "
                 "pot=%d  enc=%d",
                 (unsigned long)(sf.uptime_ms / 1000),
                 (unsigned long)ss.rx_frames,
                 (unsigned long)ss.rx_crc_errors,
                 (long)sf.axis[0].position, sf.axis[0].current_hz,
                 (long)sf.axis[1].position, sf.axis[1].current_hz,
                 (long)sf.axis[2].position, sf.axis[2].current_hz,
                 sf.pot_raw, sf.encoder_manual);
    }
}

