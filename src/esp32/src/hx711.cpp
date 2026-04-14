/* hx711.cpp — HX711 24-bit ADC driver implementation.
 *
 * Framework: ESP-IDF (no Arduino).
 * PID control removed: the Raspberry Pi is responsible for tension PID.
 * This driver only reads the sensors and makes values available for the
 * StatusFrame forwarded to the Pi.
 *
 * Protocol (per HX711 datasheet rev 1.0):
 *   1. Wait for DOUT to go LOW (conversion complete, ~80 Hz at RATE=HIGH).
 *   2. Pulse SCK HIGH → read DOUT → SCK LOW, 24 times (MSB first).
 *   3. One additional SCK pulse → selects Channel A, Gain 128 (25 pulses total).
 *   4. Sign-extend 24-bit two's complement to int32_t.
 *
 * Timing: SCK half-period ≥ 200 ns required.  Direct register writes on ESP32
 * at 240 MHz take ~15–20 ns → well within margin.
 */

#include "hx711.h"
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

static const char* TAG = "hx711";

// ── Singleton state ───────────────────────────────────────────────────────────
Hx711State g_hx711 = {};

static Hx711Cfg s_cfgs[NUM_HX711];
static bool     s_initialized = false;

// ── Fast register helpers ─────────────────────────────────────────────────────

static inline void sck_high(int pin) {
    if (pin < 32) GPIO.out_w1ts = (1U << pin);
    else          GPIO.out1_w1ts.val = (1U << (pin - 32));
}

static inline void sck_low(int pin) {
    if (pin < 32) GPIO.out_w1tc = (1U << pin);
    else          GPIO.out1_w1tc.val = (1U << (pin - 32));
}

static inline int dout_read(int pin) {
    if (pin < 32) return (GPIO.in  >>  pin)        & 1;
    else          return (GPIO.in1.val >> (pin - 32)) & 1;
}

// ── Init ──────────────────────────────────────────────────────────────────────

void hx711_init(const Hx711Cfg cfgs[NUM_HX711]) {
    for (uint8_t i = 0; i < NUM_HX711; ++i) {
        s_cfgs[i] = cfgs[i];

        // SCK → output, idle LOW
        gpio_config_t out_cfg = {};
        out_cfg.pin_bit_mask  = (1ULL << cfgs[i].sck_pin);
        out_cfg.mode          = GPIO_MODE_OUTPUT;
        out_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
        out_cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
        out_cfg.intr_type     = GPIO_INTR_DISABLE;
        gpio_config(&out_cfg);
        sck_low(cfgs[i].sck_pin);

        // DOUT → input, no pull-up (GPIO 34-39 are input-only, no pull support)
        gpio_config_t in_cfg  = {};
        in_cfg.pin_bit_mask   = (1ULL << cfgs[i].dout_pin);
        in_cfg.mode           = GPIO_MODE_INPUT;
        in_cfg.pull_up_en     = GPIO_PULLUP_DISABLE;
        in_cfg.pull_down_en   = GPIO_PULLDOWN_DISABLE;
        in_cfg.intr_type      = GPIO_INTR_DISABLE;
        gpio_config(&in_cfg);

        g_hx711.reading_dg[i] = 0;
        g_hx711.fresh[i]      = false;

        ESP_LOGI(TAG, "HX711[%u] SCK=GPIO%d DOUT=GPIO%d zero=%ld scale=%.1f",
                 i, cfgs[i].sck_pin, cfgs[i].dout_pin,
                 (long)cfgs[i].zero_raw, (double)cfgs[i].scale);
    }
    s_initialized = true;
}

// ── Bitbang read ──────────────────────────────────────────────────────────────

/// Read 24 bits from one HX711.  Must be called when DOUT is already LOW.
/// Leaves sensor in "Channel A, Gain 128" mode (25 total SCK pulses).
static int32_t read_raw(const Hx711Cfg& cfg) {
    uint32_t raw = 0;
    for (int i = 0; i < 24; ++i) {
        sck_high(cfg.sck_pin);
        raw = (raw << 1) | (uint32_t)dout_read(cfg.dout_pin);
        sck_low(cfg.sck_pin);
    }
    // 25th pulse: select Channel A, Gain 128 for next conversion.
    sck_high(cfg.sck_pin);
    sck_low(cfg.sck_pin);

    // Sign-extend 24-bit two's complement to int32_t.
    if (raw & 0x800000U) raw |= 0xFF000000U;
    return static_cast<int32_t>(raw);
}

static int16_t raw_to_dg(int32_t raw, const Hx711Cfg& cfg) {
    if (cfg.scale <= 0.0f) return 0;
    float grams = static_cast<float>(raw - cfg.zero_raw) / cfg.scale;
    float dg    = grams * 10.0f;
    if (dg >  32000.0f) dg =  32000.0f;
    if (dg < -32000.0f) dg = -32000.0f;
    return static_cast<int16_t>(dg);
}

// ── Tick ──────────────────────────────────────────────────────────────────────

bool hx711_tick() {
    if (!s_initialized) return false;

    bool got_new = false;
    for (uint8_t i = 0; i < NUM_HX711; ++i) {
        // Non-blocking: only read when DOUT is already LOW (conversion ready).
        if (dout_read(s_cfgs[i].dout_pin) != 0) continue;

        int32_t raw            = read_raw(s_cfgs[i]);
        g_hx711.reading_dg[i] = raw_to_dg(raw, s_cfgs[i]);
        g_hx711.fresh[i]      = true;
        got_new                = true;
    }
    return got_new;
}

// ── Tare ──────────────────────────────────────────────────────────────────────

void hx711_tare(uint8_t sensor_id) {
    if (sensor_id >= NUM_HX711 || !s_initialized) return;

    ESP_LOGI(TAG, "Taring HX711[%u]...", sensor_id);
    const Hx711Cfg& cfg = s_cfgs[sensor_id];

    // Average 8 readings for a stable tare.
    // Blocking — must be called only when machine is idle (from sensor_task on request).
    int32_t  sum = 0;
    uint64_t t0  = esp_timer_get_time();

    for (int t = 0; t < 8; ++t) {
        // Wait for DOUT to go LOW (conversion ready).
        while (dout_read(cfg.dout_pin) != 0) {
            if (esp_timer_get_time() - t0 > 500000) {  // 500 ms timeout
                ESP_LOGE(TAG, "Tare timeout on HX711[%u]", sensor_id);
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        sum += read_raw(cfg);
        t0 = esp_timer_get_time();
    }

    s_cfgs[sensor_id].zero_raw = sum / 8;
    ESP_LOGI(TAG, "HX711[%u] tared: zero_raw=%ld",
             sensor_id, (long)s_cfgs[sensor_id].zero_raw);
}
