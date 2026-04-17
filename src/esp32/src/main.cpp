/**
 * @file main.cpp
 * @brief ESP32 Klipper-style stepper executor — entry point.
 *
 * Architecture overview
 * ─────────────────────
 *   Core 0  (APP CPU)
 *     • comm_spi task (pri 10) : SPI slave message parser → StepperQueue
 *
 *   Core 1  (PRO CPU)
 *     • stepper_0     (pri 24) : executor for motor A  (RMT channel 0)
 *     • stepper_1     (pri 24) : executor for motor B  (RMT channel 1)
 *
 *   RMT hardware
 *     • Channel 0 → STEP_A_GPIO  (2 MHz, 64-symbol block, trans_queue=1)
 *     • Channel 1 → STEP_B_GPIO  (2 MHz, 64-symbol block, trans_queue=1)
 *     • One balanced 50/50 HIGH/LOW RMT symbol per commanded step
 *
 * Pin assignments
 * ───────────────
 *   Motor A (Bobbin / axis 0)  : STEP=GPIO26  DIR=GPIO27  EN=GPIO14
 *   Motor B (Lateral / axis 1) : STEP=GPIO32  DIR=GPIO33  EN=GPIO25
 *
 *   SPI host link              : MOSI=GPIO23  MISO=GPIO19
 *                                SCLK=GPIO18  CS=GPIO5
 *
 * The Raspberry Pi demo lives in `src/rpi/` and streams fixed-size SPI
 * message frames to this firmware.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "step_types.h"
#include "stepper_driver.h"
#include "stepper_queue.h"
#include "comm_interface.h"

static const char* TAG = "main";

// ---------------------------------------------------------------------------
// Pin definitions — adjust to your board wiring
// ---------------------------------------------------------------------------

// Motor A — Bobbin axis
static constexpr gpio_num_t STEP_A = GPIO_NUM_26;
static constexpr gpio_num_t DIR_A  = GPIO_NUM_27;
static constexpr gpio_num_t EN_A   = GPIO_NUM_14;

// Motor B — Lateral axis
static constexpr gpio_num_t STEP_B = GPIO_NUM_32;
static constexpr gpio_num_t DIR_B  = GPIO_NUM_33;
static constexpr gpio_num_t EN_B   = GPIO_NUM_25;

// SPI host link
static constexpr gpio_num_t SPI_MOSI = GPIO_NUM_23;
static constexpr gpio_num_t SPI_MISO = GPIO_NUM_19;
static constexpr gpio_num_t SPI_SCLK = GPIO_NUM_18;
static constexpr gpio_num_t SPI_CS   = GPIO_NUM_5;

// Lateral home sensor (2-contact)
static constexpr gpio_num_t HOME_NO = GPIO_NUM_21;
static constexpr gpio_num_t HOME_NC = GPIO_NUM_22;

// ---------------------------------------------------------------------------
// Global instances — static storage, constructed once
// ---------------------------------------------------------------------------

static StepperDriver motor_a(STEP_A, DIR_A, EN_A, 0);
static StepperDriver motor_b(STEP_B, DIR_B, EN_B, 1);

static StepperQueue  queue_a(motor_a, 0);
static StepperQueue  queue_b(motor_b, 1);

static StepperQueue* queues[2] = {&queue_a, &queue_b};
static CommInterface comm(queues, 2);

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "PickupWinder — Klipper-style RMT stepper executor");
    ESP_LOGI(TAG, "RMT resolution : %lu Hz  (%lu ns/tick)",
             (unsigned long)RMT_STEP_RESOLUTION_HZ,
             (unsigned long)(1000000000UL / RMT_STEP_RESOLUTION_HZ));
    ESP_LOGI(TAG, "Max step rate  : ~166 kHz  (interval_min = %u ticks = %lu µs)",
             RMT_STEP_MIN_TICKS,
             (unsigned long)(RMT_STEP_MIN_TICKS * 1000000UL / RMT_STEP_RESOLUTION_HZ));
    ESP_LOGI(TAG, "Block size     : %d steps   Queue depth : %d blocks",
             STEP_BLOCK_SIZE, STEPPER_QUEUE_DEPTH);

    // ── 1. Initialise RMT drivers ───────────────────────────────────────────
    ESP_ERROR_CHECK(motor_a.init());
    ESP_ERROR_CHECK(motor_b.init());

    // ── 2. Enable motor drivers ─────────────────────────────────────────────
    motor_a.enable();
    motor_b.enable();

    // ── 3. Launch executor tasks (Core 1, priority 24) ──────────────────────
    ESP_ERROR_CHECK(queue_a.init());
    ESP_ERROR_CHECK(queue_b.init());

    // ── 4. Start SPI communication interface (Core 0, priority 10) ────────
    ESP_ERROR_CHECK(comm.init({SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_CS, HOME_NO, HOME_NC}));

    // app_main may return — FreeRTOS scheduler continues running the tasks.
    ESP_LOGI(TAG, "Scheduler running — app_main exiting.");
}
