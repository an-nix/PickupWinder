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
 *                                SCLK=GPIO18  CS=GPIO5  READY=GPIO4
 *
 *   Raspberry Pi control       : SHUTDOWN_REQ=GPIO16
 *
 * The Raspberry Pi demo lives in `src/rpi/` and streams fixed-size SPI
 * message frames to this firmware.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "motion/step_types.h"
#include "motion/stepper_driver.h"
#include "motion/stepper_queue.h"
#include "comm/comm_interface.h"

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
static constexpr gpio_num_t SPI_READY = GPIO_NUM_4;

// Raspberry Pi sideband control
static constexpr gpio_num_t RPI_SHUTDOWN_REQ = GPIO_NUM_16;

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

static constexpr uint32_t RPI_SHUTDOWN_TASK_STACK = 2048;
static constexpr UBaseType_t RPI_SHUTDOWN_TASK_PRIO = 2;
static constexpr BaseType_t RPI_SHUTDOWN_TASK_CORE = 0;

static void rpiShutdownSignalTask(void* arg)
{
    const gpio_num_t shutdown_pin = *static_cast<const gpio_num_t*>(arg);

    gpio_config_t shutdown_cfg = {};
    shutdown_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(shutdown_pin));
    shutdown_cfg.mode = GPIO_MODE_OUTPUT;
    shutdown_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    shutdown_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    shutdown_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&shutdown_cfg));

    // Keep the shutdown request line inactive until a future firmware command
    // explicitly drives it. Host-side monitoring/handling will be added later.
    gpio_set_level(shutdown_pin, 0);
    ESP_LOGI(TAG, "Raspberry Pi shutdown request pin initialized on GPIO%d", (int)shutdown_pin);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

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

    // ── 1. Initialise RMT drivers on Core 1 ─────────────────────────────────
    // RMT interrupts are registered on the core that calls the init function.
    // By doing this on Core 1, we prevent the 40 kHz RMT ISRs from starving
    // the SPI task and its hardware interrupts on Core 0.

    // Hardware direction inversion for the lateral axis (axis 1 / motor_b).
    // Set to true when the physical wiring makes DIR=0 move toward the endstop
    // (i.e. the opposite of the logical convention expected by the host).
    // This must be configured before motor_b.init() so that the initial GPIO
    // level is set correctly. All endstop and direction logic operates on the
    // logical level; the XOR is applied only at the GPIO output stage.
    motor_b.setInvertDirection(true);

    struct InitTask {
        static void run(void*) {
            ESP_ERROR_CHECK(motor_a.init());
            ESP_ERROR_CHECK(motor_b.init());
            vTaskDelete(nullptr);
        }
    };
    xTaskCreatePinnedToCore(InitTask::run, "rmt_init", 4096, nullptr, 20, nullptr, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for core 1 init to complete

    // ── 2. Enable motor drivers ─────────────────────────────────────────────
    //motor_a.enable();
    //motor_b.enable();

    // ── 3. Launch executor tasks (Core 1, priority 24) ──────────────────────
    ESP_ERROR_CHECK(queue_a.init());
    ESP_ERROR_CHECK(queue_b.init());

    // ── 4. Initialize Raspberry Pi sideband outputs ─────────────────────────
    BaseType_t shutdown_task_ok = xTaskCreatePinnedToCore(
        rpiShutdownSignalTask,
        "rpi_shutdown",
        RPI_SHUTDOWN_TASK_STACK,
        (void*)&RPI_SHUTDOWN_REQ,
        RPI_SHUTDOWN_TASK_PRIO,
        nullptr,
        RPI_SHUTDOWN_TASK_CORE);
    ESP_ERROR_CHECK(shutdown_task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    // ── 5. Start SPI communication interface (Core 0, priority 10) ────────
    ESP_ERROR_CHECK(comm.init({SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_CS, SPI_READY, HOME_NO, HOME_NC}));

    // app_main may return — FreeRTOS scheduler continues running the tasks.
    ESP_LOGI(TAG, "Scheduler running — app_main exiting.");
}
