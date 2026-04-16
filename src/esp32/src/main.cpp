/**
 * @file main.cpp
 * @brief ESP32 Klipper-style stepper executor — entry point.
 *
 * Architecture overview
 * ─────────────────────
 *   Core 0  (APP CPU)
 *     • comm_rx task  (pri 10) : UART frame parser → StepperQueue
 *     • demo_gen task (pri  8) : local profile generator (remove in production)
 *     • demo_log task (pri  5) : frequency logger every 100 ms
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
 *   UART host link             : TX=GPIO17    RX=GPIO16   Baud=921600
 *
 * To disable the local demo and use the real UART host:
 *   Comment out the demo_local_start() call below.
 *   The CommInterface will automatically start the UART RX task.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>

#include "step_types.h"
#include "stepper_driver.h"
#include "stepper_queue.h"
#include "comm_interface.h"
#include "demo_local.h"

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

// UART host link (stub — not used when demo_local is active)
static constexpr int UART_NUM   = 1;
static constexpr int UART_TX    = 17;
static constexpr int UART_RX    = 16;
static constexpr int UART_BAUD  = 921600;

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

    // ── 4. Start communication interface (UART RX + flow control) ──────────
    //
    // Pass UART_NUM = -1 to skip UART initialisation when using demo_local.
    // Uncomment the real init() call when the Linux host is connected:
    //
    //   ESP_ERROR_CHECK(comm.init(UART_NUM, UART_TX, UART_RX, UART_BAUD));
    //
    (void)comm; // suppress unused-variable warning in demo mode

    // ── 5. Start local demo (remove in production) ──────────────────────────
    //
    // ╔══════════════════════════════════════════════════════════════════════╗
    // ║  DEMO MODE ACTIVE.  Remove demo_local_start() and uncomment         ║
    // ║  comm.init() above when using the real Raspberry Pi host.           ║
    // ╚══════════════════════════════════════════════════════════════════════╝
    ESP_ERROR_CHECK(demo_local_start(&queue_a, &queue_b));

    // app_main may return — FreeRTOS scheduler continues running the tasks.
    ESP_LOGI(TAG, "Scheduler running — app_main exiting.");
}
