/* endstop.cpp — End stop ISR + homing logic implementation.
 */

#include "endstop.h"
#include "stepper_engine.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "endstop";

// ── Emergency endstop ISR ───────────────────────────────────────────────────
// Direct GPIO interrupt → immediate motor halt.  Bypasses all queues.
// This is the fastest possible reaction: ~1 µs from pin change to step stop.

static void IRAM_ATTR endstop_isr_handler(void* arg) {
    uint8_t axis_id = reinterpret_cast<uintptr_t>(arg);
    if (axis_id < NUM_AXES) {
        // Immediate stop via direct axis method
        g_engine.axis(axis_id).emergency_stop();
        g_engine.axis(axis_id).set_endstop_active(true);
    }
}

// ── Init ────────────────────────────────────────────────────────────────────

void endstop_init(const AxisPins pins[], uint8_t num_axes) {
    for (uint8_t i = 0; i < num_axes && i < NUM_AXES; ++i) {
        if (pins[i].endstop_no < 0) continue;

        // GPIO already configured as INPUT_PULLUP by axis.init()
        // Attach interrupt on NO falling edge (NO active = LOW = triggered)
        gpio_set_intr_type(static_cast<gpio_num_t>(pins[i].endstop_no),
                           GPIO_INTR_NEGEDGE);
        gpio_isr_handler_add(static_cast<gpio_num_t>(pins[i].endstop_no),
                             endstop_isr_handler,
                             reinterpret_cast<void*>(static_cast<uintptr_t>(i)));

        ESP_LOGI(TAG, "Endstop ISR axis %d: NO=GPIO%d NC=GPIO%d",
                 i, pins[i].endstop_no,
                 pins[i].endstop_nc >= 0 ? pins[i].endstop_nc : -1);
    }
}

// ── Homing ──────────────────────────────────────────────────────────────────
// MIGRATION: On BBB, homing was managed by PRU0 orchestrator + daemon.
// Here we implement a blocking homing sequence that uses the existing
// axis motion primitives.

bool endstop_home_axis(uint8_t axis_id, uint32_t approach_hz,
                        uint32_t backoff_steps, uint32_t timeout_ms) {
    if (axis_id >= NUM_AXES) return false;

    Axis& ax = g_engine.axis(axis_id);
    if (ax.endstop_active()) {
        // Already on endstop — back off first
        ESP_LOGI(TAG, "Axis %d already on endstop — backing off", axis_id);
        ax.set_direction(false);  // forward (away from endstop)
        ax.move_to(ax.position() + static_cast<int32_t>(backoff_steps),
                   HZ_MIN, approach_hz / 2, backoff_steps / 4);

        uint64_t t0 = esp_timer_get_time();
        while (ax.state() != AxisState::IDLE) {
            if ((uint32_t)((esp_timer_get_time() - t0) / 1000) > timeout_ms) {
                ESP_LOGE(TAG, "Homing backoff timeout for axis %d", axis_id);
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        ax.clear_event();
    }

    // Approach endstop
    ESP_LOGI(TAG, "Axis %d homing: approaching at %lu Hz", axis_id,
             static_cast<unsigned long>(approach_hz));

    ax.set_direction(true);  // reverse (toward endstop)
    ax.set_speed_hz(approach_hz);

    uint64_t t0 = esp_timer_get_time();
    while (!ax.endstop_active()) {
        if ((uint32_t)((esp_timer_get_time() - t0) / 1000) > timeout_ms) {
            ESP_LOGE(TAG, "Homing approach timeout for axis %d", axis_id);
            ax.emergency_stop();
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Endstop hit — axis is already stopped by ISR
    ESP_LOGI(TAG, "Axis %d endstop hit at position %ld",
             axis_id, static_cast<long>(ax.position()));

    // Back off
    ax.clear_event();
    ax.set_direction(false);
    ax.move_to(ax.position() + static_cast<int32_t>(backoff_steps),
               HZ_MIN, approach_hz / 2, backoff_steps / 4);

    t0 = esp_timer_get_time();
    while (ax.state() != AxisState::IDLE) {
        if ((uint32_t)((esp_timer_get_time() - t0) / 1000) > timeout_ms) {
            ESP_LOGE(TAG, "Homing backoff timeout for axis %d", axis_id);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Reset position to 0 (this is the home position)
    ax.reset_position();
    ax.clear_event();

    ESP_LOGI(TAG, "Axis %d homed successfully", axis_id);
    return true;
}
