#include "step_scheduler.h"
#include "rmt_tx.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "hardware_config.h"

QueueHandle_t step_queue = nullptr;

static const int BLOCK_SIZE = 64;
static const uint32_t PULSE_US = 10;
static const uint32_t IDLE_DISABLE_MS = 100;
static bool enable_state = false;
static TickType_t last_activity_tick = 0;
static rmt_item32_t bufA[BLOCK_SIZE];
static rmt_item32_t bufB[BLOCK_SIZE];

static inline void enable_driver(bool en)
{
    gpio_set_level(ENABLE_GPIO, en ? 0 : 1);
    enable_state = en;
    if (en) {
        last_activity_tick = xTaskGetTickCount();
    }
}

void rmt_scheduler_task(void *arg)
{
    rmt_item32_t *cur = bufA;
    rmt_item32_t *next = bufB;

    while (1) {
        int idx = 0;
        uint8_t current_dir = 0xFF;
        bool local_enable_forced = false;

        while (idx < BLOCK_SIZE) {
            StepEvent evt;
            if (xQueueReceive(step_queue, &evt, pdMS_TO_TICKS(10)) != pdTRUE) {
                break;
            }

            if (evt.flags & 0x02) {
                enable_driver(false);
                local_enable_forced = true;
            } else if (evt.flags & 0x01) {
                enable_driver(true);
                local_enable_forced = true;
            } else {
                last_activity_tick = xTaskGetTickCount();
            }

            if (current_dir != 0xFF && evt.dir != current_dir && idx > 0) {
                xQueueSendToFront(step_queue, &evt, 0);
                break;
            }

            if (current_dir != evt.dir) {
                current_dir = evt.dir;
                gpio_set_level(DIR_GPIO, current_dir);
            }

            if (append_step_items(cur, BLOCK_SIZE, &idx, evt.duration_us, PULSE_US) < 0) {
                xQueueSendToFront(step_queue, &evt, 0);
                break;
            }

            last_activity_tick = xTaskGetTickCount();
        }

        if (idx == 0) {
            if (enable_state) {
                TickType_t now = xTaskGetTickCount();
                if ((now - last_activity_tick) > pdMS_TO_TICKS(IDLE_DISABLE_MS)) {
                    enable_driver(false);
                    ESP_LOGI("stepper_rmt", "Driver disabled due to idle");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (!enable_state) {
            enable_driver(true);
            ets_delay_us(5);
        }

        rmt_write_items(RMT_CHANNEL_0, cur, idx, false);
        rmt_wait_tx_done(RMT_CHANNEL_0, portMAX_DELAY);

        rmt_item32_t *tmp = cur;
        cur = next;
        next = tmp;
    }
}
