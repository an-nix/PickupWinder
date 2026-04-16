#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "step_scheduler.h"
#include "rmt_tx.h"
#include "host_sim.h"

static const char *TAG = "stepper_rmt";

void app_main(void)
{
    ESP_LOGI(TAG, "Init");

    step_queue = xQueueCreate(STEP_QUEUE_LEN, sizeof(StepEvent));
    if (!step_queue) {
        ESP_LOGE(TAG, "Queue creation failed");
        return;
    }

    rmt_init_tx();

    xTaskCreate(rmt_scheduler_task, "rmt_scheduler", 4096, NULL, 10, NULL);
    xTaskCreate(host_sim_task, "host_sim", 4096, NULL, 5, NULL);
}
