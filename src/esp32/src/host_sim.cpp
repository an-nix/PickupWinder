#include "host_sim.h"
#include "step_scheduler.h"
#include "freertos/task.h"

void host_sim_task(void *arg)
{
    const uint32_t total_steps = 2000;
    const uint32_t accel_steps = 500;
    const uint32_t decel_steps = 500;
    const uint32_t cruise_steps = total_steps - accel_steps - decel_steps;

    const uint32_t start_period = 2000;
    const uint32_t cruise_period = 500;

    for (uint32_t i = 0; i < total_steps; ++i) {
        StepEvent evt;
        evt.flags = 0;
        evt.dir = 1;

        if (i < accel_steps) {
            float t = (float)i / (float)accel_steps;
            evt.duration_us = start_period - (uint32_t)((start_period - cruise_period) * t);
        } else if (i < accel_steps + cruise_steps) {
            evt.duration_us = cruise_period;
        } else {
            uint32_t j = i - (accel_steps + cruise_steps);
            float t = (float)j / (float)decel_steps;
            evt.duration_us = cruise_period + (uint32_t)((start_period - cruise_period) * t);
        }

        evt.flags |= 0x01;
        xQueueSend(step_queue, &evt, portMAX_DELAY);
    }

    vTaskDelete(NULL);
}
