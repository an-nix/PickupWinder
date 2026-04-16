#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "types.h"

extern QueueHandle_t step_queue;
static constexpr size_t STEP_QUEUE_LEN = 8192;

void rmt_scheduler_task(void *arg);
