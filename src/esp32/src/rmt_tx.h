#pragma once

#include "driver/rmt.h"
#include <stdint.h>

void rmt_init_tx();
int append_step_items(rmt_item32_t *buffer, int buffer_len, int *idx,
                     uint32_t period_us, uint32_t pulse_us);
