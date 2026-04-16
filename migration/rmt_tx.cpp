#include "rmt_tx.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "hardware_config.h"

static const uint32_t MAX_RMT_DUR = 32767U;

void rmt_init_tx()
{
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL_0,
        .gpio_num = STEP_GPIO,
        .clk_div = 80,
        .mem_block_num = 1,
        .tx_config = {
            .loop_en = false,
            .carrier_en = false,
            .idle_output_en = true,
            .idle_level = RMT_IDLE_LEVEL_LOW,
        }
    };
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

int append_step_items(rmt_item32_t *buffer, int buffer_len, int *idx,
                      uint32_t period_us, uint32_t pulse_us)
{
    if (pulse_us < 1) pulse_us = 1;
    if (pulse_us > period_us) pulse_us = period_us;

    uint32_t low_us = period_us - pulse_us;

    if (*idx >= buffer_len) return -1;
    uint32_t first_low = (low_us > MAX_RMT_DUR) ? MAX_RMT_DUR : low_us;

    buffer[*idx].level0 = 1;
    buffer[*idx].duration0 = pulse_us;
    buffer[*idx].level1 = 0;
    buffer[*idx].duration1 = first_low;
    (*idx)++;
    low_us -= first_low;

    while (low_us > 0) {
        if (*idx >= buffer_len) return -1;
        uint32_t chunk = (low_us > MAX_RMT_DUR) ? MAX_RMT_DUR : low_us;
        buffer[*idx].level0 = 0;
        buffer[*idx].duration0 = chunk;
        buffer[*idx].level1 = 0;
        buffer[*idx].duration1 = 1;
        (*idx)++;
        low_us -= chunk;
    }
    return 0;
}
