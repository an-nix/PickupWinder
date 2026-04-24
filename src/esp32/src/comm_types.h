#pragma once

#include <driver/gpio.h>

struct SpiBusPins {
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
    gpio_num_t ready;
    gpio_num_t home_pin_no;
    gpio_num_t home_pin_nc;
};
