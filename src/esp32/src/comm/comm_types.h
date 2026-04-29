/**
 * @file comm_types.h
 * @brief Shared communication-layer types.
 */

#pragma once

#include <driver/gpio.h>

/**
 * @brief SPI bus and sideband pin assignment used by the communication stack.
 */
struct SpiBusPins {
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
    gpio_num_t ready;
    gpio_num_t home_pin_no;
    gpio_num_t home_pin_nc;
};
