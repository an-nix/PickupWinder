#pragma once

#include <driver/gpio.h>
#include <driver/spi_slave.h>
#include <esp_err.h>

#include "comm_types.h"

class CommRequestDispatcher;
class CommStatusBuilder;

class SpiSlaveTransport {
public:
    SpiSlaveTransport(CommRequestDispatcher& dispatcher,
                      const CommStatusBuilder& status_builder);

    esp_err_t init(const SpiBusPins& pins);

private:
    CommRequestDispatcher& dispatcher_;
    const CommStatusBuilder& status_builder_;

    uint8_t* rx_frame_ {nullptr};
    uint8_t* rx_frame_b_ {nullptr};
    uint8_t* tx_frame_a_ {nullptr};
    uint8_t* tx_frame_b_ {nullptr};
    gpio_num_t ready_pin_ {GPIO_NUM_NC};

    static void taskEntry(void* arg);
    void runTask();

    void setReadyPinLevel(bool high) const;
    static void IRAM_ATTR postSetupReadyCb(spi_slave_transaction_t* trans);
    static void IRAM_ATTR postTransReadyCb(spi_slave_transaction_t* trans);
};
