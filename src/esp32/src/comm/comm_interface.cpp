/**
 * @file comm_interface.cpp
 * @brief Communication subsystem façade implementation.
 */

#include "comm_interface.h"

#include <driver/gpio.h>
#include <esp_check.h>

#include "comm_request_dispatcher.h"
#include "comm_status_builder.h"
#include "../motion/multi_axis_executor.h"
#include "spi_slave_transport.h"

static const char* TAG = "comm_iface";

static constexpr uint32_t MULTI_AXIS_QUEUE_DEPTH = 64;
static constexpr uint32_t FLUSH_QUEUE_DEPTH = 4;

CommInterface::CommInterface(StepperQueue* queues[], uint8_t n_motors)
    : runtime_(queues, n_motors, planner_)
{
    request_dispatcher_ = std::make_unique<CommRequestDispatcher>(runtime_);
    status_builder_ = std::make_unique<CommStatusBuilder>(runtime_);
    spi_transport_ = std::make_unique<SpiSlaveTransport>(*request_dispatcher_, *status_builder_);
    multi_axis_executor_ = std::make_unique<MultiAxisExecutor>(runtime_);
}

CommInterface::~CommInterface() = default;

esp_err_t CommInterface::configureHomePins() const
{
    if (pins_.home_pin_no == GPIO_NUM_NC || pins_.home_pin_nc == GPIO_NUM_NC) {
        return ESP_OK;
    }

    gpio_config_t home_cfg = {};
    home_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(pins_.home_pin_no))
                           | (1ULL << static_cast<uint32_t>(pins_.home_pin_nc));
    home_cfg.mode = GPIO_MODE_INPUT;
    home_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    home_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    home_cfg.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&home_cfg);
}

esp_err_t CommInterface::createRuntimeQueues()
{
    if (runtime_.multiAxisQueue() == nullptr) {
        QueueHandle_t multi_axis_queue = xQueueCreate(MULTI_AXIS_QUEUE_DEPTH, sizeof(multi_axis_block_t));
        ESP_RETURN_ON_FALSE(multi_axis_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                            "failed to create multi-axis queue");
        runtime_.setMultiAxisQueue(multi_axis_queue);
    }

    if (runtime_.flushQueue() == nullptr) {
        QueueHandle_t flush_queue = xQueueCreate(FLUSH_QUEUE_DEPTH, sizeof(flush_request_t));
        ESP_RETURN_ON_FALSE(flush_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                            "failed to create flush queue");
        runtime_.setFlushQueue(flush_queue);
    }

    return ESP_OK;
}

esp_err_t CommInterface::registerEndstopIsr() const
{
    StepperQueue* lateral_queue = runtime_.queueForAxis(1);
    if (lateral_queue == nullptr) {
        return ESP_OK;
    }

    return lateral_queue->driver().initEndstopIsr(pins_.home_pin_no, pins_.home_pin_nc);
}

esp_err_t CommInterface::init(const SpiBusPins& pins)
{
    pins_ = pins;

    ESP_RETURN_ON_ERROR(configureHomePins(), TAG, "failed to configure home sensor pins");
    ESP_RETURN_ON_ERROR(createRuntimeQueues(), TAG, "failed to create communication queues");
    ESP_RETURN_ON_ERROR(runtime_.planner().init(runtime_.multiAxisQueue(), runtime_.flushQueue()),
                        TAG, "failed to init motion planner");
    ESP_RETURN_ON_ERROR(multi_axis_executor_->start(),
                        TAG, "failed to start multi-axis executor");
    ESP_RETURN_ON_ERROR(registerEndstopIsr(), TAG, "initEndstopIsr failed");
    ESP_RETURN_ON_ERROR(spi_transport_->init(pins_),
                        TAG, "failed to init SPI transport");
    return ESP_OK;
}
