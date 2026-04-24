#include "comm_interface.h"

#include <driver/gpio.h>
#include <esp_check.h>

#include "comm_request_dispatcher.h"
#include "comm_status_builder.h"
#include "multi_axis_executor.h"
#include "spi_slave_transport.h"

static const char* TAG = "comm_iface";

static constexpr uint32_t MULTI_AXIS_QUEUE_DEPTH = 64;
static constexpr uint32_t FLUSH_QUEUE_DEPTH = 4;

CommInterface::CommInterface(StepperQueue* queues[], uint8_t n_motors)
    : n_motors_(n_motors < SPI_MAX_AXES ? n_motors : SPI_MAX_AXES)
{
    for (uint8_t i = 0; i < SPI_MAX_AXES; ++i) {
        queues_[i] = (i < n_motors_) ? queues[i] : nullptr;
    }

    request_dispatcher_ = std::make_unique<CommRequestDispatcher>(*this);
    status_builder_ = std::make_unique<CommStatusBuilder>(*this);
    spi_transport_ = std::make_unique<SpiSlaveTransport>(*request_dispatcher_, *status_builder_);
    multi_axis_executor_ = std::make_unique<MultiAxisExecutor>(*this);
}

CommInterface::~CommInterface() = default;

StepperQueue* CommInterface::queueForAxis(uint8_t axis_id) const
{
    if (axis_id >= n_motors_) {
        return nullptr;
    }
    return queues_[axis_id];
}

bool CommInterface::hasAxis(uint8_t axis_id) const
{
    return queueForAxis(axis_id) != nullptr;
}

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
    if (multi_axis_queue_ == nullptr) {
        multi_axis_queue_ = xQueueCreate(MULTI_AXIS_QUEUE_DEPTH, sizeof(multi_axis_block_t));
        ESP_RETURN_ON_FALSE(multi_axis_queue_ != nullptr, ESP_ERR_NO_MEM, TAG,
                            "failed to create multi-axis queue");
    }

    if (flush_queue_ == nullptr) {
        flush_queue_ = xQueueCreate(FLUSH_QUEUE_DEPTH, sizeof(flush_request_t));
        ESP_RETURN_ON_FALSE(flush_queue_ != nullptr, ESP_ERR_NO_MEM, TAG,
                            "failed to create flush queue");
    }

    return ESP_OK;
}

esp_err_t CommInterface::registerEndstopIsr() const
{
    StepperQueue* lateral_queue = queueForAxis(1);
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
    ESP_RETURN_ON_ERROR(planner_.init(multi_axis_queue_, flush_queue_),
                        TAG, "failed to init motion planner");
    ESP_RETURN_ON_ERROR(multi_axis_executor_->start(),
                        TAG, "failed to start multi-axis executor");
    ESP_RETURN_ON_ERROR(registerEndstopIsr(), TAG, "initEndstopIsr failed");
    ESP_RETURN_ON_ERROR(spi_transport_->init(pins_),
                        TAG, "failed to init SPI transport");
    return ESP_OK;
}

void CommInterface::notifySegmentExecuted(uint16_t motion_seq)
{
    uint16_t current = last_executed_sequence_.load(std::memory_order_relaxed);
    if (sequence_is_newer_u16(motion_seq, current)) {
        last_executed_sequence_.store(motion_seq, std::memory_order_release);
    }
}

uint8_t CommInterface::readLateralEndstopState() const
{
    StepperQueue* lateral_queue = queueForAxis(1);
    if (lateral_queue == nullptr) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }
    return lateral_queue->driver().reportedEndstopState();
}

bool CommInterface::isLateralMovementAllowed(uint8_t axis_id, bool direction) const
{
    if (axis_id != 1) {
        return true;
    }

    StepperQueue* axis_queue = queueForAxis(axis_id);
    if (axis_queue == nullptr) {
        return false;
    }
    return axis_queue->driver().isEndstopMoveAllowed(direction);
}
