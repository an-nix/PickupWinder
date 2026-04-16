/**
 * @file comm_interface.cpp
 * @brief SPI slave communication interface implementation.
 *
 * The ESP32 is the SPI slave. Every transfer is a fixed-size wire frame:
 *
 *   Host TX frame  ──► ESP32 parses and executes request
 *   Host RX frame ◄── ESP32 returns latest status payload
 *
 * Status is therefore naturally pipelined by one SPI transaction, which keeps
 * the slave task simple and deterministic.
 */

#include "comm_interface.h"

#include <string.h>
#include <driver/spi_slave.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "comm_iface";

static constexpr uint32_t  SPI_TASK_STACK  = 4096;
static constexpr UBaseType_t SPI_TASK_PRIO = 10;
static constexpr BaseType_t  SPI_TASK_CORE = 0;

DMA_ATTR static uint8_t s_rx_frame[SPI_FRAME_SIZE];
DMA_ATTR static uint8_t s_tx_frame[SPI_FRAME_SIZE];

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommInterface::CommInterface(StepperQueue* queues[], uint8_t n_motors)
    : n_motors_(n_motors < SPI_MAX_AXES ? n_motors : SPI_MAX_AXES)
{
    for (uint8_t i = 0; i < SPI_MAX_AXES; ++i) {
        queues_[i] = (i < n_motors_) ? queues[i] : nullptr;
    }
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t CommInterface::init(const SpiBusPins& pins)
{
    pins_ = pins;

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = pins_.mosi;
    bus_cfg.miso_io_num = pins_.miso;
    bus_cfg.sclk_io_num = pins_.sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = SPI_FRAME_SIZE;

    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.mode = 0;
    slave_cfg.spics_io_num = pins_.cs;
    slave_cfg.queue_size = 1;
    slave_cfg.flags = 0;
    slave_cfg.post_setup_cb = nullptr;
    slave_cfg.post_trans_cb = nullptr;

    ESP_RETURN_ON_ERROR(
        spi_slave_initialize(SPI3_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO),
        TAG, "spi_slave_initialize failed");

    BaseType_t rc = xTaskCreatePinnedToCore(
        &CommInterface::spiTask,
        "comm_spi",
        SPI_TASK_STACK,
        this,
        SPI_TASK_PRIO,
        nullptr,
        SPI_TASK_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create comm_spi task");

    ESP_LOGI(TAG, "SPI slave ready  MOSI=%d MISO=%d SCLK=%d CS=%d  frame=%uB",
             (int)pins_.mosi, (int)pins_.miso, (int)pins_.sclk, (int)pins_.cs,
             (unsigned)SPI_FRAME_SIZE);
    return ESP_OK;
}

void CommInterface::buildStatusFrame(uint8_t* out_frame) const
{
    spi_message_zero_frame(out_frame);

    auto* header = reinterpret_cast<SpiMessageHeader*>(out_frame);
    auto* payload = reinterpret_cast<StatusPayload*>(out_frame + sizeof(SpiMessageHeader));

    spi_message_init_header(*header,
                            SpiMessageType::STATUS,
                            last_rx_sequence_,
                            sizeof(StatusPayload),
                            0);

    payload->uptime_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            payload->queue_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->available());
            payload->ring_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->driver().ringFreeSlots());
            payload->underrun_count[axis] = queues_[axis]->driver().getUnderrunCount();
            if (queues_[axis]->driver().isStreaming()) {
                payload->running_mask |= static_cast<uint8_t>(1U << axis);
            }
            if (queues_[axis]->driver().isEnabled()) {
                payload->enabled_mask |= static_cast<uint8_t>(1U << axis);
            }
        } else {
            payload->queue_free_slots[axis] = 0;
            payload->ring_free_slots[axis] = 0;
            payload->underrun_count[axis] = 0;
        }
    }
    payload->last_rx_sequence = last_rx_sequence_;
    payload->last_rx_type = last_rx_type_;
    payload->last_result = last_result_;
    payload->protocol_version = SPI_MSG_VERSION;

    spi_message_finalize(out_frame);
}

esp_err_t CommInterface::handleEnableAxis(const EnableAxisPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (payload.enable) {
        queues_[payload.axis_id]->driver().enable();
    } else {
        queues_[payload.axis_id]->driver().disable();
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleEmergencyStop(const EmergencyStopPayload& payload)
{
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < n_motors_; ++axis) {
            if (queues_[axis] != nullptr) {
                queues_[axis]->driver().emergencyStop();
            }
        }
        return ESP_OK;
    }

    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    queues_[payload.axis_id]->driver().emergencyStop();
    return ESP_OK;
}

esp_err_t CommInterface::handleStopAxis(const EmergencyStopPayload& payload)
{
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < n_motors_; ++axis) {
            if (queues_[axis] != nullptr) {
                queues_[axis]->driver().emergencyStop();
            }
        }
        return ESP_OK;
    }

    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    queues_[payload.axis_id]->driver().emergencyStop();
    return ESP_OK;
}

esp_err_t CommInterface::handleDisableAll()
{
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().disable();
        }
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleResetStats()
{
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().resetUnderrunCount();
        }
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleStepBlock(const StepBlockPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload.step_count > STEP_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    motion_block_t block {};
    block.kind = MOTION_BLOCK_KIND_STEP;
    block.payload.step.count = payload.step_count;
    for (uint32_t i = 0; i < block.payload.step.count; ++i) {
        block.payload.step.steps[i].interval_ticks = payload.entries[i].interval_ticks;
        block.payload.step.steps[i].direction = (payload.entries[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
    }

    return queues_[payload.axis_id]->enqueueMotionBlock(block, 0);
}

esp_err_t CommInterface::handleSegmentBlock(const SegmentBlockPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload.segment_count > SEGMENT_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    motion_block_t block {};
    block.kind = MOTION_BLOCK_KIND_SEGMENT;
    block.payload.segment.count = payload.segment_count;
    for (uint32_t i = 0; i < block.payload.segment.count; ++i) {
        block.payload.segment.segments[i].step_count = payload.segments[i].step_count;
        block.payload.segment.segments[i].start_ticks = payload.segments[i].start_ticks;
        block.payload.segment.segments[i].add_ticks = payload.segments[i].add_ticks;
        block.payload.segment.segments[i].direction =
            (payload.segments[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
        block.payload.segment.segments[i].reserved = 0;
    }

    return queues_[payload.axis_id]->enqueueMotionBlock(block, 0);
}

esp_err_t CommInterface::handleFrame(const SpiMessageHeader& header, const uint8_t* payload)
{
    switch (static_cast<SpiMessageType>(header.msg_type)) {
    case SpiMessageType::NOP:
    case SpiMessageType::GET_STATUS:
    case SpiMessageType::PING:
        return ESP_OK;

    case SpiMessageType::ENABLE_AXIS:
        if (header.payload_length != sizeof(EnableAxisPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleEnableAxis(*reinterpret_cast<const EnableAxisPayload*>(payload));

    case SpiMessageType::ESTOP:
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleEmergencyStop(*reinterpret_cast<const EmergencyStopPayload*>(payload));

    case SpiMessageType::STOP_AXIS:
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleStopAxis(*reinterpret_cast<const EmergencyStopPayload*>(payload));

    case SpiMessageType::DISABLE_ALL:
        if (header.payload_length != 0) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleDisableAll();

    case SpiMessageType::RESET_STATS:
        if (header.payload_length != 0) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleResetStats();

    case SpiMessageType::STEP_BLOCK:
        if (header.payload_length != sizeof(StepBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleStepBlock(*reinterpret_cast<const StepBlockPayload*>(payload));

    case SpiMessageType::SEGMENT_BLOCK:
        if (header.payload_length != sizeof(SegmentBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleSegmentBlock(*reinterpret_cast<const SegmentBlockPayload*>(payload));

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

void CommInterface::spiTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);
    ESP_LOGI(TAG, "SPI task started on core %d", xPortGetCoreID());

    for (;;) {
        self->buildStatusFrame(s_tx_frame);

        spi_slave_transaction_t txn = {};
        txn.length = SPI_FRAME_SIZE * 8;
        txn.tx_buffer = s_tx_frame;
        txn.rx_buffer = s_rx_frame;

        esp_err_t err = spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            continue;
        }

        SpiMessageHeader header {};
        memcpy(&header, s_rx_frame, sizeof(SpiMessageHeader));

        if (header.magic != SPI_MSG_MAGIC) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_MAGIC);
            continue;
        }
        if (header.version != SPI_MSG_VERSION) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_VERSION);
            continue;
        }
        if (header.payload_length > SPI_MAX_PAYLOAD_SIZE) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
            continue;
        }
        if (!spi_message_validate(s_rx_frame, header)) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_CRC);
            continue;
        }

        self->last_rx_sequence_ = header.sequence;
        self->last_rx_type_ = header.msg_type;

        const uint8_t* payload = s_rx_frame + sizeof(SpiMessageHeader);
        err = self->handleFrame(header, payload);
        if (err == ESP_OK) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::OK);
        } else if (err == ESP_ERR_TIMEOUT) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::QUEUE_FULL);
        } else if (err == ESP_ERR_INVALID_ARG) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_AXIS);
        } else if (err == ESP_ERR_INVALID_SIZE) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
        } else if (err == ESP_ERR_NOT_SUPPORTED) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::UNKNOWN_TYPE);
        } else {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::INTERNAL_ERROR);
            ESP_LOGW(TAG, "message 0x%02X failed: %s",
                     header.msg_type, esp_err_to_name(err));
        }
    }
}
