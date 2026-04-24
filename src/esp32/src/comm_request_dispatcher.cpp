#include "comm_request_dispatcher.h"

#include <string.h>
#include <esp_log.h>

#include "comm_interface.h"

static const char* TAG = "comm_dispatch";

CommRequestDispatcher::CommRequestDispatcher(CommInterface& owner)
    : owner_(owner)
{
}

bool CommRequestDispatcher::shouldPublishAckForMessageType(uint8_t msg_type)
{
    switch (static_cast<SpiMessageType>(msg_type)) {
    case SpiMessageType::NOP:
    case SpiMessageType::GET_STATUS:
    case SpiMessageType::PING:
        return false;
    default:
        return true;
    }
}

const CommRequestDispatcher::ProcessedRequestSignature*
CommRequestDispatcher::findCachedRequest(const SpiMessageHeader& header) const
{
    for (const auto& cached : recent_request_cache_) {
        if (cached.valid
            && header.sequence == cached.sequence
            && header.msg_type == cached.msg_type
            && header.payload_length == cached.payload_length
            && header.crc16 == cached.crc) {
            return &cached;
        }
    }
    return nullptr;
}

void CommRequestDispatcher::cacheProcessedRequest(const SpiMessageHeader& header,
                                                  uint8_t result)
{
    auto& cache_slot = recent_request_cache_[recent_request_cache_write_index_];
    cache_slot.valid = true;
    cache_slot.sequence = header.sequence;
    cache_slot.msg_type = header.msg_type;
    cache_slot.payload_length = header.payload_length;
    cache_slot.crc = header.crc16;
    cache_slot.result = result;
    recent_request_cache_write_index_ = static_cast<uint8_t>(
        (recent_request_cache_write_index_ + 1)
        % RECENT_REQUEST_CACHE_DEPTH);
}

void CommRequestDispatcher::publishAck(const SpiMessageHeader& header, uint8_t result)
{
    owner_.last_rx_sequence_ = header.sequence;
    owner_.last_rx_type_ = header.msg_type;
    owner_.last_result_ = result;
}

uint8_t CommRequestDispatcher::mapRequestResult(esp_err_t err)
{
    if (err == ESP_OK) {
        return static_cast<uint8_t>(SpiMessageResult::OK);
    }
    if (err == ESP_ERR_TIMEOUT) {
        return static_cast<uint8_t>(SpiMessageResult::QUEUE_FULL);
    }
    if (err == ESP_ERR_INVALID_ARG) {
        return static_cast<uint8_t>(SpiMessageResult::BAD_AXIS);
    }
    if (err == ESP_ERR_INVALID_SIZE) {
        return static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
    }
    if (err == ESP_ERR_NOT_SUPPORTED) {
        return static_cast<uint8_t>(SpiMessageResult::UNKNOWN_TYPE);
    }
    if (err == ESP_ERR_INVALID_STATE) {
        return static_cast<uint8_t>(SpiMessageResult::ENDSTOP_BLOCKED);
    }
    return static_cast<uint8_t>(SpiMessageResult::INTERNAL_ERROR);
}

uint8_t CommRequestDispatcher::processValidatedRequest(const SpiMessageHeader& header,
                                                       const uint8_t* payload)
{
    const bool publish_ack = shouldPublishAckForMessageType(header.msg_type);
    const ProcessedRequestSignature* cached_request = findCachedRequest(header);
    if (cached_request != nullptr) {
        if (publish_ack) {
            publishAck(header, cached_request->result);
        }
        ESP_LOGD(TAG, "duplicate SPI request seq=%u type=0x%02X ignored",
                 static_cast<unsigned>(header.sequence),
                 static_cast<unsigned>(header.msg_type));
        return cached_request->result;
    }

    const esp_err_t err = handleFrame(header, payload);
    const uint8_t request_result = mapRequestResult(err);
    if (err != ESP_OK && request_result == static_cast<uint8_t>(SpiMessageResult::INTERNAL_ERROR)) {
        ESP_LOGW(TAG, "message 0x%02X failed: %s",
                 header.msg_type, esp_err_to_name(err));
    }

    if (publish_ack) {
        publishAck(header, request_result);
    }
    cacheProcessedRequest(header, request_result);
    return request_result;
}

esp_err_t CommRequestDispatcher::handleEnableAxis(const EnableAxisPayload& payload)
{
    StepperQueue* axis_queue = owner_.queueForAxis(payload.axis_id);
    if (axis_queue == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (payload.enable) {
        axis_queue->driver().enable();
    } else {
        axis_queue->driver().disable();
    }
    return ESP_OK;
}

esp_err_t CommRequestDispatcher::handleEmergencyStop(const EmergencyStopPayload& payload)
{
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < owner_.n_motors_; ++axis) {
            StepperQueue* axis_queue = owner_.queueForAxis(axis);
            if (axis_queue != nullptr) {
                axis_queue->driver().emergencyStop();
            }
        }
        return ESP_OK;
    }

    StepperQueue* axis_queue = owner_.queueForAxis(payload.axis_id);
    if (axis_queue == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    axis_queue->driver().emergencyStop();
    return ESP_OK;
}

esp_err_t CommRequestDispatcher::handleStopAxis(const EmergencyStopPayload& payload)
{
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < owner_.n_motors_; ++axis) {
            StepperQueue* axis_queue = owner_.queueForAxis(axis);
            if (axis_queue != nullptr) {
                axis_queue->gracefulStop();
            }
        }
        return ESP_OK;
    }

    StepperQueue* axis_queue = owner_.queueForAxis(payload.axis_id);
    if (axis_queue == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    axis_queue->gracefulStop();
    return ESP_OK;
}

esp_err_t CommRequestDispatcher::handleDisableAll()
{
    for (uint8_t axis = 0; axis < owner_.n_motors_; ++axis) {
        StepperQueue* axis_queue = owner_.queueForAxis(axis);
        if (axis_queue != nullptr) {
            axis_queue->driver().disable();
        }
    }
    return ESP_OK;
}

esp_err_t CommRequestDispatcher::handleResetStats()
{
    for (uint8_t axis = 0; axis < owner_.n_motors_; ++axis) {
        StepperQueue* axis_queue = owner_.queueForAxis(axis);
        if (axis_queue != nullptr) {
            axis_queue->driver().resetUnderrunCount();
        }
    }
    owner_.planner_.resetStats();
    return ESP_OK;
}

esp_err_t CommRequestDispatcher::handleEnableEndstop(const EnableEndstopPayload& payload)
{
    StepperQueue* axis_queue = owner_.queueForAxis(payload.axis_id);
    if (axis_queue == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    StepperDriver& drv = axis_queue->driver();
    if (payload.arm) {
        drv.armEndstop();
        ESP_LOGI(TAG, "endstop armed on axis %u", payload.axis_id);
    } else {
        drv.disarmEndstop();
        ESP_LOGI(TAG, "endstop disarmed on axis %u", payload.axis_id);
    }
    return ESP_OK;
}

esp_err_t CommRequestDispatcher::handleStepBlock(const StepBlockPayload& payload)
{
    StepperQueue* axis_queue = owner_.queueForAxis(payload.axis_id);
    if (axis_queue == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool direction = (payload.step_count > 0)
        ? ((payload.entries[0].flags & SpiStepFlags::DIR_REVERSE) != 0)
        : false;
    if (!owner_.isLateralMovementAllowed(payload.axis_id, direction)) {
        return ESP_ERR_INVALID_STATE;
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

    return axis_queue->enqueueMotionBlock(block, 0);
}

esp_err_t CommRequestDispatcher::handleSegmentBlock(const SegmentBlockPayload& payload)
{
    StepperQueue* axis_queue = owner_.queueForAxis(payload.axis_id);
    if (axis_queue == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload.segment_count > SEGMENT_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    for (uint32_t i = 0; i < payload.segment_count; ++i) {
        if (payload.segments[i].step_count == 0) {
            continue;
        }
        const bool direction =
            (payload.segments[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
        if (!owner_.isLateralMovementAllowed(payload.axis_id, direction)) {
            return ESP_ERR_INVALID_STATE;
        }
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

    return axis_queue->enqueueMotionBlock(block, 0);
}

esp_err_t CommRequestDispatcher::handleMultiAxisSegmentBlock(const uint8_t* payload,
                                                             uint16_t payload_length)
{
    if (payload_length < sizeof(MultiAxisSegmentBlockHeader)) {
        return ESP_ERR_INVALID_SIZE;
    }

    MultiAxisSegmentBlockHeader hdr_val;
    memcpy(&hdr_val, payload, sizeof(hdr_val));
    const uint8_t axis_count = hdr_val.axis_count;
    const uint8_t segment_count = hdr_val.segment_count;

    if (axis_count == 0 || axis_count > MULTI_AXIS_MAX_AXES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (segment_count == 0 || segment_count > MULTI_AXIS_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t expected_length =
        sizeof(MultiAxisSegmentBlockHeader)
        + static_cast<size_t>(axis_count)
        + static_cast<size_t>(segment_count) * (6u + 2u * axis_count);
    if (payload_length != static_cast<uint16_t>(expected_length)) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (sequence_is_stale_or_equal_u16(hdr_val.block_seq, owner_.last_accepted_block_seq_)) {
        ESP_LOGW(TAG, "drop stale/duplicate block_seq=%u last=%u",
                 static_cast<unsigned>(hdr_val.block_seq),
                 static_cast<unsigned>(owner_.last_accepted_block_seq_));
        return ESP_OK;
    }

    multi_axis_block_t block {};
    block.axis_count = axis_count;
    block.segment_count = segment_count;

    const uint8_t* cursor = payload + sizeof(MultiAxisSegmentBlockHeader);
    for (uint8_t a = 0; a < axis_count; ++a) {
        block.axis_ids[a] = cursor[a];
        if (!owner_.hasAxis(block.axis_ids[a])) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    cursor += axis_count;

    for (uint8_t s = 0; s < segment_count; ++s) {
        uint16_t motion_seq;
        uint16_t duration_us;
        uint16_t dir_mask;
        memcpy(&motion_seq, cursor, 2);
        memcpy(&duration_us, cursor + 2, 2);
        memcpy(&dir_mask, cursor + 4, 2);
        cursor += 6;

        block.segments[s].motion_sequence = motion_seq;
        block.segments[s].duration_us = duration_us;
        block.segments[s].direction_mask = dir_mask;

        for (uint8_t a = 0; a < axis_count; ++a) {
            uint16_t steps;
            memcpy(&steps, cursor, 2);
            block.segments[s].step_counts[a] = steps;
            if (steps > 0) {
                const bool direction = (dir_mask & static_cast<uint16_t>(1U << a)) != 0;
                if (!owner_.isLateralMovementAllowed(block.axis_ids[a], direction)) {
                    return ESP_ERR_INVALID_STATE;
                }
            }
            cursor += 2;
        }
    }

    if (xQueueSend(owner_.multi_axis_queue_, &block, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    owner_.last_accepted_block_seq_ = hdr_val.block_seq;
    return ESP_OK;
}

esp_err_t CommRequestDispatcher::handleFlush(const FlushPayload& flush_payload)
{
    flush_request_t req { .flush_sequence = flush_payload.flush_sequence };
    if (xQueueSend(owner_.flush_queue_, &req, 0) != pdTRUE) {
        ESP_LOGW(TAG, "flush queue full — flush_seq=%u dropped",
                 static_cast<unsigned>(flush_payload.flush_sequence));
        return ESP_ERR_TIMEOUT;
    }
    owner_.last_accepted_block_seq_ = 0xFFFFu;
    return ESP_OK;
}

esp_err_t CommRequestDispatcher::handleFrame(const SpiMessageHeader& header,
                                             const uint8_t* payload)
{
    switch (static_cast<SpiMessageType>(header.msg_type)) {
    case SpiMessageType::NOP:
    case SpiMessageType::GET_STATUS:
    case SpiMessageType::PING:
        return ESP_OK;

    case SpiMessageType::ENABLE_AXIS: {
        if (header.payload_length != sizeof(EnableAxisPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EnableAxisPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEnableAxis(p);
    }

    case SpiMessageType::ESTOP: {
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EmergencyStopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEmergencyStop(p);
    }

    case SpiMessageType::STOP_AXIS: {
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EmergencyStopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleStopAxis(p);
    }

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

    case SpiMessageType::STEP_BLOCK: {
        if (header.payload_length != sizeof(StepBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        StepBlockPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleStepBlock(p);
    }

    case SpiMessageType::SEGMENT_BLOCK: {
        if (header.payload_length != sizeof(SegmentBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        SegmentBlockPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleSegmentBlock(p);
    }

    case SpiMessageType::MULTI_AXIS_SEGMENT_BLOCK:
        return handleMultiAxisSegmentBlock(payload, header.payload_length);

    case SpiMessageType::FLUSH: {
        if (header.payload_length != sizeof(FlushPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        FlushPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleFlush(p);
    }

    case SpiMessageType::ENABLE_ENDSTOP: {
        if (header.payload_length != sizeof(EnableEndstopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EnableEndstopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEnableEndstop(p);
    }

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}
