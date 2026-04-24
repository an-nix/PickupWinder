#include "comm_status_builder.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "comm_runtime.h"

static const char* TAG = "comm_status";

CommStatusBuilder::CommStatusBuilder(const CommRuntime& runtime)
    : runtime_(runtime)
{
}

void CommStatusBuilder::buildStatusFrame(uint8_t* out_frame) const
{
    spi_message_zero_frame(out_frame);

    auto* header = reinterpret_cast<SpiMessageHeader*>(out_frame);
    auto* payload = reinterpret_cast<StatusPayload*>(out_frame + sizeof(SpiMessageHeader));

    spi_message_init_header(*header,
                            SpiMessageType::STATUS,
                            runtime_.lastRxSequence(),
                            sizeof(StatusPayload),
                            0);

    populateAxisStatus(*payload);
    populateProtocolStatus(*payload);
    populateEndstopStatus(*payload);
    populatePlannerStatus(*payload);

    spi_message_finalize(out_frame);
}

void CommStatusBuilder::populateAxisStatus(StatusPayload& payload) const
{
    static uint32_t s_last_logged_underrun[SPI_MAX_AXES] = {0, 0, 0, 0};

    const uint32_t maqf = (runtime_.multiAxisQueue() != nullptr)
        ? static_cast<uint32_t>(uxQueueSpacesAvailable(runtime_.multiAxisQueue()))
        : 0u;
    payload.multi_axis_queue_free = static_cast<uint8_t>(maqf < 255u ? maqf : 255u);

    payload.uptime_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        StepperQueue* axis_queue = runtime_.queueForAxis(axis);
        if (axis_queue != nullptr) {
            payload.queue_free_slots[axis] = static_cast<uint16_t>(axis_queue->available());
            payload.ring_free_slots[axis] = static_cast<uint16_t>(axis_queue->driver().ringFreeSlots());
            payload.underrun_count[axis] = axis_queue->driver().getUnderrunCount();
            if (payload.underrun_count[axis] > s_last_logged_underrun[axis]) {
                ESP_LOGW(TAG,
                         "axis %u underrun_count advanced: delta=%lu total=%lu queue_free=%u ring_free=%u multi_axis_free=%u planner_free=%lu last_exec=%u last_planned=%u streaming=%d",
                         static_cast<unsigned>(axis),
                         static_cast<unsigned long>(payload.underrun_count[axis] - s_last_logged_underrun[axis]),
                         static_cast<unsigned long>(payload.underrun_count[axis]),
                         static_cast<unsigned>(payload.queue_free_slots[axis]),
                         static_cast<unsigned>(payload.ring_free_slots[axis]),
                         static_cast<unsigned>(payload.multi_axis_queue_free),
                         static_cast<unsigned long>(runtime_.planner().segmentQueueFree()),
                         static_cast<unsigned>(runtime_.lastExecutedSequence()),
                         static_cast<unsigned>(runtime_.planner().lastPlannedMotionSequence()),
                         static_cast<int>(axis_queue->driver().isStreaming()));
                s_last_logged_underrun[axis] = payload.underrun_count[axis];
            } else if (payload.underrun_count[axis] < s_last_logged_underrun[axis]) {
                s_last_logged_underrun[axis] = payload.underrun_count[axis];
            }
            if (axis_queue->driver().isStreaming()) {
                payload.running_mask |= static_cast<uint8_t>(1U << axis);
            }
            if (axis_queue->driver().isEnabled()) {
                payload.enabled_mask |= static_cast<uint8_t>(1U << axis);
            }
        } else {
            payload.queue_free_slots[axis] = 0;
            payload.ring_free_slots[axis] = 0;
            payload.underrun_count[axis] = 0;
        }
    }
}

void CommStatusBuilder::populateProtocolStatus(StatusPayload& payload) const
{
    payload.last_rx_sequence = runtime_.lastRxSequence();
    payload.last_rx_type = runtime_.lastRxType();
    payload.last_result = runtime_.lastResult();
    payload.protocol_version = SPI_MSG_VERSION;
    payload.lateral_endstop_state = runtime_.readLateralEndstopState();
}

void CommStatusBuilder::populateEndstopStatus(StatusPayload& payload) const
{
    payload.endstop_armed_mask = 0;
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        StepperQueue* axis_queue = runtime_.queueForAxis(axis);
        if (axis_queue != nullptr && axis_queue->driver().isEndstopArmed()) {
            payload.endstop_armed_mask |= static_cast<uint8_t>(1U << axis);
        }
    }

    payload.endstop_hit_mask = 0;
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        StepperQueue* axis_queue = runtime_.queueForAxis(axis);
        if (axis_queue != nullptr && axis_queue->driver().getEndstopHitCount() > 0) {
            payload.endstop_hit_mask |= static_cast<uint8_t>(1U << axis);
        }
    }
}

void CommStatusBuilder::populatePlannerStatus(StatusPayload& payload) const
{
    payload.last_executed_sequence = runtime_.lastExecutedSequence();

    const uint32_t pqf = runtime_.planner().segmentQueueFree();
    payload.planner_queue_free = static_cast<uint8_t>(pqf < 255u ? pqf : 255u);
    payload.last_planned_sequence = runtime_.planner().lastPlannedMotionSequence();
    payload.segments_dropped = static_cast<uint16_t>(runtime_.planner().segmentsDropped() & 0xFFFFu);
}
