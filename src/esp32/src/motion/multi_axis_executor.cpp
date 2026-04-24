/**
 * @file multi_axis_executor.cpp
 * @brief Multi-axis executor state machine implementation.
 */

#include "multi_axis_executor.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../comm/comm_runtime.h"

static const char* TAG = "multi_exec";

static constexpr uint32_t MULTI_EXEC_STACK = 8192;
static constexpr UBaseType_t MULTI_EXEC_PRIO = 20;
static constexpr BaseType_t MULTI_EXEC_CORE = 1;

MultiAxisExecutor::MultiAxisExecutor(CommRuntime& runtime)
    : runtime_(runtime)
{
}

esp_err_t MultiAxisExecutor::start()
{
    BaseType_t rc = xTaskCreatePinnedToCore(
        &MultiAxisExecutor::taskEntry,
        "multi_exec",
        MULTI_EXEC_STACK,
        this,
        MULTI_EXEC_PRIO,
        nullptr,
        MULTI_EXEC_CORE);

    return (rc == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

void MultiAxisExecutor::taskEntry(void* arg)
{
    static_cast<MultiAxisExecutor*>(arg)->run();
}

void MultiAxisExecutor::run()
{
    ESP_LOGI(TAG, "multi-axis executor (state machine) started on core %d",
             xPortGetCoreID());

    {
        TaskHandle_t my_handle = xTaskGetCurrentTaskHandle();
        for (uint8_t a = 0; a < runtime_.motorCount(); ++a) {
            StepperQueue* axis_queue = runtime_.queueForAxis(a);
            if (axis_queue != nullptr) {
                axis_queue->driver().setExecutorTask(my_handle);
            }
        }
    }

    ESP_LOGI(TAG, "multi_exec stack high watermark at start: %u bytes free",
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));

    QueueHandle_t seg_queue = runtime_.planner().segmentQueue();

    constexpr int DEFER_DEPTH = 256;
    int64_t defer_fire_us[DEFER_DEPTH] = {};
    uint32_t defer_seqs[DEFER_DEPTH] = {};
    int defer_head = 0;
    int defer_tail = 0;

    uint8_t active_axis_ids[MULTI_AXIS_MAX_AXES] = {};
    uint8_t active_axis_count = 0;

    planned_segment_t batch[EXEC_BATCH_LIMIT];
    uint32_t batch_count = 0;
    uint32_t batch_index = 0;

    ExecState state = ExecState::IDLE;

    auto fireDeferred = [&]() {
        const int64_t now = esp_timer_get_time();
        while (defer_head != defer_tail) {
            const int idx = defer_head & (DEFER_DEPTH - 1);
            if (now >= defer_fire_us[idx]) {
                runtime_.notifySegmentExecuted(static_cast<uint16_t>(defer_seqs[idx]));
                ++defer_head;
            } else {
                break;
            }
        }
    };

    auto kickStartActiveAxes = [&]() {
        for (uint8_t a = 0; a < active_axis_count; ++a) {
            const uint8_t axis_id = active_axis_ids[a];
            StepperQueue* axis_queue = runtime_.queueForAxis(axis_id);
            if (axis_queue != nullptr) {
                axis_queue->kickStart();
            }
        }
    };

    auto requestPlannerFlush = [&](uint16_t motion_sequence) {
        flush_request_t req { .flush_sequence = motion_sequence };
        // FIX 4: retry flush enqueue before warning.
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (xQueueSend(runtime_.flushQueue(), &req, pdMS_TO_TICKS(1)) == pdTRUE) {
                return;
            }
        }
        ESP_LOGW(TAG, "auto-flush queue full after 3 retries at seq=%u",
                 static_cast<unsigned>(motion_sequence));
    };

    uint32_t wm_iter = 0;
    for (;;) {
        if (++wm_iter % 2000 == 0) {
            ESP_LOGD(TAG, "multi_exec stack watermark: %u bytes free",
                     static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));
        }

        fireDeferred();

        switch (state) {
        case ExecState::IDLE: {
            planned_segment_t seg;
            if (xQueueReceive(seg_queue, &seg, pdMS_TO_TICKS(2)) == pdTRUE) {
                batch[0] = seg;
                batch_count = 1;
                batch_index = 0;
                state = ExecState::FETCH;
            } else {
                kickStartActiveAxes();
                if (defer_head != defer_tail && active_axis_count > 0) {
                    bool all_rings_empty = true;
                    for (uint8_t a = 0; a < active_axis_count && all_rings_empty; ++a) {
                        const uint8_t aid = active_axis_ids[a];
                        StepperQueue* axis_queue = runtime_.queueForAxis(aid);
                        if (axis_queue != nullptr && axis_queue->driver().ringFreeSlots() < STEP_RING_SIZE) {
                            all_rings_empty = false;
                        }
                    }
                    if (all_rings_empty) {
                        while (defer_head != defer_tail) {
                            const int idx = defer_head & (DEFER_DEPTH - 1);
                            runtime_.notifySegmentExecuted(static_cast<uint16_t>(defer_seqs[idx]));
                            ++defer_head;
                        }
                    }
                }
            }
            break;
        }

        case ExecState::FETCH: {
            while (batch_count < EXEC_BATCH_LIMIT) {
                planned_segment_t seg;
                if (xQueueReceive(seg_queue, &seg, 0) != pdTRUE) {
                    break;
                }
                batch[batch_count++] = seg;
            }
            batch_index = 0;
            state = ExecState::DRAIN;
            break;
        }

        case ExecState::DRAIN: {
            const int64_t drain_start = esp_timer_get_time();

            while (batch_index < batch_count) {
                planned_segment_t& seg = batch[batch_index];
                if (seg.is_flush) {
                    state = ExecState::FLUSH;
                    goto exit_drain;
                }

                active_axis_count = seg.axis_count < MULTI_AXIS_MAX_AXES
                    ? seg.axis_count : MULTI_AXIS_MAX_AXES;
                for (uint8_t a = 0; a < active_axis_count; ++a) {
                    active_axis_ids[a] = seg.axis_ids[a];
                }

                uint8_t guarded_axis_ids[MULTI_AXIS_MAX_AXES] = {};
                uint8_t guarded_axis_count = 0;

                auto clearMultiExecFlags = [&]() {
                    for (uint8_t i = 0; i < guarded_axis_count; ++i) {
                        const uint8_t axis_id = guarded_axis_ids[i];
                        StepperQueue* axis_queue = runtime_.queueForAxis(axis_id);
                        if (axis_queue != nullptr) {
                            axis_queue->setMultiExecActive(false);
                        }
                    }
                };

                bool endstop_hit = false;
                for (uint8_t a = 0; a < seg.axis_count && !endstop_hit; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    StepperQueue* axis_queue = runtime_.queueForAxis(axis_id);
                    if (axis_queue == nullptr) {
                        continue;
                    }
                    if (axis_queue->driver().isEndstopActive()) {
                        clearMultiExecFlags();
                        axis_queue->driver().emergencyStop();
                        requestPlannerFlush(seg.motion_sequence);
                        ESP_LOGW(TAG, "endstop on axis %u at seq=%u",
                                 static_cast<unsigned>(axis_id),
                                 static_cast<unsigned>(seg.motion_sequence));
                        endstop_hit = true;
                    }
                }
                if (endstop_hit) {
                    state = ExecState::RECOVERY;
                    goto exit_drain;
                }

                const uint8_t lateral_state = runtime_.readLateralEndstopState();
                const bool lateral_endstop_armed =
                    runtime_.motorCount() > 1
                    && runtime_.queueForAxis(1) != nullptr
                    && runtime_.queueForAxis(1)->driver().isEndstopArmed();

                if (lateral_endstop_armed
                    && lateral_state == static_cast<uint8_t>(LateralEndstopState::ABSENT)) {
                    ESP_LOGW(TAG, "lateral endstop ABSENT while armed at seq=%u — fail-safe stop",
                             static_cast<unsigned>(seg.motion_sequence));
                    clearMultiExecFlags();
                    StepperQueue* lateral_queue = runtime_.queueForAxis(1);
                    if (lateral_queue != nullptr) {
                        lateral_queue->driver().emergencyStop();
                    }
                    requestPlannerFlush(seg.motion_sequence);
                    state = ExecState::RECOVERY;
                    goto exit_drain;
                }

                const bool lateral_blocked =
                    lateral_endstop_armed
                    && lateral_state != static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);

                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    StepperQueue* axis_queue = runtime_.queueForAxis(axis_id);
                    if (axis_queue != nullptr) {
                        axis_queue->setMultiExecActive(true);
                        if (guarded_axis_count < MULTI_AXIS_MAX_AXES) {
                            guarded_axis_ids[guarded_axis_count++] = axis_id;
                        }
                    }
                }

                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    StepperQueue* axis_queue = runtime_.queueForAxis(axis_id);
                    if (axis_queue != nullptr && !axis_queue->driver().isStreaming()) {
                        axis_queue->kickStart();
                    }
                }

                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    StepperQueue* axis_queue = runtime_.queueForAxis(axis_id);
                    if (axis_queue == nullptr || seg.axes[a].step_count == 0) {
                        continue;
                    }
                    if (axis_id == 1 && lateral_blocked
                        && !runtime_.isLateralMovementAllowed(axis_id, seg.axes[a].direction)) {
                        clearMultiExecFlags();
                        ESP_LOGW(TAG, "axis1 blocked while armed at seq=%u",
                                 static_cast<unsigned>(seg.motion_sequence));
                        axis_queue->driver().emergencyStop();
                        requestPlannerFlush(seg.motion_sequence);
                        state = ExecState::RECOVERY;
                        goto exit_drain;
                    }

                    esp_err_t err = axis_queue->executeConstantRateBlock(
                        seg.axes[a].direction,
                        seg.axes[a].step_count,
                        seg.duration_us);

                    if (err == ESP_ERR_INVALID_STATE) {
                        clearMultiExecFlags();
                        ESP_LOGW(TAG, "axis %u endstop mid-seg seq=%u",
                                 static_cast<unsigned>(axis_id),
                                 static_cast<unsigned>(seg.motion_sequence));
                        axis_queue->driver().emergencyStop();
                        requestPlannerFlush(seg.motion_sequence);
                        state = ExecState::RECOVERY;
                        goto exit_drain;
                    } else if (err == ESP_ERR_TIMEOUT) {
                        clearMultiExecFlags();
                        ESP_LOGE(TAG, "axis %u ring timeout at seq=%u — forcing RECOVERY",
                                 static_cast<unsigned>(axis_id),
                                 static_cast<unsigned>(seg.motion_sequence));
                        axis_queue->driver().emergencyStop();
                        requestPlannerFlush(seg.motion_sequence);
                        state = ExecState::RECOVERY;
                        goto exit_drain;
                    } else if (err != ESP_OK) {
                        ESP_LOGW(TAG, "axis %u seg %u: %s",
                                 static_cast<unsigned>(axis_id),
                                 static_cast<unsigned>(seg.motion_sequence),
                                 esp_err_to_name(err));
                    }
                }
                clearMultiExecFlags();

                const int64_t now_us = esp_timer_get_time();
                const int64_t fire_at_us =
                    (seg.scheduled_time_us > now_us ? seg.scheduled_time_us : now_us)
                    + static_cast<int64_t>(seg.duration_us);
                if ((defer_tail - defer_head) < DEFER_DEPTH) {
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = fire_at_us;
                    defer_seqs[idx] = seg.motion_sequence;
                    ++defer_tail;
                } else {
                    const int evict_idx = defer_head & (DEFER_DEPTH - 1);
                    const uint32_t evicted_seq = defer_seqs[evict_idx];
                    runtime_.notifySegmentExecuted(static_cast<uint16_t>(evicted_seq));
                    ++defer_head;
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = fire_at_us;
                    defer_seqs[idx] = seg.motion_sequence;
                    ++defer_tail;
                    ESP_LOGW(TAG, "defer ring full: evicted seq=%u to make room for seq=%u",
                             static_cast<unsigned>(evicted_seq),
                             static_cast<unsigned>(seg.motion_sequence));
                }

                ++batch_index;

                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    kickStartActiveAxes();
                    taskYIELD();
                    goto exit_drain;
                }
            }

            if (batch_index >= batch_count) {
                state = ExecState::RUN;
            }
            break;

        exit_drain:
            break;
        }

        case ExecState::RUN:
            kickStartActiveAxes();
            fireDeferred();
            state = ExecState::IDLE;
            break;

        case ExecState::FLUSH: {
            const planned_segment_t& flush_seg = batch[batch_index];
            defer_head = defer_tail = 0;
            runtime_.notifySegmentExecuted(flush_seg.flush_sequence);
            ESP_LOGI(TAG, "executor flush at seq=%u",
                     static_cast<unsigned>(flush_seg.flush_sequence));
            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }

        case ExecState::RECOVERY: {
            planned_segment_t discard;
            uint32_t drained = 0;
            uint16_t last_drained_seq = 0;
            while (drained < SEGMENT_QUEUE_DEPTH
                   && xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
                if (!discard.is_flush) {
                    last_drained_seq = discard.motion_sequence;
                }
                ++drained;
            }

            defer_head = defer_tail = 0;

            ESP_LOGW(TAG, "recovery: drained %lu remaining segments (last_seq=%u)",
                     static_cast<unsigned long>(drained),
                     static_cast<unsigned>(last_drained_seq));

            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }
        }

        if (state != ExecState::IDLE) {
            taskYIELD();
        }
    }
}
