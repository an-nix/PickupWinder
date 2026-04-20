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
#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "motion_planner.h"

// Module log tag used by ESP logging macros throughout this file.
static const char* TAG = "comm_iface";

// Task configuration: stack size, priority and pinned core for SPI task.
static constexpr uint32_t  SPI_TASK_STACK  = 4096;
static constexpr UBaseType_t SPI_TASK_PRIO = 24;
static constexpr BaseType_t  SPI_TASK_CORE = 0;

// Task configuration for the multi-axis executor (separate core).
static constexpr uint32_t    MULTI_EXEC_STACK  = 8192;
static constexpr UBaseType_t MULTI_EXEC_PRIO   = 20;
static constexpr BaseType_t  MULTI_EXEC_CORE   = 1;

/**
 * @brief Hard wall-clock budget for one drain-loop sub-slice before the
 *        executor unconditionally yields to the FreeRTOS scheduler.
 *
 * The yield is UNCONDITIONAL — it does NOT depend on ring-fill level.
 * RMT pulse timing is in hardware, so a 1 ms scheduler sleep never
 * introduces step jitter.
 *
 * NOTE: Superseded by EXEC_TIME_BUDGET_US in motion_planner.h for the
 * state-machine executor. Retained for the per-axis executorTask in
 * stepper_queue.cpp which still uses it indirectly.
 */
static constexpr int64_t  YIELD_INTERVAL_US      = 400;

/**
 * @brief Maximum queue entries drained in a single bounded flush loop.
 *
 * Used by handleFlush() in the SPI ingestion path only.
 * The planner and executor have their own bounded drain constants.
 */
static constexpr uint32_t MAX_FLUSH_DRAIN        = 8;

/**
 * @brief Global queue of multi-axis segment blocks fed by the SPI task and
 *        consumed by the multi-axis executor task.
 *
 * Depth is sized to hold ~600 ms of motion at 4 ms/segment.
 */
// Depth for the global multi-axis block queue (holds planned blocks from host).
static constexpr uint32_t MULTI_AXIS_QUEUE_DEPTH = 64;
// Global queue handle for multi-axis blocks (produced by spiTask, consumed by executor).
static QueueHandle_t s_multi_axis_queue  = nullptr;

/**
 * @brief Global queue for flush requests.  Depth 4 is more than enough since
 *        the host can only issue one flush at a time.
 */
// Flush request queue depth and handle (tiny, host issues at most one in-flight flush).
static constexpr uint32_t FLUSH_QUEUE_DEPTH = 4;
static QueueHandle_t s_flush_queue = nullptr;

// DMA-capable frame buffers for SPI transactions (allocated on init()).
static uint8_t* s_rx_frame = nullptr;
static uint8_t* s_tx_frame_a = nullptr;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommInterface::CommInterface(StepperQueue* queues[], uint8_t n_motors)
    : n_motors_(n_motors < SPI_MAX_AXES ? n_motors : SPI_MAX_AXES)
{
    // Copy incoming queue pointers into the fixed-size array, null-filling unused entries.
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
    // Copy pin configuration for later use.
    // Allocate DMA-capable buffers for SPI frames (one RX, one TX buffer).
    if (s_rx_frame == nullptr) {
        s_rx_frame = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_32BIT));
        ESP_RETURN_ON_FALSE(s_rx_frame != nullptr, ESP_ERR_NO_MEM, TAG, "failed to alloc s_rx_frame");
    }
    if (s_tx_frame_a == nullptr) {
        s_tx_frame_a = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_32BIT));
        ESP_RETURN_ON_FALSE(s_tx_frame_a != nullptr, ESP_ERR_NO_MEM, TAG, "failed to alloc s_tx_frame_a");
    }

    // Configure lateral endstop pins if both NO and NC pins are provided.
    if (pins_.home_pin_no != GPIO_NUM_NC && pins_.home_pin_nc != GPIO_NUM_NC) {
        gpio_config_t home_cfg = {};
        home_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(pins_.home_pin_no))
                               | (1ULL << static_cast<uint32_t>(pins_.home_pin_nc));
        home_cfg.mode = GPIO_MODE_INPUT;
        home_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        home_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        home_cfg.intr_type = GPIO_INTR_DISABLE;
        ESP_RETURN_ON_ERROR(gpio_config(&home_cfg), TAG, "failed to configure home sensor pins");
    }

    // Create the global multi-axis segment queue (from SPI producer -> executor consumer).
    s_multi_axis_queue = xQueueCreate(MULTI_AXIS_QUEUE_DEPTH, sizeof(multi_axis_block_t));
    ESP_RETURN_ON_FALSE(s_multi_axis_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create multi-axis queue");

    // Create the global flush request queue (small, host posts flush requests here).
    s_flush_queue = xQueueCreate(FLUSH_QUEUE_DEPTH, sizeof(flush_request_t));
    ESP_RETURN_ON_FALSE(s_flush_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create flush queue");

    // Configure the SPI bus pins and maximum transfer size for the fixed frame.
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

    // Initialize the SPI slave driver with DMA channel auto-selection.
    ESP_RETURN_ON_ERROR(
        spi_slave_initialize(SPI3_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO),
        TAG, "spi_slave_initialize failed");

    // Spawn the SPI handling task pinned to Core 0.
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

    // Initialize the planner which consumes s_multi_axis_queue and watches s_flush_queue.
    ESP_RETURN_ON_ERROR(planner_.init(s_multi_axis_queue, s_flush_queue),
                        TAG, "failed to init motion planner");

    // Spawn the multi-axis executor task pinned to Core 1.
    rc = xTaskCreatePinnedToCore(
        &CommInterface::multiAxisExecutorTask,
        "multi_exec",
        MULTI_EXEC_STACK,
        this,
        MULTI_EXEC_PRIO,
        nullptr,
        MULTI_EXEC_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create multi_exec task");

    // Delegate endstop ISR registration to the lateral axis driver (axis 1).
    // StepperDriver owns endstop_active_ and executor_task_, so the ISR
    // can act without going through CommInterface.
    if (n_motors_ >= 2 && queues_[1] != nullptr) {
        ESP_RETURN_ON_ERROR(
            queues_[1]->driver().initEndstopIsr(pins_.home_pin_no, pins_.home_pin_nc),
            TAG, "initEndstopIsr failed");
    }

    // Log configured pins and frame size once initialization completes.
    ESP_LOGI(TAG, "SPI slave ready  MOSI=%d MISO=%d SCLK=%d CS=%d  frame=%uB",
             (int)pins_.mosi, (int)pins_.miso, (int)pins_.sclk, (int)pins_.cs,
             (unsigned)SPI_FRAME_SIZE);
    return ESP_OK;
}

void CommInterface::buildStatusFrame(uint8_t* out_frame) const
{
    static uint32_t s_last_logged_underrun[SPI_MAX_AXES] = {0, 0, 0, 0};

    // Zero the outgoing frame buffer and prepare header/payload overlays.
    spi_message_zero_frame(out_frame);

    auto* header = reinterpret_cast<SpiMessageHeader*>(out_frame);
    auto* payload = reinterpret_cast<StatusPayload*>(out_frame + sizeof(SpiMessageHeader));

    // Initialize the SpiMessageHeader fields for a STATUS message.

    spi_message_init_header(*header,
                            SpiMessageType::STATUS,
                            last_rx_sequence_,
                            sizeof(StatusPayload),
                            0);

    // Compute multi_axis_queue_free first so the per-axis underrun log can read it.
    {
        const uint32_t maqf = (s_multi_axis_queue != nullptr)
            ? static_cast<uint32_t>(uxQueueSpacesAvailable(s_multi_axis_queue))
            : 0u;
        payload->multi_axis_queue_free = static_cast<uint8_t>(maqf < 255u ? maqf : 255u);
    }

    // Fill basic runtime fields (uptime, per-axis diagnostics, masks).
    payload->uptime_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            payload->queue_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->available());
            payload->ring_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->driver().ringFreeSlots());
            payload->underrun_count[axis] = queues_[axis]->driver().getUnderrunCount();
            if (payload->underrun_count[axis] > s_last_logged_underrun[axis]) {
                ESP_LOGW(TAG,
                         "axis %u underrun_count advanced: delta=%lu total=%lu queue_free=%u ring_free=%u multi_axis_free=%u planner_free=%lu last_exec=%u last_planned=%u streaming=%d",
                         static_cast<unsigned>(axis),
                         static_cast<unsigned long>(payload->underrun_count[axis] - s_last_logged_underrun[axis]),
                         static_cast<unsigned long>(payload->underrun_count[axis]),
                         static_cast<unsigned>(payload->queue_free_slots[axis]),
                         static_cast<unsigned>(payload->ring_free_slots[axis]),
                         static_cast<unsigned>(payload->multi_axis_queue_free),
                         static_cast<unsigned long>(planner_.segmentQueueFree()),
                         static_cast<unsigned>(last_executed_sequence_.load(std::memory_order_relaxed)),
                         static_cast<unsigned>(planner_.lastPlannedMotionSequence()),
                         (queues_[axis] != nullptr ? (int)queues_[axis]->driver().isStreaming() : -1));
                s_last_logged_underrun[axis] = payload->underrun_count[axis];
            } else if (payload->underrun_count[axis] < s_last_logged_underrun[axis]) {
                s_last_logged_underrun[axis] = payload->underrun_count[axis];
            }
            if (queues_[axis]->driver().isStreaming()) {
                payload->running_mask |= static_cast<uint8_t>(1U << axis);
            }
            if (queues_[axis]->driver().isEnabled()) {
                payload->enabled_mask |= static_cast<uint8_t>(1U << axis);
            }
        } else {
            payload->queue_free_slots[axis] = 0;
            payload->ring_free_slots[axis]  = 0;
            payload->underrun_count[axis]   = 0;
        }
    }
    // Copy last RX/result/protocol fields into status payload.
    payload->last_rx_sequence = last_rx_sequence_;
    payload->last_rx_type     = last_rx_type_;
    payload->last_result      = last_result_;
    payload->protocol_version = SPI_MSG_VERSION;
    payload->lateral_endstop_state = readLateralEndstopState();

    payload->endstop_armed_mask = 0;
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            if (queues_[axis]->driver().isEndstopArmed()) {
                payload->endstop_armed_mask |= static_cast<uint8_t>(1U << axis);
            }
        }
    }

    payload->endstop_hit_mask = 0;
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            if (queues_[axis]->driver().getEndstopHitCount() > 0) {
                payload->endstop_hit_mask |= static_cast<uint8_t>(1U << axis);
            }
        }
    }

    // Atomic load of last executed motion sequence (written by Core 1).
    payload->last_executed_sequence = last_executed_sequence_.load(std::memory_order_acquire);

    // Planner lookahead pressure: how many slots are free in segment_queue_.
    const uint32_t pqf = planner_.segmentQueueFree();
    payload->planner_queue_free = static_cast<uint8_t>(pqf < 255u ? pqf : 255u);
    payload->last_planned_sequence = planner_.lastPlannedMotionSequence();
    payload->segments_dropped = static_cast<uint16_t>(planner_.segmentsDropped() & 0xFFFFu);

    // Finalize the frame: compute CRC and pad as needed.
    spi_message_finalize(out_frame);
}

esp_err_t CommInterface::handleEnableAxis(const EnableAxisPayload& payload)
{
    // Validate axis id and presence of a queue for the axis.
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
    // Support broadcast emergency stop when axis_id == 0xFF.
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
    // gracefulStop() marks the driver as stopped but does NOT flush the ring
    // buffer, so the motor decelerates naturally through any remaining queued
    // steps rather than cutting out instantly.
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < n_motors_; ++axis) {
            if (queues_[axis] != nullptr) {
                queues_[axis]->gracefulStop();
            }
        }
        return ESP_OK;
    }

    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    queues_[payload.axis_id]->gracefulStop();
    return ESP_OK;
}

esp_err_t CommInterface::handleDisableAll()
{
    // Disable drivers for all configured axes.
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().disable();
        }
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleResetStats()
{
    // Reset underrun counters on all axis drivers.
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().resetUnderrunCount();
        }
    }
    planner_.resetStats();
    return ESP_OK;
}

esp_err_t CommInterface::handleEnableEndstop(const EnableEndstopPayload& payload)
{
    // Arm or disarm the configured endstop for a specific axis.
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    StepperDriver& drv = queues_[payload.axis_id]->driver();
    if (payload.arm) {
        drv.armEndstop();
        ESP_LOGI(TAG, "endstop armed on axis %u", payload.axis_id);
    } else {
        drv.disarmEndstop();
        ESP_LOGI(TAG, "endstop disarmed on axis %u", payload.axis_id);
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleStepBlock(const StepBlockPayload& payload)
{
    // Convert incoming StepBlockPayload into an internal motion_block_t and enqueue.
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isLateralMovementAllowed(payload.axis_id)) {
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

    return queues_[payload.axis_id]->enqueueMotionBlock(block, 0);
}

esp_err_t CommInterface::handleSegmentBlock(const SegmentBlockPayload& payload)
{
    // Convert incoming SegmentBlockPayload into an internal motion_block_t and enqueue.
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isLateralMovementAllowed(payload.axis_id)) {
        return ESP_ERR_INVALID_STATE;
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

esp_err_t CommInterface::handleMultiAxisSegmentBlock(const uint8_t* payload,
                                                     uint16_t payload_length)
{
    /*
     * Wire layout for MULTI_AXIS_SEGMENT_BLOCK payload:
     *
     *   MultiAxisSegmentBlockHeader   (4 bytes)
     *   uint8_t  axis_ids[axis_count] (axis_count bytes)
     *   For each segment:
     *     uint16_t motion_sequence    (2 bytes)
     *     uint16_t duration_us        (2 bytes)
     *     uint16_t direction_mask     (2 bytes)
     *     uint16_t step_counts[axis_count] (2 * axis_count bytes)
     *
     * Total minimum: 4 + axis_count + segment_count * (6 + 2*axis_count)
     */
    // Basic length validation: must at least contain the header.
    if (payload_length < sizeof(MultiAxisSegmentBlockHeader)) {
        return ESP_ERR_INVALID_SIZE;
    }

    MultiAxisSegmentBlockHeader hdr_val;
    // Copy header out of payload (packed wire format -> local struct).
    memcpy(&hdr_val, payload, sizeof(hdr_val));
    const uint8_t axis_count     = hdr_val.axis_count;
    const uint8_t segment_count  = hdr_val.segment_count;

    // Validate axis_count and segment_count ranges.
    if (axis_count == 0 || axis_count > MULTI_AXIS_MAX_AXES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (segment_count == 0 || segment_count > MULTI_AXIS_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Validate total payload length before reading any further.
    const size_t expected_length =
        sizeof(MultiAxisSegmentBlockHeader)
        + static_cast<size_t>(axis_count)
        + static_cast<size_t>(segment_count) * (6u + 2u * axis_count);
    // Verify the provided payload length matches the expected size computed from header fields.
    if (payload_length != static_cast<uint16_t>(expected_length)) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Deduplicate already-accepted blocks using wrap-aware compare on block_seq.
    if (sequence_is_stale_or_equal_u16(hdr_val.block_seq, last_accepted_block_seq_)) {
        ESP_LOGW(TAG, "drop stale/duplicate block_seq=%u last=%u",
                 static_cast<unsigned>(hdr_val.block_seq),
                 static_cast<unsigned>(last_accepted_block_seq_));
        return ESP_OK;
    }

    // Deserialise.
    multi_axis_block_t block {};
    block.axis_count     = axis_count;
    block.segment_count  = segment_count;

    // Deserialize axis ids and segment entries using a cursor pointer.
    const uint8_t* cursor = payload + sizeof(MultiAxisSegmentBlockHeader);

    // axis_ids
    for (uint8_t a = 0; a < axis_count; ++a) {
        block.axis_ids[a] = cursor[a];
    }
    cursor += axis_count;

    // segments
    // For each segment: read motion_sequence, duration_us, direction_mask, then per-axis step counts.
    for (uint8_t s = 0; s < segment_count; ++s) {
        uint16_t motion_seq, duration_us, dir_mask;
        memcpy(&motion_seq,  cursor,     2);
        memcpy(&duration_us, cursor + 2, 2);
        memcpy(&dir_mask,    cursor + 4, 2);
        cursor += 6;

        block.segments[s].motion_sequence = motion_seq;
        block.segments[s].duration_us     = duration_us;
        block.segments[s].direction_mask  = dir_mask;

        for (uint8_t a = 0; a < axis_count; ++a) {
            uint16_t steps;
            memcpy(&steps, cursor, 2);
            block.segments[s].step_counts[a] = steps;
            cursor += 2;
        }
    }

    // Non-blocking enqueue: return QUEUE_FULL immediately if full.
    // Non-blocking enqueue into the global multi-axis queue; report QUEUE_FULL if unable to enqueue.
    if (xQueueSend(s_multi_axis_queue, &block, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT; // maps to QUEUE_FULL result code
    }
    last_accepted_block_seq_ = hdr_val.block_seq;
    return ESP_OK;
}

esp_err_t CommInterface::handleFlush(const FlushPayload& flush_payload)
{
    /*
     * Post a flush_request_t to the flush queue.  The executor task
     * watches this queue and applies the flush before processing the next
     * segment.  Using a queue (instead of an atomic variable) ensures that
     * a flush posted just before new segments arrive is always processed in
     * the correct order.
     */
    // Post a flush request structure to the flush queue for the executor to consume.
    flush_request_t req { .flush_sequence = flush_payload.flush_sequence };
    if (xQueueSend(s_flush_queue, &req, 0) != pdTRUE) {
        // Flush queue full — this should never happen in normal operation.
        ESP_LOGW(TAG, "flush queue full — flush_seq=%u dropped",
                 (unsigned)flush_payload.flush_sequence);
        return ESP_ERR_TIMEOUT;
    }
    // Flush sequence is a motion_sequence, not a block_seq.
    // Reset block deduplication state so the next incoming block is accepted.
    last_accepted_block_seq_ = 0xFFFFu;
    return ESP_OK;
}

void CommInterface::notifySegmentExecuted(uint16_t motion_seq)
{
    /*
     * Called by the executor task (Core 1) after each multi-axis segment
     * completes.  Updates last_executed_sequence_ with an atomic store
     * so the SPI task (Core 0) can safely read it in buildStatusFrame().
     *
     * Only advances the sequence — never moves it backward.  This handles
     * the 16-bit wrap-around case correctly because we only call this in
     * strict execution order.
     */
    // Atomically advance last_executed_sequence_ if the provided sequence is newer.
    uint16_t current = last_executed_sequence_.load(std::memory_order_relaxed);
    if (sequence_is_newer_u16(motion_seq, current)) {
        last_executed_sequence_.store(motion_seq, std::memory_order_release);
    }
}

uint8_t CommInterface::readLateralEndstopState() const
{
    // Read lateral endstop NO/NC pins and return the encoded LateralEndstopState.
    if (pins_.home_pin_no == GPIO_NUM_NC || pins_.home_pin_nc == GPIO_NUM_NC) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }

    const int no_state = gpio_get_level(pins_.home_pin_no);
    const int nc_state = gpio_get_level(pins_.home_pin_nc);

    if (no_state == nc_state) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }
    if (no_state == 0 && nc_state == 1) {
        return static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED);
    }
    return static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

bool CommInterface::isLateralMovementAllowed(uint8_t axis_id) const
{
    // Only axis 1 (lateral) is gated by the endstop; other axes are always allowed.
    if (axis_id != 1) {
        return true;
    }
    // Backoff after a homing hit must be allowed while the physical contact is
    // still closed, provided the host has explicitly DISARMED the endstop.
    // The passive gate therefore applies only when the firmware protection is armed.
    if (axis_id >= n_motors_ || queues_[axis_id] == nullptr) {
        return false;
    }
    if (!queues_[axis_id]->driver().isEndstopArmed()) {
        return true;
    }
    return readLateralEndstopState() == static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

esp_err_t CommInterface::handleFrame(const SpiMessageHeader& header, const uint8_t* payload)
{
    // Dispatch incoming requests by message type; simple size checks performed per-case.
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
        // Variable-length payload — pass raw buffer + length.
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

void CommInterface::spiTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);
    ESP_LOGI(TAG, "SPI task started on core %d", xPortGetCoreID());

    // ── Simple spi_slave_transmit() loop ─────────────────────────────────────
    //
    // Each iteration:
    //   1. buildStatusFrame(tx)  — pre-build response reflecting last result
    //   2. spi_slave_transmit()  — atomic queue+wait, blocks until master clocks
    //   3. handleFrame(rx)       — parse and execute received request
    //
    // There is a brief window between transmit() returning and the next call
    // where no transaction is queued.  If the Pi sends during that window,
    // MISO outputs zeros and MOSI is discarded.  The Python retry logic
    // handles this gracefully (typically <2% of transfers at 4 MHz).
    //
    // The previous ping-pong pre-queue approach (queue_trans/get_trans_result
    // with queue_size=2) caused a 1-byte DMA shift on ~5% of responses,
    // producing "bad magic: 0x0150" (valid frame missing the first byte).

    spi_slave_transaction_t txn = {};

    // Runtime diagnostics (rate-limited).
    uint32_t diag_cycles = 0;
    uint32_t diag_bad_magic = 0;
    uint32_t diag_bad_magic_zero = 0;
    uint32_t diag_bad_crc = 0;
    uint32_t diag_ok = 0;
    int64_t diag_last_log_us = esp_timer_get_time();

    // Pre-build the very first status frame before entering the loop so that
    // the top of the loop can call spi_slave_transmit() immediately with
    // minimal gap.
    self->buildStatusFrame(s_tx_frame_a);

    for (;;) {
        // ── Step 1: transmit immediately (status already pre-built) ───────────
        txn.length    = SPI_FRAME_SIZE * 8;
        txn.tx_buffer = s_tx_frame_a;
        txn.rx_buffer = s_rx_frame;
        esp_err_t err = spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            continue;
        }

        ++diag_cycles;

        // ── Step 3: parse and handle the received frame ───────────────────────
        SpiMessageHeader header {};
        memcpy(&header, s_rx_frame, sizeof(SpiMessageHeader));

        if (header.magic != SPI_MSG_MAGIC) {
            ++diag_bad_magic;
            if (s_rx_frame[0] == 0x00 && s_rx_frame[1] == 0x00) {
                ++diag_bad_magic_zero;
            }
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_MAGIC);
        } else if (header.version != SPI_MSG_VERSION) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_VERSION);
        } else if (header.payload_length > SPI_MAX_PAYLOAD_SIZE) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
        } else if (!spi_message_validate(s_rx_frame, header)) {
            ++diag_bad_crc;
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_CRC);
        } else {
            ++diag_ok;
            self->last_rx_sequence_ = header.sequence;
            self->last_rx_type_     = header.msg_type;

            // Detect exact duplicate retries by matching sequence/type/length/crc
            // against a small cache of recently processed requests.
            const CommInterface::ProcessedRequestSignature* cached_request = nullptr;
            for (const auto& cached : self->recent_request_cache_) {
                if (cached.valid
                    && header.sequence == cached.sequence
                    && header.msg_type == cached.msg_type
                    && header.payload_length == cached.payload_length
                    && header.crc16 == cached.crc) {
                    cached_request = &cached;
                    break;
                }
            }

            if (cached_request != nullptr) {
                // Reuse the previous result for this exact request signature.
                self->last_result_ = cached_request->result;
                ESP_LOGD(TAG, "duplicate SPI request seq=%u type=0x%02X ignored",
                         static_cast<unsigned>(header.sequence),
                         static_cast<unsigned>(header.msg_type));
            } else {
                const uint8_t* payload = s_rx_frame + sizeof(SpiMessageHeader);
                err = self->handleFrame(header, payload);
                // Map esp_err_t handler return codes to SpiMessageResult values for status reporting.
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
                } else if (err == ESP_ERR_INVALID_STATE) {
                    self->last_result_ = static_cast<uint8_t>(SpiMessageResult::ENDSTOP_BLOCKED);
                } else {
                    self->last_result_ = static_cast<uint8_t>(SpiMessageResult::INTERNAL_ERROR);
                    ESP_LOGW(TAG, "message 0x%02X failed: %s",
                             header.msg_type, esp_err_to_name(err));
                }

                auto& cache_slot =
                    self->recent_request_cache_[self->recent_request_cache_write_index_];
                cache_slot.valid = true;
                cache_slot.sequence = header.sequence;
                cache_slot.msg_type = header.msg_type;
                cache_slot.payload_length = header.payload_length;
                cache_slot.crc = header.crc16;
                cache_slot.result = self->last_result_;
                self->recent_request_cache_write_index_ = static_cast<uint8_t>(
                    (self->recent_request_cache_write_index_ + 1)
                    % RECENT_REQUEST_CACHE_DEPTH);
            }
        }

        // 1 Hz diagnostic line (only if something noteworthy happened).
        const int64_t now_us = esp_timer_get_time();
        if ((now_us - diag_last_log_us) >= 1000000) {
            if (diag_bad_magic || diag_bad_crc) {
                ESP_LOGW(TAG,
                         "spi diag: cyc=%lu ok=%lu bad_magic=%lu(b0=%lu) bad_crc=%lu",
                         (unsigned long)diag_cycles,
                         (unsigned long)diag_ok,
                         (unsigned long)diag_bad_magic,
                         (unsigned long)diag_bad_magic_zero,
                         (unsigned long)diag_bad_crc);
            }
            diag_cycles = 0;
            diag_bad_magic = 0;
            diag_bad_magic_zero = 0;
            diag_bad_crc = 0;
            diag_ok = 0;
            diag_last_log_us = now_us;
        }

        // Pre-build the next status frame to minimize the window where MISO is all-zero.
        // This reduces a race where the master might sample an all-zero MISO if it clocks
        // the bus during the small gap between transactions.
        self->buildStatusFrame(s_tx_frame_a);
        
        // Tiny delay to ensure CPU caches flush into DMA-capable RAM before the next transaction.
        // Prevents a 1-byte FIFO alignment glitch on rapid back-to-back transfers.
        esp_rom_delay_us(2);
    }
}

// ---------------------------------------------------------------------------
// multiAxisExecutorTask()  — Core 1, priority 20 — STATE MACHINE
// ---------------------------------------------------------------------------
//
// Refactored from a monolithic nested-loop drain pattern into a bounded
// state machine.  Each state transition does bounded work (≤ EXEC_TIME_BUDGET_US
// or ≤ EXEC_BATCH_LIMIT segments) then yields to the scheduler.
//
//   IDLE ──► FETCH ──► DRAIN ──► RUN ──► IDLE
//              │                          ▲
//              ├── (flush sentinel) ──► FLUSH ──┘
//              └── (error/underrun) ──► RECOVERY ──┘
//
// Watchdog safety: NO state executes for more than ~300 µs without exiting
// to the for(;;) top-level loop which naturally yields via xQueueReceive
// or explicit vTaskDelay(1).

// Multi-axis executor: state-machine that drains planned segments and writes
// per-axis constant-rate blocks into each StepperQueue without starting RMT.
void CommInterface::multiAxisExecutorTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);

    ESP_LOGI(TAG, "multi-axis executor (state machine) started on core %d",
             xPortGetCoreID());

    // Register this task for ISR ring-space wakeups on all drivers.
    {
        TaskHandle_t my_handle = xTaskGetCurrentTaskHandle();
        for (uint8_t a = 0; a < self->n_motors_; ++a) {
            if (self->queues_[a] != nullptr) {
                self->queues_[a]->driver().setExecutorTask(my_handle);
            }
        }
    }

    ESP_LOGI(TAG, "multi_exec stack high watermark at start: %u bytes free",
             (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));

    // ── Segment queue handle from the planner ─────────────────────────────
    QueueHandle_t seg_queue = self->planner_.segmentQueue();

    // ── Deferred notification ring ────────────────────────────────────────
    constexpr int DEFER_DEPTH = 256;
    int64_t  defer_fire_us[DEFER_DEPTH] = {};
    uint32_t defer_seqs[DEFER_DEPTH] = {};
    int      defer_head = 0;
    int      defer_tail = 0;

    // ── Active axis tracking (persists across iterations for recovery) ────
    uint8_t active_axis_ids[MULTI_AXIS_MAX_AXES] = {};
    uint8_t active_axis_count = 0;

    // ── Batch buffer for FETCH state ──────────────────────────────────────
    planned_segment_t batch[EXEC_BATCH_LIMIT];
    uint32_t batch_count = 0;
    uint32_t batch_index = 0;

    // ── State machine ─────────────────────────────────────────────────────
    ExecState state = ExecState::IDLE;

    // Lambda: fire all due deferred notifications.
    auto fireDeferred = [&]() {
        const int64_t now = esp_timer_get_time();
        while (defer_head != defer_tail) {
            const int idx = defer_head & (DEFER_DEPTH - 1);
            if (now >= defer_fire_us[idx]) {
                self->notifySegmentExecuted(
                    static_cast<uint16_t>(defer_seqs[idx]));
                ++defer_head;
            } else {
                break;
            }
        }
    };

    // Lambda: kickStart all active axes.
    auto kickStartActiveAxes = [&]() {
        for (uint8_t a = 0; a < active_axis_count; ++a) {
            const uint8_t axis_id = active_axis_ids[a];
            if (axis_id < self->n_motors_ && self->queues_[axis_id] != nullptr) {
                self->queues_[axis_id]->kickStart();
            }
        }
    };

    // ── Main loop ─────────────────────────────────────────────────────────
    uint32_t wm_iter = 0;
    for (;;) {
        if (++wm_iter % 2000 == 0) {
            ESP_LOGD(TAG, "multi_exec stack watermark: %u bytes free",
                     (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));
        }

        // Always fire due deferred notifications at top of loop.
        fireDeferred();

        switch (state) {

        // ══════════════════════════════════════════════════════════════════
        // IDLE: wait for segments from the planner
        // ══════════════════════════════════════════════════════════════════
        case ExecState::IDLE: {
            // Non-blocking receive: coast-mode keeps RMT alive during gaps,
            // so we must refill the ring ASAP. A blocking wait would delay
            // step delivery and cause coast-mode pauses at low speed.
            planned_segment_t seg;
            if (xQueueReceive(seg_queue, &seg, 0) == pdTRUE) {
                batch[0]    = seg;
                batch_count = 1;
                batch_index = 0;
                state = ExecState::FETCH;
            } else {
                // Timeout — kick-start any stalled axes (RMT underrun
                // while we were blocked on xQueueReceive).
                kickStartActiveAxes();
            }
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // FETCH: non-blocking batch fill up to EXEC_BATCH_LIMIT
        // ══════════════════════════════════════════════════════════════════
        case ExecState::FETCH: {
            // Fill remaining batch slots non-blocking.
            while (batch_count < EXEC_BATCH_LIMIT) {
                planned_segment_t seg;
                if (xQueueReceive(seg_queue, &seg, 0) != pdTRUE) break;
                batch[batch_count++] = seg;
            }
            batch_index = 0;
            state = ExecState::DRAIN;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // DRAIN: process batch segments — write steps to RMT ring
        // ══════════════════════════════════════════════════════════════════
        case ExecState::DRAIN: {
            const int64_t drain_start = esp_timer_get_time();

            while (batch_index < batch_count) {

                planned_segment_t& seg = batch[batch_index];

                // ── Flush sentinel ────────────────────────────────────────
                if (seg.is_flush) {
                    state = ExecState::FLUSH;
                    goto exit_drain;  // break out of DRAIN, handle in FLUSH
                }

                // ── Update active axis list ───────────────────────────────
                active_axis_count = seg.axis_count < MULTI_AXIS_MAX_AXES
                    ? seg.axis_count : MULTI_AXIS_MAX_AXES;
                for (uint8_t a = 0; a < active_axis_count; ++a) {
                    active_axis_ids[a] = seg.axis_ids[a];
                }

                // ── Declare guarded axes + clearMultiExecFlags lambda ───────────
                // Declared here (before endstop check) so the lambda is callable
                // on any RECOVERY goto path.  At this point guarded_axis_count=0
                // because setMultiExecActive has not been called yet; calling
                // clearMultiExecFlags() is therefore a no-op on the early endstop
                // path, but the explicit call documents intent and is safe for
                // future code that may set flags earlier.  (R10)
                uint8_t guarded_axis_ids[MULTI_AXIS_MAX_AXES] = {};
                uint8_t guarded_axis_count = 0;

                auto clearMultiExecFlags = [&]() {
                    for (uint8_t i = 0; i < guarded_axis_count; ++i) {
                        const uint8_t axis_id = guarded_axis_ids[i];
                        if (axis_id < self->n_motors_ && self->queues_[axis_id] != nullptr) {
                            self->queues_[axis_id]->setMultiExecActive(false);
                        }
                    }
                };

                // ── Endstop check (per-segment, real-time) ───────────────
                bool endstop_hit = false;
                for (uint8_t a = 0; a < seg.axis_count && !endstop_hit; ++a) {
                    const uint8_t eid = seg.axis_ids[a];
                    if (eid >= self->n_motors_ ||
                        self->queues_[eid] == nullptr) continue;
                    if (self->queues_[eid]->driver().isEndstopActive()) {
                        // Drain remaining batch, e-stop, notify host.
                        self->queues_[eid]->driver().emergencyStop();
                        self->notifySegmentExecuted(seg.motion_sequence);
                        ESP_LOGW(TAG, "endstop on axis %u at seq=%u",
                                 eid, seg.motion_sequence);
                        endstop_hit = true;
                    }
                }
                if (endstop_hit) {
                    clearMultiExecFlags();  // R10: libérer avant RECOVERY
                    state = ExecState::RECOVERY;
                    goto exit_drain;
                }

                // ── Lateral endstop gate (read once per segment) ──────────
                const uint8_t lateral_state = self->readLateralEndstopState();
                const bool lateral_endstop_armed =
                    self->n_motors_ > 1
                    && self->queues_[1] != nullptr
                    && self->queues_[1]->driver().isEndstopArmed();

                // R9: capteur absent + armé = fail-safe (câble coupé pendant le homing).
                // L'ISR ne peut pas détecter ABSENT (elle lit NO/NC individuellement);
                // la détection ABSENT (NO==NC) n'est possible qu'ici en task context.
                if (lateral_endstop_armed &&
                    lateral_state == static_cast<uint8_t>(LateralEndstopState::ABSENT)) {
                    ESP_LOGW(TAG, "lateral endstop ABSENT while armed at seq=%u \xe2\x80\x94 fail-safe stop",
                             seg.motion_sequence);
                    if (self->queues_[1] != nullptr) {
                        self->queues_[1]->driver().emergencyStop();
                    }
                    clearMultiExecFlags();
                    self->notifySegmentExecuted(seg.motion_sequence);
                    state = ExecState::RECOVERY;
                    goto exit_drain;
                }

                const bool lateral_blocked =
                    lateral_endstop_armed
                    && lateral_state !=
                    static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);

                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id < self->n_motors_ && self->queues_[axis_id] != nullptr) {
                        self->queues_[axis_id]->setMultiExecActive(true);
                        if (guarded_axis_count < MULTI_AXIS_MAX_AXES) {
                            guarded_axis_ids[guarded_axis_count++] = axis_id;
                        }
                    }
                }

                // Pre-start any stopped RMT streams before pushing more steps.
                // This prevents pushBlock() from stalling inside DRAIN on the
                // first segment after idle or recovery.
                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id < self->n_motors_ &&
                        self->queues_[axis_id] != nullptr &&
                        !self->queues_[axis_id]->driver().isStreaming()) {
                        self->queues_[axis_id]->kickStart();
                    }
                }

                // ── Write steps to ring buffer (no RMT start) ─────────────
                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id >= self->n_motors_ ||
                        self->queues_[axis_id] == nullptr) continue;
                    if (seg.axes[a].step_count == 0) continue;
                    if (axis_id == 1 && lateral_blocked) {
                        ESP_LOGD(TAG, "axis1 blocked, skip %u steps",
                                 seg.axes[a].step_count);
                        continue;
                    }

                    StepperQueue* axis_queue = self->queues_[axis_id];
                    esp_err_t err = axis_queue->executeConstantRateBlock(
                        seg.axes[a].direction,
                        seg.axes[a].step_count,
                        seg.duration_us);

                    if (err == ESP_ERR_INVALID_STATE) {
                        clearMultiExecFlags();
                        ESP_LOGW(TAG, "axis %u endstop mid-seg seq=%u",
                                 axis_id, seg.motion_sequence);
                        axis_queue->driver().emergencyStop();
                        self->notifySegmentExecuted(seg.motion_sequence);
                        state = ExecState::RECOVERY;
                        goto exit_drain;
                    } else if (err != ESP_OK) {
                        ESP_LOGW(TAG, "axis %u seg %u: %s",
                                 axis_id, seg.motion_sequence,
                                 esp_err_to_name(err));
                    }
                }
                clearMultiExecFlags();

                // ── Schedule deferred notification ────────────────────────
                if ((defer_tail - defer_head) < DEFER_DEPTH) {
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = esp_timer_get_time()
                                         + static_cast<int64_t>(seg.duration_us);
                    defer_seqs[idx]    = seg.motion_sequence;
                    ++defer_tail;
                } else {
                    // Ring full: evict the oldest (earliest scheduled) entry,
                    // notify it now (it is already overdue), then enqueue the
                    // current segment normally. This preserves ordering and
                    // avoids signalling completion before steps reach the ring.
                    const int evict_idx = defer_head & (DEFER_DEPTH - 1);
                    const uint32_t evicted_seq = defer_seqs[evict_idx];
                    self->notifySegmentExecuted(
                        static_cast<uint16_t>(evicted_seq));
                    ++defer_head;
                    // Enqueue current segment.
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = esp_timer_get_time()
                                         + static_cast<int64_t>(seg.duration_us);
                    defer_seqs[idx]    = seg.motion_sequence;
                    ++defer_tail;
                    ESP_LOGW(TAG, "defer ring full: evicted seq=%u to make room for seq=%u",
                             (unsigned)evicted_seq,
                             (unsigned)seg.motion_sequence);
                }

                ++batch_index;

                // ── B4 FIX: UN SEUL check budget, comportement uniforme ─────
                // kickStart + yield + reste en DRAIN (drain_start se reset au
                // prochain passage car il est local au case DRAIN).
                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    kickStartActiveAxes();
                    taskYIELD();
                    goto exit_drain;  // reste en DRAIN, batch_index progresse
                }
            }

            // Batch complet → transition vers RUN.
            // Si on est sorti par goto exit_drain (budget), batch_index < batch_count:
            // on reviendra ici au prochain tick, l'état est déjà DRAIN.
            if (batch_index >= batch_count) {
                state = ExecState::RUN;
            }
            break;

        exit_drain:
            break;  // state = DRAIN, reprend au prochain tick
        }

        // ══════════════════════════════════════════════════════════════════
        // RUN: kickStart RMT on all active axes, return to IDLE
        // ══════════════════════════════════════════════════════════════════
        case ExecState::RUN: {
            kickStartActiveAxes();
            fireDeferred();
            state = ExecState::IDLE;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // FLUSH: reset pipeline state, notify host
        // ══════════════════════════════════════════════════════════════════
        case ExecState::FLUSH: {
            // The flush sentinel is at batch[batch_index].
            const planned_segment_t& flush_seg = batch[batch_index];

            // Reset deferred notification ring.
            defer_head = defer_tail = 0;

            // Notify host with flush sequence.
            self->notifySegmentExecuted(flush_seg.flush_sequence);

            ESP_LOGI(TAG, "executor flush at seq=%u",
                     (unsigned)flush_seg.flush_sequence);

            // Clear batch and return to idle.
            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // RECOVERY: handle endstop / error, drain remaining, return to IDLE
        // ══════════════════════════════════════════════════════════════════
        case ExecState::RECOVERY: {
            // Drain any remaining segments in the planner's output queue
            // (bounded drain to avoid spending too long here).
            planned_segment_t discard;
            uint32_t drained = 0;
            uint16_t last_drained_seq = 0;
            bool has_seq = false;
            while (drained < SEGMENT_QUEUE_DEPTH &&
                   xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
                if (!discard.is_flush) {
                    last_drained_seq = discard.motion_sequence;
                    has_seq = true;
                }
                ++drained;
            }

            if (has_seq) {
                self->notifySegmentExecuted(last_drained_seq);
            }

            // Reset deferred notifications.
            defer_head = defer_tail = 0;

            ESP_LOGW(TAG, "recovery: drained %lu remaining segments (last_seq=%u)",
                     (unsigned long)drained,
                     (unsigned)last_drained_seq);

            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }

        } // switch(state)

        // ── Watchdog safety: yield if idle, sleep if very idle ───────────────
        // If we fetched zero segments in FETCH, sleep to let IDLE1 run.
        // Otherwise, yield to respect other tasks without 10ms stalls.
        if (state == ExecState::IDLE && batch_count == 0) {
            vTaskDelay(1);  // Very idle — sleep and let watchdog reset
        } else {
            taskYIELD();    // Still have work — yield but stay ready
        }
    } // for(;;)
}
