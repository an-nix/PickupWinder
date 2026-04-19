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
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "motion_planner.h"

static const char* TAG = "comm_iface";

static constexpr uint32_t  SPI_TASK_STACK  = 4096;
static constexpr UBaseType_t SPI_TASK_PRIO = 10;
static constexpr BaseType_t  SPI_TASK_CORE = 0;

static constexpr uint32_t    MULTI_EXEC_STACK  = 6144;
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
static constexpr uint32_t MULTI_AXIS_QUEUE_DEPTH = 64;
static QueueHandle_t s_multi_axis_queue  = nullptr;

/**
 * @brief Global queue for flush requests.  Depth 4 is more than enough since
 *        the host can only issue one flush at a time.
 */
static constexpr uint32_t FLUSH_QUEUE_DEPTH = 4;
static QueueHandle_t s_flush_queue = nullptr;

DMA_ATTR static uint8_t s_rx_frame[SPI_FRAME_SIZE];
DMA_ATTR static uint8_t s_tx_frame_a[SPI_FRAME_SIZE];
DMA_ATTR static uint8_t s_tx_frame_b[SPI_FRAME_SIZE];

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

    // Create the global multi-axis segment queue.
    s_multi_axis_queue = xQueueCreate(MULTI_AXIS_QUEUE_DEPTH, sizeof(multi_axis_block_t));
    ESP_RETURN_ON_FALSE(s_multi_axis_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create multi-axis queue");

    // Create the global flush request queue.
    s_flush_queue = xQueueCreate(FLUSH_QUEUE_DEPTH, sizeof(flush_request_t));
    ESP_RETURN_ON_FALSE(s_flush_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create flush queue");

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

    // ── Planner layer: decomposes multi-axis blocks into planned segments ───
    ESP_RETURN_ON_ERROR(planner_.init(s_multi_axis_queue, s_flush_queue),
                        TAG, "failed to init motion planner");

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
            payload->ring_free_slots[axis]  = 0;
            payload->underrun_count[axis]   = 0;
        }
    }
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

    // Atomic load — lock-free cross-core read (written by Core 1 executor).
    payload->last_executed_sequence = last_executed_sequence_.load(std::memory_order_acquire);

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

esp_err_t CommInterface::handleEnableEndstop(const EnableEndstopPayload& payload)
{
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
    if (payload_length < sizeof(MultiAxisSegmentBlockHeader)) {
        return ESP_ERR_INVALID_SIZE;
    }

    MultiAxisSegmentBlockHeader hdr_val;
    memcpy(&hdr_val, payload, sizeof(hdr_val));
    const uint8_t axis_count     = hdr_val.axis_count;
    const uint8_t segment_count  = hdr_val.segment_count;

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
    if (payload_length < static_cast<uint16_t>(expected_length)) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Deserialise.
    multi_axis_block_t block {};
    block.axis_count     = axis_count;
    block.segment_count  = segment_count;

    const uint8_t* cursor = payload + sizeof(MultiAxisSegmentBlockHeader);

    // axis_ids
    for (uint8_t a = 0; a < axis_count; ++a) {
        block.axis_ids[a] = cursor[a];
    }
    cursor += axis_count;

    // segments
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
    if (xQueueSend(s_multi_axis_queue, &block, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT; // maps to QUEUE_FULL result code
    }
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
    flush_request_t req { .flush_sequence = flush_payload.flush_sequence };
    if (xQueueSend(s_flush_queue, &req, 0) != pdTRUE) {
        // Flush queue full — this should never happen in normal operation.
        ESP_LOGW(TAG, "flush queue full — flush_seq=%u dropped",
                 (unsigned)flush_payload.flush_sequence);
        return ESP_ERR_TIMEOUT;
    }
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
    uint16_t current = last_executed_sequence_.load(std::memory_order_relaxed);
    if (static_cast<int16_t>(motion_seq - current) > 0) {
        last_executed_sequence_.store(motion_seq, std::memory_order_release);
    }
}

uint8_t CommInterface::readLateralEndstopState() const
{
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
    if (axis_id != 1) {
        return true;
    }
    return readLateralEndstopState() == static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

esp_err_t CommInterface::handleFrame(const SpiMessageHeader& header, const uint8_t* payload)
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

    // Double-buffer ping-pong: while DMA transmits tx_ping, we build the next
    // status frame into tx_pong.  This reduces pipeline lag by one full SPI
    // round-trip — the status sent in transaction N reflects state AFTER
    // transaction N-1 was handled, not state from before the previous transmit.
    uint8_t* tx_ping = s_tx_frame_a;
    uint8_t* tx_pong = s_tx_frame_b;

    // Pre-build the very first frame before entering the loop so the initial
    // transaction has valid (zero-but-structured) content.
    self->buildStatusFrame(tx_ping);

    for (;;) {
        // ── Transmit the previously-built status frame ────────────────────────
        spi_slave_transaction_t txn = {};
        txn.length = SPI_FRAME_SIZE * 8;
        txn.tx_buffer = tx_ping;
        txn.rx_buffer = s_rx_frame;

        esp_err_t err = spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY);
        // DMA is done with tx_ping — safe to reuse as the next write buffer.

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            // Rebuild into the same ping buffer and retry.
            self->buildStatusFrame(tx_ping);
            continue;
        }

        // ── Parse and handle incoming frame ───────────────────────────────────
        SpiMessageHeader header {};
        memcpy(&header, s_rx_frame, sizeof(SpiMessageHeader));

        if (header.magic != SPI_MSG_MAGIC) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_MAGIC);
        } else if (header.version != SPI_MSG_VERSION) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_VERSION);
        } else if (header.payload_length > SPI_MAX_PAYLOAD_SIZE) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
        } else if (!spi_message_validate(s_rx_frame, header)) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_CRC);
        } else {
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
            } else if (err == ESP_ERR_INVALID_STATE) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::ENDSTOP_BLOCKED);
            } else {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::INTERNAL_ERROR);
                ESP_LOGW(TAG, "message 0x%02X failed: %s",
                         header.msg_type, esp_err_to_name(err));
            }
        }

        // ── Build next status frame into the now-idle buffer ──────────────────
        // We write into tx_pong (the buffer NOT currently wired to DMA).
        // Reflects state AFTER handling the frame we just received.
        self->buildStatusFrame(tx_pong);

        // Swap: tx_pong becomes the next transmit buffer.
        uint8_t* tmp = tx_ping;
        tx_ping = tx_pong;
        tx_pong = tmp;
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

    // ── Segment queue handle from the planner ─────────────────────────────
    QueueHandle_t seg_queue = self->planner_.segmentQueue();

    // ── Deferred notification ring ────────────────────────────────────────
    static constexpr int DEFER_DEPTH = 128;
    int64_t  defer_fire_us[DEFER_DEPTH];
    uint32_t defer_seqs[DEFER_DEPTH];
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
    for (;;) {
        // Always fire due deferred notifications at top of loop.
        fireDeferred();

        switch (state) {

        // ══════════════════════════════════════════════════════════════════
        // IDLE: wait for segments from the planner (blocking with timeout)
        // ══════════════════════════════════════════════════════════════════
        case ExecState::IDLE: {
            // Compute wait timeout: wake early if a deferred notification
            // is about to fire.  Hard-cap at 1 ms so ISR ring-space
            // notifications (which wake ulTaskNotifyTake, not xQueueReceive)
            // don't cause >1 ms stalls.
            TickType_t wait_ticks;
            if (defer_head != defer_tail) {
                const int idx = defer_head & (DEFER_DEPTH - 1);
                const int64_t remaining_us =
                    defer_fire_us[idx] - esp_timer_get_time();
                if (remaining_us <= 500) {
                    wait_ticks = 0;
                } else {
                    wait_ticks = 1;
                }
            } else {
                wait_ticks = pdMS_TO_TICKS(1);
            }

            planned_segment_t seg;
            if (xQueueReceive(seg_queue, &seg, wait_ticks) == pdTRUE) {
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
                // ── Pre-check: yield if time budget will be exceeded ───────────
                // This prevents accumulating too much CPU time before yielding.
                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    kickStartActiveAxes();
                    taskYIELD();
                    // Restart from FETCH to get fresh batch and reset timer.
                    state = ExecState::FETCH;
                    goto exit_drain;
                }

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
                    state = ExecState::RECOVERY;
                    goto exit_drain;
                }

                // ── Lateral endstop gate (read once per segment) ──────────
                const uint8_t lateral_state = self->readLateralEndstopState();
                const bool lateral_blocked =
                    lateral_state !=
                    static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);

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

                    esp_err_t err =
                        self->queues_[axis_id]->executeConstantRateBlock(
                            seg.axes[a].direction,
                            seg.axes[a].step_count,
                            seg.duration_us);

                    if (err == ESP_ERR_INVALID_STATE) {
                        ESP_LOGW(TAG, "axis %u endstop mid-seg seq=%u",
                                 axis_id, seg.motion_sequence);
                        self->queues_[axis_id]->driver().emergencyStop();
                        self->notifySegmentExecuted(seg.motion_sequence);
                        state = ExecState::RECOVERY;
                        goto exit_drain;
                    } else if (err != ESP_OK) {
                        ESP_LOGW(TAG, "axis %u seg %u: %s",
                                 axis_id, seg.motion_sequence,
                                 esp_err_to_name(err));
                    }
                }

                // ── Schedule deferred notification ────────────────────────
                if ((defer_tail - defer_head) < DEFER_DEPTH) {
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = seg.scheduled_time_us
                                         + static_cast<int64_t>(seg.duration_us);
                    defer_seqs[idx]    = seg.motion_sequence;
                    ++defer_tail;
                } else {
                    ESP_LOGW(TAG, "defer ring overflow at seq=%u — notifying immediately",
                             (unsigned)seg.motion_sequence);
                    self->notifySegmentExecuted(
                        static_cast<uint16_t>(seg.motion_sequence));
                }

                ++batch_index;

                // ── Time budget check (watchdog safety) ───────────────────
                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    // Budget exhausted — transition to RUN to kickStart,
                    // then yield before processing remaining segments.
                    kickStartActiveAxes();
                    taskYIELD();
                    // Continue draining after yield (reset budget).
                    break;  // will re-enter DRAIN on next iteration
                }
            }

            // All segments in batch processed — transition to RUN.
            if (batch_index >= batch_count) {
                state = ExecState::RUN;
            }
            // else: budget break, stay in DRAIN for remaining segments.
            break;

        exit_drain:
            break;  // state already set by the goto target
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
            while (drained < SEGMENT_QUEUE_DEPTH &&
                   xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
                ++drained;
            }

            // Reset deferred notifications.
            defer_head = defer_tail = 0;

            ESP_LOGW(TAG, "recovery: drained %lu remaining segments",
                     (unsigned long)drained);

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
