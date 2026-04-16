/**
 * @file comm_interface.cpp
 * @brief Host communication interface — UART stub implementation.
 *
 * See comm_interface.h for protocol details and extension points.
 */

#include "comm_interface.h"

#include <string.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "comm_iface";

// UART RX task parameters
static constexpr uint32_t  RX_TASK_STACK  = 4096;
static constexpr UBaseType_t RX_TASK_PRIO = 10; // lower than stepper tasks
static constexpr BaseType_t  RX_TASK_CORE = 0;  // leave Core 1 for steppers

// Wire frame constants
static constexpr uint8_t FRAME_SYNC     = 0xAA;
static constexpr uint8_t ACK_OPCODE     = 0xAC;
static constexpr size_t  FRAME_HDR_SIZE = 3; // SYNC + LENGTH(2)
static constexpr size_t  FRAME_CRC_SIZE = 1;

// Minimal CRC-8 (polynomial 0x07, no reflect) — replace with CRC-8/MAXIM
// from protocol.h if needed.
static uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommInterface::CommInterface(StepperQueue* queues[], uint8_t n_motors)
    : n_motors_(n_motors < COMM_MAX_MOTORS ? n_motors : COMM_MAX_MOTORS)
{
    for (uint8_t i = 0; i < COMM_MAX_MOTORS; ++i) {
        queues_[i] = (i < n_motors_) ? queues[i] : nullptr;
    }
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t CommInterface::init(int uart_num, int tx_gpio, int rx_gpio, int baud_rate)
{
    uart_num_ = uart_num;

    // ── UART configuration ──────────────────────────────────────────────────
    uart_config_t cfg = {};
    cfg.baud_rate           = baud_rate;
    cfg.data_bits           = UART_DATA_8_BITS;
    cfg.parity              = UART_PARITY_DISABLE;
    cfg.stop_bits           = UART_STOP_BITS_1;
    cfg.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    cfg.rx_flow_ctrl_thresh = 0;
    cfg.source_clk          = UART_SCLK_DEFAULT;

    const uart_port_t port = static_cast<uart_port_t>(uart_num_);

    ESP_RETURN_ON_ERROR(uart_param_config(port, &cfg), TAG,
                        "uart_param_config failed");
    ESP_RETURN_ON_ERROR(
        uart_set_pin(port, tx_gpio, rx_gpio,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
        TAG, "uart_set_pin failed");

    // RX buffer = 4 × max frame size; no TX buffer (blocking writes are fine)
    const size_t rx_buf = 4 * (FRAME_HDR_SIZE + sizeof(comm_packet_t) + FRAME_CRC_SIZE);
    ESP_RETURN_ON_ERROR(uart_driver_install(port, rx_buf, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");

    // ── RX task ─────────────────────────────────────────────────────────────
    BaseType_t rc = xTaskCreatePinnedToCore(
        &CommInterface::rxTask,
        "comm_rx",
        RX_TASK_STACK,
        this,
        RX_TASK_PRIO,
        nullptr,
        RX_TASK_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create comm_rx task");

    ESP_LOGI(TAG, "init OK  uart=%d  baud=%d  tx=GPIO%d  rx=GPIO%d",
             uart_num_, baud_rate, tx_gpio, rx_gpio);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// injectPacket()  — public API for testing / demo without real UART host
// ---------------------------------------------------------------------------

esp_err_t CommInterface::injectPacket(const comm_packet_t& pkt)
{
    if (pkt.motor_id >= n_motors_ || queues_[pkt.motor_id] == nullptr) {
        ESP_LOGE(TAG, "injectPacket: invalid motor_id %u", pkt.motor_id);
        return ESP_ERR_INVALID_ARG;
    }

    // Build a step_block_t from the packet
    step_block_t block;
    block.count = pkt.count < STEP_BLOCK_SIZE ? pkt.count : STEP_BLOCK_SIZE;
    memcpy(block.steps, pkt.steps, block.count * sizeof(step_cmd_t));

    StepperQueue* q = queues_[pkt.motor_id];
    esp_err_t err = q->enqueueBlock(block, 0 /* non-blocking */);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "motor%u: queue full, packet seq=%u dropped",
                 pkt.motor_id, pkt.block_seq);
        return err;
    }

    // Flow-control: notify host if buffer is running low
    uint32_t free_slots = q->available();
    if (free_slots < FLOW_CONTROL_THRESHOLD) {
        sendFlowAck(pkt.motor_id, free_slots);
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// sendFlowAck()
// ---------------------------------------------------------------------------

void CommInterface::sendFlowAck(uint8_t motor_id, uint32_t free_slots)
{
    // In production: serialise and write to UART TX.
    // Stub: log to console.
    ESP_LOGD(TAG, "FLOW_ACK → motor=%u  free_slots=%lu  (threshold=%d)",
             motor_id, (unsigned long)free_slots, FLOW_CONTROL_THRESHOLD);

    if (uart_num_ < 0) {
        return; // UART not initialised (demo mode)
    }

    // [ACK_OPCODE] [motor_id] [free_slots_u8]
    uint8_t ack[3] = {
        ACK_OPCODE,
        motor_id,
        static_cast<uint8_t>(free_slots > 255 ? 255 : free_slots)
    };
    uart_write_bytes(static_cast<uart_port_t>(uart_num_),
                     reinterpret_cast<const char*>(ack), sizeof(ack));
}

// ---------------------------------------------------------------------------
// rxTask() — Core 0, priority 10
// ---------------------------------------------------------------------------

void CommInterface::rxTask(void* arg)
{
    CommInterface* self = static_cast<CommInterface*>(arg);
    const uart_port_t uart_num = static_cast<uart_port_t>(self->uart_num_);

    // Receive buffer large enough for one maximum-size frame
    const size_t buf_size = FRAME_HDR_SIZE + sizeof(comm_packet_t) + FRAME_CRC_SIZE + 4;
    uint8_t* buf = static_cast<uint8_t*>(malloc(buf_size));
    if (!buf) {
        ESP_LOGE(TAG, "rxTask: out of memory");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "rxTask started on UART%d", uart_num);

    for (;;) {
        // ── Wait for SYNC byte ──────────────────────────────────────────────
        uint8_t sync = 0;
        int n = uart_read_bytes(uart_num, &sync, 1, portMAX_DELAY);
        if (n <= 0 || sync != FRAME_SYNC) {
            continue; // discard out-of-sync bytes
        }

        // ── Read 2-byte little-endian length ───────────────────────────────
        uint8_t len_bytes[2];
        n = uart_read_bytes(uart_num, len_bytes, 2, pdMS_TO_TICKS(50));
        if (n < 2) { continue; }

        const uint16_t payload_len =
            static_cast<uint16_t>(len_bytes[0]) |
            (static_cast<uint16_t>(len_bytes[1]) << 8);

        if (payload_len > sizeof(comm_packet_t)) {
            ESP_LOGW(TAG, "rxTask: oversized frame (%u bytes), discarding", payload_len);
            continue;
        }

        // ── Read payload + CRC ─────────────────────────────────────────────
        n = uart_read_bytes(uart_num, buf, payload_len + FRAME_CRC_SIZE,
                            pdMS_TO_TICKS(100));
        if (n < static_cast<int>(payload_len + FRAME_CRC_SIZE)) {
            ESP_LOGW(TAG, "rxTask: short frame read (%d/%u)", n, payload_len + 1);
            continue;
        }

        // ── CRC check ──────────────────────────────────────────────────────
        const uint8_t expected_crc = crc8(buf, payload_len);
        const uint8_t received_crc = buf[payload_len];
        if (expected_crc != received_crc) {
            ESP_LOGW(TAG, "rxTask: CRC mismatch (exp 0x%02X got 0x%02X)",
                     expected_crc, received_crc);
            continue;
        }

        // ── Deserialise comm_packet_t ───────────────────────────────────────
        comm_packet_t pkt;
        memcpy(&pkt, buf, payload_len);
        self->injectPacket(pkt);
    }

    free(buf);
    vTaskDelete(nullptr);
}
