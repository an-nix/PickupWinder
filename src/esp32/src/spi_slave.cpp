/* spi_slave.cpp — Core 0 SPI slave implementation.
 *
 * Uses ESP-IDF spi_slave driver (interrupt-driven, DMA-capable).
 * Full-duplex transactions of STATUS_FRAME_SIZE (44) bytes.
 *
 * Transaction flow (per SPI cycle, ~44 bytes × 8 bits / 4 MHz ≈ 88 µs):
 *   1. g_rmt_engine.get_status() — snapshot StatusFrame under spinlock
 *   2. spi_slave_transmit()   — wait for master to drive CS+CLK
 *   3. CRC validation          — drop frame on mismatch (silently)
 *   4. cmd_queue.push()        — hand CmdFrame to Core 1 stepper task
 *
 * DMA buffers must be in DRAM and word-aligned (WORD_ALIGNED_ATTR).
 * STATUS_FRAME_SIZE = 44 bytes (protocol.h).
 */

#include "spi_slave.h"
#include "rmt_stepper.h"
#include <driver/spi_slave.h>
#include <esp_log.h>
#include <cstring>

static const char* TAG = "spi_slave";

// ── DMA-aligned buffers ─────────────────────────────────────────────────────
// ESP-IDF SPI slave requires DMA-capable memory (DRAM, word-aligned).
WORD_ALIGNED_ATTR static uint8_t rx_buf[STATUS_FRAME_SIZE];
WORD_ALIGNED_ATTR static uint8_t tx_buf[STATUS_FRAME_SIZE];

// ── Statistics ──────────────────────────────────────────────────────────────
static volatile SpiStats stats = {};

// ── SPI task ────────────────────────────────────────────────────────────────
static void spi_task(void* param) {
    auto* cmd_queue = static_cast<CmdQueue*>(param);

    ESP_LOGI(TAG, "SPI slave task started on Core %d", xPortGetCoreID());

    for (;;) {
        // Prepare status frame for next transaction
        StatusFrame sf = g_rmt_engine.get_status();
        memcpy(tx_buf, &sf, sizeof(StatusFrame));

        // Setup transaction
        spi_slave_transaction_t txn = {};
        txn.length    = STATUS_FRAME_SIZE * 8;  // bits
        txn.tx_buffer = tx_buf;
        txn.rx_buffer = rx_buf;

        // Block until transaction completes (master drives CS + CLK)
        esp_err_t ret = spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transmit error: %s", esp_err_to_name(ret));
            continue;
        }

        stats.tx_frames = stats.tx_frames + 1;

        // Extract CmdFrame from first 8 bytes of rx_buf
        CmdFrame frame;
        memcpy(&frame, rx_buf, CMD_FRAME_SIZE);

        // Validate CRC
        if (!cmd_frame_check_crc(frame)) {
            stats.rx_crc_errors = stats.rx_crc_errors + 1;
            continue;
        }

        // Skip NOP commands
        if (frame.cmd == static_cast<uint8_t>(CmdOpcode::NOP)) {
            continue;
        }

        stats.rx_frames = stats.rx_frames + 1;

        // Push to command queue for Core 1
        if (!cmd_queue->push(frame)) {
            stats.rx_queue_full = stats.rx_queue_full + 1;
            ESP_LOGW(TAG, "Command queue full — frame dropped");
        }
    }
}

// ── Init ────────────────────────────────────────────────────────────────────

void spi_slave_init(const SpiPins& pins) {
    // SPI bus configuration
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num   = pins.mosi;
    bus_cfg.miso_io_num   = pins.miso;
    bus_cfg.sclk_io_num   = pins.sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = STATUS_FRAME_SIZE;

    // SPI slave configuration
    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.mode          = 0;           // SPI Mode 0 (CPOL=0, CPHA=0)
    slave_cfg.spics_io_num  = pins.cs;
    slave_cfg.queue_size    = 1;           // one transaction at a time
    slave_cfg.flags         = 0;
    slave_cfg.post_setup_cb = nullptr;
    slave_cfg.post_trans_cb = nullptr;

    esp_err_t ret = spi_slave_initialize(SPI3_HOST, &bus_cfg, &slave_cfg,
                                          SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI slave init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SPI slave initialized on VSPI: MOSI=%d MISO=%d SCLK=%d CS=%d",
             pins.mosi, pins.miso, pins.sclk, pins.cs);
}

// ── Start ───────────────────────────────────────────────────────────────────

void spi_slave_start(CmdQueue& cmd_queue) {
    xTaskCreatePinnedToCore(
        spi_task,
        "spi_slave",
        4096,
        &cmd_queue,
        10,              // medium priority (below stepper, above idle)
        nullptr,
        0                // Core 0
    );
}

// ── Stats ───────────────────────────────────────────────────────────────────

SpiStats spi_get_stats() {
    return const_cast<const SpiStats&>(stats);
}
