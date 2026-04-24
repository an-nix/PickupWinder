#include "spi_slave_transport.h"

#include <string.h>
#include <driver/spi_common.h>
#include <driver/spi_slave.h>
#include <esp_attr.h>
#include <esp_check.h>
#include <esp_heap_caps.h>
#include <esp_intr_types.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <soc/gpio_struct.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "comm_request_dispatcher.h"
#include "comm_status_builder.h"
#include "messages.h"

static const char* TAG = "comm_spi";

static constexpr uint32_t SPI_TASK_STACK = 4096;
static constexpr UBaseType_t SPI_TASK_PRIO = 24;
static constexpr BaseType_t SPI_TASK_CORE = 0;
static constexpr bool SPI_EXPERIMENTAL_PREQUEUE = false;

static SpiSlaveTransport* s_active_transport = nullptr;

SpiSlaveTransport::SpiSlaveTransport(CommRequestDispatcher& dispatcher,
                                     const CommStatusBuilder& status_builder)
    : dispatcher_(dispatcher)
    , status_builder_(status_builder)
{
}

void SpiSlaveTransport::setReadyPinLevel(bool high) const
{
    if (ready_pin_ == GPIO_NUM_NC) {
        return;
    }

    const uint32_t pin = static_cast<uint32_t>(ready_pin_);
    if (pin < 32) {
        if (high) {
            GPIO.out_w1ts = (1UL << pin);
        } else {
            GPIO.out_w1tc = (1UL << pin);
        }
    } else {
        const uint32_t mask = (1UL << (pin - 32));
        if (high) {
            GPIO.out1_w1ts.val = mask;
        } else {
            GPIO.out1_w1tc.val = mask;
        }
    }
}

void IRAM_ATTR SpiSlaveTransport::postSetupReadyCb(spi_slave_transaction_t* trans)
{
    (void)trans;
    if (s_active_transport != nullptr) {
        s_active_transport->setReadyPinLevel(true);
    }
}

void IRAM_ATTR SpiSlaveTransport::postTransReadyCb(spi_slave_transaction_t* trans)
{
    (void)trans;
    if (s_active_transport != nullptr) {
        s_active_transport->setReadyPinLevel(false);
    }
}

esp_err_t SpiSlaveTransport::init(const SpiBusPins& pins)
{
    ready_pin_ = pins.ready;
    s_active_transport = this;

    if (rx_frame_ == nullptr) {
        rx_frame_ = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_32BIT));
        ESP_RETURN_ON_FALSE(rx_frame_ != nullptr, ESP_ERR_NO_MEM, TAG, "failed to alloc rx_frame");
    }
    if (tx_frame_a_ == nullptr) {
        tx_frame_a_ = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_32BIT));
        ESP_RETURN_ON_FALSE(tx_frame_a_ != nullptr, ESP_ERR_NO_MEM, TAG, "failed to alloc tx_frame_a");
    }
    if (SPI_EXPERIMENTAL_PREQUEUE && rx_frame_b_ == nullptr) {
        rx_frame_b_ = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_32BIT));
        ESP_RETURN_ON_FALSE(rx_frame_b_ != nullptr, ESP_ERR_NO_MEM, TAG, "failed to alloc rx_frame_b");
    }
    if (SPI_EXPERIMENTAL_PREQUEUE && tx_frame_b_ == nullptr) {
        tx_frame_b_ = static_cast<uint8_t*>(heap_caps_malloc(SPI_FRAME_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_32BIT));
        ESP_RETURN_ON_FALSE(tx_frame_b_ != nullptr, ESP_ERR_NO_MEM, TAG, "failed to alloc tx_frame_b");
    }

    if (pins.ready != GPIO_NUM_NC) {
        gpio_config_t ready_cfg = {};
        ready_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(pins.ready));
        ready_cfg.mode = GPIO_MODE_OUTPUT;
        ready_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        ready_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        ready_cfg.intr_type = GPIO_INTR_DISABLE;
        ESP_RETURN_ON_ERROR(gpio_config(&ready_cfg), TAG, "failed to configure SPI ready pin");
        setReadyPinLevel(false);
    } else {
        ESP_LOGW(TAG,
                 "no SPI ready/handshake GPIO configured; ESP-IDF recommends a ready pin for reliable slave timing");
    }

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = pins.mosi;
    bus_cfg.miso_io_num = pins.miso;
    bus_cfg.sclk_io_num = pins.sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = SPI_FRAME_SIZE;
    bus_cfg.flags = SPICOMMON_BUSFLAG_MOSI
                  | SPICOMMON_BUSFLAG_MISO
                  | SPICOMMON_BUSFLAG_SCLK
                  | SPICOMMON_BUSFLAG_IOMUX_PINS;
    bus_cfg.isr_cpu_id = (SPI_TASK_CORE == 0)
        ? ESP_INTR_CPU_AFFINITY_0
        : ESP_INTR_CPU_AFFINITY_1;

    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.mode = 1;
    slave_cfg.spics_io_num = pins.cs;
    slave_cfg.queue_size = 1;
    slave_cfg.flags = 0;
    slave_cfg.post_setup_cb = (pins.ready != GPIO_NUM_NC) ? postSetupReadyCb : nullptr;
    slave_cfg.post_trans_cb = (pins.ready != GPIO_NUM_NC) ? postTransReadyCb : nullptr;

    ESP_RETURN_ON_ERROR(
        spi_slave_initialize(SPI3_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO),
        TAG, "spi_slave_initialize failed");

    BaseType_t rc = xTaskCreatePinnedToCore(
        &SpiSlaveTransport::taskEntry,
        "comm_spi",
        SPI_TASK_STACK,
        this,
        SPI_TASK_PRIO,
        nullptr,
        SPI_TASK_CORE);
    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create comm_spi task");

    ESP_LOGI(TAG, "SPI slave ready  mode=%d MOSI=%d MISO=%d SCLK=%d CS=%d frame=%uB",
             static_cast<int>(slave_cfg.mode),
             static_cast<int>(pins.mosi),
             static_cast<int>(pins.miso),
             static_cast<int>(pins.sclk),
             static_cast<int>(pins.cs),
             static_cast<unsigned>(SPI_FRAME_SIZE));
    ESP_LOGI(TAG, "SPI queued DMA mode enabled (queue_size=%d)",
             static_cast<int>(slave_cfg.queue_size));
    if (pins.ready != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "SPI ready/handshake pin enabled on GPIO%d", static_cast<int>(pins.ready));
    }
    return ESP_OK;
}

void SpiSlaveTransport::taskEntry(void* arg)
{
    static_cast<SpiSlaveTransport*>(arg)->runTask();
}

void SpiSlaveTransport::runTask()
{
    ESP_LOGI(TAG, "SPI task started on core %d", xPortGetCoreID());

    uint32_t diag_cycles = 0;
    uint32_t diag_bad_magic = 0;
    uint32_t diag_bad_magic_zero = 0;
    uint32_t diag_bad_crc = 0;
    uint32_t diag_short_frame = 0;
    uint32_t diag_ok = 0;
    int64_t diag_last_log_us = esp_timer_get_time();

    auto process_rx_frame = [&](uint8_t* rx_frame, size_t trans_len_bits) {
        ++diag_cycles;

        if (trans_len_bits != (SPI_FRAME_SIZE * 8U)) {
            ++diag_short_frame;
            return;
        }

        SpiMessageHeader header {};
        memcpy(&header, rx_frame, sizeof(SpiMessageHeader));

        if (header.magic != SPI_MSG_MAGIC) {
            ++diag_bad_magic;
            if (rx_frame[0] == 0x00 && rx_frame[1] == 0x00) {
                ++diag_bad_magic_zero;
            }
        } else if (header.version != SPI_MSG_VERSION) {
        } else if (header.payload_length > SPI_MAX_PAYLOAD_SIZE) {
        } else if (!spi_message_validate(rx_frame, header)) {
            ++diag_bad_crc;
        } else {
            ++diag_ok;
            const uint8_t* payload = rx_frame + sizeof(SpiMessageHeader);
            dispatcher_.processValidatedRequest(header, payload);
        }
    };

    auto maybe_log_diag = [&]() {
        const int64_t now_us = esp_timer_get_time();
        if ((now_us - diag_last_log_us) >= 1000000) {
            if (diag_bad_magic || diag_bad_crc || diag_short_frame) {
                ESP_LOGW(TAG,
                         "spi diag: cyc=%lu ok=%lu bad_magic=%lu(b0=%lu) bad_crc=%lu short=%lu",
                         static_cast<unsigned long>(diag_cycles),
                         static_cast<unsigned long>(diag_ok),
                         static_cast<unsigned long>(diag_bad_magic),
                         static_cast<unsigned long>(diag_bad_magic_zero),
                         static_cast<unsigned long>(diag_bad_crc),
                         static_cast<unsigned long>(diag_short_frame));
            }
            diag_cycles = 0;
            diag_bad_magic = 0;
            diag_bad_magic_zero = 0;
            diag_bad_crc = 0;
            diag_short_frame = 0;
            diag_ok = 0;
            diag_last_log_us = now_us;
        }
    };

    if (SPI_EXPERIMENTAL_PREQUEUE) {
        spi_slave_transaction_t txn_a = {};
        spi_slave_transaction_t txn_b = {};

        status_builder_.buildStatusFrame(tx_frame_a_);
        status_builder_.buildStatusFrame(tx_frame_b_);

        txn_a.length = SPI_FRAME_SIZE * 8;
        txn_a.tx_buffer = tx_frame_a_;
        txn_a.rx_buffer = rx_frame_;
        txn_b.length = SPI_FRAME_SIZE * 8;
        txn_b.tx_buffer = tx_frame_b_;
        txn_b.rx_buffer = rx_frame_b_;

        esp_err_t err = spi_slave_queue_trans(SPI3_HOST, &txn_a, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_queue_trans(txn_a) failed: %s", esp_err_to_name(err));
            return;
        }
        err = spi_slave_queue_trans(SPI3_HOST, &txn_b, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_queue_trans(txn_b) failed: %s", esp_err_to_name(err));
            return;
        }

        for (;;) {
            spi_slave_transaction_t* done_txn = nullptr;
            err = spi_slave_get_trans_result(SPI3_HOST, &done_txn, portMAX_DELAY);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "spi_slave_get_trans_result failed: %s", esp_err_to_name(err));
                continue;
            }

            uint8_t* completed_rx = nullptr;
            uint8_t* completed_tx = nullptr;
            spi_slave_transaction_t* recycle_txn = nullptr;

            if (done_txn == &txn_a) {
                completed_rx = rx_frame_;
                completed_tx = tx_frame_a_;
                recycle_txn = &txn_a;
            } else if (done_txn == &txn_b) {
                completed_rx = rx_frame_b_;
                completed_tx = tx_frame_b_;
                recycle_txn = &txn_b;
            } else {
                ESP_LOGW(TAG, "unexpected completed SPI transaction pointer %p", done_txn);
                continue;
            }

            process_rx_frame(completed_rx, done_txn->trans_len);
            maybe_log_diag();

            status_builder_.buildStatusFrame(completed_tx);
            esp_rom_delay_us(2);
            err = spi_slave_queue_trans(SPI3_HOST, recycle_txn, portMAX_DELAY);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "spi_slave_queue_trans(recycle) failed: %s", esp_err_to_name(err));
            }
        }
    } else {
        spi_slave_transaction_t txn = {};
        txn.length = SPI_FRAME_SIZE * 8;
        txn.tx_buffer = tx_frame_a_;
        txn.rx_buffer = rx_frame_;

        status_builder_.buildStatusFrame(tx_frame_a_);

        for (;;) {
            esp_err_t err = spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
                continue;
            }

            process_rx_frame(rx_frame_, txn.trans_len);
            maybe_log_diag();
            status_builder_.buildStatusFrame(tx_frame_a_);
        }
    }
}
