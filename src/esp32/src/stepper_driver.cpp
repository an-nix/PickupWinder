/**
 * @file stepper_driver.cpp
 * @brief Physical-layer RMT stepper driver — streaming simple_encoder impl.
 *
 * See stepper_driver.h for architecture details.
 */

#include "stepper_driver.h"

#include <algorithm>
#include <cstring>
#include <esp_log.h>
#include <esp_check.h>
#include <hal/gpio_ll.h>

static const char* TAG = "stepper_driver";

// ---------------------------------------------------------------------------
// encode_steps() — simple_encoder callback, runs in ISR context (IRAM)
// ---------------------------------------------------------------------------
//
// Called by the RMT driver whenever it needs more symbols. Reads up to
// PART_SIZE entries from the ring buffer and converts each to one
// rmt_symbol_word_t with a FastAccelStepper-style balanced pulse:
//   duration0 = ticks / 2        (HIGH)
//   duration1 = ticks - duration0 (LOW)
//
// This 50/50 split is closer to the reference FastAccelStepper RMT backend
// than a fixed-width HIGH pulse and reduces timing quantization asymmetry.
//
// Direction changes:
//   If a ring entry has toggle_dir=1 and the previous chunk contained steps,
//   emit a pause chunk first (to meet driver IC setup time), then toggle
//   DIR on the next callback invocation.
//
// On starvation, the callback follows the same conservative policy as
// FastAccelStepper's ESP32 IDF5 backend: emit one LOW-level pause chunk,
// arm stop, and let the next callback finish the transaction.

extern "C" size_t IRAM_ATTR encode_steps(const void* /*data*/,
                                          size_t /*data_size*/,
                                          size_t /*symbols_written*/,
                                          size_t symbols_free,
                                          rmt_symbol_word_t* symbols,
                                          bool* done, void* arg)
{
    StepperDriver* drv = static_cast<StepperDriver*>(arg);
    *done = false;

    if (symbols_free < PART_SIZE) {
        return 0;  // Wait for more space
    }

    uint32_t rd = drv->ring_read_;
    uint32_t wr = drv->ring_write_;  // volatile read — task may be writing
    __sync_synchronize();  // CHANGE 1: compiler+hardware barrier — ensures ring entry
                           // data written by the producer is visible before we read it

    // Check for explicit stop request
    if (drv->rmt_stopped_) {
        *done = true;
        return 0;
    }

    // Ring empty — emit one LOW-level pause chunk, arm stop, and let the
    // next callback terminate the transmission.
    if (rd == wr) {
        drv->last_chunk_had_steps_ = false;
        drv->ring_underrun_count_ = drv->ring_underrun_count_ + 1;  // starvation diagnostic
        drv->rmt_stopped_ = true;
        uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
        for (uint32_t i = 0; i < PART_SIZE; i++) {
            symbols[i].level0    = 0;
            symbols[i].duration0 = t;
            symbols[i].level1    = 0;
            symbols[i].duration1 = t;
        }
        return PART_SIZE;
    }

    // Data is available after underrun — clear the stop flag so we can continue
    // encoding. This handles the case where the ring was empty, we emitted a
    // pause, and now new data has arrived before on_trans_done_isr fires.
    drv->rmt_stopped_ = false;

    // Peek at next entry — check for direction change
    ring_entry_t* entry = &drv->ring_[rd & STEP_RING_MASK];
    if (entry->toggle_dir) {
        if (drv->last_chunk_had_steps_) {
            // Previous chunk had steps — emit a fixed pause chunk so the next
            // callback can toggle DIR safely at the chunk boundary.
            drv->last_chunk_had_steps_ = false;
            uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
            for (uint32_t i = 0; i < PART_SIZE; i++) {
                symbols[i].duration0 = t;
                symbols[i].level0    = 0;
                symbols[i].duration1 = t;
                symbols[i].level1    = 0;
            }
            return PART_SIZE;
        }
        // Safe to toggle now (previous chunk was a pause or first chunk)
        gpio_ll_set_level(&GPIO, drv->dir_pin_,
                          gpio_ll_get_level(&GPIO, drv->dir_pin_) ^ 1);
        entry->toggle_dir = 0;
    }

    // Fill PART_SIZE symbols from ring buffer
    bool has_steps = false;
    for (uint32_t i = 0; i < PART_SIZE; i++) {
        if (rd != wr) {
            ring_entry_t* e = &drv->ring_[rd & STEP_RING_MASK];

            // Handle mid-chunk direction changes: stop filling, pad the
            // remainder with a fixed LOW-level pause chunk.
            if (e->toggle_dir && i > 0) {
                uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
                for (uint32_t j = i; j < PART_SIZE; j++) {
                    symbols[j].duration0 = t;
                    symbols[j].level0    = 0;
                    symbols[j].duration1 = t;
                    symbols[j].level1    = 0;
                }
                break;
            }

            uint16_t t = e->ticks;
            uint16_t high_ticks = t >> 1;
            uint16_t low_ticks = t - high_ticks;
            if (high_ticks < RMT_STEP_PULSE_TICKS) {
                high_ticks = RMT_STEP_PULSE_TICKS;
                low_ticks = t - high_ticks;
            }
            if (low_ticks < RMT_STEP_PULSE_TICKS) {
                low_ticks = RMT_STEP_PULSE_TICKS;
                high_ticks = t - low_ticks;
            }
            drv->last_ticks_ = t;
            symbols[i].level0    = 1;
            symbols[i].duration0 = high_ticks;
            symbols[i].level1    = 0;
            symbols[i].duration1 = low_ticks;

            rd++;
            has_steps = true;
        } else {
            // Ring exhausted mid-chunk — pad the remainder with a pause chunk,
            // arm stop, and let the next callback terminate the transaction.
            drv->ring_underrun_count_ = drv->ring_underrun_count_ + 1;  // starvation diagnostic
            drv->rmt_stopped_ = true;
            uint16_t t = static_cast<uint16_t>((MIN_CMD_TICKS + 2 * PART_SIZE - 1) / (2 * PART_SIZE));
            for (uint32_t j = i; j < PART_SIZE; j++) {
                symbols[j].level0    = 0;
                symbols[j].duration0 = t;
                symbols[j].level1    = 0;
                symbols[j].duration1 = t;
            }
            break;
        }
    }

    drv->ring_read_ = rd;
    drv->last_chunk_had_steps_ = has_steps;
    if (drv->producer_task_ != nullptr) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(drv->producer_task_, &woken);
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    return PART_SIZE;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

StepperDriver::StepperDriver(gpio_num_t step_pin, gpio_num_t dir_pin,
                             gpio_num_t en_pin,   uint8_t    motor_id)
    : dir_pin_(dir_pin)
    , step_pin_(step_pin)
    , en_pin_(en_pin)
    , motor_id_(motor_id)
{
    memset(ring_, 0, sizeof(ring_));
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::init()
{
    // ── 1. DIR and EN GPIO ──────────────────────────────────────────────────
    gpio_config_t io_conf = {};
    io_conf.mode          = GPIO_MODE_OUTPUT;
    io_conf.intr_type     = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask  = (1ULL << dir_pin_) | (1ULL << en_pin_);
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG,
                        "motor%u: gpio_config DIR/EN failed", motor_id_);

    gpio_set_level(en_pin_,  1);
    gpio_set_level(dir_pin_, last_dir_ ? 1 : 0);

    // ── 2. RMT TX channel ───────────────────────────────────────────────────
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num           = step_pin_;
    tx_cfg.clk_src            = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz      = RMT_STEP_RESOLUTION_HZ;
    tx_cfg.mem_block_symbols  = RMT_MEM_SYMBOLS;
    tx_cfg.trans_queue_depth  = 1;
    tx_cfg.flags.invert_out   = false;
    tx_cfg.flags.with_dma     = false;

    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &channel_), TAG,
                        "motor%u: rmt_new_tx_channel failed", motor_id_);

    // ── 3. Simple encoder with callback ─────────────────────────────────────
    rmt_simple_encoder_config_t enc_cfg = {};
    enc_cfg.callback       = encode_steps;
    enc_cfg.arg            = this;
    enc_cfg.min_chunk_size = PART_SIZE;

    ESP_RETURN_ON_ERROR(rmt_new_simple_encoder(&enc_cfg, &encoder_), TAG,
                        "motor%u: rmt_new_simple_encoder failed", motor_id_);

    // ── 4. Transmit config ──────────────────────────────────────────────────
    tx_config_.loop_count              = 0;
    tx_config_.flags.eot_level         = 0;
    tx_config_.flags.queue_nonblocking = 1;

    // ── 5. on_trans_done callback ───────────────────────────────────────────
    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = &StepperDriver::on_trans_done_isr;
    ESP_RETURN_ON_ERROR(
        rmt_tx_register_event_callbacks(channel_, &cbs, this), TAG,
        "motor%u: rmt_tx_register_event_callbacks failed", motor_id_);

    // ── 6. Enable the RMT channel ───────────────────────────────────────────
    ESP_RETURN_ON_ERROR(rmt_enable(channel_), TAG,
                        "motor%u: rmt_enable failed", motor_id_);

    ESP_LOGI(TAG, "motor%u: init OK  step=GPIO%d  dir=GPIO%d  en=GPIO%d  "
                  "ring=%u  part=%u",
             motor_id_, (int)step_pin_, (int)dir_pin_, (int)en_pin_,
             STEP_RING_SIZE, PART_SIZE);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// enable / disable / emergencyStop
// ---------------------------------------------------------------------------

void StepperDriver::enable()
{
    gpio_set_level(en_pin_, 0);
    enabled_ = true;
}

void StepperDriver::disable()
{
    gpio_set_level(en_pin_, 1);
    enabled_ = false;
}

void StepperDriver::emergencyStop()
{
    rmt_disable(channel_);
    rmt_enable(channel_);

    ring_read_  = 0;
    ring_write_ = 0;
    rmt_running_ = false;
    rmt_stopped_ = true;
    last_chunk_had_steps_ = false;

    ESP_LOGW(TAG, "motor%u: emergency stop", motor_id_);
}

// ---------------------------------------------------------------------------
// stopStream()
// ---------------------------------------------------------------------------

void StepperDriver::stopStream()
{
    if (!rmt_running_) return;

    // Signal the encoder callback to end the transmission
    rmt_stopped_ = true;

    // Wait for the RMT hardware to finish the current transaction
    rmt_tx_wait_all_done(channel_, pdMS_TO_TICKS(500));
    rmt_running_ = false;
}

// ---------------------------------------------------------------------------
// startStream()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::startStream()
{
    rmt_stopped_ = false;
    rmt_running_ = true;
    last_chunk_had_steps_ = false;

    // `this` is in internal DRAM (static global) — passes esp_ptr_internal()
    // check. sizeof(*this) > 0 passes payload_bytes != 0. The callback ignores
    // both data and data_size entirely. Reset the encoder so each new
    // transaction restarts from symbol position 0.
    encoder_->reset(encoder_);
    esp_err_t err = rmt_transmit(channel_, encoder_, this, sizeof(*this), &tx_config_);
    if (err != ESP_OK) {
        rmt_running_ = false;
        rmt_stopped_ = true;
        ESP_LOGE(TAG, "motor%u: rmt_transmit failed: %s",
                 motor_id_, esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------------------------------
// pushBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::pushBlock(const step_block_t& block)
{
    if (ring_underrun_count_ > 0) {
        ESP_LOGW(TAG, "motor%u: ring underrun x%lu since last pushBlock",
                 motor_id_, (unsigned long)ring_underrun_count_);
        ring_underrun_count_ = 0;
    }

    if (block.count == 0) {
        return ESP_OK;
    }

    if (producer_task_ == nullptr) {
        producer_task_ = xTaskGetCurrentTaskHandle();
    }

    const uint32_t count = std::min<uint32_t>(block.count, STEP_BLOCK_SIZE);

    const bool new_dir = block.steps[0].direction;
    bool need_toggle = (new_dir != last_dir_);
    last_dir_ = new_dir;

    for (uint32_t i = 0; i < count; i++) {
        // Back-pressure: wait until the encoder ISR has consumed at least one
        // chunk and notified this producer task. This avoids a CPU1 spin loop
        // and keeps the task watchdog satisfied.
        while (ringFree() == 0) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        uint32_t ticks = block.steps[i].interval_ticks;

        ticks = std::max<uint32_t>(ticks, RMT_STEP_MIN_TICKS);
        ticks = std::min<uint32_t>(ticks, RMT_STEP_MAX_TICKS);

        uint32_t wr = ring_write_;
        ring_entry_t* e = &ring_[wr & STEP_RING_MASK];
        e->ticks      = static_cast<uint16_t>(ticks);
        e->toggle_dir = (i == 0 && need_toggle) ? 1 : 0;
        e->pad        = 0;

        __sync_synchronize();
        ring_write_ = wr + 1;
    }

    // NOTE: startStream() is NOT called here.
    //
    // The executor task (stepper_queue.cpp) calls startStream() explicitly after
    // draining all available FreeRTOS queue blocks into the ring. This maximises
    // ring fill before the RMT starts, which is critical at high step rates where
    // a single 64-step block lasts less than one SPI round-trip.
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// on_trans_done ISR
// ---------------------------------------------------------------------------

bool IRAM_ATTR StepperDriver::on_trans_done_isr(
    rmt_channel_handle_t /*tx_chan*/,
    const rmt_tx_done_event_data_t* /*edata*/,
    void* user_ctx)
{
    StepperDriver* self = static_cast<StepperDriver*>(user_ctx);
    self->rmt_running_ = false;
    if (self->producer_task_ != nullptr) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(self->producer_task_, &woken);
        return woken == pdTRUE;
    }
    return false;
}
