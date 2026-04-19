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
// RMT clock: 80 MHz (1 tick = 12.5 ns).
// Called by the RMT driver whenever it needs more symbols. Reads up to
// PART_SIZE=16 entries from the ring buffer and converts each to one
// rmt_symbol_word_t with a FastAccelStepper-style balanced pulse:
//   HIGH = ticks / 2         (rounded down)
//   LOW  = ticks − HIGH
// Both halves are clamped to >= RMT_STEP_PULSE_TICKS (8) = 100 ns,
// which meets A4988/DRV8825 STEP pulse width requirements.
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

    uint32_t rd = drv->ring_read_.load(std::memory_order_acquire);
    uint32_t wr = drv->ring_write_.load(std::memory_order_acquire);

    // Check for explicit stop request
    if (drv->rmt_stopped_.load(std::memory_order_relaxed)) {
        *done = true;
        return 0;
    }

    // Check for endstop trigger — stop immediately without emitting any
    // further step pulses. The ring is NOT reset here; emergencyStop() is
    // called from the executor task after it detects endstop_active_.
    if (drv->endstop_active_.load(std::memory_order_relaxed)) {
        drv->rmt_stopped_.store(true, std::memory_order_relaxed);
        *done = true;
        return 0;
    }

    // Ring empty — emit one LOW-level pause chunk, arm stop, and let the
    // next callback terminate the transmission.
    if (rd == wr) {
        drv->last_chunk_had_steps_ = false;
        drv->ring_underrun_count_.fetch_add(1, std::memory_order_relaxed);
        drv->rmt_stopped_.store(true, std::memory_order_relaxed);
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
    drv->rmt_stopped_.store(false, std::memory_order_relaxed);

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
            drv->ring_underrun_count_.fetch_add(1, std::memory_order_relaxed);
            drv->rmt_stopped_.store(true, std::memory_order_relaxed);
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

    drv->ring_read_.store(rd, std::memory_order_release);
    drv->last_chunk_had_steps_ = has_steps;

    TaskHandle_t prod = drv->producer_task_.load(std::memory_order_relaxed);
    TaskHandle_t exec = drv->executor_task_.load(std::memory_order_relaxed);
    if (prod != nullptr || exec != nullptr) {
        BaseType_t woken = pdFALSE;
        if (prod != nullptr) {
            vTaskNotifyGiveFromISR(prod, &woken);
        }
        if (exec != nullptr && exec != prod) {
            vTaskNotifyGiveFromISR(exec, &woken);
        }
        if (woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    return PART_SIZE;
}

// ---------------------------------------------------------------------------
// endstopIsrHandler()  — GPIO ISR, IRAM_ATTR
// ---------------------------------------------------------------------------
//
// Fires on any edge of either endstop contact (NO or NC).
// Validates the dual-contact NO/NC logic to guard against noise and cable breaks:
//   NO=0, NC=1 → endstop CLOSED (triggered) → set endstop_active_
//   NO=1, NC=0 → endstop OPEN  (released)   → clear endstop_active_
//   NO==NC      → ABSENT or cable break       → fail-safe: set endstop_active_
//
// arg = StepperDriver* (owns all needed state — no CommInterface dependency).

void IRAM_ATTR StepperDriver::endstopIsrHandler(void* arg)
{
    StepperDriver* drv = static_cast<StepperDriver*>(arg);

    if (!drv->isEndstopArmed()) {
        return;
    }

    const int no_lvl = gpio_get_level(drv->endstop_no_pin_);
    const int nc_lvl = gpio_get_level(drv->endstop_nc_pin_);

    const bool triggered = (no_lvl == 0 && nc_lvl == 1) || (no_lvl == nc_lvl);

    if (triggered) {
        drv->endstop_active_.store(true, std::memory_order_release);
        // Wake the executor task so it drains the pipeline immediately.
        BaseType_t woken = pdFALSE;
        TaskHandle_t exec = drv->executor_task_.load(std::memory_order_relaxed);
        if (exec != nullptr) {
            vTaskNotifyGiveFromISR(exec, &woken);
        }
        if (woken) portYIELD_FROM_ISR();
    } else {
        // Endstop released — clear flag.
        // Host must re-arm via SPI ENABLE_ENDSTOP before the next move.
        drv->endstop_active_.store(false, std::memory_order_release);
    }
}

// ---------------------------------------------------------------------------
// initEndstopIsr()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::initEndstopIsr(gpio_num_t no_pin, gpio_num_t nc_pin)
{
    if (no_pin == GPIO_NUM_NC || nc_pin == GPIO_NUM_NC) {
        ESP_LOGI(TAG, "motor%u: endstop pins not configured — ISR not installed",
                 motor_id_);
        return ESP_OK;
    }

    endstop_no_pin_ = no_pin;
    endstop_nc_pin_ = nc_pin;

    // gpio_install_isr_service returns ESP_ERR_INVALID_STATE if already called.
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "motor%u: gpio_install_isr_service failed: %s",
                 motor_id_, esp_err_to_name(err));
        return err;
    }

    const gpio_num_t pins[2] = { no_pin, nc_pin };
    for (gpio_num_t pin : pins) {
        ESP_RETURN_ON_ERROR(
            gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE),
            TAG, "gpio_set_intr_type failed for pin %d", (int)pin);
        ESP_RETURN_ON_ERROR(
            gpio_isr_handler_add(pin, &StepperDriver::endstopIsrHandler, this),
            TAG, "gpio_isr_handler_add failed for pin %d", (int)pin);
    }

    ESP_LOGI(TAG, "motor%u: endstop ISR installed NO=GPIO%d NC=GPIO%d",
             motor_id_, (int)no_pin, (int)nc_pin);
    return ESP_OK;
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
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: gpio_config DIR/EN failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    gpio_set_level(en_pin_,  1);
    gpio_set_level(dir_pin_, last_dir_ ? 1 : 0);

    // ── 2. RMT TX channel ───────────────────────────────────────────────────
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num           = step_pin_;
    tx_cfg.clk_src            = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz      = RMT_STEP_RESOLUTION_HZ;
    tx_cfg.mem_block_symbols  = RMT_MEM_SYMBOLS;
    tx_cfg.trans_queue_depth  = 4;
    tx_cfg.flags.invert_out   = false;
    tx_cfg.flags.with_dma     = false;

    err = rmt_new_tx_channel(&tx_cfg, &channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_new_tx_channel failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    // ── 3. Simple encoder with callback ─────────────────────────────────────
    rmt_simple_encoder_config_t enc_cfg = {};
    enc_cfg.callback       = encode_steps;
    enc_cfg.arg            = this;
    enc_cfg.min_chunk_size = PART_SIZE;

    err = rmt_new_simple_encoder(&enc_cfg, &encoder_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_new_simple_encoder failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    // ── 4. Transmit config ──────────────────────────────────────────────────
    tx_config_.loop_count              = 0;
    tx_config_.flags.eot_level         = 0;
    tx_config_.flags.queue_nonblocking = 1;

    // ── 5. on_trans_done callback ───────────────────────────────────────────
    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = &StepperDriver::on_trans_done_isr;
    err = rmt_tx_register_event_callbacks(channel_, &cbs, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_tx_register_event_callbacks failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    // ── 6. Enable the RMT channel ───────────────────────────────────────────
    err = rmt_enable(channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_enable failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

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

    ring_read_.store(0, std::memory_order_relaxed);
    ring_write_.store(0, std::memory_order_relaxed);
    rmt_running_.store(false, std::memory_order_relaxed);
    rmt_stopped_.store(true, std::memory_order_relaxed);
    last_chunk_had_steps_ = false;

    ESP_LOGW(TAG, "motor%u: emergency stop", motor_id_);
}

// ---------------------------------------------------------------------------
// stopStream()
// ---------------------------------------------------------------------------

void StepperDriver::stopStream()
{
    if (!rmt_running_.load(std::memory_order_relaxed)) return;

    // Signal the encoder callback to end the transmission
    rmt_stopped_.store(true, std::memory_order_release);

    // Wait for the RMT hardware to finish the current transaction
    rmt_tx_wait_all_done(channel_, pdMS_TO_TICKS(500));
    rmt_running_.store(false, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// startStream()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::startStream()
{
    rmt_stopped_.store(false, std::memory_order_release);
    rmt_running_.store(true, std::memory_order_release);
    last_chunk_had_steps_ = false;

    // `this` is in internal DRAM (static global) — passes esp_ptr_internal()
    // check. sizeof(*this) > 0 passes payload_bytes != 0. The callback ignores
    // both data and data_size entirely. Reset the encoder so each new
    // transaction restarts from symbol position 0.
    encoder_->reset(encoder_);
    esp_err_t err = rmt_transmit(channel_, encoder_, this, sizeof(*this), &tx_config_);
    if (err != ESP_OK) {
        rmt_running_.store(false, std::memory_order_relaxed);
        rmt_stopped_.store(true, std::memory_order_relaxed);
        ESP_LOGE(TAG, "motor%u: rmt_transmit failed: %s",
                 motor_id_, esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------------------------------
// gracefulStop()
// ---------------------------------------------------------------------------

void StepperDriver::gracefulStop()
{
    // Signal the encoder callback to stop after the current ring contents
    // have been consumed (no ring reset, unlike emergencyStop).
    rmt_stopped_.store(true, std::memory_order_release);
    // Do not call rmt_tx_wait_all_done here — the caller should not block.
    // The RMT transaction will end naturally after the pause chunk fires.
    rmt_running_.store(false, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// pushBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperDriver::pushBlock(const step_block_t& block, TaskHandle_t caller_task)
{
    if (ring_underrun_count_.load(std::memory_order_relaxed) > 0) {
        ESP_LOGW(TAG, "motor%u: ring underrun x%lu since last pushBlock",
                 motor_id_, (unsigned long)ring_underrun_count_.load(std::memory_order_relaxed));
        ring_underrun_count_.store(0, std::memory_order_relaxed);
    }

    if (block.count == 0) {
        return ESP_OK;
    }

    // Always update producer_task_ unconditionally so the ISR ring-space
    // notification always wakes the task that is actually blocked here,
    // not a stale handle from a previous call.
    producer_task_.store((caller_task != nullptr)
                     ? caller_task
                     : xTaskGetCurrentTaskHandle(),
                     std::memory_order_release);

    const uint32_t count = std::min<uint32_t>(block.count, STEP_BLOCK_SIZE);

    const bool new_dir = block.steps[0].direction;
    bool need_toggle = (new_dir != last_dir_);

    // If the ring is empty and the motor is idle, explicitly set the DIR pin
    // to the requested direction now. This avoids relying on the initial
    // `last_dir_` state and ensures reverse mode is applied on the first block.
    if (!rmt_running_.load(std::memory_order_relaxed) &&
        ring_read_.load(std::memory_order_relaxed) == ring_write_.load(std::memory_order_relaxed) &&
        need_toggle) {
        gpio_set_level(dir_pin_, new_dir ? 1 : 0);
        last_dir_ = new_dir;
        need_toggle = false;
    } else {
        last_dir_ = new_dir;
    }

    // If the endstop already fired before we even start writing, abort.
    if (endstop_active_.load(std::memory_order_acquire)) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint32_t i = 0; i < count; i++) {
        // Back-pressure: wait until the encoder ISR has consumed at least one
        // chunk and notified this producer task. This avoids a CPU1 spin loop
        // and keeps the task watchdog satisfied.
        while (ringFree() == 0) {
            // The endstop ISR also notifies this task.  If it fires while we
            // are blocked here, break out immediately instead of spinning.
            if (endstop_active_.load(std::memory_order_acquire)) {
                return ESP_ERR_INVALID_STATE;
            }
            ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(5));
            if (endstop_active_.load(std::memory_order_acquire)) {
                return ESP_ERR_INVALID_STATE;
            }
        }

        uint32_t ticks = block.steps[i].interval_ticks;

        ticks = std::max<uint32_t>(ticks, RMT_STEP_MIN_TICKS);
        ticks = std::min<uint32_t>(ticks, RMT_STEP_MAX_TICKS);

        uint32_t wr = ring_write_.load(std::memory_order_relaxed);
        ring_entry_t* e = &ring_[wr & STEP_RING_MASK];
        e->ticks      = static_cast<uint16_t>(ticks);
        e->toggle_dir = (i == 0 && need_toggle) ? 1 : 0;
        e->pad        = 0;

        ring_write_.store(wr + 1, std::memory_order_release);
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
    self->rmt_running_.store(false, std::memory_order_relaxed);
    BaseType_t woken = pdFALSE;
    TaskHandle_t prod = self->producer_task_.load(std::memory_order_relaxed);
    if (prod != nullptr) {
        vTaskNotifyGiveFromISR(prod, &woken);
    }
    TaskHandle_t exec = self->executor_task_.load(std::memory_order_relaxed);
    if (exec != nullptr && exec != prod) {
        vTaskNotifyGiveFromISR(exec, &woken);
    }
    return woken == pdTRUE;
}
