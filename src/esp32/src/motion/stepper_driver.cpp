/**
 * @file stepper_driver.cpp
 * @brief Physical-layer RMT stepper driver — streaming simple_encoder impl.
 *
 * See stepper_driver.h for architecture details.
 */

#include "stepper_driver.h"

#include "../comm/messages.h"

#include <algorithm>
#include <cstring>
#include <esp_log.h>
#include <esp_check.h>
#include <hal/gpio_ll.h>

static const char* TAG = "stepper_driver";

static constexpr uint32_t COAST_IDLE_LIMIT = 250000;

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
        return 0;
    }

    if (drv->rmt_stopped_.load(std::memory_order_relaxed)) {
        *done = true;
        return 0;
    }

    if (drv->endstop_active_.load(std::memory_order_relaxed)) {
        drv->rmt_stopped_.store(true, std::memory_order_relaxed);
        *done = true;
        return 0;
    }

    uint32_t rd = drv->ring_read_.load(std::memory_order_acquire);
    uint32_t wr = drv->ring_write_.load(std::memory_order_acquire);

    if (rd == wr) {
        drv->last_chunk_had_steps_ = false;
        drv->coast_idle_count_++;

        if (drv->coast_idle_count_ >= COAST_IDLE_LIMIT) {
            drv->rmt_stopped_.store(true, std::memory_order_relaxed);
            *done = true;
            return 0;
        }

        {
            uint32_t last = drv->last_ticks_;
            if (last < MIN_CMD_TICKS) last = MIN_CMD_TICKS;
            uint16_t half = static_cast<uint16_t>(
                (last >> 1) > 32767u ? 32767u : (last >> 1));
            for (uint32_t i = 0; i < PART_SIZE; i++) {
                symbols[i].level0    = 0;
                symbols[i].duration0 = half;
                symbols[i].level1    = 0;
                symbols[i].duration1 = half;
            }
        }

        TaskHandle_t prod = drv->producer_task_.load(std::memory_order_relaxed);
        TaskHandle_t exec = drv->executor_task_.load(std::memory_order_relaxed);
        BaseType_t woken = pdFALSE;
        if (prod != nullptr) vTaskNotifyGiveFromISR(prod, &woken);
        if (exec != nullptr && exec != prod) vTaskNotifyGiveFromISR(exec, &woken);
        if (woken == pdTRUE) portYIELD_FROM_ISR();

        return PART_SIZE;
    }

    drv->coast_idle_count_ = 0;

    ring_entry_t* entry = &drv->ring_[rd & STEP_RING_MASK];
    if (entry->toggle_dir) {
        if (drv->last_chunk_had_steps_) {
            drv->last_chunk_had_steps_ = false;
            uint32_t last = drv->last_ticks_;
            if (last < MIN_CMD_TICKS) last = MIN_CMD_TICKS;
            uint16_t half = static_cast<uint16_t>(
                (last >> 1) > 32767u ? 32767u : (last >> 1));
            for (uint32_t i = 0; i < PART_SIZE; i++) {
                symbols[i].duration0 = half;
                symbols[i].level0    = 0;
                symbols[i].duration1 = half;
                symbols[i].level1    = 0;
            }
            return PART_SIZE;
        }
        // Apply hardware inversion here — target_dir is logical, GPIO is physical.
        gpio_ll_set_level(&GPIO, drv->dir_pin_,
            (entry->target_dir ^ (drv->invert_direction_ ? 1u : 0u)) ? 1 : 0);
        drv->setAppliedDirection(entry->target_dir != 0);  // store logical
        entry->toggle_dir = 0;
    }

    bool has_steps = false;
    for (uint32_t i = 0; i < PART_SIZE; i++) {
        if (rd != wr) {
            ring_entry_t* e = &drv->ring_[rd & STEP_RING_MASK];

            if (e->toggle_dir && i > 0) {
                uint32_t last = drv->last_ticks_;
                if (last < MIN_CMD_TICKS) last = MIN_CMD_TICKS;
                uint16_t half = static_cast<uint16_t>(
                    (last >> 1) > 32767u ? 32767u : (last >> 1));
                for (uint32_t j = i; j < PART_SIZE; j++) {
                    symbols[j].duration0 = half;
                    symbols[j].level0    = 0;
                    symbols[j].duration1 = half;
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
            drv->ring_underrun_count_.fetch_add(1, std::memory_order_relaxed);
            {
                uint32_t last = drv->last_ticks_;
                if (last < MIN_CMD_TICKS) last = MIN_CMD_TICKS;
                uint16_t half = static_cast<uint16_t>(
                    (last >> 1) > 32767u ? 32767u : (last >> 1));
                for (uint32_t j = i; j < PART_SIZE; j++) {
                    symbols[j].level0    = 0;
                    symbols[j].duration0 = half;
                    symbols[j].level1    = 0;
                    symbols[j].duration1 = half;
                }
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

namespace {

static inline StepperDriver::EndstopSignalState decodeEndstopSignalState(int no_lvl,
                                                                         int nc_lvl)
{
    if (no_lvl == 0 && nc_lvl == 1) {
        return StepperDriver::EndstopSignalState::CLOSED;
    }
    if (no_lvl == 1 && nc_lvl == 0) {
        return StepperDriver::EndstopSignalState::OPEN;
    }
    return StepperDriver::EndstopSignalState::INVALID;
}

} // namespace

uint8_t StepperDriver::reportedEndstopState() const
{
    if (endstop_no_pin_ == GPIO_NUM_NC || endstop_nc_pin_ == GPIO_NUM_NC) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }

    const EndstopSignalState raw = static_cast<EndstopSignalState>(
        endstop_signal_state_.load(std::memory_order_acquire));
    if (raw == EndstopSignalState::CLOSED) {
        return static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED);
    }
    if (raw == EndstopSignalState::OPEN) {
        return static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
    }

    const EndstopSignalState stable = static_cast<EndstopSignalState>(
        endstop_last_stable_state_.load(std::memory_order_acquire));
    return (stable == EndstopSignalState::CLOSED)
        ? static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED)
        : static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

bool StepperDriver::isEndstopMoveAllowed(bool direction) const
{
    if (!isEndstopArmed()) {
        return true;
    }

    const uint8_t state = reportedEndstopState();
    if (state == static_cast<uint8_t>(LateralEndstopState::ABSENT)) {
        return false;
    }

    if (state == static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN)) {
        return true;
    }

    // Armed + physically CLOSED:
    // - If we have a known post-hit clearance direction, only that direction
    //   is allowed.
    // - Otherwise (closed before any armed hit), reject movement fail-safe.
    if (state == static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED)) {
        if (!endstop_clearance_pending_.load(std::memory_order_acquire)) {
            return false;
        }
        const bool clearance_direction =
            endstop_clearance_direction_.load(std::memory_order_acquire);
        return direction == clearance_direction;
    }

    return true;
}

bool StepperDriver::prepareEndstopMove(bool direction)
{
    // FIX 2: permission check only; do not clear endstop_active_ here.
    return isEndstopMoveAllowed(direction);
}

void IRAM_ATTR StepperDriver::endstopIsrHandler(void* arg)
{
    StepperDriver* drv = static_cast<StepperDriver*>(arg);

    const int no_lvl = gpio_get_level(drv->endstop_no_pin_);
    const int nc_lvl = gpio_get_level(drv->endstop_nc_pin_);

    const EndstopSignalState raw = decodeEndstopSignalState(no_lvl, nc_lvl);
    drv->endstop_signal_state_.store(static_cast<uint8_t>(raw), std::memory_order_release);

    const TickType_t now_tick = xTaskGetTickCountFromISR();
    if (raw == EndstopSignalState::INVALID) {
        const TickType_t invalid_since =
            drv->endstop_invalid_since_tick_.load(std::memory_order_relaxed);
        if (invalid_since == 0) {
            drv->endstop_invalid_since_tick_.store(now_tick, std::memory_order_release);
        }
        return;
    }

    drv->endstop_last_stable_state_.store(static_cast<uint8_t>(raw), std::memory_order_release);
    drv->endstop_invalid_since_tick_.store(0, std::memory_order_release);

    if (raw == EndstopSignalState::OPEN) {
        drv->endstop_closed_confirmations_ = 0;
        drv->endstop_active_.store(false, std::memory_order_release);
        drv->endstop_clearance_pending_.store(false, std::memory_order_release);
        return;
    }

    if (!drv->isEndstopArmed()) {
        return;
    }

    const uint8_t confirmations = drv->endstop_closed_confirmations_;
    if (confirmations < ENDSTOP_CLOSED_CONFIRM_COUNT) {
        drv->endstop_closed_confirmations_ = static_cast<uint8_t>(confirmations + 1);
    }
    if (drv->endstop_closed_confirmations_ < ENDSTOP_CLOSED_CONFIRM_COUNT) {
        return;
    }

    const bool was_active = drv->endstop_active_.exchange(true, std::memory_order_acq_rel);
    if (!was_active) {
        drv->endstop_hit_count_.fetch_add(1, std::memory_order_relaxed);
    }
    drv->endstop_clearance_pending_.store(true, std::memory_order_release);
    drv->endstop_clearance_direction_.store(
        !drv->last_dir_commanded_.load(std::memory_order_acquire),
        std::memory_order_release);

    BaseType_t woken = pdFALSE;
    TaskHandle_t exec = drv->executor_task_.load(std::memory_order_relaxed);
    if (exec != nullptr) {
        vTaskNotifyGiveFromISR(exec, &woken);
    }
    if (woken) portYIELD_FROM_ISR();
}

esp_err_t StepperDriver::initEndstopIsr(gpio_num_t no_pin, gpio_num_t nc_pin)
{
    if (no_pin == GPIO_NUM_NC || nc_pin == GPIO_NUM_NC) {
        ESP_LOGI(TAG, "motor%u: endstop pins not configured — ISR not installed",
                 motor_id_);
        return ESP_OK;
    }

    endstop_no_pin_ = no_pin;
    endstop_nc_pin_ = nc_pin;

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

    const EndstopSignalState initial_state = decodeEndstopSignalState(
        gpio_get_level(no_pin), gpio_get_level(nc_pin));
    endstop_signal_state_.store(static_cast<uint8_t>(initial_state), std::memory_order_release);
    if (initial_state == EndstopSignalState::INVALID) {
        endstop_last_stable_state_.store(
            static_cast<uint8_t>(EndstopSignalState::OPEN),
            std::memory_order_release);
        endstop_invalid_since_tick_.store(xTaskGetTickCount(), std::memory_order_release);
    } else {
        endstop_last_stable_state_.store(static_cast<uint8_t>(initial_state), std::memory_order_release);
        endstop_invalid_since_tick_.store(0, std::memory_order_release);
    }

    ESP_LOGI(TAG, "motor%u: endstop ISR installed NO=GPIO%d NC=GPIO%d",
             motor_id_, (int)no_pin, (int)nc_pin);
    return ESP_OK;
}

StepperDriver::StepperDriver(gpio_num_t step_pin, gpio_num_t dir_pin,
                             gpio_num_t en_pin,   uint8_t    motor_id)
    : dir_pin_(dir_pin)
    , step_pin_(step_pin)
    , en_pin_(en_pin)
    , motor_id_(motor_id)
{
    memset(ring_, 0, sizeof(ring_));
}

esp_err_t StepperDriver::init()
{
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
    gpio_set_level(dir_pin_,
        (applied_dir_.load(std::memory_order_acquire) ^ invert_direction_) ? 1 : 0);

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

    rmt_simple_encoder_config_t enc_cfg = {};
    enc_cfg.callback       = encode_steps;
    enc_cfg.arg            = this;
    enc_cfg.min_chunk_size = PART_SIZE;

    err = rmt_new_simple_encoder(&enc_cfg, &encoder_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_new_simple_encoder failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

    tx_config_.loop_count              = 0;
    tx_config_.flags.eot_level         = 0;
    tx_config_.flags.queue_nonblocking = 1;

    rmt_tx_event_callbacks_t cbs = {};
    cbs.on_trans_done = &StepperDriver::on_trans_done_isr;
    err = rmt_tx_register_event_callbacks(channel_, &cbs, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor%u: rmt_tx_register_event_callbacks failed: %s", motor_id_, esp_err_to_name(err));
        return err;
    }

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
    coast_idle_count_ = 0;

    gpio_set_level(dir_pin_,
        (applied_dir_.load(std::memory_order_acquire) ^ invert_direction_) ? 1 : 0);
    // FIX 3: force STEP pin low on emergency stop.
    gpio_set_level(step_pin_, 0);

    ESP_LOGW(TAG, "motor%u: emergency stop", motor_id_);
}

void StepperDriver::stopStream()
{
    if (!rmt_running_.load(std::memory_order_relaxed)) return;

    rmt_stopped_.store(true, std::memory_order_release);

    rmt_tx_wait_all_done(channel_, pdMS_TO_TICKS(500));
    rmt_running_.store(false, std::memory_order_relaxed);
}

esp_err_t StepperDriver::startStream()
{
    rmt_running_.store(true, std::memory_order_release);
    rmt_stopped_.store(false, std::memory_order_release);
    last_chunk_had_steps_ = false;
    coast_idle_count_ = 0;

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

void StepperDriver::gracefulStop()
{
    rmt_stopped_.store(true, std::memory_order_release);
    rmt_running_.store(false, std::memory_order_relaxed);
}

esp_err_t StepperDriver::pushBlock(const step_block_t& block, TaskHandle_t caller_task)
{
    if (block.count == 0) {
        return ESP_OK;
    }

    producer_task_.store((caller_task != nullptr)
                     ? caller_task
                     : xTaskGetCurrentTaskHandle(),
                     std::memory_order_release);

    const uint32_t count = std::min<uint32_t>(block.count, STEP_BLOCK_SIZE);

    const bool new_dir = block.steps[0].direction;
    const bool applied_dir = applied_dir_.load(std::memory_order_acquire);
    bool need_toggle = (new_dir != applied_dir);

    if (!rmt_running_.load(std::memory_order_relaxed) &&
        ring_read_.load(std::memory_order_relaxed) == ring_write_.load(std::memory_order_relaxed) &&
        need_toggle)
        {
           ESP_LOGI(TAG, "motor%u: pushBlock new_dir=%d applied_dir=%d need_toggle=%d rmt_running=%d",
         motor_id_, (int)new_dir, (int)applied_dir, (int)need_toggle,
         (int)rmt_running_.load(std::memory_order_relaxed));
        gpio_set_level(dir_pin_, (new_dir ^ invert_direction_) ? 1 : 0);
        applied_dir_.store(new_dir, std::memory_order_release);  // logical
        need_toggle = false;
        }
    last_dir_commanded_.store(new_dir, std::memory_order_release);

    if (!prepareEndstopMove(new_dir)) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint32_t i = 0; i < count; i++) {
        // FIX 1: check latch before entering ringFree wait/write path.
        if (endstop_active_.load(std::memory_order_acquire)) {
            return ESP_ERR_INVALID_STATE;
        }

        static constexpr uint8_t PUSH_RETRY_MAX = 20;
        uint8_t push_retry_count = 0;

        while (ringFree() == 0) {
            if (endstop_active_.load(std::memory_order_acquire)) {
                return ESP_ERR_INVALID_STATE;
            }
            if (!rmt_running_.load(std::memory_order_acquire)) {
                esp_err_t kick_err = startStream();
                if (kick_err != ESP_OK) {
                    ESP_LOGW(TAG, "motor%u: pushBlock kick startStream: %s",
                             motor_id_, esp_err_to_name(kick_err));
                    ++push_retry_count;
                    if (push_retry_count >= PUSH_RETRY_MAX) {
                        ESP_LOGE(TAG,
                                 "motor%u: pushBlock timeout — ring full, RMT won't start",
                                 motor_id_);
                        return ESP_ERR_TIMEOUT;
                    }
                }
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
        e->target_dir = static_cast<uint8_t>(new_dir ? 1 : 0);

        ring_write_.store(wr + 1, std::memory_order_release);
    }

    return ESP_OK;
}

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
