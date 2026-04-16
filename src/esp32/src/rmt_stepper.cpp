/* rmt_stepper.cpp — RMT-based stepper implementation (ESP-IDF, no Arduino).
 *
 * Modeled on FastAccelStepper's StepperISR_idf4_esp32_rmt.cpp.
 *
 * Key design (identical to FAS):
 *   - Single mem_block_num = 1 per channel (64 items).
 *   - Buffer split: two halves of PART_SIZE (30) words + end marker at [60].
 *   - tx_conti_mode = 1 → continuous loop.  Set to 0 to stop.
 *   - TX_THR interrupt at PART_SIZE+1 → refill first half.
 *   - TX_END interrupt after end marker → refill second half (or stop).
 *   - FIFO disabled, mem_tx_wrap_en = 0.
 *   - ESP32 tick-lost compensation: last item of second half decremented by 1.
 *
 * Clock: APB 80 MHz / div=2 = 40 MHz → 25 ns/tick.
 * Step pulse: 80 ticks = 2 µs HIGH (A4988/DRV8825 min = 1 µs).
 */

#include "rmt_stepper.h"
#include "sensor_task.h"
#include <driver/gpio.h>
#include <driver/rmt.h>
#include <esp_private/periph_ctrl.h>
#include <soc/rmt_struct.h>
#include <soc/gpio_struct.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <cmath>
#include <cstring>

// RMTMEM: peripheral memory mapped by the ESP32 linker script.
extern rmt_mem_t RMTMEM;

static const char* TAG = "rmt_stepper";

// ── Singleton ────────────────────────────────────────────────────────────────
RmtEngine g_rmt_engine;

// ── Global axis pointer table (indexed by RMT channel number) ────────────────
static RmtAxis* s_axes[RMT_NUM_AXES] = {};

// ── RMT memory access macro (same as FAS) ────────────────────────────────────
#define FAS_RMT_MEM(ch) ((uint32_t*)RMTMEM.chan[ch].data32)

// ── Fast GPIO helpers (ISR-safe, direct register writes) ─────────────────────

static inline IRAM_ATTR void gpio_fast_set(int pin) {
    if (pin < 32) GPIO.out_w1ts     = (1U << pin);
    else          GPIO.out1_w1ts.val = (1U << (pin - 32));
}

static inline IRAM_ATTR void gpio_fast_clr(int pin) {
    if (pin < 32) GPIO.out_w1tc     = (1U << pin);
    else          GPIO.out1_w1tc.val = (1U << (pin - 32));
}

// ── RMT item encoding ────────────────────────────────────────────────────────
// Pack two (duration, level) sub-entries into one uint32_t.
// Low 16 bits sent first, high 16 bits second.
// Each sub-entry: bit15 = level, bits14:0 = duration.
static IRAM_ATTR inline uint32_t make_item(uint32_t dur0, uint32_t lvl0,
                                           uint32_t dur1, uint32_t lvl1) {
    return (dur0 & 0x7FFFu)
         | ((lvl0 & 1u) << 15)
         | ((dur1 & 0x7FFFu) << 16)
         | ((lvl1 & 1u) << 31);
}

// ══════════════════════════════════════════════════════════════════════════════
//  Global ISR — handles both TX_END and TX_THR for all channels
// ══════════════════════════════════════════════════════════════════════════════

// ISR — handles TX_END and TX_THR interrupts (both via direct register access).
// TX_END bit = channel number (bits 0..7 of int_st).
// TX_THR bit = 24 + channel number (bits 24..31 of int_st).
// Direct register access only — no function calls (must be IRAM-safe).
uint32_t isr_count = 0;
uint32_t thr_count = 0;
uint32_t end_count = 0;

static void IRAM_ATTR rmt_stepper_isr(void* /*arg*/) {
    uint32_t mask = RMT.int_st.val;
    RMT.int_clr.val = mask;
    isr_count++;

    for (uint8_t ch = 0; ch < RMT_NUM_AXES; ++ch) {
        RmtAxis* q = s_axes[ch];
        if (q == nullptr) continue;

        // TX_END: end of second half + end marker reached
        if (mask & (1U << ch)) {
            end_count++;
            if (q->_rmtStopped) {
                // Stop was requested — disable interrupts and mark as stopped
                RMT.int_ena.val &= ~((1U << ch) | (1U << (24 + ch)));
                q->_isRunning = false;
            } else {
                // Refill second half, then reset threshold to detect first half next
                q->fill_part(false);
                RMT.tx_lim_ch[ch].limit = RMT_PART_SIZE + 1;  // Reset to 31 for first half
            }
        }

        // TX_THR: first half consumed
        if (mask & (1U << (24 + ch))) {
            thr_count++;
            if (!q->_rmtStopped) {
                // Refill first half and set threshold to detect second half
                q->fill_part(true);
                RMT.tx_lim_ch[ch].limit = RMT_PART_SIZE * 2 + 1;  // Set to 61 for second half
            }
        }
    }
}

// ── GPIO configuration helpers ───────────────────────────────────────────────

static void configure_output(int pin) {
    if (pin < 0) return;
    gpio_config_t cfg = {};
    cfg.pin_bit_mask  = 1ULL << pin;
    cfg.mode          = GPIO_MODE_OUTPUT;
    cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
}

static void configure_input_pullup(int pin) {
    if (pin < 0) return;
    bool no_pull = (pin >= 34);
    gpio_config_t cfg = {};
    cfg.pin_bit_mask  = 1ULL << pin;
    cfg.mode          = GPIO_MODE_INPUT;
    cfg.pull_up_en    = no_pull ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
    cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
}

// ══════════════════════════════════════════════════════════════════════════════
//  Step encoder — static helpers (IRAM_ATTR)
// ══════════════════════════════════════════════════════════════════════════════

// Number of uint32_t words needed to encode one step at 'iv' ticks.
IRAM_ATTR int RmtAxis::items_per_step(uint32_t iv) {
    if (iv <= RMT_PULSE_TICKS) return 1;
    uint32_t gap = iv - RMT_PULSE_TICKS;
    if (gap <= RMT_MAX_ITEM_TICKS) return 1;    // fits in one word
    uint32_t remaining = gap - RMT_MAX_ITEM_TICKS;
    return 1 + (int)((remaining + 2 * RMT_MAX_ITEM_TICKS - 1)
                     / (2 * RMT_MAX_ITEM_TICKS));
}

// Encode one step into *data.  Returns number of words written.
// Word 0: HIGH(pulse) + LOW(first_gap)
// Word 1+: LOW(chunk_a) + LOW(chunk_b)  until gap is consumed.
IRAM_ATTR int RmtAxis::encode_step(uint32_t* data, uint32_t iv) {
    uint32_t gap = (iv > RMT_PULSE_TICKS) ? (iv - RMT_PULSE_TICKS) : 0u;
    int n = 0;

    // Word 0: step pulse HIGH + first low gap
    uint32_t first = (gap <= RMT_MAX_ITEM_TICKS) ? gap : RMT_MAX_ITEM_TICKS;
    gap -= first;
    data[n++] = make_item(RMT_PULSE_TICKS, 1, first, 0);

    // Additional LOW+LOW wait words
    while (gap > 0) {
        uint32_t a = (gap >= RMT_MAX_ITEM_TICKS) ? RMT_MAX_ITEM_TICKS : gap;
        gap -= a;
        uint32_t b = (gap >= RMT_MAX_ITEM_TICKS) ? RMT_MAX_ITEM_TICKS : gap;
        gap -= b;
        if (b == 0) b = 1;   // prevent dur=0 (end-marker sentinel); adds ≤ 25 ns
        data[n++] = make_item(a, 0, b, 0);
    }
    return n;
}

// ══════════════════════════════════════════════════════════════════════════════
//  RmtAxis::fill_part — ISR buffer fill (called from ISR and start_rmt)
// ══════════════════════════════════════════════════════════════════════════════

// Fill one half of the RMT buffer with step pulses or pauses.
// first_half=true: write to items [0 .. PART_SIZE-1]
// first_half=false: write to items [PART_SIZE .. 2*PART_SIZE-1]
//
// If the axis is IDLE or the ramp is exhausted, the "second half" fill
// triggers stop_rmt(false) to gracefully end the transmission.
IRAM_ATTR void RmtAxis::fill_part(bool first_half) {
    uint32_t* data = FAS_RMT_MEM((int)ch_);
    if (!first_half) {
        data += RMT_PART_SIZE;
    }

    // Check if we have motion to produce
    if (state_ == RmtAxisState::IDLE || interval_ == 0) {
        if (first_half) {
            // Fill first half with pauses to buy time
            for (uint32_t i = 0; i < RMT_PART_SIZE; i++) {
                data[i] = 0x00010001u * ((RMT_MIN_CMD_TICKS + 2 * RMT_PART_SIZE - 1)
                                         / (2 * RMT_PART_SIZE));
            }
        } else {
            // Second half with no data → stop
            stop_rmt(false);
        }
        return;
    }

    // Fill PART_SIZE words with steps
    uint32_t slot = 0;
    while (slot < RMT_PART_SIZE) {
        uint32_t iv = interval_;
        if (iv == 0 || state_ == RmtAxisState::IDLE) {
            // Pad rest with pauses
            while (slot < RMT_PART_SIZE) {
                data[slot++] = 0x00040004u;  // 4+4 = 8 ticks pause
            }
            break;
        }

        int need = items_per_step(iv);
        if (slot + (uint32_t)need > RMT_PART_SIZE) {
            // Not enough room for this step — pad remainder.
            // The step will be the first item of the next fill.
            while (slot < RMT_PART_SIZE) {
                data[slot++] = 0x00040004u;
            }
            break;
        }

        // Encode step
        slot += (uint32_t)encode_step(&data[slot], iv);

        // Advance position
        position_ = position_ + (direction_ ? -1 : 1);

        // Count-down finite move
        if (steps_remaining_ > 0) {
            steps_remaining_ = steps_remaining_ - 1;
            if (steps_remaining_ == 0) {
                move_active_   = false;
                state_         = RmtAxisState::IDLE;
                interval_      = 0;
                event_pending_ = true;
                pending_event_ = EventType::MOVE_COMPLETE;
                while (slot < RMT_PART_SIZE) {
                    data[slot++] = 0x00040004u;
                }
                break;
            }
        }

        // Soft limits
        if (has_limits_) {
            if (position_ <= limit_min_ || position_ >= limit_max_) {
                move_active_   = false;
                state_         = RmtAxisState::IDLE;
                interval_      = 0;
                event_pending_ = true;
                pending_event_ = EventType::LIMIT_HIT;
                while (slot < RMT_PART_SIZE) {
                    data[slot++] = 0x00040004u;
                }
                break;
            }
        }

        // Advance ramp
        advance_ramp_step();
    }

    // ESP32 tick-lost compensation (FAS: SUPPORT_ESP32_RMT_TICK_LOST).
    // In tx_conti_mode, one clk_div cycle gap is inserted on wrap-around
    // between end of second half and start of first half.  Compensate by
    // subtracting 1 tick from the last word of the second half.
    if (!first_half && slot > 0 && _isRunning) {
        // Only apply if we actually filled some items and transmission is active
        uint32_t last_item = data[RMT_PART_SIZE - 1];
        if (last_item != 0) {  // avoid underflow on zero/sentinel
            data[RMT_PART_SIZE - 1] = last_item - 1;
        }
    }
}

// ══════════════════════════════════════════════════════════════════════════════
//  RmtAxis::stop_rmt — graceful stop (FAS model)
// ══════════════════════════════════════════════════════════════════════════════

// Clear tx_conti_mode so the RMT stops at the end marker.
// Fill the buffer with short pauses to reach the end marker quickly.
// Set _rmtStopped so the TX_END ISR disables interrupts and marks not running.
IRAM_ATTR void RmtAxis::stop_rmt(bool both) {
    // Let RMT hit the end marker
    RMT.conf_ch[ch_].conf1.tx_conti_mode = 0;

    // Fill with short pauses
    uint32_t* data = FAS_RMT_MEM((int)ch_);
    uint32_t start = both ? 0 : RMT_PART_SIZE;
    uint32_t pause_val = 0x00010001u * ((RMT_MIN_CMD_TICKS + 61) / 62);
    for (uint32_t i = start; i < 2 * RMT_PART_SIZE; i++) {
        data[i] = pause_val;
    }
    // Ensure end marker
    data[2 * RMT_PART_SIZE] = 0;

    _rmtStopped = true;
}

// ══════════════════════════════════════════════════════════════════════════════
//  RmtAxis::init — per-channel initialisation (FAS model)
// ══════════════════════════════════════════════════════════════════════════════

void RmtAxis::init(uint8_t id, rmt_channel_t ch, const AxisPins& pins) {
    id_   = id;
    ch_   = ch;
    pins_ = pins;

    // GPIO setup
    configure_output(pins_.step);
    configure_output(pins_.dir);
    configure_output(pins_.enable);

    if (pins_.step   >= 0) gpio_set_level((gpio_num_t)pins_.step,   0);
    if (pins_.dir    >= 0) gpio_set_level((gpio_num_t)pins_.dir,    0);
    if (pins_.enable >= 0) gpio_set_level((gpio_num_t)pins_.enable, 1);  // disabled

    if (pins_.endstop_no >= 0) configure_input_pullup(pins_.endstop_no);
    if (pins_.endstop_nc >= 0) configure_input_pullup(pins_.endstop_nc);

    // Enable RMT peripheral (once for first channel)
    if (id == 0) {
        periph_module_enable(PERIPH_RMT_MODULE);
    }

    // Configure RMT channel using individual rmt_set_* calls (like FAS).
    // This avoids rmt_config() / rmt_driver_install() which register their
    // own ISR and conflict with our custom ISR.
    rmt_set_tx_intr_en(ch, false);
    rmt_set_tx_thr_intr_en(ch, false, RMT_PART_SIZE + 1);
    rmt_set_source_clk(ch, RMT_BASECLK_APB);
    rmt_set_clk_div(ch, RMT_CLK_DIV);
    rmt_set_mem_block_num(ch, 1);
    rmt_tx_stop(ch);
    rmt_rx_stop(ch);

    // Register ISR once (first channel), disable FIFO, disable wrap
    if (id == 0) {
        rmt_isr_register(rmt_stepper_isr, nullptr,
                         ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM, nullptr);
        RMT.apb_conf.fifo_mask      = 1;  // disable FIFO mode (direct memory)
        RMT.apb_conf.mem_tx_wrap_en = 0;  // NO wrap (FAS requirement)
    }

    // Idle output: LOW on step pin when not transmitting
    RMT.conf_ch[ch].conf1.idle_out_lv = 0;
    RMT.conf_ch[ch].conf1.idle_out_en = 1;

    // Connect step pin to RMT channel via GPIO matrix
    rmt_set_gpio(ch, RMT_MODE_TX, (gpio_num_t)pins_.step, false);

    // Clear buffer: fill with end marker
    uint32_t* mem = FAS_RMT_MEM((int)ch);
    for (uint32_t i = 0; i < RMT_BUF_SIZE; i++) {
        mem[i] = 0;
    }

    // Publish in global table
    s_axes[id] = this;

    _isRunning  = false;
    _rmtStopped = true;
    state_      = RmtAxisState::IDLE;
    position_   = 0;
    interval_   = 0;

    ESP_LOGI(TAG, "RmtAxis[%u] init: ch=%d step=%d dir=%d en=%d "
             "(PART_SIZE=%lu, end_marker@%lu)",
             id_, (int)ch_, pins_.step, pins_.dir, pins_.enable,
             (unsigned long)RMT_PART_SIZE, (unsigned long)(2 * RMT_PART_SIZE));
}

// ══════════════════════════════════════════════════════════════════════════════
//  RmtAxis::start_rmt — start transmission (FAS startQueue_rmt model)
// ══════════════════════════════════════════════════════════════════════════════

void RmtAxis::start_rmt() {
    ESP_LOGI(TAG, "[Axis %u] start_rmt: ch=%d, interval=%lu, state=%d",
             id_, (int)ch_, interval_, (int)state_);
    
    rmt_tx_stop(ch_);

    uint32_t* mem = FAS_RMT_MEM((int)ch_);

    // Pre-fill with debug pattern (like FAS)
    for (uint32_t i = 0; i < 2 * RMT_PART_SIZE; i += 2) {
        mem[i]     = 0x0fff8fffu;
        mem[i + 1] = 0x7fff8fffu;
    }
    // End marker
    mem[2 * RMT_PART_SIZE]     = 0;
    mem[2 * RMT_PART_SIZE + 1] = 0;

    _isRunning  = true;
    _rmtStopped = false;

    // Disable interrupts before setup
    rmt_set_tx_intr_en(ch_, false);
    rmt_set_tx_thr_intr_en(ch_, false, 0);

    // Fill both halves with actual step data
    fill_part(true);   // first half
    fill_part(false);  // second half
    ESP_LOGI(TAG, "[Axis %u] Buffers filled, enabling interrupts and starting TX", id_);

    // Enable threshold and end interrupts
    rmt_set_tx_thr_intr_en(ch_, true, RMT_PART_SIZE + 1);
    rmt_set_tx_intr_en(ch_, true);

    _rmtStopped = false;

    // Start: continuous mode + tx_start
    RMT.conf_ch[ch_].conf1.tx_conti_mode = 1;
    RMT.conf_ch[ch_].conf1.mem_rd_rst    = 1;
    RMT.conf_ch[ch_].conf1.mem_rd_rst    = 0;
    RMT.conf_ch[ch_].conf1.tx_start      = 1;
    
    ESP_LOGI(TAG, "[Axis %u] RMT started: tx_conti_mode=1, tx_start=1", id_);
}

void RmtAxis::force_stop_rmt() {
    stop_rmt(true);
    // Clear state
    portENTER_CRITICAL(&mux_);
    state_           = RmtAxisState::IDLE;
    interval_        = 0;
    ramp_add_        = 0;
    ramp_count_      = 0;
    steps_remaining_ = 0;
    move_active_     = false;
    n_segments_      = 0;
    seg_index_       = 0;
    portEXIT_CRITICAL(&mux_);
}

void RmtAxis::set_segments_and_move(const RmtRampSeg* segs, size_t n, int32_t target) {
    if (segs == nullptr || n == 0) return;

    portENTER_CRITICAL(&mux_);
    // copy segments (clamp to max)
    size_t nn = (n > RMT_MAX_RAMP_SEGS) ? RMT_MAX_RAMP_SEGS : n;
    n_segments_ = 0;
    for (size_t i = 0; i < nn; ++i) {
        segments_[i] = segs[i];
        n_segments_++;
    }

    // compute direction/steps remaining based on target
    int32_t cur_pos = position_;
    int32_t delta = target - cur_pos;
    set_direction(delta < 0);
    steps_remaining_ = (delta < 0) ? -delta : delta;
    move_target_ = target;
    move_active_ = true;

    // load first segment state
    seg_index_ = 0;
    if (n_segments_ > 0) {
        const RmtRampSeg& s = segments_[0];
        interval_   = s.start_iv;
        ramp_add_   = s.add;
        ramp_count_ = s.count;
        state_      = (s.add == 0) ? RmtAxisState::CONSTANT
                                    : (s.add < 0 ? RmtAxisState::ACCEL : RmtAxisState::DECEL);
    } else {
        interval_ = 0;
        ramp_add_ = 0;
        ramp_count_ = 0;
        state_ = RmtAxisState::IDLE;
    }
    portEXIT_CRITICAL(&mux_);

    // Start RMT to execute the prepared buffer
    start_rmt();
}

// ══════════════════════════════════════════════════════════════════════════════
//  GPIO helpers
// ══════════════════════════════════════════════════════════════════════════════

void RmtAxis::set_enabled(bool en) {
    enabled_ = en;
    if (pins_.enable >= 0)
        gpio_set_level((gpio_num_t)pins_.enable, en ? 0 : 1);
    if (!en) emergency_stop();
}

void RmtAxis::set_direction(bool reverse) {
    direction_ = reverse;
    if (pins_.dir >= 0) {
        gpio_set_level((gpio_num_t)pins_.dir, reverse ? 1 : 0);
        ESP_LOGI(TAG, "Axis %u DIR=%d", id_, reverse ? 1 : 0);
    }
}

// ══════════════════════════════════════════════════════════════════════════════
//  Speed helpers
// ══════════════════════════════════════════════════════════════════════════════

uint32_t RmtAxis::hz_to_iv(uint32_t hz) const {
    if (hz < HZ_MIN) hz = HZ_MIN;
    if (hz > HZ_MAX) hz = HZ_MAX;
    return RMT_CLK_HZ / hz;
}

uint32_t RmtAxis::current_hz() const {
    uint32_t iv = interval_;
    return (iv > 0) ? (RMT_CLK_HZ / iv) : 0;
}

uint8_t RmtAxis::status_flags() const {
    uint8_t f = 0;
    if (enabled_)                               f |= StatusFlags::ENABLED;
    if (state_ != RmtAxisState::IDLE)           f |= StatusFlags::MOVING;
    if (state_ == RmtAxisState::HOMING)         f |= StatusFlags::HOMING;
    if (endstop_active_)                        f |= StatusFlags::ENDSTOP_HIT;
    if (event_pending_) {
        f |= StatusFlags::EVENT_PENDING;
        if (pending_event_ == EventType::MOVE_COMPLETE) f |= StatusFlags::MOVE_COMPLETE;
        if (pending_event_ == EventType::SPEED_REACHED) f |= StatusFlags::SPEED_REACHED;
        if (pending_event_ == EventType::FAULT)         f |= StatusFlags::FAULT;
    }
    return f;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Ramp building (task context — float allowed)
// ══════════════════════════════════════════════════════════════════════════════

void RmtAxis::build_ramp_phase(uint32_t from_hz, uint32_t to_hz, uint32_t steps) {
    if (from_hz < HZ_MIN) from_hz = HZ_MIN;
    if (to_hz   < HZ_MIN) to_hz   = HZ_MIN;
    if (from_hz > HZ_MAX) from_hz = HZ_MAX;
    if (to_hz   > HZ_MAX) to_hz   = HZ_MAX;
    if (steps == 0) return;

    const double vs = (double)from_hz;
    const double ve = (double)to_hz;
    const double dv = (ve - vs) / (double)RMT_RAMP_N_SEG;

    uint32_t assigned = 0;
    for (size_t i = 0; i < RMT_RAMP_N_SEG && n_segments_ < RMT_MAX_RAMP_SEGS; ++i) {
        const double v0 = vs + (double)i       * dv;
        const double v1 = vs + (double)(i + 1) * dv;

        uint32_t iv0 = hz_to_iv((uint32_t)fabs(v0));
        uint32_t iv1 = hz_to_iv((uint32_t)fabs(v1));

        uint32_t seg_steps;
        if (i == RMT_RAMP_N_SEG - 1) {
            seg_steps = steps - assigned;
        } else {
            seg_steps = steps / RMT_RAMP_N_SEG;
            if (seg_steps == 0) seg_steps = 1;
        }
        assigned += seg_steps;

        int32_t add = (seg_steps > 1)
            ? ((int32_t)iv1 - (int32_t)iv0) / (int32_t)seg_steps
            : 0;

        segments_[n_segments_++] = RmtRampSeg{iv0, add, seg_steps};
    }
}

// ══════════════════════════════════════════════════════════════════════════════
//  Motion commands (task context)
// ══════════════════════════════════════════════════════════════════════════════

void RmtAxis::set_speed_hz(uint32_t hz) {
    if (hz == 0) { stop(); return; }

    ESP_LOGI(TAG, "[Axis %u] set_speed_hz(%lu), current_hz=%lu, state=%u, running=%d",
             id_, hz, current_hz(), (uint8_t)state_, _isRunning);

    if (state_ != RmtAxisState::IDLE && _isRunning) {
        // ── Hot-update path: axis is already running ─────────────────────────
        // Rebuild segments atomically under spinlock.  The ISR picks up the
        // new ramp on its next fill_part() call without any interruption.
        uint32_t cur_hz = current_hz();
        if (cur_hz == 0) cur_hz = HZ_MIN;

        uint32_t delta_hz    = (hz > cur_hz) ? (hz - cur_hz) : (cur_hz - hz);
        uint32_t trans_steps = (accel_ > 0)
            ? (uint32_t)((double)delta_hz / (double)accel_ + 0.5) : 1u;
        if (trans_steps < 1) trans_steps = 1;

        uint32_t iv_new = hz_to_iv(hz);

        portENTER_CRITICAL(&mux_);
        n_segments_ = 0;
        seg_index_  = 0;
        build_ramp_phase(cur_hz, hz, trans_steps);
        segments_[n_segments_++] = RmtRampSeg{iv_new, 0, 0xFFFFFFu};  // infinite cruise

        interval_        = segments_[0].start_iv;
        ramp_add_        = segments_[0].add;
        ramp_count_      = segments_[0].count;
        seg_index_       = 0;
        steps_remaining_ = 0;
        move_active_     = false;
        state_           = (hz >= cur_hz) ? RmtAxisState::ACCEL : RmtAxisState::DECEL;
        portEXIT_CRITICAL(&mux_);

        return;  // RMT keeps running — ISR consumes new segments seamlessly.
    }

    // ── Cold-start path: axis was IDLE ───────────────────────────────────────
    uint32_t cur_hz = HZ_MIN;
    uint32_t accel_steps = (uint32_t)(
        ((double)hz * hz - (double)cur_hz * cur_hz) / (2.0 * accel_) + 0.5);
    if (accel_steps < 1) accel_steps = 1;

    portENTER_CRITICAL(&mux_);
    n_segments_ = 0;
    seg_index_  = 0;
    build_ramp_phase(cur_hz, hz, accel_steps);

    uint32_t iv = hz_to_iv(hz);
    segments_[n_segments_++] = RmtRampSeg{iv, 0, 0xFFFFFFu};

    interval_        = segments_[0].start_iv;
    ramp_add_        = segments_[0].add;
    ramp_count_      = segments_[0].count;
    seg_index_       = 0;
    steps_remaining_ = 0;
    move_active_     = false;
    state_           = RmtAxisState::ACCEL;
    portEXIT_CRITICAL(&mux_);

    start_rmt();
}

void RmtAxis::move_to(int32_t target, uint32_t start_hz, uint32_t cruise_hz,
                      uint32_t /*accel_steps_per_s2*/) {
    int32_t delta = target - position_;
    if (delta == 0) {
        event_pending_ = true;
        pending_event_ = EventType::MOVE_COMPLETE;
        return;
    }

    set_direction(delta < 0);

    uint32_t total = (uint32_t)(delta < 0 ? -delta : delta);

    uint32_t accel_steps = (uint32_t)(
        ((double)cruise_hz * cruise_hz - (double)start_hz * start_hz)
        / (2.0 * accel_) + 0.5);
    if (accel_steps < 1) accel_steps = 1;
    if (accel_steps * 2 >= total) accel_steps = total / 2;
    uint32_t decel_steps  = accel_steps;
    uint32_t cruise_steps = total - accel_steps - decel_steps;

    portENTER_CRITICAL(&mux_);
    n_segments_  = 0;
    seg_index_   = 0;
    build_ramp_phase(start_hz, cruise_hz, accel_steps);
    if (cruise_steps > 0) {
        uint32_t iv = hz_to_iv(cruise_hz);
        segments_[n_segments_++] = RmtRampSeg{iv, 0, cruise_steps};
    }
    build_ramp_phase(cruise_hz, start_hz, decel_steps);

    interval_        = segments_[0].start_iv;
    ramp_add_        = segments_[0].add;
    ramp_count_      = segments_[0].count;
    seg_index_       = 0;
    steps_remaining_ = (int32_t)total;
    move_target_     = target;
    move_active_     = true;
    state_           = RmtAxisState::ACCEL;
    portEXIT_CRITICAL(&mux_);

    start_rmt();
}

void RmtAxis::stop() {
    if (state_ == RmtAxisState::IDLE) return;

    uint32_t cur_hz = current_hz();
    if (cur_hz <= HZ_MIN) {
        emergency_stop();
        return;
    }

    uint32_t decel_steps = (uint32_t)(
        ((double)cur_hz * cur_hz - (double)HZ_MIN * HZ_MIN) / (2.0 * accel_) + 0.5);
    if (decel_steps < 1) decel_steps = 1;

    portENTER_CRITICAL(&mux_);
    n_segments_  = 0;
    seg_index_   = 0;
    build_ramp_phase(cur_hz, HZ_MIN, decel_steps);

    uint32_t iv = hz_to_iv(HZ_MIN);
    segments_[n_segments_++] = RmtRampSeg{iv, 0, 1};

    interval_        = segments_[0].start_iv;
    ramp_add_        = segments_[0].add;
    ramp_count_      = segments_[0].count;
    seg_index_       = 0;
    steps_remaining_ = (int32_t)(decel_steps + 1);
    move_active_     = false;
    state_           = RmtAxisState::DECEL;
    portEXIT_CRITICAL(&mux_);
    // RMT continues running; ISR consumes decel ramp, then sets IDLE,
    // and fill_part(false) triggers stop_rmt().
}

void RmtAxis::emergency_stop() {
    force_stop_rmt();
    if (pins_.step >= 0) gpio_fast_clr(pins_.step);
}

IRAM_ATTR void RmtAxis::emergency_stop_from_isr() {
    // Direct register writes — always IRAM-safe.
    RMT.conf_ch[ch_].conf1.tx_conti_mode = 0;
    RMT.conf_ch[ch_].conf1.tx_start      = 0;
    RMT.conf_ch[ch_].conf1.mem_rd_rst    = 1;
    RMT.conf_ch[ch_].conf1.mem_rd_rst    = 0;

    portENTER_CRITICAL_ISR(&mux_);
    state_           = RmtAxisState::IDLE;
    interval_        = 0;
    ramp_add_        = 0;
    ramp_count_      = 0;
    steps_remaining_ = 0;
    move_active_     = false;
    n_segments_      = 0;
    seg_index_       = 0;
    _isRunning       = false;
    _rmtStopped      = true;
    portEXIT_CRITICAL_ISR(&mux_);

    if (pins_.step >= 0) gpio_fast_clr(pins_.step);
}

// ══════════════════════════════════════════════════════════════════════════════
//  ISR ramp helpers — all IRAM_ATTR
// ══════════════════════════════════════════════════════════════════════════════

IRAM_ATTR void RmtAxis::advance_ramp_step() {
    if (ramp_count_ == 0) return;

    interval_ = (uint32_t)((int32_t)interval_ + ramp_add_);

    const uint32_t iv_min = RMT_CLK_HZ / HZ_MAX;
    const uint32_t iv_max = RMT_CLK_HZ / HZ_MIN;
    if (interval_ < iv_min) interval_ = iv_min;
    if (interval_ > iv_max) interval_ = iv_max;

    ramp_count_ = ramp_count_ - 1;
    if (ramp_count_ == 0) {
        load_next_ramp_seg();
    }
}

IRAM_ATTR bool RmtAxis::load_next_ramp_seg() {
    portENTER_CRITICAL_ISR(&mux_);
    seg_index_ = seg_index_ + 1;
    if (seg_index_ < n_segments_) {
        const RmtRampSeg& s = segments_[seg_index_];
        interval_   = s.start_iv;
        ramp_add_   = s.add;
        ramp_count_ = s.count;

        if (s.add == 0) {
            state_ = (steps_remaining_ > 0) ? RmtAxisState::CRUISE
                                             : RmtAxisState::CONSTANT;
        } else if (s.add > 0) {
            state_ = RmtAxisState::DECEL;
        } else {
            state_ = RmtAxisState::ACCEL;
        }
        portEXIT_CRITICAL_ISR(&mux_);
        return true;
    }
    portEXIT_CRITICAL_ISR(&mux_);

    // All segments exhausted — keep infinite cruise by reloading last segment
    if (n_segments_ > 0 && seg_index_ >= n_segments_ && 
        steps_remaining_ == 0 && !move_active_) {
        // Pure speed command — reload the infinite cruise segment (last one)
        const RmtRampSeg& s = segments_[n_segments_ - 1];
        if (s.count == 0xFFFFFFu && s.add == 0) {
            // Valid infinite cruise — reload it
            portENTER_CRITICAL_ISR(&mux_);
            interval_   = s.start_iv;
            ramp_add_   = s.add;
            ramp_count_ = s.count;
            state_      = RmtAxisState::CONSTANT;
            portEXIT_CRITICAL_ISR(&mux_);
            // Log the reload (non-IRAM version of ESP_LOGI)
            // Can't call ESP_LOGI from ISR context, so we log via counter
            end_count += 0x10000;  // Special marker in high byte
            event_pending_ = true;
            pending_event_ = EventType::SPEED_REACHED;
            return true;  // Refilled, keep running
        }
    }

    ramp_add_   = 0;
    ramp_count_ = 0;
    if (steps_remaining_ == 0 && !move_active_) {
        state_ = RmtAxisState::CONSTANT;
        event_pending_ = true;
        pending_event_ = EventType::SPEED_REACHED;
    }
    return false;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Endstop
// ══════════════════════════════════════════════════════════════════════════════

void RmtAxis::update_endstop() {
    if (pins_.endstop_no < 0) return;

    bool no_triggered = (gpio_get_level((gpio_num_t)pins_.endstop_no) == 0);
    bool nc_open = (pins_.endstop_nc >= 0)
                   ? (gpio_get_level((gpio_num_t)pins_.endstop_nc) == 1)
                   : no_triggered;

    bool at_home  = no_triggered &&  nc_open;
    bool is_fault = no_triggered && !nc_open;

    if (no_triggered && !endstop_active_) {
        endstop_debounce_++;
        if (endstop_debounce_ >= ENDSTOP_DEBOUNCE_MS) {
            EventType ev = is_fault             ? EventType::FAULT
                         : (state_ == RmtAxisState::HOMING || at_home)
                                                 ? EventType::HOME_COMPLETE
                                                 : EventType::ENDSTOP_HIT;

            endstop_active_ = true;
            at_home_        = at_home;
            endstop_fault_  = is_fault;
            endstop_debounce_ = 0;

            emergency_stop();

            event_pending_ = true;
            pending_event_ = ev;
        }
    } else if (!no_triggered && endstop_active_) {
        endstop_debounce_++;
        if (endstop_debounce_ >= ENDSTOP_DEBOUNCE_MS) {
            endstop_active_   = false;
            at_home_          = false;
            endstop_fault_    = false;
            endstop_debounce_ = 0;
            event_pending_    = true;
            pending_event_    = EventType::ENDSTOP_CLEAR;
        }
    } else {
        endstop_debounce_ = 0;
    }
}

// ══════════════════════════════════════════════════════════════════════════════
//  RmtEngine
// ══════════════════════════════════════════════════════════════════════════════

void RmtEngine::init(const AxisPins pins[RMT_NUM_AXES]) {
    // Single mem_block_num = 1 → channels 0, 1, 2 (no stealing).
    static constexpr rmt_channel_t CH[RMT_NUM_AXES] = {
        RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2
    };

    for (uint8_t i = 0; i < RMT_NUM_AXES; ++i) {
        axes_[i].init(i, CH[i], pins[i]);
    }

    ESP_LOGI(TAG, "RmtEngine: %u axes, channels 0/1/2 "
             "(PART_SIZE=%lu, FAS model)", RMT_NUM_AXES,
             (unsigned long)RMT_PART_SIZE);
}

void RmtEngine::start(CmdQueue& cmd_queue) {
    struct Ctx { RmtEngine* e; CmdQueue* q; };
    static Ctx ctx;
    ctx.e = this;
    ctx.q = &cmd_queue;

    xTaskCreatePinnedToCore(
        [](void* p) {
            auto* c = static_cast<Ctx*>(p);
            c->e->run(*c->q);
        },
        "rmt_stepper", 4096, &ctx, 24, &task_handle_, 1);

    ESP_LOGI(TAG, "RmtEngine task started on Core 1, priority 24");
}

void RmtEngine::dispatch_command(const CmdFrame& frame) {
    if (frame.axis > RMT_NUM_AXES && frame.axis != 0xFF) return;

    auto apply = [&](uint8_t ax_id) {
        RmtAxis& ax = axes_[ax_id];
        uint32_t data_u = frame.get_data_u32();
        int32_t  data_s = frame.get_data_i32();

        switch ((CmdOpcode)frame.cmd) {
        case CmdOpcode::NOP:        break;
        case CmdOpcode::ENABLE:     ax.set_enabled(data_u != 0); break;
        case CmdOpcode::SET_SPEED:
            if (frame.flags & CmdFlags::DIR_REVERSE) ax.set_direction(true);
            else                                       ax.set_direction(false);
            ax.set_speed_hz(data_u);
            break;
        case CmdOpcode::MOVE_ABS:
            ax.move_to(data_s, HZ_MIN,
                       ax.current_hz() > 0 ? ax.current_hz() : (uint32_t)HZ_MIN,
                       ax.accel());
            break;
        case CmdOpcode::MOVE_REL:
            ax.move_to(ax.position() + data_s, HZ_MIN,
                       ax.current_hz() > 0 ? ax.current_hz() : (uint32_t)HZ_MIN,
                       ax.accel());
            break;
        case CmdOpcode::STOP:       ax.stop(); break;
        case CmdOpcode::ESTOP:      ax.emergency_stop(); break;
        case CmdOpcode::HOME:
            ax.set_direction(false);
            ax.set_speed_hz(HZ_MIN * 4);
            break;
        case CmdOpcode::SET_ACCEL:  ax.set_accel(data_u); break;
        case CmdOpcode::RESET_POS:  ax.reset_position(); break;
        case CmdOpcode::SET_LIMITS:
            if (frame.flags & CmdFlags::LIMIT_MAX) ax.set_limit_max(data_s);
            else                                    ax.set_limit_min(data_s);
            break;
        case CmdOpcode::ACK_EVENT:  ax.clear_event(); break;
        case CmdOpcode::GET_STATUS: break;
        default: break;
        }
    };

    if (frame.axis == 0xFF) {
        if ((CmdOpcode)frame.cmd == CmdOpcode::ESTOP) {
            emergency_stop();
        } else {
            for (uint8_t i = 0; i < RMT_NUM_AXES; ++i) apply(i);
        }
    } else {
        apply(frame.axis);
    }
}

void RmtEngine::run(CmdQueue& cmd_queue) {
    TickType_t last_wake = xTaskGetTickCount();
    CmdFrame   frame;

    for (;;) {
        while (cmd_queue.pop(frame)) {
            dispatch_command(frame);
        }

        for (uint8_t i = 0; i < RMT_NUM_AXES; ++i) {
            axes_[i].update_endstop();
        }

        StatusFrame sf = {};
        sf.uptime_ms    = (uint32_t)(esp_timer_get_time() / 1000ULL);
        sf.event_type   = (uint8_t)EventType::NONE;
        sf.event_axis   = 0;
        sf.endstop_mask = 0;

        for (uint8_t i = 0; i < RMT_NUM_AXES; ++i) {
            sf.axis[i].position   = axes_[i].position();
            sf.axis[i].current_hz = (uint16_t)(axes_[i].current_hz() > 0xFFFF
                                               ? 0xFFFF : axes_[i].current_hz());
            sf.axis[i].flags      = axes_[i].status_flags();
            sf.axis[i]._pad       = 0;

            if (axes_[i].endstop_active()) sf.endstop_mask |= (1u << i);

            if (axes_[i].event_pending() &&
                sf.event_type == (uint8_t)EventType::NONE) {
                sf.event_type   = (uint8_t)axes_[i].pending_event();
                sf.event_axis   = i;
                sf.global_flags |= StatusFlags::EVENT_PENDING;
            }
        }

        portENTER_CRITICAL(&g_sensor.mux);
        sf.tension_raw[0]   = g_sensor.tension_dg[0];
        sf.tension_raw[1]   = g_sensor.tension_dg[1];
        sf.tension_setpoint = g_sensor.tension_setpoint;
        sf.pot_raw          = g_sensor.pot_raw;
        sf.encoder_manual   = g_sensor.encoder_manual;
        portEXIT_CRITICAL(&g_sensor.mux);
        sf.reserved[0] = 0;
        sf.reserved[1] = 0;

        portENTER_CRITICAL(&status_mux_);
        status_ = sf;
        portEXIT_CRITICAL(&status_mux_);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

StatusFrame RmtEngine::get_status() const {
    StatusFrame sf;
    portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&status_mux_));
    sf = status_;
    portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&status_mux_));
    return sf;
}

void RmtEngine::emergency_stop() {
    for (uint8_t i = 0; i < RMT_NUM_AXES; ++i) {
        axes_[i].emergency_stop();
    }
}

// Apply externally-built ramp segments for an axis and start the RMT output.
void RmtEngine::apply_segments_and_move(uint8_t axis, const RmtRampSeg* segs, size_t n, int32_t target) {
    if (axis >= RMT_NUM_AXES) return;
    axes_[axis].set_segments_and_move(segs, n, target);
}


void RmtEngine::stepper_task(void* param) {
    (void)param;
}
