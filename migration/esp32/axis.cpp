/* axis.cpp — Per-axis implementation: GPIO, speed, ramp, step ISR.
 *
 * Framework: ESP-IDF (no Arduino).
 * GPIO controlled via IDF driver/gpio.h.
 * STEP pulse generation uses direct register writes for sub-µs ISR timing.
 *
 * IMPORTANT: GPIO 32-39 use the high GPIO bank registers (out1_w1ts/out1_w1tc).
 *   Axis 1 (Lateral): STEP=GPIO32 → must use GPIO.out1_w1ts.val, not out_w1ts.
 */

#include "axis.h"
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <cmath>

// ── Fast GPIO helpers (ISR-safe direct register writes) ──────────────────────
// gpio_set_level() is NOT IRAM_ATTR; use register writes in step_isr().

static inline IRAM_ATTR void gpio_fast_set(int pin) {
    if (pin < 32) {
        GPIO.out_w1ts = (1U << pin);
    } else {
        GPIO.out1_w1ts.val = (1U << (pin - 32));
    }
}

static inline IRAM_ATTR void gpio_fast_clr(int pin) {
    if (pin < 32) {
        GPIO.out_w1tc = (1U << pin);
    } else {
        GPIO.out1_w1tc.val = (1U << (pin - 32));
    }
}

// ── GPIO configuration helpers ────────────────────────────────────────────────

static void configure_output(int pin) {
    if (pin < 0) return;
    gpio_config_t cfg = {};
    cfg.pin_bit_mask  = (1ULL << pin);
    cfg.mode          = GPIO_MODE_OUTPUT;
    cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
}

static void configure_input_pullup(int pin) {
    if (pin < 0) return;
    // GPIO 34-39 are input-only and do not support internal pull-up/down.
    bool no_pullup = (pin >= 34);
    gpio_config_t cfg = {};
    cfg.pin_bit_mask  = (1ULL << pin);
    cfg.mode          = GPIO_MODE_INPUT;
    cfg.pull_up_en    = no_pullup ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
    cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
}

// ── Init ─────────────────────────────────────────────────────────────────────

void Axis::init(uint8_t id, const AxisPins& pins) {
    id_   = id;
    pins_ = pins;

    configure_output(pins_.step);
    if (pins_.step >= 0) gpio_fast_clr(pins_.step);   // STEP idle LOW

    configure_output(pins_.dir);
    if (pins_.dir >= 0)
        gpio_set_level(static_cast<gpio_num_t>(pins_.dir), 0);

    configure_output(pins_.enable);
    if (pins_.enable >= 0)
        gpio_set_level(static_cast<gpio_num_t>(pins_.enable), 1);  // disabled (active LOW)

    if (pins_.endstop_no >= 0) configure_input_pullup(pins_.endstop_no);
    if (pins_.endstop_nc >= 0) configure_input_pullup(pins_.endstop_nc);

    state_    = AxisState::IDLE;
    position_ = 0;
    interval_ = 0;
}

// ── GPIO control ─────────────────────────────────────────────────────────────

void Axis::set_enabled(bool en) {
    enabled_ = en;
    if (pins_.enable >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(pins_.enable), en ? 0 : 1);  // active LOW
    }
    if (!en) {
        state_    = AxisState::IDLE;
        interval_ = 0;
    }
}

void Axis::set_direction(bool reverse) {
    direction_ = reverse;
    if (pins_.dir >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(pins_.dir), reverse ? 1 : 0);
    }
}

// ── Speed helpers ───────────────────────────────────────────────────────────

uint32_t Axis::hz_to_interval(uint32_t hz) const {
    if (hz == 0) return 0;
    if (hz < HZ_MIN) hz = HZ_MIN;
    if (hz > HZ_MAX) hz = HZ_MAX;
    return TIMER_BASE_HZ / hz;
}

uint32_t Axis::current_hz() const {
    uint32_t iv = interval_;
    if (iv == 0) return 0;
    return TIMER_BASE_HZ / iv;
}

uint8_t Axis::status_flags() const {
    uint8_t f = 0;
    if (enabled_)                                f |= StatusFlags::ENABLED;
    if (state_ != AxisState::IDLE)               f |= StatusFlags::MOVING;
    if (state_ == AxisState::HOMING)             f |= StatusFlags::HOMING;
    if (endstop_active_)                         f |= StatusFlags::ENDSTOP_HIT;
    if (event_pending_) {
        f |= StatusFlags::EVENT_PENDING;
        if (pending_event_ == EventType::MOVE_COMPLETE) f |= StatusFlags::MOVE_COMPLETE;
        if (pending_event_ == EventType::SPEED_REACHED) f |= StatusFlags::SPEED_REACHED;
        if (pending_event_ == EventType::FAULT)         f |= StatusFlags::FAULT;
    }
    return f;
}

// ── Speed / motion commands ─────────────────────────────────────────────────

void Axis::set_speed_hz(uint32_t hz) {
    if (hz == 0) {
        stop();
        return;
    }

    uint32_t new_iv = hz_to_interval(hz);

    if (state_ == AxisState::IDLE) {
        // Start from zero — build acceleration ramp
        uint32_t start_hz = HZ_MIN;
        uint32_t steps_to_accel = (hz * hz - start_hz * start_hz) / (2 * accel_);
        if (steps_to_accel < 1) steps_to_accel = 1;

        build_simple_ramp(start_hz, hz, steps_to_accel);
        state_ = AxisState::ACCEL;
    } else if (ramp_count_ == 0) {
        // Currently at constant speed — immediate change
        // MIGRATION: On BBB, set_speed was a daemon-side ramp. Here we
        // allow immediate changes for small deltas, ramp for large ones.
        uint32_t cur_hz = current_hz();
        if (cur_hz > 0) {
            uint32_t delta = (hz > cur_hz) ? (hz - cur_hz) : (cur_hz - hz);
            if (delta > cur_hz / 4) {
                // Large speed change — ramp
                uint32_t steps = (delta * delta) / (2 * accel_) + 1;
                build_simple_ramp(cur_hz, hz, steps);
                state_ = AxisState::ACCEL;
            } else {
                // Small change — immediate
                interval_ = new_iv;
                state_ = AxisState::CONSTANT;
            }
        }
    }
    // If ramp is active (ramp_count_ > 0), new speed change is queued
    // by the caller via another set_speed_hz after ramp completes.
}

void Axis::arm_ramp(const RampSeg* segs, size_t n_segs) {
    if (n_segs == 0 || n_segs > MAX_RAMP_SEGS) return;

    // Copy segments
    for (size_t i = 0; i < n_segs; ++i) {
        segments_[i] = segs[i];
    }
    n_segments_ = n_segs;
    seg_index_  = 0;

    // Load first segment
    interval_   = segments_[0].start_iv;
    ramp_add_   = segments_[0].add;
    ramp_count_ = segments_[0].count;
    state_      = AxisState::ACCEL;
}

void Axis::move_to(int32_t target, uint32_t start_hz, uint32_t cruise_hz,
                    uint32_t accel_steps) {
    int32_t delta = target - position_;
    if (delta == 0) {
        event_pending_ = true;
        pending_event_ = EventType::MOVE_COMPLETE;
        return;
    }

    // Set direction
    set_direction(delta < 0);

    uint32_t total_steps = static_cast<uint32_t>(abs(delta));
    move_target_ = target;
    move_active_ = true;

    // Build trapezoidal profile
    if (accel_steps * 2 >= total_steps) {
        // Triangular profile: accel to midpoint, then decel
        accel_steps = total_steps / 2;
    }
    uint32_t decel_steps = accel_steps;
    uint32_t cruise_steps = total_steps - accel_steps - decel_steps;

    cruise_iv_ = hz_to_interval(cruise_hz);

    // Compute deceleration start position
    if (delta > 0) {
        decel_pos_ = target - static_cast<int32_t>(decel_steps);
    } else {
        decel_pos_ = target + static_cast<int32_t>(decel_steps);
    }

    // Build accel ramp (start → cruise)
    build_simple_ramp(start_hz, cruise_hz, accel_steps);
    state_ = AxisState::ACCEL;
}

void Axis::emergency_stop() {
    state_       = AxisState::IDLE;
    interval_    = 0;
    ramp_count_  = 0;
    n_segments_  = 0;
    move_active_ = false;
    if (pins_.step >= 0) gpio_fast_clr(pins_.step);
}

void Axis::stop() {
    if (state_ == AxisState::IDLE) return;

    // MIGRATION: On BBB, stop was immediate. Here we decelerate.
    uint32_t cur_hz = current_hz();
    if (cur_hz > HZ_MIN) {
        uint32_t steps = (cur_hz * cur_hz) / (2 * accel_) + 1;
        build_simple_ramp(cur_hz, HZ_MIN, steps);
        state_ = AxisState::DECEL;
    } else {
        state_    = AxisState::IDLE;
        interval_ = 0;
    }
}

// ── Timer ISR ───────────────────────────────────────────────────────────────

IRAM_ATTR uint32_t Axis::step_isr() {
    if (state_ == AxisState::IDLE || interval_ == 0) return 0;

    // STEP HIGH (direct register write: ISR-safe, works for all GPIO including 32+)
    if (pins_.step >= 0) gpio_fast_set(pins_.step);

    // Advance position
    position_ += direction_ ? -1 : 1;

    // Check move target
    if (move_active_) {
        bool reached = direction_ ? (position_ <= move_target_)
                                  : (position_ >= move_target_);
        if (reached) {
            // Move complete
            move_active_ = false;
            state_       = AxisState::IDLE;
            interval_    = 0;
            event_pending_ = true;
            pending_event_ = EventType::MOVE_COMPLETE;
            if (pins_.step >= 0) gpio_fast_clr(pins_.step);
            return 0;
        }

        // Check deceleration point
        bool should_decel = direction_ ? (position_ <= decel_pos_)
                                       : (position_ >= decel_pos_);
        if (should_decel && state_ == AxisState::CRUISE) {
            // Start decelerating — reverse the current ramp
            if (n_segments_ > 0 && seg_index_ > 0) {
                // Walk segments backwards
                seg_index_ = n_segments_ - 1;
                load_next_segment();
                ramp_add_ = -ramp_add_;  // reverse direction
            }
            state_ = AxisState::DECEL;
        }
    }

    if (has_limits_ && !move_active_) {
        if (position_ <= limit_min_ || position_ >= limit_max_) {
            state_         = AxisState::IDLE;
            interval_      = 0;
            event_pending_ = true;
            pending_event_ = EventType::LIMIT_HIT;
            if (pins_.step >= 0) gpio_fast_clr(pins_.step);
            return 0;
        }
    }

    // Advance ramp
    if (ramp_count_ > 0) {
        interval_ = static_cast<uint32_t>(
            static_cast<int32_t>(interval_) + ramp_add_);
        ramp_count_--;

        if (ramp_count_ == 0) {
            load_next_segment();

            if (n_segments_ == 0 && !move_active_) {
                    if (state_ == AxisState::DECEL) {
                        state_    = AxisState::IDLE;
                        interval_ = 0;
                        if (pins_.step >= 0) gpio_fast_clr(pins_.step);
                        return 0;
                    }
                    state_         = AxisState::CONSTANT;
                    event_pending_ = true;
                    pending_event_ = EventType::SPEED_REACHED;
                } else if (n_segments_ == 0 && move_active_) {
                    state_    = AxisState::CRUISE;
                    interval_ = cruise_iv_;
                }
            }
        }

    // STEP LOW (pulse width guaranteed by timer interval >> STEP_PULSE_US)
    if (pins_.step >= 0) gpio_fast_clr(pins_.step);
    // STEP LOW (pulse width guaranteed by timer interval >> STEP_PULSE_US)
    if (pins_.step >= 0) gpio_fast_clr(pins_.step);

    // return next interval for re-arming timer
    return interval_;
}

// ── Endstop ─────────────────────────────────────────────────────────────────

void Axis::update_endstop() {
    if (pins_.endstop_no < 0) return;

    // Read contacts.  NO: active = LOW (pulled to GND when triggered, pull-up).
    // NC: active = HIGH (open circuit when triggered, pull-up).
    bool no_triggered = (gpio_get_level(static_cast<gpio_num_t>(pins_.endstop_no)) == 0);
    // Single-pin mode: no NC contact → treat as always valid (no fault detection).
    bool nc_open      = (pins_.endstop_nc >= 0)
                        ? (gpio_get_level(static_cast<gpio_num_t>(pins_.endstop_nc)) == 1)
                        : no_triggered;

    // State interpretation:
    //   Away:    NO=HIGH (open),  NC=LOW  (closed)   → no_triggered=false
    //   At home: NO=LOW  (closed), NC=HIGH (open)    → at_home=true
    //   Fault:   NO=LOW  (closed), NC=LOW  (closed)  → fault=true (disconnected)
    bool at_home  = no_triggered &&  nc_open;
    bool is_fault = no_triggered && !nc_open;

    if (no_triggered && !endstop_active_) {
        endstop_debounce_++;
        if (endstop_debounce_ >= ENDSTOP_DEBOUNCE_MS) {
            // Determine event BEFORE changing state_
            EventType ev;
            if (is_fault) {
                ev = EventType::FAULT;
            } else if (state_ == AxisState::HOMING || at_home) {
                ev = EventType::HOME_COMPLETE;
            } else {
                ev = EventType::ENDSTOP_HIT;
            }

            endstop_active_ = true;
            at_home_        = at_home;
            endstop_fault_  = is_fault;
            endstop_debounce_ = 0;

            // Stop the motor unconditionally
            state_    = AxisState::IDLE;
            interval_ = 0;
            event_pending_ = true;
            pending_event_ = ev;
        }
    } else if (!no_triggered && endstop_active_) {
        endstop_debounce_++;
        if (endstop_debounce_ >= ENDSTOP_DEBOUNCE_MS) {
            endstop_active_ = false;
            at_home_        = false;
            endstop_fault_  = false;
            endstop_debounce_ = 0;
            event_pending_  = true;
            pending_event_  = EventType::ENDSTOP_CLEAR;
        }
    } else {
        endstop_debounce_ = 0;
    }
}

// ── Ramp helpers ────────────────────────────────────────────────────────────

void Axis::load_next_segment() {
    seg_index_++;
    if (seg_index_ < n_segments_) {
        interval_   = segments_[seg_index_].start_iv;
        ramp_add_   = segments_[seg_index_].add;
        ramp_count_ = segments_[seg_index_].count;
    } else {
        // No more segments
        n_segments_ = 0;
        seg_index_  = 0;
        ramp_count_ = 0;
        ramp_add_   = 0;
    }
}

void Axis::build_simple_ramp(uint32_t from_hz, uint32_t to_hz, uint32_t steps) {
    // Build a Klipper-style multi-segment ramp.
    // Divide velocity range into N_SEG uniform sub-segments.
    // MIGRATION: Replaces the BBB RampComputer::compute() which ran host-side.
    // On ESP32 we can afford this computation (240 MHz, FPU).

    if (from_hz < HZ_MIN) from_hz = HZ_MIN;
    if (to_hz   < HZ_MIN) to_hz   = HZ_MIN;
    if (from_hz > HZ_MAX) from_hz = HZ_MAX;
    if (to_hz   > HZ_MAX) to_hz   = HZ_MAX;

    constexpr size_t N_SEG = 16;
    n_segments_ = 0;
    seg_index_  = 0;

    if (steps == 0) return;

    double v_start = static_cast<double>(from_hz);
    double v_end   = static_cast<double>(to_hz);
    double dv_seg  = (v_end - v_start) / N_SEG;

    size_t total_assigned = 0;

    for (size_t i = 0; i < N_SEG && n_segments_ < MAX_RAMP_SEGS; ++i) {
        double vs = v_start + i * dv_seg;
        double ve = v_start + (i + 1) * dv_seg;

        uint32_t start_iv = hz_to_interval(static_cast<uint32_t>(fabs(vs)));
        uint32_t end_iv   = hz_to_interval(static_cast<uint32_t>(fabs(ve)));

        // Distribute steps proportionally
        uint32_t seg_steps;
        if (i == N_SEG - 1) {
            seg_steps = steps - total_assigned;  // remainder to last
        } else {
            seg_steps = steps / N_SEG;
        }
        if (seg_steps == 0) seg_steps = 1;
        total_assigned += seg_steps;

        int32_t add = 0;
        if (seg_steps > 1) {
            add = static_cast<int32_t>(end_iv) - static_cast<int32_t>(start_iv);
            add /= static_cast<int32_t>(seg_steps);
        }

        segments_[n_segments_++] = RampSeg{start_iv, add, seg_steps};
    }

    // Load first segment
    if (n_segments_ > 0) {
        interval_   = segments_[0].start_iv;
        ramp_add_   = segments_[0].add;
        ramp_count_ = segments_[0].count;
        seg_index_  = 0;
    }
}
