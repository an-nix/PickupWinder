/* axis.h — Per-axis state: position, speed, trapezoidal acceleration profile.
 *
 * Each Axis owns its hardware timer and STEP/DIR/EN pins.
 * The stepper engine calls axis.tick() from the timer ISR at exactly the
 * step interval; tick() toggles STEP and advances position.
 *
 * Acceleration uses Klipper-style multi-segment ramps:
 *   interval += add   each step
 *   when count == 0   load next segment
 * Segments are pre-computed by the host (RPi) or on-ESP for simple ramps.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <esp_attr.h>
#include "protocol.h"

// ── Hardware constants ──────────────────────────────────────────────────────
// Timer base clock: 80 MHz (APB clock) with divider=1 → 80 MHz tick.
// We use divider=2 → 40 MHz tick → 25 ns resolution, max ~107 s interval.
static constexpr uint32_t TIMER_BASE_HZ  = 40000000;  // 40 MHz after /2 prescaler
static constexpr uint32_t STEP_PULSE_US  = 2;         // minimum STEP pulse width

// Speed limits (in Hz, step frequency)
static constexpr uint32_t HZ_MIN =   100;   // ~0.9 RPM for 6400 steps/rev
static constexpr uint32_t HZ_MAX = 160000;  // ~1500 RPM for 6400 steps/rev

// ── Ramp segment ────────────────────────────────────────────────────────────
// Identical concept to BBB pru_ipc.h ramp_seg_t.
struct RampSeg {
    uint32_t start_iv;   // starting interval (timer ticks)
    int32_t  add;        // interval delta per step (signed)
    uint32_t count;      // steps in this segment
};

static constexpr size_t MAX_RAMP_SEGS = 64;

// ── Axis motion state ───────────────────────────────────────────────────────
enum class AxisState : uint8_t {
    IDLE      = 0,    // not stepping
    CONSTANT  = 1,    // running at constant speed
    ACCEL     = 2,    // executing ramp segment (accelerating)
    CRUISE    = 3,    // trapezoidal cruise phase
    DECEL     = 4,    // decelerating to stop
    HOMING    = 5,    // homing approach (slow move until endstop)
};

// ── Pin configuration ───────────────────────────────────────────────────────
struct AxisPins {
    int8_t step;        // GPIO for STEP
    int8_t dir;         // GPIO for DIR
    int8_t enable;      // GPIO for ENABLE (active LOW)
    int8_t endstop_no;  // Normally-open contact: LOW when triggered (pull-up). -1 = none.
    int8_t endstop_nc;  // Normally-closed contact: HIGH when triggered (pull-up). -1 = single-pin.
                        // At home: NO=LOW AND NC=HIGH. Fault: NO=LOW AND NC=LOW.
};

// ── Per-axis class ──────────────────────────────────────────────────────────
class Axis {
public:
    // ── Construction ────────────────────────────────────────────────────────
    Axis() = default;
    void init(uint8_t id, const AxisPins& pins);

    // ── GPIO control ────────────────────────────────────────────────────────
    void set_enabled(bool en);
    void set_direction(bool reverse);
    bool is_enabled() const { return enabled_; }

    // ── Speed / motion commands ─────────────────────────────────────────────
    /// Set constant speed (Hz). Computes interval from TIMER_BASE_HZ.
    /// If axis is running, transitions smoothly.
    void set_speed_hz(uint32_t hz);

    /// Arm a multi-segment ramp.  Segments are loaded sequentially.
    void arm_ramp(const RampSeg* segs, size_t n_segs);

    /// Move to absolute position with trapezoidal profile.
    /// start_hz / cruise_hz / accel_steps are used to build the ramp.
    void move_to(int32_t target, uint32_t start_hz, uint32_t cruise_hz,
                 uint32_t accel_steps);

    /// Immediate stop (e-stop): zero speed, disable stepping.
    void emergency_stop();

    /// Controlled stop: decelerate to zero using current accel profile.
    void stop();

    // ── Timer ISR (called from stepper_engine) ──────────────────────────────
    /// Called at each step edge.  Returns next interval in timer ticks.
    /// IRAM_ATTR — must live in IRAM for ISR safety.
    IRAM_ATTR uint32_t step_isr();

    // ── State accessors (read from any core) ────────────────────────────────
    int32_t   position()      const { return position_; }
    uint32_t  current_hz()    const;
    AxisState state()         const { return state_; }
    uint8_t   status_flags()  const;
    bool      endstop_active() const { return endstop_active_; }
    uint8_t   id()            const { return id_; }

    // ── Position management ─────────────────────────────────────────────────
    void reset_position() { position_ = 0; }
    void set_position(int32_t pos) { position_ = pos; }

    // ── Limit management ────────────────────────────────────────────────────
    void set_limit_min(int32_t v) { limit_min_ = v; has_limits_ = true; }
    void set_limit_max(int32_t v) { limit_max_ = v; has_limits_ = true; }
    void clear_limits()           { has_limits_ = false; }

    // ── Acceleration tuning ─────────────────────────────────────────────────
    void set_accel(uint32_t steps_per_s2) { accel_ = steps_per_s2; }
    uint32_t accel() const { return accel_; }

    // ── Endstop ─────────────────────────────────────────────────────────────
    void update_endstop();   // poll endstop GPIO, debounce
    void set_endstop_active(bool v) { endstop_active_ = v; }
    bool      is_at_home()     const { return at_home_; }
    bool      endstop_fault()  const { return endstop_fault_; }

    // ── Events ──────────────────────────────────────────────────────────────
    bool     event_pending()    const { return event_pending_; }
    EventType pending_event()   const { return pending_event_; }
    void     clear_event()            { event_pending_ = false; pending_event_ = EventType::NONE; }

private:
    // ── Ramp helpers ────────────────────────────────────────────────────────
    void load_next_segment();
    void build_simple_ramp(uint32_t from_hz, uint32_t to_hz, uint32_t steps);
    uint32_t hz_to_interval(uint32_t hz) const;

    // ── Identity & pins ─────────────────────────────────────────────────────
    uint8_t   id_        = 0;
    AxisPins  pins_      = {-1, -1, -1, -1};

    // ── Motion state ────────────────────────────────────────────────────────
    volatile AxisState state_     = AxisState::IDLE;
    volatile int32_t   position_  = 0;
    volatile bool      direction_ = false;  // false=forward, true=reverse
    volatile bool      enabled_   = false;

    // ── Current step timing ─────────────────────────────────────────────────
    volatile uint32_t  interval_  = 0;      // current timer interval (ticks)
    volatile int32_t   ramp_add_  = 0;      // current segment add
    volatile uint32_t  ramp_count_ = 0;     // steps remaining in segment

    // ── Ramp segment buffer ─────────────────────────────────────────────────
    RampSeg  segments_[MAX_RAMP_SEGS] = {};
    size_t   n_segments_    = 0;
    size_t   seg_index_     = 0;

    // ── Trapezoidal move ────────────────────────────────────────────────────
    int32_t  move_target_   = 0;
    bool     move_active_   = false;
    uint32_t cruise_iv_     = 0;
    uint32_t decel_pos_     = 0;  // position at which to start deceleration

    // ── Limits ──────────────────────────────────────────────────────────────
    int32_t  limit_min_     = 0;
    int32_t  limit_max_     = 0;
    bool     has_limits_    = false;

    // ── Acceleration ────────────────────────────────────────────────────────
    uint32_t accel_         = 10000;  // steps/s² default

    // ── Endstop ─────────────────────────────────────────────────────────────
    volatile bool endstop_active_ = false;
    volatile bool at_home_         = false;  // valid home position (NO=LOW, NC=HIGH)
    volatile bool endstop_fault_   = false;  // sensor fault (NO=LOW, NC=LOW)
    uint32_t endstop_debounce_    = 0;
    static constexpr uint32_t ENDSTOP_DEBOUNCE_MS = 5;

    // ── Events ──────────────────────────────────────────────────────────────
    volatile bool      event_pending_  = false;
    volatile EventType pending_event_  = EventType::NONE;
};
