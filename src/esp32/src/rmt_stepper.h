/* rmt_stepper.h — RMT-based stepper motor driver for ESP32 (ESP-IDF).
 *
 * Implementation modeled on FastAccelStepper's proven RMT approach
 * (StepperISR_idf4_esp32_rmt.cpp).
 *
 * Buffer layout (per channel, 64 items = RMT_SIZE, mem_block_num=1):
 *   Items  0 .. PART_SIZE-1              = first half  (30 words)
 *   Items  PART_SIZE .. 2*PART_SIZE-1    = second half (30 words)
 *   Item   2*PART_SIZE                   = end marker  (0x00000000)
 *
 * Interrupt scheme (both TX_END and TX_THR):
 *   TX_THR fires after the first half is consumed (threshold = PART_SIZE+1).
 *     → ISR refills first half, resets threshold to 2*PART_SIZE+1.
 *   TX_END fires after second half reaches end marker.
 *     → If _rmtStopped: disable interrupts, set _isRunning=false.
 *     → Else: refill second half.
 *
 * tx_conti_mode = 1 during operation (continuous loop at end marker).
 * To stop: set tx_conti_mode = 0, fill buffer with short pauses,
 *          set _rmtStopped = true.  RMT hits end marker and fires TX_END.
 *
 * FIFO disabled (fifo_mask = 1), mem_tx_wrap_en = 0.
 * Single mem_block_num = 1 per channel → channels 0, 1, 2.
 *
 * Timing (40 MHz RMT clock):
 *   RMT_CLK_DIV     = 2       →  APB 80 MHz / 2 = 40 MHz, 25 ns/tick
 *   RMT_PULSE_TICKS = 80      →  2 µs HIGH pulse (A4988 / DRV8825 min = 1 µs)
 *   HZ_MAX = 160 000 Hz       →  250 ticks/step  → 1 item/step
 *   HZ_MIN = 100 Hz           →  400 000 ticks/step → ~7 items/step
 *
 * ESP32 tick-lost compensation (SUPPORT_ESP32_RMT_TICK_LOST):
 *   In tx_conti_mode, one clk_div cycle is inserted between the last item
 *   and the first item on wrap-around.  The second half's last item is
 *   decremented by 1 tick to compensate.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <esp_attr.h>
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "command_queue.h"
#include "protocol.h"

// ── Number of axes & speed limits ────────────────────────────────────────────
static constexpr uint8_t  RMT_NUM_AXES = 3;
static constexpr uint32_t HZ_MIN       = 100;      // ~0.9 RPM @ 6400 steps/rev
static constexpr uint32_t HZ_MAX       = 160'000;  // ~1500 RPM @ 6400 steps/rev

// ── Pin configuration ─────────────────────────────────────────────────────────
struct AxisPins {
    int8_t step;         // STEP output GPIO
    int8_t dir;          // DIR  output GPIO
    int8_t enable;       // EN   output GPIO (active LOW)
    int8_t endstop_no;   // Endstop NO contact (-1 = none)
    int8_t endstop_nc;   // Endstop NC contact (-1 = single-pin)
};

// ── RMT clock & timing constants ─────────────────────────────────────────────
static constexpr uint8_t  RMT_CLK_DIV        = 2;
static constexpr uint32_t RMT_CLK_HZ         = 40'000'000;
static constexpr uint32_t RMT_PULSE_TICKS    = 400;          // 10 µs HIGH pulse (was 80 ticks = 2µs)
static constexpr uint32_t RMT_MAX_ITEM_TICKS = 32767;        // 15-bit max

// ── FastAccelStepper-style buffer geometry ───────────────────────────────────
// RMT_SIZE = 64 items per channel (single mem_block_num = 1).
// PART_SIZE = ((RMT_SIZE - 1) / 4) << 1  =  30  (must be even).
// Buffer: [0..29] first half, [30..59] second half, [60] end marker.
static constexpr uint32_t RMT_BUF_SIZE  = 64;
static constexpr uint32_t RMT_PART_SIZE = 30;

// Minimum command ticks: ensures each fill is long enough for the ISR to
// refill the other half before the hardware wraps around.
// At 40 MHz: 8000 ticks = 200 µs.
static constexpr uint32_t RMT_MIN_CMD_TICKS = 8000;

// ── Ramp constants ────────────────────────────────────────────────────────────
static constexpr size_t RMT_MAX_RAMP_SEGS = 96;
static constexpr size_t RMT_RAMP_N_SEG    = 16;

// ── Ramp segment ─────────────────────────────────────────────────────────────
struct RmtRampSeg {
    uint32_t start_iv;  // starting interval (RMT clock ticks)
    int32_t  add;       // per-step interval delta (negative = accelerating)
    uint32_t count;     // steps in this segment
};

// ── Axis motion state ─────────────────────────────────────────────────────────
enum class RmtAxisState : uint8_t {
    IDLE     = 0,
    ACCEL    = 1,
    CRUISE   = 2,
    DECEL    = 3,
    CONSTANT = 4,
    HOMING   = 5,
};

// ── Per-axis RMT stepper ──────────────────────────────────────────────────────
class RmtAxis {
public:
    RmtAxis() = default;

    void init(uint8_t id, rmt_channel_t ch, const AxisPins& pins);

    // ── GPIO control ─────────────────────────────────────────────────────────
    void set_enabled(bool en);
    void set_direction(bool reverse);
    bool is_enabled() const { return enabled_; }

    // ── Motion commands (task context only) ──────────────────────────────────
    void set_speed_hz(uint32_t hz);
    void move_to(int32_t target, uint32_t start_hz, uint32_t cruise_hz,
                 uint32_t accel_steps_per_s2);
    void stop();
    void emergency_stop();
    void emergency_stop_from_isr();

    // ── Position & limits ────────────────────────────────────────────────────
    void reset_position()         { position_ = 0; }
    void set_position(int32_t p)  { position_ = p; }
    void set_limit_min(int32_t v) { limit_min_ = v; has_limits_ = true; }
    void set_limit_max(int32_t v) { limit_max_ = v; has_limits_ = true; }
    void clear_limits()           { has_limits_ = false; }

    // ── Acceleration ─────────────────────────────────────────────────────────
    void     set_accel(uint32_t a) { accel_ = a; }
    uint32_t accel() const         { return accel_; }

    // ── State accessors ──────────────────────────────────────────────────────
    int32_t       position()     const { return position_; }
    uint32_t      current_hz()   const;
    RmtAxisState  state()        const { return state_; }
    uint8_t       status_flags() const;
    bool          is_running()   const { return _isRunning; }
    uint8_t       id()           const { return id_; }

    // ── Endstop ──────────────────────────────────────────────────────────────
    void update_endstop();
    void set_endstop_active(bool v) { endstop_active_ = v; }
    bool endstop_active() const { return endstop_active_; }
    bool is_at_home()     const { return at_home_; }
    bool endstop_fault()  const { return endstop_fault_; }

    // ── Events ───────────────────────────────────────────────────────────────
    bool      event_pending() const { return event_pending_; }
    EventType pending_event() const { return pending_event_; }
    void      clear_event()   { event_pending_ = false; pending_event_ = EventType::NONE; }

    // ── RMT control (called from ISR and task) ───────────────────────────────
    void stop_rmt(bool both);
    void fill_part(bool first_half);

    // ── RMT state (public for ISR access) ────────────────────────────────────
    volatile bool _isRunning  = false;
    volatile bool _rmtStopped = true;

    rmt_channel_t channel() const { return ch_; }

private:
    void start_rmt();
    void force_stop_rmt();

    // ── ISR helpers ──────────────────────────────────────────────────────────
    void advance_ramp_step();
    bool load_next_ramp_seg();

    // ── Step encoder ─────────────────────────────────────────────────────────
    static int encode_step(uint32_t* data, uint32_t interval);
    static int items_per_step(uint32_t interval);

    // ── Ramp building (task context) ─────────────────────────────────────────
    void build_ramp_phase(uint32_t from_hz, uint32_t to_hz, uint32_t steps);
    uint32_t hz_to_iv(uint32_t hz) const;

    // ── Identity ─────────────────────────────────────────────────────────────
    uint8_t        id_  = 0;
    rmt_channel_t  ch_  = RMT_CHANNEL_0;
    AxisPins       pins_ = {};

    // ── Volatile motion state ────────────────────────────────────────────────
    volatile RmtAxisState state_           = RmtAxisState::IDLE;
    volatile int32_t      position_        = 0;
    volatile bool         direction_       = false;
    volatile bool         enabled_         = false;
    volatile uint32_t     interval_        = 0;
    volatile int32_t      ramp_add_        = 0;
    volatile uint32_t     ramp_count_      = 0;
    volatile int32_t      steps_remaining_ = 0;
    volatile size_t       seg_index_       = 0;

    // ── Ramp segment buffer ──────────────────────────────────────────────────
    RmtRampSeg  segments_[RMT_MAX_RAMP_SEGS] = {};
    size_t      n_segments_  = 0;

    // ── Move target ──────────────────────────────────────────────────────────
    volatile int32_t move_target_  = 0;
    volatile bool    move_active_  = false;

    // ── Limits ───────────────────────────────────────────────────────────────
    int32_t  limit_min_  = 0;
    int32_t  limit_max_  = 0;
    bool     has_limits_ = false;

    // ── Acceleration ─────────────────────────────────────────────────────────
    uint32_t accel_ = 10000;

    // ── Endstop ──────────────────────────────────────────────────────────────
    volatile bool endstop_active_ = false;
    volatile bool at_home_        = false;
    volatile bool endstop_fault_  = false;
    uint32_t      endstop_debounce_ = 0;
    static constexpr uint32_t ENDSTOP_DEBOUNCE_MS = 5;

    // ── Events ───────────────────────────────────────────────────────────────
    volatile bool      event_pending_ = false;
    volatile EventType pending_event_ = EventType::NONE;

    // ── Spinlock ─────────────────────────────────────────────────────────────
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

// ── Multi-axis RMT engine ─────────────────────────────────────────────────────
class RmtEngine {
public:
    RmtEngine() = default;

    void init(const AxisPins pins[RMT_NUM_AXES]);
    void start(CmdQueue& cmd_queue);
    StatusFrame get_status() const;

    RmtAxis&       axis(uint8_t id)       { return axes_[id]; }
    const RmtAxis& axis(uint8_t id) const { return axes_[id]; }

    void emergency_stop();
    void set_winding_mode(bool sync) { winding_sync_ = sync; }
    bool winding_mode()        const { return winding_sync_; }

private:
    static void stepper_task(void* param);
    void run(CmdQueue& cmd_queue);
    void dispatch_command(const CmdFrame& frame);

    RmtAxis      axes_[RMT_NUM_AXES];
    bool         winding_sync_ = false;
    TaskHandle_t task_handle_  = nullptr;
    StatusFrame  status_       = {};
    portMUX_TYPE status_mux_   = portMUX_INITIALIZER_UNLOCKED;
};

extern RmtEngine g_rmt_engine;
