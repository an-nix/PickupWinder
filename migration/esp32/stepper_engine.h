/* stepper_engine.h — FastAccelStepper-based stepper engine wrapper
 *
 * Provides a lightweight replacement engine that uses the FastAccelStepper
 * library.  The API is intentionally similar to the existing RMT engine so
 * the rest of the codebase can switch to this implementation with minimal
 * changes.
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include "protocol.h"
#include "command_queue.h"
#include "sensor_task.h"

#include <FastAccelStepper.h>

// Keep the same name as the RMT implementation so call-sites need minimal
// edits.  Do not remove the existing RMT sources — they are left intact.
static constexpr uint8_t RMT_NUM_AXES = 3;

// Axis pin configuration (kept compatible with rmt_stepper.h)
struct AxisPins {
    int8_t step;         // STEP output GPIO
    int8_t dir;          // DIR  output GPIO
    int8_t enable;       // EN   output GPIO (active LOW)
    int8_t endstop_no;   // Endstop NO contact (-1 = none)
    int8_t endstop_nc;   // Endstop NC contact (-1 = single-pin)
};

// Minimal axis state enum (keeps parity with RmtAxisState used by callers)
enum class RmtAxisState : uint8_t {
    IDLE     = 0,
    ACCEL    = 1,
    CRUISE   = 2,
    DECEL    = 3,
    CONSTANT = 4,
    HOMING   = 5,
};

class StepperAxis {
public:
    StepperAxis();

    void init(uint8_t id, const AxisPins& pins, FastAccelStepperEngine& engine);

    void set_enabled(bool en);
    void set_direction(bool reverse);
    bool is_enabled() const { return enabled_; }

    void set_speed_hz(uint32_t hz);
    void set_accel(uint32_t a);

    // Simplified position command (absolute, steps)
    void move_to(int32_t target);
    void stop();

    void emergency_stop();
    void emergency_stop_from_isr();

    void reset_position();
    void set_position(int32_t p);

    int32_t position() const;
    uint32_t current_hz() const;
    uint8_t status_flags() const;

    // Endstop helpers
    void update_endstop();
    void set_endstop_active(bool v) { endstop_active_ = v; }
    bool endstop_active() const { return endstop_active_; }

    // Events
    bool event_pending() const { return event_pending_; }
    EventType pending_event() const { return pending_event_; }
    void clear_event() { event_pending_ = false; pending_event_ = EventType::NONE; }

    RmtAxisState state() const { return state_; }

    // Called from the stepper task to apply an ISR-requested emergency stop
    void emergency_stop_from_task();

private:
    uint8_t id_ = 0;
    AxisPins pins_ = {};
    FastAccelStepper* stepper_ = nullptr;

    volatile int32_t position_ = 0;
    volatile bool direction_ = false;
    volatile bool enabled_ = false;
    volatile uint32_t speed_hz_ = 0;
    volatile uint32_t accel_ = 10000;

    volatile bool event_pending_ = false;
    volatile EventType pending_event_ = EventType::NONE;

    volatile bool endstop_active_ = false;
    volatile bool endstop_fault_ = false;

    // ISR -> task flag
    volatile bool emergency_flag_ = false;

    volatile RmtAxisState state_ = RmtAxisState::IDLE;
};

class StepperEngine {
public:
    StepperEngine();

    void init(const AxisPins pins[RMT_NUM_AXES]);
    void start(CmdQueue& cmd_queue);

    StatusFrame get_status() const;

    StepperAxis& axis(uint8_t id) { return axes_[id]; }

    void emergency_stop();

private:
    void run(CmdQueue& cmd_queue);
    void dispatch_command(const CmdFrame& frame);

    FastAccelStepperEngine engine_;
    StepperAxis axes_[RMT_NUM_AXES];

    TaskHandle_t task_handle_ = nullptr;
    StatusFrame status_ = {};
    portMUX_TYPE status_mux_ = portMUX_INITIALIZER_UNLOCKED;
    // Upload state for host-provided ramp segments (one per axis)
    struct RampSeg {
        uint32_t start_iv;
        int32_t  add;
        uint32_t count;
    };

    static constexpr size_t MAX_UPLOAD_SEGS = 128;
    struct UploadState {
        bool active = false;
        size_t expected = 0;
        size_t next_index = 0;
        int field_pos = 0; // 0=start_iv,1=add,2=count
        RampSeg segs[MAX_UPLOAD_SEGS];
    } upload_[RMT_NUM_AXES];
};

extern StepperEngine g_stepper_engine;
