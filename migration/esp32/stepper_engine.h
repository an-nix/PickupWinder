/* stepper_engine.h — Core 1 real-time stepper engine.
 *
 * Manages 3 hardware timers (one per axis) for jitter-free step generation.
 * The engine is a FreeRTOS task pinned to Core 1 at highest priority.
 * Timer ISRs generate STEP pulses; the task handles command dispatch,
 * endstop polling, and axis synchronization.
 *
 * Architecture:
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ Core 1                                                  │
 *   │   ┌──────────┐   ┌──────────┐   ┌──────────┐          │
 *   │   │ Timer 0  │   │ Timer 1  │   │ Timer 2  │          │
 *   │   │ Axis 0   │   │ Axis 1   │   │ Axis 2   │          │
 *   │   │ (bobbin) │   │ (lateral)│   │(tensioner)│          │
 *   │   └────┬─────┘   └────┬─────┘   └────┬─────┘          │
 *   │        │ ISR          │ ISR          │ ISR             │
 *   │        └──────┬───────┴──────┬───────┘                 │
 *   │               ▼              ▼                          │
 *   │         axis[].step_isr()                               │
 *   │                                                         │
 *   │   ┌──────────────────────────────────────┐              │
 *   │   │ stepper_task  (FreeRTOS, pri=24)     │              │
 *   │   │ - pop from CmdQueue                  │              │
 *   │   │ - dispatch commands to axes          │              │
 *   │   │ - poll endstops every 1 ms           │              │
 *   │   │ - update StatusFrame                 │              │
 *   │   └──────────────────────────────────────┘              │
 *   └─────────────────────────────────────────────────────────┘
 */

#pragma once

#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "axis.h"
#include "command_queue.h"
#include "protocol.h"

// ── Number of axes ──────────────────────────────────────────────────────────
static constexpr uint8_t NUM_AXES = 3;

// ── Engine class ────────────────────────────────────────────────────────────
class StepperEngine {
public:
    StepperEngine() = default;

    /// Initialize axes and hardware timers.  Must be called from setup().
    void init(const AxisPins pins[NUM_AXES]);

    /// Start the stepper task on Core 1.
    void start(CmdQueue& cmd_queue);

    /// Get a read-only snapshot of the current status (for SPI response).
    /// Thread-safe: reads volatile axis state.
    StatusFrame get_status() const;

    /// Access individual axes (for direct calls from command dispatch).
    Axis& axis(uint8_t id) { return axes_[id]; }
    const Axis& axis(uint8_t id) const { return axes_[id]; }

    /// Emergency stop — all axes immediate halt.
    void emergency_stop();

    /// Winding mode: synchronize lateral with bobbin.
    void set_winding_mode(bool sync) { winding_sync_ = sync; }
    bool winding_mode() const { return winding_sync_; }

private:
    // ── FreeRTOS task ───────────────────────────────────────────────────────
    static void stepper_task(void* param);
    void run(CmdQueue& cmd_queue);
    void dispatch_command(const CmdFrame& frame);

    // ── Timer management ────────────────────────────────────────────────────
    void setup_timer(uint8_t timer_id, uint8_t axis_id);
    void start_timer(uint8_t axis_id);
    void stop_timer(uint8_t axis_id);

    // ── Axis array ──────────────────────────────────────────────────────────
    Axis axes_[NUM_AXES];

    // ── Synchronization ─────────────────────────────────────────────────────
    bool winding_sync_ = false;

    // ── Task handle ─────────────────────────────────────────────────────────
    TaskHandle_t task_handle_ = nullptr;

    // ── Global status cache ─────────────────────────────────────────────────
    StatusFrame status_ = {};
    portMUX_TYPE status_mux_ = portMUX_INITIALIZER_UNLOCKED;
};

// Singleton — the engine is accessed from ISRs.
extern StepperEngine g_engine;
