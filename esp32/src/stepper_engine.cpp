/* stepper_engine.cpp — Core 1 real-time stepper engine implementation.
 *
 * Three hardware timers (Timer Group 0 timer 0, Timer Group 0 timer 1,
 * Timer Group 1 timer 0) generate precise step pulses for each axis.
 * A FreeRTOS task handles command dispatch and endstop polling.
 */

#include "stepper_engine.h"
#include "sensor_task.h"
#include <driver/timer.h>
#include <esp_log.h>
#include <esp_timer.h>

static const char* TAG = "stepper";

// ── Singleton ───────────────────────────────────────────────────────────────
StepperEngine g_engine;

// ── Timer mapping ───────────────────────────────────────────────────────────
// Axis 0 → Timer Group 0, Timer 0
// Axis 1 → Timer Group 0, Timer 1
// Axis 2 → Timer Group 1, Timer 0
struct TimerDef {
    timer_group_t group;
    timer_idx_t   idx;
};

static constexpr TimerDef TIMER_MAP[NUM_AXES] = {
    {TIMER_GROUP_0, TIMER_0},
    {TIMER_GROUP_0, TIMER_1},
    {TIMER_GROUP_1, TIMER_0},
};

// ── Timer ISR handlers ──────────────────────────────────────────────────────
// Each ISR calls into the corresponding axis, gets the next interval,
// and re-arms the timer.  If interval == 0, the timer is stopped.
// These are free functions (not class members) to match timer_isr_register's
// expected signature: void (*)(void*).

static void IRAM_ATTR timer_isr_axis0(void*) {
    auto& ax = g_engine.axis(0);
    uint32_t next_iv = ax.step_isr();

    const auto& td = TIMER_MAP[0];
    TIMERG0.int_clr_timers.val = (1 << td.idx);

    if (next_iv > 0) {
        timer_set_alarm_value(td.group, td.idx, next_iv);
        timer_set_counter_value(td.group, td.idx, 0);
        timer_start(td.group, td.idx);
    }
}

static void IRAM_ATTR timer_isr_axis1(void*) {
    auto& ax = g_engine.axis(1);
    uint32_t next_iv = ax.step_isr();

    const auto& td = TIMER_MAP[1];
    TIMERG0.int_clr_timers.val = (1 << td.idx);

    if (next_iv > 0) {
        timer_set_alarm_value(td.group, td.idx, next_iv);
        timer_set_counter_value(td.group, td.idx, 0);
        timer_start(td.group, td.idx);
    }
}

static void IRAM_ATTR timer_isr_axis2(void*) {
    auto& ax = g_engine.axis(2);
    uint32_t next_iv = ax.step_isr();

    const auto& td = TIMER_MAP[2];
    TIMERG1.int_clr_timers.val = (1 << td.idx);

    if (next_iv > 0) {
        timer_set_alarm_value(td.group, td.idx, next_iv);
        timer_set_counter_value(td.group, td.idx, 0);
        timer_start(td.group, td.idx);
    }
}

// ISR function table for setup_timer
static void (*const TIMER_ISRS[NUM_AXES])(void*) = {
    timer_isr_axis0,
    timer_isr_axis1,
    timer_isr_axis2,
};

// ── Init ────────────────────────────────────────────────────────────────────

void StepperEngine::init(const AxisPins pins[NUM_AXES]) {
    for (uint8_t i = 0; i < NUM_AXES; ++i) {
        axes_[i].init(i, pins[i]);
        setup_timer(i, i);
    }
    ESP_LOGI(TAG, "Stepper engine initialized: %d axes, %d timers",
             NUM_AXES, NUM_AXES);
}

void StepperEngine::setup_timer(uint8_t timer_id, uint8_t axis_id) {
    const auto& td = TIMER_MAP[timer_id];

    timer_config_t cfg = {};
    cfg.alarm_en    = TIMER_ALARM_EN;
    cfg.counter_en  = TIMER_PAUSE;
    cfg.intr_type   = TIMER_INTR_LEVEL;
    cfg.counter_dir = TIMER_COUNT_UP;
    cfg.auto_reload = TIMER_AUTORELOAD_DIS;  // one-shot, ISR re-arms
    cfg.divider     = 2;  // 80 MHz APB / 2 = 40 MHz

    timer_init(td.group, td.idx, &cfg);
    timer_set_counter_value(td.group, td.idx, 0);
    timer_set_alarm_value(td.group, td.idx, 0xFFFFFFFF);  // disabled initially
    timer_enable_intr(td.group, td.idx);

    // Register ISR — free functions with void* signature
    timer_isr_register(td.group, td.idx, TIMER_ISRS[axis_id],
                       nullptr, ESP_INTR_FLAG_IRAM, nullptr);

    ESP_LOGD(TAG, "Timer %d/%d configured for axis %d (40 MHz, one-shot)",
             td.group, td.idx, axis_id);
}

void StepperEngine::start_timer(uint8_t axis_id) {
    if (axis_id >= NUM_AXES) return;
    const auto& td = TIMER_MAP[axis_id];
    uint32_t iv = axes_[axis_id].current_hz() > 0
                  ? TIMER_BASE_HZ / axes_[axis_id].current_hz()
                  : 0;
    if (iv == 0) return;

    timer_set_counter_value(td.group, td.idx, 0);
    timer_set_alarm_value(td.group, td.idx, iv);
    timer_start(td.group, td.idx);
}

void StepperEngine::stop_timer(uint8_t axis_id) {
    if (axis_id >= NUM_AXES) return;
    const auto& td = TIMER_MAP[axis_id];
    timer_pause(td.group, td.idx);
}

// ── Start task ──────────────────────────────────────────────────────────────

void StepperEngine::start(CmdQueue& cmd_queue) {
    // Use a struct to pass both pointers to the task
    struct TaskCtx {
        StepperEngine* engine;
        CmdQueue*      queue;
    };
    static TaskCtx ctx;
    ctx.engine = this;
    ctx.queue  = &cmd_queue;

    xTaskCreatePinnedToCore(
        [](void* param) {
            auto* c = static_cast<TaskCtx*>(param);
            c->engine->run(*c->queue);
        },
        "stepper",        // task name
        4096,             // stack size
        &ctx,             // parameter
        24,               // priority (high — only WiFi ISR is higher)
        &task_handle_,
        1                 // Core 1
    );

    ESP_LOGI(TAG, "Stepper task started on Core 1, priority 24");
}

// ── Main task loop ──────────────────────────────────────────────────────────

void StepperEngine::run(CmdQueue& cmd_queue) {
    TickType_t last_wake = xTaskGetTickCount();
    CmdFrame frame;

    for (;;) {
        // Process all pending commands (non-blocking drain)
        while (cmd_queue.pop(frame)) {
            dispatch_command(frame);
        }

        // Poll endstops (every ~1 ms from vTaskDelayUntil)
        for (uint8_t i = 0; i < NUM_AXES; ++i) {
            axes_[i].update_endstop();
        }

        // Update cached status frame (for SPI reads)
        StatusFrame sf = {};
        sf.global_flags = 0;
        sf.event_type   = static_cast<uint8_t>(EventType::NONE);
        sf.event_axis   = 0;
        sf.endstop_mask = 0;
        sf.uptime_ms    = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);

        for (uint8_t i = 0; i < NUM_AXES; ++i) {
            sf.axis[i].position   = axes_[i].position();
            sf.axis[i].current_hz = static_cast<uint16_t>(
                axes_[i].current_hz() > 0xFFFF ? 0xFFFF : axes_[i].current_hz());
            sf.axis[i].flags      = axes_[i].status_flags();
            sf.axis[i]._pad       = 0;

            if (axes_[i].endstop_active()) {
                sf.endstop_mask |= (1 << i);
            }
            if (axes_[i].is_enabled()) {
                sf.global_flags |= StatusFlags::ENABLED;
            }
            if (axes_[i].state() != AxisState::IDLE) {
                sf.global_flags |= StatusFlags::MOVING;
            }

            // Propagate first pending event
            if (axes_[i].event_pending() &&
                sf.event_type == static_cast<uint8_t>(EventType::NONE)) {
                sf.event_type = static_cast<uint8_t>(axes_[i].pending_event());
                sf.event_axis = i;
                sf.global_flags |= StatusFlags::EVENT_PENDING;
            }
        }

        // Sensor readings from sensor_task (Core 0) — read under spinlock
        portENTER_CRITICAL(&g_sensor.mux);
        sf.tension_raw[0]   = g_sensor.tension_dg[0];
        sf.tension_raw[1]   = g_sensor.tension_dg[1];
        sf.tension_setpoint = g_sensor.tension_setpoint;
        sf.pot_raw          = g_sensor.pot_raw;
        sf.encoder_manual   = g_sensor.encoder_manual;
        portEXIT_CRITICAL(&g_sensor.mux);
        sf.reserved[0]      = 0;
        sf.reserved[1]      = 0;

        // Thread-safe write of status
        portENTER_CRITICAL(&status_mux_);
        status_ = sf;
        portEXIT_CRITICAL(&status_mux_);

        // 1 ms tick
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

// ── Command dispatch ────────────────────────────────────────────────────────

void StepperEngine::dispatch_command(const CmdFrame& frame) {
    auto cmd  = static_cast<CmdOpcode>(frame.cmd);
    auto axis_id = frame.axis;

    // E-STOP is always global
    if (cmd == CmdOpcode::ESTOP) {
        emergency_stop();
        return;
    }

    // GET_STATUS is a no-op on engine side (SPI reads cached status_)
    if (cmd == CmdOpcode::GET_STATUS) return;

    // SET_MODE is engine-level
    if (cmd == CmdOpcode::SET_MODE) {
        set_winding_mode(frame.get_data_u32() != 0);
        return;
    }

    // SET_TENSION — store setpoint for forwarding to Pi via StatusFrame
    if (cmd == CmdOpcode::SET_TENSION) {
        sensor_set_tension_setpoint(
            static_cast<int16_t>(frame.get_data_u32() & 0xFFFF));
        return;
    }

    // TARE_HX711 — request tare from sensor_task (async, machine must be idle)
    if (cmd == CmdOpcode::TARE_HX711) {
        sensor_request_tare(static_cast<uint8_t>(frame.axis));
        return;
    }

    // Per-axis commands
    auto apply_to = [&](uint8_t id) {
        if (id >= NUM_AXES) return;
        Axis& ax = axes_[id];

        switch (cmd) {
        case CmdOpcode::SET_SPEED: {
            uint32_t hz = frame.get_data_u32();
            bool rev = (frame.flags & CmdFlags::DIR_REVERSE) != 0;
            ax.set_direction(rev);
            ax.set_speed_hz(hz);
            if (hz > 0 && ax.state() != AxisState::IDLE) {
                start_timer(id);
            }
            break;
        }
        case CmdOpcode::MOVE_ABS: {
            int32_t target = frame.get_data_i32();
            // MIGRATION: On BBB, move_to params came via host_cmd_t.
            // Here we use the axis default accel, and host sends cruise speed
            // in a preceding SET_SPEED or via flags.
            uint32_t cruise_hz = ax.current_hz();
            if (cruise_hz < HZ_MIN) cruise_hz = 2000;  // default cruise
            uint32_t accel_steps = (cruise_hz * cruise_hz) / (2 * ax.accel());
            ax.move_to(target, HZ_MIN, cruise_hz, accel_steps);
            start_timer(id);
            break;
        }
        case CmdOpcode::MOVE_REL: {
            int32_t delta = frame.get_data_i32();
            int32_t target = ax.position() + delta;
            uint32_t cruise_hz = ax.current_hz();
            if (cruise_hz < HZ_MIN) cruise_hz = 2000;
            uint32_t accel_steps = (cruise_hz * cruise_hz) / (2 * ax.accel());
            ax.move_to(target, HZ_MIN, cruise_hz, accel_steps);
            start_timer(id);
            break;
        }
        case CmdOpcode::STOP:
            ax.stop();
            break;
        case CmdOpcode::ENABLE:
            ax.set_enabled(frame.get_data_u32() != 0);
            break;
        case CmdOpcode::HOME: {
            // MIGRATION: On BBB, homing was managed by PRU0 orchestrator.
            // Here the axis itself handles homing via endstop callback.
            ax.set_direction(true);  // home direction = reverse
            ax.set_speed_hz(2000);   // homing speed
            start_timer(id);
            break;
        }
        case CmdOpcode::SET_ACCEL:
            ax.set_accel(frame.get_data_u32());
            break;
        case CmdOpcode::RESET_POS:
            ax.reset_position();
            break;
        case CmdOpcode::SET_LIMITS:
            if (frame.flags & CmdFlags::LIMIT_MAX) {
                ax.set_limit_max(frame.get_data_i32());
            } else {
                ax.set_limit_min(frame.get_data_i32());
            }
            break;
        case CmdOpcode::ACK_EVENT:
            ax.clear_event();
            break;
        default:
            ESP_LOGW(TAG, "Unknown command 0x%02X for axis %d", frame.cmd, id);
            break;
        }
    };

    if (axis_id == static_cast<uint8_t>(AxisId::ALL)) {
        for (uint8_t i = 0; i < NUM_AXES; ++i) {
            apply_to(i);
        }
    } else {
        apply_to(axis_id);
    }
}

// ── Emergency stop ──────────────────────────────────────────────────────────

void StepperEngine::emergency_stop() {
    for (uint8_t i = 0; i < NUM_AXES; ++i) {
        stop_timer(i);
        axes_[i].emergency_stop();
    }
    ESP_LOGW(TAG, "EMERGENCY STOP — all axes halted");
}

// ── Status snapshot ─────────────────────────────────────────────────────────

StatusFrame StepperEngine::get_status() const {
    // Thread-safe read via spinlock
    StatusFrame sf;
    portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&status_mux_));
    sf = status_;
    portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&status_mux_));
    return sf;
}
