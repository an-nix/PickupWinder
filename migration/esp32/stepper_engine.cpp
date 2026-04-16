#include "stepper_engine.h"
#include "rmt_stepper.h"
#include <Arduino.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <cstring>
#include <driver/gpio.h>

static const char* TAG = "stepper_engine";

StepperEngine g_stepper_engine;

StepperAxis::StepperAxis() {}

void StepperAxis::init(uint8_t id, const AxisPins& pins, FastAccelStepperEngine& engine) {
    id_ = id;
    pins_ = pins;

    // Configure enable pin (active LOW)
    if (pins_.enable >= 0) {
        digitalWrite(pins_.enable, HIGH); // disabled
        pinMode(pins_.enable, OUTPUT);
        enabled_ = false;
    }

    // Connect FastAccelStepper to STEP pin
    stepper_ = engine.stepperConnectToPin(static_cast<uint8_t>(pins_.step));
    if (!stepper_) {
        ESP_LOGE(TAG, "Axis %u: failed to allocate FastAccelStepper on pin %d", id_, pins_.step);
    } else {
        stepper_->setDirectionPin(pins_.dir);
        stepper_->setAcceleration(accel_);
        stepper_->setSpeedInHz(speed_hz_);
    }

    position_ = (stepper_) ? stepper_->getCurrentPosition() : 0;
    state_ = RmtAxisState::IDLE;
}

void StepperAxis::set_enabled(bool en) {
    enabled_ = en;
    if (pins_.enable >= 0) {
        // EN is active LOW
        digitalWrite(pins_.enable, en ? LOW : HIGH);
    }
    if (!en) emergency_stop();
}

void StepperAxis::set_direction(bool reverse) {
    direction_ = reverse;
    if (pins_.dir >= 0) {
        digitalWrite(pins_.dir, reverse ? HIGH : LOW);
    }
}

void StepperAxis::set_speed_hz(uint32_t hz) {
    speed_hz_ = hz;
    if (!stepper_) return;
    if (hz == 0) {
        // no-op: user must call stop() explicitly
        return;
    }
    stepper_->setSpeedInHz(hz);
    // If already running, commit the change immediately
    if (stepper_->isRunning()) {
        stepper_->applySpeedAcceleration();
    }
}

void StepperAxis::set_accel(uint32_t a) {
    accel_ = a;
    if (stepper_) stepper_->setAcceleration(a);
}

void StepperAxis::move_to(int32_t target) {
    if (!stepper_) {
        // update cached position for compatibility
        position_ = target;
        event_pending_ = true;
        pending_event_ = EventType::MOVE_COMPLETE;
        return;
    }
    // Ensure we have a sensible speed
    if (speed_hz_ == 0) {
        stepper_->setSpeedInHz(100);
    }
    stepper_->moveTo(target);
    state_ = RmtAxisState::ACCEL;
}

void StepperAxis::stop() {
    if (stepper_) stepper_->stopMove();
}

void StepperAxis::emergency_stop() {
    if (stepper_) {
        stepper_->forceStopAndNewPosition(stepper_->getCurrentPosition());
    }
    // disable driver if available
    if (pins_.enable >= 0) {
        digitalWrite(pins_.enable, HIGH);
        enabled_ = false;
    }
    state_ = RmtAxisState::IDLE;
}

void StepperAxis::emergency_stop_from_isr() {
    // Set a flag for the task to execute the force stop (safe from ISR)
    emergency_flag_ = true;
    // Also immediately disable driver output (fast, direct GPIO write)
    if (pins_.enable >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(pins_.enable), 1);
    }
}

void StepperAxis::emergency_stop_from_task() {
    if (emergency_flag_) {
        emergency_flag_ = false;
        emergency_stop();
    }
}

void StepperAxis::reset_position() {
    if (stepper_) {
        stepper_->setCurrentPosition(0);
    }
    position_ = 0;
}

void StepperAxis::set_position(int32_t p) {
    if (stepper_) stepper_->setCurrentPosition(p);
    position_ = p;
}

int32_t StepperAxis::position() const {
    if (stepper_) return stepper_->getCurrentPosition();
    return position_;
}

uint32_t StepperAxis::current_hz() const {
    if (!stepper_) return speed_hz_;
    int32_t mhz = stepper_->getCurrentSpeedInMilliHz();
    if (mhz < 0) mhz = -mhz;
    return static_cast<uint32_t>(mhz / 1000);
}

uint8_t StepperAxis::status_flags() const {
    uint8_t f = 0;
    if (enabled_) f |= StatusFlags::ENABLED;
    if (stepper_ && stepper_->isRunning()) f |= StatusFlags::MOVING;
    if (endstop_active_) f |= StatusFlags::ENDSTOP_HIT;
    if (event_pending_) {
        f |= StatusFlags::EVENT_PENDING;
        if (pending_event_ == EventType::MOVE_COMPLETE) f |= StatusFlags::MOVE_COMPLETE;
        if (pending_event_ == EventType::SPEED_REACHED) f |= StatusFlags::SPEED_REACHED;
        if (pending_event_ == EventType::FAULT) f |= StatusFlags::FAULT;
    }
    return f;
}

void StepperAxis::update_endstop() {
    if (pins_.endstop_no < 0) return;

    bool no_triggered = (gpio_get_level(static_cast<gpio_num_t>(pins_.endstop_no)) == 0);
    bool nc_open = (pins_.endstop_nc >= 0)
                   ? (gpio_get_level(static_cast<gpio_num_t>(pins_.endstop_nc)) == 1)
                   : no_triggered;

    bool at_home  = no_triggered && nc_open;
    bool is_fault = no_triggered && !nc_open;

    if (no_triggered && !endstop_active_) {
        // Debounce handled at caller frequency; set event immediately
        endstop_active_ = true;
        at_home ? (pending_event_ = EventType::HOME_COMPLETE)
                : (pending_event_ = EventType::ENDSTOP_HIT);
        event_pending_ = true;
        // Try to stop quickly
        emergency_stop();
    } else if (!no_triggered && endstop_active_) {
        endstop_active_ = false;
        event_pending_ = true;
        pending_event_ = EventType::ENDSTOP_CLEAR;
    }
}

// ── StepperEngine ─────────────────────────────────────────────────────────

StepperEngine::StepperEngine() {}

void StepperEngine::init(const AxisPins pins[RMT_NUM_AXES]) {
    for (uint8_t i = 0; i < RMT_NUM_AXES; ++i) {
        axes_[i].init(i, pins[i], engine_);
    }
    ESP_LOGI(TAG, "StepperEngine: initialized %u axes", (unsigned)RMT_NUM_AXES);
}

void StepperEngine::start(CmdQueue& cmd_queue) {
    struct Ctx { StepperEngine* e; CmdQueue* q; };
    static Ctx ctx;
    ctx.e = this;
    ctx.q = &cmd_queue;

    xTaskCreatePinnedToCore(
        [](void* p) {
            auto* c = static_cast<Ctx*>(p);
            c->e->run(*c->q);
        },
        "stepper_engine", 4096, &ctx, 24, &task_handle_, 1);

    ESP_LOGI(TAG, "StepperEngine task started on Core 1, priority 24");
}

void StepperEngine::dispatch_command(const CmdFrame& frame) {
    if (frame.axis > RMT_NUM_AXES && frame.axis != 0xFF) return;

    auto apply = [&](uint8_t ax_id) {
        StepperAxis& ax = axes_[ax_id];
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
            ax.move_to(data_s);
            break;
        case CmdOpcode::MOVE_REL:
            ax.move_to(ax.position() + data_s);
            break;
        case CmdOpcode::STOP:       ax.stop(); break;
        case CmdOpcode::ESTOP:      ax.emergency_stop(); break;
        case CmdOpcode::HOME:
            // Best-effort: set low speed toward home
            ax.set_direction(false);
            ax.set_speed_hz(2000);
            break;
        case CmdOpcode::SET_ACCEL:  ax.set_accel(data_u); break;
        case CmdOpcode::RESET_POS:  ax.reset_position(); break;
        case CmdOpcode::UPLOAD_RAMP_START: {
            if (ax_id >= RMT_NUM_AXES) break;
            UploadState& u = upload_[ax_id];
            u.active = true;
            u.expected = data_u;
            u.next_index = 0;
            u.field_pos = 0;
            // zero segments
            for (size_t i = 0; i < MAX_UPLOAD_SEGS; ++i) u.segs[i] = {0,0,0};
            ESP_LOGI(TAG, "UPLOAD_RAMP_START axis=%u expected=%u", ax_id, (unsigned)u.expected);
            break;
        }
        case CmdOpcode::UPLOAD_RAMP_SEG: {
            if (ax_id >= RMT_NUM_AXES) break;
            UploadState& u = upload_[ax_id];
            if (!u.active) break;
            // field sequence: start_iv (u32), add (i32), count (u32)
            if (u.next_index >= u.expected || u.next_index >= MAX_UPLOAD_SEGS) break;
            if (u.field_pos == 0) {
                u.segs[u.next_index].start_iv = data_u;
                u.field_pos = 1;
            } else if (u.field_pos == 1) {
                u.segs[u.next_index].add = data_s;
                u.field_pos = 2;
            } else {
                u.segs[u.next_index].count = data_u;
                u.field_pos = 0;
                u.next_index++;
            }
            break;
        }
        case CmdOpcode::UPLOAD_RAMP_COMMIT: {
            if (ax_id >= RMT_NUM_AXES) break;
            UploadState& u = upload_[ax_id];
            if (!u.active) break;
            // Data_s encodes target position
            int32_t target = data_s;
            // Convert local RampSeg to RmtRampSeg and apply (clamp to RMT_MAX_RAMP_SEGS)
            size_t to_copy = u.next_index;
            if (to_copy == 0 && u.expected > 0) {
                // nothing uploaded
                ESP_LOGW(TAG, "UPLOAD_RAMP_COMMIT axis=%u but no segments uploaded", ax_id);
                u.active = false;
                break;
            }
            size_t max_allowed = RMT_MAX_RAMP_SEGS;
            if (to_copy > max_allowed) to_copy = max_allowed;
            // allocate temporary array on stack if small, else use heap
            RmtRampSeg segs_local[ RMT_MAX_RAMP_SEGS ];
            for (size_t i = 0; i < to_copy; ++i) {
                segs_local[i].start_iv = u.segs[i].start_iv;
                segs_local[i].add = u.segs[i].add;
                segs_local[i].count = u.segs[i].count;
            }
            ESP_LOGI(TAG, "UPLOAD_RAMP_COMMIT axis=%u segs=%u target=%d", ax_id, (unsigned)to_copy, target);
            // Forward to low-level RMT engine to execute the precise segments
            g_rmt_engine.apply_segments_and_move((uint8_t)ax_id, segs_local, to_copy, target);
            u.active = false;
            break;
        }
        case CmdOpcode::UPLOAD_RAMP_ABORT: {
            if (ax_id >= RMT_NUM_AXES) break;
            upload_[ax_id].active = false;
            ESP_LOGI(TAG, "UPLOAD_RAMP_ABORT axis=%u", ax_id);
            break;
        }
        case CmdOpcode::SET_LIMITS:
            // Not implemented in FastAccel wrapper yet
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

void StepperEngine::run(CmdQueue& cmd_queue) {
    TickType_t last_wake = xTaskGetTickCount();
    CmdFrame   frame;

    for (;;) {
        while (cmd_queue.pop(frame)) {
            dispatch_command(frame);
        }

        // Process ISR-requested emergency flags & update endstops
        for (uint8_t i = 0; i < RMT_NUM_AXES; ++i) {
            axes_[i].emergency_stop_from_task();
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

            if (axes_[i].event_pending() && sf.event_type == (uint8_t)EventType::NONE) {
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

StatusFrame StepperEngine::get_status() const {
    StatusFrame sf;
    portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&status_mux_));
    sf = status_;
    portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&status_mux_));
    return sf;
}

void StepperEngine::emergency_stop() {
    for (uint8_t i = 0; i < RMT_NUM_AXES; ++i) {
        axes_[i].emergency_stop();
    }
}
