#include "RampPlanner.h"
#include <cstdlib>

RampPlanner::RampPlanner(StepperController& ctrl,
                         const MotionProfile& profile)
    : ctrl_(ctrl), profile_(profile) {}

esp_err_t RampPlanner::moveTo(int32_t absolute_steps) {
    return moveBy(absolute_steps - ctrl_.position());
}

esp_err_t RampPlanner::moveBy(int32_t delta) {
    if (delta == 0) return ESP_OK;
    return ctrl_.enqueue(buildCommand(delta));
}

MoveCommand RampPlanner::buildCommand(int32_t delta) const {
    const uint32_t total = static_cast<uint32_t>(
                               delta > 0 ? delta : -delta);
    MoveCommand cmd{};
    cmd.target_steps  = delta;
    cmd.start_freq_hz = profile_.start_freq_hz;
    cmd.end_freq_hz   = profile_.max_freq_hz;
    cmd.sample_points = profile_.sample_points;

    cmd.accel_steps   = static_cast<uint32_t>(total * profile_.accel_ratio);
    cmd.decel_steps   = static_cast<uint32_t>(total * profile_.decel_ratio);

    // Garde-fou : accel + decel ne peut pas dépasser total
    if (cmd.accel_steps + cmd.decel_steps > total) {
        cmd.accel_steps = total / 2;
        cmd.decel_steps = total - cmd.accel_steps;
    }
    return cmd;
}