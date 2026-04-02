/* StepperPRU.cpp — Spindle stepper API delegating to PRU0 via IpcChannel. */

#include "StepperPRU.h"
#include <cmath>
#include <algorithm>

/* Hardware constants mirrored from original Config.h */
static constexpr uint32_t STEPS_PER_REV   = 6400u;
static constexpr uint32_t SPEED_HZ_MIN    = 1067u;   /* ~10 RPM */
static constexpr uint32_t SPEED_HZ_MAX    = 160000u;
static constexpr uint32_t LAT_MIN_HZ_THRESHOLD = 100u;

static inline uint32_t clamp_hz(uint32_t v) {
    if (v < SPEED_HZ_MIN) return SPEED_HZ_MIN;
    if (v > SPEED_HZ_MAX) return SPEED_HZ_MAX;
    return v;
}

StepperPRU::StepperPRU(IpcChannel &channel) : _channel(channel) {}

void StepperPRU::begin(uint32_t accelHzPerMs) {
    if (!_channel.isOpen()) return;
    _accelHzMs = accelHzPerMs;
    _channel.sendCommand(CMD_SPINDLE_SET_ACCEL, AXIS_SPINDLE, accelHzPerMs);
    _channel.sendCommand(CMD_SPINDLE_ENABLE,    AXIS_SPINDLE, 0u); /* disabled at boot */
}

void StepperPRU::setSpeedHz(uint32_t hz) {
    uint32_t clamped = clamp_hz(hz);
    if (clamped == _speedHz) return;
    _speedHz = clamped;
    if (!_channel.isOpen()) return;
    _channel.sendCommand(CMD_SPINDLE_SET_HZ, AXIS_SPINDLE, _speedHz);
}

void StepperPRU::start(bool forward) {
    if (!_channel.isOpen()) return;
    _channel.sendCommand(CMD_SPINDLE_ENABLE, AXIS_SPINDLE, 1u);
    _channel.sendCommand(CMD_SPINDLE_SET_HZ, AXIS_SPINDLE, _speedHz);
    _channel.sendCommand(CMD_SPINDLE_START,  AXIS_SPINDLE,
                         forward ? 1u : 0u);
}

void StepperPRU::stop() {
    if (!_channel.isOpen()) return;
    _channel.sendCommand(CMD_SPINDLE_STOP, AXIS_SPINDLE);
}

void StepperPRU::forceStop() {
    if (!_channel.isOpen()) return;
    _channel.sendCommand(CMD_SPINDLE_FORCE,  AXIS_SPINDLE);
    _channel.sendCommand(CMD_SPINDLE_ENABLE, AXIS_SPINDLE, 0u);
}

void StepperPRU::disableDriver() {
    if (!_channel.isOpen()) return;
    _channel.sendCommand(CMD_SPINDLE_ENABLE, AXIS_SPINDLE, 0u);
}

bool StepperPRU::isRunning() const {
    _refreshTelem();
    return (_telem.state & STATE_RUNNING) != 0u;
}

long StepperPRU::getTurns() const {
    _refreshTelem();
    return (long)(_telem.step_count / STEPS_PER_REV);
}

void StepperPRU::resetTurns() {
    if (!_channel.isOpen()) return;
    _channel.sendCommand(CMD_RESET_POSITION, AXIS_SPINDLE);
}

float StepperPRU::getRPM() const {
    _refreshTelem();
    if (_telem.current_hz == 0u) return 0.0f;
    return (float)_telem.current_hz * 60.0f / (float)STEPS_PER_REV;
}

void StepperPRU::_refreshTelem() const {
    _channel.readSpindleTelem(_telem);
}

uint32_t StepperPRU::calculateCompensatedSpindleHz(
    uint32_t nominalSpindleHz,
    uint32_t currentLatHz,
    uint32_t nominalLatHz)
{
    if (nominalLatHz == 0u || nominalSpindleHz == 0u)
        return nominalSpindleHz;
    if (currentLatHz < LAT_MIN_HZ_THRESHOLD)
        return nominalSpindleHz;

    /* Avoid 64-bit on embedded Linux if tight; fine here on Cortex-A8 */
    uint32_t comp = (uint32_t)(((uint64_t)nominalSpindleHz * currentLatHz)
                               / nominalLatHz);
    return clamp_hz(comp);
}
