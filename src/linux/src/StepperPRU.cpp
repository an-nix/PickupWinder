/* StepperPRU.cpp — Spindle stepper API delegating to PRU0 via IpcChannel.
 *
 * Klipper-style implementation: step moves pre-computed on host, pushed to
 * the PRU move ring.  No Hz/accel commands sent to PRU; the IEP counter
 * handles precise edge timing.
 *
 * Compensation (spindle-lateral ratio) remains host-side:
 *   WinderApp calls setSpeedHz(compensatedHz) every control loop tick.
 *   tick() detects the changed target and transitions via accel segment.
 */

#include "StepperPRU.h"
#include <cmath>

static inline uint32_t clamp_hz(uint32_t v, uint32_t lo, uint32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

StepperPRU::StepperPRU(IpcChannel &channel) : _channel(channel) {}

void StepperPRU::begin(uint32_t accelHzPerMs) {
    if (!_channel.isOpen()) return;
    _accelHzPerS = accelHzPerMs * 1000u;
    _channel.sendCommand(CMD_ENABLE, AXIS_SPINDLE, 0u);   /* driver disabled */
}

void StepperPRU::setSpeedHz(uint32_t hz) {
    uint32_t clamped = clamp_hz(hz, SPEED_HZ_MIN, SPEED_HZ_MAX);
    if (clamped == _speedHz) return;
    _speedHz     = clamped;
    _speedChanged = true;
}

void StepperPRU::start(bool forward) {
    if (!_channel.isOpen()) return;
    _forward   = forward;
    _streaming = true;
    _channel.sendCommand(CMD_ENABLE, AXIS_SPINDLE, 1u);

    /* Push accel segment from SPEED_HZ_MIN to _speedHz */
    uint32_t startHz = SPEED_HZ_MIN;
    if (_speedHz > startHz) {
        MoveQueue::pushAccelSegment(_channel, AXIS_SPINDLE,
                                    startHz, _speedHz, forward, _accelHzPerS);
    }
    /* Seed the ring with initial constant-speed block */
    MoveQueue::pushConstantMs(_channel, AXIS_SPINDLE,
                               _speedHz, forward, REFILL_MS * KEEP_SLOTS);
    _pushedHz     = _speedHz;
    _speedChanged = false;
}

void StepperPRU::stop() {
    if (!_channel.isOpen()) return;
    _streaming = false;
    MoveQueue::pushDecelToStop(_channel, AXIS_SPINDLE,
                                _pushedHz, _forward, _accelHzPerS);
    _pushedHz = 0u;
}

void StepperPRU::forceStop() {
    if (!_channel.isOpen()) return;
    _streaming = false;
    _channel.sendCommand(CMD_QUEUE_FLUSH, AXIS_SPINDLE);
    _channel.sendCommand(CMD_ENABLE,      AXIS_SPINDLE, 0u);
    _pushedHz = 0u;
}

void StepperPRU::disableDriver() {
    if (!_channel.isOpen()) return;
    _channel.sendCommand(CMD_ENABLE, AXIS_SPINDLE, 0u);
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
    uint32_t iv = _telem.current_interval;
    if (iv == 0u) return 0.0f;
    /* step_hz = PRU_CLOCK/(2*interval); rpm = hz*60/SPR */
    float hz = (float)PRU_CLOCK_HZ / (2.0f * (float)iv);
    return hz * 60.0f / (float)STEPS_PER_REV;
}

void StepperPRU::tick() {
    if (!_streaming || !_channel.isOpen()) return;
    _refreshTelem();

    /* Speed transition: flush ring and ramp to new target */
    if (_speedChanged) {
        _channel.sendCommand(CMD_QUEUE_FLUSH, AXIS_SPINDLE);
        uint32_t from = (_pushedHz > 0u) ? _pushedHz : SPEED_HZ_MIN;
        if (_speedHz != from) {
            MoveQueue::pushAccelSegment(_channel, AXIS_SPINDLE,
                                        from, _speedHz, _forward, _accelHzPerS);
        }
        /* Seed constant-speed block after ramp to prevent underrun */
        MoveQueue::pushConstantMs(_channel, AXIS_SPINDLE,
                                   _speedHz, _forward, REFILL_MS * KEEP_SLOTS);
        _pushedHz     = _speedHz;
        _speedChanged = false;
    }

    /* Refill constant-speed block when ring is low */
    uint32_t slots = _channel.moveQueueFreeSlots(AXIS_SPINDLE);
    if (slots >= IPC_SP_MOVE_SLOTS - KEEP_SLOTS) {
        MoveQueue::pushConstantMs(_channel, AXIS_SPINDLE,
                                   _pushedHz, _forward, REFILL_MS);
    }
}

void StepperPRU::_refreshTelem() const {
    _channel.readSpindleTelem(_telem);
}

uint32_t StepperPRU::calculateCompensatedSpindleHz(uint32_t nominalSpindleHz,
                                                    uint32_t currentLatHz,
                                                    uint32_t nominalLatHz) {
    if (nominalLatHz == 0u || nominalSpindleHz == 0u || currentLatHz < 100u)
        return nominalSpindleHz;
    uint32_t comp = (uint32_t)(((uint64_t)nominalSpindleHz * currentLatHz)
                               / nominalLatHz);
    return clamp_hz(comp, StepperPRU::SPEED_HZ_MIN, StepperPRU::SPEED_HZ_MAX);
}


