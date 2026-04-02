/* LateralPRU.cpp — Lateral carriage API delegating to PRU1 via IpcChannel. */

#include "LateralPRU.h"
#include <cmath>
#include <cstring>

/* Hardware constants (mirror of Config.h) */
static constexpr uint32_t LAT_STEPS_PER_MM    = 3072u;
static constexpr uint32_t LAT_HOME_SPEED_HZ   = 4800u;
static constexpr uint32_t LAT_ACCEL_HZ_MS     = 48u;   /* 48 Hz/ms → 4800 Hz in 100 ms */
static constexpr uint32_t LAT_STEPS_PER_REV   = 3072u; /* at 1mm pitch */

static inline uint32_t mm_to_steps(float mm) {
    return (uint32_t)(mm * (float)LAT_STEPS_PER_MM + 0.5f);
}

static inline float steps_to_mm(int32_t steps) {
    return (float)steps / (float)LAT_STEPS_PER_MM;
}

LateralPRU::LateralPRU(IpcChannel &channel) : _channel(channel) {}

void LateralPRU::begin(float homeOffsetMm) {
    _homeOffsetMm = homeOffsetMm;
    if (!_channel.isOpen()) return;
    _channel.sendCommand(CMD_LATERAL_SET_ACCEL, AXIS_LATERAL, LAT_ACCEL_HZ_MS);
    _channel.sendCommand(CMD_LATERAL_ENABLE,    AXIS_LATERAL, 1u);
    rehome();
}

void LateralPRU::rehome() {
    if (!_channel.isOpen()) return;
    _state = LatState::HOMING;
    /* Move backward toward home at homing speed */
    _channel.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, LAT_HOME_SPEED_HZ);
    _channel.sendCommand(CMD_LATERAL_START,  AXIS_LATERAL, 0u); /* 0 = backward */
}

void LateralPRU::update() {
    if (!_channel.isOpen()) return;
    _refreshTelem();

    uint16_t st     = _telem.state;
    uint16_t faults = _telem.faults;

    /* Propagate PRU faults to Linux state */
    if (faults & FAULT_HOME_SENSOR) {
        _state = LatState::FAULT;
        return;
    }
    /* Position overrun is a firmware protection fault, not normal reversal */
    if ((faults & FAULT_OVERRUN) && _state != LatState::HOMING &&
                                    _state != LatState::HOMING_OFFSET) {
        _state = LatState::FAULT;
        return;
    }

    float posMm = getPositionMm();

    switch (_state) {
        case LatState::HOMING:
            /* PRU1 auto-stops and resets position to 0 when home sensor fires.
             * Wait for motor to stop and for STATE_AT_HOME to be set. */
            if ((st & STATE_AT_HOME) && !(st & STATE_RUNNING)) {
                _state = LatState::HOMING_OFFSET;
                /* Move forward to home offset position */
                _channel.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, LAT_HOME_SPEED_HZ / 2u);
                _channel.sendCommand(CMD_LATERAL_START,  AXIS_LATERAL, 1u); /* 1=forward (value_a=1 → direction=0) */
            }
            break;

        case LatState::HOMING_OFFSET:
            /* Linux tracks position; stop when we reach homeOffset */
            if (posMm >= _homeOffsetMm) {
                _channel.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL);
                if (!(st & STATE_RUNNING)) {
                    /* Reset position counter so this is the new zero */
                    _channel.sendCommand(CMD_RESET_POSITION, AXIS_LATERAL);
                    _state = LatState::HOMED;
                }
            }
            break;

        case LatState::POSITIONING:
            /* Stop when close enough to target */
            if (posMm >= _startPosMm - 0.05f && posMm <= _startPosMm + 0.05f) {
                _channel.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL);
                if (!(st & STATE_RUNNING))
                    _state = LatState::HOMED;
            }
            break;

        case LatState::WINDING_FWD:
            /* Detect end of forward traverse by position */
            if (posMm >= _endMm) {
                if (_stopOnNextHigh) {
                    _stopOnNextHigh = false;
                    _channel.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL);
                    _state = LatState::HOMED;
                } else if (_pauseOnNextReversal) {
                    _pauseOnNextReversal = false;
                    _pausedAtReversal    = true;
                    _channel.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL);
                    _state = LatState::HOMED;
                } else {
                    /* Reverse: CMD_LATERAL_START with value_a=0 → direction=1 (backward) */
                    _channel.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, 0u);
                    _state = LatState::WINDING_BWD;
                }
            }
            break;

        case LatState::WINDING_BWD:
            /* Detect start of backward traverse by position (or home sensor as safety) */
            if (posMm <= _startMm || (st & STATE_AT_HOME)) {
                if (_stopOnNextLow) {
                    _stopOnNextLow = false;
                    _channel.sendCommand(CMD_LATERAL_STOP, AXIS_LATERAL);
                    _state = LatState::HOMED;
                } else {
                    /* Reverse forward: value_a=1 → direction=0 */
                    _channel.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, 1u);
                    _state = LatState::WINDING_FWD;
                }
            }
            break;

        default:
            break;
    }
}

void LateralPRU::prepareStartPosition(float startMm, uint32_t speedHz) {
    if (!_channel.isOpen()) return;
    _startPosMm = startMm;
    float current = getPositionMm();
    float dist = startMm - current;
    if (dist == 0.0f) return; /* already there */
    _state = LatState::POSITIONING;
    _channel.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, speedHz);
    /* value_a=1 → direction=0 (forward); value_a=0 → direction=1 (backward) */
    uint8_t fwd = (dist > 0.0f) ? 1u : 0u;
    _channel.sendCommand(CMD_LATERAL_START, AXIS_LATERAL, fwd);
    /* update() monitors position each tick and issues CMD_LATERAL_STOP when arrived */
}

void LateralPRU::jog(float deltaMm) {
    if (!_channel.isOpen()) return;
    if (deltaMm == 0.0f) return;
    uint8_t fwd = (deltaMm > 0.0f) ? 1u : 0u;
    float   abs_mm = (deltaMm > 0.0f) ? deltaMm : -deltaMm;
    _channel.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, LAT_HOME_SPEED_HZ / 4u);
    _channel.sendCommand(CMD_LATERAL_START,  AXIS_LATERAL, fwd);
    /* Caller calls stop() after desired displacement (step-count based stopping TBD) */
    (void)abs_mm;
}

void LateralPRU::startWinding(uint32_t windingHz, long tpp,
                               float startMm, float endMm, float speedScale) {
    if (!_channel.isOpen()) return;
    _startMm = startMm;
    _endMm   = endMm;

    /* Lateral speed = effWidth × windingHz / (tpp × STEPS_PER_REV) */
    float effWidth = endMm - startMm;
    uint32_t latHz = 0u;
    if (tpp > 0 && effWidth > 0.0f) {
        latHz = (uint32_t)(effWidth * (float)windingHz /
                           ((float)tpp * (float)LAT_STEPS_PER_REV) * speedScale);
    }
    latHz = (latHz < 100u) ? 100u : latHz;

    _channel.setNominalLateralHz(latHz);
    _channel.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, latHz);
    _channel.sendCommand(CMD_LATERAL_START,  AXIS_LATERAL, 1u); /* forward */
    _state = LatState::WINDING_FWD;
}

void LateralPRU::updateWinding(uint32_t windingHz, long tpp, float speedScale) {
    /* Recalculate lateral speed and push if changed */
    if (_startMm >= _endMm || tpp <= 0) return;
    float effWidth = _endMm - _startMm;
    uint32_t latHz = (uint32_t)(effWidth * (float)windingHz /
                                ((float)tpp * (float)LAT_STEPS_PER_REV) * speedScale);
    latHz = (latHz < 100u) ? 100u : latHz;
    _channel.setNominalLateralHz(latHz);
    _channel.sendCommand(CMD_LATERAL_SET_HZ, AXIS_LATERAL, latHz);
}

bool LateralPRU::isHomed() const {
    return _state == LatState::HOMED
        || _state == LatState::WINDING_FWD
        || _state == LatState::WINDING_BWD;
}

bool LateralPRU::isBusy() const {
    _refreshTelem();
    return (_telem.state & STATE_RUNNING) != 0u;
}

bool LateralPRU::isAtZero() const {
    return (_telem.state & STATE_AT_HOME) != 0u;
}

bool LateralPRU::isPositionedForStart() const {
    return _state == LatState::HOMED
        && std::abs(getPositionMm() - _startPosMm) < 0.1f;
}

float LateralPRU::getPositionMm() const {
    _refreshTelem();
    return steps_to_mm(_telem.position_steps);
}

uint32_t LateralPRU::getCurrentLateralHz() const {
    _refreshTelem();
    return _telem.current_hz;
}

void LateralPRU::clearOneShotStops() {
    _stopOnNextHigh      = false;
    _stopOnNextLow       = false;
    _pauseOnNextReversal = false;
    _pausedAtReversal    = false;
}

bool LateralPRU::hasStopAtNextBoundArmed() const {
    return _stopOnNextHigh || _stopOnNextLow || _pauseOnNextReversal;
}

bool LateralPRU::consumePausedAtReversal() {
    bool v = _pausedAtReversal;
    _pausedAtReversal = false;
    return v;
}

const char *LateralPRU::stateStr() const {
    switch (_state) {
        case LatState::FAULT:          return "FAULT";
        case LatState::BACKOFF:        return "BACKOFF";
        case LatState::HOMING:         return "HOMING";
        case LatState::HOMING_DECEL:   return "HOMING_DECEL";
        case LatState::HOMING_ALIGN:   return "HOMING_ALIGN";
        case LatState::HOMING_OFFSET:  return "HOMING_OFFSET";
        case LatState::HOMED:          return "HOMED";
        case LatState::POSITIONING:    return "POSITIONING";
        case LatState::WINDING_FWD:    return "WINDING_FWD";
        case LatState::WINDING_BWD:    return "WINDING_BWD";
        default:                       return "UNKNOWN";
    }
}

void LateralPRU::_refreshTelem() const {
    _channel.readLateralTelem(_telem);
}

uint32_t LateralPRU::_mmToSteps(float mm) const {
    return mm_to_steps(mm);
}

float LateralPRU::_stepsToMm(int32_t steps) const {
    return steps_to_mm(steps);
}
