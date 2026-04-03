/* LateralPRU.cpp — Lateral carriage API delegating to PRU1 via IpcChannel.
 *
 * Klipper-style: all moves pre-computed as (interval, count, add) on host
 * and pushed to the PRU1 move ring.  No Hz/start/stop speed commands.
 * Homing: CMD_HOME_START tells PRU1 to enter homing_mode; PRU1 auto-stops
 * and resets position when the home sensor fires.
 */

#include "LateralPRU.h"
#include <cmath>
#include <cstring>

/* ── Hardware constants ──────────────────────────────────────────────────────*/
static constexpr uint32_t HOMING_STEPS = 700000u; /* ~228 mm > max travel   */

LateralPRU::LateralPRU(IpcChannel &channel) : _channel(channel) {}

/* ── Helpers ─────────────────────────────────────────────────────────────────*/
uint32_t LateralPRU::_mmToSteps(float mm) {
    return (uint32_t)(mm * (float)LAT_STEPS_PER_MM + 0.5f);
}

float LateralPRU::_stepsToMm(int32_t steps) {
    return (float)steps / (float)LAT_STEPS_PER_MM;
}

bool LateralPRU::_pushTraverse(float fromMm, float toMm, uint32_t latHz) {
    float delta = toMm - fromMm;
    if (latHz == 0u || std::abs(delta) < 0.001f) return false;
    bool fwd   = (delta > 0.0f);
    float absd = fwd ? delta : -delta;
    uint32_t n_edges = 2u * _mmToSteps(absd);
    if (n_edges == 0u) return false;
    return MoveQueue::pushConstantEdges(_channel, AXIS_LATERAL, latHz, fwd, n_edges);
}

/* ── State machine ───────────────────────────────────────────────────────────*/
void LateralPRU::begin(float homeOffsetMm) {
    _homeOffsetMm = homeOffsetMm;
    if (!_channel.isOpen()) return;
    _channel.sendCommand(CMD_ENABLE, AXIS_LATERAL, 1u);
    rehome();
}

void LateralPRU::rehome() {
    if (!_channel.isOpen()) return;
    _state = LatState::HOMING;
    _channel.sendCommand(CMD_QUEUE_FLUSH, AXIS_LATERAL);
    _channel.sendCommand(CMD_HOME_START,  AXIS_LATERAL);
    /* Push many slow backward edges; PRU1 flushes ring on HOME_HIT */
    MoveQueue::pushConstantEdges(_channel, AXIS_LATERAL,
                                  LAT_HOME_HZ, false, 2u * HOMING_STEPS);
}

void LateralPRU::update() {
    if (!_channel.isOpen()) return;
    _refreshTelem();

    uint16_t st     = _telem.state;
    uint16_t faults = _telem.faults;

    if (faults & FAULT_HOME_SENSOR) {
        _state = LatState::FAULT; return;
    }
    if ((faults & FAULT_OVERRUN) &&
        _state != LatState::HOMING && _state != LatState::HOMING_OFFSET) {
        _state = LatState::FAULT; return;
    }

    bool motor_idle = !(st & STATE_RUNNING) && (_telem.moves_pending == 0u);

    switch (_state) {

    case LatState::HOMING:
        /* PRU1 stops and resets position when HOME_HIT. */
        if ((st & STATE_AT_HOME) && motor_idle) {
            _state = LatState::HOMING_OFFSET;
            _pushTraverse(0.0f, _homeOffsetMm, LAT_HOME_HZ / 2u);
        }
        break;

    case LatState::HOMING_OFFSET:
        if (motor_idle) {
            _channel.sendCommand(CMD_RESET_POSITION, AXIS_LATERAL);
            _state = LatState::HOMED;
        }
        break;

    case LatState::POSITIONING:
        if (motor_idle)
            _state = LatState::HOMED;
        break;

    case LatState::WINDING_FWD:
        if (motor_idle) {
            if (_stopOnNextHigh) {
                _stopOnNextHigh = false;
                _state = LatState::HOMED;
            } else if (_pauseOnNextReversal) {
                _pauseOnNextReversal = false;
                _pausedAtReversal    = true;
                _state = LatState::HOMED;
            } else {
                /* Push full backward traverse */
                _pushTraverse(_endMm, _startMm, _windingLatHz);
                _state = LatState::WINDING_BWD;
            }
        }
        break;

    case LatState::WINDING_BWD:
        if (motor_idle || (st & STATE_AT_HOME)) {
            if (_stopOnNextLow) {
                _stopOnNextLow = false;
                _state = LatState::HOMED;
            } else {
                /* Push full forward traverse */
                _pushTraverse(_startMm, _endMm, _windingLatHz);
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
    _refreshTelem();
    float current = _stepsToMm(_telem.position_steps);
    if (std::abs(startMm - current) < 0.05f) return;
    _state = LatState::POSITIONING;
    _channel.sendCommand(CMD_QUEUE_FLUSH, AXIS_LATERAL);
    _pushTraverse(current, startMm, speedHz > 0u ? speedHz : LAT_HOME_HZ);
}

void LateralPRU::jog(float deltaMm) {
    if (!_channel.isOpen() || std::abs(deltaMm) < 0.001f) return;
    _refreshTelem();
    float current = _stepsToMm(_telem.position_steps);
    _pushTraverse(current, current + deltaMm, LAT_HOME_HZ / 4u);
}

void LateralPRU::startWinding(uint32_t windingHz, long tpp,
                               float startMm, float endMm, float speedScale) {
    if (!_channel.isOpen()) return;
    _startMm = startMm;
    _endMm   = endMm;

    float effWidth = endMm - startMm;
    if (effWidth <= 0.0f || tpp <= 0) return;

    /* Lateral Hz = effWidthMm × windingHz / (tpp × STEPS_PER_REV) */
    _windingLatHz = (uint32_t)(effWidth * (float)windingHz /
                               ((float)tpp * (float)LAT_STEPS_PER_MM) * speedScale);
    if (_windingLatHz < 10u) _windingLatHz = 10u;

    _channel.sendCommand(CMD_QUEUE_FLUSH, AXIS_LATERAL);
    _pushTraverse(startMm, endMm, _windingLatHz);
    _state = LatState::WINDING_FWD;
}

void LateralPRU::updateWinding(uint32_t windingHz, long tpp, float speedScale) {
    if (_startMm >= _endMm || tpp <= 0) return;
    float effWidth = _endMm - _startMm;
    uint32_t latHz = (uint32_t)(effWidth * (float)windingHz /
                                ((float)tpp * (float)LAT_STEPS_PER_MM) * speedScale);
    if (latHz < 10u) latHz = 10u;
    /* Speed change will apply on the next traversal push (at boundary) */
    _windingLatHz = latHz;
}

/* ── State queries ───────────────────────────────────────────────────────────*/
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
    return _stepsToMm(_telem.position_steps);
}

uint32_t LateralPRU::getCurrentLateralHz() const {
    _refreshTelem();
    uint32_t iv = _telem.current_interval;
    if (iv == 0u) return 0u;
    return PRU_CLOCK_HZ / (2u * iv);
}

/* ── One-shot stop flags ─────────────────────────────────────────────────────*/
void LateralPRU::clearOneShotStops() {
    _stopOnNextHigh = _stopOnNextLow = _pauseOnNextReversal = _pausedAtReversal = false;
}

bool LateralPRU::hasStopAtNextBoundArmed() const {
    return _stopOnNextHigh || _stopOnNextLow || _pauseOnNextReversal;
}

bool LateralPRU::consumePausedAtReversal() {
    bool v = _pausedAtReversal; _pausedAtReversal = false; return v;
}

/* ── stateStr ────────────────────────────────────────────────────────────────*/
const char *LateralPRU::stateStr() const {
    switch (_state) {
    case LatState::FAULT:          return "FAULT";
    case LatState::HOMING:         return "HOMING";
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




