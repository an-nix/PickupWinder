#include "SessionController.h"
#include "Diag.h"
#include "Types.h"

SessionController::SessionController(WinderApp& winder)
    : _winder(winder)
{
}

void SessionController::setPotLevel(float level) {
    float newLevel = constrain(level, 0.0f, 1.0f);
    if (fabs(newLevel - _potLevel) >= 0.001f) {
        _potLevel = newLevel;
        _lastInput = InputSource::Pot;
    }
}

void SessionController::setFootswitch(bool pressed) {
    _footswitch = pressed;
    _lastInput = InputSource::Footswitch;
    if (pressed) requestStart();
    else requestPause();
}

void SessionController::requestStart() {
    _reqStart = true;
    _lastInput = InputSource::IHM;
}

void SessionController::requestPause() {
    _reqPause = true;
    _lastInput = InputSource::IHM;
}

void SessionController::requestStop() {
    _reqStop = true;
    _lastInput = InputSource::IHM;
}

bool SessionController::handleCommand(const String& cmd, const String& value) {
    if (cmd == "start") {
        requestStart();
        return true;
    }
    if (cmd == "pause") {
        requestPause();
        return true;
    }
    if (cmd == "stop") {
        requestStop();
        return true;
    }
    if (cmd == "target") {
        long t = value.toInt();
        if (t > 0) _winder.setTargetTurns(t);
        return true;
    }
    if (cmd == "freerun") {
        _winder.setFreerun(value == "true");
        return true;
    }
    if (cmd == "direction") {
        _winder.setDirection(value == "cw" ? Direction::CW : Direction::CCW);
        return true;
    }
    if (cmd == "max_rpm" || cmd == "max-rpm") {
        _winder.setMaxRpm((uint16_t)constrain(value.toInt(), 10, 1500));
        return true;
    }

    
    return false;
}

void SessionController::applyPower() {
    if (_state == SessionState::RUNNING) {
        uint32_t speedHz = (uint32_t)(_potLevel * (float)_winder.getMaxSpeedHz());
        _winder.setControlHz(speedHz);
        Serial.printf("[Session] run speedHz=%u pot=%.3f source=%d\n", speedHz, _potLevel, (int)_lastInput);
    } else {
        _winder.setControlHz(0);
        Serial.printf("[Session] paused source=%d pot=%.3f\n", (int)_lastInput, _potLevel);
    }
}

void SessionController::tick(const TickInput& in) {
    // Integrate runtime inputs passed in via TickInput
    if (in.hasPot) {
        float newLevel = constrain(in.potLevel, 0.0f, 1.0f);
        if (fabs(newLevel - _potLevel) >= 0.001f) {
            _potLevel = newLevel;
            _lastInput = InputSource::Pot;
        }
    }
    if (in.hasFootswitch) {
        _footswitch = in.footswitch;
        _lastInput = InputSource::Footswitch;
        if (_footswitch) requestStart();
        else requestPause();
    }

    // Process commands provided in TickInput (bounded, no dynamic alloc)
    for (int i = 0; i < in.cmdCount; ++i) {
        const char* c = in.commands[i].cmd;
        const char* v = in.commands[i].val;
        String sc(c);
        String sv(v);
        // Session-level commands handled here; others forwarded to WinderApp
        if (!handleCommand(sc, sv)) {
            _winder.handleCommand(sc, sv);
        }
    }

    //Serial.printf("[Session] tick state=%d lastInput=%d pot=%.3f foot=%d reqS=%d reqP=%d reqT=%d\n", (int)_state, (int)_lastInput, _potLevel, (int)_footswitch, (int)_reqStart, (int)_reqPause, (int)_reqStop);
    if (_reqStop) {
        _winder.stopWinding();
        _state = SessionState::IDLE;
        _reqStop = false;
    } else if (_reqPause) {
        _winder.pauseWinding();
        _state = SessionState::PAUSED;
        _reqPause = false;
    } else if (_reqStart) {
        if (_state != SessionState::RUNNING) {
            _winder.handleCommand("start", "");
            _state = SessionState::RUNNING;
        }
        _reqStart = false;
    } else {
        // Priority to latest input source.
        if (_lastInput == InputSource::IHM) {
            // iHM was already processed by reqStart/reqPause.
        } else if (_lastInput == InputSource::Footswitch) {
            if (_footswitch) {
                if (_state != SessionState::RUNNING) {
                    _winder.handleCommand("start", "");
                    _state = SessionState::RUNNING;
                }
            } else if (_state != SessionState::PAUSED) {
                _winder.pauseWinding();
                _state = SessionState::PAUSED;
            }
        } else if (_lastInput == InputSource::Pot) {
            if (_potLevel > 0.001f) {
                if (_state != SessionState::RUNNING) {
                    _winder.handleCommand("start", "");
                    _state = SessionState::RUNNING;
                }
            } else if (_state != SessionState::PAUSED) {
                _winder.pauseWinding();
                _state = SessionState::PAUSED;
            }
        }
    }

    applyPower();
    _winder.tick();
}

// populateStatus removed; status retrieval handled directly from WinderApp in main
