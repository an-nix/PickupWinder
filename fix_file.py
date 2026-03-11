import sys

content = """#include "WinderApp.h"
#include <Arduino.h>

void WinderApp::begin() {
    Serial.begin(115200);
    Serial.println("\\n=== Pickup Winder ===");

    // Initialise each subsystem in dependency order.
    _engine.init();        // Une seule engine FastAccelStepper pour tous les steppers
    _stepper.begin(_engine);  // GPIO + stepper bobine
    _pot.begin();      // Pre-fill ADC filter buffer
    _led.begin();      // Set LED pin as output
    _web.begin();      // WiFi + HTTP + WebSocket  (peut prendre 2-5 s)
    _link.begin();     // UART2 liaison vers ESP écran
    _lateral.begin(_engine);  // GPIO + stepper latéral + homing automatique

    // Register the command callback so WebSocket messages are routed to
    // _handleCommand() on this instance.
    _web.setCommandCallback([this](const String& cmd, const String& val) {
        _handleCommand(cmd, val);
    });

    if (_web.isConnected()) {
        Serial.printf("→ Web interface: http://%s\\n", _web.getIP().c_str());
    }
    Serial.printf("Ready — Geometry: %.1fmm usable, %ld turns/pass\\n",
                  _geom.effectiveWidth(), _geom.turnsPerPass());
}

void WinderApp::run() {
    uint32_t now = millis();

    // Machine d'états du homing latéral — non-bloquante, appelée à chaque itération.
    _lateral.update();

    if (_mode == WinderMode::MANUAL) {

        if (now - _lastPotMs >= POT_READ_INTERVAL) {
            _lastPotMs = now;
            uint32_t hz        = _pot.readHz();

            bool potActive = (hz > 0);
            bool potStop   = (hz == 0);

            if (potStop) _potWasZero = true;

            if (!_motorEnabled && _potWasZero && potActive && !_targetReached
                               && _lateral.isHomed()) {
                _motorEnabled = true;
                Serial.println("▶ Pot active — motor ready");
            }

            if (_motorEnabled && potActive) {
                if (!_freerun && _stepper.isRunning()) {
                    long remaining = (long)_targetTurns - _stepper.getTurns();
                    if (remaining > 0 && remaining <= APPROACH_TURNS) {
                        float ratio = (float)remaining / APPROACH_TURNS;
                        uint32_t maxHz = APPROACH_SPEED_HZ_FLOOR
                                       + (uint32_t)(ratio * (float)(SPEED_HZ_MAX - APPROACH_SPEED_HZ_FLOOR));
                        hz = min(hz, maxHz);
                    }
                }
                if (_lateral.isReversing()) {
                    hz = max((uint32_t)SPEED_HZ_MIN,
                             (uint32_t)((float)hz * LAT_REVERSAL_SLOWDOWN));
                }
                _stepper.setSpeedHz(hz);
                if (!_stepper.isRunning()) {
                    bool forward = (_direction == Direction::CW) != (bool)WINDING_MOTOR_INVERTED;
                    _stepper.start(forward);
                    _lateral.startWinding(hz, _geom.turnsPerPass(), _geom.effectiveWidth());
                    Serial.printf("▶ Start %s — %u Hz\\n",
                                  _direction == Direction::CW ? "CW" : "CCW", hz);
                } else {
                    if (_lateral.getState() == LatState::HOMED) {
                        _lateral.startWinding(hz, _geom.turnsPerPass(), _geom.effectiveWidth());
                    } else {
                        _lateral.updateWinding(hz, _geom.turnsPerPass(), _geom.effectiveWidth());
                    }
                }
            } else if (_motorEnabled && potStop && _stepper.isRunning()) {
                _stop();
            }

            _led.update(_stepper.getTurns(), _geom.turnsPerPass(), _stepper.isRunning());
        }

        if (_motorEnabled && !_freerun && _stepper.isRunning() && _stepper.getTurns() >= _targetTurns) {
            _targetReached = true;
            _stop();
            Serial.printf("✓ Winding complete! %ld turns done.\\n", _stepper.getTurns());
        }
    }

    if (_playingMusic) {
        const MusicNote& n = MELODY_LA_BAMBA[_musicIdx];
        if (now - _musicNoteMs >= n.durationMs) {
            _musicIdx++;
            if (_musicIdx >= LA_BAMBA_LENGTH || MELODY_LA_BAMBA[_musicIdx].durationMs == 0) {
                _playingMusic = false;
                _stepper.stopNote();
                Serial.println("\\xF0\\x9F\\x8E\\xB5 La Bamba finished!");
            } else {
                _stepper.playNote(MELODY_LA_BAMBA[_musicIdx].freq);
                _musicNoteMs = now;
            }
        }
    }

    if (_pendingDisable && !_stepper.isRunning()) {
        _stepper.disableDriver();
        _pendingDisable = false;
    }

    _link.poll([this](const String& cmd, const String& val) {
        _handleCommand(cmd, val);
    });
    if (now - _lastLinkMs >= LINK_UPDATE_MS) {
        _lastLinkMs = now;
        _link.sendStatus(
            _stepper.getRPM(),
            _stepper.getSpeedHz(),
            _stepper.getTurns(),
            (long)_targetTurns,
            _stepper.isRunning(),
            (bool)_motorEnabled,
            (bool)_freerun,
            _direction == Direction::CW
        );
    }

    if (now - _lastWsMs >= WS_UPDATE_MS) {
        _lastWsMs = now;
        _web.sendUpdate({
            _stepper.getRPM(),
            _stepper.getSpeedHz(),
            _stepper.getTurns(),
            (long)_targetTurns,
            _stepper.isRunning(),
            (bool)_motorEnabled,
            (bool)_freerun,
            _direction == Direction::CW,
            _mode == WinderMode::AUTO,
            _geom.turnsPerPass(),
            _geom.turnsPerPassCalc(),
            _geom.turnsPerPassOffset,
            _geom.scatterFactor,
            _led.getCurrentPass(),
            _geom.effectiveWidth(),
            _geom.totalWidth_mm, _geom.flangeBottom_mm, _geom.flangeTop_mm,
            _geom.margin_mm, _geom.wireDiameter_mm,
            _lateral.getHomeOffset()
        });
    }
}

void WinderApp::_stop() {
    _motorEnabled   = false;
    _potWasZero     = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    _led.reset();
    Serial.println("■ Stopped — return pot to 0 to restart");
}

void WinderApp::_pause() {
    _midWindingPaused = true;
    _pendingDisable = true;
    _resumeFromCurrentPos = true;
    _stepper.stop();
    _lateral.stopWinding();
    Serial.println("⏸ Paused");
}

void WinderApp::_handleCommand(const String& cmd, const String& value) {
    auto updateCarriage = [this]() {
        if (_startRequested && !_stepper.isRunning()) {
            _lateral.prepareStartPosition(_geom.windingStartMm());
        }
    };

    if (cmd == "stop") {
        if (_playingMusic) {
            _playingMusic = false;
            _stepper.stopNote();
        }
        _stop();

    } else if (cmd == "pause") {
        _pause();

    } else if (cmd == "music") {
        if (_stepper.isRunning()) _stepper.stop();
        _motorEnabled = false;
        _potWasZero   = false;
        _musicIdx     = 0;
        _musicNoteMs  = millis();
        _playingMusic = true;
        _stepper.playNote(MELODY_LA_BAMBA[0].freq);
        Serial.println("\\xF0\\x9F\\x8E\\xB5 Playing music!");

    } else if (cmd == "music_stop") {
        _playingMusic = false;
        _stepper.stopNote();

    } else if (cmd == "reset") {
        _targetReached = false;
        _stop();
        _stepper.resetTurns();
        _led.reset();
        Serial.println("↺ Turn counter reset");

    } else if (cmd == "target") {
        long t = value.toInt();
        if (t > 0) {
            _targetTurns = t;
            if (_targetReached && t > _stepper.getTurns()) {
                _targetReached = false;
                Serial.println("▶ Target extended — motor unblocked");
            }
            Serial.printf("Target: %ld turns\\n", t);
        }

    } else if (cmd == "freerun") {
        _freerun = (value == "true");
        if (_freerun) {
            if (_targetReached) {
                _targetReached = false;
            }
        } else {
            if (_stepper.getTurns() >= _targetTurns) {
                _targetReached = true;
                if (_stepper.isRunning()) {
                    _stop();
                }
            }
        }

    } else if (cmd == "direction") {
        Direction newDir = (value == "cw") ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            if (_stepper.isRunning()) _stepper.forceStop();
            _lateral.stopWinding();
            Serial.printf("Direction: %s\\n", value.c_str());
        }

    } else if (cmd == "mode") {
        _mode = (value == "auto") ? WinderMode::AUTO : WinderMode::MANUAL;
        if (_stepper.isRunning()) _stepper.stop();

    } else if (cmd == "geom_preset") {
        uint8_t idx = (uint8_t)value.toInt();
        _geom.applyPreset(idx);
        updateCarriage();

    } else if (cmd == "geom_total")  { _geom.totalWidth_mm   = value.toFloat(); updateCarriage(); }
      else if (cmd == "geom_bottom") { _geom.flangeBottom_mm = value.toFloat(); updateCarriage(); }
      else if (cmd == "geom_top")    { _geom.flangeTop_mm    = value.toFloat(); updateCarriage(); }
      else if (cmd == "geom_margin") { _geom.margin_mm       = value.toFloat(); updateCarriage(); }
      else if (cmd == "geom_wire")   {
        _geom.wireDiameter_mm = value.toFloat();
    } else if (cmd == "geom_tpp_ofs") {
        _geom.turnsPerPassOffset = value.toInt();
    } else if (cmd == "geom_scatter") {
        float f = value.toFloat();
        if (f >= 0.5f && f <= 5.0f) {
            _geom.scatterFactor = f;
        }

    } else if (cmd == "lat_offset") {
        float mm = value.toFloat();
        if (mm >= 0.0f) {
            _lateral.setHomeOffset(mm);
            updateCarriage();
        }
    }
}
"""

with open("src/WinderApp.cpp", "w") as f:
    f.write(content)

print("Restored and updated src/WinderApp.cpp successfully")
