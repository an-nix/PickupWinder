#include "WinderApp.h"
#include <Arduino.h>

void WinderApp::begin() {
    Serial.begin(115200);
    Serial.println("\n=== Pickup Winder ===");

    // Initialise each subsystem in dependency order.
    _stepper.begin();  // GPIO + FastAccelStepper engine
    _pot.begin();      // Pre-fill ADC filter buffer
    _led.begin();      // Set LED pin as output
    _web.begin();      // WiFi + HTTP + WebSocket

    // Register the command callback so WebSocket messages are routed to
    // _handleCommand() on this instance.
    _web.setCommandCallback([this](const String& cmd, const String& val) {
        _handleCommand(cmd, val);
    });

    if (_web.isConnected()) {
        Serial.printf("→ Web interface: http://%s\n", _web.getIP().c_str());
    }
    Serial.printf("Ready — Geometry: %.1fmm usable, %ld turns/pass\n",
                  _geom.effectiveWidth(), _geom.turnsPerPass());
}

void WinderApp::run() {
    uint32_t now = millis();

    // ── MANUAL mode ───────────────────────────────────────────────────────
    if (_mode == WinderMode::MANUAL) {

        // Sample the potentiometer at a fixed interval to avoid flooding the ADC
        // and to keep the sliding-window filter meaningful.
        if (now - _lastPotMs >= POT_READ_INTERVAL) {
            _lastPotMs = now;
            uint32_t hz        = _pot.readHz();

            // potActive: pot is above the start threshold → user wants the motor running.
            bool potActive = (hz > SPEED_HZ_MIN + POT_DEADZONE_HZ);
            // potStop: pot is below the stop threshold (hysteresis band below potActive).
            // Using a lower threshold prevents chatter around the start point.
            bool potStop   = (hz < SPEED_HZ_MIN + POT_DEADZONE_STOP);

            // Track when the pot returns to zero so we can re-arm the motor
            // after a stop (safety interlock).
            if (potStop) _potWasZero = true;

            // Re-enable only if: motor was disabled, pot returned to zero, pot is now active,
            // AND the target has not been reached (requires explicit reset to clear).
            if (!_motorEnabled && _potWasZero && potActive && !_targetReached) {
                _motorEnabled = true;
                Serial.println("▶ Pot active — motor ready");
            }

            if (_motorEnabled && potActive) {
                // In the approach zone (last APPROACH_TURNS before target),
                // linearly cap the speed from the pot value down to SPEED_HZ_MIN.
                // This ensures the motor is already slow when _stop() is called,
                // making the final deceleration gentle on the wire.
                if (!_freerun && _stepper.isRunning()) {
                    long remaining = (long)_targetTurns - _stepper.getTurns();
                    if (remaining > 0 && remaining <= APPROACH_TURNS) {
                        float ratio = (float)remaining / APPROACH_TURNS;  // 1.0 → 0.0
                        float ratioSq = ratio * ratio;  // quadratic: drops fast then levels off low
                        uint32_t maxHz = SPEED_HZ_MIN + (uint32_t)(ratioSq * (SPEED_HZ_MAX - SPEED_HZ_MIN));
                        hz = min(hz, maxHz);
                    }
                }
                // Update speed setpoint — will ramp to new speed using ACCELERATION.
                _stepper.setSpeedHz(hz);
                // If the motor is not yet running, start it in the configured direction.
                if (!_stepper.isRunning()) {
                    _stepper.start(_direction == Direction::CW);
                    Serial.printf("▶ Start %s — %u Hz\n",
                                  _direction == Direction::CW ? "CW" : "CCW", hz);
                }
            } else if (_motorEnabled && potStop && _stepper.isRunning()) {
                // Pot returned to zero while motor was running → immediate stop.
                // forceStop() is used here (no deceleration ramp) because the user
                // explicitly turned the pot to zero.
                _stepper.forceStop();
                _led.reset();
            }

            // Update the traverse guide LED based on the current turn count.
            // The LED toggles at each new pass (aller / retour).
            _led.update(_stepper.getTurns(), _geom.turnsPerPass(), _stepper.isRunning());
        }

        // Auto-stop when the target turn count is reached (non-freerun mode).
        // _motorEnabled guard ensures this fires only once — _stop() sets
        // _motorEnabled=false so the condition cannot trigger again during deceleration.
        if (_motorEnabled && !_freerun && _stepper.isRunning() && _stepper.getTurns() >= _targetTurns) {
            _targetReached = true;  // Block restart until explicit reset
            _stop();
            Serial.printf("✓ Winding complete! %ld turns done.\n", _stepper.getTurns());
        }
    }

    // ── Deferred driver disable ───────────────────────────────────────────
    // After a controlled stop (_stop()), the motor decelerates smoothly.
    // Once isRunning() returns false the deceleration is complete and we can
    // safely cut power to the driver coils.
    if (_pendingDisable && !_stepper.isRunning()) {
        _stepper.disableDriver();
        _pendingDisable = false;
    }

    // ── AUTO mode (future: lateral guide axis) ────────────────────────────
    // if (_mode == WinderMode::AUTO) { ... }

    // ── WebSocket status push ─────────────────────────────────────────────
    // Send a full machine state snapshot to all connected clients at a fixed
    // interval to keep the UI up to date without saturating the network.
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
            _led.getCurrentPass(),
            _geom.effectiveWidth(),
            _geom.totalWidth_mm, _geom.flangeBottom_mm, _geom.flangeTop_mm,
            _geom.margin_mm, _geom.wireDiameter_mm
        });
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

void WinderApp::_stop() {
    // Disable the motor control logic immediately so no further commands
    // (including repeated auto-stop triggers) can be processed.
    _motorEnabled   = false;
    // Force the pot-to-zero interlock: the motor won't restart until the pot
    // physically returns to zero. Without this, _potWasZero stays true and
    // _motorEnabled would be re-set on the very next pot read (20ms later),
    // calling start() while the motor is still decelerating → conflict/noise.
    _potWasZero     = false;
    // Request deferred driver disable: the driver will be cut once the
    // deceleration ramp completes (checked in run() via _pendingDisable).
    _pendingDisable = true;
    // Initiate a smooth deceleration ramp to zero.
    _stepper.stop();
    _led.reset();
    Serial.println("■ Stopped — return pot to 0 to restart");
}

void WinderApp::_handleCommand(const String& cmd, const String& value) {
    if (cmd == "stop") {
        // Emergency/manual stop from the web UI.
        _stop();

    } else if (cmd == "reset") {
        // Stop, reset the turn counter, LED guide, and unblock the motor.
        _targetReached = false;
        _stop();
        _stepper.resetTurns();
        _led.reset();
        Serial.println("↺ Turn counter reset");

    } else if (cmd == "target") {
        // Update the target turn count for auto-stop mode.
        long t = value.toInt();
        if (t > 0) {
            _targetTurns = t;
            // If the new target is beyond the current turn count, unblock the motor
            // so the user can continue winding without pressing Reset.
            if (_targetReached && t > _stepper.getTurns()) {
                _targetReached = false;
                Serial.println("▶ Target extended — motor unblocked");
            }
            Serial.printf("Target: %ld turns\n", t);
        }

    } else if (cmd == "freerun") {
        // Toggle between free-run (no auto-stop) and target mode.
        _freerun = (value == "true");
        Serial.printf("Mode: %s\n", _freerun ? "FreeRun" : "Target");

    } else if (cmd == "direction") {
        Direction newDir = (value == "cw") ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            // forceStop() instead of stop() to avoid coasting through the
            // resonance zone during a slow deceleration ramp.
            if (_stepper.isRunning()) _stepper.forceStop();
            Serial.printf("Direction: %s\n", value.c_str());
        }

    } else if (cmd == "mode") {
        // Switch between MANUAL and AUTO operating modes.
        _mode = (value == "auto") ? WinderMode::AUTO : WinderMode::MANUAL;
        if (_stepper.isRunning()) _stepper.stop();
        Serial.printf("Mode: %s\n", value.c_str());

    // ── Winding geometry ──────────────────────────────────────────────────
    } else if (cmd == "geom_preset") {
        // Apply a predefined bobbin profile (Strat, Tele, P90, Humbucker).
        uint8_t idx = (uint8_t)value.toInt();
        _geom.applyPreset(idx);
        Serial.printf("Bobbin preset: %s — %ld turns/pass\n",
                      BOBBIN_PRESETS[idx].name, _geom.turnsPerPass());

    // Individual geometry parameter updates from the web UI.
    } else if (cmd == "geom_total")  { _geom.totalWidth_mm   = value.toFloat(); }
      else if (cmd == "geom_bottom") { _geom.flangeBottom_mm = value.toFloat(); }
      else if (cmd == "geom_top")    { _geom.flangeTop_mm    = value.toFloat(); }
      else if (cmd == "geom_margin") { _geom.margin_mm       = value.toFloat(); }
      else if (cmd == "geom_wire")   {
        _geom.wireDiameter_mm = value.toFloat();
        Serial.printf("Wire: %.4f mm — %ld turns/pass\n",
                      _geom.wireDiameter_mm, _geom.turnsPerPass());
    }
}
