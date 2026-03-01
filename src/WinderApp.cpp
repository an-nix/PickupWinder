#include "WinderApp.h"
#include <Arduino.h>

void WinderApp::begin() {
    Serial.begin(115200);
    Serial.println("\n=== Pickup Winder ===");

    _stepper.begin();
    _pot.begin();
    _led.begin();
    _web.begin();

    _web.setCommandCallback([this](const String& cmd, const String& val) {
        _handleCommand(cmd, val);
    });

    if (_web.isConnected()) {
        Serial.printf("→ Interface web : http://%s\n", _web.getIP().c_str());
    }
    Serial.printf("Prêt — Géométrie : %.1fmm utile, %ld tours/passage\n",
                  _geom.effectiveWidth(), _geom.turnsPerPass());
}

void WinderApp::run() {
    uint32_t now = millis();

    // ── Mode MANUEL ───────────────────────────────────────────────────────
    if (_mode == WinderMode::MANUAL) {

        if (now - _lastPotMs >= POT_READ_INTERVAL) {
            _lastPotMs = now;
            uint32_t hz        = _pot.readHz();
            bool     potActive = (hz > SPEED_HZ_MIN + POT_DEADZONE_HZ);
            bool potStop   = (hz < SPEED_HZ_MIN + POT_DEADZONE_STOP);  // seuil d'arrêt plus bas

            if (potStop) _potWasZero = true;

            if (!_motorEnabled && _potWasZero && potActive) {
                _motorEnabled = true;
                Serial.println("▶ Potentiomètre actif — moteur prêt");
            }

            if (_motorEnabled && potActive) {
                _stepper.setSpeedHz(hz);
                if (!_stepper.isRunning()) {
                    _stepper.start(_direction == Direction::CW);
                    Serial.printf("▶ Démarrage %s — %u Hz\n",
                                  _direction == Direction::CW ? "CW" : "CCW", hz);
                }
            } else if (potStop && _stepper.isRunning()) {
                _stepper.forceStop();   // Arrêt immédiat, pas de décélération
                _led.reset();
            }

            // Mise à jour LED guide aller-retour
            _led.update(_stepper.getTurns(), _geom.turnsPerPass(), _stepper.isRunning());
        }

        // Arrêt automatique sur cible (mode non-freerun)
        if (!_freerun && _stepper.isRunning() && _stepper.getTurns() >= _targetTurns) {
            _stop();
            Serial.printf("✓ Bobinage terminé ! %ld tours effectués.\n", _stepper.getTurns());
        }
    }

    // ── Désactivation différée du driver (après décélération complète) ────────
    if (_pendingDisable && !_stepper.isRunning()) {
        _stepper.disableDriver();
        _pendingDisable = false;
    }

    // ── Mode AUTO (futur axe latéral) ─────────────────────────────────────
    // if (_mode == WinderMode::AUTO) { ... }

    // ── Envoi de l'état via WebSocket ─────────────────────────────────────
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

// ── Commandes privées ─────────────────────────────────────────────────────────

void WinderApp::_stop() {
    _motorEnabled    = false;
    _pendingDisable  = true;   // le driver sera désactivé une fois la décélération terminée
    _stepper.stop();
    _led.reset();
    Serial.println("■ Arrêt — ramenez le pot à 0 pour redémarrer");
}

void WinderApp::_handleCommand(const String& cmd, const String& value) {
    if (cmd == "stop") {
        _stop();

    } else if (cmd == "reset") {
        _stop();
        _stepper.resetTurns();
        _led.reset();
        Serial.println("↺ Compteur remis à zéro");

    } else if (cmd == "target") {
        long t = value.toInt();
        if (t > 0) { _targetTurns = t; Serial.printf("Cible : %ld tours\n", t); }

    } else if (cmd == "freerun") {
        _freerun = (value == "true");
        Serial.printf("Mode : %s\n", _freerun ? "FreeRun" : "Cible");

    } else if (cmd == "direction") {
        Direction newDir = (value == "cw") ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            if (_stepper.isRunning()) _stepper.forceStop();
            Serial.printf("Sens : %s\n", value.c_str());
        }

    } else if (cmd == "mode") {
        _mode = (value == "auto") ? WinderMode::AUTO : WinderMode::MANUAL;
        if (_stepper.isRunning()) _stepper.stop();
        Serial.printf("Mode : %s\n", value.c_str());

    // ── Géométrie ─────────────────────────────────────────────────────────
    } else if (cmd == "geom_preset") {
        uint8_t idx = (uint8_t)value.toInt();
        _geom.applyPreset(idx);
        Serial.printf("Préréglage bobine : %s — %ld tours/passage\n",
                      BOBBIN_PRESETS[idx].name, _geom.turnsPerPass());

    } else if (cmd == "geom_total")  { _geom.totalWidth_mm   = value.toFloat(); }
      else if (cmd == "geom_bottom") { _geom.flangeBottom_mm = value.toFloat(); }
      else if (cmd == "geom_top")    { _geom.flangeTop_mm    = value.toFloat(); }
      else if (cmd == "geom_margin") { _geom.margin_mm       = value.toFloat(); }
      else if (cmd == "geom_wire")   {
        _geom.wireDiameter_mm = value.toFloat();
        Serial.printf("Fil : %.4f mm — %ld tours/passage\n",
                      _geom.wireDiameter_mm, _geom.turnsPerPass());
    }
}
