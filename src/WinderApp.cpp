#include "WinderApp.h"
#include <Arduino.h>

void WinderApp::begin() {
    Serial.begin(115200);
    Serial.println("\n=== Pickup Winder ===");

    _recipeStore.begin();
    _recipe = _captureRecipe();
    if (_recipeStore.load(_recipe)) {
        Serial.println("[Recipe] Recette restaurée depuis la NVS.");
    } else {
        Serial.println("[Recipe] Recette par défaut chargée.");
    }
    _applyRecipe(_recipe, false);

    // Initialise each subsystem in dependency order.
    _engine.init();        // Une seule engine FastAccelStepper pour tous les steppers
    _stepper.begin(_engine);  // GPIO + stepper bobine
    _pot.begin();      // Pre-fill ADC filter buffer
    _led.begin();      // Set LED pin as output
    _web.begin();      // WiFi + HTTP + WebSocket  (peut prendre 2-5 s)
    _link.begin();     // UART2 liaison vers ESP écran
    _lateral.begin(_engine, _recipe.latOffsetMm);  // GPIO + stepper latéral + homing automatique

    // Register the command callback so WebSocket messages are routed to
    // _handleCommand() on this instance.
    _web.setCommandCallback([this](const String& cmd, const String& val) {
        _handleCommand(cmd, val);
    });
    _web.setRecipeProvider([this]() {
        return _recipeStore.toJson(_captureRecipe());
    });

    _activePlan = _planner.getPlan(0, 0.0f);

    if (_web.isConnected()) {
        Serial.printf("→ Web interface: http://%s\n", _web.getIP().c_str());
    }
    Serial.printf("Ready — Geometry: %.1fmm usable, %ld turns/pass, profile=%s\n",
                  _geom.effectiveWidth(), _geom.turnsPerPass(),
                  WindingPatternPlanner::styleName(_recipe.style));
}

void WinderApp::run() {
    uint32_t now = millis();

    _handleLateralEvents();
    _handlePotCycle();
    _checkAutoStop();
    _applyDeferredDisable();
    _pollSerialLink(now);
    _pushWebStatus(now);
}

void WinderApp::_handleLateralEvents() {
    // Machine d'états du homing latéral — non-bloquante, appelée à chaque itération.
    _lateral.update();
    if (!_lateral.consumePausedAtReversal()) return;

    _motorEnabled = false;
    // On garde _startRequested = true : l'utilisateur peut reprendre au pot.
    _pausedForVerification = true;
    _pauseOnFirstReversal  = false;
    _inVerificationRun     = false;  // passe de vérification terminée
    _midWindingPaused      = false;
    _resumeFromCurrentPos  = false;
    _pendingDisable        = true;
    _potWasZero            = false;
    // Arrêt net pour synchroniser avec l'arrêt latéral au point de butée.
    _stepper.forceStop();
    Serial.println("[Start] ⏸ Première inversion — vérifie la butée max et remonte le pot pour reprendre.");
}

void WinderApp::_handlePotCycle() {
    uint32_t now = millis();
    if (now - _lastPotMs < POT_READ_INTERVAL) return;
    _lastPotMs = now;

    uint32_t hz = _pot.readHz();
    bool potActive = (hz > 0);
    bool potStop   = (hz == 0);

    // Interlock : le moteur ne redémarre qu'après retour au zéro physique.
    if (potStop) _potWasZero = true;

    // Si Start n'est pas armé, toujours revenir à la position 0.
    if (!_startRequested && _lateral.getState() == LatState::HOMED
        && !_lateral.isBusy() && !_lateral.isAtZero()) {
        _lateral.parkAtZero();
    }

    // Start armé mais moteur pas encore lancé : garder le chariot à la position de départ.
    if (_startRequested && !_stepper.isRunning() && _lateral.getState() == LatState::HOMED
        && !_lateral.isBusy() && !_pausedForVerification && !_resumeFromCurrentPos) {
        _lateral.prepareStartPosition(_windingStartMm());
    }

    if (!_motorEnabled && _startRequested && _potWasZero && potActive && !_targetReached
                       && _readyForSpin()) {
        _motorEnabled = true;
        Serial.println("▶ Start confirmé — moteur prêt");
    }

    if (_motorEnabled && potActive) {
        _runWindingAtHz(hz);
    } else if (_motorEnabled && potStop && _stepper.isRunning()) {
        _pause();
    }

    _led.update(_stepper.getTurns(), max(1L, _activePlan.turnsPerPass), _stepper.isRunning());
}

bool WinderApp::_readyForSpin() const {
    return (_pausedForVerification || _resumeFromCurrentPos)
        ? (_lateral.isHomed() && !_lateral.isBusy())
        : (_lateral.isHomed() && _lateral.isPositionedForStart());
}

void WinderApp::_runWindingAtHz(uint32_t hz) {
    // Zone d'approche (APPROACH_TURNS derniers tours avant la cible).
    if (!_freerun && _stepper.isRunning()) {
        long remaining = (long)_targetTurns - _stepper.getTurns();
        if (remaining > 0 && remaining <= APPROACH_TURNS) {
            float ratio = (float)remaining / APPROACH_TURNS;
            uint32_t maxHz = APPROACH_SPEED_HZ_FLOOR
                           + (uint32_t)(ratio * (float)(SPEED_HZ_MAX - APPROACH_SPEED_HZ_FLOOR));
            hz = min(hz, maxHz);
        }
    }

    // Passe de vérification : limiter la vitesse pour un arrêt synchronisé avec le latéral.
    if (_inVerificationRun) hz = min(hz, (uint32_t)VERIFY_SPEED_HZ_MAX);

    _activePlan = _buildTraversePlan(hz);

    // Ralentissement aux demi-tours latéraux.
    if (_lateral.isReversing()) {
        hz = max((uint32_t)SPEED_HZ_MIN,
                 (uint32_t)((float)hz * LAT_REVERSAL_SLOWDOWN));
    }

    _stepper.setSpeedHz(hz);
    if (!_stepper.isRunning()) {
        bool forward = (_direction == Direction::CW) != (bool)WINDING_MOTOR_INVERTED;
        if (_pauseOnFirstReversal) {
            _lateral.armPauseOnNextReversal();
            _pauseOnFirstReversal = false;
            _inVerificationRun = true;
        }
        _midWindingPaused = false;
        _resumeFromCurrentPos = false;
        _stepper.start(forward);
        _lateral.startWinding(hz, _activePlan.turnsPerPass,
                              _windingStartMm(), _windingEndMm(),
                              _activePlan.speedScale);
        _pausedForVerification = false;
        Serial.printf("▶ Start %s — %u Hz — profil=%s tpp=%ld scale=%.2f\n",
                      _direction == Direction::CW ? "CW" : "CCW", hz,
                      WindingPatternPlanner::styleName(_recipe.style),
                      _activePlan.turnsPerPass, _activePlan.speedScale);
        return;
    }

    if (_lateral.getState() == LatState::HOMED) {
        _lateral.startWinding(hz, _activePlan.turnsPerPass,
                              _windingStartMm(), _windingEndMm(),
                              _activePlan.speedScale);
    } else {
        _lateral.updateWinding(hz, _activePlan.turnsPerPass,
                               _windingStartMm(), _windingEndMm(),
                               _activePlan.speedScale);
    }
}

void WinderApp::_checkAutoStop() {
    if (_motorEnabled && !_freerun && _stepper.isRunning() && _stepper.getTurns() >= _targetTurns) {
        _targetReached = true;
        _stop();
        Serial.printf("✓ Winding complete! %ld turns done.\n", _stepper.getTurns());
    }
}

void WinderApp::_applyDeferredDisable() {
    if (_pendingDisable && !_stepper.isRunning()) {
        _stepper.disableDriver();
        _pendingDisable = false;
    }
}

void WinderApp::_pollSerialLink(uint32_t now) {
    _link.poll([this](const String& cmd, const String& val) {
        _handleCommand(cmd, val);
    });

    if (now - _lastLinkMs < LINK_UPDATE_MS) return;
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

void WinderApp::_pushWebStatus(uint32_t now) {
    if (now - _lastWsMs < WS_UPDATE_MS) return;
    _lastWsMs = now;
    _web.sendUpdate({
        _stepper.getRPM(),
        _stepper.getSpeedHz(),
        _stepper.getTurns(),
        (long)_targetTurns,
        _stepper.isRunning(),
        (bool)_motorEnabled,
        (bool)_startRequested,
        _lateral.isPositionedForStart(),
        _pausedForVerification,
        (bool)_freerun,
        _direction == Direction::CW,
        false,
        _geom.turnsPerPass(),
        _geom.turnsPerPassCalc(),
        _geom.turnsPerPassOffset,
        _geom.scatterFactor,
        (int)_lateral.getPassCount(),
        _activePlan.turnsPerPass,
        _activePlan.speedScale,
        _lateral.getTraversalProgress(),
        _lateral.getCurrentPositionMm(),
        _windingStartMm(), _windingEndMm(), _geom.windingStartTrim_mm, _geom.windingEndTrim_mm,
        _geom.effectiveWidth(),
        _geom.totalWidth_mm, _geom.flangeBottom_mm, _geom.flangeTop_mm,
        _geom.margin_mm, _geom.wireDiameter_mm,
        _lateral.getHomeOffset(),
        WindingPatternPlanner::styleKey(_recipe.style).c_str(),
        _recipe.seed,
        _recipe.layerJitterPct,
        _recipe.layerSpeedPct,
        _recipe.humanTraversePct,
        _recipe.humanSpeedPct
    });
}

// ── Private helpers ───────────────────────────────────────────────────────────

void WinderApp::_pause() {
    _motorEnabled        = false;
    _potWasZero          = false;
    _midWindingPaused    = true;
    _resumeFromCurrentPos = true;
    _pendingDisable      = true;
    _pauseOnFirstReversal = false;
    _stepper.stop();
    _lateral.stopWinding();
    Serial.println("⏸ Pause — reprise depuis position courante");
}

void WinderApp::_stop() {
    // Disable the motor control logic immediately so no further commands
    // (including repeated auto-stop triggers) can be processed.
    _motorEnabled   = false;
    // Force the pot-to-zero interlock: the motor won't restart until the pot
    // physically returns to zero. Without this, _potWasZero stays true and
    // _motorEnabled would be re-set on the very next pot read (20ms later),
    // calling start() while the motor is still decelerating → conflict/noise.
    _potWasZero     = false;
    _startRequested        = false;
    _pauseOnFirstReversal  = false;
    _pausedForVerification = false;
    _midWindingPaused      = false;
    _inVerificationRun     = false;
    _resumeFromCurrentPos  = false;
    // Request deferred driver disable: the driver will be cut once the
    // deceleration ramp completes (checked in run() via _pendingDisable).
    _pendingDisable = true;
    // Initiate a smooth deceleration ramp to zero.
    _stepper.stop();
    // Arrêter le guide-fil proprement (décélération douce, retour à HOMED).
    _lateral.stopWinding();
    _lateral.parkAtZero();
    _planner.reset();
    _activePlan = _planner.getPlan(_stepper.getTurns(), 0.0f);
    _led.reset();
    Serial.println("■ Stopped — chariot retour position 0");
}

bool WinderApp::_parametersLocked() const {
    return _startRequested || _motorEnabled || _stepper.isRunning()
        || _pausedForVerification || _midWindingPaused;
}

void WinderApp::_refreshStartPositionIfArmed() {
    // Ne JAMAIS recaler le chariot pendant une pause :
    // pause = reprise depuis position courante, sans retour à la position initiale.
    if (_startRequested
        && !_midWindingPaused
        && !_resumeFromCurrentPos
        && !_stepper.isRunning()
        && _lateral.getState() == LatState::HOMED
        && !_lateral.isBusy()) {
        _lateral.prepareStartPosition(_windingStartMm());
    }
}

bool WinderApp::_handleImmediateCommand(const String& cmd, const String& value) {
    if (cmd == "stop") {
        _stop();
        return true;
    }

    if (cmd == "reset") {
        _targetReached = false;
        _stop();
        _stepper.resetTurns();
        _planner.reset();
        _activePlan = _planner.getPlan(0, 0.0f);
        _pauseOnFirstReversal = false;
        _pausedForVerification = false;
        _led.reset();
        Serial.println("↺ Turn counter reset");
        return true;
    }

    if (cmd == "start") {
        if (_startRequested) return true;
        if (_lateral.getState() != LatState::HOMED || _lateral.isBusy()) {
            Serial.println("[Start] Impossible: axe latéral non prêt.");
            return true;
        }
        if (!_lateral.isAtZero()) {
            Serial.println("[Start] Refusé: chariot pas en position 0.");
            return true;
        }
        if (_targetReached) _targetReached = false;
        _startRequested        = true;
        _motorEnabled          = false;
        _pausedForVerification = false;
        _midWindingPaused      = false;
        _inVerificationRun     = false;
        _resumeFromCurrentPos  = false;
        _pauseOnFirstReversal  = (_stepper.getTurns() == 0);
        _lateral.prepareStartPosition(_windingStartMm());
        Serial.printf("[Start] Fenêtre bobinage %.2f → %.2f mm\n",
                      _windingStartMm(), _windingEndMm());
        return true;
    }

    return false;
}

bool WinderApp::_handleGeometryCommand(const String& cmd, const String& value) {
    if (cmd == "geom_start_trim") {
        _geom.windingStartTrim_mm = constrain(value.toFloat(), -5.0f, 5.0f);
        _refreshStartPositionIfArmed();
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_end_trim") {
        _geom.windingEndTrim_mm = constrain(value.toFloat(), -5.0f, 5.0f);
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_start_trim_nudge") {
        float delta = constrain(value.toFloat(), -1.0f, 1.0f);
        _geom.windingStartTrim_mm = constrain(_geom.windingStartTrim_mm + delta, -5.0f, 5.0f);
        _refreshStartPositionIfArmed();
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_end_trim_nudge") {
        float delta = constrain(value.toFloat(), -1.0f, 1.0f);
        _geom.windingEndTrim_mm = constrain(_geom.windingEndTrim_mm + delta, -5.0f, 5.0f);
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_preset") {
        uint8_t idx = (uint8_t)value.toInt();
        _geom.applyPreset(idx);
        _refreshStartPositionIfArmed();
        Serial.printf("Bobbin preset: %s — %ld turns/pass\n",
                      BOBBIN_PRESETS[idx].name, _geom.turnsPerPass());
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_total")  { _geom.totalWidth_mm   = value.toFloat(); _refreshStartPositionIfArmed(); _saveRecipe(); return true; }
    if (cmd == "geom_bottom") { _geom.flangeBottom_mm = value.toFloat(); _refreshStartPositionIfArmed(); _saveRecipe(); return true; }
    if (cmd == "geom_top")    { _geom.flangeTop_mm    = value.toFloat(); _saveRecipe(); return true; }
    if (cmd == "geom_margin") { _geom.margin_mm       = value.toFloat(); _refreshStartPositionIfArmed(); _saveRecipe(); return true; }

    if (cmd == "geom_wire") {
        _geom.wireDiameter_mm = value.toFloat();
        Serial.printf("Wire: %.4f mm — %ld turns/pass (calc: %ld)\n",
                      _geom.wireDiameter_mm, _geom.turnsPerPass(), _geom.turnsPerPassCalc());
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_tpp_ofs") {
        _geom.turnsPerPassOffset = value.toInt();
        Serial.printf("Turns/pass offset: %+ld (calc %ld → effective %ld)\n",
                      _geom.turnsPerPassOffset, _geom.turnsPerPassCalc(),
                      _geom.turnsPerPass());
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_scatter") {
        float f = value.toFloat();
        if (f >= 0.5f && f <= 5.0f) {
            _geom.scatterFactor = f;
            Serial.printf("Scatter factor: %.2f → %ld tours/pass\n",
                          _geom.scatterFactor, _geom.turnsPerPass());
            _saveRecipe();
        }
        return true;
    }

    return false;
}

bool WinderApp::_handlePatternCommand(const String& cmd, const String& value) {
    if (cmd == "winding_style") {
        _recipe.style = WindingPatternPlanner::styleFromString(value);
        _planner.setRecipe(_captureRecipe());
        Serial.printf("Winding style: %s\n", WindingPatternPlanner::styleName(_recipe.style));
        _saveRecipe();
        return true;
    }

    if (cmd == "winding_seed") {
        uint32_t seed = (uint32_t)((value.toInt() > 0) ? value.toInt() : 1);
        _recipe.seed = seed;
        _planner.setRecipe(_captureRecipe());
        Serial.printf("Winding seed: %lu\n", (unsigned long)_recipe.seed);
        _saveRecipe();
        return true;
    }

    if (cmd == "winding_layer_jitter") {
        _recipe.layerJitterPct = constrain(value.toFloat(), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (cmd == "winding_layer_speed") {
        _recipe.layerSpeedPct = constrain(value.toFloat(), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (cmd == "winding_human_traverse") {
        _recipe.humanTraversePct = constrain(value.toFloat(), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (cmd == "winding_human_speed") {
        _recipe.humanSpeedPct = constrain(value.toFloat(), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    return false;
}

void WinderApp::_handleCommand(const String& cmd, const String& value) {
    if (_handleImmediateCommand(cmd, value)) return;
    if (_handleGeometryCommand(cmd, value)) return;

    if (_parametersLocked()) {
        Serial.printf("[Lock] Paramètre ignoré pendant le bobinage: %s\n", cmd.c_str());
        return;
    }

    if (cmd == "target") {
        long t = value.toInt();
        if (t > 0) {
            _targetTurns = t;
            Serial.printf("Target: %ld turns\n", t);
            _saveRecipe();
        }
        return;
    }

    if (cmd == "freerun") {
        _freerun = (value == "true");
        Serial.printf("Mode: %s\n", _freerun ? "FreeRun" : "Target");
        _saveRecipe();
        return;
    }

    if (cmd == "direction") {
        Direction newDir = (value == "cw") ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            Serial.printf("Direction: %s\n", value.c_str());
            _saveRecipe();
        }
        return;
    }

    if (_handlePatternCommand(cmd, value)) return;

    if (cmd == "recipe_import") {
        WindingRecipe imported;
        if (_recipeStore.fromJson(value, imported)) {
            _applyRecipe(imported, true);
            Serial.println("[Recipe] Recette importée.");
        } else {
            Serial.println("[Recipe] ERREUR : JSON recette invalide.");
        }
        return;
    }

    if (cmd == "lat_offset") {
        float mm = value.toFloat();
        if (mm >= 0.0f) {
            _lateral.setHomeOffset(mm);
            _startRequested = false;
            _motorEnabled = false;
            _pausedForVerification = false;
            _midWindingPaused = false;
            _inVerificationRun = false;
                _resumeFromCurrentPos = false;
            // Appliquer immédiatement le nouvel offset : homing complet,
            // puis retour automatique à la position 0 logique (avec offset).
            _lateral.rehome();
            Serial.printf("[Lateral] Offset modifié à %.2f mm — rehoming lancé.\n", mm);
            _saveRecipe();
        }
        return;
    }
}

TraversePlan WinderApp::_buildTraversePlan(uint32_t windingHz) const {
    (void)windingHz;
    return _planner.getPlan(_stepper.getTurns(), _lateral.getTraversalProgress());
}

void WinderApp::_applyRecipe(const WindingRecipe& recipe, bool persist) {
    _recipe = recipe;
    _geom = recipe.geometry;
    _targetTurns = recipe.targetTurns;
    _freerun = recipe.freerun;
    _direction = recipe.directionCW ? Direction::CW : Direction::CCW;
    _startRequested = false;
    _motorEnabled = false;
    _pauseOnFirstReversal = false;
    _pausedForVerification = false;
    _midWindingPaused = false;
    _inVerificationRun = false;
    _resumeFromCurrentPos = false;
    _lateral.setHomeOffset(recipe.latOffsetMm);
    _planner.setRecipe(_recipe);
    _activePlan = _planner.getPlan(_stepper.getTurns(), 0.0f);
    if (persist) _saveRecipe();
}

WindingRecipe WinderApp::_captureRecipe() const {
    WindingRecipe recipe = _recipe;
    recipe.targetTurns = _targetTurns;
    recipe.freerun = _freerun;
    recipe.directionCW = (_direction == Direction::CW);
    recipe.geometry = _geom;
    recipe.latOffsetMm = _lateral.getHomeOffset();
    return recipe;
}

void WinderApp::_saveRecipe() {
    _recipe = _captureRecipe();
    _planner.setRecipe(_recipe);
    _recipeStore.save(_recipe);
}
