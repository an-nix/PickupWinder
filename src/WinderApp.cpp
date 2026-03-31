#include "WinderApp.h"
#include <Arduino.h>
#include "Diag.h"

// ═══════════════════════════════════════════════════════════════════════════════
// Initialization
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::begin() {
    // burst mode removed

    _recipeStore.begin();
    _recipe = _captureRecipe();
    if (_recipeStore.load(_recipe)) Diag::info("[Recipe] Recipe restored from NVS.");
    else Diag::info("[Recipe] Default recipe loaded.");
    _applyRecipe(_recipe, false);
    _engine.init();
    _stepper.begin(_engine);
    _lateral.begin(_engine, _recipe.latOffsetMm);
    _activePlan = _planner.getPlan(0, 0.0f);
    Diag::infof("[Winder] Ready — %.1fmm usable, %ld tpp, profile=%s",
            _geom.effectiveWidth(), _geom.turnsPerPass(),
            WindingPatternPlanner::styleName(_recipe.style));
}

void WinderApp::tick() {
    if (_pauseRequested) {
        _pauseRequested = false;
        if (_state != WindingState::IDLE && _state != WindingState::TARGET_REACHED) {
            _toPaused();
        }
    }

    _handleLateralEvents();
    _processInputHz(_inputHz);
    _checkAutoStop();
    _applyDeferredDisable();
}

// ═══════════════════════════════════════════════════════════════════════════════
// State transitions — every _toXxx() is self-contained
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::_toIdle() {
    const WindingState prev = _state;
    _state              = WindingState::IDLE;
    _pendingDisable     = true;
    // burst mode removed
    _verifyLowPending   = false;
    _verifyHighPending  = false;
    _positioningToLow   = false;
    _endPosArmed        = false;
    _stepper.stop();
    _lateral.stopWinding();
    _lateral.parkAtZero();
    _stepper.resetTurns();
    _planner.reset();
    _activePlan = _planner.getPlan(0, 0.0f);
    Diag::infof("[IDLE] %s -> IDLE — carriage to home, counter reset",
            windingStateName(prev));
}

void WinderApp::_toWinding() {
    _state              = WindingState::WINDING;
    _pendingDisable     = false;
    _positioningToLow   = false;
    _endPosArmed        = false;
    Diag::infof("[WINDING] %.2f -> %.2f mm",
            _windingStartMm(), _windingEndMm());
}

void WinderApp::_toPaused() {
    _state              = WindingState::PAUSED;
    _pendingDisable     = true;
    _stepper.stop();
    _lateral.stopWinding();
    Diag::info("[PAUSED] Resume when pot goes up or press Start");
}

void WinderApp::_toTargetReached() {
    _state              = WindingState::TARGET_REACHED;
    _pendingDisable     = true;
    _stepper.stop();
    _lateral.stopWinding();
    _endPosArmed        = false;
    Diag::infof("[TARGET_REACHED] %ld turns — raise target or press Stop",
            _stepper.getTurns());
}

void WinderApp::_toRodage() {
    _state              = WindingState::RODAGE;
    _pendingDisable     = true;
    _stepper.stop();
    _lateral.stopWinding();
    _rodagePassDone     = 0;
    _rodageFwd          = true;
    if (_lateral.isHomed())
        _lateral.prepareStartPosition(_rodageDistMm, LAT_RODAGE_SPEED_HZ);
    Diag::infof("[RODAGE] %d passes, dist=%.1f mm", _rodagePasses, _rodageDistMm);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Lateral events
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::_handleLateralEvents() {
    _lateral.update();

    // Verify step 1: positioning to low bound completed
    if (_positioningToLow && _lateral.isHomed() && !_lateral.isBusy()) {
        _positioningToLow   = false;
        _verifyLowPending   = false;
        _state              = WindingState::PAUSED;
        _pendingDisable     = true;
        Diag::infof("[VERIFY] Low bound reached at %.2f mm — press Start to wind toward high",
                _windingStartMm());
        return;
    }

    // Winding: lateral stopped at an armed bound
    if (_state == WindingState::WINDING && _lateral.consumePausedAtReversal()) {
        if (_verifyHighPending) {
            _verifyHighPending = false;
            _toPaused();
            Diag::infof("[VERIFY] High bound reached at %.2f mm — press Start to begin winding",
                    _windingEndMm());
        } else {
            _toPaused();
            Diag::info("[WINDING] Bound stop — PAUSED");
        }
        return;
    }

    // Break-in shuttle mode.
    if (_state == WindingState::RODAGE && _lateral.isHomed()) {
        if (_rodageFwd) {
            _rodageFwd = false;
            _lateral.prepareStartPosition(0.0f, LAT_RODAGE_SPEED_HZ);
        } else {
            _rodagePassDone++;
            Diag::infof("[RODAGE] Pass %d/%d", _rodagePassDone, _rodagePasses);
            if (_rodagePassDone >= _rodagePasses) {
                Diag::infof("[RODAGE] Complete (%d passes) — IDLE", _rodagePassDone);
                _toIdle();
            } else {
                _rodageFwd = true;
                _lateral.prepareStartPosition(_rodageDistMm, LAT_RODAGE_SPEED_HZ);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Pot cycle — Start button = pot at max, Pause button = pot at zero
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::_processInputHz(uint32_t hz) {
    switch (_state) {
    case WindingState::IDLE:
        if (_lateral.isHomed() && !_lateral.isBusy() && !_lateral.isAtZero())
            _lateral.parkAtZero();
        break;

    case WindingState::WINDING:
        if (hz > 0) {
            _runWindingAtHz(hz);
        } else {
            _toPaused();
        }
        break;

    case WindingState::PAUSED:
        // Pot level is handled by SessionController; do not auto-start from WinderApp.
        break;

    case WindingState::TARGET_REACHED:
    case WindingState::RODAGE:
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Motor control
// ═══════════════════════════════════════════════════════════════════════════════

bool WinderApp::_readyForSpin() const {
    return _lateral.isHomed() && !_lateral.isBusy();
}

void WinderApp::_runWindingAtHz(uint32_t hz) {
    hz = min(hz, (uint32_t)_maxSpeedHz);

    // 1. Approach zone slowdown (near target)
    if (!_freerun && _stepper.isRunning()) {
        long remaining = (long)_targetTurns - _stepper.getTurns();
        if (remaining > 0 && remaining <= APPROACH_TURNS) {
            float ratio = (float)remaining / APPROACH_TURNS;
            uint32_t maxHz = APPROACH_SPEED_HZ_FLOOR
                           + (uint32_t)(ratio * (float)(SPEED_HZ_MAX - APPROACH_SPEED_HZ_FLOOR));
            hz = min(hz, maxHz);
        }
    }

    // 2. Build traverse plan
    _activePlan = _buildTraversePlan(hz);
    float traverseScale = _activePlan.speedScale;

    // 3. End position prediction — adjust traverseScale to land on desired bound
    if (_state == WindingState::WINDING && _endPosArmed && _recipe.endPos != WindingEndPos::NONE) {
        const long remainingTurns = max(0L, (long)_targetTurns - _stepper.getTurns());
        const long holdTurns = max(1L, (long)_recipe.endPosTurns);
        const float moveTurnsRemaining = max(0.25f, (float)remainingTurns - (float)holdTurns);
        const float progress = constrain(_lateral.getTraversalProgress(), 0.0f, 1.0f);
        const LatState latState = _lateral.getState();

        float pathUnits = 0.0f;
        bool atTargetBound = false;
        if (_recipe.endPos == WindingEndPos::TOP) {
            if (latState == LatState::WINDING_FWD) pathUnits = 1.0f - progress;
            else if (latState == LatState::WINDING_BWD) pathUnits = 2.0f - progress;
            else if (latState == LatState::HOMED
                     && fabsf(_lateral.getCurrentPositionMm() - _windingEndMm()) <= 0.10f)
                atTargetBound = true;
        } else if (_recipe.endPos == WindingEndPos::BOTTOM) {
            if (latState == LatState::WINDING_BWD) pathUnits = 1.0f - progress;
            else if (latState == LatState::WINDING_FWD) pathUnits = 2.0f - progress;
            else if (latState == LatState::HOMED
                     && fabsf(_lateral.getCurrentPositionMm() - _windingStartMm()) <= 0.10f)
                atTargetBound = true;
        }

        if (!atTargetBound && pathUnits > 0.0f && _activePlan.turnsPerPass > 0) {
            const float nominalTurnsToBound = pathUnits * (float)_activePlan.turnsPerPass;
            const float desiredScale = nominalTurnsToBound / moveTurnsRemaining;
            traverseScale = constrain(desiredScale, 0.40f, 1.80f);
        }
    }
    _activePlan.speedScale = traverseScale;

    // 4. Reversal slowdown
    if (_lateral.isReversing())
        hz = max((uint32_t)SPEED_HZ_MIN, (uint32_t)((float)hz * LAT_REVERSAL_SLOWDOWN));

    // 5. Stop-at-bound slowdown
    if (_lateral.hasStopAtNextBoundArmed()) {
        bool applySlowdown = false;
        if (_lateral.getState() == LatState::WINDING_FWD && _lateral.isStopOnNextHighArmed())
            applySlowdown = true;
        else if (_lateral.getState() == LatState::WINDING_BWD && _lateral.isStopOnNextLowArmed())
            applySlowdown = true;

        if (applySlowdown) {
            float progress = _lateral.getTraversalProgress();
            const float zoneStart = 0.80f;
            const float floorFactor = 0.30f;
            if (progress >= zoneStart) {
                float t = constrain((progress - zoneStart) / (1.0f - zoneStart), 0.0f, 1.0f);
                float factor = 1.0f - (1.0f - floorFactor) * t;
                hz = max((uint32_t)SPEED_HZ_MIN, (uint32_t)((float)hz * factor));
            }
        }
    }

    _stepper.setSpeedHz(hz);

    // ── Motor start ──
    if (!_stepper.isRunning()) {
        bool forward = (_direction == Direction::CW) != (bool)WINDING_MOTOR_INVERTED;
        _state = WindingState::WINDING;
        Diag::infof("[WINDING] %s — %u Hz — tpp=%ld scale=%.2f",
                forward ? "CW" : "CCW", hz,
                _activePlan.turnsPerPass, _activePlan.speedScale);
        _stepper.start(forward);
        _lateral.startWinding(hz, _activePlan.turnsPerPass,
                              _windingStartMm(), _windingEndMm(),
                              _activePlan.speedScale);
        return;
    }

    // ── Motor already running ──

    // End position hold: keep stop armed and don't restart traverse
    if (_state == WindingState::WINDING && _endPosArmed) {
        if (_recipe.endPos == WindingEndPos::TOP)    _lateral.armStopAtNextHigh();
        else if (_recipe.endPos == WindingEndPos::BOTTOM) _lateral.armStopAtNextLow();
        if (_lateral.getState() == LatState::HOMED) return;
    }

    // Keep verify-high stop armed while pending
    if (_verifyHighPending) _lateral.armStopAtNextHigh();

    // Update traverse
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

// ═══════════════════════════════════════════════════════════════════════════════
// Auto-stop
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::_checkAutoStop() {
    if (_state != WindingState::WINDING || _freerun || !_stepper.isRunning()) return;

    // End position hook — arm stop to land on desired bound
    if (_recipe.endPos != WindingEndPos::NONE) {
        const long remaining = (long)_targetTurns - _stepper.getTurns();
        if (remaining <= (long)_recipe.endPosTurns && !_endPosArmed) {
            _endPosArmed = true;
            if (_recipe.endPos == WindingEndPos::TOP)    _lateral.armStopAtNextHigh();
            else if (_recipe.endPos == WindingEndPos::BOTTOM) _lateral.armStopAtNextLow();
            Diag::infof("[EndPos] Armed %s stop — %ld turns to go, hold=%ld",
                    _recipe.endPos == WindingEndPos::TOP ? "HIGH" : "LOW",
                    remaining, (long)_recipe.endPosTurns);
        }
    }

    // burst mode removed

    // Auto-stop at target
    if (_stepper.getTurns() >= (long)_targetTurns) {
        _toTargetReached();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Deferred driver disable
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::_applyDeferredDisable() {
    if (!_pendingDisable) return;
    if (_stepper.isRunning() || _lateral.isBusy()) return;
    _pendingDisable = false;
    _stepper.disableDriver();
    Diag::info("[Winder] Driver disabled (idle)");
}

// ═══════════════════════════════════════════════════════════════════════════════
// Status / telemetry
// ═══════════════════════════════════════════════════════════════════════════════

WinderStatus WinderApp::getStatus() const {
    const bool sessionActive = (_state != WindingState::IDLE);
    const bool motorEnabled  = (_state == WindingState::WINDING);

    // (debug trim log déplacé dans _handleGeometryCommand)
    return {
        _stepper.getRPM(),
        _stepper.getSpeedHz(),
        _stepper.getTurns(),
        (long)_targetTurns,
        (uint16_t)((_maxSpeedHz * 60UL + (STEPS_PER_REV / 2)) / STEPS_PER_REV),
        _stepper.isRunning(),
        motorEnabled,
        sessionActive,
        _lateral.isPositionedForStart(),
        (_verifyLowPending || _positioningToLow),   // verifyLow
        _verifyHighPending,                          // verifyHigh
        (_state == WindingState::RODAGE),
        _rodagePassDone,
        _rodagePasses,
        _rodageDistMm,
        (bool)_freerun,
        (_direction == Direction::CW),
        (float)_geom.turnsPerPass(),
        (float)_geom.turnsPerPassCalc(),
        (float)_geom.turnsPerPassOffset,
        _geom.scatterFactor,
        (int)_lateral.getPassCount(),
        _activePlan.turnsPerPass,
        _activePlan.speedScale,
        _lateral.getTraversalProgress(),
        _lateral.getCurrentPositionMm(),
        _windingStartMm(), _windingEndMm(),
        _geom.windingStartTrim_mm, _geom.windingEndTrim_mm,
        _geom.effectiveWidth(),
        _geom.totalWidth_mm, _geom.flangeBottom_mm, _geom.flangeTop_mm,
        _geom.margin_mm, _geom.wireDiameter_mm,
        _lateral.getHomeOffset(),
        WindingPatternPlanner::styleKey(_recipe.style),
        _recipe.seed,
        _recipe.layerJitterPct,
        _recipe.layerSpeedPct,
        _recipe.humanTraversePct,
        _recipe.humanSpeedPct,
        _recipe.firstPassTraverseFactor,
        (int)_recipe.endPos,
        _recipe.endPosTurns,
        windingStateName(_state),
    };
}

void WinderApp::recipeJson(char* buf, size_t len) const {
    if (!buf || len == 0) return;
    _recipeStore.toJson(_captureRecipe(), buf, len);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Geometry change — live carriage repositioning
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::_refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged) {
    if (!_lateral.isHomed() || _lateral.isBusy()) return;

    if (_stepper.isRunning()) {
        // While winding, the geometry update is applied to the target window,
        // and lateral traversal will use the new window bounds on the next tick.
        Diag::info("[WINDOW_SHIFT] Running — geometry updated, reposition deferred");
        return;
    }

    switch (_state) {
    case WindingState::PAUSED:
        // Only reposition if carriage is already near the changed bound.
        if (startBoundChanged && fabsf(_lateral.getCurrentPositionMm() - _windingStartMm()) <= 1.0f) {
            _lateral.prepareStartPosition(_windingStartMm());
        } else if (endBoundChanged && fabsf(_lateral.getCurrentPositionMm() - _windingEndMm()) <= 1.0f) {
            _lateral.prepareStartPosition(_windingEndMm());
        }
        break;

    case WindingState::TARGET_REACHED:
    case WindingState::IDLE:
        // When idle/target reached, change will take effect at next start.
        break;

    default:
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Immediate commands
// ═══════════════════════════════════════════════════════════════════════════════

bool WinderApp::_handleImmediateCommand(const char* cmd, const char* value) {
    if (strcmp(cmd, "stop") == 0) {
        _toIdle();
        return true;
    }

    if (strcmp(cmd, "reset") == 0) {
        _toIdle();
        Diag::info("[IDLE] Turn counter reset");
        return true;
    }

    // ── Start: position to low bound with verify flags, or resume ────────────
    if (strcmp(cmd, "start") == 0) {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            if (!_lateral.isHomed() || _lateral.isBusy()) {
                Diag::error("[Start] Lateral axis not ready");
                return true;
            }

            // start: Fresh session — reset and begin verify sequence

            // Fresh session — reset and begin verify sequence
            _stepper.resetTurns();
            _planner.reset();
            _verifyLowPending  = true;
            _verifyHighPending = true;
            _positioningToLow  = true;
            _state             = WindingState::PAUSED;
            _pendingDisable    = false;
            _lateral.prepareStartPosition(_windingStartMm());
            Diag::infof("[START] Positioning to low bound %.2f mm", _windingStartMm());
            return true;
        }
        if (_state == WindingState::PAUSED) {
            // Resume via explicit Start command (from SessionController) and allow current pot-based speed.
            _toWinding();
            return true;
        }
        return true;
    }

    if (strcmp(cmd, "pause") == 0) {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            Diag::info("[Pause] Ignored — no active session");
        } else {
            _pauseRequested = true;
            Diag::info("[Pause] Pause requested");
        }
        return true;
    }

    if (strcmp(cmd, "resume") == 0) {
        if (_state == WindingState::PAUSED) {
            _toWinding();
            Diag::info("[Resume] Armed — transitioning to winding");
        } else {
            Diag::infof("[Resume] Ignored in state %s", windingStateName(_state));
        }
        return true;
    }

    // Allow changing max RPM even while a session is active.
    if (strcmp(cmd, "max_rpm") == 0 || strcmp(cmd, "max-rpm") == 0) {
        int rpm = constrain((int)strtol(value, nullptr, 10), 10, 1500);
        _maxSpeedHz = (uint32_t)rpm * (uint32_t)STEPS_PER_REV / 60UL;
        Diag::infof("[MaxRPM] (immediate) Set %d RPM -> %u Hz", rpm, (unsigned)_maxSpeedHz);
        return true;
    }

    if (strcmp(cmd, "stop_next_high") == 0) {
        _lateral.armStopAtNextHigh();
        Diag::info("[Mode] Stop armed on next high bound");
        return true;
    }

    if (strcmp(cmd, "stop_next_low") == 0) {
        _lateral.armStopAtNextLow();
        Diag::info("[Mode] Stop armed on next low bound");
        return true;
    }

    // ── Position finale de bobinage ──────────────────────────────────────────
    if (strcmp(cmd, "end_pos") == 0) {
        _recipe.endPos = windingEndPosFromString(value);
        _saveRecipe();
        Diag::infof("[END_POS] Position finale: %s", windingEndPosKey(_recipe.endPos));
        return true;
    }
    if (strcmp(cmd, "end_pos_turns") == 0) {
        _recipe.endPosTurns = (int)constrain((int)strtol(value, nullptr, 10), 1, 20);
        _saveRecipe();
        Diag::infof("[END_POS] Tours finaux: %d", _recipe.endPosTurns);
        return true;
    }

    // (no burst commands) -- simplified command set

    // ── Rodage axe lateral ───────────────────────────────────────────────────
    if (strcmp(cmd, "rodage_dist") == 0) {
        _rodageDistMm = constrain(atof(value), 5.0f, (float)LAT_TRAVERSE_MM);
        Diag::infof("[RODAGE] Distance: %.1f mm", _rodageDistMm);
        return true;
    }
    if (strcmp(cmd, "rodage_passes") == 0) {
        _rodagePasses = (int)constrain((int)strtol(value, nullptr, 10), 1, 200);
        Diag::infof("[RODAGE] Passes: %d", _rodagePasses);
        return true;
    }
    // burst commands removed

    if (strcmp(cmd, "rodage") == 0) {
        if (_state == WindingState::IDLE) {
            _toRodage();
        } else {
            Diag::infof("[RODAGE] Ignore — etat actuel: %s", windingStateName(_state));
        }
        return true;
    }
    if (strcmp(cmd, "rodage_stop") == 0) {
        if (_state == WindingState::RODAGE) {
            _toIdle();
            Diag::info("[RODAGE] Arret demande par l'operateur");
        }
        return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Geometry commands
// ═══════════════════════════════════════════════════════════════════════════════

static bool startsWith(const char* str, const char* prefix) {
    return strncmp(str, prefix, strlen(prefix)) == 0;
}

bool WinderApp::_handleGeometryCommand(const char* cmd, const char* value) {
    if (strcmp(cmd, "geom_start_trim") == 0) {
        _geom.windingStartTrim_mm = constrain(atof(value), -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(true, false);
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "geom_end_trim") == 0) {
        _geom.windingEndTrim_mm = constrain((float)atof(value), -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(false, true);
        _saveRecipe();
        return true;
    }

    auto applyTrimShift = [&](float dStart, float dEnd, const char* label) {
        float prevStart = _geom.windingStartTrim_mm;
        float prevEnd   = _geom.windingEndTrim_mm;
        _geom.windingStartTrim_mm = constrain(prevStart + dStart, -5.0f, 5.0f);
        _geom.windingEndTrim_mm   = constrain(prevEnd   + dEnd,   -5.0f, 5.0f);

        // Harmony with window_shift: update geometry values and defer carriage readjust
        // to _refreshCarriageForGeometryChange.
        _refreshCarriageForGeometryChange(dStart != 0.0f, dEnd != 0.0f);
        _saveRecipe();

        Diag::infof("[%s] startTrim old=%.3f -> new=%.3f endTrim old=%.3f -> new=%.3f",
            label,
            prevStart, _geom.windingStartTrim_mm,
            prevEnd,   _geom.windingEndTrim_mm);
    };

    auto isWindowShift = [&](const char* key) {
        return strcmp(key, "window_shift") == 0
            || strcmp(key, "windows_shift") == 0
            || strncmp(key, "window_shift", strlen("window_shift")) == 0;
    };

    if (isWindowShift(cmd)) {
        float delta = constrain(atof(value), -5.0f, 5.0f);
        if (strstr(cmd, "nudge") != nullptr) delta = constrain(delta, -1.0f, 1.0f);

        applyTrimShift(delta, delta, "WINDOW_SHIFT");
        return true;
    }

    if (strcmp(cmd, "geom_start_trim_nudge") == 0) {
        float delta = constrain(atof(value), -1.0f, 1.0f);
        Diag::infof("[DEBUG] geom_start_trim_nudge: value=%.3f, prev=%.3f", delta, _geom.windingStartTrim_mm);
        applyTrimShift(delta, 0.0f, "STRT_TRIM_NUDGE");
        return true;
    }

    if (strcmp(cmd, "geom_end_trim_nudge") == 0) {
        float delta = constrain(atof(value), -1.0f, 1.0f);
        Diag::infof("[DEBUG] geom_end_trim_nudge: value=%.3f, prev=%.3f", delta, _geom.windingEndTrim_mm);
        applyTrimShift(0.0f, delta, "END_TRIM_NUDGE");
        return true;
    }

    if (strcmp(cmd, "geom_preset") == 0) {
        uint8_t idx = (uint8_t)strtol(value, nullptr, 10);
        _geom.applyPreset(idx);
        _refreshCarriageForGeometryChange(true, true);
        Diag::infof("Bobbin preset: %s — %ld turns/pass",
            BOBBIN_PRESETS[idx].name, _geom.turnsPerPass());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "geom_total") == 0)  { _geom.totalWidth_mm   = atof(value); _refreshCarriageForGeometryChange(true, true); _saveRecipe(); return true; }
    if (strcmp(cmd, "geom_bottom") == 0) { _geom.flangeBottom_mm = atof(value); _refreshCarriageForGeometryChange(true, false); _saveRecipe(); return true; }
    if (strcmp(cmd, "geom_top") == 0)    { _geom.flangeTop_mm    = atof(value); _saveRecipe(); return true; }
    if (strcmp(cmd, "geom_margin") == 0) { _geom.margin_mm       = atof(value); _refreshCarriageForGeometryChange(true, true); _saveRecipe(); return true; }

    if (strcmp(cmd, "geom_wire") == 0) {
        _geom.wireDiameter_mm = atof(value);
        Diag::infof("Wire: %.4f mm — %ld turns/pass (calc: %ld)",
            _geom.wireDiameter_mm, _geom.turnsPerPass(), _geom.turnsPerPassCalc());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "geom_tpp_ofs") == 0) {
        _geom.turnsPerPassOffset = (int)strtol(value, nullptr, 10);
        Diag::infof("Turns/pass offset: %+ld (calc %ld -> effective %ld)",
            _geom.turnsPerPassOffset, _geom.turnsPerPassCalc(), _geom.turnsPerPass());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "geom_scatter") == 0) {
        float f = atof(value);
        if (f >= 0.5f && f <= 5.0f) {
            _geom.scatterFactor = f;
            Diag::infof("Scatter factor: %.2f -> %ld turns/pass",
                _geom.scatterFactor, _geom.turnsPerPass());
            _saveRecipe();
        }
        return true;
    }

    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Pattern commands
// ═══════════════════════════════════════════════════════════════════════════════

bool WinderApp::_handlePatternCommand(const char* cmd, const char* value) {
    if (strcmp(cmd, "winding_style") == 0) {
        _recipe.style = WindingPatternPlanner::styleFromString(value);
        _planner.setRecipe(_captureRecipe());
        Diag::infof("Winding style: %s", WindingPatternPlanner::styleName(_recipe.style));
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_seed") == 0) {
        long parsed = strtol(value, nullptr, 10);
        uint32_t seed = (uint32_t)((parsed > 0) ? parsed : 1);
        _recipe.seed = seed;
        _planner.setRecipe(_captureRecipe());
        Diag::infof("Winding seed: %lu", (unsigned long)_recipe.seed);
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_layer_jitter") == 0) {
        _recipe.layerJitterPct = constrain(atof(value), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_layer_speed") == 0) {
        _recipe.layerSpeedPct = constrain(atof(value), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_human_traverse") == 0) {
        _recipe.humanTraversePct = constrain(atof(value), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_human_speed") == 0) {
        _recipe.humanSpeedPct = constrain(atof(value), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_first_pass_traverse") == 0) {
        _recipe.firstPassTraverseFactor = constrain(atof(value), 0.40f, 1.80f);
        _planner.setRecipe(_captureRecipe());
        Diag::infof("First pass traverse factor: %.2f", _recipe.firstPassTraverseFactor);
        _saveRecipe();
        return true;
    }

    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Encoder
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::handleEncoderDelta(int32_t delta) {
    if (delta == 0) return;

    if (_state == WindingState::PAUSED) {
        // Near a bound: adjust trim with encoder jog
        float currentPos = _lateral.getCurrentPositionMm();
        if (fabsf(currentPos - _windingEndMm()) < 0.5f) {
            float step = delta * ENC_STEP_MM;
            _lateral.jog(step);
            float newPos = _lateral.getTargetPositionMm();
            _geom.windingEndTrim_mm = constrain(
                newPos - (_geom.totalWidth_mm - _geom.flangeTop_mm - _geom.margin_mm), -5.0f, 5.0f);
            _saveRecipe();
            Diag::infof("[Encoder] Butee haute ajustee en PAUSE: %.2f mm", newPos);
        } else if (fabsf(currentPos - _windingStartMm()) < 0.5f) {
            float step = delta * ENC_STEP_MM;
            _lateral.jog(step);
            float newPos = _lateral.getTargetPositionMm();
            _geom.windingStartTrim_mm = constrain(
                newPos - (_geom.flangeBottom_mm + _geom.margin_mm), -5.0f, 5.0f);
            _saveRecipe();
            Diag::infof("[Encoder] Butee basse ajustee en PAUSE: %.2f mm", newPos);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Command dispatch
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::handleCommand(const char* cmd, const char* value) {
    Diag::infof("[APP-CMD] cmd='%s' val='%s'", cmd, value);

    if (_handleImmediateCommand(cmd, value)) return;
    if (_handleGeometryCommand(cmd, value))  return;

    // Turn target: modifiable at any time.
    if (strcmp(cmd, "target") == 0) {
        long t = strtol(value, nullptr, 10);
        if (t > 0) {
            _targetTurns = t;
            if (_state == WindingState::TARGET_REACHED && t > _stepper.getTurns()) {
                _toPaused();
                Diag::info("[PAUSED] Target raised — resume possible");
            }
            Diag::infof("Target: %ld turns", t);
            _saveRecipe();
        }
        return;
    }

    // Parameters locked during active session
    if (_parametersLocked()) {
        Diag::infof("[Lock] Ignored during session (%s): %s",
            windingStateName(_state), cmd);
        return;
    }

    // Update maximum spindle speed (RPM -> stepper Hz)
    // (handled as immediate command) -- nothing to do here

    if (strcmp(cmd, "freerun") == 0) {
        _freerun = (strcmp(value, "true") == 0);
        Diag::infof("Mode: %s", _freerun ? "FreeRun" : "Target");
        _saveRecipe();
        return;
    }

    if (strcmp(cmd, "direction") == 0) {
        Direction newDir = (strcmp(value, "cw") == 0) ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            Diag::infof("Direction: %s", value);
            _saveRecipe();
        }
        return;
    }

    if (_handlePatternCommand(cmd, value)) return;

    if (strcmp(cmd, "recipe_import") == 0) {
        WindingRecipe imported;
        if (_recipeStore.fromJson(value, imported)) {
            _applyRecipe(imported, true);
            Diag::info("[Recipe] Recipe imported");
        } else {
            Diag::error("[Recipe] ERROR — invalid JSON");
        }
        return;
    }

    if (strcmp(cmd, "lat_offset") == 0) {
        float mm = atof(value);
        if (mm >= 0.0f) {
            _lateral.setHomeOffset(mm);
            _toIdle();
            _lateral.rehome();
            Diag::infof("[Lateral] Offset %.2f mm — rehoming started", mm);
            _saveRecipe();
        }
        return;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Traverse plan + recipe helpers
// ═══════════════════════════════════════════════════════════════════════════════

TraversePlan WinderApp::_buildTraversePlan(uint32_t windingHz) const {
    (void)windingHz;
    return _planner.getPlan(_stepper.getTurns(), _lateral.getTraversalProgress());
}

void WinderApp::_applyRecipe(const WindingRecipe& recipe, bool persist) {
    _recipe      = recipe;
    _geom        = recipe.geometry;
    _targetTurns = recipe.targetTurns;
    _freerun     = recipe.freerun;
    _direction   = recipe.directionCW ? Direction::CW : Direction::CCW;
    _state       = WindingState::IDLE;
    _pendingDisable = false;
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
