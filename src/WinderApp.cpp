#include "WinderApp.h"
#include <Arduino.h>
#include "Diag.h"

// ═══════════════════════════════════════════════════════════════════════════════
// Initialization
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::begin() {
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
    _handlePotCycle(_inputHz);
    _checkAutoStop();
    _applyDeferredDisable();
}

// ═══════════════════════════════════════════════════════════════════════════════
// State transitions — every _toXxx() is self-contained
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::_toIdle() {
    const WindingState prev = _state;
    _state              = WindingState::IDLE;
    _canStart           = false;
    _pendingDisable     = true;
    _startButtonMax     = false;
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
    _canStart           = false;
    _startButtonMax     = false;
    _pendingDisable     = false;
    _positioningToLow   = false;
    _endPosArmed        = false;
    Diag::infof("[WINDING] %.2f -> %.2f mm",
            _windingStartMm(), _windingEndMm());
}

void WinderApp::_toPaused() {
    _state              = WindingState::PAUSED;
    _canStart           = false;
    _startButtonMax     = false;
    _pendingDisable     = true;
    _stepper.stop();
    _lateral.stopWinding();
    Diag::info("[PAUSED] Resume when pot goes up or press Start");
}

void WinderApp::_toTargetReached() {
    _state              = WindingState::TARGET_REACHED;
    _canStart           = false;
    _startButtonMax     = false;
    _pendingDisable     = true;
    _stepper.stop();
    _lateral.stopWinding();
    _endPosArmed        = false;
    Diag::infof("[TARGET_REACHED] %ld turns — raise target or press Stop",
            _stepper.getTurns());
}

void WinderApp::_toManual() {
    _state              = WindingState::MANUAL;
    _canStart           = false;
    _pendingDisable     = true;
    _pendingManual      = false;
    _startButtonMax     = false;
    _verifyLowPending   = false;
    _verifyHighPending  = false;
    _positioningToLow   = false;
    _manualFirstPass    = true;
    _stepper.stop();
    _lateral.stopWinding();
    _captureActive      = false;
    _captureLastPosMm   = -999.0f;
    Diag::infof("[MANUAL] Window [%.2f -> %.2f mm], step: %.2f mm",
            _windingStartMm(), _windingEndMm(), _manualJogStepMm);
}

void WinderApp::_toRodage() {
    _state              = WindingState::RODAGE;
    _canStart           = false;
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
        _canStart           = true;
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

    // Rodage
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

void WinderApp::_handlePotCycle(uint32_t hz) {
    uint32_t driveHz = hz;
    if (_startButtonMax) {
        if (hz > 0) {
            _startButtonMax = false;
            driveHz = hz;
        } else {
            driveHz = _maxSpeedHz;
        }
    }
    if (driveHz == 0) _canStart = true;

    switch (_state) {
    case WindingState::IDLE:
        if (_lateral.isHomed() && !_lateral.isBusy() && !_lateral.isAtZero())
            _lateral.parkAtZero();
        break;

    case WindingState::WINDING:
        if (driveHz > 0)
            _runWindingAtHz(driveHz);
        else
            _toPaused();
        break;

    case WindingState::PAUSED:
        if (driveHz > 0 && _canStart && _lateral.isHomed() && !_lateral.isBusy()) {
            if (_verifyHighPending) {
                _lateral.clearOneShotStops();
                _lateral.armStopAtNextHigh();
            }
            _runWindingAtHz(driveHz);
        }
        break;

    case WindingState::MANUAL:
        if (driveHz == 0) {
            _canStart = true;
            if (_stepper.isRunning()) {
                _pendingDisable = true;
                _stepper.stop();
            }
        } else if (_canStart) {
            bool forward = (_direction == Direction::CW) != (bool)WINDING_MOTOR_INVERTED;
            _stepper.setSpeedHz(driveHz);
            if (!_stepper.isRunning()) _stepper.start(forward);
        }
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
        (_state == WindingState::MANUAL),
        (_state == WindingState::RODAGE),
        _rodagePassDone,
        _rodagePasses,
        _rodageDistMm,
        (bool)_freerun,
        (_direction == Direction::CW),
        false,   // autoMode — reserved
        _geom.turnsPerPass(),
        _geom.turnsPerPassCalc(),
        _geom.turnsPerPassOffset,
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
        WindingPatternPlanner::styleKey(_recipe.style).c_str(),
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

String WinderApp::recipeJson() const {
    return _recipeStore.toJson(_captureRecipe());
}

// ═══════════════════════════════════════════════════════════════════════════════
// Geometry change — live carriage repositioning
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::_refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged) {
    if (_stepper.isRunning()) return;
    if (!_lateral.isHomed() || _lateral.isBusy()) return;

    switch (_state) {
    case WindingState::PAUSED:
        // Only reposition if carriage is already near the changed bound.
        if (startBoundChanged && fabsf(_lateral.getCurrentPositionMm() - _windingStartMm()) <= 1.0f) {
            _lateral.prepareStartPosition(_windingStartMm());
        } else if (endBoundChanged && fabsf(_lateral.getCurrentPositionMm() - _windingEndMm()) <= 1.0f) {
            _lateral.prepareStartPosition(_windingEndMm());
        }
        break;
    default:
        break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Immediate commands
// ═══════════════════════════════════════════════════════════════════════════════

bool WinderApp::_handleImmediateCommand(const String& cmd, const String& value) {
    if (cmd == "stop") {
        _toIdle();
        return true;
    }

    if (cmd == "reset") {
        _toIdle();
        Diag::info("[IDLE] Turn counter reset");
        return true;
    }

    // ── Start: position to low bound with verify flags, or resume ────────────
    if (cmd == "start") {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            if (!_lateral.isHomed() || _lateral.isBusy()) {
                Diag::error("[Start] Lateral axis not ready");
                return true;
            }
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
            // Resume — equivalent to pot at max
            _startButtonMax = true;
            _canStart       = true;
            return true;
        }
        return true;
    }

    if (cmd == "pause") {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            Diag::info("[Pause] Ignored — no active session");
        } else {
            // Handle pause inside tick() to avoid cross-task race with control loop.
            _startButtonMax = false;
            _pauseRequested = true;
            Diag::info("[Pause] Pause requested");
        }
        return true;
    }

    if (cmd == "resume") {
        if (_state == WindingState::PAUSED) {
            _startButtonMax = true;
            _canStart       = true;
            Diag::info("[Resume] Armed — starting at max speed");
        } else {
            Diag::infof("[Resume] Ignored in state %s", windingStateName(_state));
        }
        return true;
    }

    // Allow changing max RPM even while a session is active.
    if (cmd == "max_rpm" || cmd == "max-rpm") {
        int rpm = constrain(value.toInt(), 10, 1500);
        _maxSpeedHz = (uint32_t)rpm * (uint32_t)STEPS_PER_REV / 60UL;
        Diag::infof("[MaxRPM] (immediate) Set %d RPM -> %u Hz", rpm, (unsigned)_maxSpeedHz);
        return true;
    }

    if (cmd == "stop_next_high") {
        _lateral.armStopAtNextHigh();
        Diag::info("[Mode] Stop armed on next high bound");
        return true;
    }

    if (cmd == "stop_next_low") {
        _lateral.armStopAtNextLow();
        Diag::info("[Mode] Stop armed on next low bound");
        return true;
    }

    // ── Mode manuel ──────────────────────────────────────────────────────────
    if (cmd == "manual") {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            _pendingManual = true;
            _verifyLowPending  = true;
            _verifyHighPending = false; // No high verify needed for manual
            _positioningToLow  = true;
            _state             = WindingState::PAUSED;
            _pendingDisable    = false;
            if (_state == WindingState::TARGET_REACHED) _stepper.resetTurns();
            _lateral.prepareStartPosition(_windingStartMm());
            Diag::info("[MANUAL] Positioning to low bound before manual mode...");
        } else {
            _toManual();
        }
        return true;
    }

    if (cmd == "manual_stop") {
        if (_state == WindingState::MANUAL) {
            _toIdle();
            Diag::info("[MANUAL] Mode manuel termine -> IDLE");
        }
        return true;
    }

    if (cmd == "manual_step") {
        float s = value.toFloat();
        if (s > 0.0f && s <= 5.0f) {
            _manualJogStepMm = s;
            Diag::infof("[MANUAL] Pas jog: %.3f mm/cran", _manualJogStepMm);
        }
        return true;
    }

    if (cmd == "manual_capture_start") {
        if (_state == WindingState::MANUAL) {
            _captureActive    = true;
            _captureLastPosMm = -999.0f;
            Diag::info("[MANUAL] Capture demarree");
        }
        return true;
    }

    if (cmd == "manual_capture_stop") {
        _captureActive = false;
        Diag::info("[MANUAL] Capture arretee");
        return true;
    }

    // ── Position finale de bobinage ──────────────────────────────────────────
    if (cmd == "end_pos") {
        _recipe.endPos = windingEndPosFromString(value);
        _saveRecipe();
        Diag::infof("[END_POS] Position finale: %s", windingEndPosKey(_recipe.endPos));
        return true;
    }
    if (cmd == "end_pos_turns") {
        _recipe.endPosTurns = (int)constrain(value.toInt(), 1L, 20L);
        _saveRecipe();
        Diag::infof("[END_POS] Tours finaux: %d", _recipe.endPosTurns);
        return true;
    }

    // ── Rodage axe lateral ───────────────────────────────────────────────────
    if (cmd == "rodage_dist") {
        _rodageDistMm = constrain(value.toFloat(), 5.0f, (float)LAT_TRAVERSE_MM);
        Diag::infof("[RODAGE] Distance: %.1f mm", _rodageDistMm);
        return true;
    }
    if (cmd == "rodage_passes") {
        _rodagePasses = (int)constrain(value.toInt(), 1L, 200L);
        Diag::infof("[RODAGE] Passes: %d", _rodagePasses);
        return true;
    }
    if (cmd == "rodage") {
        if (_state == WindingState::IDLE) {
            _toRodage();
        } else {
            Diag::infof("[RODAGE] Ignore — etat actuel: %s", windingStateName(_state));
        }
        return true;
    }
    if (cmd == "rodage_stop") {
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

bool WinderApp::_handleGeometryCommand(const String& cmd, const String& value) {
    if (cmd == "geom_start_trim") {
        _geom.windingStartTrim_mm = constrain(value.toFloat(), -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(true, false);
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_end_trim") {
        _geom.windingEndTrim_mm = constrain(value.toFloat(), -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(false, true);
        _saveRecipe();
        return true;
    }

    if (cmd == "window_shift") {
        float delta = constrain(value.toFloat(), -5.0f, 5.0f);
        _geom.windingStartTrim_mm = constrain(_geom.windingStartTrim_mm + delta, -5.0f, 5.0f);
        _geom.windingEndTrim_mm   = constrain(_geom.windingEndTrim_mm   + delta, -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(true, true);
        _saveRecipe();
        Diag::infof("[WINDOW_SHIFT] %.2f mm -> fenetre [%.2f -> %.2f mm]",
            delta, _windingStartMm(), _windingEndMm());
        return true;
    }

    if (cmd == "window_shift_nudge") {
        float delta = constrain(value.toFloat(), -1.0f, 1.0f);
        _geom.windingStartTrim_mm = constrain(_geom.windingStartTrim_mm + delta, -5.0f, 5.0f);
        _geom.windingEndTrim_mm   = constrain(_geom.windingEndTrim_mm   + delta, -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(true, true);
        _saveRecipe();
        Diag::infof("[WINDOW_SHIFT] Nudge %.2f mm -> fenetre [%.2f -> %.2f mm]",
            delta, _windingStartMm(), _windingEndMm());
        return true;
    }

    if (cmd == "geom_start_trim_nudge") {
        if (_state != WindingState::PAUSED) {
            Diag::info("[geom_start_trim_nudge] Ignored — only available while paused near bound");
            return true;
        }
        if (_lateral.isBusy() || fabsf(_lateral.getCurrentPositionMm() - _windingStartMm()) > 0.50f) {
            Diag::info("[geom_start_trim_nudge] Ignored — carriage is not settled on the low bound");
            return true;
        }
        float delta = constrain(value.toFloat(), -1.0f, 1.0f);
        _lateral.jog(delta);
        float newPos = _lateral.getTargetPositionMm();
        _geom.windingStartTrim_mm = constrain(
            newPos - (_geom.flangeBottom_mm + _geom.margin_mm), -5.0f, 5.0f);
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_end_trim_nudge") {
        if (_state != WindingState::PAUSED) {
            Diag::info("[geom_end_trim_nudge] Ignored — only available while paused near bound");
            return true;
        }
        if (_lateral.isBusy() || fabsf(_lateral.getCurrentPositionMm() - _windingEndMm()) > 0.50f) {
            Diag::info("[geom_end_trim_nudge] Ignored — carriage is not settled on the high bound");
            return true;
        }
        float delta = constrain(value.toFloat(), -1.0f, 1.0f);
        _lateral.jog(delta);
        float newPos = _lateral.getTargetPositionMm();
        _geom.windingEndTrim_mm = constrain(
            newPos - (_geom.totalWidth_mm - _geom.flangeTop_mm - _geom.margin_mm), -5.0f, 5.0f);
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_preset") {
        uint8_t idx = (uint8_t)value.toInt();
        _geom.applyPreset(idx);
        _refreshCarriageForGeometryChange(true, true);
        Diag::infof("Bobbin preset: %s — %ld turns/pass",
            BOBBIN_PRESETS[idx].name, _geom.turnsPerPass());
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_total")  { _geom.totalWidth_mm   = value.toFloat(); _refreshCarriageForGeometryChange(true, true); _saveRecipe(); return true; }
    if (cmd == "geom_bottom") { _geom.flangeBottom_mm = value.toFloat(); _refreshCarriageForGeometryChange(true, false); _saveRecipe(); return true; }
    if (cmd == "geom_top")    { _geom.flangeTop_mm    = value.toFloat(); _saveRecipe(); return true; }
    if (cmd == "geom_margin") { _geom.margin_mm       = value.toFloat(); _refreshCarriageForGeometryChange(true, true); _saveRecipe(); return true; }

    if (cmd == "geom_wire") {
        _geom.wireDiameter_mm = value.toFloat();
        Diag::infof("Wire: %.4f mm — %ld turns/pass (calc: %ld)",
            _geom.wireDiameter_mm, _geom.turnsPerPass(), _geom.turnsPerPassCalc());
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_tpp_ofs") {
        _geom.turnsPerPassOffset = value.toInt();
        Diag::infof("Turns/pass offset: %+ld (calc %ld -> effective %ld)",
            _geom.turnsPerPassOffset, _geom.turnsPerPassCalc(), _geom.turnsPerPass());
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_scatter") {
        float f = value.toFloat();
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

bool WinderApp::_handlePatternCommand(const String& cmd, const String& value) {
    if (cmd == "winding_style") {
        _recipe.style = WindingPatternPlanner::styleFromString(value);
        _planner.setRecipe(_captureRecipe());
        Diag::infof("Winding style: %s", WindingPatternPlanner::styleName(_recipe.style));
        _saveRecipe();
        return true;
    }

    if (cmd == "winding_seed") {
        uint32_t seed = (uint32_t)((value.toInt() > 0) ? value.toInt() : 1);
        _recipe.seed = seed;
        _planner.setRecipe(_captureRecipe());
        Diag::infof("Winding seed: %lu", (unsigned long)_recipe.seed);
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

    if (cmd == "winding_first_pass_traverse") {
        _recipe.firstPassTraverseFactor = constrain(value.toFloat(), 0.40f, 1.80f);
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
    } else if (_state == WindingState::MANUAL) {
        float stepMm = _manualFirstPass
            ? (_manualJogStepMm * (float)MANUAL_FAST_STEP_MULT)
            : _manualJogStepMm;

        float baseMm    = _lateral.getTargetPositionMm();
        float newTarget = constrain(baseMm + delta * stepMm,
                                    _windingStartMm(), _windingEndMm());
        float actualDelta = newTarget - baseMm;
        if (fabsf(actualDelta) > 0.001f) {
            _lateral.jog(actualDelta);
            if (_manualFirstPass &&
                (newTarget >= _windingEndMm()   - 0.2f ||
                 newTarget <= _windingStartMm() + 0.2f)) {
                _manualFirstPass = false;
                Diag::infof("[MANUAL] Premiere bute atteinte — passage aux pas fins (%.2f mm)",
                    _manualJogStepMm);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Command dispatch
// ═══════════════════════════════════════════════════════════════════════════════

void WinderApp::handleCommand(const String& cmd, const String& value) {
    if (_handleImmediateCommand(cmd, value)) return;
    if (_handleGeometryCommand(cmd, value))  return;

    // Turn target: modifiable at any time.
    if (cmd == "target") {
        long t = value.toInt();
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
            windingStateName(_state), cmd.c_str());
        return;
    }

    // Update maximum spindle speed (RPM -> stepper Hz)
    // (handled as immediate command) -- nothing to do here

    if (cmd == "freerun") {
        _freerun = (value == "true");
        Diag::infof("Mode: %s", _freerun ? "FreeRun" : "Target");
        _saveRecipe();
        return;
    }

    if (cmd == "direction") {
        Direction newDir = (value == "cw") ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            Diag::infof("Direction: %s", value.c_str());
            _saveRecipe();
        }
        return;
    }

    if (_handlePatternCommand(cmd, value)) return;

    if (cmd == "recipe_import") {
        WindingRecipe imported;
        if (_recipeStore.fromJson(value, imported)) {
            _applyRecipe(imported, true);
            Diag::info("[Recipe] Recipe imported");
        } else {
            Diag::error("[Recipe] ERROR — invalid JSON");
        }
        return;
    }

    if (cmd == "lat_offset") {
        float mm = value.toFloat();
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
    _canStart    = false;
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
