#include "WinderApp.h"

#include <Arduino.h>

#include "Diag.h"

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
    Diag::infof("[Winder] Ready - %.1fmm usable, %ld tpp, profile=%s",
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

void WinderApp::_toIdle() {
    const WindingState prev = _state;
    _state = WindingState::IDLE;
    _pendingDisable = true;
    _verifyLowPending = false;
    _verifyHighPending = false;
    _positioningToLow = false;
    _endPosArmed = false;
    _stepper.stop();
    _lateral.stopWinding();
    _lateral.parkAtZero();
    _stepper.resetTurns();
    _planner.reset();
    _activePlan = _planner.getPlan(0, 0.0f);
    Diag::infof("[IDLE] %s -> IDLE - carriage to home, counter reset", windingStateName(prev));
}

void WinderApp::_toWinding() {
    _state = WindingState::WINDING;
    _pendingDisable = false;
    _positioningToLow = false;
    _endPosArmed = false;
    Diag::infof("[WINDING] %.2f -> %.2f mm", _windingStartMm(), _windingEndMm());
}

void WinderApp::_toPaused() {
    _state = WindingState::PAUSED;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    Diag::info("[PAUSED] Resume when pot goes up or press Start");
}

void WinderApp::_toTargetReached() {
    _state = WindingState::TARGET_REACHED;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    _endPosArmed = false;
    Diag::infof("[TARGET_REACHED] %ld turns - raise target or press Stop", _stepper.getTurns());
}

void WinderApp::_toRodage() {
    _state = WindingState::RODAGE;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    _rodagePassDone = 0;
    _rodageFwd = true;
    if (_lateral.isHomed()) {
        _lateral.prepareStartPosition(_rodageDistMm, LAT_RODAGE_SPEED_HZ);
    }
    Diag::infof("[RODAGE] %d passes, dist=%.1f mm", _rodagePasses, _rodageDistMm);
}

void WinderApp::_handleLateralEvents() {
    _lateral.update();

    if (_positioningToLow && _lateral.isHomed() && !_lateral.isBusy()) {
        _positioningToLow = false;
        _verifyLowPending = false;
        _state = WindingState::PAUSED;
        _pendingDisable = true;
        Diag::infof("[VERIFY] Low bound reached at %.2f mm - press Start to wind toward high", _windingStartMm());
        return;
    }

    if (_state == WindingState::WINDING && _lateral.consumePausedAtReversal()) {
        if (_verifyHighPending) {
            _verifyHighPending = false;
            _toPaused();
            Diag::infof("[VERIFY] High bound reached at %.2f mm - press Start to begin winding", _windingEndMm());
        } else {
            _toPaused();
            Diag::info("[WINDING] Bound stop - PAUSED");
        }
        return;
    }

    if (_state == WindingState::RODAGE && _lateral.isHomed()) {
        if (_rodageFwd) {
            _rodageFwd = false;
            _lateral.prepareStartPosition(0.0f, LAT_RODAGE_SPEED_HZ);
        } else {
            _rodagePassDone++;
            Diag::infof("[RODAGE] Pass %d/%d", _rodagePassDone, _rodagePasses);
            if (_rodagePassDone >= _rodagePasses) {
                Diag::infof("[RODAGE] Complete (%d passes) - IDLE", _rodagePassDone);
                _toIdle();
            } else {
                _rodageFwd = true;
                _lateral.prepareStartPosition(_rodageDistMm, LAT_RODAGE_SPEED_HZ);
            }
        }
    }
}

void WinderApp::_processInputHz(uint32_t hz) {
    switch (_state) {
    case WindingState::IDLE:
        if (_lateral.isHomed() && !_lateral.isBusy() && !_lateral.isAtZero()) {
            _lateral.parkAtZero();
        }
        break;

    case WindingState::WINDING:
        if (hz > 0) _runWindingAtHz(hz);
        else _toPaused();
        break;

    case WindingState::PAUSED:
    case WindingState::TARGET_REACHED:
    case WindingState::RODAGE:
        break;
    }
}

bool WinderApp::_readyForSpin() const {
    return _lateral.isHomed() && !_lateral.isBusy();
}

void WinderApp::_runWindingAtHz(uint32_t hz) {
    hz = min(hz, (uint32_t)_maxSpeedHz);

    // ── Approach zone: compute cap now, apply as hard limit AFTER compensation ──
    // Pre-computing avoids having the cap interact with the traverse plan build.
    uint32_t approachCapHz = UINT32_MAX;
    if (!_freerun && _stepper.isRunning()) {
        const long remaining = (long)_targetTurns - _stepper.getTurns();
        if (remaining > 0 && remaining <= APPROACH_TURNS) {
            const float ratio = (float)remaining / APPROACH_TURNS;
            approachCapHz = APPROACH_SPEED_HZ_FLOOR
                          + (uint32_t)(ratio * (float)(SPEED_HZ_MAX - APPROACH_SPEED_HZ_FLOOR));
        }
    }

    _activePlan = _buildTraversePlan(hz);
    float traverseScale = _activePlan.speedScale;

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
                && fabsf(_lateral.getCurrentPositionMm() - _windingEndMm()) <= 0.10f) atTargetBound = true;
        } else if (_recipe.endPos == WindingEndPos::BOTTOM) {
            if (latState == LatState::WINDING_BWD) pathUnits = 1.0f - progress;
            else if (latState == LatState::WINDING_FWD) pathUnits = 2.0f - progress;
            else if (latState == LatState::HOMED
                && fabsf(_lateral.getCurrentPositionMm() - _windingStartMm()) <= 0.10f) atTargetBound = true;
        }

        if (!atTargetBound && pathUnits > 0.0f && _activePlan.turnsPerPass > 0) {
            const float nominalTurnsToBound = pathUnits * (float)_activePlan.turnsPerPass;
            const float desiredScale = nominalTurnsToBound / moveTurnsRemaining;
            traverseScale = constrain(desiredScale, 0.40f, 1.80f);
        }
    }
    _activePlan.speedScale = traverseScale;

    // Nominal spindle Hz forwarded to lateral tracking (pre-compensation).
    // The lateral controller uses this to compute its own velocity target via
    // _calcLatHz(); it must receive the intended spindle speed, not the
    // compensation-adjusted value, to avoid a positive-feedback loop.
    const uint32_t hzNominal = hz;

    // ── Dynamic carriage-spindle compensation for uniform wire density ────────
    // Maintains constant Turns_per_mm = Spindle_RPM / Carriage_velocity_mm_s
    // by scaling spindle speed proportionally to the real-time carriage velocity.
    // Applies during traversal ramps (reversal decel/accel) where FastAccelStepper
    // ramps the carriage between 0 and _latHz using LAT_ACCEL.
    // Only active when the lateral is traversing to avoid over-constraining other states.
    if (_lateral.isTraversing()) {
        const uint32_t nominalLatHz = _lateral.getInstantaneousLatHzNominal();
        const uint32_t actualLatHz  = _lateral.getActualVelocityHz();
        if (nominalLatHz > 0) {
            hz = StepperController::calculateCompensatedSpindleHz(hz, actualLatHz, nominalLatHz);
        }
    }

    // ── Hard limits applied AFTER compensation ────────────────────────────────
    // Approach zone cap: protect final-turn quality regardless of compensation.
    if (approachCapHz < UINT32_MAX) {
        hz = min(hz, approachCapHz);
    }

    // Stop-at-next-bound: progressive spindle slowdown over the last 20% of travel.
    if (_lateral.hasStopAtNextBoundArmed()) {
        bool applySlowdown = false;
        if (_lateral.getState() == LatState::WINDING_FWD && _lateral.isStopOnNextHighArmed()) applySlowdown = true;
        else if (_lateral.getState() == LatState::WINDING_BWD && _lateral.isStopOnNextLowArmed()) applySlowdown = true;

        if (applySlowdown) {
            const float progress = _lateral.getTraversalProgress();
            const float zoneStart = 0.80f;
            const float floorFactor = 0.30f;
            if (progress >= zoneStart) {
                const float t = constrain((progress - zoneStart) / (1.0f - zoneStart), 0.0f, 1.0f);
                const float factor = 1.0f - (1.0f - floorFactor) * t;
                hz = max((uint32_t)SPEED_HZ_MIN, (uint32_t)((float)hz * factor));
            }
        }
    }

    _stepper.setSpeedHz(hz);

    if (!_stepper.isRunning()) {
        const bool forward = (_direction == Direction::CW) != (bool)WINDING_MOTOR_INVERTED;
        _state = WindingState::WINDING;
        Diag::infof("[WINDING] %s - %u Hz - tpp=%ld scale=%.2f",
                    forward ? "CW" : "CCW", hz,
                    _activePlan.turnsPerPass, _activePlan.speedScale);
        _stepper.start(forward);
        _lateral.startWinding(hzNominal, _activePlan.turnsPerPass,
                              _windingStartMm(), _windingEndMm(),
                              _activePlan.speedScale);
        return;
    }

    if (_state == WindingState::WINDING && _endPosArmed) {
        if (_recipe.endPos == WindingEndPos::TOP) _lateral.armStopAtNextHigh();
        else if (_recipe.endPos == WindingEndPos::BOTTOM) _lateral.armStopAtNextLow();
        if (_lateral.getState() == LatState::HOMED) return;
    }

    if (_verifyHighPending) _lateral.armStopAtNextHigh();

    if (_lateral.getState() == LatState::HOMED) {
        _lateral.startWinding(hzNominal, _activePlan.turnsPerPass,
                              _windingStartMm(), _windingEndMm(),
                              _activePlan.speedScale);
    } else {
        _lateral.updateWinding(hzNominal, _activePlan.turnsPerPass,
                               _windingStartMm(), _windingEndMm(),
                               _activePlan.speedScale);
    }
}
void WinderApp::_checkAutoStop() {
    if (_state != WindingState::WINDING || _freerun || !_stepper.isRunning()) return;

    if (_recipe.endPos != WindingEndPos::NONE) {
        const long remaining = (long)_targetTurns - _stepper.getTurns();
        if (remaining <= (long)_recipe.endPosTurns && !_endPosArmed) {
            _endPosArmed = true;
            if (_recipe.endPos == WindingEndPos::TOP) _lateral.armStopAtNextHigh();
            else if (_recipe.endPos == WindingEndPos::BOTTOM) _lateral.armStopAtNextLow();
            Diag::infof("[EndPos] Armed %s stop - %ld turns to go, hold=%ld",
                        _recipe.endPos == WindingEndPos::TOP ? "HIGH" : "LOW",
                        remaining, (long)_recipe.endPosTurns);
        }
    }

    if (_stepper.getTurns() >= (long)_targetTurns) {
        _toTargetReached();
    }
}

void WinderApp::_applyDeferredDisable() {
    if (!_pendingDisable) return;
    if (_stepper.isRunning() || _lateral.isBusy()) return;
    _pendingDisable = false;
    _stepper.disableDriver();
    Diag::info("[Winder] Driver disabled (idle)");
}
