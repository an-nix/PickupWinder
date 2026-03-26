#include "WinderApp.h"
#include <Arduino.h>
#include "Diag.h"

namespace {
uint32_t rpmToHz(uint16_t rpm) {
    return (uint32_t)rpm * (uint32_t)STEPS_PER_REV / 60UL;
}
}

/**
 * @brief Initialize the winding application and subsystems.
 *
 * Loads recipe data from storage, applies it to runtime state, initializes
 * motor/lateral/LED subsystems, and builds an initial traverse plan.
 *
 * @par Usage
 * Called once during startup before the main loop starts calling `tick()`.
 */
void WinderApp::begin() {
    _recipeStore.begin();
    _recipe = _captureRecipe();
    if (_recipeStore.load(_recipe)) {
        Diag::info("[Recipe] Recipe restored from NVS.");
    } else {
        Diag::info("[Recipe] Default recipe loaded.");
    }
    _applyRecipe(_recipe, false);

    // Initialise each subsystem in dependency order.
    _engine.init();                                    // One shared FastAccelStepper engine for all steppers
    _stepper.begin(_engine);                           // GPIO setup + bobbin stepper
    _led.begin();                                      // Set LED pin as output
    _lateral.begin(_engine, _recipe.latOffsetMm);      // GPIO setup + traverse stepper + automatic homing sequence

    _activePlan = _planner.getPlan(0, 0.0f);

    Diag::infof("[Winder] Ready — Geometry: %.1fmm usable, %ld turns/pass, profile=%s",
            _geom.effectiveWidth(), _geom.turnsPerPass(),
                  WindingPatternPlanner::styleName(_recipe.style));
}

/**
 * @brief Run one control-loop iteration.
 *
 * Processes lateral events, pot-controlled state behavior, auto-stop logic,
 * and deferred driver disable.
 *
 * @param potHz Filtered speed command from the potentiometer in Hz.
 * @par Usage
 * Called continuously from the Arduino loop.
 */
void WinderApp::tick(uint32_t potHz) {
    _handleLateralEvents();
    _handlePotCycle(potHz);
    _checkAutoStop();
    _applyDeferredDisable();
}

// ── State transitions ──────────────────────────────────────────────────────────────
//
// Every _toXxx() is self-contained: sets _state, adjusts hardware, clears flags.

/**
 * @brief Transition to IDLE and reset session-related runtime state.
 *
 * Stops motion, clears verification flags, resets turn count and planner state,
 * and parks the lateral axis at home.
 *
 * @par Usage
 * Used by stop/reset flows and at the end of a run.
 */
void WinderApp::_toIdle() {
    _state                 = WindingState::IDLE;
    _canStart              = false;
    _pendingDisable        = true;
    _lowVerified           = false;
    _highVerified          = false;
    _pendingVerify         = PendingVerifyBound::NONE;
    _endPosArmed           = false;
    _stepper.stop();
    _lateral.stopWinding();
    _lateral.parkAtZero();
    _stepper.resetTurns();
    _planner.reset();
    _activePlan = _planner.getPlan(0, 0.0f);
    _led.reset();
    Diag::info("[IDLE] Stop -- carriage to home, counter reset");
}

/**
 * @brief Transition to low-bound verification mode.
 *
 * Depending on current state, either arms a stop on next low bound or directly
 * positions carriage at low bound for operator verification.
 *
 * @par Usage
 * Triggered by startup flow and `verify_low` command.
 */
void WinderApp::_toVerifyLow() {
    if (_state == WindingState::WINDING) {
        // In-session verify: stop on the next natural low bound, then switch to
        // VERIFY_LOW once the carriage is actually sitting on that bound.
        _pendingVerify = PendingVerifyBound::START;
        _lateral.clearOneShotStops();
        _lateral.armStopAtNextLow();
        Diag::info("[VERIFY_LOW] Armed — will stop on next low bound for adjustment");
        return;
    }
    if (_state == WindingState::PAUSED) {
        _pendingVerify   = PendingVerifyBound::NONE;
        _state           = WindingState::VERIFY_LOW;
        _canStart        = false;
        _pendingDisable  = true;
        _lateral.clearOneShotStops();
        _lateral.stopWinding();
        _lateral.prepareStartPosition(_windingStartMm());
        Diag::infof("[VERIFY_LOW] Paused-session verify — carriage → %.2f mm (low bound)",
                _windingStartMm());
        return;
    }
    // Normal startup flow.
    _state          = WindingState::VERIFY_LOW;
    _canStart       = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    _lateral.prepareStartPosition(_windingStartMm());
    Diag::infof("[VERIFY_LOW] Carriage -> %.2f mm (low bound)%s",
            _windingStartMm(), _lowVerified ? " [already confirmed]" : "");
}

/**
 * @brief Transition to high-bound verification mode.
 *
 * Depending on current state, either arms a stop on next high bound or directly
 * positions carriage at high bound for operator verification.
 *
 * @par Usage
 * Triggered by startup flow and `verify_high` command.
 */
void WinderApp::_toVerifyHigh() {
    if (_state == WindingState::WINDING) {
        // In-session verify: stop on the next natural high bound, then switch to
        // VERIFY_HIGH once the carriage is actually sitting on that bound.
        _pendingVerify = PendingVerifyBound::END;
        _lateral.clearOneShotStops();
        _lateral.armStopAtNextHigh();
        Diag::info("[VERIFY_HIGH] Armed — will stop on next high bound for adjustment");
        return;
    }
    if (_state == WindingState::PAUSED) {
        _pendingVerify   = PendingVerifyBound::NONE;
        _state           = WindingState::VERIFY_HIGH;
        _canStart        = false;
        _pendingDisable  = true;
        _lateral.clearOneShotStops();
        _lateral.stopWinding();
        _lateral.prepareStartPosition(_windingEndMm());
        Diag::infof("[VERIFY_HIGH] Paused-session verify — carriage → %.2f mm (high bound)",
                _windingEndMm());
        return;
    }
    // Auto-transition from VERIFY_LOW or normal operator command: go to VERIFY_HIGH.
    // Preserve _lowVerified if we're coming from VERIFY_LOW auto-transition.
    _state          = WindingState::VERIFY_HIGH;
    _canStart       = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    _lateral.prepareStartPosition(_windingEndMm());
    Diag::infof("[VERIFY_HIGH] Carriage -> %.2f mm (high bound)%s",
            _windingEndMm(), _highVerified ? " [already confirmed]" : "");
}

/**
 * @brief Transition to winding-ready state.
 *
 * Clears transient verification/stop flags while preserving carriage position.
 * Actual movement starts when potentiometer conditions allow.
 *
 * @par Usage
 * Entered after both bounds are confirmed.
 */
void WinderApp::_toWinding() {
    _state          = WindingState::WINDING;
    _canStart     = false;
    _pendingDisable = false;
    _pendingVerify  = PendingVerifyBound::NONE;
    _endPosArmed    = false;
    _lateral.clearOneShotStops();
    // Do NOT reposition the carriage here. After verification the carriage is
    // already at the high bound (VERIFY_HIGH). startWinding() will choose the
    // correct initial direction based on the current position.
    Diag::infof("[WINDING] Both bounds verified -- %.2f -> %.2f mm, waiting for pot",
            _windingStartMm(), _windingEndMm());
}

/**
 * @brief Transition to paused state.
 *
 * Stops spindle and lateral motion while preserving current position.
 *
 * @par Usage
 * Entered when operator drops pot to zero or a bound-stop pauses winding.
 */
void WinderApp::_toPaused() {
    _state          = WindingState::PAUSED;
    _canStart     = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    Diag::info("[PAUSED] Resume from current position when pot goes up");
}

/**
 * @brief Transition to target-reached state.
 *
 * Stops winding and clears final-position arm once target turns are reached.
 *
 * @par Usage
 * Called by `_checkAutoStop()` when turns reach configured target.
 */
void WinderApp::_toTargetReached() {
    _state          = WindingState::TARGET_REACHED;
    _canStart     = false;
    _pendingDisable = true;
    _stepper.stop();
    // À la cible, tout doit être déjà en place. On stoppe donc toujours l'axe
    // latéral ; si le positionnement final n'est pas terminé, on préfère un arrêt
    // franc à un mouvement résiduel après l'arrêt du bobinage.
    _lateral.stopWinding();
    _endPosArmed = false;
    Diag::infof("[TARGET_REACHED] %ld turns -- raise target to continue or press Stop",
            _stepper.getTurns());
}

/**
 * @brief Transition to manual jogging mode.
 *
 * Disables automatic traverse behavior and enables encoder-driven carriage jog,
 * with optional capture streaming.
 *
 * @par Usage
 * Entered through `manual` flow.
 */
void WinderApp::_toManual() {
    // Accessible depuis n'importe quel état (sauf MANUAL lui-même).
    // Le moteur est arrêté / le chariot est libéré de la traverse.
    // Le pot redémarre le moteur directement sans gestion de traverse automatique.
    // Les butes windingStartMm/windingEndMm définissent la fenêtre autorisée.
    _state           = WindingState::MANUAL;
    _canStart        = false;
    _pendingDisable  = true;
    _pendingManual   = false;   // consommé
    _manualFirstPass = true;    // premier passage = pas rapide
    _stepper.stop();
    _lateral.stopWinding();    // détache le mode WINDING_FWD/BWD, repasse HOMED
    _captureActive    = false;
    _captureLastPosMm = -999.0f;
    Diag::infof("[MANUAL] Mode manuel actif — fenêtre [%.2f → %.2f mm], pas: %.2f mm (x%d sur 1er passage)",
        _windingStartMm(), _windingEndMm(), _manualJogStepMm, MANUAL_FAST_STEP_MULT);
}

/**
 * @brief Transition to lateral run-in (rodage) mode.
 *
 * Executes repeated shuttle passes on the lateral axis while spindle remains
 * stopped.
 *
 * @par Usage
 * Triggered by `rodage` command from IDLE.
 */
void WinderApp::_toRodage() {
    // Passe le syst\u00e8me en mode rodage : le chariot effectue N allers-retours
    // entre 0 et _rodageDistMm. Le moteur bobineur reste arr\u00eat\u00e9.
    // Uniquement accessible depuis IDLE (chariot d\u00e9j\u00e0 en position 0).
    _state         = WindingState::RODAGE;
    _canStart      = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();  // no-op si pas en WINDING, mais s\u00e9curit\u00e9
    _rodagePassDone = 0;
    _rodageFwd      = true;  // premier mouvement : vers _rodageDistMm
    // D\u00e9marrer le premier passage imm\u00e9diatement (chariot est \u00e0 0 en IDLE).
    if (_lateral.isHomed()) {
        _lateral.prepareStartPosition(_rodageDistMm, LAT_RODAGE_SPEED_HZ);
    }
    Diag::infof("[RODAGE] D\u00e9marrage: %d passes, dist=%.1f mm",
        _rodagePasses, _rodageDistMm);
}

/**
 * @brief Process lateral-axis events and related state transitions.
 *
 * Handles stop-at-bound events, verify transitions, and rodage pass toggling.
 *
 * @par Usage
 * Called from `tick()` every control-loop iteration.
 */
void WinderApp::_handleLateralEvents() {
    _lateral.update();

    // WINDING: the operator armed stop_next_high or stop_next_low.
    // The lateral stopped at the bound (_pausedAtReversal = true, state = HOMED).
    // Transition to PAUSED so the pot must return to 0 before winding resumes.
    if (_state == WindingState::WINDING && _lateral.consumePausedAtReversal()) {
        if (_pendingVerify == PendingVerifyBound::END) {
            _pendingVerify   = PendingVerifyBound::NONE;
            _state           = WindingState::VERIFY_HIGH;
            _canStart        = false;
            _pendingDisable  = true;
            _stepper.stop();
            Diag::info("[VERIFY_HIGH] High bound reached — adjust with encoder/web, then resume");
            return;
        }
        if (_pendingVerify == PendingVerifyBound::START) {
            _pendingVerify   = PendingVerifyBound::NONE;
            _state           = WindingState::VERIFY_LOW;
            _canStart        = false;
            _pendingDisable  = true;
            _stepper.stop();
            Diag::info("[VERIFY_LOW] Low bound reached — adjust with encoder/web, then resume");
            return;
        }
        _toPaused();
        Diag::info("[WINDING] Butée atteinte (stop armé) — passage en PAUSE.");
        return;
    }

    // VERIFY_LOW (normal startup flow): when the carriage reaches the high
    // bound, mark low as confirmed and auto-transition to VERIFY_HIGH.
    if (_state == WindingState::VERIFY_LOW && _lateral.consumePausedAtReversal()) {
        _lowVerified = true;
        _toVerifyHigh();
        Diag::info("[VERIFY_LOW] Butée haute atteinte — passage en VERIFY_HIGH (confirmez pour démarrer).");
    }
    // RODAGE : le chariot vient d'atteindre une but\u00e9e (HOMED apr\u00e8s POSITIONING).
    // Alterner aller-retour jusqu'\u00e0 ce que _rodagePassDone >= _rodagePasses.
    if (_state == WindingState::RODAGE && _lateral.isHomed()) {
        if (_rodageFwd) {
            // Arriv\u00e9 \u00e0 _rodageDistMm \u2014 repartir vers 0.
            _rodageFwd = false;
            _lateral.prepareStartPosition(0.0f, LAT_RODAGE_SPEED_HZ);
        } else {
            // Arriv\u00e9 \u00e0 0 \u2014 aller-retour compl\u00e9t\u00e9.
            _rodagePassDone++;
            Diag::infof("[RODAGE] Passe %d/%d termin\u00e9e", _rodagePassDone, _rodagePasses);
            if (_rodagePassDone >= _rodagePasses) {
                Diag::infof("[RODAGE] Termin\u00e9 (%d passes) \u2014 retour IDLE", _rodagePassDone);
                _toIdle();
            } else {
                _rodageFwd = true;
                _lateral.prepareStartPosition(_rodageDistMm, LAT_RODAGE_SPEED_HZ);
            }
        }
    }}


/**
 * @brief Process potentiometer-driven state behavior.
 *
 * Applies mode-specific start/resume/pause behavior and dispatches to winding
 * update logic when motion is allowed.
 *
 * @param hz Requested spindle speed in Hz.
 * @par Usage
 * Called from `tick()` every control-loop iteration.
 */
void WinderApp::_handlePotCycle(uint32_t hz) {
    if (_rewindMode && _state == WindingState::WINDING) {
        _runWindingAtHz(rpmToHz(_rewindBatchRpm));
        _led.update(_stepper.getTurns(), max(1L, _activePlan.turnsPerPass), _stepper.isRunning());
        return;
    }

    // Arm flag (_canStart): requires the pot to physically return to zero before
    // every (re)start. This prevents unexpected motion if the pot is already
    // raised when entering a new state. Can also be set without a physical zero
    // trip by the UI 'resume' command (useful after a brief pause).
    if (hz == 0) _canStart = true;

    switch (_state) {

    case WindingState::IDLE:
        // When idle the carriage should always rest at the home (zero) position.
        // If an operator moved it manually, or a previous session ended mid-travel,
        // silently drive it back to zero whenever the axis is free.
        if (_lateral.isHomed() && !_lateral.isBusy() && !_lateral.isAtZero())
            _lateral.parkAtZero();
        break;

    case WindingState::VERIFY_LOW:
    case WindingState::VERIFY_HIGH: {
        // Fresh entry (_canStart == false): drive carriage to the verification
        // bound and wait for the operator to raise the pot.
        // Mid-verify pause (_canStart == true): motor stopped because pot hit
        // zero; leave carriage in place so the operator resumes from there.
        if (!_stepper.isRunning() && !_canStart && _lateral.isHomed() && !_lateral.isBusy()) {
            float pos = (_state == WindingState::VERIFY_LOW)
                        ? _windingStartMm() : _windingEndMm();
            _lateral.prepareStartPosition(pos);
        }
        // VERIFY_HIGH after auto-transition from VERIFY_LOW (_lowVerified is true):
        // both bounds have been seen. Raising the pot starts winding directly.
        // Exception : si _pendingManual, on ne démarre pas automatiquement—
        // l'opérateur doit confirmer explicitement pour basculer en MANUAL.
        if (_state == WindingState::VERIFY_HIGH && _lowVerified && hz > 0 && _canStart && !_pendingManual) {
            _highVerified = true;
            _toWinding();
            _runWindingAtHz(hz);
            break;
        }
        if (hz > 0 && _canStart) {
            _runWindingAtHz(hz);         // resumes from current position at verify speed
        } else if (hz == 0 && _stepper.isRunning()) {
            _pendingDisable = true;
            _stepper.stop();
            _lateral.stopWinding();
            Diag::infof("[%s] Pot zero -- paused, will resume from current position",
            windingStateName(_state));
        }
        break;
    }

    case WindingState::WINDING:
        if (hz > 0)
            _runWindingAtHz(hz);
        else
            _toPaused();
        break;

    case WindingState::PAUSED:
        // Pot up + arm flag set + lateral ready -> resume winding.
        if (hz > 0 && _canStart && _lateral.isHomed() && !_lateral.isBusy())
            _runWindingAtHz(hz);
        break;

    case WindingState::MANUAL:
        // En mode manuel le chariot est piloté exclusivement par l'encodeur.
        // Le pot contrôle uniquement le moteur principal (sans limite de vitesse
        // de vérification et sans gestionnaire de traverse automatique).
        if (hz == 0) {
            _canStart = true;
            if (_stepper.isRunning()) {
                _pendingDisable = true;
                _stepper.stop();
            }
        } else if (_canStart) {
            // Démarrage direct sans traverse : on tourne librement.
            bool forward = (_direction == Direction::CW) != (bool)WINDING_MOTOR_INVERTED;
            _stepper.setSpeedHz(hz);
            if (!_stepper.isRunning()) _stepper.start(forward);
        }
        break;

    case WindingState::TARGET_REACHED:
        // Locked -- only raising the target can unlock.
        break;

    case WindingState::RODAGE:
        // Le moteur bobineux ne tourne pas pendant le rodage.
        // Le pot est ignor\u00e9. Le chariot est pilot\u00e9 uniquement par _handleLateralEvents.
        break;
    }

    _led.update(_stepper.getTurns(), max(1L, _activePlan.turnsPerPass), _stepper.isRunning());
}

/**
 * @brief Check whether spindle restart is currently safe.
 *
 * @return true when lateral axis is homed and idle.
 * @par Usage
 * Used as a guard for resume/start transitions.
 */
bool WinderApp::_readyForSpin() const {
    // Used to guard resume after pause: lateral must be homed and idle.
    return _lateral.isHomed() && !_lateral.isBusy();
}

/**
 * @brief Run or update winding at requested spindle speed.
 *
 * Applies all dynamic speed constraints, requests/updates traverse plan,
 * manages verify behavior, and pushes parameters to spindle/lateral control.
 *
 * @param hz Requested spindle speed in Hz.
 * @par Usage
 * Main runtime motor-control path used from `_handlePotCycle()`.
 */
void WinderApp::_runWindingAtHz(uint32_t hz) {
    // ── Speed caps (applied in priority order) ───────────────────────────────
    //
    // 1. Approach zone: linearly ramp the maximum allowed speed down to
    //    APPROACH_SPEED_HZ_FLOOR during the last APPROACH_TURNS turns so the
    //    bobbin coasts smoothly to the target count without overshoot.
    if (!_freerun && _stepper.isRunning()) {
        long remaining = (long)_targetTurns - _stepper.getTurns();
        if (remaining > 0 && remaining <= APPROACH_TURNS) {
            float ratio = (float)remaining / APPROACH_TURNS;
            uint32_t maxHz = APPROACH_SPEED_HZ_FLOOR
                           + (uint32_t)(ratio * (float)(SPEED_HZ_MAX - APPROACH_SPEED_HZ_FLOOR));
            hz = min(hz, maxHz);
        }
    }
    // 2. Verification pass: hard cap at VERIFY_SPEED_HZ_MAX so the traverse axis
    //    can always stop within one half-stroke when the pot drops to zero.
    if (_state == WindingState::VERIFY_LOW || _state == WindingState::VERIFY_HIGH)
        hz = min(hz, (uint32_t)VERIFY_SPEED_HZ_MAX);

    _activePlan = _buildTraversePlan(hz);

    // Actual traverse scale applied.
    //
    // During the final phase (`_endPosArmed`), the firmware does not simply stop
    // at the next bound. It tries to predict the remaining motion toward the
    // requested final bound and spread that motion over the number of turns left
    // before the hold zone begins.
    //
    // Principle:
    //   - `pathUnits` expresses the remaining path to the target bound in
    //     "fractions of a pass":
    //       1.0 = finish the current pass to the bound,
    //       2.0 = first go to the opposite bound, then come back.
    //   - `nominalTurnsToBound = pathUnits * turnsPerPass` is the number of
    //     turns this path would consume with `speedScale = 1.0`.
    //   - `moveTurnsRemaining = remainingTurns - holdTurns` is the turn budget
    //     available before the carriage must already be resting on the bound.
    //   - `desiredScale = nominalTurnsToBound / moveTurnsRemaining` is then used
    //     to target approximately: `turnsToBound ~= moveTurnsRemaining`.
    //
    // Practical effect: if the bound must be reached later, the current pass is
    // slowed down; if it must be reached sooner, it may be sped up (within the
    // allowed 0.40 .. 1.80 range).
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
            if (latState == LatState::WINDING_FWD) {
                pathUnits = 1.0f - progress;
            } else if (latState == LatState::WINDING_BWD) {
                pathUnits = 2.0f - progress;
            } else if (latState == LatState::HOMED && fabsf(_lateral.getCurrentPositionMm() - _windingEndMm()) <= 0.10f) {
                atTargetBound = true;
            }
        } else if (_recipe.endPos == WindingEndPos::BOTTOM) {
            if (latState == LatState::WINDING_BWD) {
                pathUnits = 1.0f - progress;
            } else if (latState == LatState::WINDING_FWD) {
                pathUnits = 2.0f - progress;
            } else if (latState == LatState::HOMED && fabsf(_lateral.getCurrentPositionMm() - _windingStartMm()) <= 0.10f) {
                atTargetBound = true;
            }
        }

        if (!atTargetBound && pathUnits > 0.0f && _activePlan.turnsPerPass > 0) {
            // Nombre de tours que la trajectoire restante consommerait avec scale=1.0.
            const float nominalTurnsToBound = pathUnits * (float)_activePlan.turnsPerPass;
            // Choisir le scale qui fait correspondre la fin de trajectoire avec
            // le début des tours de maintien : turnsToBound = nominal / scale.
            const float desiredScale = nominalTurnsToBound / moveTurnsRemaining;
            traverseScale = constrain(desiredScale, 0.40f, 1.80f);
        }
    }

    _activePlan.speedScale = traverseScale;

    // 3. Reversal slow-down: briefly reduce speed at each traverse flip point
    //    to prevent wire bunching and reduce mechanical shock on the carriage.
    if (_lateral.isReversing())
        hz = max((uint32_t)SPEED_HZ_MIN, (uint32_t)((float)hz * LAT_REVERSAL_SLOWDOWN));

    // 4. "Stop at next bound" mode: once the carriage enters the last 20% of the
    //    current traverse stroke, linearly scale speed down to 30% of its current
    //    value. This ensures a soft, synchronised stop at the flip point without
    //    stalling (the floor is always at least SPEED_HZ_MIN).
    //    Crucially, we only apply this if the carriage is currently traveling
    //    TOWARD the bound where the stop is actually requested.
    if (_lateral.hasStopAtNextBoundArmed()) {
        bool applySlowdown = false;
        if (_lateral.getState() == LatState::WINDING_FWD && _lateral.isStopOnNextHighArmed()) {
            applySlowdown = true;
        } else if (_lateral.getState() == LatState::WINDING_BWD && _lateral.isStopOnNextLowArmed()) {
            applySlowdown = true;
        }

        if (applySlowdown) {
            float progress = _lateral.getTraversalProgress(); // 0..1 in the current direction
            const float zoneStart  = 0.80f;  // start slowing at 80% of the traverse stroke
            const float floorFactor = 0.30f; // never go below 30% of the current speed
            if (progress >= zoneStart) {
                float t = (progress - zoneStart) / (1.0f - zoneStart); // 0..1 inside the zone
                t = constrain(t, 0.0f, 1.0f);
                float factor = 1.0f - (1.0f - floorFactor) * t;
                hz = max((uint32_t)SPEED_HZ_MIN, (uint32_t)((float)hz * factor));
            }
        }
    }

    _stepper.setSpeedHz(hz);

    // ── Motor start ───────────────────────────────────────────────────────────
    if (!_stepper.isRunning()) {
        bool forward = (_direction == Direction::CW) != (bool)WINDING_MOTOR_INVERTED;

        // Transition the state to WINDING on first spin, unless we are in a
        // VERIFY state — verify passes intentionally leave _state unchanged so
        // the operator can continue confirming bounds without losing context.
        if (_state != WindingState::VERIFY_LOW && _state != WindingState::VERIFY_HIGH) {
            _state = WindingState::WINDING;
            Diag::infof("[WINDING] %s -- %u Hz -- profile=%s tpp=%ld scale=%.2f",
            _direction == Direction::CW ? "CW" : "CCW", hz,
                          WindingPatternPlanner::styleName(_recipe.style),
                          _activePlan.turnsPerPass, _activePlan.speedScale);
        }
        // VERIFY_LOW / VERIFY_HIGH: motor spins at capped speed but _state is
        // preserved so the operator can still issue 'confirm' or 'verify_high'.
        //
        // VERIFY_HIGH specifically: the carriage is already at the high bound.
        // Do NOT start lateral traversal — the carriage must stay on the high
        // bound so the operator can visually confirm the end position.
        // VERIFY_LOW starts traversal normally (low → high) so the operator
        // sees the full forward pass from the start bound.

        _stepper.start(forward);
        if (_state == WindingState::VERIFY_LOW) {
            // Arm stop at high bound: lateral will halt at the end of this pass
            // instead of reversing, allowing the operator to confirm the position.
            _lateral.armStopAtNextHigh();
            _lateral.startWinding(hz, _activePlan.turnsPerPass,
                                  _windingStartMm(), _windingEndMm(),
                                  _activePlan.speedScale);
        } else if (_state != WindingState::VERIFY_HIGH) {
            _lateral.startWinding(hz, _activePlan.turnsPerPass,
                                  _windingStartMm(), _windingEndMm(),
                                  _activePlan.speedScale);
        }
        return;
    }

    // ── Motor already running: update traverse parameters ────────────────────
    // VERIFY_HIGH: carriage stays fixed at the high bound — skip lateral cmds.
    if (_state == WindingState::VERIFY_HIGH) return;
    // VERIFY_LOW: keep stop-at-high-bound arm active every tick.
    if (_state == WindingState::VERIFY_LOW)
        _lateral.armStopAtNextHigh();

    // Final phase: once stop-at-bound has been armed, let the traverse complete
    // its natural motion to the requested bound, then keep the carriage still
    // until the final target is reached. Most importantly, do not restart a new
    // traverse when the lateral axis returns to HOMED on that bound, otherwise
    // we would reintroduce the exact unwanted behavior: "reaches the bound,
    // leaves it, then comes back".
    if (_state == WindingState::WINDING && _endPosArmed) {
        if (_recipe.endPos == WindingEndPos::TOP) {
            _lateral.armStopAtNextHigh();
        } else if (_recipe.endPos == WindingEndPos::BOTTOM) {
            _lateral.armStopAtNextLow();
        }
        if (_lateral.getState() == LatState::HOMED) {
            return;
        }
    }

    // If the lateral is in HOMED state it means it completed a positioning move
    // and is waiting for a new winding command — call startWinding() to enter
    // closed-loop traverse. Otherwise the traverse is already active; call
    // updateWinding() to push new speed/tpp values without restarting the pass.
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

/**
 * @brief Evaluate automatic stop and final-position behavior.
 *
 * Arms final bound stop when needed and transitions to TARGET_REACHED
 * once target turns are reached.
 *
 * @par Usage
 * Called every tick while winding is active.
 */
void WinderApp::_checkAutoStop() {
    if (_state != WindingState::WINDING || (bool)_freerun) return;

    // ── Hook position finale ─────────────────────────────────────────────────
    // No direct repositioning toward the final bound is used anymore.
    //
    // Instead, stop-at-next-natural-bound is armed early enough so that the
    // predictive logic in `_runWindingAtHz()` has time to slow the current pass
    // — and if needed the next one — and make the carriage reach the requested
    // bound exactly when the hold turns begin. This avoids artificial end-of-run
    // back-and-forth motion.
    if (_recipe.endPos != WindingEndPos::NONE) {
        const long remainingTurns = (long)_targetTurns - _stepper.getTurns();
        if (remainingTurns > 0 && !_endPosArmed) {
            const long holdTurns = max(1L, (long)_recipe.endPosTurns);
            const float progress = constrain(_lateral.getTraversalProgress(), 0.0f, 1.0f);
            const long tpp = max(1L, _activePlan.turnsPerPass);
            const LatState latState = _lateral.getState();
            float pathUnits = 0.0f;
            bool targetAlreadyReached = false;

            if (_recipe.endPos == WindingEndPos::TOP) {
                if (latState == LatState::WINDING_FWD) {
                    pathUnits = 1.0f - progress;
                } else if (latState == LatState::WINDING_BWD) {
                    pathUnits = 2.0f - progress;
                } else if (latState == LatState::HOMED && fabsf(_lateral.getCurrentPositionMm() - _windingEndMm()) <= 0.10f) {
                    targetAlreadyReached = true;
                }
            } else if (_recipe.endPos == WindingEndPos::BOTTOM) {
                if (latState == LatState::WINDING_BWD) {
                    pathUnits = 1.0f - progress;
                } else if (latState == LatState::WINDING_FWD) {
                    pathUnits = 2.0f - progress;
                } else if (latState == LatState::HOMED && fabsf(_lateral.getCurrentPositionMm() - _windingStartMm()) <= 0.10f) {
                    targetAlreadyReached = true;
                }
            }

            // Arm early enough to allow slowing down as far as scale=0.40.
            // `maxPredictiveTurns` is therefore the largest number of turns over
            // which the remaining path can be spread without leaving the planner's
            // mechanically acceptable range.
            const long maxPredictiveTurns = (long)ceilf((pathUnits * (float)tpp) / 0.40f);

            if (targetAlreadyReached || (pathUnits > 0.0f && remainingTurns <= (holdTurns + maxPredictiveTurns))) {
                _endPosArmed = true;
                if (_recipe.endPos == WindingEndPos::TOP) {
                    _lateral.armStopAtNextHigh();
                } else {
                    _lateral.armStopAtNextLow();
                }
                Diag::infof("[END_POS] Armé: reste %ld tours, trajectoire %.2f passes, maintien %ld tours",
                    remainingTurns, pathUnits, holdTurns);
            }
        }
    }

    // Auto-stop sur la cible.
    if (_stepper.getTurns() >= _targetTurns) {
        _toTargetReached();
    }
}

/**
 * @brief Safely disable stepper driver after stop is complete.
 *
 * Prevents cutting driver current during deceleration ramps.
 *
 * @par Usage
 * Called each tick after stop transitions.
 */
void WinderApp::_applyDeferredDisable() {
    // The stepper driver must not be disabled mid-ramp: cutting power during
    // deceleration causes a mechanical jerk and loses position. _pendingDisable
    // is set by every stop/pause/transition instead of cutting the driver
    // directly. Here, once the stepper reports it has actually stopped, the
    // driver current is safely cut to reduce heat and coil noise.
    if (_pendingDisable && !_stepper.isRunning()) {
        _stepper.disableDriver();
        _pendingDisable = false;
    }
}

/**
 * @brief Build the public runtime status payload.
 *
 * Maps internal state-machine status to transport-facing fields expected by
 * WebSocket and serial links.
 *
 * @return Snapshot of current machine status.
 * @par Usage
 * Queried by telemetry/UI publishing paths.
 */
WinderStatus WinderApp::getStatus() const {
    // Map the current WindingState enum to the flat boolean fields expected by
    // the WebSocket protocol and the LinkSerial bridge. The protocol was designed
    // when WinderApp used individual flags; the state machine now drives all
    // behaviour, but these derived booleans keep the UI layer unchanged.
    const bool sessionActive = (_state != WindingState::IDLE);
    const bool motorEnabled  = (_state == WindingState::WINDING
                             || _state == WindingState::VERIFY_LOW
                             || _state == WindingState::VERIFY_HIGH);

    return {
        _stepper.getRPM(),
        _stepper.getSpeedHz(),
        _stepper.getTurns(),
        (long)_targetTurns,
        (bool)_rewindMode,
        (long)_rewindBatchTurns,
        (uint16_t)_rewindBatchRpm,
        _stepper.isRunning(),
        motorEnabled,
        sessionActive,
        _lateral.isPositionedForStart(),
        (_state == WindingState::VERIFY_LOW),    // verifyLow
        (_state == WindingState::VERIFY_HIGH),   // verifyHigh
        (_state == WindingState::MANUAL),        // manualMode
        (_state == WindingState::RODAGE),        // rodageMode
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

/**
 * @brief Export current recipe as JSON.
 * @return Serialized recipe JSON string.
 * @par Usage
 * Used by UI/remote clients for recipe export.
 */
String WinderApp::recipeJson() const {
    return _recipeStore.toJson(_captureRecipe());
}

// ── Geometry change — live carriage repositioning ────────────────────────────
// When the operator adjusts a geometry parameter (trim, width, wire diameter…)
// the winding bounds change in real time. If the machine is at rest, drive the
// carriage to the affected bound immediately so the operator gets instant visual
// feedback of the new position before starting or resuming.

/**
 * @brief Reposition carriage after geometry bounds update.
 *
 * Repositions only when safe (no spindle movement, lateral free) and only
 * where movement is meaningful for current state.
 *
 * @param startBoundChanged true if low/start bound changed.
 * @param endBoundChanged true if high/end bound changed.
 * @par Usage
 * Called by geometry command handlers after changing trims/dimensions.
 */
void WinderApp::_refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged) {
    // Never interrupt a running motor or an in-progress positioning move.
    if (_stepper.isRunning()) return;
    if (!_lateral.isHomed() || _lateral.isBusy()) return;

    switch (_state) {
    case WindingState::VERIFY_LOW:
        if (startBoundChanged) _lateral.prepareStartPosition(_windingStartMm());
        break;
    case WindingState::VERIFY_HIGH:
        if (endBoundChanged) _lateral.prepareStartPosition(_windingEndMm());
        break;
    case WindingState::WINDING:
    case WindingState::PAUSED:
        // Si le chariot est arrêté en cours de session (PAUSED) ou en attente pot (WINDING),
        // on ne déplace physiquement le chariot QUE s'il est déjà positionné sur la butée
        // que l'utilisateur est en train de modifier. Cela évite un travers traumatique
        // de toute la bobine si un paramètre est changé alors que le chariot est au milieu.
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

// Processes state-machine lifecycle commands (start, stop, verify_low/high,
// confirm, resume, stop_next_low/high). These are always accepted regardless
// of parameter-lock state. Returns true if the command was consumed.
/**
 * @brief Handle immediate lifecycle commands.
 *
 * Processes commands that should always be available regardless of parameter
 * lock state (start/stop/verify/confirm/manual/end-position/rodage).
 *
 * @param cmd Command key.
 * @param value Command payload.
 * @return true if command was consumed.
 * @par Usage
 * First stage of `handleCommand()` dispatch.
 */
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

    if (cmd == "start") {
        if (_rewindMode) {
            if (_state != WindingState::IDLE && _state != WindingState::TARGET_REACHED) {
                Diag::info("[Start] Ignored -- rewind batch mode requires IDLE or TARGET_REACHED");
                return true;
            }
            if (!_lateral.isHomed() || _lateral.isBusy()) {
                Diag::error("[Start] Impossible -- lateral axis not ready");
                return true;
            }

            const long batchTurns = constrain((long)_rewindBatchTurns, 1L, 5000L);
            if (_state == WindingState::IDLE) {
                _stepper.resetTurns();
                _planner.reset();
            }
            _lowVerified = true;
            _highVerified = true;
            _targetTurns = _stepper.getTurns() + batchTurns;
            _toWinding();
            _runWindingAtHz(rpmToHz(_rewindBatchRpm));
            Diag::infof("[REWIND] Batch start -- +%ld turns at %u RPM (target=%ld)",
                        batchTurns, (unsigned)_rewindBatchRpm, (long)_targetTurns);
            return true;
        }

        if (_state != WindingState::IDLE && _state != WindingState::TARGET_REACHED) {
            Diag::info("[Start] Ignored -- session already active");
            return true;
        }
        if (!_lateral.isHomed() || _lateral.isBusy()) {
            Diag::error("[Start] Impossible -- lateral axis not ready");
            return true;
        }
        // Fresh session: reset turn counter and verification state.
        if (_state == WindingState::TARGET_REACHED) {
            _stepper.resetTurns();
            _planner.reset();
        }
        _lowVerified  = false;
        _highVerified = false;
        _toVerifyLow();
        return true;
    }

    // Jump to low-bound verification from any active state.
    if (cmd == "verify_low") {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            Diag::info("[verify_low] No active session");
            return true;
        }
        _toVerifyLow();
        return true;
    }

    // Jump to high-bound verification from any active state.
    if (cmd == "verify_high") {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            Diag::info("[verify_high] No active session");
            return true;
        }
        _toVerifyHigh();
        return true;
    }

    // Confirm the current verification bound; auto-advances when both done.
    if (cmd == "confirm") {
        if (_state == WindingState::VERIFY_LOW) {
            _lowVerified = true;
            Diag::info("[VERIFY_LOW] Low bound confirmed");
            if (_highVerified) {
                if (_pendingManual) _toManual(); else _toWinding();
            } else {
                _toVerifyHigh();
            }
        } else if (_state == WindingState::VERIFY_HIGH) {
            _highVerified = true;
            Diag::info("[VERIFY_HIGH] High bound confirmed");
            if (_lowVerified) {
                if (_pendingManual) _toManual(); else _toWinding();
            } else {
                _toVerifyLow();
            }
        } else {
            Diag::infof("[Confirm] Ignored in state %s",
            windingStateName(_state));
        }
        return true;
    }

    // Resume: arms the motor without requiring a physical pot-zero trip.
    if (cmd == "resume") {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            Diag::info("[Resume] No active session to resume");
        } else if (_canStart) {
            Diag::info("[Resume] Already armed -- raise the pot to start");
        } else {
            _canStart = true;
            Diag::infof("[Resume] Armed in %s -- raise pot to run",
            windingStateName(_state));
        }
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
            // Depuis IDLE : passer d'abord par la vérification des butes pour
            // définir la fenêtre, puis basculer en MANUAL au lieu de WINDING.
            _pendingManual = true;
            _lowVerified   = false;
            _highVerified  = false;
            if (_state == WindingState::TARGET_REACHED) _stepper.resetTurns();
            _toVerifyLow();
            Diag::info("[MANUAL] Vérification des butes avant mode manuel...");
        } else {
            // Déjà dans une session (butes connues) : basculer directement.
            _toManual();
        }
        return true;
    }

    if (cmd == "manual_stop") {
        if (_state == WindingState::MANUAL) {
            _toIdle();
            Diag::info("[MANUAL] Mode manuel terminé → IDLE");
        }
        return true;
    }

    // Réglage du pas de jog manuel (en mm par cran encodeur).
    if (cmd == "manual_step") {
        float s = value.toFloat();
        if (s > 0.0f && s <= 5.0f) {
            _manualJogStepMm = s;
            Diag::infof("[MANUAL] Pas jog: %.3f mm/cran", _manualJogStepMm);
        }
        return true;
    }

    // Démarrage / arrêt de la capture streaming WS.
    if (cmd == "manual_capture_start") {
        if (_state == WindingState::MANUAL) {
            _captureActive    = true;
            _captureLastPosMm = -999.0f;
            Diag::info("[MANUAL] Capture démarrée");
        }
        return true;
    }

    if (cmd == "manual_capture_stop") {
        _captureActive = false;
        Diag::info("[MANUAL] Capture arrêtée");
        return true;
    }

    // ── Position finale de bobinage ───────────────────────────────────────────
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
    // \u2500\u2500 Rodage axe lat\u00e9ral \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
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
            Diag::infof("[RODAGE] Ignor\u00e9 \u2014 \u00e9tat actuel: %s",
                windingStateName(_state));
        }
        return true;
    }
    if (cmd == "rodage_stop") {
        if (_state == WindingState::RODAGE) {
            _toIdle();
            Diag::info("[RODAGE] Arr\u00eat demand\u00e9 par l'op\u00e9rateur");
        }
        return true;
    }
    return false;
}

// Handles all coil geometry parameters (dimensions, wire diameter, trim offsets,
// presets). Geometry can always be adjusted, even during a session — changes take
// effect immediately and the carriage is repositioned if the machine is at rest.
/**
 * @brief Handle geometry-related commands.
 *
 * Applies dimension/trim/preset updates and triggers carriage refresh when
 * appropriate.
 *
 * @param cmd Command key.
 * @param value Command payload.
 * @return true if command was consumed.
 * @par Usage
 * Second stage of `handleCommand()` dispatch.
 */
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

    // Décalage global de la fenêtre : déplace les deux butées du même delta.
    // Utilisable en cours de bobinage (PAUSED, WINDING, VERIFY_*).
    // Positif = décale vers le haut (high), négatif = vers le bas (low).
    if (cmd == "window_shift") {
        float delta = constrain(value.toFloat(), -5.0f, 5.0f);
        _geom.windingStartTrim_mm = constrain(_geom.windingStartTrim_mm + delta, -5.0f, 5.0f);
        _geom.windingEndTrim_mm   = constrain(_geom.windingEndTrim_mm   + delta, -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(true, true);
        _saveRecipe();
        Diag::infof("[WINDOW_SHIFT] Décalage %.2f mm → start_trim=%.2f end_trim=%.2f  fenêtre [%.2f → %.2f mm]",
            delta, _geom.windingStartTrim_mm, _geom.windingEndTrim_mm,
            _windingStartMm(), _windingEndMm());
        return true;
    }

    // Nudge : micro-décalage (+/- 0.05 mm) de toute la fenêtre.
    // Fonctionne dans tous les états sauf IDLE.
    if (cmd == "window_shift_nudge") {
        float delta = constrain(value.toFloat(), -1.0f, 1.0f);
        _geom.windingStartTrim_mm = constrain(_geom.windingStartTrim_mm + delta, -5.0f, 5.0f);
        _geom.windingEndTrim_mm   = constrain(_geom.windingEndTrim_mm   + delta, -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(true, true);
        _saveRecipe();
        Diag::infof("[WINDOW_SHIFT] Nudge %.2f mm → fenêtre [%.2f → %.2f mm]",
            delta, _windingStartMm(), _windingEndMm());
        return true;
    }

    if (cmd == "geom_start_trim_nudge") {
        if (_state != WindingState::VERIFY_LOW && _state != WindingState::PAUSED) {
            Diag::info("[geom_start_trim_nudge] Ignored — only available while verifying the low bound or paused near it");
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
        if (_state != WindingState::VERIFY_HIGH && _state != WindingState::PAUSED) {
            Diag::info("[geom_end_trim_nudge] Ignored — only available while verifying the high bound or paused near it");
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
        Diag::infof("Turns/pass offset: %+ld (calc %ld → effective %ld)",
            _geom.turnsPerPassOffset, _geom.turnsPerPassCalc(),
                      _geom.turnsPerPass());
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

/**
 * @brief Handle winding-pattern commands.
 *
 * Updates style/seed/jitter/humanization parameters and synchronizes planner
 * and persistent recipe state.
 *
 * @param cmd Command key.
 * @param value Command payload.
 * @return true if command was consumed.
 * @par Usage
 * Called by `handleCommand()` for pattern-specific settings.
 */
bool WinderApp::_handlePatternCommand(const String& cmd, const String& value) {
    if (cmd == "winding_style") {
        _recipe.style = WindingPatternPlanner::styleFromString(value);
        _planner.setRecipe(_captureRecipe());
        Diag::infof("Winding style: %s",
            WindingPatternPlanner::styleName(_recipe.style));
        _saveRecipe();
        return true;
    }

    if (cmd == "winding_seed") {
        uint32_t seed = (uint32_t)((value.toInt() > 0) ? value.toInt() : 1);
        _recipe.seed = seed;
        _planner.setRecipe(_captureRecipe());
        Diag::infof("Winding seed: %lu",
            (unsigned long)_recipe.seed);
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

// ── Command dispatch ─────────────────────────────────────────────────────────
// Priority order:
//   1. Immediate / state-machine commands (start, stop, confirm, resume, …)
//   2. Geometry commands (always editable — live feedback is useful mid-session)
//   3. Turn target (always editable — operator may increase target while winding)
//   4. All other parameters (freerun, direction, pattern…) — locked during session
/**
 * @brief Apply encoder delta to active editing/jog behavior.
 *
 * In VERIFY states updates trim bounds; in PAUSED allows near-bound tweaks;
 * in MANUAL mode performs bounded carriage jogging.
 *
 * @param delta Encoder increment/decrement since previous call.
 * @par Usage
 * Called by encoder event consumer in the main loop.
 */
void WinderApp::handleEncoderDelta(int32_t delta) {
    if (delta == 0) return;
    // En mode vérification : l'encodeur contrôle directement le chariot.
    // jog() met à jour la cible même si le chariot est déjà en mouvement.
    // Le trim est mis à jour simultanément pour mémoriser la position en NVS.
    if (_state == WindingState::VERIFY_LOW) {
        float step = delta * ENC_STEP_MM;
        // Jog direct — fonctionne depuis HOMED ou POSITIONING (pas de garde
        // _stepper.isRunning). Mettre à jour le trim depuis la nouvelle cible.
        _lateral.jog(step);
        float newPos = _lateral.getTargetPositionMm();
        _geom.windingStartTrim_mm = constrain(
            newPos - (_geom.flangeBottom_mm + _geom.margin_mm), -5.0f, 5.0f);
        _saveRecipe();
        Diag::infof("[Encoder] Butée basse: %.2f mm (trim %.2f)",
            newPos, _geom.windingStartTrim_mm);
    } else if (_state == WindingState::VERIFY_HIGH) {
        float step = delta * ENC_STEP_MM;
        _lateral.jog(step);
        float newPos = _lateral.getTargetPositionMm();
        _geom.windingEndTrim_mm = constrain(
            newPos - (_geom.totalWidth_mm - _geom.flangeTop_mm - _geom.margin_mm), -5.0f, 5.0f);
        _saveRecipe();
        Diag::infof("[Encoder] Butée haute: %.2f mm (trim %.2f)",
            newPos, _geom.windingEndTrim_mm);
    } else if (_state == WindingState::PAUSED) {
        // En pause, si le chariot est arrêté très proche d'une butée,
        // l'utilisateur peut avoir envie de l'ajuster avec l'encodeur.
        float currentPos = _lateral.getCurrentPositionMm();
        if (fabsf(currentPos - _windingEndMm()) < 0.5f) {
            float step = delta * ENC_STEP_MM;
            _lateral.jog(step);
            float newPos = _lateral.getTargetPositionMm();
            _geom.windingEndTrim_mm = constrain(
                newPos - (_geom.totalWidth_mm - _geom.flangeTop_mm - _geom.margin_mm), -5.0f, 5.0f);
            _saveRecipe();
            Diag::infof("[Encoder] Butée haute ajustée en PAUSE: %.2f mm", newPos);
        } else if (fabsf(currentPos - _windingStartMm()) < 0.5f) {
            float step = delta * ENC_STEP_MM;
            _lateral.jog(step);
            float newPos = _lateral.getTargetPositionMm();
            _geom.windingStartTrim_mm = constrain(
                newPos - (_geom.flangeBottom_mm + _geom.margin_mm), -5.0f, 5.0f);
            _saveRecipe();
            Diag::infof("[Encoder] Butée basse ajustée en PAUSE: %.2f mm", newPos);
        }
    } else if (_state == WindingState::MANUAL) {
        // Pas rapide (x10) sur le premier passage pour traverser vite la fenêtre,
        // puis pas fins dès qu'une bute a été atteinte.
        float stepMm = _manualFirstPass
            ? (_manualJogStepMm * (float)MANUAL_FAST_STEP_MULT)
            : _manualJogStepMm;

        // Calculer la nouvelle cible à partir de la cible actuelle du stepper
        // (pas la position physique, pour éviter les écarts pendant un mouvement).
        float baseMm    = _lateral.getTargetPositionMm();
        float newTarget = constrain(baseMm + delta * stepMm,
                                    _windingStartMm(), _windingEndMm());
        float actualDelta = newTarget - baseMm;
        if (fabsf(actualDelta) > 0.001f) {
            _lateral.jog(actualDelta);
            // Dès que la cible touche une bute, on passe aux pas fins.
            if (_manualFirstPass &&
                (newTarget >= _windingEndMm()   - 0.2f ||
                 newTarget <= _windingStartMm() + 0.2f)) {
                _manualFirstPass = false;
                Diag::infof("[MANUAL] Première bute atteinte — passage aux pas fins (%.2f mm)",
                    _manualJogStepMm);
            }
        }
    }
}

/**
 * @brief Main command dispatcher.
 *
 * Routes incoming commands through immediate/geometric/pattern handlers and
 * applies global command-lock rules for active sessions.
 *
 * @param cmd Command key.
 * @param value Command payload.
 * @par Usage
 * Entry point for WebSocket and serial control commands.
 */
void WinderApp::handleCommand(const String& cmd, const String& value) {
    if (_handleImmediateCommand(cmd, value)) return;
    if (_handleGeometryCommand(cmd, value))  return;

    // Turn target: modifiable at any time. Raising the target above the current
    // count while in TARGET_REACHED transitions back to PAUSED automatically.
    if (cmd == "target") {
        long t = value.toInt();
        if (t > 0) {
            _targetTurns = t;
            if (_state == WindingState::TARGET_REACHED && t > _stepper.getTurns()) {
                _toPaused();
                Diag::info("[PAUSED] Target raised -- resume possible");
            }
            Diag::infof("Target: %ld turns",
            t);
            _saveRecipe();
        }
        return;
    }

    if (cmd == "rewind_mode") {
        _rewindMode = (value == "true");
        Diag::infof("[REWIND] Mode: %s", _rewindMode ? "enabled" : "disabled");
        return;
    }

    if (cmd == "rewind_batch_turns") {
        long t = constrain(value.toInt(), 1L, 5000L);
        _rewindBatchTurns = t;
        Diag::infof("[REWIND] Batch turns: %ld", (long)_rewindBatchTurns);
        return;
    }

    if (cmd == "rewind_batch_rpm") {
        long rpm = constrain(value.toInt(), 10L, 1500L);
        _rewindBatchRpm = (uint16_t)rpm;
        Diag::infof("[REWIND] Batch speed: %u RPM", (unsigned)_rewindBatchRpm);
        return;
    }

    // All other parameters are locked once a session is active.
    if (_parametersLocked()) {
        Diag::infof("[Lock] Ignored during session (%s): %s",
            windingStateName(_state), cmd.c_str());
        return;
    }

    if (cmd == "freerun") {
        _freerun = (value == "true");
        Diag::infof("Mode: %s",
            _freerun ? "FreeRun" : "Target");
        _saveRecipe();
        return;
    }

    if (cmd == "direction") {
        Direction newDir = (value == "cw") ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            Diag::infof("Direction: %s",
            value.c_str());
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
            Diag::error("[Recipe] ERROR -- invalid JSON");
        }
        return;
    }

    if (cmd == "lat_offset") {
        float mm = value.toFloat();
        if (mm >= 0.0f) {
            _lateral.setHomeOffset(mm);
            _toIdle();
            _lateral.rehome();
            Diag::infof("[Lateral] Offset %.2f mm -- rehoming started",
            mm);
            _saveRecipe();
        }
        return;
    }
}

// Queries the pattern planner for the current traverse parameters (turns-per-pass,
// speed scale). The planner may vary these over time to produce scatter, jitter or
// human-like patterns. windingHz is forwarded for future adaptive-speed recipes;
// it is currently unused and suppressed to avoid a compiler warning.
/**
 * @brief Query planner for current traverse parameters.
 *
 * @param windingHz Current spindle speed in Hz (reserved for future adaptive logic).
 * @return Traverse plan for current turns/progress.
 * @par Usage
 * Called from `_runWindingAtHz()` each control update.
 */
TraversePlan WinderApp::_buildTraversePlan(uint32_t windingHz) const {
    (void)windingHz;
    return _planner.getPlan(_stepper.getTurns(), _lateral.getTraversalProgress());
}

// Applies all fields from a recipe struct to every live subsystem.
// Always forces the machine to IDLE: applying a recipe mid-session would leave
// subsystem state inconsistent (different geometry, wrong target, etc.).
// The caller is responsible for issuing a new 'start' command once ready.
/**
 * @brief Apply recipe values to all live subsystems.
 *
 * Resets session state to IDLE to avoid inconsistent mixed-state execution.
 *
 * @param recipe Recipe snapshot to apply.
 * @param persist true to save after apply.
 * @par Usage
 * Used at startup, recipe import, and settings synchronization points.
 */
void WinderApp::_applyRecipe(const WindingRecipe& recipe, bool persist) {
    _recipe      = recipe;
    _geom        = recipe.geometry;
    _targetTurns = recipe.targetTurns;
    _freerun     = recipe.freerun;
    _direction   = recipe.directionCW ? Direction::CW : Direction::CCW;
    // Hard-reset the session: no partial state can survive a recipe change.
    _state                = WindingState::IDLE;
    _canStart           = false;
    _pendingDisable       = false;
    _lateral.setHomeOffset(recipe.latOffsetMm);
    _planner.setRecipe(_recipe);
    _activePlan = _planner.getPlan(_stepper.getTurns(), 0.0f);
    if (persist) _saveRecipe();
}

// Takes a point-in-time snapshot of all mutable runtime parameters
// (target, freerun, direction, geometry, lateral offset) and merges them
// into a copy of _recipe. Used both for NVS persistence and JSON export.
/**
 * @brief Capture current runtime parameters into a recipe snapshot.
 *
 * @return A recipe containing current mutable runtime fields.
 * @par Usage
 * Used before save/export and when updating planner state.
 */
WindingRecipe WinderApp::_captureRecipe() const {
    WindingRecipe recipe = _recipe;
    recipe.targetTurns = _targetTurns;
    recipe.freerun = _freerun;
    recipe.directionCW = (_direction == Direction::CW);
    recipe.geometry = _geom;
    recipe.latOffsetMm = _lateral.getHomeOffset();
    return recipe;
}

// Commits the current live state to NVS. Always call this after mutating any
// recipe field so the planner and persistent store stay in sync. The capture
// step is intentional: _recipe may lag behind volatile fields (direction,
// targetTurns, freerun) that are mutated without going through _applyRecipe.
/**
 * @brief Persist current recipe and synchronize planner input.
 *
 * @par Usage
 * Called after recipe-affecting command changes.
 */
void WinderApp::_saveRecipe() {
    _recipe = _captureRecipe();
    _planner.setRecipe(_recipe);
    _recipeStore.save(_recipe);
}
