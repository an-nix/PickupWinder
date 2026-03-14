#include "WinderApp.h"
#include <Arduino.h>
#include "Diag.h"

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

void WinderApp::tick(uint32_t potHz) {
    _handleLateralEvents();
    _handlePotCycle(potHz);
    _checkAutoStop();
    _applyDeferredDisable();
}

// ── State transitions ──────────────────────────────────────────────────────────────
//
// Every _toXxx() is self-contained: sets _state, adjusts hardware, clears flags.

void WinderApp::_toIdle() {
    _state                 = WindingState::IDLE;
    _canStart              = false;
    _pendingDisable        = true;
    _lowVerified           = false;
    _highVerified          = false;
    _stepper.stop();
    _lateral.stopWinding();
    _lateral.parkAtZero();
    _stepper.resetTurns();
    _planner.reset();
    _activePlan = _planner.getPlan(0, 0.0f);
    _led.reset();
    Diag::info("[IDLE] Stop -- carriage to home, counter reset");
}

void WinderApp::_toVerifyLow() {
    const bool fromWinding = (_state == WindingState::WINDING || _state == WindingState::PAUSED);
    if (fromWinding) {
        // During an active session: stop motor immediately, move carriage to low
        // bound, stay in PAUSED. Pot →0 then ↑ resumes winding from that position.
        _state          = WindingState::PAUSED;
        _canStart       = false;
        _pendingDisable = false;  // forceStop already cuts the driver
        _stepper.forceStop();
        _lateral.stopWinding();
        _lateral.prepareStartPosition(_windingStartMm());
        Diag::infof("[VERIFY_LOW] Quick check depuis bobinage — chariot → %.2f mm (butée basse)",
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

void WinderApp::_toVerifyHigh() {
    const bool fromWinding = (_state == WindingState::WINDING || _state == WindingState::PAUSED);
    if (fromWinding) {
        // During an active session: stop motor immediately, move carriage to high
        // bound, stay in PAUSED. Pot →0 then ↑ resumes winding from that position.
        _state          = WindingState::PAUSED;
        _canStart       = false;
        _pendingDisable = false;  // forceStop already cuts the driver
        _stepper.forceStop();
        _lateral.stopWinding();
        _lateral.prepareStartPosition(_windingEndMm());
        Diag::infof("[VERIFY_HIGH] Quick check depuis bobinage — chariot → %.2f mm (butée haute)",
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

void WinderApp::_toWinding() {
    _state          = WindingState::WINDING;
    _canStart     = false;
    _pendingDisable = false;
    // Do NOT reposition the carriage here. After verification the carriage is
    // already at the high bound (VERIFY_HIGH). startWinding() will choose the
    // correct initial direction based on the current position.
    Diag::infof("[WINDING] Both bounds verified -- %.2f -> %.2f mm, waiting for pot",
            _windingStartMm(), _windingEndMm());
}

void WinderApp::_toPaused() {
    _state          = WindingState::PAUSED;
    _canStart     = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    Diag::info("[PAUSED] Resume from current position when pot goes up");
}

void WinderApp::_toTargetReached() {
    _state          = WindingState::TARGET_REACHED;
    _canStart     = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    Diag::infof("[TARGET_REACHED] %ld turns -- raise target to continue or press Stop",
            _stepper.getTurns());
}

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
void WinderApp::_handleLateralEvents() {
    _lateral.update();

    // WINDING: the operator armed stop_next_high or stop_next_low.
    // The lateral stopped at the bound (_pausedAtReversal = true, state = HOMED).
    // Transition to PAUSED so the pot must return to 0 before winding resumes.
    if (_state == WindingState::WINDING && _lateral.consumePausedAtReversal()) {
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


void WinderApp::_handlePotCycle(uint32_t hz) {
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

bool WinderApp::_readyForSpin() const {
    // Used to guard resume after pause: lateral must be homed and idle.
    return _lateral.isHomed() && !_lateral.isBusy();
}

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

    // 3. Reversal slow-down: briefly reduce speed at each traverse flip point
    //    to prevent wire bunching and reduce mechanical shock on the carriage.
    if (_lateral.isReversing())
        hz = max((uint32_t)SPEED_HZ_MIN, (uint32_t)((float)hz * LAT_REVERSAL_SLOWDOWN));

    // 4. "Stop at next bound" mode: once the carriage enters the last 20% of the
    //    current traverse stroke, linearly scale speed down to 30% of its current
    //    value. This ensures a soft, synchronised stop at the flip point without
    //    stalling (the floor is always at least SPEED_HZ_MIN).
    if (_lateral.hasStopAtNextBoundArmed()) {
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

void WinderApp::_checkAutoStop() {
    // Evaluate the turn target every loop. Only fires during active WINDING and
    // when freerun mode is off. Uses >= rather than == to handle the edge case
    // where the pot drops at the exact target turn and the stepper overshoots by
    // one count before the ISR processes the stop command.
    if (_state == WindingState::WINDING && !_freerun
        && _stepper.getTurns() >= _targetTurns) {
        _toTargetReached();
    }
}

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
        windingStateName(_state),
    };
}

String WinderApp::recipeJson() const {
    return _recipeStore.toJson(_captureRecipe());
}

// ── Geometry change — live carriage repositioning ────────────────────────────
// When the operator adjusts a geometry parameter (trim, width, wire diameter…)
// the winding bounds change in real time. If the machine is at rest, drive the
// carriage to the affected bound immediately so the operator gets instant visual
// feedback of the new position before starting or resuming.

void WinderApp::_refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged) {
    // Never interrupt a running motor or an in-progress positioning move.
    if (_stepper.isRunning()) return;
    if (!_lateral.isHomed() || _lateral.isBusy()) return;

    switch (_state) {
    case WindingState::VERIFY_LOW:
    case WindingState::WINDING:
    case WindingState::PAUSED:
        if (startBoundChanged) _lateral.prepareStartPosition(_windingStartMm());
        break;
    case WindingState::VERIFY_HIGH:
        if (endBoundChanged) _lateral.prepareStartPosition(_windingEndMm());
        break;
    default:
        break;
    }
}

// Processes state-machine lifecycle commands (start, stop, verify_low/high,
// confirm, resume, stop_next_low/high). These are always accepted regardless
// of parameter-lock state. Returns true if the command was consumed.
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

    if (cmd == "geom_start_trim_nudge") {
        float delta = constrain(value.toFloat(), -1.0f, 1.0f);
        _geom.windingStartTrim_mm = constrain(_geom.windingStartTrim_mm + delta, -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(true, false);
        _saveRecipe();
        return true;
    }

    if (cmd == "geom_end_trim_nudge") {
        float delta = constrain(value.toFloat(), -1.0f, 1.0f);
        _geom.windingEndTrim_mm = constrain(_geom.windingEndTrim_mm + delta, -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(false, true);
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

    return false;
}

// ── Command dispatch ─────────────────────────────────────────────────────────
// Priority order:
//   1. Immediate / state-machine commands (start, stop, confirm, resume, …)
//   2. Geometry commands (always editable — live feedback is useful mid-session)
//   3. Turn target (always editable — operator may increase target while winding)
//   4. All other parameters (freerun, direction, pattern…) — locked during session
void WinderApp::handleEncoderDelta(int32_t delta) {
    if (delta == 0) return;
    // En mode vérification : l'encodeur contrôle directement le chariot.
    // jog() met à jour la cible même si le chariot est déjà en mouvement.
    // Le trim est mis à jour simultanément pour mémoriser la position en NVS.
    if (_state == WindingState::VERIFY_LOW) {
        float step = delta * ENC_STEP_MM;
        _geom.windingStartTrim_mm = constrain(_geom.windingStartTrim_mm + step, -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(true, false);
        _saveRecipe();
        Diag::infof("[Encoder] Butée basse: %.2f mm (trim %.2f)",
            _windingStartMm(), _geom.windingStartTrim_mm);
    } else if (_state == WindingState::VERIFY_HIGH) {
        float step = delta * ENC_STEP_MM;
        _geom.windingEndTrim_mm = constrain(_geom.windingEndTrim_mm + step, -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(false, true);
        _saveRecipe();
        Diag::infof("[Encoder] Butée haute: %.2f mm (trim %.2f)",
            _windingEndMm(), _geom.windingEndTrim_mm);
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
TraversePlan WinderApp::_buildTraversePlan(uint32_t windingHz) const {
    (void)windingHz;
    return _planner.getPlan(_stepper.getTurns(), _lateral.getTraversalProgress());
}

// Applies all fields from a recipe struct to every live subsystem.
// Always forces the machine to IDLE: applying a recipe mid-session would leave
// subsystem state inconsistent (different geometry, wrong target, etc.).
// The caller is responsible for issuing a new 'start' command once ready.
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
void WinderApp::_saveRecipe() {
    _recipe = _captureRecipe();
    _planner.setRecipe(_recipe);
    _recipeStore.save(_recipe);
}
