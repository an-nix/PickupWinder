#include "LateralController.h"
#include <Arduino.h>
#include "Diag.h"

void LateralController::begin(FastAccelStepperEngine& engine, float homeOffsetMm) 
{
    pinMode(HOME_PIN_NO, INPUT_PULLUP);
    pinMode(HOME_PIN_NC, INPUT_PULLUP);

    digitalWrite(ENABLE_PIN_LAT, HIGH);
    pinMode(ENABLE_PIN_LAT, OUTPUT);

    _stepper = engine.stepperConnectToPin(STEP_PIN_LAT);
    if (!_stepper) 
    {
        Diag::error("[Lateral] ERREUR : impossible d'initialiser le stepper latéral !");
        _state = LatState::FAULT;
        return;
    }

    _stepper->setDirectionPin(DIR_PIN_LAT);
    _stepper->setAcceleration(LAT_ACCEL);
    _homeOffsetMm = max(0.0f, homeOffsetMm);
    _passCount = 0;
    Diag::infof("[Lateral] Offset home : %.2f mm",
            _homeOffsetMm);

                delay(10);

                if (!_sensorPresent()) {
                    Diag::error("[Lateral] ERREUR : capteur de position absent ou défaillant !");
                    Diag::infof("[Lateral]   NO=GPIO%d=%d  NC=GPIO%d=%d (attendu : l'un LOW, l'autre HIGH)",
            HOME_PIN_NO, digitalRead(HOME_PIN_NO),
                                  HOME_PIN_NC, digitalRead(HOME_PIN_NC));
                    _state = LatState::FAULT;
                    return;
                }

                if (_atHome()) {
                    Diag::info("[Lateral] Capteur actif au démarrage — recul avant homing...");
                    _startBackoff();
                } else {
                    Diag::info("[Lateral] Homing en cours — déplacement vers la position initiale...");
                    _startHoming();
                }
            }

            void LateralController::update() {
                if (!_stepper) return;

                switch (_state) {
                case LatState::FAULT:
                    if (millis() - _lastCheckMs >= 1000) {
                        _lastCheckMs = millis();
                        if (_sensorPresent()) {
                            Diag::info("[Lateral] Capteur détecté — reprise du homing.");
                            if (_atHome()) _startBackoff();
                            else           _startHoming();
                        }
                    }
                    break;

                case LatState::BACKOFF:
                    if (!_atHome()) {
                        _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
                        Diag::info("[Lateral] Capteur libéré — départ homing...");
                        _startHoming();
                    } else if (!_stepper->isRunning()) {
                        Diag::warn("[Lateral] AVERTISSEMENT : recul terminé, capteur encore actif !");
                        _startHoming();
                    }
                    break;

                case LatState::HOMING:
                    if (_homeFlag || _atHome()) {
                        _homeFlag = false;
                        _detachHomeISR();
                        _stepper->stopMove();
                        _state = LatState::HOMING_DECEL;
                        Diag::info("[Lateral] Capteur atteint — décélération...");
                    } else if (!_stepper->isRunning()) {
                        _detachHomeISR();
                        Diag::error("[Lateral] ERREUR : homing échoué (home non atteint) !");
                        _disableDriver();
                        _state = LatState::FAULT;
                    }
                    break;

                case LatState::HOMING_DECEL:
                    if (!_stepper->isRunning()) {
                        int32_t pos = _stepper->getCurrentPosition();
                        int32_t rem = ((pos % MICROSTEPPING) + MICROSTEPPING) % MICROSTEPPING;
                        if (rem != 0) {
                            int32_t target = (rem <= MICROSTEPPING / 2)
                                             ? pos - rem
                                             : pos + (MICROSTEPPING - rem);
                            _stepper->setSpeedInHz(LAT_HOME_SPEED_HZ / 4);
                            _stepper->moveTo(target);
                            _state = LatState::HOMING_ALIGN;
                        } else {
                            _stepper->forceStopAndNewPosition(0);
                            _applyOffsetOrNext();
                        }
                    }
                    break;

                case LatState::HOMING_ALIGN:
                    if (!_stepper->isRunning()) {
                        _stepper->forceStopAndNewPosition(0);
                        _applyOffsetOrNext();
                    }
                    break;

                case LatState::HOMING_OFFSET:
                    if (!_stepper->isRunning()) {
                        _stepper->forceStopAndNewPosition(0);
                        _gotoHomed();
                    }
                    break;

                case LatState::HOMED:
                    if (millis() - _lastCheckMs >= 5000) {
                        _lastCheckMs = millis();
                        if (!_sensorPresent()) {
                            Diag::warn("[Lateral] AVERTISSEMENT : capteur non détecté (câble ?)");
                        }
                    }
                    break;

                case LatState::POSITIONING:
                    if (!_stepper->isRunning()) {
                        _stepper->forceStopAndNewPosition(_latStartSteps);
                        _state = LatState::HOMED;
                        Diag::infof("[Lateral] ✓ Position de départ atteinte : %.2f mm",
            getCurrentPositionMm());
                    }
                    break;

                case LatState::WINDING_FWD:
                    if (_latEndSteps <= _latStartSteps) {
                        _stepper->stopMove();
                        _state = LatState::HOMED;
                        break;
                    }
                    if (_stepper->getCurrentPosition() >= _latEndSteps) {
                        _stepper->forceStopAndNewPosition(_latEndSteps);
                        _passCount++;
                        _onReversal();
                           if (_pauseOnNextReversal || _stopOnNextHigh) {
                               _pauseOnNextReversal = false;
                               _stopOnNextHigh = false;
                               _pausedAtReversal = true;
                               _state = LatState::HOMED;
                               Diag::infof("[Lateral] ⏸ Arrêt sur butée haute : %.2f mm",
            getCurrentPositionMm());
                               break;
                           }
                        _stepper->setSpeedInHz(max(1u, _latHz));
                        _stepper->runBackward();
                        _state = LatState::WINDING_BWD;
                        Diag::infof("[Lateral] ↩ Demi-tour → BWD (pos=%ld)",
            (long)_stepper->getCurrentPosition());
                    }
                    break;

                case LatState::WINDING_BWD:
                    if (_latEndSteps <= _latStartSteps) {
                        _stepper->stopMove();
                        _state = LatState::HOMED;
                        break;
                    }
                    if (_stepper->getCurrentPosition() <= _latStartSteps) {
                        _stepper->forceStopAndNewPosition(_latStartSteps);
                        _passCount++;
                        _onReversal();
                        if (_stopOnNextLow) {
                            _stopOnNextLow = false;
                            _pausedAtReversal = true;
                            _state = LatState::HOMED;
                            Diag::infof("[Lateral] ⏸ Arrêt sur butée basse : %.2f mm",
            getCurrentPositionMm());
                            break;
                        }
                        _stepper->setSpeedInHz(max(1u, _latHz));
                        _stepper->runForward();
                        _state = LatState::WINDING_FWD;
                        Diag::infof("[Lateral] ↪ Demi-tour → FWD (pos=%ld)",
            (long)_stepper->getCurrentPosition());
                    }
                    break;
                }
            }

            void LateralController::rehome() {
                if (!_stepper) return;
                if (!_sensorPresent()) {
                    Diag::error("[Lateral] Rehoming impossible : capteur absent.");
                    return;
                }
                if (_state == LatState::HOMING) _detachHomeISR();
                if (_stepper->isRunning()) {
                    _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
                }
                _disableDriver();
                _passCount = 0;
                _latStartSteps = 0;
                _latEndSteps = 0;
                    _pauseOnNextReversal = false;
                    _stopOnNextHigh = false;
                    _stopOnNextLow = false;
                    _pausedAtReversal = false;
                if (_atHome()) _startBackoff();
                else           _startHoming();
                Diag::info("[Lateral] Rehoming lancé.");
            }

            const char* LateralController::stateStr() const {
                switch (_state) {
                    case LatState::FAULT:         return "FAULT";
                    case LatState::BACKOFF:       return "BACKOFF";
                    case LatState::HOMING:        return "HOMING";
                    case LatState::HOMING_DECEL:  return "HOMING_DECEL";
                    case LatState::HOMING_ALIGN:  return "HOMING_ALIGN";
                    case LatState::HOMING_OFFSET: return "HOMING_OFFSET";
                    case LatState::HOMED:         return "HOMED";
                    case LatState::POSITIONING:   return "POSITIONING";
                    case LatState::WINDING_FWD:   return "WIND_FWD";
                    case LatState::WINDING_BWD:   return "WIND_BWD";
                    default:                      return "?";
                }
            }

            // Move the carriage toward an absolute target. During the final
            // winding phase this method may be called repeatedly with the same
            // target but a different speed; therefore both HOMED and POSITIONING
            // states are accepted so an in-flight move can be retimed without
            // changing the target or forcing a return to zero.
            void LateralController::prepareStartPosition(float startMm, uint32_t speedHz) {
                if (_state != LatState::HOMED && _state != LatState::POSITIONING) return;

                const bool wasPositioning = (_state == LatState::POSITIONING);
                const int32_t prevTarget = _latStartSteps;
                _setTraverseBounds(startMm, startMm);

                if (_isAtStartPosition()) {
                    if (_stepper->isRunning()) {
                        _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
                    }
                    _state = LatState::HOMED;
                    return;
                }

                _enableDriver();
                _stepper->setSpeedInHz(max(1u, speedHz));
                _stepper->moveTo(_latStartSteps);
                _state = LatState::POSITIONING;

                if (!wasPositioning || prevTarget != _latStartSteps) {
                    Diag::infof("[Lateral] Positionnement départ bobinage : %.2f mm",
                startMm);
                }
            }

            void LateralController::_startHoming() {
                _attachHomeISR();
                _enableDriver();
                _stepper->setSpeedInHz(LAT_HOME_SPEED_HZ);
                if (LAT_HOME_DIR) _stepper->runForward();
                else              _stepper->runBackward();
                _state = LatState::HOMING;
            }

            void LateralController::_startBackoff() {
                _enableDriver();
                _stepper->setSpeedInHz(LAT_HOME_SPEED_HZ);
                if (LAT_HOME_DIR) _stepper->runBackward();
                else              _stepper->runForward();
                _state = LatState::BACKOFF;
            }

            void LateralController::_enableDriver() {
                digitalWrite(ENABLE_PIN_LAT, LOW);
            }

            void LateralController::_disableDriver() {
                if (_stepper && _stepper->isRunning()) {
                    _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
                }
                digitalWrite(ENABLE_PIN_LAT, HIGH);
            }

            void LateralController::setHomeOffset(float mm) {
                _homeOffsetMm = (mm < 0.0f) ? 0.0f : mm;
                Diag::infof("[Lateral] Offset home enregistré : %.2f mm",
            _homeOffsetMm);
            }

            void LateralController::_applyOffsetOrNext() {
                int32_t offsetSteps = (int32_t)(_homeOffsetMm * (float)LAT_STEPS_PER_MM);
                if (offsetSteps > 0) {
                    _stepper->setSpeedInHz(LAT_HOME_SPEED_HZ);
                    _stepper->moveTo(offsetSteps);
                    _state = LatState::HOMING_OFFSET;
                    Diag::infof("[Lateral] Offset : déplacement de %.2f mm (%ld steps)...",
            _homeOffsetMm, (long)offsetSteps);
                } else {
                    _gotoHomed();
                }
            }

            void LateralController::_gotoHomed() {
                _state = LatState::HOMED;
                _passCount = 0;
                    _pauseOnNextReversal = false;
                    _stopOnNextHigh = false;
                    _stopOnNextLow = false;
                    _pausedAtReversal = false;
                Diag::info("[Lateral] ✓ Position 0 atteinte — axe latéral prêt.");
            }

            uint32_t LateralController::_calcLatHz(uint32_t windingHz, long tpp, float effWidthMm, float speedScale) const {
                if (tpp <= 0 || windingHz == 0 || effWidthMm <= 0.0f) return 0;
                float effSteps = effWidthMm * (float)LAT_STEPS_PER_MM;
                float hz = effSteps * (float)windingHz / ((float)tpp * (float)STEPS_PER_REV);
                hz *= constrain(speedScale, 0.4f, 1.8f);
                return (uint32_t)max(1.0f, hz);
            }

            void LateralController::_onReversal() {
                uint32_t ms = (_latHz > 0)
                    ? (uint32_t)(2000.0f * (float)_latHz / (float)LAT_ACCEL)
                    : 200;
                _reversingUntilMs = millis() + max(100u, min(600u, ms));
            }

            void LateralController::startWinding(uint32_t windingHz, long tpp, float startMm, float endMm, float speedScale) {
                if (_state == LatState::WINDING_FWD || _state == LatState::WINDING_BWD) {
                    updateWinding(windingHz, tpp, startMm, endMm, speedScale);
                    return;
                }
                if (_state != LatState::HOMED) {
                    Diag::infof("[Lateral] startWinding ignoré — état : %s",
            stateStr());
                    return;
                }
                if (windingHz == 0 || tpp <= 0 || endMm <= startMm) {
                    Diag::errorf("[Lateral] startWinding ignoré — paramètres invalides : windHz=%u tpp=%ld start=%.2f end=%.2f",
             windingHz, (long)tpp, startMm, endMm);
                    return;
                }

                _enableDriver();
                _setTraverseBounds(startMm, endMm);
                _latHz = _calcLatHz(windingHz, tpp, endMm - startMm, speedScale);
                _passCount = 0;
                    _pausedAtReversal = false;

                _stepper->setSpeedInHz(_latHz);
                _stepper->applySpeedAcceleration();

                int32_t pos = _stepper->getCurrentPosition();
                // Si arrêté en cours de passe (ni à la butée start ni à la butée end),
                // reprendre dans la même direction qu'avant la pause.
                // Sinon, décider par position (cas normal : départ depuis start).
                bool goFwd;
                if (pos > _latStartSteps && pos < _latEndSteps) {
                    goFwd = _lastDirFwd;
                } else {
                    goFwd = (pos < (_latStartSteps + _latEndSteps) / 2);
                }
                if (goFwd) {
                    _stepper->runForward();
                    _state = LatState::WINDING_FWD;
                } else {
                    _stepper->runBackward();
                    _state = LatState::WINDING_BWD;
                }

                float mmPerSec = (float)_latHz / (float)LAT_STEPS_PER_MM;
                float passDuration = (mmPerSec > 0.0f) ? ((endMm - startMm) / mmPerSec) : 0.0f;
                Diag::infof("[Lateral] ▶ Bobinage démarré : %u Hz (%.2f mm/s)  passe=%.1fs  dir=%s",
            _latHz, mmPerSec, passDuration,
                              (_state == LatState::WINDING_FWD) ? "FWD" : "BWD");
            }

            void LateralController::updateWinding(uint32_t windingHz, long tpp, float startMm, float endMm, float speedScale) {
                if (_state != LatState::WINDING_FWD && _state != LatState::WINDING_BWD) return;

                float s = max(0.0f, startMm);
                float e = max(s, endMm);
                int32_t newStart = (int32_t)(s * (float)LAT_STEPS_PER_MM);
                int32_t newEnd   = (int32_t)(e * (float)LAT_STEPS_PER_MM);

                // Apply the bound the carriage is heading toward immediately.
                // The bound behind the carriage will be applied on the next pass
                // (applying it now would have no effect anyway since we are
                // already travelling away from it).
                if (_state == LatState::WINDING_FWD) {
                    _latEndSteps = max(newEnd, _latStartSteps);
                } else {
                    _latStartSteps = min(newStart, _latEndSteps);
                }

                // ── Overrun detection ─────────────────────────────────────────
                // The operator may have shrunk the active bound to a position
                // that is already behind the carriage (e.g. end bound moved from
                // 40 mm to 30 mm while the carriage is at 35 mm heading forward).
                // In that case the carriage has logically overshot the new bound.
                // Trigger the same reversal sequence that update() would normally
                // produce, so that behaviour stays consistent with every other
                // reversal: snap position to the bound (corrects the offset error)
                // and immediately change direction.
                int32_t pos = _stepper->getCurrentPosition();

                if (_state == LatState::WINDING_FWD && pos >= _latEndSteps) {
                    _stepper->forceStopAndNewPosition(_latEndSteps);
                    _passCount++;
                    _onReversal();
                    if (_pauseOnNextReversal || _stopOnNextHigh) {
                        _pauseOnNextReversal = false;
                        _stopOnNextHigh      = false;
                        _pausedAtReversal    = true;
                        _state = LatState::HOMED;
                        Diag::infof("[Lateral] Overrun corrected — stopped at high bound (%.2f mm)",
                            getCurrentPositionMm());
                    } else {
                        _stepper->setSpeedInHz(max(1u, _latHz));
                        _stepper->runBackward();
                        _state = LatState::WINDING_BWD;
                        Diag::infof("[Lateral] Overrun corrected — reversed to BWD at %.2f mm (bound moved behind carriage)",
                            getCurrentPositionMm());
                    }
                } else if (_state == LatState::WINDING_BWD && pos <= _latStartSteps) {
                    _stepper->forceStopAndNewPosition(_latStartSteps);
                    _passCount++;
                    _onReversal();
                    if (_stopOnNextLow) {
                        _stopOnNextLow    = false;
                        _pausedAtReversal = true;
                        _state = LatState::HOMED;
                        Diag::infof("[Lateral] Overrun corrected — stopped at low bound (%.2f mm)",
                            getCurrentPositionMm());
                    } else {
                        _stepper->setSpeedInHz(max(1u, _latHz));
                        _stepper->runForward();
                        _state = LatState::WINDING_FWD;
                        Diag::infof("[Lateral] Overrun corrected — reversed to FWD at %.2f mm (bound moved behind carriage)",
                            getCurrentPositionMm());
                    }
                }

                // ── Speed update ──────────────────────────────────────────────
                uint32_t newHz = _calcLatHz(windingHz, tpp, endMm - startMm, speedScale);
                if (newHz < 1) newHz = 1;
                if (newHz != _latHz) {
                    _latHz = newHz;
                    _stepper->setSpeedInHz(_latHz);
                    _stepper->applySpeedAcceleration();
                }
            }

            void LateralController::jog(float deltaMm) {
                if (_state != LatState::HOMED && _state != LatState::POSITIONING) return;
                // Base = cible actuelle si on est déjà en mouvement, sinon position courante.
                float baseMm = (_state == LatState::POSITIONING)
                    ? (float)_latStartSteps / (float)LAT_STEPS_PER_MM
                    : getCurrentPositionMm();
                float targetMm = constrain(baseMm + deltaMm, 0.0f, (float)LAT_TRAVERSE_MM);
                _latStartSteps = (int32_t)(targetMm * (float)LAT_STEPS_PER_MM);
                _latEndSteps   = _latStartSteps;
                _enableDriver();
                _stepper->setSpeedInHz(LAT_TRAVERSE_SPEED_HZ);
                _stepper->moveTo(_latStartSteps);
                _state = LatState::POSITIONING;
                Diag::infof("[Lateral] Jog → %.2f mm", targetMm);
            }

            void LateralController::clearOneShotStops() {
                _pauseOnNextReversal = false;
                _stopOnNextHigh = false;
                _stopOnNextLow = false;
                _pausedAtReversal = false;
            }

            void LateralController::stopWinding() {
                // POSITIONING : mouvement de repositionnement en cours (ex. phase finale
                // de bobinage). Forcer l'arrêt et repasser en HOMED.
                if (_state == LatState::POSITIONING) {
                    if (_stepper->isRunning()) {
                        _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
                    }
                    _state = LatState::HOMED;
                    Diag::info("[Lateral] Positionnement annulé (stopWinding).");
                    return;
                }
                if (_state != LatState::WINDING_FWD && _state != LatState::WINDING_BWD) return;
                _lastDirFwd = (_state == LatState::WINDING_FWD);  // mémorise la direction
                clearOneShotStops();
                if (_stepper->isRunning()) {
                    _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
                }
                _state = LatState::HOMED;
                _reversingUntilMs = 0;
                Diag::info("[Lateral] Bobinage arrêté.");
            }

            float LateralController::getTraversalProgress() const {
                if (!_stepper || _latEndSteps <= _latStartSteps) return 0.0f;
                float pos = ((float)_stepper->getCurrentPosition() - (float)_latStartSteps)
                          / ((float)_latEndSteps - (float)_latStartSteps);
                if (_state == LatState::WINDING_BWD) pos = 1.0f - pos;
                return constrain(pos, 0.0f, 1.0f);
            }

            float LateralController::getCurrentPositionMm() const {
                if (!_stepper) return 0.0f;
                return (float)_stepper->getCurrentPosition() / (float)LAT_STEPS_PER_MM;
            }

            float LateralController::getTargetPositionMm() const {
                if (!_stepper) return 0.0f;
                // En POSITIONING, utiliser la cible (_latStartSteps) plutôt que la
                // position physique pour éviter l'accumulation d'erreur lors d'appels
                // répétés alors que le chariot est encore en mouvement.
                if (_state == LatState::POSITIONING)
                    return (float)_latStartSteps / (float)LAT_STEPS_PER_MM;
                return (float)_stepper->getCurrentPosition() / (float)LAT_STEPS_PER_MM;
            }

            bool LateralController::_isAtStartPosition() const {
                if (!_stepper) return false;
                return abs(_stepper->getCurrentPosition() - _latStartSteps) <= MICROSTEPPING;
            }

            void LateralController::_setTraverseBounds(float startMm, float endMm) {
                float s = max(0.0f, startMm);
                float e = max(s, endMm);
                _latStartSteps = (int32_t)(s * (float)LAT_STEPS_PER_MM);
                _latEndSteps   = (int32_t)(e * (float)LAT_STEPS_PER_MM);
            }

            void IRAM_ATTR LateralController::_homePinISR(void* arg) {
                LateralController* self = static_cast<LateralController*>(arg);
                if (self->_state == LatState::HOMING && !self->_homeFlag) {
                    self->_homeFlag = true;
                }
            }

            void LateralController::_attachHomeISR() {
                _homeFlag = false;
                attachInterruptArg(digitalPinToInterrupt(HOME_PIN_NO), _homePinISR, this, FALLING);
            }

            void LateralController::_detachHomeISR() {
                detachInterrupt(digitalPinToInterrupt(HOME_PIN_NO));
            }
