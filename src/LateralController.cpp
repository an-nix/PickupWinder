#include "LateralController.h"
#include <Arduino.h>

            void LateralController::begin(FastAccelStepperEngine& engine, float homeOffsetMm) {
                pinMode(HOME_PIN_NO, INPUT_PULLUP);
                pinMode(HOME_PIN_NC, INPUT_PULLUP);

                digitalWrite(ENABLE_PIN_LAT, HIGH);
                pinMode(ENABLE_PIN_LAT, OUTPUT);

                _stepper = engine.stepperConnectToPin(STEP_PIN_LAT);
                if (!_stepper) {
                    Serial.println("[Lateral] ERREUR : impossible d'initialiser le stepper latéral !");
                    _state = LatState::FAULT;
                    return;
                }

                _stepper->setDirectionPin(DIR_PIN_LAT);
                _stepper->setAcceleration(LAT_ACCEL);
                _homeOffsetMm = max(0.0f, homeOffsetMm);
                _passCount = 0;
                Serial.printf("[Lateral] Offset home : %.2f mm\n", _homeOffsetMm);

                delay(10);

                if (!_sensorPresent()) {
                    Serial.println("[Lateral] ERREUR : capteur de position absent ou défaillant !");
                    Serial.printf("[Lateral]   NO=GPIO%d=%d  NC=GPIO%d=%d (attendu : l'un LOW, l'autre HIGH)\n",
                                  HOME_PIN_NO, digitalRead(HOME_PIN_NO),
                                  HOME_PIN_NC, digitalRead(HOME_PIN_NC));
                    _state = LatState::FAULT;
                    return;
                }

                if (_atHome()) {
                    Serial.println("[Lateral] Capteur actif au démarrage — recul avant homing...");
                    _startBackoff();
                } else {
                    Serial.println("[Lateral] Homing en cours — déplacement vers la position initiale...");
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
                            Serial.println("[Lateral] Capteur détecté — reprise du homing.");
                            if (_atHome()) _startBackoff();
                            else           _startHoming();
                        }
                    }
                    break;

                case LatState::BACKOFF:
                    if (!_atHome()) {
                        _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
                        Serial.println("[Lateral] Capteur libéré — départ homing...");
                        _startHoming();
                    } else if (!_stepper->isRunning()) {
                        Serial.println("[Lateral] AVERTISSEMENT : recul terminé, capteur encore actif !");
                        _startHoming();
                    }
                    break;

                case LatState::HOMING:
                    if (_homeFlag || _atHome()) {
                        _homeFlag = false;
                        _detachHomeISR();
                        _stepper->stopMove();
                        _state = LatState::HOMING_DECEL;
                        Serial.println("[Lateral] Capteur atteint — décélération...");
                    } else if (!_stepper->isRunning()) {
                        _detachHomeISR();
                        Serial.println("[Lateral] ERREUR : homing échoué (home non atteint) !");
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
                            Serial.println("[Lateral] AVERTISSEMENT : capteur non détecté (câble ?)");
                        }
                    }
                    break;

                case LatState::POSITIONING:
                    if (!_stepper->isRunning()) {
                        _stepper->forceStopAndNewPosition(_latStartSteps);
                        _state = LatState::HOMED;
                        Serial.printf("[Lateral] ✓ Position de départ atteinte : %.2f mm\n", getCurrentPositionMm());
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
                           if (_pauseOnNextReversal) {
                               _pauseOnNextReversal = false;
                               _pausedAtReversal = true;
                               _state = LatState::HOMED;
                               Serial.printf("[Lateral] ⏸ Pause à la première inversion : %.2f mm\n", getCurrentPositionMm());
                               break;
                           }
                        _stepper->setSpeedInHz(max(1u, _latHz));
                        _stepper->runBackward();
                        _state = LatState::WINDING_BWD;
                        Serial.printf("[Lateral] ↩ Demi-tour → BWD (pos=%ld)\n", (long)_stepper->getCurrentPosition());
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
                        _stepper->setSpeedInHz(max(1u, _latHz));
                        _stepper->runForward();
                        _state = LatState::WINDING_FWD;
                        Serial.printf("[Lateral] ↪ Demi-tour → FWD (pos=%ld)\n", (long)_stepper->getCurrentPosition());
                    }
                    break;
                }
            }

            void LateralController::rehome() {
                if (!_stepper) return;
                if (!_sensorPresent()) {
                    Serial.println("[Lateral] Rehoming impossible : capteur absent.");
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
                    _pausedAtReversal = false;
                if (_atHome()) _startBackoff();
                else           _startHoming();
                Serial.println("[Lateral] Rehoming lancé.");
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

            void LateralController::prepareStartPosition(float startMm) {
                if (_state != LatState::HOMED) return;
                _setTraverseBounds(startMm, startMm);
                if (_isAtStartPosition()) return;
                _enableDriver();
                _stepper->setSpeedInHz(LAT_TRAVERSE_SPEED_HZ);
                _stepper->moveTo(_latStartSteps);
                _state = LatState::POSITIONING;
                Serial.printf("[Lateral] Positionnement départ bobinage : %.2f mm\n", startMm);
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
                Serial.printf("[Lateral] Offset home enregistré : %.2f mm\n", _homeOffsetMm);
            }

            void LateralController::_applyOffsetOrNext() {
                int32_t offsetSteps = (int32_t)(_homeOffsetMm * (float)LAT_STEPS_PER_MM);
                if (offsetSteps > 0) {
                    _stepper->setSpeedInHz(LAT_HOME_SPEED_HZ);
                    _stepper->moveTo(offsetSteps);
                    _state = LatState::HOMING_OFFSET;
                    Serial.printf("[Lateral] Offset : déplacement de %.2f mm (%ld steps)...\n",
                                  _homeOffsetMm, (long)offsetSteps);
                } else {
                    _gotoHomed();
                }
            }

            void LateralController::_gotoHomed() {
                _state = LatState::HOMED;
                _passCount = 0;
                    _pauseOnNextReversal = false;
                    _pausedAtReversal = false;
                Serial.println("[Lateral] ✓ Position 0 atteinte — axe latéral prêt.");
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
                    Serial.printf("[Lateral] startWinding ignoré — état : %s\n", stateStr());
                    return;
                }
                if (windingHz == 0 || tpp <= 0 || endMm <= startMm) {
                    Serial.printf("[Lateral] startWinding ignoré — paramètres invalides : windHz=%u tpp=%ld start=%.2f end=%.2f\n",
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
                Serial.printf("[Lateral] ▶ Bobinage démarré : %u Hz (%.2f mm/s)  passe=%.1fs  dir=%s\n",
                              _latHz, mmPerSec, passDuration,
                              (_state == LatState::WINDING_FWD) ? "FWD" : "BWD");
            }

            void LateralController::updateWinding(uint32_t windingHz, long tpp, float startMm, float endMm, float speedScale) {
                if (_state != LatState::WINDING_FWD && _state != LatState::WINDING_BWD) return;

                _setTraverseBounds(startMm, endMm);
                uint32_t newHz = _calcLatHz(windingHz, tpp, endMm - startMm, speedScale);
                if (newHz < 1) newHz = 1;
                if (newHz != _latHz) {
                    _latHz = newHz;
                    _stepper->setSpeedInHz(_latHz);
                    _stepper->applySpeedAcceleration();
                }
            }

            void LateralController::stopWinding() {
                if (_state != LatState::WINDING_FWD && _state != LatState::WINDING_BWD) return;
                _lastDirFwd = (_state == LatState::WINDING_FWD);  // mémorise la direction
                if (_stepper->isRunning()) {
                    _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
                }
                _state = LatState::HOMED;
                _reversingUntilMs = 0;
                Serial.println("[Lateral] Bobinage arrêté.");
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
