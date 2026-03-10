#include "LateralController.h"
#include <Arduino.h>
#include <Preferences.h>

// ─────────────────────────────────────────────────────────────────────────────

void LateralController::begin(FastAccelStepperEngine& engine) {
    // Configurer les pins capteur en INPUT_PULLUP.
    pinMode(HOME_PIN_NO, INPUT_PULLUP);
    pinMode(HOME_PIN_NC, INPUT_PULLUP);

    // Pré-positionner ENABLE_PIN_LAT à HIGH AVANT d'activer la sortie.
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

    // Charger l'offset depuis la mémoire NVS (persistant entre les redémarrages).
    {
        Preferences prefs;
        prefs.begin("lateral", true);  // read-only
        _homeOffsetMm = prefs.getFloat("offset_mm", LAT_HOME_OFFSET_DEFAULT_MM);
        prefs.end();
        Serial.printf("[Lateral] Offset home : %.2f mm\n", _homeOffsetMm);
    }

    // Laisser les pull-ups se stabiliser avant de lire le capteur.
    delay(10);

    // ── Vérification de la présence du capteur ────────────────────────────
    if (!_sensorPresent()) {
        Serial.println("[Lateral] ERREUR : capteur de position absent ou défaillant !");
        Serial.printf ("[Lateral]   NO=GPIO%d=%d  NC=GPIO%d=%d"
                       " (attendu : l'un LOW, l'autre HIGH)\n",
                       HOME_PIN_NO, digitalRead(HOME_PIN_NO),
                       HOME_PIN_NC, digitalRead(HOME_PIN_NC));
        _state = LatState::FAULT;
        return;
    }

    // ── Décision initiale ─────────────────────────────────────────────────
    if (_atHome()) {
        // Capteur déjà actif : reculer d'abord pour pouvoir refaire un homing propre.
        Serial.println("[Lateral] Capteur actif au démarrage — recul avant homing...");
        _startBackoff();
    } else {
        Serial.println("[Lateral] Homing en cours — déplacement vers la position initiale...");
        _startHoming();
    }
}

// ─────────────────────────────────────────────────────────────────────────────

void LateralController::update() {
    if (!_stepper) return;

    switch (_state) {

    // ── FAULT ─────────────────────────────────────────────────────────────
    case LatState::FAULT:
        // Ré-vérification toutes les secondes pour récupérer si le capteur
        // vient d'être branché ou remis en état.
        if (millis() - _lastCheckMs >= 1000) {
            _lastCheckMs = millis();
            if (_sensorPresent()) {
                Serial.println("[Lateral] Capteur détecté — reprise du homing.");
                if (_atHome()) _startBackoff();
                else           _startHoming();
            }
        }
        break;

    // ── BACKOFF ───────────────────────────────────────────────────────────
    case LatState::BACKOFF:
        if (!_atHome()) {
            // Capteur libéré → arrêt immédiat et départ en homing.
            _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
            Serial.println("[Lateral] Capteur libéré — départ homing...");
            _startHoming();
        } else if (!_stepper->isRunning()) {
            // Stepper arrêté sans libérer le capteur (limite de course atteinte ?).
            // On tente quand même le homing en sens inverse.
            Serial.println("[Lateral] AVERTISSEMENT : recul terminé, capteur encore actif !");
            _startHoming();
        }
        break;

    // ── HOMING ────────────────────────────────────────────────────────────
    case LatState::HOMING:
        // Priorité : drapeau ISR (réaction instantanée, indépendante de la boucle).
        // Fallback  : polling _atHome() pour le cas où l'ISR aurait été manquée.
        if (_homeFlag || _atHome()) {
            _homeFlag = false;
            _detachHomeISR();
            // Décélération normale : le moteur s'arrête sur sa rampe d'accélération,
            // sans choc mécanique. On attend l'arrêt réel dans HOMING_DECEL.
            _stepper->stopMove();
            _state = LatState::HOMING_DECEL;
            Serial.println("[Lateral] Capteur atteint — décélération...");
        } else if (!_stepper->isRunning()) {
            // Stepper stoppé sans avoir trouvé le home → faute.
            _detachHomeISR();
            Serial.println("[Lateral] ERREUR : homing échoué (home non atteint) !");
            _disableDriver();
            _state = LatState::FAULT;
        }
        break;

    // ── HOMING_DECEL ──────────────────────────────────────────────────────
    case LatState::HOMING_DECEL:
        if (!_stepper->isRunning()) {
            // Le moteur s'est arrêté à un microstep arbitraire à l'intérieur d'un
            // pas complet. Selon la phase exacte, le courant de maintien génère
            // un grésillement intermittent (résonance mécanique).
            // Solution : micro-mouvement vers le PAS COMPLET LE PLUS PROCHE
            // (MICROSTEPPING = 32 microsteps), où l'équilibre des bobines est optimal.
            int32_t pos = _stepper->getCurrentPosition();
            int32_t rem = ((pos % MICROSTEPPING) + MICROSTEPPING) % MICROSTEPPING;
            if (rem != 0) {
                // Choisir la direction la plus courte vers un pas complet.
                int32_t target = (rem <= MICROSTEPPING / 2)
                                 ? pos - rem                   // reculer
                                 : pos + (MICROSTEPPING - rem); // avancer
                _stepper->setSpeedInHz(LAT_HOME_SPEED_HZ / 4); // très lent, doux
                _stepper->moveTo(target);
                _state = LatState::HOMING_ALIGN;
            } else {
                // Déjà aligné — pas de mouvement nécessaire.
                _stepper->forceStopAndNewPosition(0);
                _applyOffsetOrNext();
            }
        }
        break;

    // ── HOMING_ALIGN ────────────────────────────────────────────────────
    case LatState::HOMING_ALIGN:
        if (!_stepper->isRunning()) {
            _stepper->forceStopAndNewPosition(0);
            _applyOffsetOrNext();
        }
        break;

    // ── HOMING_OFFSET ──────────────────────────────────────────────────
    case LatState::HOMING_OFFSET:
        if (!_stepper->isRunning()) {
            // Offset atteint : cette position devient le zéro réel.
            _stepper->forceStopAndNewPosition(0);
            _gotoHomed();
        }
        break;

    // ── HOMED ──────────────────────────────────────────────────────────
    case LatState::HOMED:
        // Driver maintenu actif en permanence.
        // Vérification périodique de la santé du capteur (toutes les 5 s).
        if (millis() - _lastCheckMs >= 5000) {
            _lastCheckMs = millis();
            if (!_sensorPresent()) {
                Serial.println("[Lateral] AVERTISSEMENT : capteur non détecté (câble ?)");
            }
        }
        break;
    // ── WINDING_FWD ───────────────────────────────────────────
    // Bobinage réel : aller (0 → effWidth). Quand le stepper s'arrête :
    //   1. on déclenche la fenêtre de ralentissement pour WinderApp,
    //   2. on repart immédiatement vers 0 (pas d'attente d'arrêt complet).
    case LatState::WINDING_FWD:
        if (_latEndSteps <= 0) {
            _stepper->stopMove();
            _state = LatState::HOMED;
            break;
        }
        if (_stepper->getCurrentPosition() >= _latEndSteps) {
            _stepper->forceStopAndNewPosition(_latEndSteps);
            _onReversal();
            _stepper->setSpeedInHz(max(1u, _latHz));
            _stepper->runBackward();
            _state = LatState::WINDING_BWD;
            Serial.printf("[Lateral] ↩ Demi-tour → BWD (pos=%ld)\n", (long)_stepper->getCurrentPosition());
        }
        break;

    // ── WINDING_BWD ──────────────────────────────────────────
    // Bobinage réel : retour (effWidth → 0). Même logique.
    case LatState::WINDING_BWD:
        if (_latEndSteps <= 0) {
            _stepper->stopMove();
            _state = LatState::HOMED;
            break;
        }
        if (_stepper->getCurrentPosition() <= 0) {
            _stepper->forceStopAndNewPosition(0);
            _onReversal();
            _stepper->setSpeedInHz(max(1u, _latHz));
            _stepper->runForward();
            _state = LatState::WINDING_FWD;
            Serial.printf("[Lateral] ↪ Demi-tour → FWD (pos=%ld)\n", (long)_stepper->getCurrentPosition());
        }
        break;
#ifdef LAT_TEST_TRAVERSE
    // ── TRAVERSE_FWD ───────────────────────────────────────────────────
    // Aller : 0 → effWidth. Chaque sens compte +1 passe.
    case LatState::TRAVERSE_FWD:
        if (!_stepper->isRunning()) {
            _simPassesDone++;
            if (_simPassesDone >= _simPassesTotal) {
                // Scénario terminé alors qu'on est à effW → retour rapide à 0 (non compté)
                _simNeedReturn = true;
                _stepper->setSpeedInHz(LAT_TRAVERSE_SPEED_HZ);
                _stepper->moveTo(0);
                _state = LatState::TRAVERSE_BWD;
            } else {
                _stepper->setSpeedInHz(_simSpeedHz);
                _stepper->moveTo(0);
                _state = LatState::TRAVERSE_BWD;
            }
        }
        break;

    // ── TRAVERSE_BWD ────────────────────────────────────────────────
    // Retour : effWidth → 0. Comptage de la passe sauf si retour de fin de scénario.
    case LatState::TRAVERSE_BWD:
        if (!_stepper->isRunning()) {
            if (_simNeedReturn) {
                // Retour non compté après fin de scénario
                _simNeedReturn = false;
                _finishSimScenario();
            } else {
                _simPassesDone++;
                if (_simPassesDone >= _simPassesTotal) {
                    _finishSimScenario(); // déjà en position 0
                } else {
                    _stepper->setSpeedInHz(_simSpeedHz);
                    _stepper->moveTo(_simEndSteps);
                    _state = LatState::TRAVERSE_FWD;
                }
            }
        }
        break;

    // ── SIM_PAUSE ─────────────────────────────────────────────────
    // Attente non-bloquante entre deux scénarios de scatter.
    case LatState::SIM_PAUSE:
        if (millis() - _simPauseStart >= SIM_PAUSE_BETWEEN_MS) {
            _startSimScenario(_simIdx + 1);
        }
        break;
#endif
    }
}

// ─────────────────────────────────────────────────────────────────────────────

void LateralController::rehome() {
    if (!_stepper) return;
    if (!_sensorPresent()) {
        Serial.println("[Lateral] Rehoming impossible : capteur absent.");
        return;
    }
    // Détacher l'ISR au cas où un homing précédent était en cours.
    if (_state == LatState::HOMING) _detachHomeISR();
    if (_stepper->isRunning()) {
        _stepper->forceStopAndNewPosition(_stepper->getCurrentPosition());
    }
    _disableDriver();
    if (_atHome()) _startBackoff();
    else           _startHoming();
    Serial.println("[Lateral] Rehoming lancé.");
}

const char* LateralController::stateStr() const {
    switch (_state) {
        case LatState::FAULT:           return "FAULT";
        case LatState::BACKOFF:         return "BACKOFF";
        case LatState::HOMING:          return "HOMING";
        case LatState::HOMING_DECEL:    return "HOMING_DECEL";
        case LatState::HOMING_ALIGN:    return "HOMING_ALIGN";
        case LatState::HOMING_OFFSET:   return "HOMING_OFFSET";
        case LatState::HOMED:           return "HOMED";
        case LatState::WINDING_FWD:     return "WIND_FWD";
        case LatState::WINDING_BWD:     return "WIND_BWD";
        case LatState::TRAVERSE_FWD:    return "SIM_FWD";
        case LatState::TRAVERSE_BWD:    return "SIM_BWD";
        case LatState::SIM_PAUSE:       return "SIM_PAUSE";
        default:                        return "?";
    }
}

// ── Helpers privés ────────────────────────────────────────────────────────────

void LateralController::_startHoming() {
    _attachHomeISR();   // détection immédiate via interruption GPIO
    _enableDriver();
    _stepper->setSpeedInHz(LAT_HOME_SPEED_HZ);
    if (LAT_HOME_DIR) _stepper->runForward();
    else              _stepper->runBackward();
    _state = LatState::HOMING;
}

void LateralController::_startBackoff() {
    _enableDriver();
    _stepper->setSpeedInHz(LAT_HOME_SPEED_HZ);
    // Sens opposé au homing pour s'éloigner du capteur.
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
    Preferences prefs;
    prefs.begin("lateral", false);  // read-write
    prefs.putFloat("offset_mm", _homeOffsetMm);
    prefs.end();
    Serial.printf("[Lateral] Offset home enregistré : %.2f mm\n", _homeOffsetMm);
}

void LateralController::_applyOffsetOrNext() {
    // Appelé après forceStopAndNewPosition(0) — le capteur est à la position 0.
    // Si un offset est défini, on se déplace de cet offset dans la direction
    // opposée au capteur (direction positive), puis on remet la position à 0.
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
#ifdef LAT_TEST_TRAVERSE
    _startSimScenario(0);
#else
    _state = LatState::HOMED;
    Serial.println("[Lateral] ✓ Position 0 atteinte — axe latéral prêt.");
#endif
}

#ifdef LAT_TEST_TRAVERSE
void LateralController::_startSimScenario(uint8_t idx) {
    // Géométrie Strat (BOBBIN_PRESETS[0]) avec marge par défaut 0.5 mm
    constexpr float effW_mm = 17.0f - 1.5f - 1.5f - 2.0f * 0.5f; // 13.0 mm utile
    constexpr float wire_mm = 0.071f;                              // AWG42

    const float scatters[3] = { SIM_SCATTER_LOW, SIM_SCATTER_MED, SIM_SCATTER_HIGH };
    const char* names[3]    = { "faible (1.0)", "moyen (2.0)", "eleve (3.5)" };

    _simIdx       = idx;
    float scatter = scatters[idx];
    float spacing = wire_mm * scatter;
    long  tpp     = (long)(effW_mm / spacing);
    if (tpp < 1) tpp = 1;

    _simPassesTotal = ((long)SIM_TARGET_TURNS + tpp - 1) / tpp;
    _simPassesDone  = 0;
    _simNeedReturn  = false;

    // Vitesse latérale : effWidth doit être parcouru en (tpp tours @ SIM_BOBBIN_RPM)
    // pass_duration_s = tpp / RPM * 60
    // lat_speed_Hz    = effW_steps / pass_duration_s = effW_steps * RPM / (tpp * 60)
    float effW_steps = effW_mm * (float)LAT_STEPS_PER_MM;
    float latHz      = effW_steps * (float)SIM_BOBBIN_RPM / ((float)tpp * 60.0f);
    _simSpeedHz      = (uint32_t)max(100.0f, latHz);
    _simEndSteps     = (int32_t)effW_steps;

    Serial.printf("\n[Sim] \u2550\u2550\u2550 Sc\u00e9nario %d/3 : scatter %s \u2550\u2550\u2550\n", idx + 1, names[idx]);
    Serial.printf("[Sim]   effWidth=%.1f mm  tpp=%ld tours/passe  passes=%ld\n",
                  effW_mm, tpp, _simPassesTotal);
    Serial.printf("[Sim]   vitesse lat=%u Hz (%.3f mm/s)\n",
                  _simSpeedHz, (float)_simSpeedHz / (float)LAT_STEPS_PER_MM);
    Serial.printf("[Sim]   dur\u00e9e estim\u00e9e : %.1f min\n",
                  (float)_simPassesTotal * (float)tpp / (float)SIM_BOBBIN_RPM);

    _stepper->setSpeedInHz(_simSpeedHz);
    _stepper->moveTo(_simEndSteps);
    _state = LatState::TRAVERSE_FWD;
}

void LateralController::_finishSimScenario() {
    uint8_t next = _simIdx + 1;
    if (next < 3) {
        Serial.printf("[Sim] Sc\u00e9nario %d/3 termin\u00e9. Pause %d ms avant le suivant...\n",
                      _simIdx + 1, (int)SIM_PAUSE_BETWEEN_MS);
        _simPauseStart = millis();
        _state = LatState::SIM_PAUSE;
    } else {
        _state = LatState::HOMED;
        Serial.println("[Sim] \u2713 Tous les sc\u00e9narios termin\u00e9s \u2014 axe en position 0.");
    }
}
#endif
// ── Bobinage synchronisé ──────────────────────────────────────────────────────

uint32_t LateralController::_calcLatHz(uint32_t windingHz, long tpp, float effWidthMm) const {
    if (tpp <= 0 || windingHz == 0 || effWidthMm <= 0.0f) return 0;
    float effSteps = effWidthMm * (float)LAT_STEPS_PER_MM;
    // lat_Hz = effWidth_steps × windingHz / (tpp × STEPS_PER_REV)
    float hz = effSteps * (float)windingHz / ((float)tpp * (float)STEPS_PER_REV);
    return (uint32_t)max(1.0f, hz);
}

void LateralController::_onReversal() {
    // Durée de la fenêtre : 2 × v/a secondes (équivalent à la décél + accél du latéral).
    uint32_t ms = (_latHz > 0)
        ? (uint32_t)(2000.0f * (float)_latHz / (float)LAT_ACCEL)
        : 200;
    _reversingUntilMs = millis() + max(100u, min(600u, ms));
}

void LateralController::startWinding(uint32_t windingHz, long tpp, float effWidthMm) {
    // Si déjà en mouvement de bobinage, mettre à jour vitesse + fin de course uniquement.
    if (_state == LatState::WINDING_FWD || _state == LatState::WINDING_BWD) {
        updateWinding(windingHz, tpp, effWidthMm);
        return;
    }
    // Sinon : ne démarre que depuis l'état HOMED (après homing complet).
    if (_state != LatState::HOMED) {
        Serial.printf("[Lateral] startWinding ignoré — état : %s\n", stateStr());
        return;
    }
    if (windingHz == 0 || tpp <= 0 || effWidthMm <= 0.0f) {
        Serial.printf("[Lateral] startWinding ignoré — paramètres invalides : windHz=%u tpp=%ld eff=%.2f\n",
                      windingHz, (long)tpp, effWidthMm);
        return;
    }

    _enableDriver();  // Sécurité : s'assurer que le driver est actif
    _latEndSteps = (int32_t)(effWidthMm * (float)LAT_STEPS_PER_MM);
    _latHz = _calcLatHz(windingHz, tpp, effWidthMm);
    if (_latHz < 1) _latHz = 1;

    _stepper->setSpeedInHz(_latHz);
    _stepper->applySpeedAcceleration();
    // Choisir le sens en fonction de la position courante :
    // première moitié de course → FWD, deuxième moitié → BWD.
    int32_t pos = _stepper->getCurrentPosition();
    if (pos < _latEndSteps / 2) {
        _stepper->runForward();
        _state = LatState::WINDING_FWD;
    } else {
        _stepper->runBackward();
        _state = LatState::WINDING_BWD;
    }
    float mmPerSec = (float)_latHz / (float)LAT_STEPS_PER_MM;
    float passDuration = (mmPerSec > 0.0f) ? (effWidthMm / mmPerSec) : 0.0f;
    Serial.printf("[Lateral] ▶ Bobinage démarré : %u Hz (%.2f mm/s)  passe=%.1fs  dir=%s\n",
                  _latHz, mmPerSec, passDuration,
                  (_state == LatState::WINDING_FWD) ? "FWD" : "BWD");
}

void LateralController::updateWinding(uint32_t windingHz, long tpp, float effWidthMm) {
    if (_state != LatState::WINDING_FWD && _state != LatState::WINDING_BWD) return;

    // Mettre à jour la position fin de course (peut changer si la géométrie est modifiée).
    _latEndSteps = (int32_t)(effWidthMm * (float)LAT_STEPS_PER_MM);

    uint32_t newHz = _calcLatHz(windingHz, tpp, effWidthMm);
    if (newHz < 1) newHz = 1;
    if (newHz != _latHz) {
        _latHz = newHz;
        _stepper->setSpeedInHz(_latHz);
        _stepper->applySpeedAcceleration();
    }
}

void LateralController::stopWinding() {
    if (_state != LatState::WINDING_FWD && _state != LatState::WINDING_BWD) return;
    if (_stepper->isRunning()) _stepper->stopMove();
    _state = LatState::HOMED;
    _reversingUntilMs = 0;
    Serial.println("[Lateral] Bobinage arrêté.");
}
// ── ISR GPIO — détection instantanée du capteur de home ──────────────────────
// Appelée sur le front descendant de HOME_PIN_NO (INPUT_PULLUP, actif à GND).
// On vérifie _state == HOMING pour éviter les déclenchements parasites
// pendant BACKOFF ou HOMED, et _homeFlag pour ignorer les rebonds.
void IRAM_ATTR LateralController::_homePinISR(void* arg) {
    LateralController* self = static_cast<LateralController*>(arg);
    if (self->_state == LatState::HOMING && !self->_homeFlag) {
        self->_homeFlag = true;
    }
}

void LateralController::_attachHomeISR() {
    _homeFlag = false;
    attachInterruptArg(digitalPinToInterrupt(HOME_PIN_NO),
                       _homePinISR, this, FALLING);
}

void LateralController::_detachHomeISR() {
    detachInterrupt(digitalPinToInterrupt(HOME_PIN_NO));
}
