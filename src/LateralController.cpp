#include "LateralController.h"
#include <Arduino.h>

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
#ifdef LAT_TEST_TRAVERSE
                _startTraverseFwd();
#else
                _state = LatState::HOMED;
                Serial.println("[Lateral] ✓ Position initiale atteinte — axe latéral prêt.");
#endif
            }
        }
        break;

    // ── HOMING_ALIGN ────────────────────────────────────────────────────
    case LatState::HOMING_ALIGN:
        if (!_stepper->isRunning()) {
            _stepper->forceStopAndNewPosition(0);
#ifdef LAT_TEST_TRAVERSE
            _startTraverseFwd();
#else
            _state = LatState::HOMED;
            Serial.println("[Lateral] ✓ Position initiale atteinte — axe latéral prêt.");
#endif
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

#ifdef LAT_TEST_TRAVERSE
    // ── TRAVERSE_FWD ───────────────────────────────────────────────────
    case LatState::TRAVERSE_FWD:
        if (!_stepper->isRunning()) {
            // Position max atteinte → retour vers 0 (home)
            _stepper->setSpeedInHz(LAT_TRAVERSE_SPEED_HZ);
            _stepper->moveTo(0);
            _state = LatState::TRAVERSE_BWD;
            Serial.println("[Lateral] Traverse: retour home...");
        }
        break;

    // ── TRAVERSE_BWD ───────────────────────────────────────────────────
    case LatState::TRAVERSE_BWD:
        if (!_stepper->isRunning()) {
            // Home atteint → repartir en avant
            _startTraverseFwd();
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
        case LatState::FAULT:         return "FAULT";
        case LatState::BACKOFF:       return "BACKOFF";
        case LatState::HOMING:        return "HOMING";
        case LatState::HOMING_DECEL:  return "HOMING_DECEL";
        case LatState::HOMING_ALIGN:  return "HOMING_ALIGN";
        case LatState::HOMED:         return "HOMED";
        case LatState::TRAVERSE_FWD:  return "TRAVERSE_FWD";
        case LatState::TRAVERSE_BWD:  return "TRAVERSE_BWD";
        default:                      return "?";
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

void LateralController::_startTraverseFwd() {
    constexpr int32_t target = (int32_t)LAT_TRAVERSE_MM * (int32_t)LAT_STEPS_PER_MM;
    _stepper->setSpeedInHz(LAT_TRAVERSE_SPEED_HZ);
    _stepper->moveTo(target);
    _state = LatState::TRAVERSE_FWD;
    Serial.printf("[Lateral] Traverse: aller → %d mm (%ld steps)...\n",
                  (int)LAT_TRAVERSE_MM, (long)target);
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
