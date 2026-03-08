#include "Config.h"  // doit être inclus avant #ifdef TEST_MOTORS
#include "MotorTest.h"
#ifdef TEST_MOTORS

#include <FastAccelStepper.h>

// ── Instances FastAccelStepper (locales au test) ──────────────────────────────
static FastAccelStepperEngine engine;
static FastAccelStepper* stepMain = nullptr;
static FastAccelStepper* stepLat  = nullptr;

// Conversion RPM → Hz pour FastAccelStepper
static inline uint32_t rpmToHz(uint32_t rpm) {
    return rpm * STEPS_PER_REV / 60;
}

void MotorTest::begin() {
    Serial.begin(115200);
    Serial.println("\n=== MODE TEST MOTEURS ===");
    Serial.println("Séquence : bobine CW → CCW → latéral CW → CCW → les deux CW");
    Serial.printf("Vitesses : bobine %u RPM, latéral %u RPM\n",
                  TEST_RPM_MAIN, TEST_RPM_LAT);

    // ── Configurer les pins ENABLE en sortie, driver OFF ──────────────────
    digitalWrite(ENABLE_PIN,     HIGH);  // pré-set HIGH avant pinMode
    digitalWrite(ENABLE_PIN_LAT, HIGH);
    pinMode(ENABLE_PIN,     OUTPUT);
    pinMode(ENABLE_PIN_LAT, OUTPUT);

    // ── Initialiser FastAccelStepper ──────────────────────────────────────
    engine.init();

    stepMain = engine.stepperConnectToPin(STEP_PIN);
    if (!stepMain) { Serial.println("[Test] ERREUR : stepper bobine introuvable!"); while(true); }
    stepMain->setDirectionPin(DIR_PIN);
    stepMain->setAcceleration(ACCELERATION);
    stepMain->setSpeedInHz(rpmToHz(TEST_RPM_MAIN));

    stepLat = engine.stepperConnectToPin(STEP_PIN_LAT);
    if (!stepLat) { Serial.println("[Test] ERREUR : stepper latéral introuvable!"); while(true); }
    stepLat->setDirectionPin(DIR_PIN_LAT);
    stepLat->setAcceleration(TEST_ACCEL_LAT);  // accélération douce pour l'axe latéral
    stepLat->setSpeedInHz(rpmToHz(TEST_RPM_LAT));

    _stepStart = millis();
    _startStep();
}

void MotorTest::run() {
    uint32_t now = millis();
    uint32_t elapsed = now - _stepStart;

    // Durée de la phase courante
    bool isRunning = (_step != Step::PAUSE1 && _step != Step::PAUSE2 &&
                      _step != Step::PAUSE3 && _step != Step::PAUSE4 &&
                      _step != Step::PAUSE5);
    uint32_t phaseDuration = isRunning ? TEST_RUN_MS : TEST_PAUSE_MS;

    if (elapsed >= phaseDuration) {
        _stopAll();
        // Avancer à la phase suivante
        switch (_step) {
            case Step::MAIN_CW:  _step = Step::PAUSE1;  break;
            case Step::PAUSE1:   _step = Step::MAIN_CCW; break;
            case Step::MAIN_CCW: _step = Step::PAUSE2;  break;
            case Step::PAUSE2:   _step = Step::LAT_CW;  break;
            case Step::LAT_CW:   _step = Step::PAUSE3;  break;
            case Step::PAUSE3:   _step = Step::LAT_CCW; break;
            case Step::LAT_CCW:  _step = Step::PAUSE4;  break;
            case Step::PAUSE4:   _step = Step::BOTH_CW; break;
            case Step::BOTH_CW:  _step = Step::PAUSE5;  break;
            case Step::PAUSE5:   _step = Step::MAIN_CW; break;  // boucle
        }
        _stepStart = now;
        _startStep();
    }
}

void MotorTest::_startStep() {
    Serial.printf("[Test] ▶ %s\n", _stepName());

    switch (_step) {
        case Step::MAIN_CW:
            digitalWrite(ENABLE_PIN, LOW);
            stepMain->setSpeedInHz(rpmToHz(TEST_RPM_MAIN));
            stepMain->runForward();
            break;

        case Step::MAIN_CCW:
            digitalWrite(ENABLE_PIN, LOW);
            stepMain->setSpeedInHz(rpmToHz(TEST_RPM_MAIN));
            stepMain->runBackward();
            break;

        case Step::LAT_CW:
            digitalWrite(ENABLE_PIN_LAT, LOW);
            stepLat->setSpeedInHz(rpmToHz(TEST_RPM_LAT));
            stepLat->runForward();
            break;

        case Step::LAT_CCW:
            digitalWrite(ENABLE_PIN_LAT, LOW);
            stepLat->setSpeedInHz(rpmToHz(TEST_RPM_LAT));
            stepLat->runBackward();
            break;

        case Step::BOTH_CW:
            digitalWrite(ENABLE_PIN,     LOW);
            digitalWrite(ENABLE_PIN_LAT, LOW);
            stepMain->setSpeedInHz(rpmToHz(TEST_RPM_MAIN));
            stepLat->setSpeedInHz(rpmToHz(TEST_RPM_LAT));
            stepMain->runForward();
            stepLat->runForward();
            break;

        default:
            // Phases de pause : rien à démarrer
            break;
    }
}

void MotorTest::_stopAll() {
    if (stepMain && stepMain->isRunning()) stepMain->forceStopAndNewPosition(0);
    if (stepLat  && stepLat->isRunning())  stepLat->forceStopAndNewPosition(0);
    digitalWrite(ENABLE_PIN,     HIGH);
    digitalWrite(ENABLE_PIN_LAT, HIGH);
}

const char* MotorTest::_stepName() const {
    switch (_step) {
        case Step::MAIN_CW:  return "Bobine CW";
        case Step::PAUSE1:   return "Pause";
        case Step::MAIN_CCW: return "Bobine CCW";
        case Step::PAUSE2:   return "Pause";
        case Step::LAT_CW:   return "Latéral CW";
        case Step::PAUSE3:   return "Pause";
        case Step::LAT_CCW:  return "Latéral CCW";
        case Step::PAUSE4:   return "Pause";
        case Step::BOTH_CW:  return "Bobine + Latéral CW simultanés";
        case Step::PAUSE5:   return "Pause";
        default:             return "?";
    }
}

#endif // TEST_MOTORS
