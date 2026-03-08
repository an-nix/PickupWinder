#pragma once
#include "Config.h"  // doit être inclus avant #ifdef TEST_MOTORS
#ifdef TEST_MOTORS

#include <Arduino.h>

// ── MotorTest ─────────────────────────────────────────────────────────────────
// Mode test de câblage : fait tourner les deux steppers séquentiellement.
// Séquence répétée en boucle :
//   1. Stepper bobine  CW  pendant TEST_RUN_MS
//   2. Pause TEST_PAUSE_MS
//   3. Stepper bobine  CCW pendant TEST_RUN_MS
//   4. Pause TEST_PAUSE_MS
//   5. Stepper latéral CW  pendant TEST_RUN_MS
//   6. Pause TEST_PAUSE_MS
//   7. Stepper latéral CCW pendant TEST_RUN_MS
//   8. Pause TEST_PAUSE_MS
//   9. Les deux simultanément CW pendant TEST_RUN_MS
//  10. Pause TEST_PAUSE_MS  → retour à l'étape 1
//
// Activé uniquement si TEST_MOTORS est défini (Config.h ou build_flags).
class MotorTest {
public:
    void begin();
    void run();   // Appeler depuis loop()

private:
    enum class Step {
        MAIN_CW, PAUSE1,
        MAIN_CCW, PAUSE2,
        LAT_CW,  PAUSE3,
        LAT_CCW, PAUSE4,
        BOTH_CW, PAUSE5
    };

    Step     _step      = Step::MAIN_CW;
    uint32_t _stepStart = 0;

    void _startStep();
    void _stopAll();
    const char* _stepName() const;
};

#endif // TEST_MOTORS
