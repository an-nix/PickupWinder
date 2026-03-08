#include <Arduino.h>
#include "Config.h"

#ifdef TEST_MOTORS
// ── Mode test câblage moteurs ─────────────────────────────
// Activé par #define TEST_MOTORS dans Config.h ou -DTEST_MOTORS dans build_flags.
// L'application normale (WinderApp) n'est PAS démarrée.
#include "MotorTest.h"
MotorTest test;
void setup() { test.begin(); }
void loop()  { test.run();   }

#else
// ── Application normale ───────────────────────────────────
#include "WinderApp.h"
WinderApp app;
void setup() { app.begin(); }
void loop()  { app.run();   }

#endif
