#include <Arduino.h>
#include "WinderApp.h"

WinderApp app;

void setup() {
    app.begin();
}

void loop() {
    app.run();
}