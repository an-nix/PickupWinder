#include "LEDController.h"

void LEDController::begin() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.println("[LED] OK");
}

int LEDController::update(long currentTurns, long turnsPerPass, bool motorRunning) {
    if (!motorRunning || turnsPerPass <= 0) {
        reset();
        return 0;
    }

    int pass = (int)(currentTurns / turnsPerPass);

    if (pass != _currentPass) {
        _currentPass = pass;
        _ledState    = !_ledState;
        digitalWrite(LED_PIN, _ledState ? HIGH : LOW);
        Serial.printf("[LED] Passage %d — guide %s\n", pass, _ledState ? "→" : "←");
    }

    return _currentPass;
}

void LEDController::reset() {
    _currentPass = -1;
    _ledState    = false;
    digitalWrite(LED_PIN, LOW);
}
