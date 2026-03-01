#include "LEDController.h"

void LEDController::begin() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // Start with LED off
    Serial.println("[LED] OK");
}

int LEDController::update(long currentTurns, long turnsPerPass, bool motorRunning) {
    // ── Auto-off: turn LED off once flash duration has elapsed ──
    if (_ledOn && (millis() - _flashStartMs >= LED_FLASH_MS)) {
        _ledOn = false;
        digitalWrite(LED_PIN, LOW);
    }

    // If the motor is stopped or geometry is invalid, reset.
    if (!motorRunning || turnsPerPass <= 0) {
        reset();
        return 0;
    }

    // Compute which pass we are on (integer division).
    // Each increment of `pass` means the wire has traversed the full bobbin width.
    int pass = (int)(currentTurns / turnsPerPass);

    if (pass != _currentPass) {
        // Pass number changed → flash the LED to signal the operator to
        // reverse the wire guide direction.
        _currentPass  = pass;
        _ledOn        = true;
        _flashStartMs = millis();
        digitalWrite(LED_PIN, HIGH);
        Serial.printf("[LED] Pass %d — direction change!\n", pass);
    }

    return _currentPass;
}

void LEDController::reset() {
    // Return to the initial state: LED off, pass counter cleared.
    _currentPass  = -1;
    _ledOn        = false;
    _flashStartMs = 0;
    digitalWrite(LED_PIN, LOW);
}
