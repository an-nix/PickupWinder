#include "LEDController.h"

void LEDController::begin() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // Start with LED off
    Serial.println("[LED] OK");
}

int LEDController::update(long currentTurns, long turnsPerPass, bool motorRunning) {
    // If the motor is stopped or geometry is invalid, turn off the LED and reset.
    if (!motorRunning || turnsPerPass <= 0) {
        reset();
        return 0;
    }

    // Compute which pass we are on (integer division).
    // Each increment of `pass` means the wire has traversed the full bobbin width.
    int pass = (int)(currentTurns / turnsPerPass);

    if (pass != _currentPass) {
        // Pass number changed → toggle the LED to signal the operator to
        // reverse the wire guide direction.
        _currentPass = pass;
        _ledState    = !_ledState;
        digitalWrite(LED_PIN, _ledState ? HIGH : LOW);
        Serial.printf("[LED] Pass %d — guide %s\n", pass, _ledState ? "→" : "←");
    }

    return _currentPass;
}

void LEDController::reset() {
    // Return to the initial state: LED off, pass counter cleared.
    _currentPass = -1;
    _ledState    = false;
    digitalWrite(LED_PIN, LOW);
}
