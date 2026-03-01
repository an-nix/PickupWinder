#pragma once
#include <Arduino.h>
#include "Config.h"

// ── LEDController ─────────────────────────────────────────────────────────────
// Drives a single LED (LED_PIN) that acts as a manual traverse guide.
// The LED toggles state at the start of each new pass (aller / retour),
// telling the operator when to reverse the wire guide direction.
class LEDController {
public:
    // Configure LED_PIN as output and ensure the LED is off.
    void begin();

    // Call this every loop iteration with the latest turn count.
    // Computes the current pass number (currentTurns / turnsPerPass) and
    // toggles the LED whenever the pass number increments.
    // Resets automatically if the motor is stopped or turnsPerPass is invalid.
    // Returns the current pass number (0-based).
    int  update(long currentTurns, long turnsPerPass, bool motorRunning);

    // Turn the LED off and reset the pass counter to -1.
    void reset();

    // Returns the most recently computed pass number.
    int  getCurrentPass() const { return _currentPass; }

private:
    int  _currentPass = -1;   // -1 = not started / reset
    bool _ledState    = false; // Current LED output state (false = off)
};
