#pragma once
#include <Arduino.h>
#include "Config.h"

// ── LEDController ─────────────────────────────────────────────────────────────
// Drives a single LED (LED_PIN) that acts as a manual traverse guide.
// The LED flashes briefly at the start of each new pass (aller / retour),
// telling the operator when to reverse the wire guide direction.
// The flash lasts LED_FLASH_MS milliseconds, then the LED turns off
// automatically — giving a clear, visible blink at every direction change.
class LEDController {
public:
    // Configure LED_PIN as output and ensure the LED is off.
    void begin();

    // Call this every loop iteration with the latest turn count.
    // Computes the current pass number (currentTurns / turnsPerPass) and
    // flashes the LED whenever the pass number increments.
    // Also handles auto-off after LED_FLASH_MS.
    // Returns the current pass number (0-based).
    int  update(long currentTurns, long turnsPerPass, bool motorRunning);

    // Turn the LED off and reset the pass counter to -1.
    void reset();

    // Returns the most recently computed pass number.
    int  getCurrentPass() const { return _currentPass; }

private:
    int           _currentPass   = -1;    // -1 = not started / reset
    bool          _ledOn         = false;  // Is the LED currently lit?
    bool          _needsSync     = true;   // True = resync pass on next update (no flash)
    unsigned long _flashStartMs  = 0;      // millis() when flash started
};
