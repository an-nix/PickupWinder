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
    /**
     * @brief Initialize LED GPIO and force LED off.
     */
    void begin();

    /**
     * @brief Update LED guide state from winding progress.
     *
     * Computes current pass index and flashes on pass transitions.
     * Also handles timed auto-off.
     *
     * @param currentTurns Current turns count.
     * @param turnsPerPass Active turns-per-pass value.
     * @param motorRunning true if spindle is currently running.
     * @return Current zero-based pass index.
     */
    int  update(long currentTurns, long turnsPerPass, bool motorRunning);

    /**
     * @brief Turn LED off and reset pass tracking.
     */
    void reset();

    /**
     * @brief Get last computed pass index.
     * @return Last pass index, or -1 if not started.
     */
    int  getCurrentPass() const { return _currentPass; }

private:
    int           _currentPass   = -1;    // -1 = not started / reset
    bool          _ledOn         = false;  // Is the LED currently lit?
    bool          _needsSync     = true;   // True = resync pass on next update (no flash)
    unsigned long _flashStartMs  = 0;      // millis() when flash started
};
