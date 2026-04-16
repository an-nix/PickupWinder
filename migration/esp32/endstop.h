/* endstop.h — End stop ISR and homing logic.
 *
 * Supports 2-contact endstop sensors (NO + NC contacts) as used in the
 * lateral axis home sensor:
 *   - NO (Normally Open):    LOW when triggered (closed to GND), pull-up.
 *   - NC (Normally Closed):  HIGH when triggered (open circuit), pull-up.
 *   - Valid home:   NO=LOW AND NC=HIGH
 *   - Sensor fault: NO=LOW AND NC=LOW (wiring break / disconnected)
 *
 * The GPIO ISR fires on the falling edge of the NO pin (active = LOW) and
 * immediately pauses the corresponding axis timer — no software delay.
 * The NC contact is polled by the stepper task for home/fault discrimination.
 */

#pragma once

#include <cstdint>
#include "stepper_engine.h"

/// Configure endstop GPIO interrupts for safety stop.
/// Must be called after axes are initialized.
/// Maps endstop pins to their corresponding axis for ISR dispatch.
void endstop_init(const AxisPins pins[], uint8_t num_axes);

/// Homing sequence for one axis.
/// Blocking call: moves axis toward endstop at low speed,
/// waits for endstop trigger, then backs off and resets position.
///
/// Returns true if home was successful, false on timeout.
///
/// This runs on Core 0 (command context), NOT in ISR.
/// The actual stepping is still done by Core 1 timers.
bool endstop_home_axis(uint8_t axis_id, uint32_t approach_hz = 2000,
                        uint32_t backoff_steps = 200,
                        uint32_t timeout_ms = 30000);
