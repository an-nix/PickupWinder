#include "SessionController.h"
#include "Diag.h"
#include "Types.h"

namespace {
// Pot is considered active above this normalized threshold.
// This is intentionally tiny: we only need robust zero/non-zero edge detection.
constexpr float POT_RUN_THRESHOLD = 0.001f;
}

// -----------------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------------
SessionController::SessionController(WinderApp& winder)
    : _winder(winder)
{
    // No runtime initialization required here.
}

// -----------------------------------------------------------------------------
// Intent arbitration
// -----------------------------------------------------------------------------
void SessionController::recordIntent(ControlIntent intent, InputSource src) {
    // Last event always overrides any previous pending intent.
    _pendingControlIntent = intent; // Store the latest intent to apply.
    _pendingIntentSource = src;     // Remember who produced that intent.
    _lastEventSource = src;         // Keep latest observed input source for diagnostics.

    // If another source takes control while pot is non-zero, require a pot
    // return-to-zero before pot can start the machine again.
    if (src != InputSource::Pot && _potAboveZero) {
        _potNeedsRearm = true; // Arm pot lock until next explicit zero crossing.
    }
}

void SessionController::applyPendingIntent() {
    if (_pendingControlIntent == ControlIntent::None) return; // Nothing to apply this tick.

    // Snapshot source to preserve traceability after pending fields are cleared.
    const InputSource src = _pendingIntentSource; // Local copy for state attribution.

    switch (_pendingControlIntent) {
    case ControlIntent::Stop:
        // Hard session stop: return to IDLE and stop winding domain.
        _winder.stopWinding();      // Delegate full stop to domain controller.
        _sessionState = SessionState::IDLE; // Reflect stop at session layer.
        _appliedSource = src;              // Mark source that produced effective state.
        break;

    case ControlIntent::Pause:
        // Pause is idempotent; avoid duplicate pause commands.
        if (_sessionState != SessionState::PAUSED) {
            _winder.pauseWinding();      // Request domain pause only on transition.
            _sessionState = SessionState::PAUSED; // Persist paused session state.
        }
        _appliedSource = src; // Mark effective source even if already paused.
        break;

    case ControlIntent::Start:
        // Start is also idempotent; only trigger transition when needed.
        if (_sessionState != SessionState::ARMED_OR_RUNNING) {
            _winder.handleCommand("start", ""); // Reuse start path in WinderApp.
            _sessionState = SessionState::ARMED_OR_RUNNING; // Persist armed/running session state.
        }
        _appliedSource = src; // Mark effective source even if already running.
        if (src == InputSource::Pot) {
            // Pot regained control explicitly.
            _potNeedsRearm = false; // Clear lock because pot intentionally restarted.
        }
        break;

    case ControlIntent::None:
        break; // Defensive branch, should be filtered by early return.
    }

    _pendingControlIntent = ControlIntent::None; // Clear one-shot intent after application.
    _pendingIntentSource = InputSource::None;    // Clear source paired with consumed intent.
}

// -----------------------------------------------------------------------------
// Public requests (UI / command side)
// -----------------------------------------------------------------------------
void SessionController::requestStart() {
    recordIntent(ControlIntent::Start, InputSource::IHM); // UI requests start.
}

void SessionController::requestPause() {
    recordIntent(ControlIntent::Pause, InputSource::IHM); // UI requests pause.
}

void SessionController::requestStop() {
    recordIntent(ControlIntent::Stop, InputSource::IHM); // UI requests stop.
}

// -----------------------------------------------------------------------------
// Command decoding
// -----------------------------------------------------------------------------
bool SessionController::handleCommand(const String& cmd, const String& value) {
    // Session lifecycle commands are converted to intents.
    if (cmd == "start") {
        requestStart(); // Convert transport command into start intent.
        return true;    // Mark as consumed by SessionController.
    }
    if (cmd == "pause") {
        requestPause(); // Convert transport command into pause intent.
        return true;    // Mark as consumed by SessionController.
    }
    if (cmd == "stop") {
        requestStop(); // Convert transport command into stop intent.
        return true;   // Mark as consumed by SessionController.
    }
    if (cmd == "target") {
        long t = value.toInt();         // Parse new target turns.
        if (t > 0) _winder.setTargetTurns(t); // Apply only valid positive targets.
        return true;                    // Command handled at session/domain boundary.
    }
    if (cmd == "freerun") {
        _winder.setFreerun(value == "true"); // Map text flag to boolean mode.
        return true;                           // Command consumed.
    }
    if (cmd == "direction") {
        _winder.setDirection(value == "cw" ? Direction::CW : Direction::CCW); // Select spindle direction.
        return true; // Command consumed.
    }
    if (cmd == "max_rpm" || cmd == "max-rpm") {
        _winder.setMaxRpm((uint16_t)constrain(value.toInt(), 10, 1500)); // Clamp safe operating range.
        return true; // Command consumed.
    }

    // Unknown command for SessionController: caller may forward it.
    return false;
}

// -----------------------------------------------------------------------------
// Output application
// -----------------------------------------------------------------------------
void SessionController::applyPower() 
{
    if (_sessionState == SessionState::ARMED_OR_RUNNING) 
    {
        // ARMED_OR_RUNNING means the session is ready for or actively running motion.
        // Always use the same logic for both fresh start and resume:
        // - If the start came from the UI or footswitch while the pot is still at zero,
        //   the commanded speed remains zero here, so the spindle will not move until the pot rises.
        // - WinderApp will be positioned/armed, but not spinning.
        uint32_t commandedSpeedHz = (uint32_t)(_potLevel * (float)_winder.getMaxSpeedHz()); // Scale max speed by normalized pot.
        _winder.setControlHz(commandedSpeedHz); // Push speed command to winding domain.
        Serial.printf("[Session] ARMED_OR_RUNNING speedHz=%u pot=%.3f source=%d\n", commandedSpeedHz, _potLevel, (int)_lastEventSource); // Debug trace.
    }
    else
    {
        // Any non-armed state forces zero speed command.
        _winder.setControlHz(0); // Guarantee no speed command outside ARMED_OR_RUNNING.
        Serial.printf("[Session] paused source=%d pot=%.3f\n", (int)_lastEventSource, _potLevel); // Debug trace.
    }
}

// -----------------------------------------------------------------------------
// Tick update (single deterministic update step)
// -----------------------------------------------------------------------------
void SessionController::tick(const TickInput& in) {
    // 0) Handle encoder delta (if any) for paused-position trim.
    if (in.encoderDelta != 0) {
        _winder.handleEncoderDelta(in.encoderDelta);
    }

    // 1) Integrate pot input and detect zero/non-zero edges.
    if (in.hasPot) {
        float newLevel = constrain(in.potLevel, 0.0f, 1.0f); // Keep pot value normalized.
        if (fabsf(newLevel - _potLevel) >= 0.001f) {
            _potLevel = newLevel; // Update cached pot level when change is meaningful.
        }

        const bool potAboveZero = (_potLevel > POT_RUN_THRESHOLD); // Digital state derived from analog level.
        if (!_hasPotSample) {
            _hasPotSample = true;       // Initialize edge detector on first valid sample.
            _potAboveZero = potAboveZero; // Store baseline digital pot state.
        } else if (potAboveZero != _potAboveZero) {
            // Pot transitions are the only moments allowed to trigger
            // start/pause intents from analog input.
            _potAboveZero = potAboveZero; // Latch the new pot digital state.

            if (!potAboveZero) {
                // Pot reached zero: clear ownership lock and request pause.
                _potNeedsRearm = false; // Pot is now rearmed for future takeovers.
                recordIntent(ControlIntent::Pause, InputSource::Pot); // Pot falling edge requests pause.
            } else if (!_potNeedsRearm) {
                // Pot rising edge above zero can start only when rearmed.
                recordIntent(ControlIntent::Start, InputSource::Pot); // Pot rising edge requests start.
            }
        }
    }

    // 2) Integrate footswitch edge events.
    if (in.hasFootswitch) {
        if (!_hasFootswitchSample || _footswitch != in.footswitch) {
            _hasFootswitchSample = true; // Baseline becomes valid after first sample.
            _footswitch = in.footswitch; // Store latest footswitch state.
            recordIntent(_footswitch ? ControlIntent::Start : ControlIntent::Pause,
                InputSource::Footswitch); // Press->start, release->pause.
        }
    }

    // 3) Decode commands provided in TickInput (bounded, no dynamic alloc).
    for (int i = 0; i < in.cmdCount; ++i) {
        const char* c = in.commands[i].cmd; // Raw command key from queue.
        const char* v = in.commands[i].val; // Raw command value from queue.
        String sc(c); // Build String view used by existing command handlers.
        String sv(v); // Build String view used by existing command handlers.
        // Session-level commands handled here; others forwarded to WinderApp.
        if (!handleCommand(sc, sv)) {
            _winder.handleCommand(sc, sv); // Forward unknown session command.
        }
    }

    // 4) Apply exactly one resolved intent (last event wins).
    applyPendingIntent(); // Commit the most recent pending intent.

    // 5) Push output command and let winding domain advance one step.
    applyPower();  // Update spindle speed command for this cycle.
    _winder.tick(); // Advance domain state machine and motor/lateral logic.
}
