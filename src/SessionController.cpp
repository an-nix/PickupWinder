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

    // Debug tracing for last-event-wins arbitration.
    const char* intentName = "None";
    const char* srcName = "None";
    switch (intent) {
        case ControlIntent::Start: intentName = "Start"; break;
        case ControlIntent::Pause: intentName = "Pause"; break;
        case ControlIntent::Stop: intentName = "Stop"; break;
        default: break;
    }
    switch (src) {
        case InputSource::Pot: srcName = "Pot"; break;
        case InputSource::IHM: srcName = "IHM"; break;
        case InputSource::Footswitch: srcName = "Footswitch"; break;
        default: break;
    }
    Serial.printf("[Session] recordIntent intent=%s src=%s potAboveZero=%.3f potNeedsRearm=%d\n",
                  intentName, srcName, _potLevel, _potNeedsRearm);

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
        _winder.stopWinding();      // Delegate full stop to domain controller.
        _sessionState = SessionState::IDLE;
        _appliedSource = src;
        _runMode = RunMode::None;
        break;

    case ControlIntent::Pause:
        if (_sessionState != SessionState::PAUSED) {
            _winder.pauseWinding();
            _sessionState = SessionState::PAUSED;
        }
        _appliedSource = src;
        _runMode = RunMode::None;
        break;

    case ControlIntent::Start:
        // Always forward explicit Start intents to WinderApp so UI/footswitch can
        // resume from PAUSED even if the session is already marked ARMED_OR_RUNNING.
        _winder.handleCommand("start", "");
        _sessionState = SessionState::ARMED_OR_RUNNING;
        _appliedSource = src;
        if (src == InputSource::Pot) {
            _potNeedsRearm = false;
            _runMode = RunMode::Pot;
        } else {
            // Start via UI/footswitch re-arms pot so pot can immediately take over
            // if the user moves it after pressing Start.
            _potNeedsRearm = false;
            _runMode = RunMode::Max;
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
        uint32_t commandedSpeedHz;
        switch (_runMode) {
        case RunMode::Pot:
            commandedSpeedHz = (uint32_t)(_potLevel * (float)_winder.getMaxSpeedHz());
            break;
        case RunMode::Max:
            commandedSpeedHz = _winder.getMaxSpeedHz();
            break;
        default:
            commandedSpeedHz = 0;
            break;
        }

        _winder.setControlHz(commandedSpeedHz);
        Serial.printf("[Session] ARMED_OR_RUNNING speedHz=%u pot=%.3f source=%d runMode=%d\n",
            commandedSpeedHz, _potLevel, (int)_lastEventSource, (int)_runMode);
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
            const bool levelChanged = (fabsf(newLevel - _potLevel) >= 0.001f);
            if (levelChanged) {
                _potLevel = newLevel;
            }

            const bool potAboveZero = (_potLevel > POT_RUN_THRESHOLD);
            if (!_hasPotSample) {
                _hasPotSample = true;
                _potAboveZero = potAboveZero;
            } else if (potAboveZero != _potAboveZero) {
                _potAboveZero = potAboveZero;

                if (!potAboveZero) {
                    _potNeedsRearm = false;
                    recordIntent(ControlIntent::Pause, InputSource::Pot);
                } else if (!_potNeedsRearm) {
                    recordIntent(ControlIntent::Start, InputSource::Pot);
                }
            }

            if (_sessionState == SessionState::ARMED_OR_RUNNING && potAboveZero) {
                // Pot changes override previous source once in run state.
                _runMode = RunMode::Pot;
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
