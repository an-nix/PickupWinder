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
    // If we're IDLE, only accept an explicit Start from the UI. Reject any
    // Start intents originating from Pot or Footswitch to avoid accidental
    // validation while idle.
    if (intent == ControlIntent::Start && _sessionState == SessionState::IDLE
        && src != InputSource::IHM) {
#if DIAG_VERBOSE
        Diag::infof("[Session] recordIntent: rejected non-UI Start while IDLE (src=%d)", (int)src);
#endif
        return;
    }

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
    // Log all intent observations for debugging the start/arm flow.
#if DIAG_VERBOSE
    Diag::infof("[Session] recordIntent observed intent=%s src=%s pot=%.3f potNeedsRearm=%d session=%d",
                intentName, srcName, _potLevel, _potNeedsRearm, (int)_sessionState);
#endif

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
        // If we're currently IDLE, require an explicit UI start: ignore Start
        // intents coming from Pot or Footswitch to avoid accidental validation
        // when the user is only adjusting controls.
        if (_sessionState == SessionState::IDLE && src != InputSource::IHM) {
#if DIAG_VERBOSE
            Diag::infof("[Session] Ignoring non-UI Start while IDLE (src=%d)", (int)src);
#endif
            break; // Do not transition out of IDLE
        }

        // Diagnostic: log intent application for tracing race conditions.
    #if DIAG_VERBOSE
        Diag::infof("[Session] applyPendingIntent applying Start src=%d sessionBefore=%d",
            (int)src, (int)_sessionState);
    #endif

        // Forward Start intents to WinderApp so UI/footswitch can resume from
        // PAUSED or re-arm the session when appropriate.
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
bool SessionController::handleCommand(const char* cmd, const char* value) {
    // Session lifecycle commands are converted to intents.
    if (strcmp(cmd, "start") == 0) {
        requestStart(); // Convert transport command into start intent.
        return true;    // Mark as consumed by SessionController.
    }
    if (strcmp(cmd, "pause") == 0) {
        requestPause(); // Convert transport command into pause intent.
        return true;    // Mark as consumed by SessionController.
    }
    if (strcmp(cmd, "stop") == 0) {
        requestStop(); // Convert transport command into stop intent.
        return true;   // Mark as consumed by SessionController.
    }
    if (strcmp(cmd, "target") == 0) {
        long t = strtol(value, nullptr, 10);         // Parse new target turns.
        if (t > 0) _winder.setTargetTurns(t); // Apply only valid positive targets.
        return true;                    // Command handled at session/domain boundary.
    }
    if (strcmp(cmd, "freerun") == 0) {
        _winder.setFreerun(strcmp(value, "true") == 0); // Map text flag to boolean mode.
        return true;                           // Command consumed.
    }
    if (strcmp(cmd, "direction") == 0) {
        _winder.setDirection(strcmp(value, "cw") == 0 ? Direction::CW : Direction::CCW); // Select spindle direction.
        return true; // Command consumed.
    }
    if (strcmp(cmd, "max_rpm") == 0 || strcmp(cmd, "max-rpm") == 0) {
        _winder.setMaxRpm((uint16_t)constrain((int)strtol(value, nullptr, 10), 10, 1500)); // Clamp safe operating range.
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
        // Log only on meaningful changes to avoid spamming the serial console.
        if (DIAG_VERBOSE && (_sessionState != _lastLoggedSessionState || _runMode != _lastLoggedRunMode
            || _lastEventSource != _lastLoggedSource)) {
            Diag::infof("[Session] ARMED speedHz=%u pot=%.3f source=%d runMode=%d",
                        commandedSpeedHz, _potLevel, (int)_lastEventSource, (int)_runMode);
            _lastLoggedSessionState = _sessionState;
            _lastLoggedRunMode = _runMode;
            _lastLoggedSource = _lastEventSource;
        }
    }
    else
    {
        // Any non-armed state forces zero speed command.
        _winder.setControlHz(0); // Guarantee no speed command outside ARMED_OR_RUNNING.
        if (DIAG_VERBOSE && (_sessionState != _lastLoggedSessionState || _lastEventSource != _lastLoggedSource)) {
            Diag::infof("[Session] PAUSED source=%d pot=%.3f", (int)_lastEventSource, _potLevel);
            _lastLoggedSessionState = _sessionState;
            _lastLoggedSource = _lastEventSource;
        }
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

                // When IDLE, do not emit pot-derived intents — just update baseline
                // so moving the pot doesn't arm or pause the session unexpectedly.
                if (_sessionState == SessionState::IDLE) {
#if DIAG_VERBOSE
                    Diag::infof("[Session] Ignoring pot event while IDLE (aboveZero=%d)", (int)potAboveZero);
#endif
                } else {
                    if (!potAboveZero) {
                        _potNeedsRearm = false;
                        recordIntent(ControlIntent::Pause, InputSource::Pot);
                    } else if (!_potNeedsRearm) {
                        recordIntent(ControlIntent::Start, InputSource::Pot);
                    }
                }
            }

            if (_sessionState == SessionState::ARMED_OR_RUNNING && potAboveZero) {
                // Pot changes override previous source once in run state.
                _runMode = RunMode::Pot;
            }
        }
    // 2) Integrate footswitch edge events.
    if (in.hasFootswitch) {
        // Establish baseline on first sample and skip intent emission while IDLE
        // so that release (and its Pause) cannot flip the session into PAUSED
        // causing the next press to be accepted. While IDLE we simply sample
        // the input without producing intents.
        if (!_hasFootswitchSample) {
            _hasFootswitchSample = true;
            _footswitch = in.footswitch;
        } else if (_footswitch != in.footswitch) {
            _footswitch = in.footswitch; // update latest state
            if (_sessionState == SessionState::IDLE) {
#if DIAG_VERBOSE
                Diag::infof("[Session] Ignoring footswitch event while IDLE (pressed=%d)", (int)_footswitch);
#endif
            } else {
                recordIntent(_footswitch ? ControlIntent::Start : ControlIntent::Pause,
                             InputSource::Footswitch); // Press->start, release->pause.
            }
        }
    }

    // 3) Decode commands provided in TickInput (bounded, no dynamic alloc).
    for (int i = 0; i < in.cmdCount; ++i) {
        const char* c = in.commands[i].cmd; // Raw command key from queue.
        const char* v = in.commands[i].val; // Raw command value from queue.
        // Session-level commands handled here; others forwarded to WinderApp.
        if (!handleCommand(c, v)) {
            _winder.handleCommand(c, v); // Forward unknown session command.
        }
    }

    // 4) Apply exactly one resolved intent (last event wins).
    applyPendingIntent(); // Commit the most recent pending intent.

    // 5) Push output command and let winding domain advance one step.
    applyPower();  // Update spindle speed command for this cycle.
    _winder.tick(); // Advance domain state machine and motor/lateral logic.
}
