#!/usr/bin/env python3
"""Rewrite ARMED/VERIFY_RUN/VERIFY_PAUSE → VERIFY_LOW/VERIFY_HIGH FSM."""
import pathlib, re, sys

ROOT = pathlib.Path(r"c:\temp\PickupWinder")

# ─────────────────────────────────────────────────────────────────────────────
# Types.h
# ─────────────────────────────────────────────────────────────────────────────
TYPES = ROOT / "include" / "Types.h"
types_text = TYPES.read_text(encoding="utf-8")

NEW_ENUM_BLOCK = '''\
// ── WindingState ──────────────────────────────────────────────────────────────
// Single source of truth for the winding session lifecycle.
//
//  IDLE ──(start)──► VERIFY_LOW ──(confirm low)──► VERIFY_HIGH ──(confirm high)──► WINDING
//                        ▲               │                  ▲           │                 │
//                        │     (verify_low)◄────────────────┘           │           (pot→0)▼
//                        │               └───────────────(verify_high)──►          PAUSED
//                   (re-verify from                                             (pot↑)──►┘
//                   WINDING/PAUSED)                                         (turns≥target)──► TARGET_REACHED
//
//  Any state ──(stop/reset)──► IDLE
//  VERIFY_LOW ◄──► VERIFY_HIGH at any time via verify_low / verify_high commands.
//  When both low and high have been confirmed, auto-transitions to WINDING.
enum class WindingState {
    IDLE,           // No session — all parameters writable, carriage at home
    VERIFY_LOW,     // Verifying low (start) bound — carriage at start, pot runs motor at verify speed
    VERIFY_HIGH,    // Verifying high (end) bound  — carriage at end,   pot runs motor at verify speed
    WINDING,        // Active winding at user-set speed
    PAUSED,         // Mid-winding pause (pot→0) — resumes from current position
    TARGET_REACHED, // Auto-stop: target turns hit — blocked until reset or target raised
};

inline const char* windingStateName(WindingState s) {
    switch (s) {
        case WindingState::IDLE:           return "IDLE";
        case WindingState::VERIFY_LOW:     return "VERIFY_LOW";
        case WindingState::VERIFY_HIGH:    return "VERIFY_HIGH";
        case WindingState::WINDING:        return "WINDING";
        case WindingState::PAUSED:         return "PAUSED";
        case WindingState::TARGET_REACHED: return "TARGET_REACHED";
        default:                           return "UNKNOWN";
    }
}'''

# Replace from "// ── WindingState ──" up to (and including) the closing } of windingStateName
pattern = re.compile(
    r'// \u2500\u2500 WindingState \u2500.*?^}',
    re.DOTALL | re.MULTILINE
)
types_new = pattern.sub(NEW_ENUM_BLOCK, types_text)
if types_new == types_text:
    print("FAIL: Types.h WindingState block not matched"); sys.exit(1)

# Also fix getStatus firstReversalPaused field comment if present
types_new = types_new.replace(
    "bool     firstReversalPaused; // True in VERIFY_PAUSE state",
    "bool     verifyLow;           // True in VERIFY_LOW state\n"
    "    bool     verifyHigh;          // True in VERIFY_HIGH state"
)

TYPES.write_text(types_new, encoding="utf-8")
print(f"OK: Types.h updated (+{len(types_new)-len(types_text):+d} chars)")

# ─────────────────────────────────────────────────────────────────────────────
# WinderApp.h
# ─────────────────────────────────────────────────────────────────────────────
HEADER = ROOT / "include" / "WinderApp.h"
header_text = HEADER.read_text(encoding="utf-8")

# Replace the state transitions block (private declarations)
OLD_TRANSITIONS = '''\
    // ── State transitions ─────────────────────────────────────────────────────
    void _toIdle();
    void _toArmed();
    void _toVerifyPause();
    void _toTargetReached();
    void _toPaused();
    // _toVerifyRun and _toWinding are triggered inside _runWindingAtHz when
    // the motor starts, because the plan/speed must be computed first.'''

NEW_TRANSITIONS = '''\
    // ── State transitions ─────────────────────────────────────────────────────
    void _toIdle();
    void _toVerifyLow();
    void _toVerifyHigh();
    void _toWinding();        // called from confirm once both bounds verified
    void _toPaused();
    void _toTargetReached();'''

if OLD_TRANSITIONS not in header_text:
    print("FAIL: WinderApp.h transitions block not matched"); sys.exit(1)
header_text = header_text.replace(OLD_TRANSITIONS, NEW_TRANSITIONS)

# Replace transient flags block
OLD_FLAGS = '''\
    // Safety interlock: pot must physically return to zero before the motor
    // restarts — cleared on every transition that stops the motor.
    bool _potWasZero = false;

    // Deferred driver disable: set when stopping so the driver is cut only
    // after the deceleration ramp finishes (checked in _applyDeferredDisable).
    bool _pendingDisable = false;

    // Set in _toArmed() when turns == 0: arms the lateral to pause at the
    // first reversal so the operator can validate the high end-stop position.
    bool _pauseOnFirstReversal = false;'''

NEW_FLAGS = '''\
    // Safety interlock: pot must physically return to zero before the motor
    // restarts — cleared on every transition that stops the motor.
    bool _potWasZero = false;

    // Deferred driver disable: set when stopping so the driver is cut only
    // after the deceleration ramp finishes (checked in _applyDeferredDisable).
    bool _pendingDisable = false;

    // Verification tracking: set to true when the operator confirms each bound.
    // Both must be true before WINDING starts. Reset on _toIdle().
    bool _lowVerified  = false;
    bool _highVerified = false;'''

if OLD_FLAGS not in header_text:
    print("FAIL: WinderApp.h flags block not matched"); sys.exit(1)
header_text = header_text.replace(OLD_FLAGS, NEW_FLAGS)

HEADER.write_text(header_text, encoding="utf-8")
print(f"OK: WinderApp.h updated")

# ─────────────────────────────────────────────────────────────────────────────
# WinderApp.cpp — full rewrite of the changed sections
# ─────────────────────────────────────────────────────────────────────────────
CPP = ROOT / "src" / "WinderApp.cpp"
cpp_text = CPP.read_text(encoding="utf-8")

# ── 1. Replace ALL transition functions (from _toIdle through _handleLateralEvents) ──
OLD_TRANSITIONS_CPP = re.compile(
    r'// \u2500\u2500 State transitions \u2500.*?(?=\nvoid WinderApp::_handlePotCycle)',
    re.DOTALL
)

NEW_TRANSITIONS_CPP = '''\
// ── State transitions ──────────────────────────────────────────────────────────────
//
// Every _toXxx() is self-contained: sets _state, adjusts hardware, clears flags.

void WinderApp::_toIdle() {
    _state          = WindingState::IDLE;
    _potWasZero     = false;
    _pendingDisable = true;
    _lowVerified    = false;
    _highVerified   = false;
    _stepper.stop();
    _lateral.stopWinding();
    _lateral.parkAtZero();
    _stepper.resetTurns();
    _planner.reset();
    _activePlan = _planner.getPlan(0, 0.0f);
    _led.reset();
    Serial.println("[IDLE] Stop -- carriage to home, counter reset");
}

void WinderApp::_toVerifyLow() {
    _state          = WindingState::VERIFY_LOW;
    _potWasZero     = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    _lateral.prepareStartPosition(_windingStartMm());
    Serial.printf("[VERIFY_LOW] Carriage -> %.2f mm (low bound)%s\\n",
                  _windingStartMm(), _lowVerified ? " [already confirmed]" : "");
}

void WinderApp::_toVerifyHigh() {
    _state          = WindingState::VERIFY_HIGH;
    _potWasZero     = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    _lateral.prepareStartPosition(_windingEndMm());
    Serial.printf("[VERIFY_HIGH] Carriage -> %.2f mm (high bound)%s\\n",
                  _windingEndMm(), _highVerified ? " [already confirmed]" : "");
}

void WinderApp::_toWinding() {
    _state          = WindingState::WINDING;
    _potWasZero     = false;
    _pendingDisable = false;
    _lateral.prepareStartPosition(_windingStartMm());
    Serial.printf("[WINDING] Both bounds verified -- %.2f -> %.2f mm, waiting for pot\\n",
                  _windingStartMm(), _windingEndMm());
}

void WinderApp::_toPaused() {
    _state          = WindingState::PAUSED;
    _potWasZero     = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    Serial.println("[PAUSED] Resume from current position when pot goes up");
}

void WinderApp::_toTargetReached() {
    _state          = WindingState::TARGET_REACHED;
    _potWasZero     = false;
    _pendingDisable = true;
    _stepper.stop();
    _lateral.stopWinding();
    Serial.printf("[TARGET_REACHED] %ld turns -- raise target to continue or press Stop\\n",
                  _stepper.getTurns());
}

void WinderApp::_handleLateralEvents() {
    _lateral.update();
    // No automatic pause at reversal in this FSM -- verifications are explicit.
}

'''

cpp_new = OLD_TRANSITIONS_CPP.sub(NEW_TRANSITIONS_CPP, cpp_text)
if cpp_new == cpp_text:
    print("FAIL: CPP transitions block not matched"); sys.exit(1)
print("OK: CPP transitions replaced")

# ── 2. Replace _handlePotCycle ──────────────────────────────────────────────
OLD_POT = re.compile(
    r'void WinderApp::_handlePotCycle\(uint32_t hz\) \{.*?\n\}(?=\n\nbool WinderApp::_readyForSpin)',
    re.DOTALL
)

NEW_POT = '''\
void WinderApp::_handlePotCycle(uint32_t hz) {
    const bool potActive = (hz > 0);

    // Interlock: cleared only when the pot physically returns to zero.
    if (!potActive) _potWasZero = true;

    switch (_state) {

    case WindingState::IDLE:
        // Drift back to home if carriage moved.
        if (_lateral.isHomed() && !_lateral.isBusy() && !_lateral.isAtZero())
            _lateral.parkAtZero();
        break;

    case WindingState::VERIFY_LOW:
    case WindingState::VERIFY_HIGH: {
        // Hold carriage at the verification bound when motor is stopped.
        if (!_stepper.isRunning() && _lateral.isHomed() && !_lateral.isBusy()) {
            float pos = (_state == WindingState::VERIFY_LOW)
                        ? _windingStartMm() : _windingEndMm();
            _lateral.prepareStartPosition(pos);
        }
        if (potActive && _potWasZero) {
            _runWindingAtHz(hz);         // runs at verify speed, stays in VERIFY_xx state
        } else if (!potActive && _stepper.isRunning()) {
            _pendingDisable = true;
            _stepper.stop();
            _lateral.stopWinding();
            Serial.printf("[%s] Pot zero -- motor stopped\\n", windingStateName(_state));
        }
        break;
    }

    case WindingState::WINDING:
        if (potActive)
            _runWindingAtHz(hz);
        else
            _toPaused();
        break;

    case WindingState::PAUSED:
        // Pot up + interlock cleared + lateral ready -> resume winding.
        if (potActive && _potWasZero && _lateral.isHomed() && !_lateral.isBusy())
            _runWindingAtHz(hz);   // transitions to WINDING inside _runWindingAtHz
        break;

    case WindingState::TARGET_REACHED:
        // Locked -- only raising the target can unlock.
        break;
    }

    _led.update(_stepper.getTurns(), max(1L, _activePlan.turnsPerPass), _stepper.isRunning());
}'''

cpp_new2 = OLD_POT.sub(NEW_POT, cpp_new)
if cpp_new2 == cpp_new:
    print("FAIL: _handlePotCycle not matched"); sys.exit(1)
print("OK: _handlePotCycle replaced")

# ── 3. Replace _readyForSpin ────────────────────────────────────────────────
OLD_READY = re.compile(
    r'bool WinderApp::_readyForSpin\(\) const \{.*?\n\}(?=\n\nvoid WinderApp::_runWindingAtHz)',
    re.DOTALL
)
# _readyForSpin is no longer needed in _handlePotCycle (inlined there),
# but keep as a helper used in _runWindingAtHz for the PAUSED resume guard.
NEW_READY = '''\
bool WinderApp::_readyForSpin() const {
    // Used to guard resume after pause: lateral must be homed and idle.
    return _lateral.isHomed() && !_lateral.isBusy();
}'''

cpp_new3 = OLD_READY.sub(NEW_READY, cpp_new2)
if cpp_new3 == cpp_new2:
    print("FAIL: _readyForSpin not matched"); sys.exit(1)
print("OK: _readyForSpin replaced")

# ── 4. Replace _runWindingAtHz motor-start block ────────────────────────────
# Replace the _pauseOnFirstReversal / VERIFY_RUN / WINDING start block.
OLD_MOTOR_START = re.compile(
    r'    // -- Motor start --\n    if \(!_stepper\.isRunning\(\)\) \{.*?        return;\n    \}',
    re.DOTALL
)
NEW_MOTOR_START = '''\
    // -- Motor start --
    if (!_stepper.isRunning()) {
        bool forward = (_direction == Direction::CW) != (bool)WINDING_MOTOR_INVERTED;

        // In WINDING (entered from _toWinding after confirm) or PAUSED -> WINDING transition.
        if (_state != WindingState::VERIFY_LOW && _state != WindingState::VERIFY_HIGH) {
            _state = WindingState::WINDING;
            Serial.printf("[WINDING] %s -- %u Hz -- profile=%s tpp=%ld scale=%.2f\\n",
                          _direction == Direction::CW ? "CW" : "CCW", hz,
                          WindingPatternPlanner::styleName(_recipe.style),
                          _activePlan.turnsPerPass, _activePlan.speedScale);
        }
        // In VERIFY_LOW/HIGH: stay in the verify state, motor runs verify pass.

        _stepper.start(forward);
        _lateral.startWinding(hz, _activePlan.turnsPerPass,
                              _windingStartMm(), _windingEndMm(),
                              _activePlan.speedScale);
        return;
    }'''

cpp_new4 = OLD_MOTOR_START.sub(NEW_MOTOR_START, cpp_new3)
if cpp_new4 == cpp_new3:
    print("FAIL: motor start block not matched"); sys.exit(1)
print("OK: _runWindingAtHz motor start replaced")

# ── 5. Update speed cap: VERIFY_RUN -> VERIFY_LOW || VERIFY_HIGH ────────────
cpp_new4 = cpp_new4.replace(
    "    if (_state == WindingState::VERIFY_RUN)\n        hz = min(hz, (uint32_t)VERIFY_SPEED_HZ_MAX);",
    "    if (_state == WindingState::VERIFY_LOW || _state == WindingState::VERIFY_HIGH)\n        hz = min(hz, (uint32_t)VERIFY_SPEED_HZ_MAX);"
)
print("OK: VERIFY_RUN speed cap updated")

# ── 6. Replace getStatus() state mappings ──────────────────────────────────
cpp_new4 = cpp_new4.replace(
    "    const bool motorEnabled  = (_state == WindingState::WINDING\n                             || _state == WindingState::VERIFY_RUN);",
    "    const bool motorEnabled  = (_state == WindingState::WINDING\n                             || _state == WindingState::VERIFY_LOW\n                             || _state == WindingState::VERIFY_HIGH);"
)

cpp_new4 = cpp_new4.replace(
    "        (_state == WindingState::VERIFY_PAUSE),   // firstReversalPaused",
    "        (_state == WindingState::VERIFY_LOW),    // verifyLow\n        (_state == WindingState::VERIFY_HIGH),   // verifyHigh"
)
print("OK: getStatus mappings updated")

# ── 7. Replace _refreshCarriageForGeometryChange ───────────────────────────
OLD_REFRESH = re.compile(
    r'void WinderApp::_refreshCarriageForGeometryChange.*?\n\}(?=\n\nbool WinderApp::_handleImmediateCommand)',
    re.DOTALL
)
NEW_REFRESH = '''\
void WinderApp::_refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged) {
    if (_stepper.isRunning()) return;
    if (!_lateral.isHomed() || _lateral.isBusy()) return;

    switch (_state) {
    case WindingState::VERIFY_LOW:
    case WindingState::WINDING:
    case WindingState::PAUSED:
        if (startBoundChanged) _lateral.prepareStartPosition(_windingStartMm());
        break;
    case WindingState::VERIFY_HIGH:
        if (endBoundChanged) _lateral.prepareStartPosition(_windingEndMm());
        break;
    default:
        break;
    }
}'''

cpp_new5 = OLD_REFRESH.sub(NEW_REFRESH, cpp_new4)
if cpp_new5 == cpp_new4:
    print("FAIL: _refreshCarriageForGeometryChange not matched"); sys.exit(1)
print("OK: _refreshCarriageForGeometryChange replaced")

# ── 8. Replace _handleImmediateCommand ─────────────────────────────────────
OLD_IMMED = re.compile(
    r'bool WinderApp::_handleImmediateCommand\(const String& cmd, const String& value\) \{.*?\n\}(?=\n\nbool WinderApp::_handleGeometryCommand)',
    re.DOTALL
)
NEW_IMMED = '''\
bool WinderApp::_handleImmediateCommand(const String& cmd, const String& value) {
    if (cmd == "stop") {
        _toIdle();
        return true;
    }

    if (cmd == "reset") {
        _toIdle();
        Serial.println("[IDLE] Turn counter reset");
        return true;
    }

    if (cmd == "start") {
        if (_state != WindingState::IDLE && _state != WindingState::TARGET_REACHED) {
            Serial.println("[Start] Ignored -- session already active");
            return true;
        }
        if (!_lateral.isHomed() || _lateral.isBusy()) {
            Serial.println("[Start] Impossible -- lateral axis not ready");
            return true;
        }
        // Fresh session: reset turn counter and verification state.
        if (_state == WindingState::TARGET_REACHED) {
            _stepper.resetTurns();
            _planner.reset();
        }
        _lowVerified  = false;
        _highVerified = false;
        _toVerifyLow();
        return true;
    }

    // Jump to low-bound verification from any active state.
    if (cmd == "verify_low") {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            Serial.println("[verify_low] No active session");
            return true;
        }
        _toVerifyLow();
        return true;
    }

    // Jump to high-bound verification from any active state.
    if (cmd == "verify_high") {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            Serial.println("[verify_high] No active session");
            return true;
        }
        _toVerifyHigh();
        return true;
    }

    // Confirm the current verification bound; auto-advances when both done.
    if (cmd == "confirm") {
        if (_state == WindingState::VERIFY_LOW) {
            _lowVerified = true;
            Serial.println("[VERIFY_LOW] Low bound confirmed");
            if (_highVerified) _toWinding();
            else               _toVerifyHigh();
        } else if (_state == WindingState::VERIFY_HIGH) {
            _highVerified = true;
            Serial.println("[VERIFY_HIGH] High bound confirmed");
            if (_lowVerified) _toWinding();
            else              _toVerifyLow();
        } else {
            Serial.printf("[Confirm] Ignored in state %s\\n", windingStateName(_state));
        }
        return true;
    }

    if (cmd == "stop_next_high") {
        _lateral.armStopAtNextHigh();
        Serial.println("[Mode] Stop armed on next high bound");
        return true;
    }

    if (cmd == "stop_next_low") {
        _lateral.armStopAtNextLow();
        Serial.println("[Mode] Stop armed on next low bound");
        return true;
    }

    return false;
}'''

cpp_new5 = OLD_IMMED.sub(NEW_IMMED, cpp_new5)
if cpp_new5 == cpp_new4:
    print("FAIL: _handleImmediateCommand not matched"); sys.exit(1)
print("OK: _handleImmediateCommand replaced")

# ── 9. Fix handleCommand (currently truncated at "// Target is always...") ──
OLD_HANDLE_CMD = re.compile(
    r'void WinderApp::handleCommand\(const String& cmd, const String& value\) \{.*?// Target is always modifiable -- even during winding\.\n\n',
    re.DOTALL
)
NEW_HANDLE_CMD = '''\
void WinderApp::handleCommand(const String& cmd, const String& value) {
    if (_handleImmediateCommand(cmd, value)) return;
    if (_handleGeometryCommand(cmd, value))  return;

    // Target is always modifiable -- even during winding.
    if (cmd == "target") {
        long t = value.toInt();
        if (t > 0) {
            _targetTurns = t;
            if (_state == WindingState::TARGET_REACHED && t > _stepper.getTurns()) {
                _toPaused();
                Serial.println("[PAUSED] Target raised -- resume possible");
            }
            Serial.printf("Target: %ld turns\\n", t);
            _saveRecipe();
        }
        return;
    }

    // All other parameters are locked once a session is active.
    if (_parametersLocked()) {
        Serial.printf("[Lock] Ignored during session (%s): %s\\n",
                      windingStateName(_state), cmd.c_str());
        return;
    }

    if (cmd == "freerun") {
        _freerun = (value == "true");
        Serial.printf("Mode: %s\\n", _freerun ? "FreeRun" : "Target");
        _saveRecipe();
        return;
    }

    if (cmd == "direction") {
        Direction newDir = (value == "cw") ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            Serial.printf("Direction: %s\\n", value.c_str());
            _saveRecipe();
        }
        return;
    }

    if (_handlePatternCommand(cmd, value)) return;

    if (cmd == "recipe_import") {
        WindingRecipe imported;
        if (_recipeStore.fromJson(value, imported)) {
            _applyRecipe(imported, true);
            Serial.println("[Recipe] Recipe imported");
        } else {
            Serial.println("[Recipe] ERROR -- invalid JSON");
        }
        return;
    }

    if (cmd == "lat_offset") {
        float mm = value.toFloat();
        if (mm >= 0.0f) {
            _lateral.setHomeOffset(mm);
            _toIdle();
            _lateral.rehome();
            Serial.printf("[Lateral] Offset %.2f mm -- rehoming started\\n", mm);
            _saveRecipe();
        }
        return;
    }
}

'''

cpp_new6 = OLD_HANDLE_CMD.sub(NEW_HANDLE_CMD, cpp_new5)
if cpp_new6 == cpp_new5:
    print("FAIL: handleCommand not matched"); sys.exit(1)
print("OK: handleCommand replaced/restored")

# ── 10. Fix _applyRecipe: remove _pauseOnFirstReversal reference ────────────
cpp_new6 = cpp_new6.replace(
    "    _pauseOnFirstReversal = false;\n    _lateral.setHomeOffset(recipe.latOffsetMm);",
    "    _lateral.setHomeOffset(recipe.latOffsetMm);"
)
print("OK: _applyRecipe _pauseOnFirstReversal removed")

CPP.write_text(cpp_new6, encoding="utf-8")
print(f"\\nAll done. WinderApp.cpp: {len(cpp_new6.splitlines())} lines")
