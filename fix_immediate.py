#!/usr/bin/env python3
"""Replace old _handleImmediateCommand and duplicate handleCommand tail in WinderApp.cpp"""
import re, pathlib, sys

FILE = pathlib.Path(r"c:\temp\PickupWinder\src\WinderApp.cpp")
text = FILE.read_text(encoding="utf-8")

# ── 1. Replace _handleImmediateCommand ──────────────────────────────────────
NEW_IMMEDIATE = '''\
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
        if (!_lateral.isAtZero()) {
            Serial.println("[Start] Refused -- carriage not at position 0");
            return true;
        }
        if (_state == WindingState::TARGET_REACHED) {
            _stepper.resetTurns();
            _planner.reset();
        }
        _toArmed();
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

pattern1 = re.compile(
    r'bool WinderApp::_handleImmediateCommand\(const String& cmd, const String& value\) \{.*?\n\}(?=\n\nbool WinderApp::_handleGeometryCommand)',
    re.DOTALL
)
text2 = pattern1.sub(NEW_IMMEDIATE, text)
if text2 == text:
    print("FAIL: _handleImmediateCommand pattern did not match")
    sys.exit(1)
print(f"OK: _handleImmediateCommand replaced ({len(text)-len(text2):+d} chars)")

# ── 2. Remove duplicate old handleCommand tail ───────────────────────────────
# After the new handleCommand closes, the old body is orphaned
# (starts with "    if (cmd == "target") {" and uses _targetReached)
# Find the second "void WinderApp::handleCommand" occurrence signature is not
# present (it was just the body); anchor on the duplicate "if (cmd == "target")"
# followed by "_targetReached" which appears only in that tail.
pattern2 = re.compile(
    r'\n    if \(cmd == "target"\) \{.*?\n\}(?=\n\nTraversePlan WinderApp::)',
    re.DOTALL
)
text3 = pattern2.sub('', text2)
if text3 == text2:
    print("FAIL: duplicate handleCommand tail pattern did not match")
    sys.exit(1)
print(f"OK: duplicate handleCommand tail removed ({len(text2)-len(text3):+d} chars)")

# ── 3. Remove orphan old _handlePotCycle body ────────────────────────────────
# After the new switch-based _handlePotCycle closes, an old body fragment remains
# (starts with a French comment about _startRequested, ends with a duplicate
#  _led.update call + closing brace, just before _readyForSpin).
pattern3 = re.compile(
    r'(?<=\n\})\n\n    // Si Start.*?_led\.update\(_stepper\.getTurns\(\), max\(1L, _activePlan\.turnsPerPass\), _stepper\.isRunning\(\)\);\n\}(?=\n\nbool WinderApp::_readyForSpin)',
    re.DOTALL
)
text4 = pattern3.sub('', text3)
if text4 == text3:
    print("INFO: no orphan _handlePotCycle body found (may already be clean)")
    text4 = text3
else:
    print(f"OK: orphan _handlePotCycle body removed ({len(text3)-len(text4):+d} chars)")

FILE.write_text(text4, encoding="utf-8")
print("File written.")
