import pathlib

ROOT = pathlib.Path(r"c:\temp\PickupWinder")

# ── WinderApp.h ──────────────────────────────────────────────────────────────
fh = ROOT / "include" / "WinderApp.h"
th = fh.read_text(encoding="utf-8")

th = th.replace(
    "    // Safety interlock: pot must physically return to zero before the motor\n"
    "    // restarts — cleared on every transition that stops the motor.\n"
    "    bool _potWasZero = false;",
    "    // Start/resume arm flag: must be true before the motor (re)starts.\n"
    "    // Set by the pot returning to zero, or by a UI \"resume\" command.\n"
    "    // Cleared on every state transition so each stop requires an explicit re-arm.\n"
    "    bool _canStart = false;"
)

# Also try ASCII dash variant (in case file uses -- not —)
th = th.replace(
    "    // Safety interlock: pot must physically return to zero before the motor\n"
    "    // restarts -- cleared on every transition that stops the motor.\n"
    "    bool _potWasZero = false;",
    "    // Start/resume arm flag: must be true before the motor (re)starts.\n"
    "    // Set by the pot returning to zero, or by a UI \"resume\" command.\n"
    "    // Cleared on every state transition so each stop requires an explicit re-arm.\n"
    "    bool _canStart = false;"
)

th = th.replace("_potWasZero", "_canStart")
fh.write_text(th, encoding="utf-8")
print("header: OK")

# ── WinderApp.cpp ────────────────────────────────────────────────────────────
fc = ROOT / "src" / "WinderApp.cpp"
tc = fc.read_text(encoding="utf-8")

# Mass rename
tc = tc.replace("_potWasZero", "_canStart")

# Update comment above the arm-flag line in _handlePotCycle
tc = tc.replace(
    "    // Interlock: cleared only when the pot physically returns to zero.\n"
    "    if (!potActive) _canStart = true;",
    "    // Arm flag: set when pot hits zero (physical safety) or via UI 'resume'.\n"
    "    if (!potActive) _canStart = true;"
)

# Add 'resume' command before stop_next_high
OLD = (
    '    if (cmd == "stop_next_high") {\n'
    '        _lateral.armStopAtNextHigh();\n'
    '        Serial.println("[Mode] Stop armed on next high bound");\n'
    '        return true;\n'
    '    }'
)
NEW = (
    '    // Resume: arms the motor without requiring a physical pot-zero trip.\n'
    '    if (cmd == "resume") {\n'
    '        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {\n'
    '            Serial.println("[Resume] No active session to resume");\n'
    '        } else if (_canStart) {\n'
    '            Serial.println("[Resume] Already armed -- raise the pot to start");\n'
    '        } else {\n'
    '            _canStart = true;\n'
    '            Serial.printf("[Resume] Armed in %s -- raise pot to run\\n",\n'
    '                          windingStateName(_state));\n'
    '        }\n'
    '        return true;\n'
    '    }\n'
    '\n'
    '    if (cmd == "stop_next_high") {\n'
    '        _lateral.armStopAtNextHigh();\n'
    '        Serial.println("[Mode] Stop armed on next high bound");\n'
    '        return true;\n'
    '    }'
)

if OLD not in tc:
    print("FAIL: stop_next_high anchor not found")
else:
    tc = tc.replace(OLD, NEW, 1)
    print("resume command: OK")

fc.write_text(tc, encoding="utf-8")
print("cpp: OK")
print(f"Total _canStart references in cpp: {tc.count('_canStart')}")
