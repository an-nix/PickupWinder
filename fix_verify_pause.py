import pathlib

f = pathlib.Path(r"c:\temp\PickupWinder\src\WinderApp.cpp")
t = f.read_text(encoding="utf-8")

# Diagnose
idx = t.find('Pot zero -- motor stopped')
if idx < 0:
    idx = t.find('VERIFY_HIGH: {')
print('anchor at:', idx)
if idx > 0:
    print(repr(t[max(0,idx-20):idx+100]))

OLD = (
    '    case WindingState::VERIFY_LOW:\n'
    '    case WindingState::VERIFY_HIGH: {\n'
    '        // Hold carriage at the verification bound when motor is stopped.\n'
    '        if (!_stepper.isRunning() && _lateral.isHomed() && !_lateral.isBusy()) {\n'
    '            float pos = (_state == WindingState::VERIFY_LOW)\n'
    '                        ? _windingStartMm() : _windingEndMm();\n'
    '            _lateral.prepareStartPosition(pos);\n'
    '        }\n'
    '        if (potActive && _potWasZero) {\n'
    '            _runWindingAtHz(hz);         // runs at verify speed, stays in VERIFY_xx state\n'
    '        } else if (!potActive && _stepper.isRunning()) {\n'
    '            _pendingDisable = true;\n'
    '            _stepper.stop();\n'
    '            _lateral.stopWinding();\n'
    '            Serial.printf("[%s] Pot zero -- motor stopped\n", windingStateName(_state));\n'
    '        }\n'
    '        break;\n'
    '    }'
)

NEW = (
    '    case WindingState::VERIFY_LOW:\n'
    '    case WindingState::VERIFY_HIGH: {\n'
    '        // Fresh entry (_potWasZero == false): drive carriage to the verification\n'
    '        // bound and wait for the operator to raise the pot.\n'
    '        // Mid-verify pause (_potWasZero == true): motor stopped because pot hit\n'
    '        // zero; leave carriage in place so the operator resumes from there.\n'
    '        if (!_stepper.isRunning() && !_potWasZero && _lateral.isHomed() && !_lateral.isBusy()) {\n'
    '            float pos = (_state == WindingState::VERIFY_LOW)\n'
    '                        ? _windingStartMm() : _windingEndMm();\n'
    '            _lateral.prepareStartPosition(pos);\n'
    '        }\n'
    '        if (potActive && _potWasZero) {\n'
    '            _runWindingAtHz(hz);         // resumes from current position at verify speed\n'
    '        } else if (!potActive && _stepper.isRunning()) {\n'
    '            _pendingDisable = true;\n'
    '            _stepper.stop();\n'
    '            _lateral.stopWinding();\n'
    '            Serial.printf("[%s] Pot zero -- paused, resume from current position\\n",\n'
    '                          windingStateName(_state));\n'
    '        }\n'
    '        break;\n'
    '    }'
)

if OLD not in t:
    print("FAIL: OLD block not found in file")
    raise SystemExit(1)

t2 = t.replace(OLD, NEW, 1)
f.write_text(t2, encoding="utf-8")
print("OK: VERIFY pot-pause behaviour updated")

# The file has a literal newline inside the printf format string (bug from script),
# so we normalise it: find the block regardless of whether \n is escaped or literal.
pattern = re.compile(
    r'(    case WindingState::VERIFY_LOW:\n'
    r'    case WindingState::VERIFY_HIGH: \{)\n'
    r'        // Hold carriage at the verification bound when motor is stopped\.\n'
    r'        (if \(!_stepper\.isRunning\(\)) && (_lateral\.isHomed\(\) && !_lateral\.isBusy\(\)) \{',
    re.DOTALL
)

NEW = '''\
    case WindingState::VERIFY_LOW:
    case WindingState::VERIFY_HIGH: {
        // Fresh entry (_potWasZero == false): drive carriage to the verification
        // bound and wait for the operator to raise the pot.
        // Mid-verify pause (_potWasZero == true): motor was running, pot dropped
        // to zero; leave the carriage in place and resume from there on pot-up.
        if (!_stepper.isRunning() && !_potWasZero && _lateral.isHomed() && !_lateral.isBusy()) {
            float pos = (_state == WindingState::VERIFY_LOW)
                        ? _windingStartMm() : _windingEndMm();
            _lateral.prepareStartPosition(pos);
        }
        if (potActive && _potWasZero) {
            _runWindingAtHz(hz);         // resumes from current position at verify speed
        } else if (!potActive && _stepper.isRunning()) {
            _pendingDisable = true;
            _stepper.stop();
            _lateral.stopWinding();
            Serial.printf("[%s] Pot zero -- paused, resume from current position\\n",
                          windingStateName(_state));
        }
        break;
    }'''

t2, n = pattern.subn(NEW, t)
if n == 0:
    print("FAIL: pattern did not match")
    # Debug: show what's around VERIFY_HIGH
    idx = t.find("Hold carriage at the verification bound")
    if idx >= 0:
        print(repr(t[idx-200:idx+400]))
    raise SystemExit(1)

f.write_text(t2, encoding="utf-8")
print(f"OK: VERIFY pot-pause behaviour updated ({n} replacement(s))")
