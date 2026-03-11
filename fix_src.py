import re

with open("src/WinderApp.cpp", "r") as f:
    text = f.read()

# 1. Remove references to _pauseParkPending
text = re.sub(r'^\s*_pauseParkPending\s*=\s*(true|false|!_resumeFromCurrentPos);\s*?\n', '', text, flags=re.MULTILINE)
text = re.sub(r'^\s*&& !_pauseParkPending\s*(?=\s*&&|\))', '', text, flags=re.MULTILINE)

# Remove the specific if-block
block_to_remove = """    // Pause pot=0 : on attend l'arrêt complet du latéral avant de demander
    // un retour en position 0, pour éviter tout mouvement parasite/à-coup.
    if (_pauseParkPending && _lateral.getState() == LatState::HOMED && !_lateral.isBusy()) {
        if (_lateral.isAtZero()) {
            _pauseParkPending = false;
            Serial.println("[Pause] Chariot revenu à 0.");
        } else {
            _lateral.parkAtZero();
        }
    }"""
text = text.replace(block_to_remove, "")

# 2. Replace _pause()
old_pause = re.search(r'void WinderApp::_pause\(\)\s*\{[^\}]+\}', text, flags=re.MULTILINE)
if old_pause:
    new_pause = """void WinderApp::_pause() {
    _motorEnabled        = false;
    _potWasZero          = false;
    _midWindingPaused    = true;
    _resumeFromCurrentPos = true;
    _pendingDisable      = true;
    _pauseOnFirstReversal = false;
    _stepper.stop();
    _lateral.stopWinding();
    Serial.println("⏸ Pause — reprise depuis position courante");
}"""
    text = text.replace(old_pause.group(0), new_pause)
else:
    print("Could not find _pause()")

# 3. Add updateCarriage lambda and call it.
# Find _handleCommand
text = re.sub(r'(void WinderApp::_handleCommand\(const String& cmd, const String& value\)\s*\{)',
              r'\1\n    auto updateCarriage = [this]() {\n        if ((_startRequested || _midWindingPaused) && !_stepper.isRunning()) {\n            _lateral.prepareStartPosition(_windingStartMm());\n        }\n    };\n',
              text)

# We also need to add updateCarriage() calls to geom commands.
# Let's replace commands
text = re.sub(r'\} else if \(cmd == "geom_total"\)\s*\{ _geom\.totalWidth_mm\s*=\s*value\.toFloat\(\);\s*_saveRecipe\(\);\s*\}',
              r'} else if (cmd == "geom_total")  { _geom.totalWidth_mm   = value.toFloat(); _saveRecipe(); updateCarriage(); }', text)
text = re.sub(r'\} else if \(cmd == "geom_bottom"\)\s*\{ _geom\.flangeBottom_mm\s*=\s*value\.toFloat\(\);\s*_saveRecipe\(\);\s*\}',
              r'} else if (cmd == "geom_bottom") { _geom.flangeBottom_mm = value.toFloat(); _saveRecipe(); updateCarriage(); }', text)
text = re.sub(r'\} else if \(cmd == "geom_top"\)\s*\{ _geom\.flangeTop_mm\s*=\s*value\.toFloat\(\);\s*_saveRecipe\(\);\s*\}',
              r'} else if (cmd == "geom_top")    { _geom.flangeTop_mm    = value.toFloat(); _saveRecipe(); updateCarriage(); }', text)
text = re.sub(r'\} else if \(cmd == "geom_margin"\)\s*\{ _geom\.margin_mm\s*=\s*value\.toFloat\(\);\s*_saveRecipe\(\);\s*\}',
              r'} else if (cmd == "geom_margin") { _geom.margin_mm       = value.toFloat(); _saveRecipe(); updateCarriage(); }', text)

with open("src/WinderApp.cpp", "w") as f:
    f.write(text)

