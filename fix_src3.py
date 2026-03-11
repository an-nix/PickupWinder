with open("src/WinderApp.cpp", "r") as f:
    text = f.read()

# 1. Remove if (_pauseParkPending ...) block
block = """    // Pause pot=0 : on attend l'arrêt complet du latéral avant de demander
    // un retour en position 0, pour éviter tout mouvement parasite/à-coup.
    if (_pauseParkPending && _lateral.getState() == LatState::HOMED && !_lateral.isBusy()) {
        if (_lateral.isAtZero()) {
            _pauseParkPending = false;
            Serial.println("[Pause] Chariot revenu à 0.");
        } else {
            _lateral.parkAtZero();
        }
    }"""
text = text.replace(block, "")

# Remove other _pauseParkPending lines
text = text.replace("        _pauseParkPending      = false;\n", "")
text = text.replace("    _pauseParkPending      = false;\n", "")
text = text.replace("        _pauseParkPending = false;\n", "")
text = text.replace("             && !_pauseParkPending && !_resumeFromCurrentPos) {", "             && !_resumeFromCurrentPos) {")


# 2. Replace _pause()
old_pause = """void WinderApp::_pause() {
    // Pause mid-winding (pot → 0). Keeps _startRequested so the user can
    // resume just by raising the pot, without pressing Start again.
    // The carriage returns to 0 then auto-repositions to windingStart before
    // the motor can restart (same flow as after pressing Start from idle).
    _motorEnabled        = false;
    _potWasZero          = false;
    _midWindingPaused    = true;
    // Pendant la passe de vérification initiale, une pause pot=0 doit
    // reprendre depuis la position courante (pas de retour automatique à 0).
    _resumeFromCurrentPos = _inVerificationRun;
    _pauseParkPending    = !_resumeFromCurrentPos;
    _pendingDisable      = true;
    _pauseOnFirstReversal = false;  // déjà armé dans le latéral si mid-vérification
    _stepper.stop();
    _lateral.stopWinding();
    if (_resumeFromCurrentPos) {
        Serial.println("⏸ Pause — reprise depuis position courante");
    } else {
        Serial.println("⏸ Pause — arrêt puis retour chariot à 0");
    }
}"""

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
text = text.replace(old_pause, new_pause)


# 3. Add updateCarriage lambda and call it.
old_handle = """void WinderApp::_handleCommand(const String& cmd, const String& value) {"""
new_handle = """void WinderApp::_handleCommand(const String& cmd, const String& value) {
    auto updateCarriage = [this]() {
        if ((_startRequested || _midWindingPaused || _pausedForVerification) && !_stepper.isRunning()) {
            _lateral.prepareStartPosition(_windingStartMm());
        }
    };"""
text = text.replace(old_handle, new_handle)

# replace geom commands
text = text.replace("""} else if (cmd == "geom_total")  { _geom.totalWidth_mm   = value.toFloat(); _saveRecipe(); }""",
                    """} else if (cmd == "geom_total")  { _geom.totalWidth_mm   = value.toFloat(); _saveRecipe(); updateCarriage(); }""")

text = text.replace("""} else if (cmd == "geom_bottom") { _geom.flangeBottom_mm = value.toFloat(); _saveRecipe(); }""",
                    """} else if (cmd == "geom_bottom") { _geom.flangeBottom_mm = value.toFloat(); _saveRecipe(); updateCarriage(); }""")

text = text.replace("""} else if (cmd == "geom_top")    { _geom.flangeTop_mm    = value.toFloat(); _saveRecipe(); }""",
                    """} else if (cmd == "geom_top")    { _geom.flangeTop_mm    = value.toFloat(); _saveRecipe(); updateCarriage(); }""")

text = text.replace("""} else if (cmd == "geom_margin") { _geom.margin_mm       = value.toFloat(); _saveRecipe(); }""",
                    """} else if (cmd == "geom_margin") { _geom.margin_mm       = value.toFloat(); _saveRecipe(); updateCarriage(); }""")

with open("src/WinderApp.cpp", "w") as f:
    f.write(text)

