#include "WinderApp.h"

#include <Arduino.h>

#include "Diag.h"

bool WinderApp::_handleImmediateCommand(const char* cmd, const char* value) {
    if (strcmp(cmd, "stop") == 0) {
        _toIdle();
        return true;
    }

    if (strcmp(cmd, "reset") == 0) {
        _toIdle();
        Diag::info("[IDLE] Turn counter reset");
        return true;
    }

    if (strcmp(cmd, "start") == 0) {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            if (!_lateral.isHomed() || _lateral.isBusy()) {
                Diag::error("[Start] Lateral axis not ready");
                return true;
            }

            _stepper.resetTurns();
            _planner.reset();
            _verifyLowPending = true;
            _verifyHighPending = true;
            _positioningToLow = true;
            _state = WindingState::PAUSED;
            _pendingDisable = false;
            _lateral.prepareStartPosition(_windingStartMm());
            Diag::infof("[START] Positioning to low bound %.2f mm", _windingStartMm());
            return true;
        }
        if (_state == WindingState::PAUSED) {
            if (_positioningToLow) {
                Diag::warn("[Start] Ignored - carriage still positioning to low bound");
                return true;
            }
            _toWinding();
            return true;
        }
        return true;
    }

    if (strcmp(cmd, "pause") == 0) {
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            Diag::info("[Pause] Ignored - no active session");
        } else {
            _pauseRequested = true;
            Diag::info("[Pause] Pause requested");
        }
        return true;
    }

    if (strcmp(cmd, "resume") == 0) {
        if (_state == WindingState::PAUSED) {
            _toWinding();
            Diag::info("[Resume] Transitioning to WINDING");
        } else {
            Diag::infof("[Resume] Ignored in state %s", windingStateName(_state));
        }
        return true;
    }

    if (strcmp(cmd, "max_rpm") == 0 || strcmp(cmd, "max-rpm") == 0) {
        const int rpm = constrain((int)strtol(value, nullptr, 10), 10, 1500);
        _maxSpeedHz = (uint32_t)rpm * (uint32_t)STEPS_PER_REV / 60UL;
        Diag::infof("[MaxRPM] (immediate) Set %d RPM -> %u Hz", rpm, (unsigned)_maxSpeedHz);
        return true;
    }

    if (strcmp(cmd, "stop_next_high") == 0) {
        _lateral.armStopAtNextHigh();
        Diag::info("[Mode] Stop armed on next high bound");
        return true;
    }

    if (strcmp(cmd, "stop_next_low") == 0) {
        _lateral.armStopAtNextLow();
        Diag::info("[Mode] Stop armed on next low bound");
        return true;
    }

    if (strcmp(cmd, "end_pos") == 0) {
        _recipe.endPos = windingEndPosFromString(value);
        _saveRecipe();
        Diag::infof("[END_POS] Position finale: %s", windingEndPosKey(_recipe.endPos));
        return true;
    }

    if (strcmp(cmd, "end_pos_turns") == 0) {
        _recipe.endPosTurns = (int)constrain((int)strtol(value, nullptr, 10), 1, 20);
        _saveRecipe();
        Diag::infof("[END_POS] Tours finaux: %d", _recipe.endPosTurns);
        return true;
    }

    if (strcmp(cmd, "rodage_dist") == 0) {
        _rodageDistMm = constrain(atof(value), 5.0f, (float)LAT_TRAVERSE_MM);
        Diag::infof("[RODAGE] Distance: %.1f mm", _rodageDistMm);
        return true;
    }

    if (strcmp(cmd, "rodage_passes") == 0) {
        _rodagePasses = (int)constrain((int)strtol(value, nullptr, 10), 1, 200);
        Diag::infof("[RODAGE] Passes: %d", _rodagePasses);
        return true;
    }

    if (strcmp(cmd, "rodage") == 0) {
        if (_state == WindingState::IDLE) {
            _toRodage();
        } else {
            Diag::infof("[RODAGE] Ignore - etat actuel: %s", windingStateName(_state));
        }
        return true;
    }

    if (strcmp(cmd, "rodage_stop") == 0) {
        if (_state == WindingState::RODAGE) {
            _toIdle();
            Diag::info("[RODAGE] Arret demande par l'operateur");
        }
        return true;
    }

    return false;
}

void WinderApp::handleCommand(const char* cmd, const char* value) {
#if DIAG_VERBOSE
    Diag::infof("[APP-CMD] cmd='%s' val='%s'", cmd, value);
#endif

    if (_handleImmediateCommand(cmd, value)) return;
    if (_handleGeometryCommand(cmd, value)) return;

    if (strcmp(cmd, "target") == 0) {
        const long t = strtol(value, nullptr, 10);
        if (t > 0) {
            _targetTurns = t;
            if (_state == WindingState::TARGET_REACHED && t > _stepper.getTurns()) {
                _toPaused();
                Diag::info("[PAUSED] Target raised - resume possible");
            }
            Diag::infof("Target: %ld turns", t);
            _saveRecipe();
        }
        return;
    }

    if (_parametersLocked()) {
        Diag::infof("[Lock] Ignored during session (%s): %s", windingStateName(_state), cmd);
        return;
    }

    if (strcmp(cmd, "freerun") == 0) {
        _freerun = (strcmp(value, "true") == 0);
        Diag::infof("Mode: %s", _freerun ? "FreeRun" : "Target");
        _saveRecipe();
        return;
    }

    if (strcmp(cmd, "direction") == 0) {
        const Direction newDir = (strcmp(value, "cw") == 0) ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            Diag::infof("Direction: %s", value);
            _saveRecipe();
        }
        return;
    }

    if (_handlePatternCommand(cmd, value)) return;

    if (strcmp(cmd, "recipe_import") == 0) {
        WindingRecipe imported;
        if (_recipeStore.fromJson(value, imported)) {
            _applyRecipe(imported, true);
            Diag::info("[Recipe] Recipe imported");
        } else {
            Diag::error("[Recipe] ERROR - invalid JSON");
        }
        return;
    }

    if (strcmp(cmd, "lat_offset") == 0) {
        const float mm = constrain((float)atof(value), 0.0f, (float)LAT_TRAVERSE_MM);
        _lateral.setHomeOffset(mm);
        _toIdle();
        _lateral.rehome();
        Diag::infof("[Lateral] Offset %.2f mm - rehoming started", mm);
        _saveRecipe();
        return;
    }
}
