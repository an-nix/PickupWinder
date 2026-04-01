#include "WinderApp.h"
#include "CommandRegistry.h"

#include <Arduino.h>

#include "Diag.h"

bool WinderApp::_handleImmediateCommand(CommandId id, const char* value) {
    switch (id) {
    case CommandId::Stop:
        _toIdle();
        return true;
    case CommandId::Reset:
        _toIdle();
        Diag::info("[IDLE] Turn counter reset");
        return true;
    case CommandId::Start:
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
    case CommandId::Pause:
        if (_state == WindingState::IDLE || _state == WindingState::TARGET_REACHED) {
            Diag::info("[Pause] Ignored - no active session");
        } else {
            _pauseRequested = true;
            Diag::info("[Pause] Pause requested");
        }
        return true;
    case CommandId::Resume:
        if (_state == WindingState::PAUSED) {
            _toWinding();
            Diag::info("[Resume] Transitioning to WINDING");
        } else {
            Diag::infof("[Resume] Ignored in state %s", windingStateName(_state));
        }
        return true;
    case CommandId::MaxRpm:
        {
            const int rpm = constrain((int)strtol(value, nullptr, 10), 10, 1500);
            _maxSpeedHz = (uint32_t)rpm * (uint32_t)STEPS_PER_REV / 60UL;
            Diag::infof("[MaxRPM] (immediate) Set %d RPM -> %u Hz", rpm, (unsigned)_maxSpeedHz);
            return true;
        }
    case CommandId::StopNextHigh:
        _lateral.armStopAtNextHigh();
        Diag::info("[Mode] Stop armed on next high bound");
        return true;
    case CommandId::StopNextLow:
        _lateral.armStopAtNextLow();
        Diag::info("[Mode] Stop armed on next low bound");
        return true;
    case CommandId::EndPos:
        _recipe.endPos = windingEndPosFromString(value);
        _saveRecipe();
        Diag::infof("[END_POS] Position finale: %s", windingEndPosKey(_recipe.endPos));
        return true;
    case CommandId::EndPosTurns:
        _recipe.endPosTurns = (int)constrain((int)strtol(value, nullptr, 10), 1, 20);
        _saveRecipe();
        Diag::infof("[END_POS] Tours finaux: %d", _recipe.endPosTurns);
        return true;
    case CommandId::RodageDist:
        _rodageDistMm = constrain(atof(value), 5.0f, (float)LAT_TRAVERSE_MM);
        Diag::infof("[RODAGE] Distance: %.1f mm", _rodageDistMm);
        return true;
    case CommandId::RodagePasses:
        _rodagePasses = (int)constrain((int)strtol(value, nullptr, 10), 1, 200);
        Diag::infof("[RODAGE] Passes: %d", _rodagePasses);
        return true;
    case CommandId::Rodage:
        if (_state == WindingState::IDLE) {
            _toRodage();
        } else {
            Diag::infof("[RODAGE] Ignore - etat actuel: %s", windingStateName(_state));
        }
        return true;
    case CommandId::RodageStop:
        if (_state == WindingState::RODAGE) {
            _toIdle();
            Diag::info("[RODAGE] Arret demande par l'operateur");
        }
        return true;
    default:
        return false;
    }
}

void WinderApp::handleCommand(const char* cmd, const char* value) {
    const CommandDefinition* def = CommandRegistry::findByKey(cmd);
    if (!def) {
        return;
    }
    handleCommand(def->id, value);
}

void WinderApp::handleCommand(CommandId id, const char* value) {
#if DIAG_VERBOSE
    Diag::infof("[APP-CMD] cmdId=%d val='%s'", (int)id, value);
#endif

    if (_handleImmediateCommand(id, value)) return;
    if (_handleGeometryCommand(id, value)) return;

    switch (id) {
    case CommandId::Target: {
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
    case CommandId::Freerun:
        _freerun = (strcmp(value, "true") == 0);
        Diag::infof("Mode: %s", _freerun ? "FreeRun" : "Target");
        _saveRecipe();
        return;
    case CommandId::Direction: {
        const Direction newDir = (strcmp(value, "cw") == 0) ? Direction::CW : Direction::CCW;
        if (newDir != _direction) {
            _direction = newDir;
            Diag::infof("Direction: %s", value);
            _saveRecipe();
        }
        return;
    }
    case CommandId::RecipeImport: {
        WindingRecipe imported;
        if (_recipeStore.fromJson(value, imported)) {
            _applyRecipe(imported, true);
            Diag::info("[Recipe] Recipe imported");
        } else {
            Diag::error("[Recipe] ERROR - invalid JSON");
        }
        return;
    }
    case CommandId::LatOffset: {
        const float mm = constrain((float)atof(value), 0.0f, (float)LAT_TRAVERSE_MM);
        _lateral.setHomeOffset(mm);
        _toIdle();
        _lateral.rehome();
        Diag::infof("[Lateral] Offset %.2f mm - rehoming started", mm);
        _saveRecipe();
        return;
    }
    default:
        break;
    }

    if (_parametersLocked()) {
        Diag::infof("[Lock] Ignored during session (%s)", windingStateName(_state));
        return;
    }

    if (_handlePatternCommand(id, value)) return;
}
