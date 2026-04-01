#include "WinderApp.h"

#include <Arduino.h>

#include "Diag.h"

void WinderApp::_refreshCarriageForGeometryChange(bool startBoundChanged, bool endBoundChanged) {
    if (!_lateral.isHomed() || _lateral.isBusy()) return;

    if (_stepper.isRunning()) {
        Diag::info("[WINDOW_SHIFT] Running - geometry updated, reposition deferred");
        return;
    }

    switch (_state) {
    case WindingState::PAUSED:
        if (startBoundChanged && fabsf(_lateral.getCurrentPositionMm() - _windingStartMm()) <= 1.0f) {
            _lateral.prepareStartPosition(_windingStartMm());
        } else if (endBoundChanged && fabsf(_lateral.getCurrentPositionMm() - _windingEndMm()) <= 1.0f) {
            _lateral.prepareStartPosition(_windingEndMm());
        }
        break;

    case WindingState::TARGET_REACHED:
    case WindingState::IDLE:
    default:
        break;
    }
}

bool WinderApp::_handleGeometryCommand(const char* cmd, const char* value) {
    if (strcmp(cmd, "geom_start_trim") == 0) {
        _geom.windingStartTrim_mm = constrain(atof(value), -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(true, false);
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "geom_end_trim") == 0) {
        _geom.windingEndTrim_mm = constrain((float)atof(value), -5.0f, 5.0f);
        _refreshCarriageForGeometryChange(false, true);
        _saveRecipe();
        return true;
    }

    auto applyTrimShift = [&](float dStart, float dEnd, const char* label) {
        const float prevStart = _geom.windingStartTrim_mm;
        const float prevEnd = _geom.windingEndTrim_mm;
        _geom.windingStartTrim_mm = constrain(prevStart + dStart, -5.0f, 5.0f);
        _geom.windingEndTrim_mm = constrain(prevEnd + dEnd, -5.0f, 5.0f);

        _refreshCarriageForGeometryChange(dStart != 0.0f, dEnd != 0.0f);
        _saveRecipe();

        Diag::infof("[%s] startTrim old=%.3f -> new=%.3f endTrim old=%.3f -> new=%.3f",
                    label,
                    prevStart, _geom.windingStartTrim_mm,
                    prevEnd, _geom.windingEndTrim_mm);
    };

    const bool isWindowShift = (strcmp(cmd, "window_shift") == 0)
                            || (strcmp(cmd, "windows_shift") == 0)
                            || (strncmp(cmd, "window_shift", strlen("window_shift")) == 0);

    if (isWindowShift) {
        float delta = constrain(atof(value), -5.0f, 5.0f);
        if (strstr(cmd, "nudge") != nullptr) delta = constrain(delta, -1.0f, 1.0f);
        applyTrimShift(delta, delta, "WINDOW_SHIFT");
        return true;
    }

    if (strcmp(cmd, "geom_start_trim_nudge") == 0) {
        const float delta = constrain(atof(value), -1.0f, 1.0f);
    #if DIAG_VERBOSE
        Diag::infof("[DEBUG] geom_start_trim_nudge: value=%.3f, prev=%.3f", delta, _geom.windingStartTrim_mm);
    #endif
        applyTrimShift(delta, 0.0f, "STRT_TRIM_NUDGE");
        return true;
    }

    if (strcmp(cmd, "geom_end_trim_nudge") == 0) {
        const float delta = constrain(atof(value), -1.0f, 1.0f);
    #if DIAG_VERBOSE
        Diag::infof("[DEBUG] geom_end_trim_nudge: value=%.3f, prev=%.3f", delta, _geom.windingEndTrim_mm);
    #endif
        applyTrimShift(0.0f, delta, "END_TRIM_NUDGE");
        return true;
    }

    if (strcmp(cmd, "geom_preset") == 0) {
        const uint8_t idx = (uint8_t)strtol(value, nullptr, 10);
        if (idx >= BOBBIN_PRESET_COUNT) {
            Diag::warnf("[Preset] Index %u out of range (max %u)", (unsigned)idx, (unsigned)(BOBBIN_PRESET_COUNT - 1));
            return true;
        }
        _geom.applyPreset(idx);
        _refreshCarriageForGeometryChange(true, true);
        Diag::infof("Bobbin preset: %s - %ld turns/pass", BOBBIN_PRESETS[idx].name, _geom.turnsPerPass());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "geom_total") == 0) {
        _geom.totalWidth_mm = constrain((float)atof(value), 0.0f, 200.0f);
        _refreshCarriageForGeometryChange(true, true);
        _saveRecipe();
        return true;
    }
    if (strcmp(cmd, "geom_bottom") == 0) {
        _geom.flangeBottom_mm = constrain((float)atof(value), 0.0f, 50.0f);
        _refreshCarriageForGeometryChange(true, false);
        _saveRecipe();
        return true;
    }
    if (strcmp(cmd, "geom_top") == 0) {
        _geom.flangeTop_mm = constrain((float)atof(value), 0.0f, 50.0f);
        _refreshCarriageForGeometryChange(false, true);
        _saveRecipe();
        return true;
    }
    if (strcmp(cmd, "geom_margin") == 0) {
        _geom.margin_mm = constrain((float)atof(value), 0.0f, 20.0f);
        _refreshCarriageForGeometryChange(true, true);
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "geom_wire") == 0) {
        _geom.wireDiameter_mm = constrain((float)atof(value), 0.01f, 1.0f);
        Diag::infof("Wire: %.4f mm - %ld turns/pass (calc: %ld)",
                    _geom.wireDiameter_mm, _geom.turnsPerPass(), _geom.turnsPerPassCalc());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "geom_tpp_ofs") == 0) {
        _geom.turnsPerPassOffset = constrain((int)strtol(value, nullptr, 10), -2000, 2000);
        Diag::infof("Turns/pass offset: %+ld (calc %ld -> effective %ld)",
                    _geom.turnsPerPassOffset, _geom.turnsPerPassCalc(), _geom.turnsPerPass());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "geom_scatter") == 0) {
        _geom.scatterFactor = constrain((float)atof(value), 0.5f, 5.0f);
        Diag::infof("Scatter factor: %.2f -> %ld turns/pass", _geom.scatterFactor, _geom.turnsPerPass());
        _saveRecipe();
        return true;
    }

    return false;
}

bool WinderApp::_handlePatternCommand(const char* cmd, const char* value) {
    if (strcmp(cmd, "winding_style") == 0) {
        _recipe.style = WindingPatternPlanner::styleFromString(value);
        _planner.setRecipe(_captureRecipe());
        Diag::infof("Winding style: %s", WindingPatternPlanner::styleName(_recipe.style));
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_seed") == 0) {
        const long parsed = strtol(value, nullptr, 10);
        _recipe.seed = (uint32_t)((parsed > 0) ? parsed : 1);
        _planner.setRecipe(_captureRecipe());
        Diag::infof("Winding seed: %lu", (unsigned long)_recipe.seed);
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_layer_jitter") == 0) {
        _recipe.layerJitterPct = constrain(atof(value), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_layer_speed") == 0) {
        _recipe.layerSpeedPct = constrain(atof(value), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_human_traverse") == 0) {
        _recipe.humanTraversePct = constrain(atof(value), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_human_speed") == 0) {
        _recipe.humanSpeedPct = constrain(atof(value), 0.0f, 0.45f);
        _planner.setRecipe(_captureRecipe());
        _saveRecipe();
        return true;
    }

    if (strcmp(cmd, "winding_first_pass_traverse") == 0) {
        _recipe.firstPassTraverseFactor = constrain(atof(value), 0.40f, 1.80f);
        _planner.setRecipe(_captureRecipe());
        Diag::infof("First pass traverse factor: %.2f", _recipe.firstPassTraverseFactor);
        _saveRecipe();
        return true;
    }

    return false;
}
