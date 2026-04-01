#include "WinderApp.h"

#include <Arduino.h>

#include "Diag.h"

WinderStatus WinderApp::getStatus() const {
    const bool sessionActive = (_state != WindingState::IDLE);
    const bool motorEnabled = (_state == WindingState::WINDING);

    return {
        _stepper.getRPM(),
        _stepper.getSpeedHz(),
        _stepper.getTurns(),
        (long)_targetTurns,
        (uint16_t)((_maxSpeedHz * 60UL + (STEPS_PER_REV / 2)) / STEPS_PER_REV),
        _stepper.isRunning(),
        motorEnabled,
        sessionActive,
        _lateral.isPositionedForStart(),
        (_verifyLowPending || _positioningToLow),
        _verifyHighPending,
        (_state == WindingState::RODAGE),
        _rodagePassDone,
        _rodagePasses,
        _rodageDistMm,
        (bool)_freerun,
        (_direction == Direction::CW),
        (float)_geom.turnsPerPass(),
        (float)_geom.turnsPerPassCalc(),
        (float)_geom.turnsPerPassOffset,
        _geom.scatterFactor,
        (int)_lateral.getPassCount(),
        _activePlan.turnsPerPass,
        _activePlan.speedScale,
        _lateral.getTraversalProgress(),
        _lateral.getCurrentPositionMm(),
        _windingStartMm(), _windingEndMm(),
        _geom.windingStartTrim_mm, _geom.windingEndTrim_mm,
        _geom.effectiveWidth(),
        _geom.totalWidth_mm, _geom.flangeBottom_mm, _geom.flangeTop_mm,
        _geom.margin_mm, _geom.wireDiameter_mm,
        _lateral.getHomeOffset(),
        WindingPatternPlanner::styleKey(_recipe.style),
        _recipe.seed,
        _recipe.layerJitterPct,
        _recipe.layerSpeedPct,
        _recipe.humanTraversePct,
        _recipe.humanSpeedPct,
        _recipe.firstPassTraverseFactor,
        (int)_recipe.endPos,
        _recipe.endPosTurns,
        windingStateName(_state),
    };
}

void WinderApp::recipeJson(char* buf, size_t len) const {
    if (!buf || len == 0) return;
    _recipeStore.toJson(_captureRecipe(), buf, len);
}

void WinderApp::handleEncoderDelta(int32_t delta) {
    if (delta == 0) return;

    if (_state == WindingState::PAUSED) {
        const float currentPos = _lateral.getCurrentPositionMm();
        if (fabsf(currentPos - _windingEndMm()) < 0.5f) {
            const float step = delta * ENC_STEP_MM;
            _lateral.jog(step);
            const float newPos = _lateral.getTargetPositionMm();
            _geom.windingEndTrim_mm = constrain(
                newPos - (_geom.totalWidth_mm - _geom.flangeTop_mm - _geom.margin_mm), -5.0f, 5.0f);
            _saveRecipe();
            Diag::infof("[Encoder] Butee haute ajustee en PAUSE: %.2f mm", newPos);
        } else if (fabsf(currentPos - _windingStartMm()) < 0.5f) {
            const float step = delta * ENC_STEP_MM;
            _lateral.jog(step);
            const float newPos = _lateral.getTargetPositionMm();
            _geom.windingStartTrim_mm = constrain(
                newPos - (_geom.flangeBottom_mm + _geom.margin_mm), -5.0f, 5.0f);
            _saveRecipe();
            Diag::infof("[Encoder] Butee basse ajustee en PAUSE: %.2f mm", newPos);
        }
    }
}
