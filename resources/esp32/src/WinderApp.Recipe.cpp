#include "WinderApp.h"

#include <Arduino.h>

#include "Protocol.h"

TraversePlan WinderApp::_buildTraversePlan(uint32_t windingHz) const {
    (void)windingHz;
    return _planner.getPlan(_stepper.getTurns(), _lateral.getTraversalProgress());
}

void WinderApp::_applyRecipe(const WindingRecipe& recipe, bool persist) {
    _recipe = recipe;
    _geom = recipe.geometry;
    _targetTurns = recipe.targetTurns;
    _freerun = recipe.freerun;
    _direction = recipe.directionCW ? Direction::CW : Direction::CCW;
    _state = WindingState::IDLE;
    _pendingDisable = false;
    _lateral.setHomeOffset(recipe.latOffsetMm);
    _planner.setRecipe(_recipe);
    _activePlan = _planner.getPlan(_stepper.getTurns(), 0.0f);
    if (persist) _saveRecipe();
}

WindingRecipe WinderApp::_captureRecipe() const {
    WindingRecipe recipe = _recipe;
    recipe.version = PICKUP_RECIPE_FORMAT_VERSION;
    recipe.targetTurns = _targetTurns;
    recipe.freerun = _freerun;
    recipe.directionCW = (_direction == Direction::CW);
    recipe.geometry = _geom;
    recipe.latOffsetMm = _lateral.getHomeOffset();
    return recipe;
}

void WinderApp::_saveRecipe() {
    _recipe = _captureRecipe();
    _planner.setRecipe(_recipe);
    _recipeStore.save(_recipe);
}
