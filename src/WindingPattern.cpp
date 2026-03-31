#include "WindingPattern.h"

namespace {
/**
 * @brief Clamp a floating-point value to an inclusive range.
 *
 * Helper placed in an anonymous namespace to restrict linkage to this
 * translation unit. Use when a value must be constrained to a known range.
 *
 * @param value The value to clamp.
 * @param lo The lower inclusive bound.
 * @param hi The upper inclusive bound.
 * @return The clamped value: `lo` if value < lo, `hi` if value > hi,
 *         otherwise `value`.
 * @par Usage
 * Internal helper used throughout this file to constrain computed scales
 * and progress values. Not exposed outside this translation unit.
 */
float clampf(float value, float lo, float hi) {
	return (value < lo) ? lo : ((value > hi) ? hi : value);
}
}

/**
 * @brief Fast non-cryptographic 32-bit integer mixer.
 *
 * Produces a well-mixed 32-bit value from the input. Intended for
 * deterministic pseudo-random generation and hashing inside this planner.
 *
 * @param x Input 32-bit integer to mix.
 * @return Mixed 32-bit integer.
 * @par Usage
 * Internal mixing primitive used only by the planner's noise functions
 * (`_noiseSigned`) to produce deterministic pseudo-random values.
 */
uint32_t WindingPatternPlanner::_mix(uint32_t x) {
	// Small 32-bit integer mixing function to create pseudo-randomness.
	// This is a fast, non-cryptographic mixer used to decorrelate bits.
	x ^= x >> 16;
	x *= 0x7feb352dU;
	x ^= x >> 15;
	x *= 0x846ca68bU;
	x ^= x >> 16;
	return x;
}
// Noise helpers removed - planner uses straight-only behaviour (no scatter/human).

/**
 * @brief Compute a traverse plan for the current winding state.
 *
 * This function inspects the planner's recipe and the number of turns
 * already completed to derive per-pass parameters such as the number
 * of turns-per-pass and a speed multiplier. The returned TraversePlan
 * contains fields used by the motion controller when executing the pass.
 *
 * @param turnsDone Total number of turns already wound (used to derive
 *                  the current pass index).
 * @param progressInPass Normalized progress inside the current pass [0,1].
 * @return A TraversePlan with `turnsPerPass`, `speedScale`, and `passIndex`.
 * @note The result depends on the recipe's style (STRAIGHT, SCATTER, HUMAN)
 *       and various jitter/speed percentage parameters present in the recipe.
 * @par Usage
 * Queried by the application to obtain the current traverse plan. Callers
 * include `WinderApp::begin()`, `WinderApp::_toIdle()`,
 * `WinderApp::_buildTraversePlan()` (the main runtime path), and
 * `WinderApp::_applyRecipe()` when the recipe changes.
 */
TraversePlan WindingPatternPlanner::getPlan(long turnsDone, float progressInPass) const {
	TraversePlan plan;
	float baseTpp = _recipe.geometry.turnsPerPassFloat();
	plan.turnsPerPass = max(1.0f, baseTpp);
	if (baseTpp <= 0.0f) return plan;

	// Normalize progress and compute pass index from completed turns.
	progressInPass = clampf(progressInPass, 0.0f, 1.0f);
	uint32_t passIndex = (uint32_t)floorf((float)turnsDone / baseTpp);
	plan.passIndex = passIndex;

	// Straight-only behaviour: no scatter/human perturbations.
	float tppScale = 1.0f;
	float speedScale = 1.0f;

	// Optional dedicated multiplier for the very first traverse pass.
	if (passIndex == 0) {
		speedScale *= _recipe.firstPassTraverseFactor;
	}

	// Constrain scales to reasonable bounds.
	tppScale = clampf(tppScale, 0.55f, 1.60f);
	speedScale = clampf(speedScale, 0.55f, 1.60f);

	float tppValue = baseTpp * tppScale;
	if (fabsf(tppValue - roundf(tppValue)) < 0.01f) tppValue += 0.12f;
	plan.turnsPerPass = max(0.55f, tppValue);
	plan.speedScale = speedScale;
	return plan;
}

/**
 * @brief Get a human-readable display name for a winding style.
 *
 * Names are localized (French) and suitable for UI labels and log messages.
 *
 * @param style Winding style enum value.
 * @return Null-terminated C string with a short display name.
 * @par Usage
 * Used by the UI/logging code in `WinderApp` (e.g. `WinderApp::begin()` and
 * numerous `Diag::infof` calls) to display the active profile name.
 */
/**
 * @brief Get a stable serialization key for a `WindingStyle`.
 *
 * These lower-case keys are used for storing the recipe and for the web/UI
 * protocol. They are stable and suitable for JSON fields.
 *
 * @param style Winding style enum.
 * @return String key such as "straight", "scatter" or "human".
 * @par Usage
 * Used by `WinderApp::getStatus()` to expose the current style to the UI
 * and by `WindingRecipeStore` when serializing recipes to JSON.
 */
/**
 * @brief Parse a style key string back to the enum value.
 *
 * Accepts lower-case keys used by `styleKey`. Unknown or unrecognized
 * values are mapped to `WindingStyle::STRAIGHT` as a safe default.
 *
 * @param value Key string such as "scatter" or "human".
 * @return Corresponding `WindingStyle` enum value.
 * @par Usage
 * Used when importing recipes from JSON (`WindingRecipeStore::fromJson`) and
 * when receiving pattern commands in `WinderApp::_handlePatternCommand()`.
 */
// Style helpers removed; planner implements straight-only behaviour.
