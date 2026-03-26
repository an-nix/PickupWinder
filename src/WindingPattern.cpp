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

/**
 * @brief Produce a deterministic pseudo-random value in [-1, 1].
 *
 * Combines a seed with two integer coordinates to generate a repeatable
 * pseudo-random float. Only the lower 24 bits of the hashed value are
 * used to create a uniform value in [0,1], then remapped to [-1,1].
 *
 * @param seed Global seed controlling repeatability.
 * @param a First coordinate / discriminator.
 * @param b Second coordinate / discriminator.
 * @return Deterministic pseudo-random float in the range [-1, 1].
 * @par Usage
 * Called by `_smoothNoise` and by the planner to introduce per-pass
 * variability. This function is internal to the planner and not exported.
 */
float WindingPatternPlanner::_noiseSigned(uint32_t seed, uint32_t a, uint32_t b) {
	uint32_t h = _mix(seed ^ _mix(a + 0x9e3779b9U) ^ _mix(b + 0x85ebca6bU));
	float unit = (float)(h & 0x00ffffffU) / 16777215.0f;
	return unit * 2.0f - 1.0f;
}

float WindingPatternPlanner::_smoothNoise(uint32_t seed, uint32_t passIndex, float x) {
	constexpr uint32_t SEGMENTS = 7;
	x = clampf(x, 0.0f, 1.0f) * (float)SEGMENTS;
	uint32_t i0 = (uint32_t)x;
	if (i0 >= SEGMENTS) i0 = SEGMENTS - 1;
	uint32_t i1 = i0 + 1;
	float t = x - (float)i0;
	float s = t * t * (3.0f - 2.0f * t);
	/**
	 * @brief Produce smooth interpolated noise over `SEGMENTS` using cubic
	 * Hermite interpolation (s = 3t^2 - 2t^3).
	 *
	 * The function samples `_noiseSigned` at two adjacent segment points and
	 * interpolates between them to get a continuous, smooth value for the
	 * given x in [0,1]. `passIndex` and `seed` ensure repeatability per pass.
	 * @par Usage
	 * Used by `getPlan()` to compute smoothly-varying human-style offsets.
	 * Called indirectly from `WinderApp::_buildTraversePlan()` and other
	 * WinderApp call sites when the planner is queried for traverse parameters.
	 */
	float a = _noiseSigned(seed, passIndex, 100 + i0);
	float b = _noiseSigned(seed, passIndex, 100 + i1);
	return a + (b - a) * s;
}

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
	long baseTpp = _recipe.geometry.turnsPerPass();
	plan.turnsPerPass = max(1L, baseTpp);
	if (baseTpp <= 0) return plan;

	// Ensure progress is in [0,1] and compute which traverse pass we're on
	// based on `turnsDone` and the base `turnsPerPass` from the geometry.
	progressInPass = clampf(progressInPass, 0.0f, 1.0f);
	uint32_t passIndex = (uint32_t)max(0L, turnsDone / max(1L, baseTpp));
	plan.passIndex = passIndex;

	// Derive per-pass perturbations from pseudo-random noise. These values
	// are scaled by recipe percentages to control the magnitude.
	float layerJitter = _noiseSigned(_recipe.seed, passIndex, 1) * _recipe.layerJitterPct;
	float layerSpeed  = _noiseSigned(_recipe.seed, passIndex, 2) * _recipe.layerSpeedPct;
	float tppScale    = 1.0f; // multiplier for turns-per-pass
	float speedScale  = 1.0f; // multiplier for traverse speed

	switch (_recipe.style) {
	case WindingStyle::STRAIGHT:
		break;

	case WindingStyle::SCATTER:
		// Scatter style adds layer-level jitter and speed variation.
		tppScale   += layerJitter;
		speedScale += layerSpeed;
		break;

	case WindingStyle::HUMAN: {
		// Human style adds smoothly-varying traverse and speed offsets that
		// change across the pass progress to mimic natural, non-uniform
		// winding behavior.
		float humanTraverse = _smoothNoise(_recipe.seed + 17U, passIndex, progressInPass)
							* _recipe.humanTraversePct;
		float humanSpeed    = _smoothNoise(_recipe.seed + 31U, passIndex, progressInPass)
							* _recipe.humanSpeedPct;
		tppScale   += layerJitter + humanTraverse;
		speedScale += layerSpeed + humanSpeed;
		break;
	}
	}

	// Optional dedicated multiplier for the very first traverse pass.
	if (passIndex == 0) {
		speedScale *= _recipe.firstPassTraverseFactor;
	}

	// Constrain scales to reasonable bounds to avoid extreme values.
	tppScale   = clampf(tppScale, 0.55f, 1.60f);
	speedScale = clampf(speedScale, 0.55f, 1.60f);

	// Compute final plan values: integer turns-per-pass and floating speed
	// scale. `lroundf` rounds to nearest long.
	plan.turnsPerPass = max(1L, lroundf((float)baseTpp * tppScale));
	plan.speedScale   = speedScale;
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
const char* WindingPatternPlanner::styleName(WindingStyle style) {
	switch (style) {
		// Return a human-readable name for the winding style. These are
		// used for display in the UI; names here are localized (French).
		case WindingStyle::STRAIGHT: return "Droit";
		case WindingStyle::SCATTER:  return "Scatter";
		case WindingStyle::HUMAN:    return "Humain";
		default:                     return "Droit";
	}
}

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
String WindingPatternPlanner::styleKey(WindingStyle style) {
	switch (style) {
		// Return a stable key string for each style used in settings or JSON
		// serialization. Keys are lower-case and consistent.
		case WindingStyle::STRAIGHT: return "straight";
		case WindingStyle::SCATTER:  return "scatter";
		case WindingStyle::HUMAN:    return "human";
		default:                     return "straight";
	}
}

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
WindingStyle WindingPatternPlanner::styleFromString(const String& value) {
	// Parse a style key string back into the enum. Unknown values default
	// to `STRAIGHT` to ensure safe behavior.
	if (value == "scatter") return WindingStyle::SCATTER;
	if (value == "human")   return WindingStyle::HUMAN;
	return WindingStyle::STRAIGHT;
}
