#pragma once
#include <Arduino.h>
#include "Config.h"
#include "WindingGeometry.h"

// Winding styles removed; planner uses straight-only behaviour.

// Desired final carriage position at the end of winding.
// HIGH/LOW return the carriage to the corresponding bound a few turns before
// completion so the final turns are wound in place without traversal.
// NOTE: avoid using HIGH/LOW as enum values because Arduino defines them as macros.
enum class WindingEndPos : uint8_t {
	NONE  = 0,  // Default behavior: no forced final position.
	TOP   = 1,  // Finish on the high bound to lock the wire.
	BOTTOM = 2, // Finish on the low bound to lock the wire.
};

inline const char* windingEndPosKey(WindingEndPos p) {
	switch (p) {
		case WindingEndPos::TOP:    return "high";
		case WindingEndPos::BOTTOM: return "low";
		default:                    return "none";
	}
}

/**
 * @brief Parse end-position key from string.
 * @param s Input key (`none`, `high`, `low`).
 * @return Parsed enum value, `NONE` on unknown input.
 */
inline WindingEndPos windingEndPosFromString(const String& s) {
	if (s == "high") return WindingEndPos::TOP;
	if (s == "low")  return WindingEndPos::BOTTOM;
	return WindingEndPos::NONE;
}

/**
 * @brief Complete winding recipe persisted to storage and used at runtime.
 */
struct WindingRecipe {
	uint32_t        version            = 1;
	long            targetTurns        = DEFAULT_TARGET_TURNS;
	bool            freerun            = false;
	bool            directionCW        = true;
	WindingGeometry geometry           = {};
	float           latOffset_mm        = LAT_HOME_OFFSET_DEFAULT_MM;
	// Style selection removed; planner uses straight-only behaviour.
	uint32_t        seed               = WINDING_DEFAULT_SEED;
	float           layerJitterPct     = WINDING_LAYER_JITTER_DEFAULT;
	float           layerSpeedPct      = WINDING_LAYER_SPEED_JITTER_DEFAULT;
	float           humanTraversePct   = WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT;
	float           humanSpeedPct      = WINDING_HUMAN_SPEED_JITTER_DEFAULT;
	// Multiplier applied only to the first traverse pass speed.
	// 1.0 = unchanged, <1 slower, >1 faster.
	float           firstPassTraverseFactor = 1.0f;
	// Desired final carriage location (none / high bound / low bound).
	WindingEndPos   endPos             = WindingEndPos::NONE;
	// Number of turns executed while holding the final position before stop.
	int             endPosTurns        = 3;
};

/**
 * @brief Per-pass traverse planning output.
 */
struct TraversePlan {
	/** Turns to execute before lateral reversal. */
	float    turnsPerPass = 1.0f;
	/** Multiplicative traverse speed factor. */
	float    speedScale   = 1.0f;
	/** Zero-based pass index currently planned. */
	uint32_t passIndex    = 0;
};

/**
 * @brief Planner generating winding traverse variations from recipe state.
 */
class WindingPatternPlanner {
public:
	/**
	 * @brief Set active recipe used for planning.
	 * @param recipe Recipe snapshot.
	 */
	void setRecipe(const WindingRecipe& recipe) { _recipe = recipe; }

	/**
	 * @brief Reset planner internal state.
	 */
	void reset() {}

	/**
	 * @brief Build current traverse plan.
	 * @param turnsDone Total spindle turns already completed.
	 * @param progressInPass Current normalized pass progress in `[0,1]`.
	 * @return Traverse plan for the current pass.
	 */
	TraversePlan getPlan(long turnsDone, float progressInPass) const;

	// Style helper methods removed; only straight behaviour is supported.

private:
	WindingRecipe _recipe;

	static uint32_t _mix(uint32_t x);
	static float    _noiseSigned(uint32_t seed, uint32_t a, uint32_t b);
	static float    _smoothNoise(uint32_t seed, uint32_t passIndex, float x);
};
