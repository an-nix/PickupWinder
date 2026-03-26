#pragma once
#include <Arduino.h>
#include "Config.h"
#include "WindingGeometry.h"

enum class WindingStyle : uint8_t {
	STRAIGHT = 0,
	SCATTER  = 1,
	HUMAN    = 2,
};

// Position finale du chariot en fin de bobinage.
// HIGH/LOW : le chariot est ramené sur la bute correspondante N tours avant
// la fin pour bloquer le fil ; ces derniers tours se font sur place (no traverse).
// NOTE : éviter HIGH/LOW qui sont des macros Arduino (#define HIGH 0x1 / LOW 0x0)
enum class WindingEndPos : uint8_t {
	NONE  = 0,  // Comportement actuel : aucune position finale imposée
	TOP   = 1,  // Terminer sur la butée haute (bloquer le fil)
	BOTTOM = 2, // Terminer sur la butée basse (bloquer le fil)
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
	float           latOffsetMm        = LAT_HOME_OFFSET_DEFAULT_MM;
	WindingStyle    style              = WindingStyle::STRAIGHT;
	uint32_t        seed               = WINDING_DEFAULT_SEED;
	float           layerJitterPct     = WINDING_LAYER_JITTER_DEFAULT;
	float           layerSpeedPct      = WINDING_LAYER_SPEED_JITTER_DEFAULT;
	float           humanTraversePct   = WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT;
	float           humanSpeedPct      = WINDING_HUMAN_SPEED_JITTER_DEFAULT;
	// Multiplier applied only to the first traverse pass speed.
	// 1.0 = unchanged, <1 slower, >1 faster.
	float           firstPassTraverseFactor = 1.0f;
	// Position finale du chariot (aucune / butée haute / butée basse).
	WindingEndPos   endPos             = WindingEndPos::NONE;
	// Nombre de tours effectués sur la position finale avant l'arrêt.
	int             endPosTurns        = 3;
};

/**
 * @brief Per-pass traverse planning output.
 */
struct TraversePlan {
	/** Turns to execute before lateral reversal. */
	long     turnsPerPass = 1;
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

	/** @brief Localized display name for style enum. */
	static const char* styleName(WindingStyle style);
	/** @brief Stable lowercase serialization key for style enum. */
	static String      styleKey(WindingStyle style);
	/** @brief Parse style key into enum value. */
	static WindingStyle styleFromString(const String& value);

private:
	WindingRecipe _recipe;

	static uint32_t _mix(uint32_t x);
	static float    _noiseSigned(uint32_t seed, uint32_t a, uint32_t b);
	static float    _smoothNoise(uint32_t seed, uint32_t passIndex, float x);
};
