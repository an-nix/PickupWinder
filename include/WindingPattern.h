#pragma once
#include <Arduino.h>
#include "Config.h"
#include "WindingGeometry.h"

enum class WindingStyle : uint8_t {
	STRAIGHT = 0,
	SCATTER  = 1,
	HUMAN    = 2,
};

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
};

struct TraversePlan {
	long     turnsPerPass = 1;
	float    speedScale   = 1.0f;
	uint32_t passIndex    = 0;
};

class WindingPatternPlanner {
public:
	void setRecipe(const WindingRecipe& recipe) { _recipe = recipe; }
	void reset() {}

	TraversePlan getPlan(long turnsDone, float progressInPass) const;

	static const char* styleName(WindingStyle style);
	static String      styleKey(WindingStyle style);
	static WindingStyle styleFromString(const String& value);

private:
	WindingRecipe _recipe;

	static uint32_t _mix(uint32_t x);
	static float    _noiseSigned(uint32_t seed, uint32_t a, uint32_t b);
	static float    _smoothNoise(uint32_t seed, uint32_t passIndex, float x);
};
