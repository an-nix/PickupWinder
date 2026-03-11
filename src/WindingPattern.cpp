#include "WindingPattern.h"

namespace {
float clampf(float value, float lo, float hi) {
	return (value < lo) ? lo : ((value > hi) ? hi : value);
}
}

uint32_t WindingPatternPlanner::_mix(uint32_t x) {
	x ^= x >> 16;
	x *= 0x7feb352dU;
	x ^= x >> 15;
	x *= 0x846ca68bU;
	x ^= x >> 16;
	return x;
}

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
	float a = _noiseSigned(seed, passIndex, 100 + i0);
	float b = _noiseSigned(seed, passIndex, 100 + i1);
	return a + (b - a) * s;
}

TraversePlan WindingPatternPlanner::getPlan(long turnsDone, float progressInPass) const {
	TraversePlan plan;
	long baseTpp = _recipe.geometry.turnsPerPass();
	plan.turnsPerPass = max(1L, baseTpp);
	if (baseTpp <= 0) return plan;

	progressInPass = clampf(progressInPass, 0.0f, 1.0f);
	uint32_t passIndex = (uint32_t)max(0L, turnsDone / max(1L, baseTpp));
	plan.passIndex = passIndex;

	float layerJitter = _noiseSigned(_recipe.seed, passIndex, 1) * _recipe.layerJitterPct;
	float layerSpeed  = _noiseSigned(_recipe.seed, passIndex, 2) * _recipe.layerSpeedPct;
	float tppScale    = 1.0f;
	float speedScale  = 1.0f;

	switch (_recipe.style) {
	case WindingStyle::STRAIGHT:
		break;

	case WindingStyle::SCATTER:
		tppScale   += layerJitter;
		speedScale += layerSpeed;
		break;

	case WindingStyle::HUMAN: {
		float humanTraverse = _smoothNoise(_recipe.seed + 17U, passIndex, progressInPass)
							* _recipe.humanTraversePct;
		float humanSpeed    = _smoothNoise(_recipe.seed + 31U, passIndex, progressInPass)
							* _recipe.humanSpeedPct;
		tppScale   += layerJitter + humanTraverse;
		speedScale += layerSpeed + humanSpeed;
		break;
	}
	}

	tppScale   = clampf(tppScale, 0.55f, 1.60f);
	speedScale = clampf(speedScale, 0.55f, 1.60f);

	plan.turnsPerPass = max(1L, lroundf((float)baseTpp * tppScale));
	plan.speedScale   = speedScale;
	return plan;
}

const char* WindingPatternPlanner::styleName(WindingStyle style) {
	switch (style) {
		case WindingStyle::STRAIGHT: return "Droit";
		case WindingStyle::SCATTER:  return "Scatter";
		case WindingStyle::HUMAN:    return "Humain";
		default:                     return "Droit";
	}
}

String WindingPatternPlanner::styleKey(WindingStyle style) {
	switch (style) {
		case WindingStyle::STRAIGHT: return "straight";
		case WindingStyle::SCATTER:  return "scatter";
		case WindingStyle::HUMAN:    return "human";
		default:                     return "straight";
	}
}

WindingStyle WindingPatternPlanner::styleFromString(const String& value) {
	if (value == "scatter") return WindingStyle::SCATTER;
	if (value == "human")   return WindingStyle::HUMAN;
	return WindingStyle::STRAIGHT;
}
