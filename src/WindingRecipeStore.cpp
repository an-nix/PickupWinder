#include "WindingRecipeStore.h"
#include <ArduinoJson.h>
#include <Preferences.h>

namespace {
constexpr const char* RECIPE_NS  = "winder";
constexpr const char* RECIPE_KEY = "recipe_json";
}

/**
 * @brief Initialize recipe store backend.
 */
void WindingRecipeStore::begin() {}

/**
 * @brief Load recipe from NVS.
 * @param recipe Output recipe.
 * @return true if load + parse succeeded.
 */
bool WindingRecipeStore::load(WindingRecipe& recipe) {
	// NOTE: 2 KB stack buffer — safe because load() is called only from
	// setup() (single-threaded) or controlTask (8 KB stack, low call depth).
	Preferences prefs;
	if (!prefs.begin(RECIPE_NS, true)) return false;
	size_t len = prefs.getBytesLength(RECIPE_KEY);
	if (len == 0 || len >= 2048) {
		prefs.end();
		return false;
	}
	char json[2048];
	size_t got = prefs.getBytes(RECIPE_KEY, json, len);
	prefs.end();
	if (got == 0 || got >= sizeof(json)) return false;
	json[got] = '\0';
	return fromJson(json, recipe);
}

/**
 * @brief Save recipe to NVS.
 * @param recipe Recipe to persist.
 * @return true when write length matches serialized length.
 */
bool WindingRecipeStore::save(const WindingRecipe& recipe) {
	// NOTE: 2 KB stack buffer — same constraint as load().
	char json[2048];
	toJson(recipe, json, sizeof(json));
	Preferences prefs;
	if (!prefs.begin(RECIPE_NS, false)) return false;
	size_t written = prefs.putBytes(RECIPE_KEY, json, strlen(json));
	prefs.end();
	return written == strlen(json);
}

/**
 * @brief Serialize recipe to compact JSON.
 * @param recipe Recipe input.
 * @param outBuf Caller-provided buffer for JSON output.
 * @param outBufLen Buffer size in bytes.
 */
void WindingRecipeStore::toJson(const WindingRecipe& recipe, char* outBuf, size_t outBufLen) const {
	if (!outBuf || outBufLen == 0) return;
	StaticJsonDocument<1024> doc;
	doc["version"] = PICKUP_RECIPE_FORMAT_VERSION;
	doc["targetTurns"] = recipe.targetTurns;
	doc["freerun"] = recipe.freerun;
	doc["directionCW"] = recipe.directionCW;
	doc["style"] = WindingPatternPlanner::styleKey(recipe.style);
	doc["seed"] = recipe.seed;
	doc["layerJitterPct"] = recipe.layerJitterPct;
	doc["layerSpeedPct"] = recipe.layerSpeedPct;
	doc["humanTraversePct"] = recipe.humanTraversePct;
	doc["humanSpeedPct"] = recipe.humanSpeedPct;
	doc["firstPassTraverseFactor"] = recipe.firstPassTraverseFactor;
	doc["latOffsetMm"] = recipe.latOffsetMm;
	doc["endPos"] = windingEndPosKey(recipe.endPos);
	doc["endPosTurns"] = recipe.endPosTurns;

	JsonObject geom = doc["geometry"].to<JsonObject>();
	geom["totalWidthMm"] = recipe.geometry.totalWidth_mm;
	geom["flangeBottomMm"] = recipe.geometry.flangeBottom_mm;
	geom["flangeTopMm"] = recipe.geometry.flangeTop_mm;
	geom["marginMm"] = recipe.geometry.margin_mm;
	geom["windingStartTrimMm"] = recipe.geometry.windingStartTrim_mm;
	geom["windingEndTrimMm"] = recipe.geometry.windingEndTrim_mm;
	geom["wireDiameterMm"] = recipe.geometry.wireDiameter_mm;
	geom["turnsPerPassOffset"] = recipe.geometry.turnsPerPassOffset;
	geom["scatterFactor"] = recipe.geometry.scatterFactor;

	serializeJson(doc, outBuf, outBufLen);
}

/**
 * @brief Parse recipe from JSON and apply safe bounds.
 * @param json JSON input.
 * @param recipe Output recipe.
 * @return true when JSON parsing succeeds.
 */
bool WindingRecipeStore::fromJson(const char* json, WindingRecipe& recipe) const {
	if (!json) return false;
	StaticJsonDocument<1024> doc;
	DeserializationError err = deserializeJson(doc, json);
	if (err) return false;

	const uint32_t recipeVersion = doc["version"] | PICKUP_RECIPE_FORMAT_VERSION;
	if (recipeVersion > PICKUP_RECIPE_FORMAT_VERSION) return false;

	recipe.version          = PICKUP_RECIPE_FORMAT_VERSION;
	recipe.targetTurns      = doc["targetTurns"] | DEFAULT_TARGET_TURNS;
	recipe.freerun          = doc["freerun"] | false;
	recipe.directionCW      = doc["directionCW"] | true;
	recipe.style            = WindingPatternPlanner::styleFromString((const char*)(doc["style"] | "straight"));
	recipe.seed             = doc["seed"] | WINDING_DEFAULT_SEED;
	recipe.layerJitterPct   = doc["layerJitterPct"] | WINDING_LAYER_JITTER_DEFAULT;
	recipe.layerSpeedPct    = doc["layerSpeedPct"] | WINDING_LAYER_SPEED_JITTER_DEFAULT;
	recipe.humanTraversePct = doc["humanTraversePct"] | WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT;
	recipe.humanSpeedPct    = doc["humanSpeedPct"] | WINDING_HUMAN_SPEED_JITTER_DEFAULT;
	recipe.firstPassTraverseFactor = constrain((float)(doc["firstPassTraverseFactor"] | 1.0f), 0.40f, 1.80f);
	recipe.latOffsetMm      = doc["latOffsetMm"] | LAT_HOME_OFFSET_DEFAULT_MM;
	recipe.endPos           = windingEndPosFromString((const char*)(doc["endPos"] | "none"));
	recipe.endPosTurns      = constrain((int)(doc["endPosTurns"] | 3), 1, 20);

	JsonObject geom = doc["geometry"].as<JsonObject>();
	recipe.geometry.totalWidth_mm     = geom["totalWidthMm"] | 17.0f;
	recipe.geometry.flangeBottom_mm   = geom["flangeBottomMm"] | 1.5f;
	recipe.geometry.flangeTop_mm      = geom["flangeTopMm"] | 1.5f;
	recipe.geometry.margin_mm         = geom["marginMm"] | 0.5f;
	recipe.geometry.windingStartTrim_mm = geom["windingStartTrimMm"] | 0.0f;
	recipe.geometry.windingEndTrim_mm = geom["windingEndTrimMm"] | 0.0f;
	recipe.geometry.wireDiameter_mm   = geom["wireDiameterMm"] | WireGauge::AWG42;
	recipe.geometry.turnsPerPassOffset = geom["turnsPerPassOffset"] | 0;
	recipe.geometry.scatterFactor     = geom["scatterFactor"] | 1.0f;

	// Clamp externally provided values to safe runtime ranges.
	recipe.layerJitterPct   = constrain(recipe.layerJitterPct, 0.0f, 0.45f);
	recipe.layerSpeedPct    = constrain(recipe.layerSpeedPct, 0.0f, 0.45f);
	recipe.humanTraversePct = constrain(recipe.humanTraversePct, 0.0f, 0.45f);
	recipe.humanSpeedPct    = constrain(recipe.humanSpeedPct, 0.0f, 0.45f);
	recipe.geometry.scatterFactor = constrain(recipe.geometry.scatterFactor, 0.5f, 4.0f);
	recipe.latOffsetMm = max(0.0f, recipe.latOffsetMm);
	return true;
}
