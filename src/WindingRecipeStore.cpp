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
	Preferences prefs;
	if (!prefs.begin(RECIPE_NS, true)) return false;
	String json = prefs.getString(RECIPE_KEY, "");
	prefs.end();
	if (json.isEmpty()) return false;
	return fromJson(json, recipe);
}

/**
 * @brief Save recipe to NVS.
 * @param recipe Recipe to persist.
 * @return true when write length matches serialized length.
 */
bool WindingRecipeStore::save(const WindingRecipe& recipe) {
	String json = toJson(recipe);
	Preferences prefs;
	if (!prefs.begin(RECIPE_NS, false)) return false;
	size_t written = prefs.putString(RECIPE_KEY, json);
	prefs.end();
	return written == json.length();
}

/**
 * @brief Serialize recipe to pretty JSON.
 * @param recipe Recipe input.
 * @return Pretty-printed JSON string.
 */
String WindingRecipeStore::toJson(const WindingRecipe& recipe) const {
	JsonDocument doc;
	doc["version"] = recipe.version;
	doc["targetTurns"] = recipe.targetTurns;
	doc["freerun"] = recipe.freerun;
	doc["directionCW"] = recipe.directionCW;
	doc["seed"] = recipe.seed;
	doc["layerJitterPct"] = recipe.layerJitterPct;
	doc["layerSpeedPct"] = recipe.layerSpeedPct;
	doc["humanTraversePct"] = recipe.humanTraversePct;
	doc["humanSpeedPct"] = recipe.humanSpeedPct;
	doc["firstPassTraverseFactor"] = recipe.firstPassTraverseFactor;
	doc["latOffsetMm"] = recipe.latOffset_mm;
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

	String out;
	serializeJsonPretty(doc, out);
	return out;
}

/**
 * @brief Parse recipe from JSON and apply safe bounds.
 * @param json JSON input.
 * @param recipe Output recipe.
 * @return true when JSON parsing succeeds.
 */
bool WindingRecipeStore::fromJson(const String& json, WindingRecipe& recipe) const {
	JsonDocument doc;
	DeserializationError err = deserializeJson(doc, json);
	if (err) return false;

	recipe.version          = doc["version"] | 1;
	recipe.targetTurns      = doc["targetTurns"] | DEFAULT_TARGET_TURNS;
	recipe.freerun          = doc["freerun"] | false;
	recipe.directionCW      = doc["directionCW"] | true;
	recipe.seed             = doc["seed"] | WINDING_DEFAULT_SEED;
	recipe.layerJitterPct   = doc["layerJitterPct"] | WINDING_LAYER_JITTER_DEFAULT;
	recipe.layerSpeedPct    = doc["layerSpeedPct"] | WINDING_LAYER_SPEED_JITTER_DEFAULT;
	recipe.humanTraversePct = doc["humanTraversePct"] | WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT;
	recipe.humanSpeedPct    = doc["humanSpeedPct"] | WINDING_HUMAN_SPEED_JITTER_DEFAULT;
	recipe.firstPassTraverseFactor = constrain((float)(doc["firstPassTraverseFactor"] | 1.0f), 0.40f, 1.80f);
	recipe.latOffset_mm      = doc["latOffsetMm"] | LAT_HOME_OFFSET_DEFAULT_MM;
	recipe.endPos           = windingEndPosFromString(String((const char*)(doc["endPos"] | "none")));
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
	recipe.latOffset_mm = max(0.0f, recipe.latOffset_mm);
	return true;
}
