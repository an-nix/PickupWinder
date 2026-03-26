#pragma once
#include <Arduino.h>
#include "WindingPattern.h"

class WindingRecipeStore {
public:
	/**
	 * @brief Initialize persistent storage backend.
	 * @par Usage
	 * Call once during application startup before load/save operations.
	 */
	void   begin();

	/**
	 * @brief Load a recipe from persistent storage.
	 * @param recipe Output recipe structure.
	 * @return true if a valid recipe was loaded, false otherwise.
	 */
	bool   load(WindingRecipe& recipe);

	/**
	 * @brief Save a recipe to persistent storage.
	 * @param recipe Recipe to persist.
	 * @return true on success, false on write/serialization failure.
	 */
	bool   save(const WindingRecipe& recipe);

	/**
	 * @brief Serialize a recipe to JSON.
	 * @param recipe Recipe to serialize.
	 * @return JSON payload as a string.
	 */
	String toJson(const WindingRecipe& recipe) const;

	/**
	 * @brief Parse a recipe from JSON.
	 * @param json Source JSON payload.
	 * @param recipe Output recipe structure.
	 * @return true if parsing and validation succeeded.
	 */
	bool   fromJson(const String& json, WindingRecipe& recipe) const;
};
