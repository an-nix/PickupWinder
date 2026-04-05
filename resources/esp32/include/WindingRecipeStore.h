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
	 * @param outBuf Output buffer for JSON text.
	 * @param outBufLen Output buffer length in bytes (including null terminator).
	 */
	void toJson(const WindingRecipe& recipe, char* outBuf, size_t outBufLen) const;

	/**
	 * @brief Parse a recipe from JSON.
	 * @param json Source JSON payload (null-terminated).
	 * @param recipe Output recipe structure.
	 * @return true if parsing and validation succeeded.
	 */
	bool fromJson(const char* json, WindingRecipe& recipe) const;
};
