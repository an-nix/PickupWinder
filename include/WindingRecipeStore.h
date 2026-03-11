#pragma once
#include <Arduino.h>
#include "WindingPattern.h"

class WindingRecipeStore {
public:
	void   begin();
	bool   load(WindingRecipe& recipe);
	bool   save(const WindingRecipe& recipe);
	String toJson(const WindingRecipe& recipe) const;
	bool   fromJson(const String& json, WindingRecipe& recipe) const;
};
