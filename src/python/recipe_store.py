"""WindingRecipeStore - JSON persistence to eMMC file (replaces ESP32 NVS)."""
import json
import os

from pattern import WindingRecipe

RECIPE_PATH = "/var/lib/pickupwinder/recipe.json"


class WindingRecipeStore:
    def __init__(self, path: str = RECIPE_PATH):
        self._path = path

    def begin(self):
        os.makedirs(os.path.dirname(self._path), exist_ok=True)

    def load(self, recipe: WindingRecipe) -> bool:
        try:
            with open(self._path, "r") as f:
                d = json.load(f)
            recipe.from_dict(d)
            return True
        except (OSError, ValueError, KeyError):
            return False

    def save(self, recipe: WindingRecipe) -> bool:
        try:
            os.makedirs(os.path.dirname(self._path), exist_ok=True)
            with open(self._path, "w") as f:
                json.dump(recipe.to_dict(), f, separators=(",", ":"))
            return True
        except OSError:
            return False

    def to_json(self, recipe: WindingRecipe) -> str:
        return json.dumps(recipe.to_dict(), separators=(",", ":"))

    def from_json(self, text: str, recipe: WindingRecipe) -> bool:
        try:
            d = json.loads(text)
            recipe.from_dict(d)
            return True
        except (ValueError, KeyError):
            return False
