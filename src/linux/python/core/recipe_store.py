"""core/recipe_store.py — JSON file persistence for WindingRecipe.

Equivalent of the ESP32 WindingRecipeStore (NVS replaced by a JSON file).
Migrated from resources/esp32/src/WindingRecipeStore.cpp.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Optional

from .geometry import WindingGeometry, WireGauge
from .recipe import PICKUP_RECIPE_FORMAT_VERSION, WindingRecipe
from .types import WindingEndPos, WindingStyle

_log = logging.getLogger(__name__)

DEFAULT_RECIPE_PATH: str = "/usr/local/lib/pickup-winder/recipe.json"


class RecipeStore:
    """Load and save WindingRecipe as a JSON file.

    Usage::

        store = RecipeStore()
        recipe = store.load() or WindingRecipe().normalized()
        # ... modify recipe ...
        store.save(recipe)
    """

    def __init__(self, path: str = DEFAULT_RECIPE_PATH):
        self._path = path

    # ── Public API ──────────────────────────────────────────────────────────────

    def load(self) -> Optional[WindingRecipe]:
        """Load recipe from file.  Returns None on missing file or parse error."""
        try:
            with open(self._path, "r") as fh:
                data = json.load(fh)
            recipe = self._from_dict(data)
            _log.info("Recipe loaded from %s", self._path)
            return recipe
        except FileNotFoundError:
            _log.info("Recipe file not found (%s), using defaults.", self._path)
            return None
        except Exception as exc:  # noqa: BLE001
            _log.warning("Recipe load failed (%s): %s — using defaults.", self._path, exc)
            return None

    def save(self, recipe: WindingRecipe) -> bool:
        """Save recipe to file.  Returns True on success."""
        try:
            os.makedirs(os.path.dirname(self._path) or ".", exist_ok=True)
            data = self._to_dict(recipe)
            tmp = self._path + ".tmp"
            with open(tmp, "w") as fh:
                json.dump(data, fh, indent=2)
            os.replace(tmp, self._path)
            _log.debug("Recipe saved to %s", self._path)
            return True
        except Exception as exc:  # noqa: BLE001
            _log.warning("Recipe save failed: %s", exc)
            return False

    # ── Serialization ───────────────────────────────────────────────────────────

    def _to_dict(self, recipe: WindingRecipe) -> dict:
        g = recipe.geometry
        return {
            "version": PICKUP_RECIPE_FORMAT_VERSION,
            "targetTurns": recipe.target_turns,
            "freerun": recipe.freerun,
            "directionCW": recipe.direction_cw,
            "style": recipe.style.value,
            "seed": recipe.seed,
            "layerJitterPct": recipe.layer_jitter_pct,
            "layerSpeedPct": recipe.layer_speed_pct,
            "humanTraversePct": recipe.human_traverse_pct,
            "humanSpeedPct": recipe.human_speed_pct,
            "firstPassTraverseFactor": recipe.first_pass_traverse_factor,
            "latOffsetMm": recipe.lat_offset_mm,
            "endPos": recipe.end_pos.value,
            "endPosTurns": recipe.end_pos_turns,
            "geometry": {
                "totalWidthMm": g.total_width_mm,
                "flangeBottomMm": g.flange_bottom_mm,
                "flangeTopMm": g.flange_top_mm,
                "marginMm": g.margin_mm,
                "windingStartTrimMm": g.winding_start_trim_mm,
                "windingEndTrimMm": g.winding_end_trim_mm,
                "wireDiameterMm": g.wire_diameter_mm,
                "turnsPerPassOffset": g.turns_per_pass_offset,
                "scatterFactor": g.scatter_factor,
            },
        }

    def _from_dict(self, data: dict) -> WindingRecipe:
        version = int(data.get("version", PICKUP_RECIPE_FORMAT_VERSION))
        if version > PICKUP_RECIPE_FORMAT_VERSION:
            raise ValueError(
                f"Recipe version {version} > supported {PICKUP_RECIPE_FORMAT_VERSION}"
            )

        # Parse geometry sub-object
        g_data = data.get("geometry", {})
        geom = WindingGeometry(
            total_width_mm=float(g_data.get("totalWidthMm", 17.0)),
            flange_bottom_mm=float(g_data.get("flangeBottomMm", 1.5)),
            flange_top_mm=float(g_data.get("flangeTopMm", 1.5)),
            margin_mm=float(g_data.get("marginMm", 0.5)),
            winding_start_trim_mm=float(g_data.get("windingStartTrimMm", 0.0)),
            winding_end_trim_mm=float(g_data.get("windingEndTrimMm", 0.0)),
            wire_diameter_mm=float(g_data.get("wireDiameterMm", WireGauge.AWG42)),
            turns_per_pass_offset=int(g_data.get("turnsPerPassOffset", 0)),
            scatter_factor=float(g_data.get("scatterFactor", 1.0)),
        )

        # Parse enums with fallback
        try:
            style = WindingStyle(data.get("style", "straight"))
        except ValueError:
            style = WindingStyle.STRAIGHT

        try:
            end_pos = WindingEndPos(data.get("endPos", "none"))
        except ValueError:
            end_pos = WindingEndPos.NONE

        recipe = WindingRecipe(
            version=PICKUP_RECIPE_FORMAT_VERSION,
            target_turns=int(data.get("targetTurns", 7500)),
            freerun=bool(data.get("freerun", False)),
            direction_cw=bool(data.get("directionCW", True)),
            style=style,
            seed=int(data.get("seed", 1337)),
            layer_jitter_pct=float(data.get("layerJitterPct", 0.0)),
            layer_speed_pct=float(data.get("layerSpeedPct", 0.0)),
            human_traverse_pct=float(data.get("humanTraversePct", 0.0)),
            human_speed_pct=float(data.get("humanSpeedPct", 0.0)),
            first_pass_traverse_factor=float(data.get("firstPassTraverseFactor", 1.0)),
            lat_offset_mm=float(data.get("latOffsetMm", 0.0)),
            end_pos=end_pos,
            end_pos_turns=int(data.get("endPosTurns", 3)),
            geometry=geom,
        )
        return recipe.normalized()
