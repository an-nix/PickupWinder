import math

from .recipe import WindingRecipe
from .types import TraversePlan, WindingStyle


def _clampf(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class WindingPatternPlanner:
    def __init__(self, recipe: WindingRecipe | None = None):
        self._recipe = recipe or WindingRecipe()

    def set_recipe(self, recipe: WindingRecipe) -> None:
        self._recipe = recipe

    @staticmethod
    def _mix(x: int) -> int:
        x &= 0xFFFFFFFF
        x ^= (x >> 16)
        x = (x * 0x7FEB352D) & 0xFFFFFFFF
        x ^= (x >> 15)
        x = (x * 0x846CA68B) & 0xFFFFFFFF
        x ^= (x >> 16)
        return x & 0xFFFFFFFF

    @classmethod
    def _noise_signed(cls, seed: int, a: int, b: int) -> float:
        h = cls._mix(seed ^ cls._mix(a + 0x9E3779B9) ^ cls._mix(b + 0x85EBCA6B))
        unit = float(h & 0x00FFFFFF) / 16777215.0
        return unit * 2.0 - 1.0

    @classmethod
    def _smooth_noise(cls, seed: int, pass_index: int, x: float) -> float:
        segments = 7
        x = _clampf(x, 0.0, 1.0) * segments
        i0 = min(int(x), segments - 1)
        i1 = i0 + 1
        t = x - float(i0)
        s = t * t * (3.0 - 2.0 * t)
        a = cls._noise_signed(seed, pass_index, 100 + i0)
        b = cls._noise_signed(seed, pass_index, 100 + i1)
        return a + (b - a) * s

    @staticmethod
    def style_key(style: WindingStyle) -> str:
        return style.value

    @staticmethod
    def style_from_string(s: str) -> WindingStyle:
        try:
            return WindingStyle(s.lower())
        except ValueError:
            return WindingStyle.STRAIGHT

    def reset(self) -> None:
        pass  # Stateless planner; hook kept for ESP32 API compatibility.

    def get_plan(self, turns_done: int, progress_in_pass: float) -> TraversePlan:
        recipe = self._recipe
        base_tpp = max(1, int(recipe.geometry.turns_per_pass()))

        progress_in_pass = _clampf(progress_in_pass, 0.0, 1.0)
        pass_index = max(0, int(turns_done // base_tpp))

        layer_jitter = self._noise_signed(recipe.seed, pass_index, 1) * recipe.layer_jitter_pct
        layer_speed = self._noise_signed(recipe.seed, pass_index, 2) * recipe.layer_speed_pct

        tpp_scale = 1.0
        speed_scale = 1.0

        if recipe.style == WindingStyle.SCATTER:
            tpp_scale += layer_jitter
            speed_scale += layer_speed
        elif recipe.style == WindingStyle.HUMAN:
            human_traverse = self._smooth_noise(recipe.seed + 17, pass_index, progress_in_pass) * recipe.human_traverse_pct
            human_speed = self._smooth_noise(recipe.seed + 31, pass_index, progress_in_pass) * recipe.human_speed_pct
            tpp_scale += layer_jitter + human_traverse
            speed_scale += layer_speed + human_speed

        if pass_index == 0:
            speed_scale *= recipe.first_pass_traverse_factor

        tpp_scale = _clampf(tpp_scale, 0.55, 1.60)
        speed_scale = _clampf(speed_scale, 0.55, 1.60)

        return TraversePlan(
            turns_per_pass=max(1, int(round(base_tpp * tpp_scale))),
            speed_scale=float(speed_scale),
            pass_index=pass_index,
        )
