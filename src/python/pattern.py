"""WindingPattern / WindingPatternPlanner - mirrors WindingPattern.cpp."""
import ctypes
import math

from geometry import WindingGeometry
from config import WINDING_LAYER_JITTER_DEFAULT, WINDING_LAYER_SPEED_JITTER_DEFAULT
from config import WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT, WINDING_HUMAN_SPEED_JITTER_DEFAULT
from config import WINDING_DEFAULT_SEED


STYLE_STRAIGHT = "straight"
STYLE_SCATTER = "scatter"
STYLE_HUMAN = "human"

STYLE_NAMES = {STYLE_STRAIGHT: "Droit", STYLE_SCATTER: "Scatter", STYLE_HUMAN: "Humain"}
STYLE_KEYS = [STYLE_STRAIGHT, STYLE_SCATTER, STYLE_HUMAN]


def _to_uint32(v: int) -> int:
    return v & 0xFFFFFFFF


def _mix(x: int) -> int:
    """Fast 32-bit integer mixer (non-cryptographic)."""
    x = _to_uint32(x)
    x = _to_uint32(x ^ (x >> 16))
    x = _to_uint32(x * 0x7feb352d)
    x = _to_uint32(x ^ (x >> 15))
    x = _to_uint32(x * 0x846ca68b)
    x = _to_uint32(x ^ (x >> 16))
    return x


def _noise_signed(seed: int, a: int, b: int) -> float:
    h = _mix(_to_uint32(seed) ^ _mix(_to_uint32(a) + 0x9e3779b9) ^ _mix(_to_uint32(b) + 0x85ebca6b))
    unit = float(h & 0x00FFFFFF) / 16777215.0
    return unit * 2.0 - 1.0


def _smooth_noise(seed: int, pass_index: int, x: float) -> float:
    SEGMENTS = 7
    x = max(0.0, min(1.0, x)) * SEGMENTS
    i0 = min(int(x), SEGMENTS - 1)
    i1 = i0 + 1
    t = x - float(i0)
    s = t * t * (3.0 - 2.0 * t)
    a = _noise_signed(seed, pass_index, 100 + i0)
    b = _noise_signed(seed, pass_index, 100 + i1)
    return a + (b - a) * s


class TraversePlan:
    def __init__(self):
        self.turns_per_pass: int = 1
        self.speed_scale: float = 1.0
        self.pass_index: int = 0


class WindingRecipe:
    """Mirrors the C++ WindingRecipe struct."""
    def __init__(self):
        self.version = 2
        self.target_turns = 8000
        self.freerun = False
        self.direction_cw = True
        self.style = STYLE_STRAIGHT
        self.seed = WINDING_DEFAULT_SEED
        self.layer_jitter_pct = WINDING_LAYER_JITTER_DEFAULT
        self.layer_speed_pct = WINDING_LAYER_SPEED_JITTER_DEFAULT
        self.human_traverse_pct = WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT
        self.human_speed_pct = WINDING_HUMAN_SPEED_JITTER_DEFAULT
        self.first_pass_traverse_factor = 1.0
        self.lat_offset_mm = 15.0
        self.end_pos = 0          # 0=none, 1=high, 2=low
        self.end_pos_turns = 3
        self.geometry = WindingGeometry()

    def to_dict(self) -> dict:
        return {
            "version": self.version,
            "targetTurns": self.target_turns,
            "freerun": self.freerun,
            "directionCW": self.direction_cw,
            "style": self.style,
            "seed": self.seed,
            "layerJitterPct": self.layer_jitter_pct,
            "layerSpeedPct": self.layer_speed_pct,
            "humanTraversePct": self.human_traverse_pct,
            "humanSpeedPct": self.human_speed_pct,
            "firstPassTraverseFactor": self.first_pass_traverse_factor,
            "latOffsetMm": self.lat_offset_mm,
            "endPos": ["none", "high", "low"][self.end_pos],
            "endPosTurns": self.end_pos_turns,
            "geometry": self.geometry.to_dict(),
        }

    def from_dict(self, d: dict):
        from config import WINDING_DEFAULT_SEED, DEFAULT_TARGET_TURNS
        ver = int(d.get("version", 2))
        if ver > 2:
            raise ValueError(f"Recipe version {ver} too new")
        self.version = 2
        self.target_turns = int(d.get("targetTurns", DEFAULT_TARGET_TURNS))
        self.freerun = bool(d.get("freerun", False))
        self.direction_cw = bool(d.get("directionCW", True))
        self.style = d.get("style", STYLE_STRAIGHT)
        if self.style not in STYLE_KEYS:
            self.style = STYLE_STRAIGHT
        self.seed = int(d.get("seed", WINDING_DEFAULT_SEED))
        self.layer_jitter_pct = max(0.0, min(0.45, float(d.get("layerJitterPct", WINDING_LAYER_JITTER_DEFAULT))))
        self.layer_speed_pct = max(0.0, min(0.45, float(d.get("layerSpeedPct", WINDING_LAYER_SPEED_JITTER_DEFAULT))))
        self.human_traverse_pct = max(0.0, min(0.45, float(d.get("humanTraversePct", WINDING_HUMAN_TRAVERSE_JITTER_DEFAULT))))
        self.human_speed_pct = max(0.0, min(0.45, float(d.get("humanSpeedPct", WINDING_HUMAN_SPEED_JITTER_DEFAULT))))
        self.first_pass_traverse_factor = max(0.40, min(1.80, float(d.get("firstPassTraverseFactor", 1.0))))
        self.lat_offset_mm = max(0.0, float(d.get("latOffsetMm", 15.0)))
        _ep_map = {"none": 0, "high": 1, "low": 2}
        self.end_pos = _ep_map.get(d.get("endPos", "none"), 0)
        self.end_pos_turns = max(1, min(20, int(d.get("endPosTurns", 3))))
        geom_d = d.get("geometry", {})
        if geom_d:
            self.geometry.from_dict(geom_d)


class WindingPatternPlanner:
    def __init__(self):
        self._recipe = WindingRecipe()
        self._pass_index = 0

    def set_recipe(self, recipe: WindingRecipe):
        self._recipe = recipe

    def reset(self):
        self._pass_index = 0

    def get_plan(self, turns_done: int, progress_in_pass: float) -> TraversePlan:
        plan = TraversePlan()
        base_tpp = self._recipe.geometry.turns_per_pass()
        plan.turns_per_pass = max(1, base_tpp)
        if base_tpp <= 0:
            return plan

        progress_in_pass = max(0.0, min(1.0, progress_in_pass))
        pass_index = max(0, turns_done // max(1, base_tpp))
        plan.pass_index = pass_index

        layer_jitter = _noise_signed(self._recipe.seed, pass_index, 1) * self._recipe.layer_jitter_pct
        layer_speed = _noise_signed(self._recipe.seed, pass_index, 2) * self._recipe.layer_speed_pct
        tpp_scale = 1.0
        speed_scale = 1.0

        if self._recipe.style == STYLE_SCATTER:
            tpp_scale += layer_jitter
            speed_scale += layer_speed
        elif self._recipe.style == STYLE_HUMAN:
            seed = self._recipe.seed
            human_traverse = _smooth_noise(seed + 17, pass_index, progress_in_pass) * self._recipe.human_traverse_pct
            human_speed = _smooth_noise(seed + 31, pass_index, progress_in_pass) * self._recipe.human_speed_pct
            tpp_scale += layer_jitter + human_traverse
            speed_scale += layer_speed + human_speed

        if pass_index == 0:
            speed_scale *= self._recipe.first_pass_traverse_factor

        tpp_scale = max(0.55, min(1.60, tpp_scale))
        speed_scale = max(0.55, min(1.60, speed_scale))

        plan.turns_per_pass = max(1, round(base_tpp * tpp_scale))
        plan.speed_scale = speed_scale
        return plan
