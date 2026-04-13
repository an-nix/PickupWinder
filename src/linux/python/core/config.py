"""core/config.py — Application-wide configuration.

Single source of truth for every constant that was previously scattered
as a module-level literal across lateral_controller.py, winder_app.py,
session_controller.py, etc.

Configuration is loaded from a JSON file at startup; any missing key
falls back to the Python default encoded in the dataclasses below.

Default JSON location: /usr/local/lib/pickup-winder/config.json
Override at runtime via the PICKUP_CONFIG environment variable.
"""

from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass, field
from hal.hardware_definition import HardWareDefinition

_log = logging.getLogger(__name__)

CONFIG_PATH = "/usr/local/lib/pickup-winder/config.json"


# ── Application parameters ─────────────────────────────────────────────────────

@dataclass
class AppConfig:
    """All runtime-tunable application parameters.

    Instantiated with Python defaults and optionally overridden by
    ``load_config()`` from a JSON file.
    """

    # I/O paths
    socket_path: str = "/run/pickup-winder.sock"
    recipe_path: str = "/usr/local/lib/pickup-winder/recipe.json"

    # Hardware (model defined in HAL)
    hw: HardWareDefinition = field(default_factory=HardWareDefinition)

    # Main loop
    tick_hz: int = 100                 # Target ticks per second (10 ms period)

    # Approach window: spindle decelerates as turns approach the target
    approach_turns: int = 50           # Window width in turns before target
    approach_hz_floor: int = 3200      # Minimum Hz at the near edge of the window

    # Rodage (mechanical break-in) defaults
    rodage_dist_mm: float = 10.0
    rodage_passes: int = 10

    # Pot input dead-zone
    pot_run_threshold: float = 0.001   # Below this → motor is stopped

    # (Bobbin presets moved out of AppConfig — see core.geometry.BOBBIN_PRESETS)


# ── JSON load / save ───────────────────────────────────────────────────────────

def load_config(path: str = CONFIG_PATH) -> AppConfig:
    """Load AppConfig from *path*.  Missing keys fall back to Python defaults.

    Never raises: on any error the function logs a warning and returns a
    default-initialised AppConfig.
    """
    cfg = AppConfig()
    try:
        with open(path) as fh:
            data = json.load(fh)
    except FileNotFoundError:
        _log.info("Config file not found at %s — using defaults.", path)
        return cfg
    except Exception as exc:          # noqa: BLE001
        _log.warning("Config load error (%s): %s — using defaults.", path, exc)
        return cfg

    # ── hardware sub-object ────────────────────────────────────────────────
    hw_raw = data.get("hardware", {})
    hw = cfg.hw
    _apply_int_fields(hw, hw_raw, (
        "lat_steps_per_mm", "lat_home_hz", "lat_traverse_hz",
        "lat_rodage_hz", "lat_accel",
        "sp_steps_per_rev", "sp_hz_min", "sp_hz_max",
    ))
    if "lat_traverse_max_mm" in hw_raw:
        hw.lat_traverse_max_mm = float(hw_raw["lat_traverse_max_mm"])

    # ── top-level keys ────────────────────────────────────────────────────
    cfg.socket_path       = str  (data.get("socket_path",       cfg.socket_path))
    cfg.recipe_path       = str  (data.get("recipe_path",       cfg.recipe_path))
    cfg.tick_hz           = int  (data.get("tick_hz",           cfg.tick_hz))
    cfg.approach_turns    = int  (data.get("approach_turns",    cfg.approach_turns))
    cfg.approach_hz_floor = int  (data.get("approach_hz_floor", cfg.approach_hz_floor))
    cfg.rodage_dist_mm    = float(data.get("rodage_dist_mm",    cfg.rodage_dist_mm))
    cfg.rodage_passes     = int  (data.get("rodage_passes",     cfg.rodage_passes))
    cfg.pot_run_threshold = float(data.get("pot_run_threshold", cfg.pot_run_threshold))

    # Bobbin presets removed from AppConfig — geometry presets live in
    # `core.geometry.BOBBIN_PRESETS` or a separate presets file managed by the user.

    _log.info("Config loaded from %s", path)
    return cfg


def save_config(cfg: AppConfig, path: str = CONFIG_PATH) -> bool:
    """Persist AppConfig to JSON.  Returns True on success."""
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        hw = cfg.hw
        data: dict = {
            "socket_path":       cfg.socket_path,
            "recipe_path":       cfg.recipe_path,
            "tick_hz":           cfg.tick_hz,
            "approach_turns":    cfg.approach_turns,
            "approach_hz_floor": cfg.approach_hz_floor,
            "rodage_dist_mm":    cfg.rodage_dist_mm,
            "rodage_passes":     cfg.rodage_passes,
            "pot_run_threshold": cfg.pot_run_threshold,
            "hardware": {
                "lat_steps_per_mm":    hw.lat_steps_per_mm,
                "lat_traverse_max_mm": hw.lat_traverse_max_mm,
                "lat_home_hz":         hw.lat_home_hz,
                "lat_traverse_hz":     hw.lat_traverse_hz,
                "lat_rodage_hz":       hw.lat_rodage_hz,
                "lat_accel":           hw.lat_accel,
                "sp_steps_per_rev":    hw.sp_steps_per_rev,
                "sp_hz_min":           hw.sp_hz_min,
                "sp_hz_max":           hw.sp_hz_max,
            },
        }
        # Presets intentionally omitted from saved config. Use
        # core.geometry.BOBBIN_PRESETS or a separate presets file.
        tmp = path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(data, fh, indent=2)
        os.replace(tmp, path)
        _log.info("Config saved to %s", path)
        return True
    except Exception as exc:          # noqa: BLE001
        _log.warning("Config save failed: %s", exc)
        return False


# ── Internal helpers ───────────────────────────────────────────────────────────

def _apply_int_fields(obj: object, data: dict, keys: tuple) -> None:
    """Set integer fields on *obj* from *data* for each key present."""
    for k in keys:
        if k in data:
            setattr(obj, k, int(data[k]))
