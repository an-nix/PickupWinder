"""CommandRegistry - mirrors CommandRegistry.cpp."""

# ── CommandId enum ────────────────────────────────────────────────────────────
class CommandId:
    Unknown = "unknown"
    Start = "start"
    Pause = "pause"
    Stop = "stop"
    Resume = "resume"
    Reset = "reset"
    Target = "target"
    MaxRpm = "max_rpm"
    Freerun = "freerun"
    Direction = "direction"
    StopNextHigh = "stop_next_high"
    StopNextLow = "stop_next_low"
    EndPos = "end_pos"
    EndPosTurns = "end_pos_turns"
    Rodage = "rodage"
    RodageStop = "rodage_stop"
    RodageDist = "rodage_dist"
    RodagePasses = "rodage_passes"
    GeomStartTrim = "geom_start_trim"
    GeomEndTrim = "geom_end_trim"
    GeomStartTrimNudge = "geom_start_trim_nudge"
    GeomEndTrimNudge = "geom_end_trim_nudge"
    GeomPreset = "geom_preset"
    GeomTotal = "geom_total"
    GeomBottom = "geom_bottom"
    GeomTop = "geom_top"
    GeomMargin = "geom_margin"
    GeomWire = "geom_wire"
    GeomTppOffset = "geom_tpp_ofs"
    GeomScatter = "geom_scatter"
    WindowShift = "window_shift"
    WindingStyle = "winding_style"
    WindingSeed = "winding_seed"
    WindingLayerJitter = "winding_layer_jitter"
    WindingLayerSpeed = "winding_layer_speed"
    WindingHumanTraverse = "winding_human_traverse"
    WindingHumanSpeed = "winding_human_speed"
    WindingFirstPassTraverse = "winding_first_pass_traverse"
    RecipeImport = "recipe_import"
    LatOffset = "lat_offset"

    # All IDs for reverse lookup
    ALL = None  # filled below


# ── Value kinds ───────────────────────────────────────────────────────────────
KIND_NONE = "none"
KIND_INT = "integer"
KIND_FLOAT = "float"
KIND_BOOL = "boolean"
KIND_ENUM = "enum"
KIND_JSON = "json"

_COMMANDS = [
    (CommandId.Start, KIND_NONE, None, None, None, None, "", True, "Start winding session."),
    (CommandId.Pause, KIND_NONE, None, None, None, None, "", True, "Pause winding session."),
    (CommandId.Stop, KIND_NONE, None, None, None, None, "", True, "Stop winding and return to idle."),
    (CommandId.Resume, KIND_NONE, None, None, None, None, "", True, "Resume from paused state."),
    (CommandId.Reset, KIND_NONE, None, None, None, None, "", True, "Reset turns and return to idle."),
    (CommandId.Target, KIND_INT, 1, 200000, None, None, "", True, "Target turns count."),
    (CommandId.MaxRpm, KIND_INT, 10, 1500, None, None, "", True, "Max spindle RPM."),
    (CommandId.Freerun, KIND_BOOL, None, None, None, None, "true,false", False, "Freerun mode."),
    (CommandId.Direction, KIND_ENUM, None, None, None, None, "cw,ccw", False, "Spindle direction."),
    (CommandId.StopNextHigh, KIND_NONE, None, None, None, None, "", True, "Stop lateral at next high bound."),
    (CommandId.StopNextLow, KIND_NONE, None, None, None, None, "", True, "Stop lateral at next low bound."),
    (CommandId.EndPos, KIND_ENUM, None, None, None, None, "none,high,low", False, "Final carriage position."),
    (CommandId.EndPosTurns, KIND_INT, 1, 20, None, None, "", False, "Hold turns on final position."),
    (CommandId.Rodage, KIND_NONE, None, None, None, None, "", True, "Start rodage mode."),
    (CommandId.RodageStop, KIND_NONE, None, None, None, None, "", True, "Stop rodage mode."),
    (CommandId.RodageDist, KIND_FLOAT, None, None, 5.0, 100.0, "", False, "Rodage shuttle distance (mm)."),
    (CommandId.RodagePasses, KIND_INT, 1, 200, None, None, "", False, "Rodage pass count."),
    (CommandId.GeomStartTrim, KIND_FLOAT, None, None, -5.0, 5.0, "", False, "Low bound trim (mm)."),
    (CommandId.GeomEndTrim, KIND_FLOAT, None, None, -5.0, 5.0, "", False, "High bound trim (mm)."),
    (CommandId.GeomStartTrimNudge, KIND_FLOAT, None, None, -1.0, 1.0, "", False, "Nudge low bound trim."),
    (CommandId.GeomEndTrimNudge, KIND_FLOAT, None, None, -1.0, 1.0, "", False, "Nudge high bound trim."),
    (CommandId.GeomPreset, KIND_INT, 0, 20, None, None, "", False, "Apply geometry preset."),
    (CommandId.GeomTotal, KIND_FLOAT, None, None, 0.0, 200.0, "", False, "Total bobbin width (mm)."),
    (CommandId.GeomBottom, KIND_FLOAT, None, None, 0.0, 50.0, "", False, "Bottom flange width (mm)."),
    (CommandId.GeomTop, KIND_FLOAT, None, None, 0.0, 50.0, "", False, "Top flange width (mm)."),
    (CommandId.GeomMargin, KIND_FLOAT, None, None, 0.0, 20.0, "", False, "Winding margin (mm)."),
    (CommandId.GeomWire, KIND_FLOAT, None, None, 0.01, 1.0, "", False, "Wire diameter (mm)."),
    (CommandId.GeomTppOffset, KIND_INT, -2000, 2000, None, None, "", False, "Turns-per-pass offset."),
    (CommandId.GeomScatter, KIND_FLOAT, None, None, 0.5, 5.0, "", False, "Scatter factor."),
    (CommandId.WindowShift, KIND_FLOAT, None, None, -5.0, 5.0, "", True, "Shift both winding bounds (mm)."),
    (CommandId.WindingStyle, KIND_ENUM, None, None, None, None, "straight,scatter,human", False, "Pattern style."),
    (CommandId.WindingSeed, KIND_INT, 1, 2147483647, None, None, "", False, "Pattern random seed."),
    (CommandId.WindingLayerJitter, KIND_FLOAT, None, None, 0.0, 0.45, "", False, "Per-layer TPP jitter."),
    (CommandId.WindingLayerSpeed, KIND_FLOAT, None, None, 0.0, 0.45, "", False, "Per-layer speed jitter."),
    (CommandId.WindingHumanTraverse, KIND_FLOAT, None, None, 0.0, 0.45, "", False, "Smooth traverse variation."),
    (CommandId.WindingHumanSpeed, KIND_FLOAT, None, None, 0.0, 0.45, "", False, "Smooth speed variation."),
    (CommandId.WindingFirstPassTraverse, KIND_FLOAT, None, None, 0.40, 1.80, "", False, "First pass traverse scale."),
    (CommandId.RecipeImport, KIND_JSON, None, None, None, None, "", False, "Import full recipe JSON."),
    (CommandId.LatOffset, KIND_FLOAT, None, None, 0.0, 200.0, "", False, "Lateral home offset (mm)."),
]

# Build key-indexed dict
_CMD_BY_KEY: dict[str, dict] = {}
for (cid, kind, imin, imax, fmin, fmax, csv, mutable, help_) in _COMMANDS:
    _CMD_BY_KEY[cid] = {
        "id": cid,
        "key": cid,
        "kind": kind,
        "min_int": imin,
        "max_int": imax,
        "min_float": fmin,
        "max_float": fmax,
        "enum_csv": csv,
        "mutable_during_session": mutable,
        "help": help_,
    }

# Key aliases (matches normalizeKeyImpl in C++)
_KEY_ALIASES = {
    "max-rpm": "max_rpm",
    "windows_shift": "window_shift",
}


def _normalize_key(key: str) -> str:
    k = _KEY_ALIASES.get(key, key)
    # window_shift prefix normalization
    if k.startswith("window_shift"):
        k = "window_shift"
    return k


def _validate_value(defn: dict, value: str) -> bool:
    kind = defn["kind"]
    if kind == KIND_NONE:
        return value == ""
    if kind == KIND_BOOL:
        return value in ("true", "false")
    if kind == KIND_ENUM:
        return value in defn["enum_csv"].split(",")
    if kind == KIND_JSON:
        return len(value) >= 2 and value[0] == "{"
    if kind == KIND_INT:
        try:
            v = int(value)
            return defn["min_int"] <= v <= defn["max_int"]
        except (ValueError, TypeError):
            return False
    if kind == KIND_FLOAT:
        try:
            v = float(value)
            return defn["min_float"] <= v <= defn["max_float"]
        except (ValueError, TypeError):
            return False
    return False


class CommandRegistry:
    @staticmethod
    def find_by_key(key: str) -> dict | None:
        k = _normalize_key(key)
        return _CMD_BY_KEY.get(k)

    @staticmethod
    def normalize_and_validate(cmd: str, value: str) -> tuple[str, str] | None:
        """Returns (canonical_key, command_id) or None if invalid."""
        k = _normalize_key(cmd)
        defn = _CMD_BY_KEY.get(k)
        if defn is None:
            return None
        if not _validate_value(defn, value):
            return None
        return (k, defn["id"])

    @staticmethod
    def all() -> list[dict]:
        return list(_CMD_BY_KEY.values())

    @staticmethod
    def capabilities_json() -> str:
        import json
        from config import PICKUP_WS_PROTOCOL_VERSION, PICKUP_RECIPE_FORMAT_VERSION
        cmds = [
            {
                "key": d["key"],
                "type": d["kind"],
                "mutableDuringSession": d["mutable_during_session"],
                "help": d["help"],
            }
            for d in _CMD_BY_KEY.values()
        ]
        payload = {
            "capabilitiesVersion": "1",
            "protocol": {
                "ws": PICKUP_WS_PROTOCOL_VERSION,
                "recipe": PICKUP_RECIPE_FORMAT_VERSION,
            },
            "commands": cmds,
        }
        return json.dumps(payload, separators=(",", ":"))