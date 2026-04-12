"""core/command_router.py — Text-command dispatcher for WinderApp.

Maps bare-string commands (e.g. from a WebSocket client, serial line,
or interactive stdin) to WinderApp.handle_command() calls.

Command format (either variant accepted):
    "cmd [value]"         plain text, fields separated by whitespace
    {"cmd":"...", "value":"..."}  JSON object (value optional)

Boolean values accept: "1", "true", "yes", "on"  (case-insensitive).
Float / int values are passed verbatim to WinderApp which does clamping.
Unknown commands are logged at WARNING level.
"""

from __future__ import annotations

import json
import logging
from typing import Optional

_log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Command aliases
# ---------------------------------------------------------------------------

# Canonical command list (must match _handle_*_command keys in winder_app.py).
_KNOWN_COMMANDS: frozenset[str] = frozenset({
    # Lifecycle / control
    "start", "stop", "pause", "resume", "reset",
    # Winding parameters
    "target", "max_rpm", "freerun", "direction",
    # End-position
    "end_pos", "end_pos_turns",
    "stop_next_high", "stop_next_low",
    # Rodage (break-in)
    "rodage", "rodage_stop", "rodage_dist", "rodage_passes",
    # Geometry trims
    "geom_start_trim", "geom_end_trim",
    "geom_start_trim_nudge", "geom_end_trim_nudge",
    "window_shift",
    # Geometry structure
    "geom_preset",
    "geom_total", "geom_bottom", "geom_top", "geom_margin",
    "geom_wire", "geom_tpp_offset", "geom_scatter",
    # Pattern
    "winding_style", "winding_seed",
    "winding_layer_jitter", "winding_layer_speed",
    "winding_human_traverse", "winding_human_speed",
    "winding_first_pass_traverse",
    # Lateral
    "lat_offset",
    # Status queries (handled here, not delegated to WinderApp)
    "status", "recipe",
})

# Short-form aliases → canonical name
_ALIASES: dict[str, str] = {
    "s": "start",
    "q": "stop",
    "p": "pause",
    "r": "resume",
    "t": "target",
    "?": "status",
    "free": "freerun",
    "dir": "direction",
    "cw": "direction",
    "ccw": "direction",
    "style": "winding_style",
    "seed": "winding_seed",
    "preset": "geom_preset",
    "wire": "geom_wire",
    "margin": "geom_margin",
}

class CommandRouter:
    """Routes text commands to a WinderApp instance.

    Usage::

        app = WinderApp(controller)
        router = CommandRouter(app)
        router.dispatch("start")
        router.dispatch("target 8500")
        router.dispatch('{"cmd":"geom_wire","value":"0.064"}')
        result = router.dispatch("status")   # returns dict
    """

    def __init__(self, app) -> None:
        self._app = app

    def dispatch(self, raw: str) -> Optional[dict]:
        """Parse *raw* and call the appropriate WinderApp method.

        Returns a dict for query commands (``status``, ``recipe``), else None.
        """
        raw = raw.strip()
        if not raw:
            return None

        # JSON object?
        if raw.startswith("{"):
            try:
                obj = json.loads(raw)
                cmd = str(obj.get("cmd", "")).lower().strip()
                val = str(obj.get("value", "")).strip()
            except json.JSONDecodeError:
                _log.warning("[Router] Malformed JSON: %s", raw)
                return None
        else:
            parts = raw.split(maxsplit=1)
            cmd = parts[0].lower().strip()
            val = parts[1].strip() if len(parts) > 1 else ""

        # Resolve alias
        cmd = _ALIASES.get(cmd, cmd)

        # Handle "cw" / "ccw" shorthands
        if cmd == "direction" and not val:
            val = "cw" if raw.startswith("cw") else "ccw"

        # Query commands
        if cmd == "status":
            return self._app.status_dict()
        if cmd == "recipe":
            return self._app.recipe_dict()

        # Unknown?
        if cmd not in _KNOWN_COMMANDS:
            _log.warning("[Router] Unknown command: '%s'", cmd)
            return None

        # Normalise boolean shorthands for commands that use them
        if cmd == "freerun" and not val:
            val = "true"
        if cmd == "direction" and not val:
            val = "cw"

        ok = self._app.handle_command(cmd, val)
        if not ok:
            _log.debug("[Router] handle_command('%s', '%s') returned False", cmd, val)
        return None

    def dispatch_lines(self, text: str) -> list[Optional[dict]]:
        """Dispatch one line at a time; useful for batch / file input."""
        results: list[Optional[dict]] = []
        for line in text.splitlines():
            r = self.dispatch(line)
            if r is not None:
                results.append(r)
        return results
