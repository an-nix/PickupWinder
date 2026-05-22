"""Session parameters for a winding run.

``SessionParams`` captures the operator's execution choices at launch time.
It is intentionally separate from ``WindingProgram`` (the persistent recipe)
and from machine-level settings stored in ``AppConfiguration``.

Live-adjustable fields (via ``winding.update_session``):
  - spindle_rpm
  - total_turns
  - window_low_mm / window_high_mm
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, fields
from typing import Any


@dataclass(slots=True)
class SessionParams:
    """Transient execution context for one winding session.

    Args:
        spindle_rpm:     Target spindle speed (required, > 0).
        total_turns:     Total turn count override.  ``None`` → use
                         ``WindingProgram.total_turns()``.
        window_low_mm:   Lower winding-window bound override.  ``None`` →
                         resolved to the configured lateral start position.
        window_high_mm:  Upper winding-window bound override.  ``None`` →
                         resolved to ``window_low_mm + bobbin_width_mm``.
        chunk_time_s:    Adaptive planning chunk duration in seconds.
    """

    spindle_rpm: float
    total_turns: float | None = None
    window_low_mm: float | None = None
    window_high_mm: float | None = None
    chunk_time_s: float = 0.25

    def validate(self) -> None:
        """Raise ``ValueError`` if any field is out of range."""
        if self.spindle_rpm <= 0.0:
            raise ValueError("spindle_rpm must be positive")
        if self.total_turns is not None and self.total_turns <= 0.0:
            raise ValueError("total_turns must be positive when specified")
        if self.chunk_time_s <= 0.0:
            raise ValueError("chunk_time_s must be positive")

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> SessionParams:
        """Build from a raw dict, silently ignoring unknown keys."""
        if not isinstance(payload, dict):
            raise ValueError("session payload must be an object")
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in payload.items() if k in known})

    def to_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)
