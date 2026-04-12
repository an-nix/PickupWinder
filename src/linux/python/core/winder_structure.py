"""core/winder_structure.py — Static winder axis registry."""
from __future__ import annotations

from .axis import Axis, LATERAL, SPINDLE


class WinderStructure:
    """Static axis registry for the two-axis pickup winder."""

    SPINDLE: Axis = SPINDLE
    LATERAL: Axis = LATERAL
    ALL: tuple[Axis, ...] = (SPINDLE, LATERAL)

    def __repr__(self) -> str:  # pragma: no cover
        return f"WinderStructure(axes={[a.name for a in self.ALL]})"
