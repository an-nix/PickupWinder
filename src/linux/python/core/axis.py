"""core/axis.py — Axis identity dataclass."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Axis:
    """Immutable identity record for a motor axis."""

    id: int
    name: str
    free_move: bool = False

    def __str__(self) -> str:
        return self.name


# Module-level singletons — import as `from core import SPINDLE, LATERAL`
SPINDLE = Axis(id=0, name="spindle", free_move=True)
LATERAL = Axis(id=1, name="lateral", free_move=False)