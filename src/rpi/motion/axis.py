from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class Axis:
    """Representation d'un axe pour le host.

    Attributes:
        axis_id: Identifiant logique de l'axe (0-based).
        name: Nom humain de l'axe.
        can_move_without_homing: True si l'axe peut bouger avant homing.
        homed: True si l'axe a déjà été homé.
    """

    axis_id: int
    name: str
    can_move_without_homing: bool = False
    homed: bool = False

    @property
    def requires_homing(self) -> bool:
        return not self.can_move_without_homing and not self.homed

    @property
    def can_move(self) -> bool:
        return self.can_move_without_homing or self.homed

    def mark_homed(self) -> None:
        self.homed = True

    def clear_homed(self) -> None:
        self.homed = False
