from dataclasses import dataclass
from enum import Enum, auto


class WindingState(Enum):
    IDLE = auto()
    WINDING = auto()
    PAUSED = auto()
    TARGET_REACHED = auto()
    RODAGE = auto()


class InputSource(Enum):
    NONE = auto()
    POT = auto()
    IHM = auto()
    FOOTSWITCH = auto()


class ControlIntent(Enum):
    NONE = auto()
    START = auto()
    PAUSE = auto()
    STOP = auto()


class SessionState(Enum):
    IDLE = auto()
    ARMED_OR_RUNNING = auto()
    PAUSED = auto()


class RunMode(Enum):
    NONE = auto()
    MAX = auto()
    POT = auto()


class WindingStyle(Enum):
    STRAIGHT = "straight"
    SCATTER = "scatter"
    HUMAN = "human"


class WindingEndPos(Enum):
    NONE = "none"
    TOP = "high"
    BOTTOM = "low"


@dataclass
class TraversePlan:
    turns_per_pass: int = 1
    speed_scale: float = 1.0
    pass_index: int = 0


@dataclass
class TickInput:
    pot_level: float = 0.0
    has_pot: bool = False
    footswitch: bool = False
    has_footswitch: bool = False
    encoder_delta: int = 0
