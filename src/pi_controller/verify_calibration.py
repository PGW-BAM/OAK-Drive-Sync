"""
State machine for the Quick Re-verify calibration flow.

Pure data — no NiceGUI imports. The GUI page in gui.py drives the
transitions by mutating DriveVerifyState.step and VerifySession.current_index.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


class VerifyStep(Enum):
    IDLE = auto()
    MOVING_TO_MIN = auto()
    AWAITING_MIN_CONFIRM = auto()
    MOVING_TO_MAX = auto()
    AWAITING_MAX_CONFIRM = auto()
    DONE = auto()
    NEEDS_RECALIBRATE = auto()


@dataclass
class DriveVerifyState:
    key: str
    saved_min: float
    saved_max: float
    step: VerifyStep = VerifyStep.IDLE
    confirmed: bool = False


@dataclass
class VerifySession:
    drives: list[DriveVerifyState]
    current_index: int = 0
    recalibrate_keys: list[str] = field(default_factory=list)

    @property
    def current(self) -> DriveVerifyState:
        return self.drives[self.current_index]

    @property
    def is_complete(self) -> bool:
        return self.current_index >= len(self.drives)
