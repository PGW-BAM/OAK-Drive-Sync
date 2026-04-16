"""Abstract base class for all drive types."""

from __future__ import annotations

import asyncio
from abc import ABC, abstractmethod
from enum import StrEnum


class DriveState(StrEnum):
    IDLE = "idle"
    MOVING = "moving"
    REACHED = "reached"
    FAULT = "fault"
    HOMING = "homing"


class BaseDrive(ABC):
    """Abstract interface for a single drive axis.

    All drive implementations (GPIO stepper, Tinkerforge, etc.)
    must implement this interface so the DriveManager can treat
    them uniformly.
    """

    def __init__(self, cam_id: str, axis: str) -> None:
        self.cam_id = cam_id
        self.axis = axis
        self._state = DriveState.IDLE
        self._current_position: float = 0.0
        self._target_position: float | None = None
        self._move_lock = asyncio.Lock()
        self.calibration_mode: bool = False
        self.min_calibrated: bool = False
        self.max_calibrated: bool = False

    @property
    def key(self) -> str:
        return f"{self.cam_id}:{self.axis}"

    @property
    def state(self) -> DriveState:
        return self._state

    @property
    def current_position(self) -> float:
        return self._current_position

    @property
    def target_position(self) -> float | None:
        return self._target_position

    @property
    def calibrated(self) -> bool:
        return self.min_calibrated and self.max_calibrated

    @abstractmethod
    async def setup(self) -> None:
        """Initialize hardware resources (GPIO, Tinkerforge connection, etc.)."""

    @abstractmethod
    async def cleanup(self) -> None:
        """Release hardware resources."""

    @abstractmethod
    async def move_to(self, position: float, speed: float = 1.0) -> None:
        """Move to an absolute position.

        Must update _state to MOVING at start, REACHED on success,
        FAULT on error. Must update _current_position during movement.

        Args:
            position: Target position in drive-native units.
            speed: Normalized speed 0.0–1.0 (1.0 = max).
        """

    @abstractmethod
    async def home(self) -> None:
        """Home the drive to its reference/zero position.

        For GPIO drives without limit switches: move to position 0.
        For Tinkerforge: move to min_position.
        """

    @abstractmethod
    def emergency_stop(self) -> None:
        """Immediately stop movement. Must be synchronous and safe to call anytime."""

    @abstractmethod
    def get_min_position(self) -> float:
        """Return the minimum allowed position."""

    @abstractmethod
    def get_max_position(self) -> float:
        """Return the maximum allowed position."""

    @abstractmethod
    def set_min_position(self, value: float) -> None:
        """Update the minimum allowed position (calibration)."""

    @abstractmethod
    def set_max_position(self, value: float) -> None:
        """Update the maximum allowed position (calibration)."""

    def set_zero(self) -> None:
        """Reset current position to 0 (define current physical location as zero)."""
        self._current_position = 0.0

    def seed_position(self, value: float) -> None:
        """Pre-seed current position from persisted log at startup.
        Only call before any movement has occurred."""
        self._current_position = value

    def set_current_position(self, value: float) -> None:
        """Overwrite the internal position counter without physical motion.

        Used by the closed-loop angle-convergence feature to re-anchor the
        motor's step counter to the teach-time reference after IMU-based
        corrections — otherwise the accumulated nudges permanently offset
        the drive's min/max envelope.

        Only safe to call when the drive is IDLE or REACHED. Subclasses that
        own a hardware counter (e.g. Tinkerforge bricklet) must override this
        to keep the hardware and software counters in sync.
        """
        if self._state not in (DriveState.IDLE, DriveState.REACHED):
            raise RuntimeError(
                f"set_current_position refused: drive {self.key} state={self._state}"
            )
        self._current_position = value
        self._target_position = value

    def get_midpoint_position(self) -> float:
        """Median of min and max — default midpoint for linear drives.

        Overridden by TinkerforgeDrive for radial drives, which compute a
        camera-level (roll = 0°) midpoint from IMU checkpoints when available.
        """
        return (self.get_min_position() + self.get_max_position()) / 2.0

    @property
    def is_simulated(self) -> bool:
        """True if the drive is running in software simulation (no real hardware)."""
        return False

    @property
    def setup_error(self) -> str | None:
        """Human-readable reason why setup failed, or None."""
        return None
