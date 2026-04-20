"""
Pydantic v2 models for all MQTT message payloads.

Both the Pi controller and Windows controller import these models
to ensure consistent serialization/deserialization.
"""

from __future__ import annotations

from datetime import datetime, timezone
from enum import StrEnum
from typing import Literal

from pydantic import BaseModel, Field


def _now() -> datetime:
    return datetime.now(timezone.utc)


# ──────────────────────────────────────────────
# Enums
# ──────────────────────────────────────────────

class DriveState(StrEnum):
    IDLE = "idle"
    MOVING = "moving"
    REACHED = "reached"
    FAULT = "fault"
    HOMING = "homing"


class CameraState(StrEnum):
    ONLINE = "online"
    OFFLINE = "offline"
    CAPTURING = "capturing"
    ERROR = "error"


class DriveErrorType(StrEnum):
    STALL = "stall"
    LIMIT_SWITCH = "limit_switch"
    TIMEOUT = "timeout"
    GPIO_FAULT = "gpio_fault"


class AlertSeverity(StrEnum):
    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


# ──────────────────────────────────────────────
# Command Payloads (Windows → Pi)
# ──────────────────────────────────────────────

class MoveCommand(BaseModel):
    sequence_id: str
    drive_axis: Literal["a", "b"]
    target_position: float
    speed: float = Field(default=1.0, ge=0.0, le=1.0)
    checkpoint_name: str | None = None  # if set, Pi runs IMU drift check after settling
    target_angle_deg: float | None = None
    active_angle: Literal["roll", "pitch"] | None = None
    resync_position: float | None = None
    timestamp: datetime = Field(default_factory=_now)


class HomeCommand(BaseModel):
    sequence_id: str
    drive_axis: Literal["a", "b"]
    timestamp: datetime = Field(default_factory=_now)


class StopCommand(BaseModel):
    sequence_id: str | None = None
    drive_axis: Literal["a", "b"] | None = None  # None = stop all
    timestamp: datetime = Field(default_factory=_now)


# ──────────────────────────────────────────────
# Status Payloads (Pi → Windows)
# ──────────────────────────────────────────────

class DrivePosition(BaseModel):
    sequence_id: str | None = None
    drive_axis: Literal["a", "b"]
    current_position: float
    target_position: float | None = None
    state: DriveState
    timestamp: datetime = Field(default_factory=_now)


class CameraStatus(BaseModel):
    cam_id: str
    state: CameraState
    fps: float | None = None
    resolution: str | None = None
    timestamp: datetime = Field(default_factory=_now)


# ──────────────────────────────────────────────
# Health Beacons
# ──────────────────────────────────────────────

class PiHealth(BaseModel):
    online: bool = True
    cpu_temp_c: float = 0.0
    uptime_s: int = 0
    drive_states: dict[str, str] = Field(default_factory=dict)
    timestamp: datetime = Field(default_factory=_now)


class WinControllerHealth(BaseModel):
    online: bool = True
    cameras_connected: list[str] = Field(default_factory=list)
    active_sequence: str | None = None
    timestamp: datetime = Field(default_factory=_now)


class CameraHealth(BaseModel):
    cam_id: str
    online: bool = True
    ip_address: str | None = None
    mx_id: str | None = None
    timestamp: datetime = Field(default_factory=_now)


# ──────────────────────────────────────────────
# Error Events
# ──────────────────────────────────────────────

class DriveError(BaseModel):
    sequence_id: str | None = None
    drive_axis: Literal["a", "b"]
    error_type: DriveErrorType
    message: str
    timestamp: datetime = Field(default_factory=_now)


class CameraError(BaseModel):
    cam_id: str
    error_type: str
    message: str
    timestamp: datetime = Field(default_factory=_now)


class OrchestrationError(BaseModel):
    event: str
    sequence_id: str | None = None
    cam_id: str | None = None
    message: str
    timestamp: datetime = Field(default_factory=_now)


# ──────────────────────────────────────────────
# Monitoring
# ──────────────────────────────────────────────

class ConnectivityState(BaseModel):
    pi_online: bool
    broker_connected: bool
    cameras: dict[str, CameraState] = Field(default_factory=dict)
    drives: dict[str, DriveState] = Field(default_factory=dict)
    last_update: datetime = Field(default_factory=_now)


# ──────────────────────────────────────────────
# Capture Sequence
# ──────────────────────────────────────────────

class PositionTarget(BaseModel):
    drive_a: float
    drive_b: float


class CaptureStep(BaseModel):
    cam_id: str
    position: PositionTarget
    settling_delay_ms: int = 150


class CaptureSequence(BaseModel):
    sequence_id: str
    name: str
    mode: Literal["sequential", "parallel"] = "sequential"
    steps: list[CaptureStep]
    repeat_count: int = 1
    created_at: datetime = Field(default_factory=_now)


# ──────────────────────────────────────────────
# Position Presets (GUI)
# ──────────────────────────────────────────────

class PositionPreset(BaseModel):
    name: str
    cam1_a: float = 0.0
    cam1_b: float = 0.0   # last-known motor position for cam1:b (seed only — drifts)
    cam2_a: float = 0.0
    cam2_b: float = 0.0   # last-known motor position for cam2:b (seed only — drifts)
    # Target IMU angles for radial drives. When set, sequences converge to this angle
    # instead of open-loop moving to the motor position above. The motor position is
    # used only as a seed to speed up the Newton search.
    cam1_b_angle_deg: float | None = None
    cam2_b_angle_deg: float | None = None

    def as_drive_targets(self) -> dict[str, float]:
        return {
            "cam1:a": self.cam1_a,
            "cam1:b": self.cam1_b,
            "cam2:a": self.cam2_a,
            "cam2:b": self.cam2_b,
        }

    def has_angle_target(self, cam_id: str) -> bool:
        return getattr(self, f"{cam_id}_b_angle_deg", None) is not None


class SequenceConfig(BaseModel):
    positions: list[str]  # list of PositionPreset names
    dwell_time_ms: int = 500  # pause at each position
    repeat_count: int = 1  # 0 = infinite
    settling_delay_ms: int = 150


class CheckpointRef(BaseModel):
    """Reference to a named IMU checkpoint within a sequence step."""
    cam_id: Literal["cam1", "cam2"]
    name: str


class SequenceStep(BaseModel):
    """One step in a mixed sequence — open-loop position preset OR closed-loop checkpoint."""
    type: Literal["position", "checkpoint"]
    preset: PositionPreset | None = None      # set when type="position"
    checkpoint: CheckpointRef | None = None   # set when type="checkpoint"


class SavedSequence(BaseModel):
    """Persisted sequence definition (config/sequences/*.yaml)."""
    name: str
    steps: list[SequenceStep]
    dwell_time_ms: int = 500
    repeat_count: int = 1
    created_at: datetime = Field(default_factory=_now)


# ──────────────────────────────────────────────
# Email Alert
# ──────────────────────────────────────────────

class AlertEvent(BaseModel):
    alert_type: str
    severity: AlertSeverity
    component: str
    message: str
    system_state: ConnectivityState | None = None
    timestamp: datetime = Field(default_factory=_now)


# ──────────────────────────────────────────────
# IMU / Drift Detection
# ──────────────────────────────────────────────

class IMUAngle(BaseModel):
    """Published by Windows continuously (~2 Hz) and in response to IMUCheckRequest."""
    cam_id: str
    roll_deg: float
    pitch_deg: float
    request_id: str | None = None  # echoed from IMUCheckRequest; None for background publishes
    timestamp: datetime = Field(default_factory=_now)


class IMUCheckRequest(BaseModel):
    """Published by Pi to request a fresh IMU reading from Windows."""
    request_id: str  # uuid4 — echoed back in IMUAngle response for correlation
    cam_id: str
    checkpoint_name: str
    timestamp: datetime = Field(default_factory=_now)


class DriftDetectionEvent(BaseModel):
    """Published by Pi when drive drift is detected and a correction is attempted.

    Closed-loop converge() uses this model per iteration: the final event for a move
    sets `corrected=True` once |drift_deg| <= threshold, or `corrected=False` once
    `max_iterations` is exhausted. `iterations` counts attempts in the current move.
    `resynced_to` records the motor-counter value written after a successful converge
    so cumulative nudges don't permanently offset the drive's min/max envelope.
    """
    request_id: str
    cam_id: str
    drive_axis: Literal["a", "b"]  # always "b" (Tinkerforge radial drives only)
    checkpoint_name: str
    expected_angle_deg: float
    actual_angle_deg: float
    drift_deg: float    # actual − expected (signed)
    correction_steps: int   # steps applied (signed); 0 if steps_per_degree not configured
    corrected: bool     # True once |drift_deg| <= threshold
    iterations: int = 1
    resynced_to: float | None = None
    timestamp: datetime = Field(default_factory=_now)
