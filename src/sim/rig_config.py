"""Pydantic v2 config models for the simulated rig + loader for ``config/sim_rig.yaml``.

All lengths in mm, all angles in degrees. Coordinate system: see ``src/sim/kinematics.py``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import structlog
import yaml
from pydantic import BaseModel, Field, model_validator

log = structlog.get_logger(__name__)

DEFAULT_RIG_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "sim_rig.yaml"


class BeadConfig(BaseModel):
    """Weld bead (Wulst) profile, identical on both faces. PLACEHOLDER until scan data."""

    profile: Literal["cos2", "ellipse"] = "cos2"
    height: float = Field(2.0, ge=0)  # mm above the face
    width: float = Field(19.0, gt=0)  # mm total width along z


class SpecimenConfig(BaseModel):
    """Dogbone with horizontal weld at mid-length (z=0)."""

    length: float = Field(600.0, gt=0)  # total length L
    grip_width: float = Field(150.0, gt=0)  # B
    gauge_width: float = Field(100.0, gt=0)  # b
    thickness: float = Field(25.0, gt=0)  # t
    fillet_radius: float = Field(200.0, gt=0)  # R, ESTIMATE from drawing
    clamp_length: float = Field(125.0, ge=0)  # clamped length per end (full width B)
    bead: BeadConfig = BeadConfig()

    @model_validator(mode="after")
    def _check(self) -> SpecimenConfig:
        if self.grip_width < self.gauge_width:
            raise ValueError("grip_width must be >= gauge_width")
        if (self.grip_width - self.gauge_width) / 2 > self.fillet_radius:
            raise ValueError("fillet_radius too small for width transition")
        return self


class AxisConfig(BaseModel):
    """One drive axis: steps <-> physical unit (mm for axis a, deg for axis b) + limits."""

    steps_per_unit: float = Field(gt=0)
    direction_sign: Literal[1, -1] = 1
    zero_offset_steps: int = 0  # step count at physical position 0
    travel_min: float  # nominal travel end (physical units)
    travel_max: float
    soft_margin: float = Field(0.0, ge=0)  # soft limits sit this far inside the travel ends
    switch_overtravel: float = Field(0.0, ge=0)  # virtual switches sit this far beyond the ends
    # Explicit switch positions override the default (travel end +/- overtravel)
    switch_min: float | None = None
    switch_max: float | None = None

    @model_validator(mode="after")
    def _check(self) -> AxisConfig:
        if self.travel_min >= self.travel_max:
            raise ValueError("travel_min must be < travel_max")
        if self.soft_min >= self.soft_max:
            raise ValueError("soft_margin leaves no usable range")
        return self

    @property
    def soft_min(self) -> float:
        return self.travel_min + self.soft_margin

    @property
    def soft_max(self) -> float:
        return self.travel_max - self.soft_margin

    @property
    def switch_min_pos(self) -> float:
        return (
            self.switch_min
            if self.switch_min is not None
            else (self.travel_min - self.switch_overtravel)
        )

    @property
    def switch_max_pos(self) -> float:
        return (
            self.switch_max
            if self.switch_max is not None
            else (self.travel_max + self.switch_overtravel)
        )


class FovConfig(BaseModel):
    """PLACEHOLDER values: verify against the OAK-D 4 Pro datasheet and the active sensor/mode."""

    hfov_deg: float = Field(69.0, gt=0, lt=180)
    vfov_deg: float = Field(55.0, gt=0, lt=180)
    width_px: int = Field(1920, gt=0)
    height_px: int = Field(1080, gt=0)


class ImuMount(BaseModel):
    """Mapping tilt -> IMU angles: roll = roll_sign * tilt + roll_offset_deg."""

    roll_sign: Literal[1, -1] = 1
    roll_offset_deg: float = 0.0
    pitch_offset_deg: float = 0.0


class CameraConfig(BaseModel):
    side: Literal[1, -1]  # +1 = cam1 (front face), -1 = cam2 (back face)
    working_distance: float = Field(300.0, gt=0)  # lens -> face at tilt 0
    lens_to_pivot: float = Field(38.0, ge=0)  # pivot lies between lens and specimen
    axis_a: AxisConfig  # vertical linear drive, unit mm = height h of the pivot
    axis_b: AxisConfig  # tilt drive, unit deg
    fov: FovConfig = FovConfig()
    imu: ImuMount = ImuMount()

    @property
    def pivot_distance(self) -> float:
        """Horizontal distance pivot axis -> specimen face (D_pivot)."""
        return self.working_distance - self.lens_to_pivot


class RigConfig(BaseModel):
    specimen: SpecimenConfig = SpecimenConfig()
    cameras: dict[str, CameraConfig]


def load_rig_config(path: str | Path | None = None) -> RigConfig:
    """Load and validate the rig config (default: ``config/sim_rig.yaml``)."""
    p = Path(path) if path else DEFAULT_RIG_CONFIG_PATH
    with p.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    cfg = RigConfig.model_validate(data)
    log.info("sim_rig_loaded", path=str(p), cameras=sorted(cfg.cameras))
    return cfg
