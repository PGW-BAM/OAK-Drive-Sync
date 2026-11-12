"""Provisional Soll/Ist target format for the digital twin (headless, no UI).

A target says "camera ``cam_id`` should look at specimen height ``z`` (mm)". JSON example::

    [
      {"cam_id": "cam1", "z": 0.0},
      {"cam_id": "cam1", "z": 12.5, "theta_deg": 10.0},
      {"cam_id": "cam2", "z": -8.0, "normal": [-0.97, 0.0, 0.24]}
    ]

- no extra key: view perpendicular to the local surface (weld bead slope included)
- ``normal`` ``[nx, ny, nz]``: surface normal (specimen frame, pointing out of the face the
  camera looks at) the camera should be anti-parallel to. The camera can only tilt in the x-z
  plane, so ``ny`` must be ~0 and the normal must point toward the camera (cam1: nx > 0,
  cam2: nx < 0). Otherwise the target is reported as not reachable.
- ``theta_deg``: fixed tilt angle instead (only the height is solved). Mutually exclusive with
  ``normal``.

This format is PROVISIONAL (not yet agreed with the real Soll/Ist data source).
"""

from __future__ import annotations

import json
import math
from typing import TYPE_CHECKING, Literal

from pydantic import BaseModel, TypeAdapter, model_validator

from src.sim import kinematics as kin

if TYPE_CHECKING:
    from src.sim.rig_model import RigModel

_NORMAL_Y_TOL = 0.05  # |ny| / |n| above this cannot be reached (no yaw axis)


class Target(BaseModel):
    cam_id: str
    z: float = 0.0
    normal: tuple[float, float, float] | None = None
    theta_deg: float | None = None

    @model_validator(mode="after")
    def _exclusive(self) -> Target:
        if self.normal is not None and self.theta_deg is not None:
            raise ValueError("'normal' and 'theta_deg' are mutually exclusive")
        if self.normal is not None and math.hypot(*self.normal) < 1e-9:
            raise ValueError("'normal' must not be the zero vector")
        return self

    @property
    def mode(self) -> Literal["perpendicular", "normal", "fixed"]:
        if self.normal is not None:
            return "normal"
        return "fixed" if self.theta_deg is not None else "perpendicular"


class TargetResult(BaseModel):
    target: Target
    height_mm: float | None = None
    tilt_deg: float | None = None
    steps_a: int | None = None
    steps_b: int | None = None
    reachable: bool = False
    # "a_max", "b_min", ... or "normal_y", "normal_direction", "unknown_cam"
    violations: tuple[str, ...] = ()
    hit_z: float | None = None  # FK check: where the optical axis really hits (should be ~z)
    incidence_deg: float | None = None


_ADAPTER = TypeAdapter(list[Target])


def parse_targets(text: str | bytes) -> list[Target]:
    """Parse the JSON list; raises ``ValueError`` (incl. pydantic errors) on bad input."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"ungültiges JSON: {exc}") from exc
    if not isinstance(data, list):
        raise ValueError("JSON muss eine Liste von Zielen sein")
    return _ADAPTER.validate_python(data)


def _tilt_from_normal(side: int, normal: tuple[float, float, float]) -> float | str:
    """Tilt (deg) making the optical axis anti-parallel to ``normal``, or a violation name."""
    nx, ny, nz = normal
    if abs(ny) / math.hypot(nx, ny, nz) > _NORMAL_Y_TOL:
        return "normal_y"
    if side * nx <= 0:
        return "normal_direction"
    return math.degrees(math.atan2(-nz, side * nx))


def solve_targets(rig: RigModel, targets: list[Target]) -> list[TargetResult]:
    """IK for every target (the rig state is not modified)."""
    out: list[TargetResult] = []
    for t in targets:
        if t.cam_id not in rig.config.cameras:
            out.append(TargetResult(target=t, violations=("unknown_cam",)))
            continue
        cam = rig.cam(t.cam_id)
        theta: float | None = t.theta_deg
        if t.normal is not None:
            res = _tilt_from_normal(cam.side, t.normal)
            if isinstance(res, str):
                out.append(TargetResult(target=t, violations=(res,)))
                continue
            theta = res
        ik = rig.solve(t.cam_id, t.z, theta_deg=theta)
        pose = kin.forward(rig.specimen, cam, ik.height_mm, ik.tilt_deg)
        hit = kin.intersect_surface(rig.specimen, cam, pose)
        out.append(
            TargetResult(
                target=t,
                height_mm=ik.height_mm,
                tilt_deg=ik.tilt_deg,
                steps_a=ik.steps_a,
                steps_b=ik.steps_b,
                reachable=ik.reachable,
                violations=ik.violations,
                hit_z=hit.z if hit.hit else None,
                incidence_deg=hit.incidence_deg if hit.hit else None,
            )
        )
    return out
