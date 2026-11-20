"""RigModel: rig config + current axis positions (steps) per camera.

Entry point for later steps (simulated drives, simulated IMU, 3D view).
Positions are stored in motor steps, exactly like the real drives.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

from src.sim import fov as fov_mod
from src.sim import kinematics as kin
from src.sim.rig_config import load_rig_config
from src.sim.specimen import Specimen

if TYPE_CHECKING:
    from pathlib import Path

    from src.sim.rig_config import CameraConfig, RigConfig


class RigModel:
    def __init__(self, config: RigConfig) -> None:
        self.config = config
        self.specimen = Specimen(config.specimen)
        # initial: h = 0 (weld height), tilt = 0
        self._steps: dict[str, dict[str, int]] = {
            cid: {
                "a": kin.units_to_steps(c.axis_a, 0.0),
                "b": kin.units_to_steps(c.axis_b, 0.0),
            }
            for cid, c in config.cameras.items()
        }

    @classmethod
    def from_yaml(cls, path: str | Path | None = None) -> RigModel:
        return cls(load_rig_config(path))

    def cam(self, cam_id: str) -> CameraConfig:
        return self.config.cameras[cam_id]

    # --- state -------------------------------------------------------------
    def set_steps(self, cam_id: str, axis: Literal["a", "b"], steps: int) -> None:
        self._steps[cam_id][axis] = int(steps)

    def get_steps(self, cam_id: str, axis: Literal["a", "b"]) -> int:
        return self._steps[cam_id][axis]

    def height(self, cam_id: str) -> float:
        """Pivot height h in mm."""
        return kin.steps_to_units(self.cam(cam_id).axis_a, self._steps[cam_id]["a"])

    def tilt(self, cam_id: str) -> float:
        """Tilt theta in degrees."""
        return kin.steps_to_units(self.cam(cam_id).axis_b, self._steps[cam_id]["b"])

    def set_pose(self, cam_id: str, h: float | None = None, theta_deg: float | None = None) -> None:
        c = self.cam(cam_id)
        if h is not None:
            self._steps[cam_id]["a"] = kin.units_to_steps(c.axis_a, h)
        if theta_deg is not None:
            self._steps[cam_id]["b"] = kin.units_to_steps(c.axis_b, theta_deg)

    # --- queries -----------------------------------------------------------
    def pose(self, cam_id: str) -> kin.CameraPose:
        return kin.forward(self.specimen, self.cam(cam_id), self.height(cam_id), self.tilt(cam_id))

    def hit(self, cam_id: str, mode: Literal["flat", "bead"] = "bead") -> kin.SurfaceHit:
        return kin.intersect_surface(self.specimen, self.cam(cam_id), self.pose(cam_id), mode)

    def footprint(self, cam_id: str) -> fov_mod.Footprint:
        return fov_mod.footprint(self.specimen, self.cam(cam_id), self.pose(cam_id))

    def limits(self, cam_id: str) -> dict[str, kin.LimitStatus]:
        c = self.cam(cam_id)
        return {
            "a": kin.check_limits(c.axis_a, self.height(cam_id)),
            "b": kin.check_limits(c.axis_b, self.tilt(cam_id)),
        }

    def solve(
        self,
        cam_id: str,
        target_z: float,
        theta_deg: float | None = None,
        view_offset_deg: float = 0.0,
    ) -> kin.IKResult:
        """IK for surface point z (perpendicular view by default, or fixed tilt)."""
        return kin.solve_view(self.specimen, self.cam(cam_id), target_z, theta_deg, view_offset_deg)

    def move_to(
        self,
        cam_id: str,
        target_z: float,
        theta_deg: float | None = None,
        view_offset_deg: float = 0.0,
    ) -> kin.IKResult:
        """Solve and, if reachable, apply the resulting step positions."""
        res = self.solve(cam_id, target_z, theta_deg, view_offset_deg)
        if res.reachable:
            self._steps[cam_id]["a"] = res.steps_a
            self._steps[cam_id]["b"] = res.steps_b
        return res

    def imu_angles(self, cam_id: str) -> tuple[float, float]:
        """Simulated IMU (roll_deg, pitch_deg): roll = sign * tilt + mounting offset."""
        m = self.cam(cam_id).imu
        return (m.roll_sign * self.tilt(cam_id) + m.roll_offset_deg, m.pitch_offset_deg)
