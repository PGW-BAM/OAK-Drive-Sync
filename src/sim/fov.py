"""Camera frustum footprint on the face plane x = s*t/2 (specimen coordinates).

HFOV/VFOV are config PLACEHOLDERS (see rig_config.FovConfig). Image-right = ``pose.right``,
image-up = ``pose.up``. The footprint is computed on the flat face (bead ignored).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.sim.kinematics import vadd, vnorm, vscale

if TYPE_CHECKING:
    from src.sim.kinematics import CameraPose
    from src.sim.rig_config import CameraConfig
    from src.sim.specimen import Specimen

Point2 = tuple[float, float]  # (y, z) on the face


@dataclass(frozen=True)
class Footprint:
    valid: bool  # False if any corner ray misses the face plane
    polygon: tuple[Point2, ...] = ()  # corners in order: (-,-), (+,-), (+,+), (-,+) image corners
    weld_coverage: float = 0.0  # fraction of the weld centreline (z=0, |y|<b/2) inside
    gsd_h_mm_px: float = math.nan  # mm per pixel along image-horizontal at the optical-axis hit
    gsd_v_mm_px: float = math.nan  # mm per pixel along image-vertical (incl. foreshortening)

    @property
    def weld_in_view(self) -> bool:
        return self.weld_coverage >= 0.999

    @property
    def width(self) -> float:
        ys = [p[0] for p in self.polygon]
        return max(ys) - min(ys) if ys else math.nan

    @property
    def height(self) -> float:
        zs = [p[1] for p in self.polygon]
        return max(zs) - min(zs) if zs else math.nan


def _inside_convex(poly: tuple[Point2, ...], p: Point2) -> bool:
    sign = 0.0
    for (y0, z0), (y1, z1) in zip(poly, poly[1:] + poly[:1], strict=True):
        c = (y1 - y0) * (p[1] - z0) - (z1 - z0) * (p[0] - y0)
        if abs(c) < 1e-9:
            continue
        if sign == 0.0:
            sign = c
        elif c * sign < 0:
            return False
    return True


def footprint(spec: Specimen, cam: CameraConfig, pose: CameraPose) -> Footprint:
    s = cam.side
    fov = cam.fov
    th = math.tan(math.radians(fov.hfov_deg) / 2)
    tv = math.tan(math.radians(fov.vfov_deg) / 2)
    plane_x = s * spec.half_thickness
    pts: list[Point2] = []
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        ray = vnorm(
            vadd(
                vadd(pose.direction, vscale(pose.right, sx * th)),
                vscale(pose.up, sy * tv),
            )
        )
        if s * ray[0] >= -1e-9:
            return Footprint(False)
        tau = (plane_x - pose.lens[0]) / ray[0]
        if tau <= 0:
            return Footprint(False)
        p = vadd(pose.lens, vscale(ray, tau))
        pts.append((p[1], p[2]))
    poly = tuple(pts)
    # weld centreline coverage by sampling
    n = 200
    hb = spec.cfg.gauge_width / 2
    inside = sum(_inside_convex(poly, (-hb + 2 * hb * i / n, 0.0)) for i in range(n + 1))
    # GSD at the optical axis hit on the flat face
    dist = (plane_x - pose.lens[0]) / pose.direction[0]
    cos_i = max(-s * pose.direction[0], 1e-6)
    gsd_h = 2 * dist * th / fov.width_px
    gsd_v = 2 * dist * tv / fov.height_px / cos_i
    # mm/px along image-up on the face = scale along the face-projected image-up direction
    return Footprint(True, poly, inside / (n + 1), gsd_h, gsd_v)
