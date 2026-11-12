"""Kinematics of one camera: steps <-> physical, FK, surface hit, IK, limit checks.

Coordinate system (specimen frame, mm, degrees)
-----------------------------------------------
Origin at the weld centre on the specimen mid-plane; z up along the specimen length;
x = normal of the front face pointing toward cam1; y across the width.
Front face at x=+t/2, back face at x=-t/2. Camera side s=+1 (cam1) / -1 (cam2).

Axis a (vertical linear drive) sets the pivot height h (mm). Axis b tilts the camera about the
horizontal pivot axis (parallel to y). Pivot P = (s*(t/2 + D_pivot), 0, h) with
D_pivot = working_distance - lens_to_pivot. Tilt theta (positive = optical axis up):
    d    = (-s*cos(theta), 0, sin(theta))          optical direction
    lens = P - lens_to_pivot * d                    pivot lies between lens and specimen
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    from src.sim.rig_config import AxisConfig, CameraConfig
    from src.sim.specimen import Specimen, Vec3

# --- tiny vector helpers --------------------------------------------------------------------


def vadd(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vscale(a: Vec3, k: float) -> Vec3:
    return (a[0] * k, a[1] * k, a[2] * k)


def vdot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vnorm(a: Vec3) -> Vec3:
    n = math.sqrt(vdot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


# --- steps <-> physical ----------------------------------------------------------------------


def steps_to_units(axis: AxisConfig, steps: float) -> float:
    """Motor steps -> mm (axis a) or degrees (axis b)."""
    return axis.direction_sign * (steps - axis.zero_offset_steps) / axis.steps_per_unit


def units_to_steps(axis: AxisConfig, units: float) -> int:
    """mm / degrees -> motor steps (rounded)."""
    return round(axis.zero_offset_steps + axis.direction_sign * units * axis.steps_per_unit)


# --- limits ------------------------------------------------------------------------------------


@dataclass(frozen=True)
class LimitStatus:
    soft_min: bool  # value below soft minimum
    soft_max: bool
    switch_min: bool  # virtual min switch triggered
    switch_max: bool

    @property
    def within_soft(self) -> bool:
        return not (self.soft_min or self.soft_max)

    @property
    def switch_triggered(self) -> bool:
        return self.switch_min or self.switch_max


def check_limits(axis: AxisConfig, value: float) -> LimitStatus:
    """Evaluate soft limits and virtual limit switches for a physical position."""
    return LimitStatus(
        soft_min=value < axis.soft_min,
        soft_max=value > axis.soft_max,
        switch_min=value <= axis.switch_min_pos,
        switch_max=value >= axis.switch_max_pos,
    )


# --- forward kinematics ------------------------------------------------------------------------


@dataclass(frozen=True)
class CameraPose:
    lens: Vec3
    pivot: Vec3
    direction: Vec3  # optical axis (unit)
    up: Vec3  # image-up (unit, perpendicular to direction)
    right: Vec3  # image-right (unit) = direction x up


def forward(spec: Specimen, cam: CameraConfig, h: float, theta_deg: float) -> CameraPose:
    """Forward kinematics: pivot height h (mm), tilt theta (deg) -> camera pose."""
    s = cam.side
    th = math.radians(theta_deg)
    pivot: Vec3 = (s * (spec.half_thickness + cam.pivot_distance), 0.0, h)
    d: Vec3 = (-s * math.cos(th), 0.0, math.sin(th))
    lens = vadd(pivot, vscale(d, -cam.lens_to_pivot))
    up: Vec3 = (s * math.sin(th), 0.0, math.cos(th))
    return CameraPose(lens, pivot, d, up, vcross(d, up))


@dataclass(frozen=True)
class SurfaceHit:
    hit: bool
    point: Vec3 = (0.0, 0.0, 0.0)
    distance: float = math.nan  # lens -> hit point
    incidence_deg: float = math.nan  # angle between -optical axis and surface normal
    normal: Vec3 = (0.0, 0.0, 0.0)
    on_specimen: bool = False  # |z| within specimen length

    @property
    def z(self) -> float:
        return self.point[2]


def intersect_surface(
    spec: Specimen,
    cam: CameraConfig,
    pose: CameraPose,
    mode: Literal["flat", "bead"] = "bead",
) -> SurfaceHit:
    """Intersect the optical axis (from the lens) with the face; 'bead' honours the profile."""
    s = cam.side
    d, o = pose.direction, pose.lens
    ds = s * d[0]  # component pointing away from the face's outside: must be < 0
    if ds >= -1e-9:
        return SurfaceHit(False)
    t2 = spec.half_thickness
    # tau along ray (unit d). Face level s*x = t/2 + f: tau(f) = (s*o_x - t/2 - f) / -ds
    tau_flat = (s * o[0] - t2) / -ds
    if tau_flat < 0:
        return SurfaceHit(False)

    def g(tau: float) -> float:
        p = vadd(o, vscale(d, tau))
        return s * p[0] - t2 - spec.bead_height(p[2])

    if mode == "flat":
        tau = tau_flat
    else:
        lo_b = max((s * o[0] - t2 - spec.cfg.bead.height) / -ds, 0.0)
        n = 64
        taus = [lo_b + (tau_flat - lo_b) * i / n for i in range(n + 1)]
        lo, hi = taus[0], taus[-1]
        for a, b in zip(taus, taus[1:], strict=False):  # first sign change = first hit
            if g(b) <= 0.0:
                lo, hi = a, b
                break
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if g(mid) > 0.0:
                lo = mid
            else:
                hi = mid
        tau = 0.5 * (lo + hi)
    p = vadd(o, vscale(d, tau))
    normal = (s, 0.0, 0.0) if mode == "flat" else spec.surface_normal(p[2], s)
    cosi = max(-1.0, min(1.0, -vdot(d, normal)))
    return SurfaceHit(
        True, p, tau, math.degrees(math.acos(cosi)), normal, abs(p[2]) <= spec.cfg.length / 2
    )


# --- inverse kinematics ------------------------------------------------------------------------


@dataclass(frozen=True)
class IKResult:
    target_z: float
    height_mm: float
    tilt_deg: float
    steps_a: int
    steps_b: int
    reachable: bool
    violations: tuple[str, ...] = field(default=())  # e.g. ("a_max", "b_min")


def solve_height_for_tilt(
    spec: Specimen, cam: CameraConfig, target_z: float, theta_deg: float
) -> float:
    """Pivot height h so that the optical axis at tilt theta hits the surface at target_z.

    Exact: P + tau*d = Q  =>  h = z0 - (D_pivot - f(z0)) * tan(theta).
    """
    f = spec.bead_height(target_z)
    return target_z - (cam.pivot_distance - f) * math.tan(math.radians(theta_deg))


def solve_view(
    spec: Specimen,
    cam: CameraConfig,
    target_z: float,
    theta_deg: float | None = None,
    view_offset_deg: float = 0.0,
) -> IKResult:
    """IK: aim at surface point z0.

    theta_deg=None -> perpendicular view (axis anti-parallel to local normal, tan(theta)=f'(z0)),
    plus ``view_offset_deg`` (tilt added on top). theta_deg given -> fixed tilt, solve h only.
    """
    if theta_deg is None:
        theta = math.degrees(math.atan(spec.bead_slope(target_z))) + view_offset_deg
    else:
        theta = theta_deg
    h = solve_height_for_tilt(spec, cam, target_z, theta)
    viol: list[str] = []
    for name, ax, val in (("a", cam.axis_a, h), ("b", cam.axis_b, theta)):
        st = check_limits(ax, val)
        if st.soft_min:
            viol.append(f"{name}_min")
        if st.soft_max:
            viol.append(f"{name}_max")
    return IKResult(
        target_z,
        h,
        theta,
        units_to_steps(cam.axis_a, h),
        units_to_steps(cam.axis_b, theta),
        not viol,
        tuple(viol),
    )
