"""Camera pose of the OAK-D 4 Pro rig cameras from axis values (numpy only).

Export for external renderers (e.g. Mitsuba). Same model as src/sim/kinematics.py in
OAK-Drive-Sync; see camera_kinematics.json next to this file for frame, sources and certainty.

Specimen frame (right-handed, mm, degrees):
    origin = weld centre on the specimen mid-plane (half thickness)
    +z     = up, along the specimen length axis (specimen is clamped vertically)
    +x     = normal of the FRONT face, pointing toward cam1 (cam2 looks from -x)
    +y     = z x x, across the specimen width

Kinematic chain per camera (side s = +1 cam1, -1 cam2):
    base -> axis a: vertical linear stage, translation h along +z (pivot height)
         -> axis b: rotation theta about the horizontal ring axis (parallel to y) through the
                    pivot P = (s*(t/2 + D_pivot), 0, h); D_pivot = working_distance - lens_to_pivot
         -> camera: optical centre 38 mm BEHIND the pivot on the optical axis (the ring axis lies
                    between lens and specimen), optical axis always passes through P.
    theta > 0 tilts the optical axis upward (+z).
"""

from __future__ import annotations

import numpy as np

# Defaults (see camera_kinematics.json -> "sources" for certainty)
THICKNESS_MM = 25.0  # t, specimen thickness (drawing)
WORKING_DISTANCE_MM = 300.0  # optical centre -> front face at theta = 0 (user estimate "ca. 30 cm")
LENS_TO_PIVOT_MM = 38.0  # optical centre -> ring axis (CAD, front glass ~37.7-39.7 mm)
SIDE = {"cam1": 1, "cam2": -1}
# Rotation of the sensor about its optical axis relative to "image-up = world-up" (deg).
# cam1 is mounted upside-down per the IMU calibration (roll ~ -90 vs +90 for cam2) -> 180.
# UNVERIFIED: check against the real cam1 image before relying on it.
IMAGE_ROLL_DEG = {"cam1": 180.0, "cam2": 0.0}


def camera_pose(
    axis_values: dict,
    *,
    thickness_mm: float = THICKNESS_MM,
    working_distance_mm: float = WORKING_DISTANCE_MM,
    lens_to_pivot_mm: float = LENS_TO_PIVOT_MM,
    apply_image_roll: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (origin, forward, up) in the specimen frame (mm, unit vectors).

    axis_values: {"cam": "cam1"|"cam2", "h_mm": float, "tilt_deg": float}
      h_mm     axis a: pivot height above the weld centre (z), 0 = level with the weld
      tilt_deg axis b: camera tilt about the ring axis, + = looking up
    origin  = optical centre, forward = optical axis, up = image-up direction (for look_at).
    Mitsuba: look_at(origin=origin, target=origin + forward, up=up); scene units mm.
    """
    cam = axis_values.get("cam", "cam1")
    s = SIDE[cam]
    h = float(axis_values.get("h_mm", 0.0))
    th = np.radians(float(axis_values.get("tilt_deg", 0.0)))

    d_pivot = working_distance_mm - lens_to_pivot_mm
    pivot = np.array([s * (thickness_mm / 2 + d_pivot), 0.0, h])
    forward = np.array([-s * np.cos(th), 0.0, np.sin(th)])
    up = np.array([s * np.sin(th), 0.0, np.cos(th)])
    origin = pivot - lens_to_pivot_mm * forward

    if apply_image_roll and IMAGE_ROLL_DEG.get(cam, 0.0):
        r = np.radians(IMAGE_ROLL_DEG[cam])
        right = np.cross(forward, up)
        up = np.cos(r) * up - np.sin(r) * right  # rotate up about forward
    return origin, forward, up / np.linalg.norm(up)


def surface_hit_z(axis_values: dict, **kw: float) -> float:
    """z where the optical axis hits the (flat) face plane x = s*t/2."""
    cam = axis_values.get("cam", "cam1")
    s = SIDE[cam]
    t = kw.get("thickness_mm", THICKNESS_MM)
    origin, forward, _ = camera_pose(axis_values, **kw)
    tau = (s * origin[0] - t / 2) / (-s * forward[0])
    return float(origin[2] + tau * forward[2])


def _test_reference_pose() -> None:
    # Reference "frontal": h = 0, tilt = 0 -> 300 mm in front of the weld centre, looking at it.
    o, f, u = camera_pose({"cam": "cam1", "h_mm": 0.0, "tilt_deg": 0.0}, apply_image_roll=False)
    assert np.allclose(o, [12.5 + 300.0, 0.0, 0.0])
    assert np.allclose(f, [-1.0, 0.0, 0.0]) and np.allclose(u, [0.0, 0.0, 1.0])
    o2, f2, _ = camera_pose({"cam": "cam2", "h_mm": 0.0, "tilt_deg": 0.0})
    assert np.allclose(o2, [-312.5, 0.0, 0.0]) and np.allclose(f2, [1.0, 0.0, 0.0])
    # cam1 image roll 180 deg flips image-up
    _, _, u1 = camera_pose({"cam": "cam1", "h_mm": 0.0, "tilt_deg": 0.0})
    assert np.allclose(u1, [0.0, 0.0, -1.0])
    # Tilted 20 deg at h = 0: axis passes through the pivot and hits z = 262 * tan(20 deg)
    z = surface_hit_z({"cam": "cam1", "h_mm": 0.0, "tilt_deg": 20.0})
    assert abs(z - 262.0 * np.tan(np.radians(20.0))) < 1e-9
    # Weld-centred tilt: h = -262 * tan(theta) keeps the axis on the weld centre
    th = 15.0
    z0 = surface_hit_z({"cam": "cam2", "h_mm": -262.0 * np.tan(np.radians(th)), "tilt_deg": th})
    assert abs(z0) < 1e-9
    print("camera_pose: reference pose tests OK")


if __name__ == "__main__":
    _test_reference_pose()
