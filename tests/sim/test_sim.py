"""Tests for the headless digital-twin kinematics core (src/sim)."""

from __future__ import annotations

import math

import pytest

from src.sim import RigModel
from src.sim import kinematics as kin
from src.sim.fov import footprint
from src.sim.specimen import Specimen
from tests.sim.conftest import neutral_rig_config

CAMS = ["cam1", "cam2"]


@pytest.fixture(scope="module")
def rig_cfg():
    return neutral_rig_config()


@pytest.fixture()
def rig(rig_cfg) -> RigModel:
    return RigModel(rig_cfg)


# --- specimen --------------------------------------------------------------------------------


def test_half_width_grip_and_gauge(rig: RigModel) -> None:
    sp = rig.specimen
    assert sp.half_width(0.0) == pytest.approx(50.0)
    assert sp.half_width(sp.gauge_half_length - 1) == pytest.approx(50.0)
    assert sp.half_width(299.0) == pytest.approx(75.0)
    assert sp.half_width(-299.0) == pytest.approx(75.0)
    assert 50.0 < sp.half_width(sp.gauge_half_length + sp.fillet_length / 2) < 75.0
    assert sp.gauge_length == pytest.approx(156.4, abs=0.5)  # drawing: ~156 mm


def test_length_is_parametric(rig_cfg) -> None:
    longer = rig_cfg.specimen.model_copy(update={"length": 700.0})
    base = Specimen(rig_cfg.specimen).gauge_length
    assert Specimen(longer).gauge_length == pytest.approx(base + 100)


def test_bead_normal_horizontal_at_apex_tilted_on_flanks(rig: RigModel) -> None:
    sp = rig.specimen
    assert sp.bead_height(0.0) == pytest.approx(2.0)
    assert sp.bead_height(10.0) == 0.0
    n0 = sp.surface_normal(0.0, 1)
    assert n0 == pytest.approx((1.0, 0.0, 0.0))
    up, low = sp.surface_normal(4.0, 1), sp.surface_normal(-4.0, 1)
    assert up[2] > 0.05 > -0.05 > low[2]  # normal tilts away from the bead on each flank
    assert sp.surface_normal(0.0, -1)[0] == pytest.approx(-1.0)
    assert sp.surface_normal(50.0, 1) == pytest.approx((1.0, 0.0, 0.0))


def test_bead_slope_matches_numeric_derivative(rig: RigModel) -> None:
    sp = rig.specimen
    for z in (-7.0, -3.0, 1.0, 5.0):
        num = (sp.bead_height(z + 1e-5) - sp.bead_height(z - 1e-5)) / 2e-5
        assert sp.bead_slope(z) == pytest.approx(num, abs=1e-6)


def test_mesh_basic(rig: RigModel) -> None:
    m = rig.specimen.mesh()
    assert len(m.vertices) > 100 and len(m.faces) > 100
    assert max(v[0] for v in m.vertices) == pytest.approx(14.5)
    assert min(v[0] for v in m.vertices) == pytest.approx(-14.5)
    # a front-face triangle at the weld must have an outward (+x) normal
    best = None
    for f in m.faces:
        a, b, c = (m.vertices[i] for i in f)
        if min(abs(a[2]), abs(b[2]), abs(c[2])) < 1 and min(a[0], b[0], c[0]) > 10:
            best = (a, b, c)
            break
    assert best is not None
    a, b, c = best
    u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    assert kin.vcross(u, v)[0] > 0


# --- steps conversion ------------------------------------------------------------------------


def test_steps_roundtrip_and_sign(rig_cfg) -> None:
    b1, b2 = rig_cfg.cameras["cam1"].axis_b, rig_cfg.cameras["cam2"].axis_b
    assert b1.steps_per_unit == pytest.approx(200 * 8 / 0.8)
    assert kin.units_to_steps(b1, 10.0) == 20000
    assert kin.units_to_steps(b2, 10.0) == -20000
    for ax in (b1, b2, rig_cfg.cameras["cam1"].axis_a):
        for u in (-30.0, -0.5, 0.0, 12.345):
            assert kin.steps_to_units(ax, kin.units_to_steps(ax, u)) == pytest.approx(
                u, abs=0.5 / ax.steps_per_unit
            )
    off = b1.model_copy(update={"zero_offset_steps": 1000})
    assert kin.units_to_steps(off, 0.0) == 1000


# --- limits ----------------------------------------------------------------------------------


def test_soft_limits_and_switches(rig_cfg) -> None:
    b = rig_cfg.cameras["cam1"].axis_b  # travel +-52, soft margin 1, overtravel 1
    assert kin.check_limits(b, 0.0).within_soft
    assert kin.check_limits(b, 51.5).soft_max and not kin.check_limits(b, 51.5).switch_max
    assert kin.check_limits(b, -51.5).soft_min
    assert kin.check_limits(b, 53.0).switch_max
    assert kin.check_limits(b, -53.5).switch_min
    assert not kin.check_limits(b, 52.0).switch_triggered
    custom = b.model_copy(update={"switch_max": 40.0})
    assert kin.check_limits(custom, 41.0).switch_max


def test_ik_reports_violated_limit(rig: RigModel) -> None:
    r = rig.solve("cam1", 0.0, theta_deg=60.0)
    assert not r.reachable and "b_max" in r.violations
    r = rig.solve("cam1", 400.0, theta_deg=0.0)
    assert not r.reachable and r.violations == ("a_max",)
    r = rig.solve("cam2", 0.0, theta_deg=-60.0)
    assert "b_min" in r.violations
    assert rig.solve("cam1", 20.0, theta_deg=10.0).reachable


# --- FK / IK ---------------------------------------------------------------------------------


@pytest.mark.parametrize("cam_id", CAMS)
def test_theta0_hits_pivot_height(rig: RigModel, cam_id: str) -> None:
    for h in (-100.0, -3.0, 0.0, 42.0):
        rig.set_pose(cam_id, h=h, theta_deg=0.0)
        hit = rig.hit(cam_id, "flat")
        assert hit.hit and hit.z == pytest.approx(h, abs=1e-3)
        assert hit.distance == pytest.approx(300.0)
        assert hit.incidence_deg == pytest.approx(0.0)


@pytest.mark.parametrize("cam_id", CAMS)
def test_known_tan_case(rig: RigModel, cam_id: str) -> None:
    rig.set_pose(cam_id, h=0.0, theta_deg=10.0)
    hit = rig.hit(cam_id, "flat")
    d_pivot = 262.0
    assert hit.z == pytest.approx(d_pivot * math.tan(math.radians(10)), rel=1e-6)
    assert hit.distance == pytest.approx(38.0 + 262.0 / math.cos(math.radians(10)))
    assert hit.incidence_deg == pytest.approx(10.0)
    # optical axis passes through the pivot
    p = rig.pose(cam_id)
    v = kin.vnorm(tuple(a - b for a, b in zip(p.pivot, p.lens, strict=True)))
    assert v == pytest.approx(p.direction)


def test_pose_vectors_orthonormal(rig: RigModel) -> None:
    rig.set_pose("cam2", h=5.0, theta_deg=33.0)
    p = rig.pose("cam2")
    assert kin.vdot(p.direction, p.up) == pytest.approx(0.0, abs=1e-12)
    assert kin.vdot(p.right, p.up) == pytest.approx(0.0, abs=1e-12)
    assert kin.vdot(p.right, p.right) == pytest.approx(1.0)


@pytest.mark.parametrize("cam_id", CAMS)
def test_fixed_tilt_ik_roundtrip(rig: RigModel, cam_id: str) -> None:
    cam = rig.cam(cam_id)
    for z in (-60.0, -8.0, -2.0, 0.0, 3.0, 9.0, 80.0):
        for th in range(-45, 46, 15):
            h = kin.solve_height_for_tilt(rig.specimen, cam, z, float(th))
            pose = kin.forward(rig.specimen, cam, h, float(th))
            hit = kin.intersect_surface(rig.specimen, cam, pose, "bead")
            assert hit.hit and hit.z == pytest.approx(z, abs=1e-6)


@pytest.mark.parametrize("cam_id", CAMS)
def test_perpendicular_ik_roundtrip(rig: RigModel, cam_id: str) -> None:
    for z in (-80.0, -6.0, -2.0, 0.0, 2.0, 6.0, 80.0):
        res = rig.move_to(cam_id, z)
        assert res.reachable
        hit = rig.hit(cam_id)
        assert hit.z == pytest.approx(z, abs=5e-3)  # steps are quantised
        assert hit.incidence_deg < 0.05
        assert rig.tilt(cam_id) == pytest.approx(res.tilt_deg, abs=1e-3)
    # on a flat region the perpendicular view has zero tilt
    assert rig.solve(cam_id, 80.0).tilt_deg == pytest.approx(0.0)


def test_perpendicular_on_bead_flank_is_tilted(rig: RigModel) -> None:
    # upper flank: normal points up, so the perpendicular view looks slightly downward
    assert rig.solve("cam1", 4.0).tilt_deg < -5.0
    assert rig.solve("cam1", -4.0).tilt_deg > 5.0
    assert rig.solve("cam1", 0.0).tilt_deg == pytest.approx(0.0)


def test_bead_changes_distance(rig: RigModel) -> None:
    rig.set_pose("cam1", h=0.0, theta_deg=0.0)
    assert rig.hit("cam1", "flat").distance - rig.hit("cam1", "bead").distance == pytest.approx(2.0)


def test_mirror_symmetry(rig: RigModel) -> None:
    for h, th in ((0.0, 0.0), (30.0, 12.0), (-50.0, -25.0)):
        rig.set_pose("cam1", h=h, theta_deg=th)
        rig.set_pose("cam2", h=h, theta_deg=th)
        p1, p2 = rig.pose("cam1"), rig.pose("cam2")
        for a, b in ((p1.lens, p2.lens), (p1.pivot, p2.pivot), (p1.direction, p2.direction)):
            assert (a[0], a[1], a[2]) == pytest.approx((-b[0], b[1], b[2]))
        h1, h2 = rig.hit("cam1"), rig.hit("cam2")
        assert h1.z == pytest.approx(h2.z)
        assert h1.point[0] == pytest.approx(-h2.point[0])
        assert h1.distance == pytest.approx(h2.distance)


# --- IMU -------------------------------------------------------------------------------------


def test_imu_angles(rig_cfg) -> None:
    rig = RigModel(rig_cfg)
    rig.set_pose("cam1", theta_deg=12.0)
    assert rig.imu_angles("cam1") == pytest.approx((12.0, 0.0))
    cfg = rig_cfg.model_copy(deep=True)
    cfg.cameras["cam2"].imu.roll_sign = -1
    cfg.cameras["cam2"].imu.roll_offset_deg = 90.0
    rig2 = RigModel(cfg)
    rig2.set_pose("cam2", theta_deg=12.0)
    assert rig2.imu_angles("cam2")[0] == pytest.approx(78.0)


def test_steps_drive_pose(rig: RigModel) -> None:
    rig.set_steps("cam1", "b", 20000)
    rig.set_steps("cam2", "b", 20000)
    assert rig.tilt("cam1") == pytest.approx(10.0)
    assert rig.tilt("cam2") == pytest.approx(-10.0)  # direction_sign -1


# --- FOV -------------------------------------------------------------------------------------


@pytest.mark.parametrize("cam_id", CAMS)
def test_footprint_size_at_theta0(rig: RigModel, cam_id: str) -> None:
    rig.set_pose(cam_id, h=0.0, theta_deg=0.0)
    cam = rig.cam(cam_id)
    fp = footprint(rig.specimen, cam, rig.pose(cam_id))
    assert fp.valid
    exp_w = 2 * 300 * math.tan(math.radians(cam.fov.hfov_deg / 2))
    exp_h = 2 * 300 * math.tan(math.radians(cam.fov.vfov_deg / 2))
    assert fp.width == pytest.approx(exp_w, rel=1e-6)
    assert fp.height == pytest.approx(exp_h, rel=1e-6)
    assert fp.weld_in_view  # footprint width ~412 mm >> gauge width 100
    assert fp.gsd_h_mm_px == pytest.approx(fp.width / cam.fov.width_px)


def test_weld_leaves_view_when_height_moves(rig: RigModel) -> None:
    rig.set_pose("cam1", h=200.0, theta_deg=0.0)
    assert not rig.footprint("cam1").weld_in_view
    assert rig.footprint("cam1").weld_coverage == 0.0
    rig.set_pose("cam1", h=0.0, theta_deg=0.0)
    assert rig.footprint("cam1").weld_in_view


def test_footprint_invalid_when_rays_miss(rig_cfg) -> None:
    rig = RigModel(rig_cfg)
    cam = rig_cfg.cameras["cam1"].model_copy(deep=True)
    cam.fov.vfov_deg = 170.0
    rig.set_pose("cam1", theta_deg=40.0)
    assert not footprint(rig.specimen, cam, rig.pose("cam1")).valid
