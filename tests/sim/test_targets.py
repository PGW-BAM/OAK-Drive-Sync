"""Soll/Ist target format + headless IK (src/sim/targets.py)."""

from __future__ import annotations

import math

import pytest

from src.sim.targets import Target, parse_targets, solve_targets
from tests.sim.conftest import neutral_rig


def test_parse_valid_and_defaults() -> None:
    ts = parse_targets('[{"cam_id": "cam1"}, {"cam_id": "cam2", "z": 5, "theta_deg": 10}]')
    assert ts[0].z == 0.0
    assert ts[0].mode == "perpendicular"
    assert ts[1].mode == "fixed"


@pytest.mark.parametrize(
    "text",
    [
        "not json",
        '{"cam_id": "cam1"}',  # not a list
        '[{"z": 1.0}]',  # cam_id missing
        '[{"cam_id": "cam1", "normal": [1, 0, 0], "theta_deg": 3}]',  # exclusive
        '[{"cam_id": "cam1", "normal": [0, 0, 0]}]',
        '[{"cam_id": "cam1", "normal": [1, 0]}]',
    ],
)
def test_parse_invalid(text: str) -> None:
    with pytest.raises(ValueError):
        parse_targets(text)


def test_perpendicular_weld_hits_target() -> None:
    rig = neutral_rig()
    res = solve_targets(rig, [Target(cam_id="cam1", z=0.0), Target(cam_id="cam2", z=0.0)])
    for r in res:
        assert r.reachable
        assert r.tilt_deg == pytest.approx(0.0, abs=1e-9)  # bead slope is 0 at its crest
        assert r.hit_z == pytest.approx(0.0, abs=1e-6)
        assert r.incidence_deg == pytest.approx(0.0, abs=1e-6)


def test_solve_does_not_move_rig() -> None:
    rig = neutral_rig()
    before = (rig.get_steps("cam1", "a"), rig.get_steps("cam1", "b"))
    solve_targets(rig, [Target(cam_id="cam1", z=30.0)])
    assert (rig.get_steps("cam1", "a"), rig.get_steps("cam1", "b")) == before


def test_normal_equals_perpendicular() -> None:
    rig = neutral_rig()
    spec = rig.specimen
    for cam_id, side in (("cam1", 1), ("cam2", -1)):
        z = 4.0  # on the bead flank
        n = spec.surface_normal(z, side)
        a = solve_targets(rig, [Target(cam_id=cam_id, z=z)])[0]
        b = solve_targets(rig, [Target(cam_id=cam_id, z=z, normal=n)])[0]
        assert b.tilt_deg == pytest.approx(a.tilt_deg, abs=1e-9)
        assert b.height_mm == pytest.approx(a.height_mm, abs=1e-9)
        assert b.hit_z == pytest.approx(z, abs=1e-5)


def test_fixed_angle_hits_target_height() -> None:
    rig = neutral_rig()
    r = solve_targets(rig, [Target(cam_id="cam1", z=15.0, theta_deg=20.0)])[0]
    assert r.reachable
    assert r.tilt_deg == 20.0
    assert r.hit_z == pytest.approx(15.0, abs=1e-5)
    # tilt up -> pivot must sit below the target
    assert r.height_mm is not None
    assert r.height_mm < 15.0


def test_unreachable_reports_limit() -> None:
    rig = neutral_rig()
    far = solve_targets(rig, [Target(cam_id="cam1", z=400.0)])[0]
    assert not far.reachable
    assert "a_max" in far.violations
    steep = solve_targets(rig, [Target(cam_id="cam2", z=0.0, theta_deg=60.0)])[0]
    assert not steep.reachable
    assert "b_max" in steep.violations


def test_normal_rejections() -> None:
    rig = neutral_rig()
    away = solve_targets(rig, [Target(cam_id="cam1", normal=(-1.0, 0.0, 0.0))])[0]
    assert not away.reachable
    assert away.violations == ("normal_direction",)
    yaw = solve_targets(rig, [Target(cam_id="cam1", normal=(0.8, 0.6, 0.0))])[0]
    assert yaw.violations == ("normal_y",)


def test_normal_tilt_angle() -> None:
    rig = neutral_rig()
    # cam1 normal tilted down by 30 deg -> optical axis anti-parallel -> tilt +30 deg (up)
    n = (math.cos(math.radians(30)), 0.0, -math.sin(math.radians(30)))
    r = solve_targets(rig, [Target(cam_id="cam1", z=0.0, normal=n)])[0]
    assert r.tilt_deg == pytest.approx(30.0, abs=1e-9)


def test_unknown_cam() -> None:
    r = solve_targets(neutral_rig(), [Target(cam_id="cam9")])[0]
    assert not r.reachable
    assert r.violations == ("unknown_cam",)
