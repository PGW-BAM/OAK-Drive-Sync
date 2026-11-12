"""Tests for the simulated drives, IMU responder, fake broker and the Pi control logic on top."""

from __future__ import annotations

import asyncio
import math
import time
from typing import TYPE_CHECKING, Any

import pytest
import yaml

from src.pi_controller import position_log
from src.pi_controller.drives.base import DriveState
from src.pi_controller.imu.drift_detector import DriftDetector
from src.pi_controller.main import DriveManager, create_drive
from src.shared.models import IMUAngle, IMUCheckRequest, MoveCommand
from src.shared.mqtt_topics import (
    CMD_DRIVE_MOVE,
    CMD_IMU_CHECK,
    STATUS_DRIVE_POSITION,
    SUB_ALL_DRIVE_CMDS,
    SUB_ALL_IMU_TELEMETRY,
    TELEMETRY_IMU,
    topic,
)
from src.sim import RigModel
from src.sim.drives import MotionProfile, SimLinearDrive, SimRadialDrive, set_shared_rig
from src.sim.fake_mqtt import FakeBroker
from src.sim.imu import SimImuResponder
from tests.sim.conftest import neutral_rig

if TYPE_CHECKING:
    from pathlib import Path

FAST = {"update_interval_s": 0.005}


@pytest.fixture()
def rig() -> RigModel:
    return neutral_rig()


def _lin(rig: RigModel, cam: str = "cam1", **kw: Any) -> SimLinearDrive:
    kw = {**FAST, "max_speed": 5000.0, **kw}
    return SimLinearDrive(cam, "a", rig=rig, **kw)


def _rad(rig: RigModel, cam: str = "cam1", **kw: Any) -> SimRadialDrive:
    kw = {**FAST, "max_velocity": 1e6, "acceleration": None, "deceleration": None, **kw}
    return SimRadialDrive(cam, "b", rig=rig, **kw)


# --- motion profile / timing -------------------------------------------------------------------


def test_profile_constant_and_trapezoid() -> None:
    p = MotionProfile(1000, 1000)
    assert p.duration == pytest.approx(1.0)
    assert p.distance(0.5) == pytest.approx(500)
    t = MotionProfile(2000, 1000, accel=1000, decel=1000)  # 1 s ramps (500 each) + 1 s cruise
    assert t.duration == pytest.approx(3.0)
    assert t.distance(t.duration) == pytest.approx(2000)
    assert t.distance(1.0) == pytest.approx(500)
    ts = [i * t.duration / 50 for i in range(51)]
    ds = [t.distance(x) for x in ts]
    assert all(b >= a for a, b in zip(ds, ds[1:], strict=False))
    tri = MotionProfile(100, 1000, accel=1000, decel=1000)  # never reaches max speed
    assert tri.duration == pytest.approx(2 * math.sqrt(100 / 1000) * 1.0, rel=1e-6)
    assert tri.distance(tri.duration / 2) == pytest.approx(50)


async def test_linear_move_timing_and_incremental_position(rig: RigModel) -> None:
    d = _lin(rig, max_speed=2000.0)  # 2 mm = 400 steps -> 0.2 s
    seen: list[float] = []

    async def poll() -> None:
        while d.state != DriveState.REACHED:
            seen.append(d.current_position)
            await asyncio.sleep(0.004)

    t0 = time.monotonic()
    poller = asyncio.create_task(poll())
    await d.move_to(2.0)
    dt = time.monotonic() - t0
    await poller
    assert 0.18 <= dt <= 0.4
    assert d.state == DriveState.REACHED
    assert d.current_position == 2.0
    assert rig.get_steps("cam1", "a") == 400
    mid = sorted({round(p, 3) for p in seen if 0.0 < p < 2.0})
    assert len(mid) >= 5, "position must be updated incrementally while moving"
    assert seen == sorted(seen), "monotonic"


async def test_speed_argument_scales_duration(rig: RigModel) -> None:
    d = _rad(rig, max_velocity=10000.0)
    t0 = time.monotonic()
    await d.move_to(2000, speed=0.5)  # 2000 steps at 5000 steps/s = 0.4 s
    assert 0.36 <= time.monotonic() - t0 <= 0.6


async def test_emergency_stop_mid_move(rig: RigModel) -> None:
    d = _lin(rig, max_speed=5000.0)  # 10 mm = 2000 steps -> 0.4 s
    task = asyncio.create_task(d.move_to(10.0))
    await asyncio.sleep(0.1)
    assert d.state == DriveState.MOVING
    d.emergency_stop()
    await asyncio.wait_for(task, 1.0)
    assert d.state == DriveState.IDLE
    pos = d.current_position
    assert 1.0 < pos < 6.0
    assert rig.get_steps("cam1", "a") == round(pos * 200)
    await asyncio.sleep(0.05)
    assert d.current_position == pos, "must not creep after the stop"
    await d.move_to(1.0)  # drive usable again
    assert d.state == DriveState.REACHED


# --- limit switches, soft limits ---------------------------------------------------------------


async def test_switch_triggers_fault_radial(rig: RigModel) -> None:
    d = _rad(rig)
    d.calibration_mode = True  # bypass soft envelope, like converge()
    await d.move_to(120000)  # 60 deg; switch_max at 53 deg = 106000 steps
    assert d.state == DriveState.FAULT
    assert d.triggered_switch == "max"
    assert rig.get_steps("cam1", "b") == 106000
    assert rig.tilt("cam1") == pytest.approx(53.0)
    assert d.current_position < 120000  # motor was cut at the switch
    # pushing further faults again, moving away clears it
    await d.move_to(120000)
    assert d.state == DriveState.FAULT
    await d.move_to(0)
    assert d.state == DriveState.REACHED
    assert d.triggered_switch is None
    assert rig.get_steps("cam1", "b") == round(0)


async def test_switch_name_is_physical_sense_for_inverted_axis(rig: RigModel) -> None:
    d = _rad(rig, "cam2")  # direction_sign -1: positive steps = negative tilt
    d.calibration_mode = True
    await d.move_to(120000)
    assert d.state == DriveState.FAULT
    assert d.triggered_switch == "min"
    assert rig.tilt("cam2") == pytest.approx(-53.0)


async def test_switch_triggers_fault_linear(rig: RigModel) -> None:
    d = _lin(rig, max_speed=1e6)
    d.calibration_mode = True
    await d.move_to(160.0)  # switch_max = 150 + 3 mm
    assert d.state == DriveState.FAULT and d.triggered_switch == "max"
    assert rig.height("cam1") == pytest.approx(153.0)
    await d.move_to(-160.0)  # lower bound -145 is hard (also in calibration mode)
    assert d.state == DriveState.REACHED and d.current_position == -145.0
    assert d.triggered_switch is None


async def test_linear_soft_limits_like_gpio_drive(rig: RigModel) -> None:
    d = _lin(rig, max_speed=1e6)
    assert (d.get_min_position(), d.get_max_position()) == (-145.0, 145.0)
    await d.move_to(200.0)  # upper bound clamped outside calibration mode
    assert d.current_position == 145.0 and d.state == DriveState.REACHED
    await d.move_to(-200.0)  # lower bound always clamped
    assert d.current_position == -145.0
    d.calibration_mode = True
    await d.move_to(150.0)  # upper bound open in calibration mode (switch is at 153)
    assert d.current_position == 150.0 and d.state == DriveState.REACHED
    d.set_max_position(10.0)
    d.calibration_mode = False
    await d.move_to(50.0)
    assert d.current_position == 10.0


async def test_radial_soft_limits_like_tinkerforge_drive(rig: RigModel) -> None:
    d = _rad(rig, min_position=-1000, max_position=5000)
    await d.move_to(9000)
    assert d.current_position == 5000.0
    await d.move_to(-9000)
    assert d.current_position == -1000.0  # unlike GPIO the lower bound is soft too
    d.calibration_mode = True
    await d.move_to(7000)
    assert d.current_position == 7000.0


# --- rig coupling, counter vs physical, error injection ----------------------------------------


async def test_rig_pose_follows_drives(rig: RigModel) -> None:
    a = _lin(rig, max_speed=1e6)
    b = _rad(rig)
    await a.move_to(50.0)
    await b.move_to(40000)  # 20 deg
    assert rig.height("cam1") == pytest.approx(50.0)
    assert rig.tilt("cam1") == pytest.approx(20.0)
    assert rig.imu_angles("cam1")[0] == pytest.approx(20.0)
    assert rig.pose("cam1").pivot[2] == pytest.approx(50.0)
    # cam2 untouched
    assert rig.tilt("cam2") == 0.0


async def test_set_current_position_moves_counter_not_rig(rig: RigModel) -> None:
    b = _rad(rig)
    await b.move_to(4000)
    b.set_current_position(500)
    assert b.current_position == 500.0
    assert rig.get_steps("cam1", "b") == 4000
    await b.move_to(1500)  # +1000 steps relative to the new counter
    assert rig.get_steps("cam1", "b") == 5000


async def test_step_loss_and_backlash_injection(rig: RigModel) -> None:
    b = _rad(rig, step_loss_fraction=0.1)
    await b.move_to(1000)
    assert b.current_position == 1000.0  # open-loop counter does not know
    assert rig.get_steps("cam1", "b") == 900

    rig2 = neutral_rig()
    b2 = _rad(rig2, backlash_steps=100)
    await b2.move_to(1000)  # first move: no slack yet
    assert rig2.get_steps("cam1", "b") == 1000
    await b2.move_to(500)  # reversal eats 100 steps of play
    assert rig2.get_steps("cam1", "b") == 600
    assert b2.current_position == 500.0


def test_create_drive_factory_sim_types(rig: RigModel) -> None:
    set_shared_rig(rig)
    try:
        a = create_drive("cam1", "a", {"type": "sim_linear", "step_pin": 14, "dir_pin": 15})
        b = create_drive("cam2", "b", {"type": "sim_radial", "uid": "x", "host": "h"})
    finally:
        set_shared_rig(None)
    assert isinstance(a, SimLinearDrive) and not a.supports_angle_feedback
    assert isinstance(b, SimRadialDrive) and b.supports_angle_feedback
    assert b.get_motor_current_mA() == 1200


# --- fake broker ---------------------------------------------------------------------------------


async def _eventually(cond: Any, timeout: float = 3.0) -> None:
    end = time.monotonic() + timeout
    while not cond():
        if time.monotonic() > end:
            raise AssertionError("condition not met in time")
        await asyncio.sleep(0.005)


async def test_fake_broker_wildcards_and_retained() -> None:
    broker = FakeBroker()
    pub = broker.client("pub")
    sub = broker.client("sub")
    got: list[tuple[str, dict]] = []

    async def h(t: str, p: dict) -> None:
        got.append((t, p))

    sub.subscribe("status/drives/+/position", h)
    tasks = [asyncio.create_task(pub.run())]
    await pub._connected.wait()
    await pub.publish("status/drives/cam1/position", {"x": 1}, retain=True)
    tasks.append(asyncio.create_task(sub.run()))  # late joiner gets the retained message
    await _eventually(lambda: len(got) == 1)
    await pub.publish("status/drives/cam2/position", {"x": 2})
    await pub.publish("other/topic", {"x": 3})
    await _eventually(lambda: len(got) == 2)
    assert [t for t, _ in got] == ["status/drives/cam1/position", "status/drives/cam2/position"]
    await sub.stop()
    await pub.stop()
    await asyncio.gather(*tasks)


async def test_fake_broker_serial_dispatch_blocks_waiting_handler() -> None:
    """Documents the real MQTTClient behaviour: reply is only dispatched AFTER the handler."""
    broker = FakeBroker()
    c = broker.client("c", dispatch="serial")
    other = broker.client("o")
    order: list[str] = []
    gate = asyncio.Event()

    async def on_req(_t: str, _p: dict) -> None:
        await other.publish("reply", {})
        try:
            await asyncio.wait_for(gate.wait(), 0.2)  # would be set by on_reply
            order.append("reply_seen_inside_handler")
        except TimeoutError:
            order.append("handler_timed_out")

    async def on_reply(_t: str, _p: dict) -> None:
        order.append("reply_dispatched")
        gate.set()

    c.subscribe("req", on_req)
    c.subscribe("reply", on_reply)
    tasks = [asyncio.create_task(c.run()), asyncio.create_task(other.run())]
    await other._connected.wait()
    await other.publish("req", {})
    await _eventually(lambda: "reply_dispatched" in order)
    for t in tasks:
        t.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)
    assert order == ["handler_timed_out", "reply_dispatched"]


async def test_fake_broker_concurrent_dispatch_delivers_during_handler() -> None:
    broker = FakeBroker()
    c = broker.client("c", dispatch="concurrent")
    other = broker.client("o")
    order: list[str] = []
    gate = asyncio.Event()

    async def on_req(_t: str, _p: dict) -> None:
        await other.publish("reply", {})
        await asyncio.wait_for(gate.wait(), 0.5)
        order.append("reply_seen_inside_handler")

    async def on_reply(_t: str, _p: dict) -> None:
        order.append("reply_dispatched")
        gate.set()

    c.subscribe("req", on_req)
    c.subscribe("reply", on_reply)
    tasks = [asyncio.create_task(c.run()), asyncio.create_task(other.run())]
    await other._connected.wait()
    await other.publish("req", {})
    await _eventually(lambda: "reply_seen_inside_handler" in order)
    for t in tasks:
        t.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)
    assert order == ["reply_dispatched", "reply_seen_inside_handler"]


# --- IMU responder -------------------------------------------------------------------------------


async def test_imu_responder_echoes_request_id_and_tracks_tilt(rig: RigModel) -> None:
    broker = FakeBroker()
    cam, pi = broker.client("cam"), broker.client("pi")
    SimImuResponder(cam, rig, ["cam1", "cam2"], rate_hz=1000.0, noise_deg=0.0)
    got: list[tuple[str, IMUAngle]] = []

    async def on_imu(t: str, p: dict) -> None:
        got.append((t, IMUAngle.model_validate(p)))

    pi.subscribe(SUB_ALL_IMU_TELEMETRY, on_imu)  # no 'run' of responder: only request path
    tasks = [asyncio.create_task(cam.run()), asyncio.create_task(pi.run())]
    await pi._connected.wait()
    rig.set_pose("cam1", theta_deg=12.5)
    req = IMUCheckRequest(request_id="req-123", cam_id="cam1", checkpoint_name="t")
    await pi.publish(topic(CMD_IMU_CHECK, cam_id="cam1"), req)
    await _eventually(lambda: len(got) == 1)
    t, ang = got[0]
    assert t == topic(TELEMETRY_IMU, cam_id="cam1")
    assert ang.request_id == "req-123" and ang.cam_id == "cam1"
    assert ang.roll_deg == pytest.approx(12.5, abs=1e-6)
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


async def test_imu_responder_background_noise_and_lag(rig: RigModel) -> None:
    broker = FakeBroker()
    cam, pi = broker.client("cam"), broker.client("pi")
    resp = SimImuResponder(
        cam, rig, ["cam1"], rate_hz=50.0, noise_deg=0.5, settle_tau_s=0.05, seed=1
    )
    got: list[IMUAngle] = []

    async def on_imu(_t: str, p: dict) -> None:
        got.append(IMUAngle.model_validate(p))

    pi.subscribe(SUB_ALL_IMU_TELEMETRY, on_imu)
    tasks = [
        asyncio.create_task(cam.run()),
        asyncio.create_task(pi.run()),
        asyncio.create_task(resp.run()),
    ]
    await _eventually(lambda: len(got) >= 3)
    assert all(g.request_id is None for g in got)
    rig.set_pose("cam1", theta_deg=30.0)
    await asyncio.sleep(0.02)
    early = got[-1].roll_deg
    assert early < 25.0, "settle lag: reading has not caught up yet"
    await asyncio.sleep(0.6)
    late = [g.roll_deg for g in got[-8:]]
    assert all(abs(x - 30.0) < 2.5 for x in late)  # within ~5 sigma
    assert len({round(x, 6) for x in late}) > 1, "noise is applied"
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


# --- end-to-end: DriveManager + sim drives + SimImuResponder over the fake broker ----------------

CALIBRATION = {
    "drift_threshold_deg": 2.0,
    "checkpoints": {"cam1": {"b": []}},
    "steps_per_degree": {"cam1_b": 2000.0},
    "convergence": {
        "max_iterations": 30,
        "max_step_per_iter": 8000,
        "motion_threshold_deg": 0.2,
        "threshold_deg": 0.1,
        "settle_speed": 1.0,
        "seed_speed": 1.0,
        "settle_ms": 0,
        "steps_per_degree_sign": {"cam1_b": 1},
    },
    "angle_history": {"cam1_b": {}},
}


async def _run_e2e(dispatch: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # DriveManager/DriftDetector write to relative paths; keep them out of the repo
    monkeypatch.setattr(position_log, "LOG_PATH", tmp_path / "drive_positions.jsonl")
    cal_path = tmp_path / "imu_calibration.yaml"
    cal_path.write_text(yaml.safe_dump(CALIBRATION))

    rig = neutral_rig()
    set_shared_rig(rig)
    broker = FakeBroker()
    pi = broker.client("pi", dispatch=dispatch)  # type: ignore[arg-type]
    cam = broker.client("cam")  # camera side (Windows) -> SimImuResponder
    win = broker.client("win")  # orchestrator side: sends commands, listens to status

    config = {
        "settling_delay_ms": 1,
        "drives": {
            "cam1": {
                "axis_a": {"type": "sim_linear", "max_speed": 1e6, **FAST},
                "axis_b": {
                    "type": "sim_radial",
                    "max_velocity": 1e6,
                    "acceleration": None,
                    "deceleration": None,
                    **FAST,
                },
            }
        },
    }
    mgr = DriveManager(config, pi)
    drift = DriftDetector(pi, cal_path)
    mgr.drift_detector = drift
    orig_check = drift.request_imu_check

    def quick_check(cam_id: str, name: str, timeout: float = 0.4) -> Any:
        return orig_check(cam_id, name, timeout=timeout)  # keep the xfail variant short

    drift.request_imu_check = quick_check  # type: ignore[method-assign]
    await mgr.setup_all()

    # Same wiring as main._setup_controller (which hard-codes a real MQTTClient)
    pi.subscribe(SUB_ALL_IMU_TELEMETRY, drift.on_imu_message)

    async def dispatch_command(topic_str: str, payload: dict) -> None:
        if topic_str.endswith("/move"):
            await mgr.handle_move(topic_str, payload)

    pi.subscribe(SUB_ALL_DRIVE_CMDS, dispatch_command)
    SimImuResponder(cam, rig, ["cam1", "cam2"], rate_hz=20.0, noise_deg=0.0)

    statuses: list[dict] = []

    async def on_status(_t: str, p: dict) -> None:
        statuses.append(p)

    win.subscribe("status/drives/+/position", on_status)

    tasks = [asyncio.create_task(c.run()) for c in (pi, cam, win)]
    try:
        for c in (pi, cam, win):
            await asyncio.wait_for(c._connected.wait(), 1.0)
        cmd = MoveCommand(
            sequence_id="e2e-1",
            drive_axis="b",
            target_position=0.0,
            target_angle_deg=10.0,
            active_angle="roll",
        )
        await win.publish(topic(CMD_DRIVE_MOVE, cam_id="cam1"), cmd)
        await _eventually(
            lambda: any(
                s["sequence_id"] == "e2e-1" and s["state"] in ("reached", "fault") for s in statuses
            ),
            timeout=5.0,
        )
        mine = [s for s in statuses if s["sequence_id"] == "e2e-1"]
        # First status for a new command must say MOVING, never a stale REACHED.
        assert mine[0]["state"] == "moving"
        final = mine[-1]
        assert final["state"] == "reached"
        assert rig.imu_angles("cam1")[0] == pytest.approx(10.0, abs=0.25)
        assert mgr.drives["cam1:b"].state == DriveState.REACHED
        assert next(broker.messages(topic(STATUS_DRIVE_POSITION, cam_id="cam1")))
    finally:
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        set_shared_rig(None)


@pytest.mark.xfail(
    strict=True,
    reason="OLD MQTTClient behaviour (fixed by TopicDispatcher): handlers awaited inline -> "
    "imu_check reply cannot be dispatched during converge",
)
async def test_e2e_angle_move_serial_dispatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    await _run_e2e("serial", tmp_path, monkeypatch)


async def test_e2e_angle_move_per_topic_dispatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The fixed MQTTClient dispatch (TopicDispatcher): converge gets its IMU replies."""
    await _run_e2e("per_topic", tmp_path, monkeypatch)


async def test_per_topic_dispatch_stop_overtakes_running_move() -> None:
    """/stop must be handled while a /move handler is still running; moves stay ordered."""
    broker = FakeBroker()
    c = broker.client("pi")  # default = per_topic, same dispatcher as MQTTClient
    log: list[str] = []
    stop_seen = asyncio.Event()

    async def on_move(topic: str, payload: dict) -> None:
        log.append(f"move{payload['n']}_start")
        if payload["n"] == 1:
            await asyncio.wait_for(stop_seen.wait(), timeout=1.0)
        log.append(f"move{payload['n']}_end")

    async def on_stop(topic: str, payload: dict) -> None:
        log.append("stop")
        stop_seen.set()

    c.subscribe("cmd/drives/+/move", on_move)
    c.subscribe("cmd/drives/+/stop", on_stop)
    task = asyncio.create_task(c.run())
    await c._connected.wait()
    await c.publish("cmd/drives/cam1/move", {"n": 1})
    await c.publish("cmd/drives/cam1/move", {"n": 2})
    await c.publish("cmd/drives/cam1/stop", {})
    await _eventually(lambda: "move2_end" in log)
    await c.stop()
    await task
    assert log == ["move1_start", "stop", "move1_end", "move2_start", "move2_end"]


async def test_e2e_angle_move_concurrent_dispatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    await _run_e2e("concurrent", tmp_path, monkeypatch)


def test_sim_mode_never_writes_real_state_files(tmp_path, monkeypatch):
    """--sim must redirect calibration / position log / presets away from config/."""
    import src.pi_controller.main as pi_main
    from src.pi_controller import position_log

    monkeypatch.setattr(position_log, "LOG_PATH", position_log.LOG_PATH)
    sim_cfg = {"state_dir": str(tmp_path / "sim")}
    pi_main._isolate_sim_state(sim_cfg)
    assert position_log.LOG_PATH.parent == tmp_path / "sim"
    imu_path = pi_main._imu_calibration_path({"sim": sim_cfg})
    assert imu_path.parent == tmp_path / "sim"
    assert imu_path != pi_main.REAL_IMU_CALIBRATION
    assert imu_path.exists()  # seeded from the real file as template
    assert pi_main._imu_calibration_path({}) == pi_main.REAL_IMU_CALIBRATION


async def test_calibration_mode_respects_hard_envelope() -> None:
    """converge lifts soft limits but a hard envelope must still clamp runaway targets."""
    from src.sim.drives import SimRadialDrive

    rig = neutral_rig()
    drive = SimRadialDrive("cam1", "b", rig=rig, max_velocity=10_000_000, acceleration=0)
    await drive.setup()
    drive.calibration_mode = True
    drive.hard_envelope = (-500.0, 500.0)
    await drive.move_to(1_000_000)
    assert drive.current_position <= 500.0
    drive.hard_envelope = None
