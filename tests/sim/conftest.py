"""Shared helpers: tests use a rig with neutral IMU mounting (roll == tilt).

config/sim_rig.yaml carries the real mounting offsets (cam1 ~ -90 deg, cam2 ~ +90 deg),
which tests must not depend on.
"""

from __future__ import annotations

from src.sim import RigConfig, RigModel, load_rig_config


def neutral_rig_config() -> RigConfig:
    cfg = load_rig_config()
    for cam in cfg.cameras.values():
        cam.imu.roll_sign = 1
        cam.imu.roll_offset_deg = 0.0
    return cfg


def neutral_rig() -> RigModel:
    return RigModel(neutral_rig_config())
