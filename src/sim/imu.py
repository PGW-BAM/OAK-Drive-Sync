"""Simulated camera IMU (replaces the Windows side in sim mode).

``SimImuResponder`` publishes ``IMUAngle`` per camera at ``rate_hz`` on the telemetry topic
(background readings, ``request_id=None``) and answers ``cmd/cam/{cam}/imu_check`` requests by
echoing the ``request_id`` -- the contract ``DriftDetector.request_imu_check`` relies on
(it resolves its Future on ``IMUAngle.request_id`` and reads ``roll_deg`` / ``pitch_deg``).
Angles come from ``RigModel.imu_angles()``, with optional Gaussian noise and a first-order
settle lag (``settle_tau_s``) that mimics the accelerometer swinging after a move.
"""

from __future__ import annotations

import asyncio
import math
import random
import time
from typing import TYPE_CHECKING, Any

import structlog

from src.shared.models import IMUAngle, IMUCheckRequest
from src.shared.mqtt_topics import SUB_ALL_IMU_CHECKS, TELEMETRY_IMU, topic

if TYPE_CHECKING:
    from src.shared.mqtt_client import MQTTClient
    from src.sim.rig_model import RigModel

logger = structlog.get_logger()


class SimImuResponder:
    """IMU publisher/responder. ``mqtt`` may be an ``MQTTClient`` or ``FakeMQTTClient``.

    Construct (this registers the imu_check handler) BEFORE ``mqtt.run()`` starts, then run
    ``responder.run()`` as a task next to it.
    """

    def __init__(
        self,
        mqtt: MQTTClient | Any,
        rig: RigModel,
        cam_ids: list[str] | None = None,
        *,
        rate_hz: float = 2.0,
        noise_deg: float = 0.0,
        settle_tau_s: float = 0.0,
        seed: int | None = None,
    ) -> None:
        self._mqtt = mqtt
        self._rig = rig
        self.cam_ids = list(cam_ids) if cam_ids is not None else sorted(rig.config.cameras)
        self.rate_hz = rate_hz
        self.noise_deg = noise_deg
        self.settle_tau_s = settle_tau_s
        self._rng = random.Random(seed)
        self._filtered: dict[str, tuple[float, float]] = {
            c: rig.imu_angles(c) for c in self.cam_ids
        }
        self._last_update = time.monotonic()
        mqtt.subscribe(SUB_ALL_IMU_CHECKS, self.on_imu_check)

    # -- readings --------------------------------------------------------------------------------

    def _track(self) -> None:
        """Advance the settle-lag low-pass to 'now' (no-op without lag)."""
        now = time.monotonic()
        dt = now - self._last_update
        self._last_update = now
        if self.settle_tau_s <= 0:
            return
        k = 1.0 - math.exp(-dt / self.settle_tau_s)
        for c in self.cam_ids:
            roll, pitch = self._rig.imu_angles(c)
            fr, fp = self._filtered[c]
            self._filtered[c] = (fr + (roll - fr) * k, fp + (pitch - fp) * k)

    def read(self, cam_id: str) -> tuple[float, float]:
        """Current (roll_deg, pitch_deg) incl. lag and noise."""
        if self.settle_tau_s > 0:
            self._track()
            roll, pitch = self._filtered[cam_id]
        else:
            roll, pitch = self._rig.imu_angles(cam_id)
        if self.noise_deg > 0:
            roll += self._rng.gauss(0.0, self.noise_deg)
            pitch += self._rng.gauss(0.0, self.noise_deg)
        return roll, pitch

    async def _publish_reading(self, cam_id: str, request_id: str | None) -> None:
        roll, pitch = self.read(cam_id)
        msg = IMUAngle(cam_id=cam_id, roll_deg=roll, pitch_deg=pitch, request_id=request_id)
        await self._mqtt.publish(topic(TELEMETRY_IMU, cam_id=cam_id), msg, qos=0)

    # -- MQTT ------------------------------------------------------------------------------------

    async def on_imu_check(self, topic_str: str, payload: dict[str, Any]) -> None:
        """Handle ``cmd/cam/{cam}/imu_check``: answer with the same ``request_id``."""
        req = IMUCheckRequest.model_validate(payload)
        cam_id = topic_str.split("/")[2]
        if cam_id not in self.cam_ids:
            logger.warning("sim_imu.unknown_cam", cam_id=cam_id)
            return
        await self._publish_reading(cam_id, req.request_id)

    async def run(self) -> None:
        """Background telemetry loop (cancel to stop)."""
        period = 1.0 / self.rate_hz
        lag_task = asyncio.create_task(self._lag_loop()) if self.settle_tau_s > 0 else None
        try:
            while True:
                for cam_id in self.cam_ids:
                    await self._publish_reading(cam_id, None)
                await asyncio.sleep(period)
        finally:
            if lag_task is not None:
                lag_task.cancel()

    async def _lag_loop(self) -> None:
        """Sample the rig finely so the low-pass sees the motion, not just 2 Hz snapshots."""
        step = max(0.005, min(0.05, self.settle_tau_s / 5))
        while True:
            self._track()
            await asyncio.sleep(step)


async def run_sim_imu(
    rig: RigModel,
    broker_host: str,
    broker_port: int,
    *,
    rate_hz: float = 2.0,
    noise_deg: float = 0.0,
    settle_tau_s: float = 0.0,
) -> None:
    """Run a SimImuResponder on its own real MQTT connection (used by ``--sim``)."""
    from src.shared.mqtt_client import MQTTClient

    client = MQTTClient(broker_host=broker_host, broker_port=broker_port, client_id="oak-sim-imu")
    responder = SimImuResponder(
        client, rig, rate_hz=rate_hz, noise_deg=noise_deg, settle_tau_s=settle_tau_s
    )
    async with asyncio.TaskGroup() as tg:
        tg.create_task(client.run())
        tg.create_task(responder.run())
