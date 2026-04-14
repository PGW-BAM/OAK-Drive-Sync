"""
IMU-based drift detection for Tinkerforge radial drives (axis_b).

When a drive arrives at a named checkpoint, DriftDetector requests a fresh
IMU angle reading from the Windows controller via MQTT, compares it against
the calibrated expected angle, and applies a corrective step move if the
drift exceeds the configured threshold.
"""

from __future__ import annotations

import asyncio
import os
import uuid
from pathlib import Path
from typing import TYPE_CHECKING

import structlog
import yaml

from src.shared.models import DriftDetectionEvent, IMUAngle, IMUCheckRequest
from src.shared.mqtt_topics import CMD_IMU_CHECK, EVENT_DRIFT, topic

if TYPE_CHECKING:
    from src.pi_controller.drives.tinkerforge_drive import TinkerforgeDrive
    from src.shared.mqtt_client import MQTTClient

logger = structlog.get_logger()


class DriftDetector:
    """
    Detects and auto-corrects positional drift in Tinkerforge radial drives
    using camera IMU angle data published over MQTT by the Windows controller.

    Construction is synchronous (safe to call before the asyncio event loop
    starts). No asyncio objects are created in __init__ — Futures are created
    on demand inside request_imu_check().
    """

    def __init__(self, mqtt_client: MQTTClient, calibration_path: Path | str) -> None:
        self._mqtt = mqtt_client
        self._calibration_path = Path(calibration_path)
        self._calibration: dict = self._load_calibration()
        self._latest_imu: dict[str, IMUAngle] = {}   # cam_id → most recent IMUAngle
        self._pending: dict[str, asyncio.Future] = {} # request_id → Future[IMUAngle]

    # ──────────────────────────────────────────────
    # MQTT message handler (registered as subscriber)
    # ──────────────────────────────────────────────

    async def on_imu_message(self, topic_str: str, payload: dict) -> None:
        """Handle incoming telemetry/cam/{cam_id}/imu messages."""
        cam_id = topic_str.split("/")[2]
        angle = IMUAngle.model_validate(payload)
        self._latest_imu[cam_id] = angle

        # Resolve a pending request/response Future if this message is a correlated response.
        # Background 2Hz publishes have request_id=None and must NOT resolve Futures.
        if angle.request_id and angle.request_id in self._pending:
            fut = self._pending.pop(angle.request_id)
            if not fut.done():
                fut.set_result(angle)

    # ──────────────────────────────────────────────
    # Request/response IMU check
    # ──────────────────────────────────────────────

    async def request_imu_check(
        self,
        cam_id: str,
        checkpoint_name: str,
        timeout: float = 3.0,
    ) -> IMUAngle:
        """
        Publish an IMUCheckRequest and await the correlated IMUAngle response.

        Raises TimeoutError if Windows does not respond within `timeout` seconds.
        """
        request_id = str(uuid.uuid4())
        loop = asyncio.get_running_loop()
        fut: asyncio.Future[IMUAngle] = loop.create_future()
        self._pending[request_id] = fut

        req = IMUCheckRequest(
            request_id=request_id,
            cam_id=cam_id,
            checkpoint_name=checkpoint_name,
        )
        await self._mqtt.publish(
            topic(CMD_IMU_CHECK, cam_id=cam_id),
            req,
            qos=1,
        )

        try:
            # asyncio.shield keeps the Future alive if the outer wait_for is cancelled
            return await asyncio.wait_for(asyncio.shield(fut), timeout=timeout)
        except asyncio.TimeoutError:
            self._pending.pop(request_id, None)
            raise TimeoutError(
                f"IMU check request timed out for {cam_id}/{checkpoint_name} "
                f"(waited {timeout}s)"
            )

    # ──────────────────────────────────────────────
    # Drift check + auto-correction
    # ──────────────────────────────────────────────

    async def check_and_correct(
        self,
        cam_id: str,
        drive: TinkerforgeDrive,
        checkpoint_name: str,
    ) -> DriftDetectionEvent | None:
        """
        Request an IMU reading, compare against calibrated angle, and apply
        a corrective move if drift exceeds the threshold.

        Returns a DriftDetectionEvent if drift was detected, None otherwise.
        The drive sequence continues regardless of the outcome.
        """
        checkpoint = self._find_checkpoint(cam_id, checkpoint_name)
        if checkpoint is None:
            logger.debug(
                "drift_detector.no_checkpoint",
                cam_id=cam_id,
                name=checkpoint_name,
            )
            return None

        spd_key = f"{cam_id}_b"
        steps_per_deg: float = (
            self._calibration.get("steps_per_degree", {}).get(spd_key, 0.0)
        )
        if steps_per_deg == 0.0:
            logger.error(
                "drift_detector.no_steps_per_degree",
                cam_id=cam_id,
                hint="Set steps_per_degree in config/imu_calibration.yaml or via the GUI",
            )
            return None

        # Request fresh IMU reading
        try:
            imu = await self.request_imu_check(cam_id, checkpoint_name)
        except TimeoutError as exc:
            logger.error(
                "drift_detector.imu_timeout",
                cam_id=cam_id,
                checkpoint=checkpoint_name,
                error=str(exc),
            )
            return None

        # Compare angles
        active: str = checkpoint["active_angle"]  # "roll" or "pitch"
        actual: float = imu.roll_deg if active == "roll" else imu.pitch_deg
        expected: float = checkpoint[f"expected_{active}_deg"]
        drift: float = actual - expected

        threshold: float = self._calibration.get("drift_threshold_deg", 2.0)
        if abs(drift) <= threshold:
            logger.debug(
                "drift_detector.within_tolerance",
                cam_id=cam_id,
                checkpoint=checkpoint_name,
                drift_deg=round(drift, 3),
                threshold=threshold,
            )
            return None

        # Clamp correction to 10° equivalent to guard against IMU noise spikes
        max_corr_steps = int(10.0 * steps_per_deg)
        raw_correction = int(-drift * steps_per_deg)
        correction_steps = max(-max_corr_steps, min(max_corr_steps, raw_correction))

        if abs(raw_correction) > max_corr_steps:
            logger.critical(
                "drift_detector.correction_clamped",
                cam_id=cam_id,
                checkpoint=checkpoint_name,
                raw_correction_steps=raw_correction,
                clamped_to=correction_steps,
                drift_deg=round(drift, 3),
                msg="Possible IMU spike — correction was clamped. Verify calibration.",
            )

        # Apply correction
        corrected = False
        new_position = drive.current_position + correction_steps
        try:
            await drive.move_to(new_position, speed=0.5)
            corrected = True
            logger.info(
                "drift_detector.correction_applied",
                cam_id=cam_id,
                checkpoint=checkpoint_name,
                drift_deg=round(drift, 3),
                correction_steps=correction_steps,
                new_position=new_position,
            )
        except Exception:
            logger.exception(
                "drift_detector.correction_failed",
                cam_id=cam_id,
                checkpoint=checkpoint_name,
            )

        event = DriftDetectionEvent(
            request_id=imu.request_id or "",
            cam_id=cam_id,
            drive_axis="b",
            checkpoint_name=checkpoint_name,
            expected_angle_deg=expected,
            actual_angle_deg=actual,
            drift_deg=drift,
            correction_steps=correction_steps,
            corrected=corrected,
        )

        await self._mqtt.publish(
            topic(EVENT_DRIFT, cam_id=cam_id, axis="b"),
            event,
            qos=1,
        )

        logger.warning(
            "drift_detector.drift_event_published",
            cam_id=cam_id,
            checkpoint=checkpoint_name,
            drift_deg=round(drift, 3),
            correction_steps=correction_steps,
            corrected=corrected,
        )
        return event

    # ──────────────────────────────────────────────
    # Calibration management
    # ──────────────────────────────────────────────

    def calibrate_checkpoint(
        self,
        cam_id: str,
        name: str,
        drive_position: float,
        imu_angle: IMUAngle,
        active_angle: str = "roll",
    ) -> None:
        """
        Store a named checkpoint with the current drive position and IMU angle.
        Replaces any existing checkpoint with the same name. Persists to YAML.
        """
        entries: list = (
            self._calibration
            .setdefault("checkpoints", {})
            .setdefault(cam_id, {})
            .setdefault("b", [])
        )
        entry = {
            "name": name,
            "drive_position": int(drive_position),
            "expected_roll_deg": round(imu_angle.roll_deg, 3),
            "expected_pitch_deg": round(imu_angle.pitch_deg, 3),
            "active_angle": active_angle,
        }
        for i, cp in enumerate(entries):
            if cp["name"] == name:
                entries[i] = entry
                break
        else:
            entries.append(entry)

        self._save_calibration()
        logger.info(
            "drift_detector.checkpoint_saved",
            cam_id=cam_id,
            name=name,
            drive_position=int(drive_position),
            active_angle=active_angle,
        )

    def delete_checkpoint(self, cam_id: str, name: str) -> bool:
        """Remove a checkpoint by name. Returns True if found and deleted."""
        entries: list = (
            self._calibration
            .get("checkpoints", {})
            .get(cam_id, {})
            .get("b", [])
        )
        new_entries = [cp for cp in entries if cp["name"] != name]
        if len(new_entries) == len(entries):
            return False
        self._calibration["checkpoints"][cam_id]["b"] = new_entries
        self._save_calibration()
        logger.info("drift_detector.checkpoint_deleted", cam_id=cam_id, name=name)
        return True

    def get_checkpoints(self, cam_id: str) -> list[dict]:
        """Return the list of calibrated checkpoints for the given camera's axis_b."""
        return (
            self._calibration
            .get("checkpoints", {})
            .get(cam_id, {})
            .get("b", [])
        )

    def get_latest_imu(self, cam_id: str) -> IMUAngle | None:
        """Return the most recently received IMU angle for the given camera."""
        return self._latest_imu.get(cam_id)

    def get_drift_threshold(self) -> float:
        return self._calibration.get("drift_threshold_deg", 2.0)

    def set_drift_threshold(self, degrees: float) -> None:
        self._calibration["drift_threshold_deg"] = degrees
        self._save_calibration()

    def get_steps_per_degree(self, cam_id: str) -> float:
        return self._calibration.get("steps_per_degree", {}).get(f"{cam_id}_b", 0.0)

    def set_steps_per_degree(self, cam_id: str, value: float) -> None:
        self._calibration.setdefault("steps_per_degree", {})[f"{cam_id}_b"] = value
        self._save_calibration()

    # ──────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────

    def _find_checkpoint(self, cam_id: str, name: str) -> dict | None:
        return next(
            (cp for cp in self.get_checkpoints(cam_id) if cp["name"] == name),
            None,
        )

    def _load_calibration(self) -> dict:
        if self._calibration_path.exists():
            with open(self._calibration_path) as f:
                return yaml.safe_load(f) or {}
        return {
            "drift_threshold_deg": 2.0,
            "checkpoints": {"cam1": {"b": []}, "cam2": {"b": []}},
            "steps_per_degree": {"cam1_b": 120.0, "cam2_b": 120.0},
        }

    def _save_calibration(self) -> None:
        """Atomic write: write to .tmp then rename to prevent corruption on crash."""
        tmp = self._calibration_path.with_suffix(".yaml.tmp")
        with open(tmp, "w") as f:
            yaml.dump(self._calibration, f, default_flow_style=False, allow_unicode=True)
        os.replace(tmp, self._calibration_path)
