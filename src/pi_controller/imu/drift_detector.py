"""
IMU-based drift detection for Tinkerforge radial drives (axis_b).

When a drive arrives at a named checkpoint, DriftDetector requests a fresh
IMU angle reading from the Windows controller via MQTT, compares it against
the calibrated expected angle, and applies a corrective step move if the
drift exceeds the configured threshold.
"""

from __future__ import annotations

import asyncio
import math
import os
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import structlog
import yaml

from src.pi_controller.imu.angle_history import AngleHistory
from src.pi_controller.imu.newton_search import newton_angle_search
from src.shared.models import DriftDetectionEvent, IMUAngle, IMUCheckRequest
from src.shared.mqtt_topics import CMD_IMU_CHECK, EVENT_DRIFT, topic


@dataclass
class ConvergeResult:
    converged: bool
    iterations: int
    final_error_deg: float
    resynced_to: float | None
    event: DriftDetectionEvent | None  # terminal event that was published

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
        # Per-cam (target_angle → motor_position) cache so repeat visits to
        # the same angle only need small-step corrections.
        self.angle_history = AngleHistory(
            self._calibration, self._save_calibration
        )

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
    # Closed-loop angle convergence (Windows supplies target_angle_deg)
    # ──────────────────────────────────────────────

    async def converge(
        self,
        cam_id: str,
        drive: TinkerforgeDrive,
        target_angle_deg: float,
        active_angle: Literal["roll", "pitch"],
        *,
        checkpoint_name: str = "",
        resync_position: float | None = None,
    ) -> ConvergeResult:
        """Seed-then-Newton closed-loop angle converge.

        Delegates the inner search to `newton_angle_search`, the same helper
        used by the startup auto-calibrator's Phase 2 — adaptive est_spd EMA,
        small ~4° clamps, crossover detection, and stuck/wrong-direction
        recovery. Before the Newton loop we seed the drive open-loop to a
        best-guess position (history hit → stored; else reference-anchor
        formula). On a successful converge we record the final position so
        the next visit to this angle gets a close-to-correct seed.

        `resync_position` behavior is unchanged: overwrites the drive's
        internal counter after a successful converge.

        One DriftDetectionEvent is published on the terminal iteration.
        """
        conv = self._calibration.get("convergence", {}) or {}
        max_iterations: int = int(conv.get("max_iterations", 30))
        max_step_per_iter: int = int(conv.get("max_step_per_iter", 400))
        motion_threshold_deg: float = float(conv.get("motion_threshold_deg", 0.2))
        threshold_deg: float = float(conv.get("threshold_deg", 0.3))
        settle_speed: float = float(conv.get("settle_speed", 0.3))
        seed_speed: float = float(conv.get("seed_speed", 0.8))
        settle_ms: int = int(conv.get("settle_ms", 400))
        manual_stuck_tolerance: int = int(conv.get("manual_stuck_tolerance", 4))
        manual_regression_tolerance: int = int(
            conv.get("manual_regression_tolerance", 3)
        )
        sign_map: dict = conv.get("steps_per_degree_sign", {}) or {}
        direction_sign: int = int(sign_map.get(f"{cam_id}_b", 1))

        steps_per_deg = self.get_steps_per_degree(cam_id)
        if steps_per_deg == 0.0:
            logger.error(
                "drift_detector.converge_no_steps_per_degree",
                cam_id=cam_id,
                hint="Set steps_per_degree in config/imu_calibration.yaml",
            )
            return ConvergeResult(
                converged=False,
                iterations=0,
                final_error_deg=float("nan"),
                resynced_to=None,
                event=None,
            )

        reference_position = self.get_reference_position(cam_id, axis="b")
        reference_angle_deg = self.get_reference_angle_deg(cam_id, axis="b")

        # Bypass the soft envelope — see original comment. Also lets the
        # seed pre-move land wherever the last visit to this angle parked.
        was_cal_mode = drive.calibration_mode
        drive.calibration_mode = True
        try:
            # ── Seed phase ──────────────────────────────────────────────
            # Open-loop pre-move to a best-guess position before the Newton
            # loop. Saves many iterations and, more importantly, keeps the
            # first Newton step small — no more "goes totally wrong way"
            # when the current position is 40° off target.
            seed = self.angle_history.seed_for(
                cam_id=cam_id,
                target_angle_deg=target_angle_deg,
                reference_position=(
                    int(reference_position) if reference_position is not None else None
                ),
                reference_angle_deg=reference_angle_deg,
                steps_per_degree=steps_per_deg,
                direction_sign=direction_sign,
            )
            if seed is not None:
                pre_pos = drive.current_position
                if abs(seed - pre_pos) >= max(1, int(steps_per_deg * motion_threshold_deg)):
                    try:
                        await drive.move_to(seed, speed=seed_speed)
                        if settle_ms > 0:
                            await asyncio.sleep(settle_ms / 1000.0)
                        logger.info(
                            "drift_detector.converge_seed_move",
                            cam_id=cam_id,
                            target_angle_deg=target_angle_deg,
                            pre_pos=pre_pos,
                            seed_pos=seed,
                            travel=seed - pre_pos,
                        )
                    except Exception:
                        logger.exception(
                            "drift_detector.converge_seed_move_failed",
                            cam_id=cam_id,
                            seed_pos=seed,
                        )
            start_pos = drive.current_position

            # ── Newton fine-tune ────────────────────────────────────────
            async def _read(label: str) -> float | None:
                try:
                    imu = await self.request_imu_check(
                        cam_id,
                        checkpoint_name or f"converge:{active_angle}",
                    )
                except TimeoutError as exc:
                    logger.error(
                        "drift_detector.converge_imu_timeout",
                        cam_id=cam_id,
                        label=label,
                        error=str(exc),
                    )
                    return None
                val = imu.roll_deg if active_angle == "roll" else imu.pitch_deg
                nonlocal last_request_id, last_actual
                last_request_id = imu.request_id or ""
                last_actual = val
                return val

            last_request_id = ""
            last_actual = 0.0

            def _on_sign_change(new_sign: int) -> None:
                self.set_target_sign(cam_id, new_sign, axis="b")

            newton = await newton_angle_search(
                cam_id=cam_id,
                drive=drive,
                read_angle=_read,
                target_angle_deg=target_angle_deg,
                threshold_deg=threshold_deg,
                motion_threshold_deg=motion_threshold_deg,
                max_iterations=max_iterations,
                max_step_per_iter=max_step_per_iter,
                est_spd_init=steps_per_deg,
                direction_sign=direction_sign,
                settle_speed=settle_speed,
                settle_ms=settle_ms,
                safety_travel=None,   # manual moves may span full envelope
                start_pos=int(start_pos),
                log_prefix="drift_detector.converge",
                on_direction_sign_change=_on_sign_change,
                # Direction is known from auto-cal; a stuck iteration means
                # we hit a physical end-stop, so abort instead of flipping
                # and grinding the wrong way. Require several consecutive
                # stuck/regression iterations before aborting so IMU
                # settling noise and sub-noise corrections near target
                # don't bail out after one bad sample.
                allow_direction_flip=False,
                stuck_tolerance=manual_stuck_tolerance,
                regression_tolerance=manual_regression_tolerance,
            )

            # Translate NewtonResult → ConvergeResult + DriftDetectionEvent.
            err = float(newton.final_err) if newton.final_err is not None else float("nan")
            total_correction_steps = int(drive.current_position - start_pos)

            if newton.converged:
                resynced: float | None = None
                if resync_position is not None:
                    try:
                        drive.set_current_position(resync_position)
                        resynced = resync_position
                    except Exception:
                        logger.exception(
                            "drift_detector.converge_resync_failed",
                            cam_id=cam_id,
                            resync_position=resync_position,
                        )
                # Record pre-resync motor position under the target-angle
                # bucket. If resync ran, the recorded value is the pre-
                # resync position (before the counter was overwritten); the
                # seed formula uses absolute counter values so recording
                # post-resync would poison the cache on next boot.
                try:
                    self.angle_history.record(
                        cam_id=cam_id,
                        target_angle_deg=target_angle_deg,
                        position=int(newton.final_position),
                    )
                except Exception:
                    logger.exception(
                        "drift_detector.converge_history_record_failed",
                        cam_id=cam_id,
                    )
                event = await self._publish_converge_event(
                    cam_id=cam_id,
                    checkpoint_name=checkpoint_name,
                    request_id=last_request_id,
                    expected=target_angle_deg,
                    actual=last_actual,
                    drift=err,
                    correction_steps=total_correction_steps,
                    corrected=True,
                    iterations=newton.iterations,
                    resynced_to=resynced,
                )
                return ConvergeResult(
                    converged=True,
                    iterations=newton.iterations,
                    final_error_deg=err,
                    resynced_to=resynced,
                    event=event,
                )

            logger.warning(
                "drift_detector.converge_failed",
                cam_id=cam_id,
                error=newton.error,
                iterations=newton.iterations,
                final_error_deg=round(err, 3) if not math.isnan(err) else None,
            )
            event = await self._publish_converge_event(
                cam_id=cam_id,
                checkpoint_name=checkpoint_name,
                request_id=last_request_id,
                expected=target_angle_deg,
                actual=last_actual,
                drift=err,
                correction_steps=total_correction_steps,
                corrected=False,
                iterations=newton.iterations,
                resynced_to=None,
            )
            return ConvergeResult(
                converged=False,
                iterations=newton.iterations,
                final_error_deg=err,
                resynced_to=None,
                event=event,
            )
        finally:
            drive.calibration_mode = was_cal_mode

    async def _publish_converge_event(
        self,
        *,
        cam_id: str,
        checkpoint_name: str,
        request_id: str,
        expected: float,
        actual: float,
        drift: float,
        correction_steps: int,
        corrected: bool,
        iterations: int,
        resynced_to: float | None,
    ) -> DriftDetectionEvent:
        event = DriftDetectionEvent(
            request_id=request_id,
            cam_id=cam_id,
            drive_axis="b",
            checkpoint_name=checkpoint_name,
            expected_angle_deg=expected,
            actual_angle_deg=actual,
            drift_deg=drift,
            correction_steps=correction_steps,
            corrected=corrected,
            iterations=iterations,
            resynced_to=resynced_to,
        )
        await self._mqtt.publish(
            topic(EVENT_DRIFT, cam_id=cam_id, axis="b"),
            event,
            qos=1,
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

    def get_reference_position(self, cam_id: str, axis: str = "b") -> float | None:
        """Motor-step count at which the IMU active-angle reads the per-camera
        reference_angle_deg (−90° for upside-down, +90° for right-side-up).
        Discovered at startup by auto_calibrator; None until auto-cal succeeds."""
        val = (
            self._calibration
            .get("reference_position", {})
            .get(f"{cam_id}_{axis}")
        )
        return float(val) if val is not None else None

    def set_reference_position(self, cam_id: str, value: float, axis: str = "b") -> None:
        self._calibration.setdefault("reference_position", {})[f"{cam_id}_{axis}"] = value
        self._save_calibration()

    def get_reference_angle_deg(self, cam_id: str, axis: str = "b") -> float | None:
        """Per-camera rest anchor (−90° or +90°). Auto-detected at each startup
        by auto_calibrator from the sign of the first IMU reading."""
        val = (
            self._calibration
            .get("reference_angle_deg", {})
            .get(f"{cam_id}_{axis}")
        )
        return float(val) if val is not None else None

    def set_reference_angle_deg(self, cam_id: str, value: float, axis: str = "b") -> None:
        self._calibration.setdefault("reference_angle_deg", {})[f"{cam_id}_{axis}"] = value
        self._save_calibration()

    def set_target_sign(self, cam_id: str, sign: int, axis: str = "b") -> None:
        conv = self._calibration.setdefault("convergence", {})
        conv.setdefault("steps_per_degree_sign", {})[f"{cam_id}_{axis}"] = int(sign)
        self._save_calibration()

    def get_target_sign(self, cam_id: str, axis: str = "b") -> int:
        """Per-camera sign used to turn a user-entered angle magnitude into a
        signed target for converge(). Mirrors `steps_per_degree_sign` in the
        convergence block: +1 for right-side-up, -1 for flipped (cam2)."""
        conv = self._calibration.get("convergence", {}) or {}
        sign_map: dict = conv.get("steps_per_degree_sign", {}) or {}
        return int(sign_map.get(f"{cam_id}_{axis}", 1))

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
