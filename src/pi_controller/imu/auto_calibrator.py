"""
Auto-calibration of radial (axis-b) drives at service startup.

The Tinkerforge stepper's internal position counter zeroes on every power-up
and has no intrinsic geometric meaning — the physical "0°" orientation of
the camera maps to a different motor-step value each boot. This module
discovers two per-camera-per-axis quantities live at startup:

  zero_position[cam_id_b]   — motor-step count where IMU active-angle reads 0°
  steps_per_degree[cam_id_b] — signed magnitude of steps-per-degree-of-IMU

Runtime code (main.handle_move) can then compute
  coarse_target = zero_position + target_angle_deg * steps_per_degree * sign
instead of relying on the teach-time motor-position snapshot sent by Windows.

All moves run inside `drive.calibration_mode = True` so the envelope doesn't
clamp probes (motor counter may be outside [min,max] after restart drift).
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

import structlog

if TYPE_CHECKING:
    from src.pi_controller.drives.tinkerforge_drive import TinkerforgeDrive
    from src.pi_controller.imu.drift_detector import DriftDetector


logger = structlog.get_logger()


@dataclass
class AutoCalResult:
    cam_id: str
    axis: str  # always "b" for now
    success: bool
    zero_position: float | None
    steps_per_degree: float | None  # magnitude; sign stored separately
    direction_sign: int | None      # +1 if +steps increases active-angle, else -1
    iterations: int
    error: str | None


def _pick_angle(imu_obj, active_angle: str) -> float:
    return imu_obj.roll_deg if active_angle == "roll" else imu_obj.pitch_deg


async def _read_angle(
    drift_detector: DriftDetector,
    cam_id: str,
    active_angle: str,
    label: str,
) -> float | None:
    """Request an IMU reading; return the selected angle or None on timeout."""
    try:
        imu = await drift_detector.request_imu_check(cam_id, label)
    except TimeoutError:
        logger.error("auto_calibrator.imu_timeout", cam_id=cam_id, label=label)
        return None
    return _pick_angle(imu, active_angle)


async def auto_calibrate(
    drift_detector: DriftDetector,
    drive: TinkerforgeDrive,
    cam_id: str,
    *,
    active_angle: Literal["roll", "pitch"] = "roll",
    config: dict,
) -> AutoCalResult:
    """Discover zero_position and steps_per_degree for a single cam:b drive.

    `config` is the `auto_calibration` sub-dict from imu_calibration.yaml.
    """
    axis = "b"
    probe_steps = int(config.get("probe_steps", 200))
    measure_steps = int(config.get("measure_steps", 500))
    zero_threshold_deg = float(config.get("zero_threshold_deg", 0.2))
    max_iterations = int(config.get("max_iterations", 30))
    max_step_per_iter = int(config.get("max_step_per_iter", 1000))
    safety_travel = int(config.get("safety_travel", 8000))
    settle_speed = float(config.get("settle_speed", 0.3))
    yaml_default_spd = float(config.get("fallback_steps_per_degree", 120.0))

    start_pos = drive.current_position
    was_cal_mode = drive.calibration_mode
    drive.calibration_mode = True
    iterations = 0
    try:
        # ───────────── Phase 1: direction probe ─────────────
        start_angle = await _read_angle(
            drift_detector, cam_id, active_angle, "autocal:phase1_start"
        )
        if start_angle is None:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, iterations, "imu_timeout"
            )

        await drive.move_to(start_pos + probe_steps, speed=settle_speed)
        iterations += 1
        probe_angle = await _read_angle(
            drift_detector, cam_id, active_angle, "autocal:phase1_probe"
        )
        if probe_angle is None:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, iterations, "imu_timeout"
            )

        delta = probe_angle - start_angle
        logger.info(
            "auto_calibrator.phase1_probe",
            cam_id=cam_id,
            start_angle=round(start_angle, 3),
            probe_angle=round(probe_angle, 3),
            delta_deg=round(delta, 3),
            probe_steps=probe_steps,
        )

        if abs(delta) < zero_threshold_deg:
            # Motor moved but camera angle didn't — stuck, uncoupled, or
            # probe too small. Abort so operator can investigate.
            await drive.move_to(start_pos, speed=settle_speed)
            iterations += 1
            return AutoCalResult(
                cam_id, axis, False, None, None, None, iterations, "probe_no_motion"
            )

        direction_sign = 1 if delta > 0 else -1
        est_spd = probe_steps / abs(delta)

        # Return to start so Phase 2 searches symmetrically.
        await drive.move_to(start_pos, speed=settle_speed)
        iterations += 1

        # ───────────── Phase 2: zero-angle Newton search ─────────────
        for _ in range(max_iterations):
            angle = await _read_angle(
                drift_detector, cam_id, active_angle, "autocal:phase2_search"
            )
            if angle is None:
                return AutoCalResult(
                    cam_id, axis, False, None, None, None, iterations, "imu_timeout"
                )

            if abs(angle) <= zero_threshold_deg:
                break

            delta_steps = int(-angle * est_spd * direction_sign)
            if delta_steps > max_step_per_iter:
                delta_steps = max_step_per_iter
            elif delta_steps < -max_step_per_iter:
                delta_steps = -max_step_per_iter

            next_pos = drive.current_position + delta_steps
            if abs(next_pos - start_pos) > safety_travel:
                return AutoCalResult(
                    cam_id,
                    axis,
                    False,
                    None,
                    None,
                    None,
                    iterations,
                    "safety_travel_exceeded",
                )

            await drive.move_to(next_pos, speed=settle_speed)
            iterations += 1
        else:
            return AutoCalResult(
                cam_id,
                axis,
                False,
                None,
                None,
                None,
                iterations,
                "zero_not_found",
            )

        zero_position = drive.current_position
        logger.info(
            "auto_calibrator.phase2_zero_found",
            cam_id=cam_id,
            zero_position=zero_position,
            iterations=iterations,
        )

        # ───────────── Phase 3: steps-per-degree measurement ─────────────
        await drive.move_to(zero_position + measure_steps, speed=settle_speed)
        iterations += 1
        angle_pos = await _read_angle(
            drift_detector, cam_id, active_angle, "autocal:phase3_pos"
        )
        if angle_pos is None:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, iterations, "imu_timeout"
            )

        await drive.move_to(zero_position - measure_steps, speed=settle_speed)
        iterations += 1
        angle_neg = await _read_angle(
            drift_detector, cam_id, active_angle, "autocal:phase3_neg"
        )
        if angle_neg is None:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, iterations, "imu_timeout"
            )

        span = abs(angle_pos - angle_neg)
        if span < zero_threshold_deg:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, iterations, "span_too_small"
            )
        steps_per_degree = (2.0 * measure_steps) / span

        logger.info(
            "auto_calibrator.phase3_spd",
            cam_id=cam_id,
            angle_pos=round(angle_pos, 3),
            angle_neg=round(angle_neg, 3),
            span_deg=round(span, 3),
            steps_per_degree=round(steps_per_degree, 3),
            fallback_spd=yaml_default_spd,
        )

        # ───────────── Phase 4: return to origin ─────────────
        await drive.move_to(start_pos, speed=settle_speed)
        iterations += 1

        return AutoCalResult(
            cam_id=cam_id,
            axis=axis,
            success=True,
            zero_position=float(zero_position),
            steps_per_degree=float(steps_per_degree),
            direction_sign=direction_sign,
            iterations=iterations,
            error=None,
        )
    except Exception as exc:  # defensive — any drive/IMU fault
        logger.exception("auto_calibrator.unexpected_error", cam_id=cam_id)
        return AutoCalResult(
            cam_id, axis, False, None, None, None, iterations, f"exception:{exc}"
        )
    finally:
        drive.calibration_mode = was_cal_mode
