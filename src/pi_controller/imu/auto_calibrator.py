"""
Auto-calibration of radial (axis-b) drives at service startup.

The Tinkerforge stepper's internal position counter zeroes on every power-up
and has no intrinsic geometric meaning — the physical rest orientation of
the camera maps to a different motor-step value each boot. This module
discovers three per-camera-per-axis quantities live at startup:

  reference_angle_deg[cam_id_b] — the physical mounting's rest-side anchor:
      −90° for upside-down (cam1), +90° for right-side-up (cam2). Auto-
      detected from the sign of the first IMU reading.
  reference_position[cam_id_b]  — motor-step count where IMU active-angle
      reads the reference_angle (to within `reference_threshold_deg`).
  steps_per_degree[cam_id_b]    — signed magnitude of steps-per-degree-of-IMU

Runtime code (main.handle_move) can then compute
  coarse_target = reference_position
                + (target_angle_deg - reference_angle_deg) * spd * sign
instead of relying on the teach-time motor-position snapshot sent by Windows.

All moves run inside `drive.calibration_mode = True` so the envelope doesn't
clamp probes (motor counter may be outside [min,max] after restart drift).
"""

from __future__ import annotations

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
    reference_position: float | None   # motor-step count at reference_angle_deg
    reference_angle_deg: float | None  # ±90, auto-detected per cam
    steps_per_degree: float | None     # magnitude; sign stored separately
    direction_sign: int | None         # +1 if +steps increases active-angle, else -1
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
    """Discover (reference_angle_deg, reference_position, steps_per_degree) for
    a single cam:b drive.

    `config` is the `auto_calibration` sub-dict from imu_calibration.yaml.
    """
    axis = "b"
    probe_steps = int(config.get("probe_steps", 200))
    measure_steps = int(config.get("measure_steps", 500))
    ref_threshold_deg = float(config.get("reference_threshold_deg", 0.5))
    motion_threshold_deg = float(config.get("motion_threshold_deg", 0.2))
    max_iterations = int(config.get("max_iterations", 30))
    max_step_per_iter = int(config.get("max_step_per_iter", 1000))
    safety_travel = int(config.get("safety_travel", 8000))
    settle_speed = float(config.get("settle_speed", 0.3))

    start_pos = drive.current_position
    was_cal_mode = drive.calibration_mode
    drive.calibration_mode = True
    iterations = 0
    try:
        # ───────────── Phase 0: detect reference_angle from sign ─────────────
        start_angle = await _read_angle(
            drift_detector, cam_id, active_angle, "autocal:phase0_sign"
        )
        if start_angle is None:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations, "imu_timeout"
            )
        # Cam1 is mounted upside-down → IMU always reads negative.
        # Cam2 is right-side-up → IMU always reads positive.
        # The sign of the first reading tells us which rest anchor (−90 or +90)
        # this camera's axis-b drive belongs to.
        reference_angle_deg = -90.0 if start_angle < 0 else 90.0
        logger.info(
            "auto_calibrator.phase0_reference",
            cam_id=cam_id,
            start_angle=round(start_angle, 3),
            reference_angle_deg=reference_angle_deg,
        )

        # ───────────── Phase 1: direction probe ─────────────
        # Probe toward the target reference angle so the first commanded
        # move never drives the camera AWAY from where we need to go (and
        # in particular never pushes a motor that's already near a
        # physical end-stop deeper into it). We guess direction_sign=+1
        # and pick a step direction that — under that guess — reduces the
        # angle error. If the guess is wrong, we detect it from the
        # measured delta and flip direction_sign. If the motor can't move
        # in the first chosen direction (stuck at a physical limit), we
        # reverse and retry before giving up.
        stuck_pos_threshold = max(1, probe_steps // 10)

        probe_dir = 1 if reference_angle_deg > start_angle else -1
        probe_offset = probe_dir * probe_steps
        await drive.move_to(start_pos + probe_offset, speed=settle_speed)
        iterations += 1
        probe_angle = await _read_angle(
            drift_detector, cam_id, active_angle, "autocal:phase1_probe"
        )
        if probe_angle is None:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations, "imu_timeout"
            )
        delta_pos = drive.current_position - start_pos
        delta_angle = probe_angle - start_angle
        logger.info(
            "auto_calibrator.phase1_probe",
            cam_id=cam_id,
            start_angle=round(start_angle, 3),
            probe_angle=round(probe_angle, 3),
            delta_deg=round(delta_angle, 3),
            delta_pos=delta_pos,
            probe_offset=probe_offset,
        )

        # Motor didn't advance — probably at a physical end-stop in this
        # direction. Return to start and probe the other way.
        if abs(delta_pos) < stuck_pos_threshold:
            logger.warning(
                "auto_calibrator.phase1_stuck_retry_reverse",
                cam_id=cam_id,
                delta_pos=delta_pos,
            )
            await drive.move_to(start_pos, speed=settle_speed)
            iterations += 1
            probe_dir = -probe_dir
            probe_offset = probe_dir * probe_steps
            await drive.move_to(start_pos + probe_offset, speed=settle_speed)
            iterations += 1
            probe_angle = await _read_angle(
                drift_detector, cam_id, active_angle, "autocal:phase1_probe_reverse"
            )
            if probe_angle is None:
                return AutoCalResult(
                    cam_id, axis, False, None, None, None, None, iterations, "imu_timeout"
                )
            delta_pos = drive.current_position - start_pos
            delta_angle = probe_angle - start_angle
            logger.info(
                "auto_calibrator.phase1_probe_reverse",
                cam_id=cam_id,
                probe_angle=round(probe_angle, 3),
                delta_deg=round(delta_angle, 3),
                delta_pos=delta_pos,
            )
            if abs(delta_pos) < stuck_pos_threshold:
                return AutoCalResult(
                    cam_id, axis, False, None, None, None, None, iterations,
                    "drive_stuck_both_directions",
                )

        if abs(delta_angle) < motion_threshold_deg:
            # Motor moved but camera angle didn't — uncoupled, or probe
            # too small. Abort so operator can investigate.
            await drive.move_to(start_pos, speed=settle_speed)
            iterations += 1
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations,
                "probe_no_motion",
            )

        # direction_sign = +1 iff positive motor steps produce positive angle delta.
        direction_sign = 1 if (delta_angle * delta_pos) > 0 else -1
        est_spd = abs(delta_pos) / abs(delta_angle)

        # Return to start so Phase 2 searches symmetrically.
        await drive.move_to(start_pos, speed=settle_speed)
        iterations += 1

        # ───────────── Phase 2: reference-angle Newton search ─────────────
        # Correction drives (current_angle - reference_angle_deg) toward zero.
        # Bail out if the motor stops advancing — that means we've hit a
        # physical end-stop and further commands just pound the mechanism.
        phase2_stuck_threshold = max(1, int(est_spd * 0.1))
        for _ in range(max_iterations):
            angle = await _read_angle(
                drift_detector, cam_id, active_angle, "autocal:phase2_search"
            )
            if angle is None:
                return AutoCalResult(
                    cam_id, axis, False, None, None, None, None, iterations,
                    "imu_timeout",
                )

            err = angle - reference_angle_deg
            if abs(err) <= ref_threshold_deg:
                break

            delta_steps = int(-err * est_spd * direction_sign)
            if delta_steps > max_step_per_iter:
                delta_steps = max_step_per_iter
            elif delta_steps < -max_step_per_iter:
                delta_steps = -max_step_per_iter

            pre_pos = drive.current_position
            next_pos = pre_pos + delta_steps
            if abs(next_pos - start_pos) > safety_travel:
                return AutoCalResult(
                    cam_id, axis, False, None, None, None, None, iterations,
                    "safety_travel_exceeded",
                )

            await drive.move_to(next_pos, speed=settle_speed)
            iterations += 1
            advancement = drive.current_position - pre_pos
            if abs(delta_steps) > phase2_stuck_threshold and abs(advancement) < phase2_stuck_threshold:
                logger.warning(
                    "auto_calibrator.phase2_drive_stuck",
                    cam_id=cam_id,
                    commanded=delta_steps,
                    advanced=advancement,
                    angle=round(angle, 3),
                    reference_angle_deg=reference_angle_deg,
                )
                return AutoCalResult(
                    cam_id, axis, False, None, None, None, None, iterations,
                    "phase2_drive_stuck",
                )
        else:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations,
                "reference_not_found",
            )

        reference_position = drive.current_position
        logger.info(
            "auto_calibrator.phase2_reference_found",
            cam_id=cam_id,
            reference_position=reference_position,
            reference_angle_deg=reference_angle_deg,
            iterations=iterations,
        )

        # ───────────── Phase 3: steps-per-degree measurement ─────────────
        await drive.move_to(reference_position + measure_steps, speed=settle_speed)
        iterations += 1
        angle_pos = await _read_angle(
            drift_detector, cam_id, active_angle, "autocal:phase3_pos"
        )
        if angle_pos is None:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations,
                "imu_timeout",
            )

        await drive.move_to(reference_position - measure_steps, speed=settle_speed)
        iterations += 1
        angle_neg = await _read_angle(
            drift_detector, cam_id, active_angle, "autocal:phase3_neg"
        )
        if angle_neg is None:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations,
                "imu_timeout",
            )

        span = abs(angle_pos - angle_neg)
        if span < motion_threshold_deg:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations,
                "span_too_small",
            )
        steps_per_degree = (2.0 * measure_steps) / span

        logger.info(
            "auto_calibrator.phase3_spd",
            cam_id=cam_id,
            angle_pos=round(angle_pos, 3),
            angle_neg=round(angle_neg, 3),
            span_deg=round(span, 3),
            steps_per_degree=round(steps_per_degree, 3),
        )

        # ───────────── Phase 4: return to origin ─────────────
        await drive.move_to(start_pos, speed=settle_speed)
        iterations += 1

        return AutoCalResult(
            cam_id=cam_id,
            axis=axis,
            success=True,
            reference_position=float(reference_position),
            reference_angle_deg=float(reference_angle_deg),
            steps_per_degree=float(steps_per_degree),
            direction_sign=direction_sign,
            iterations=iterations,
            error=None,
        )
    except Exception as exc:  # defensive — any drive/IMU fault
        logger.exception("auto_calibrator.unexpected_error", cam_id=cam_id)
        return AutoCalResult(
            cam_id, axis, False, None, None, None, None, iterations,
            f"exception:{exc}",
        )
    finally:
        drive.calibration_mode = was_cal_mode
