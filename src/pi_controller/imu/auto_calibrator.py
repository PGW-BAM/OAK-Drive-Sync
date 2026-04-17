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

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

import structlog

from src.pi_controller.imu.newton_search import newton_angle_search

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


async def _move_and_settle(
    drive: TinkerforgeDrive,
    target: float,
    *,
    speed: float,
    settle_ms: int,
) -> None:
    """Move the drive, then wait for the camera body to stop oscillating.

    The Windows-side IMU derives roll/pitch from the accelerometer only — any
    residual body swing after `position_reached` fires adds vector noise to
    gravity and produces an unreliable reading. A short settle delay ensures
    the angle we sample reflects the static post-move orientation, not the
    transient wobble.
    """
    await drive.move_to(target, speed=speed)
    if settle_ms > 0:
        await asyncio.sleep(settle_ms / 1000.0)


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
    # Delay between `position_reached` and the next IMU sample. Accelerometer-
    # derived roll is noisy during and immediately after motion; without this
    # pause we would read a swinging value and fail to detect stuck / end-stop
    # conditions accurately.
    settle_ms = int(config.get("imu_settle_ms", 400))

    start_pos = drive.current_position
    was_cal_mode = drive.calibration_mode
    drive.calibration_mode = True

    # Temporarily raise motor current for the calibration pass. A cam parked
    # near ±90° roll generates significant off-axis gravitational load; at the
    # default 1200 mA the Silent Stepper 2.0 can stall silently (no stall
    # detection on this part) so the bricklet reports every position_reached
    # while the camera hasn't actually rotated — that produces the symptom of
    # flat IMU across a ±2000-step sweep in both directions. Boosting to 1800
    # mA during auto-cal lets the motor overcome the load. Snapshot and
    # restore the prior value so runtime moves don't run hot.
    boost_current_mA = int(config.get("motor_current_boost_mA", 0))
    orig_current_mA: int | None = None
    if boost_current_mA > 0 and hasattr(drive, "get_motor_current_mA"):
        try:
            orig_current_mA = drive.get_motor_current_mA()
            if orig_current_mA is not None and boost_current_mA > orig_current_mA:
                drive.set_motor_current_mA(boost_current_mA)
                logger.info(
                    "auto_calibrator.motor_current_boost",
                    cam_id=cam_id,
                    original_mA=orig_current_mA,
                    boosted_mA=boost_current_mA,
                )
            else:
                orig_current_mA = None  # no change needed — skip restore
        except Exception:
            logger.exception("auto_calibrator.motor_current_boost_failed", cam_id=cam_id)
            orig_current_mA = None

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

        # ───────────── Phase 1: direction probe (escalating) ─────────────
        # Probe toward the target reference angle so the first commanded
        # move never drives the camera AWAY from where we need to go.
        # Small probes (e.g. ±200 steps) often live entirely inside
        # mechanical backlash and produce no camera rotation, even when
        # the mechanism is perfectly fine — so we escalate step size
        # (1×, 3×, 5× `probe_steps`) in the preferred direction before
        # reversing, and repeat the escalation in reverse before giving
        # up. The angle (not the motor position) is the authoritative
        # signal: Tinkerforge steppers have no stall feedback, so
        # drive.current_position always equals the commanded target
        # regardless of whether the motor physically moved.
        preferred_dir = 1 if reference_angle_deg > start_angle else -1
        probe_multipliers = (1, 3, 5)
        # Alternate directions on each size before escalating: if the 1× probe
        # in the preferred direction reads 0°, the drive is almost certainly
        # pressed against an end-stop in THAT direction — more steps the same
        # way just push harder into the stop. Trying 1× the other way first
        # catches that case in one move instead of three. Only after both 1×
        # probes fail do we escalate to 3×, and so on.
        probe_order: list[tuple[int, int]] = []
        for mult in probe_multipliers:
            probe_order.append((preferred_dir, mult))
            probe_order.append((-preferred_dir, mult))

        delta_pos = 0.0
        delta_angle = 0.0
        probe_found = False
        for probe_dir, mult in probe_order:
            probe_offset = probe_dir * probe_steps * mult
            if abs(probe_offset) > safety_travel:
                continue  # don't probe beyond safety budget
            await _move_and_settle(
                drive, start_pos + probe_offset,
                speed=settle_speed, settle_ms=settle_ms,
            )
            iterations += 1
            probe_angle = await _read_angle(
                drift_detector,
                cam_id,
                active_angle,
                f"autocal:phase1_probe_dir{probe_dir:+d}_x{mult}",
            )
            if probe_angle is None:
                return AutoCalResult(
                    cam_id, axis, False, None, None, None, None, iterations, "imu_timeout"
                )
            cur_delta_pos = drive.current_position - start_pos
            cur_delta_angle = probe_angle - start_angle
            logger.info(
                "auto_calibrator.phase1_probe_attempt",
                cam_id=cam_id,
                probe_dir=probe_dir,
                step_multiplier=mult,
                probe_offset=probe_offset,
                start_angle=round(start_angle, 3),
                probe_angle=round(probe_angle, 3),
                delta_deg=round(cur_delta_angle, 3),
                delta_pos=cur_delta_pos,
                motion_threshold_deg=motion_threshold_deg,
            )
            if abs(cur_delta_angle) >= motion_threshold_deg:
                delta_pos = cur_delta_pos
                delta_angle = cur_delta_angle
                probe_found = True
                break
            # No response yet — return to start_pos and try the next probe
            # (larger magnitude, or reversed direction).
            await _move_and_settle(
                drive, start_pos,
                speed=settle_speed, settle_ms=settle_ms,
            )
            iterations += 1

        if not probe_found:
            logger.error(
                "auto_calibrator.phase1_no_angle_response",
                cam_id=cam_id,
                start_pos=start_pos,
                start_angle=round(start_angle, 3),
                attempted_multipliers=list(probe_multipliers),
                probe_steps=probe_steps,
                hint=(
                    "No IMU response at any probe size up to "
                    f"{probe_steps * max(probe_multipliers)} steps in either "
                    "direction. Likely a disengaged drive coupling, camera "
                    "pinned between tight end-stops, or wrong active_angle "
                    "axis for this mounting."
                ),
            )
            await _move_and_settle(
                drive, start_pos,
                speed=settle_speed, settle_ms=settle_ms,
            )
            iterations += 1
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations,
                "no_angle_response_both_directions",
            )

        # direction_sign = +1 iff positive motor steps produce positive angle delta.
        direction_sign = 1 if (delta_angle * delta_pos) > 0 else -1
        est_spd = abs(delta_pos) / abs(delta_angle)
        logger.info(
            "auto_calibrator.phase1_direction_locked",
            cam_id=cam_id,
            direction_sign=direction_sign,
            est_spd=round(est_spd, 3),
            delta_pos=delta_pos,
            delta_angle=round(delta_angle, 3),
        )

        # Return to start so Phase 2 searches symmetrically.
        await _move_and_settle(
            drive, start_pos,
            speed=settle_speed, settle_ms=settle_ms,
        )
        iterations += 1

        # ───────────── Phase 2: reference-angle Newton search ─────────────
        # Delegates to the shared newton_angle_search helper so the startup
        # auto-cal and the manual converge() path stay in sync. See
        # src/pi_controller/imu/newton_search.py for the algorithm details
        # (EMA est_spd re-estimation, crossover, err-regression flip, angle-
        # stuck end-stop recovery).
        async def _read(label: str) -> float | None:
            return await _read_angle(drift_detector, cam_id, active_angle, label)

        newton = await newton_angle_search(
            cam_id=cam_id,
            drive=drive,
            read_angle=_read,
            target_angle_deg=reference_angle_deg,
            threshold_deg=ref_threshold_deg,
            motion_threshold_deg=motion_threshold_deg,
            max_iterations=max_iterations,
            max_step_per_iter=max_step_per_iter,
            est_spd_init=est_spd,
            direction_sign=direction_sign,
            settle_speed=settle_speed,
            settle_ms=settle_ms,
            safety_travel=safety_travel,
            start_pos=start_pos,
            log_prefix="auto_calibrator.phase2",
        )
        iterations += newton.iterations
        direction_sign = newton.direction_sign
        est_spd = newton.est_spd

        if not newton.converged:
            error_code = newton.error or "reference_not_found"
            # Map helper error codes to legacy auto-cal codes for downstream
            # consumers (GUI shows the exact string; changing would break
            # operator muscle memory).
            mapped = {
                "both_directions_err_regression": "phase2_both_directions_err_regression",
                "both_directions_angle_stuck": "phase2_both_directions_angle_stuck",
                "max_iterations_exceeded": "reference_not_found",
            }.get(error_code, error_code)
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations,
                mapped,
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
        await _move_and_settle(
            drive, reference_position + measure_steps,
            speed=settle_speed, settle_ms=settle_ms,
        )
        iterations += 1
        angle_pos = await _read_angle(
            drift_detector, cam_id, active_angle, "autocal:phase3_pos"
        )
        if angle_pos is None:
            return AutoCalResult(
                cam_id, axis, False, None, None, None, None, iterations,
                "imu_timeout",
            )

        await _move_and_settle(
            drive, reference_position - measure_steps,
            speed=settle_speed, settle_ms=settle_ms,
        )
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

        # ───────────── Phase 4: park at reference position ─────────────
        # Leave the camera at the ±90° anchor it just found. Previously we
        # moved back to start_pos, which undid all the search work and left
        # the user looking at the camera's arbitrary boot angle (~±75°)
        # instead of the calibrated ±90° reference. Operators expect the
        # camera to stay where auto-cal pointed it.
        await _move_and_settle(
            drive, reference_position,
            speed=settle_speed, settle_ms=settle_ms,
        )
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
        if orig_current_mA is not None and hasattr(drive, "set_motor_current_mA"):
            try:
                drive.set_motor_current_mA(orig_current_mA)
                logger.info(
                    "auto_calibrator.motor_current_restored",
                    cam_id=cam_id,
                    motor_current_mA=orig_current_mA,
                )
            except Exception:
                logger.exception(
                    "auto_calibrator.motor_current_restore_failed",
                    cam_id=cam_id,
                )
