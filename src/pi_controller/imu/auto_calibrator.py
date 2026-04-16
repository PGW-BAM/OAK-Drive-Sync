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
        # Correction drives (current_angle - reference_angle_deg) toward zero.
        # Phase 1's direction_sign inference can be wrong if IMU noise flips
        # the measured probe delta. Three recovery signals:
        #   - err-regression: |err| grows after a substantive correction AND
        #                     the error sign didn't flip. Real wrong-direction
        #                     move. Flip immediately (one bad step is enough).
        #   - crossover:      err sign flipped — we stepped past the target.
        #                     Not a direction problem; est_spd was too high and
        #                     we overshot. Don't flip; just let the re-estimated
        #                     est_spd shrink the next step.
        #   - angle-stuck:    commanded a meaningful move, IMU didn't respond.
        #                     Interpret as "end-stop reached in that motor
        #                     direction" — DO NOT push further the same way.
        #                     Back the drive off to start_pos, flip direction,
        #                     and try from the other side once. If stuck again
        #                     after the flip, the camera is pinned between two
        #                     end-stops (or the mechanism is decoupled) — abort
        #                     with a clear hint instead of wasting more travel.
        #
        # est_spd is re-estimated after every substantive iteration (smoothed)
        # because Phase 1's probe often runs inside backlash and underestimates
        # the true steps-per-degree ratio by several × once real motion starts.
        # Without this, Newton steps chronically overshoot near the target.
        #
        # On Tinkerforge, drive.current_position always equals the last
        # commanded target (no stall feedback), so angle is the only reliable
        # signal of physical motion.
        min_response_deg = 0.2
        regression_threshold_deg = max(ref_threshold_deg, motion_threshold_deg)
        spd_smoothing = 0.5  # 0 = trust new measurement fully, 1 = never update
        spd_min = 1.0        # clamp so we never divide by ~zero
        prev_angle: float | None = None
        prev_abs_err: float | None = None
        prev_err_sign: int = 0
        prev_delta_steps: int = 0
        flipped_in_phase2 = False
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
            err_sign = 1 if err > 0 else (-1 if err < 0 else 0)

            # Re-estimate steps-per-degree from the last substantive iteration's
            # observed response. This is critical when the Phase 1 probe ran
            # mostly inside backlash: once real motion starts, the true ratio
            # can be many × smaller, and an over-high est_spd causes repeated
            # Newton overshoot. Only update from meaningful moves that actually
            # rotated the camera in the expected motor direction.
            if (
                prev_angle is not None
                and abs(prev_delta_steps)
                >= max(1, int(est_spd * regression_threshold_deg))
                and abs(angle - prev_angle) >= min_response_deg
            ):
                measured_spd = abs(prev_delta_steps) / abs(angle - prev_angle)
                measured_spd = max(spd_min, measured_spd)
                new_spd = spd_smoothing * est_spd + (1.0 - spd_smoothing) * measured_spd
                if abs(new_spd - est_spd) >= 1.0:
                    logger.info(
                        "auto_calibrator.phase2_spd_update",
                        cam_id=cam_id,
                        prev_spd=round(est_spd, 3),
                        measured_spd=round(measured_spd, 3),
                        new_spd=round(new_spd, 3),
                        prev_delta_steps=prev_delta_steps,
                        angle_delta=round(angle - prev_angle, 3),
                    )
                est_spd = new_spd

            # Detect an err sign flip (crossover). If prev_err was non-zero and
            # current err has the opposite sign, we stepped across the target.
            # That's overshoot, not wrong direction — the spd re-estimate above
            # will damp the next step naturally. We skip err_regression in that
            # case so we don't abort near-convergence runs.
            crossed = (
                prev_err_sign != 0 and err_sign != 0 and err_sign != prev_err_sign
            )
            if crossed:
                logger.info(
                    "auto_calibrator.phase2_crossover",
                    cam_id=cam_id,
                    prev_abs_err=round(prev_abs_err, 3) if prev_abs_err else None,
                    current_abs_err=round(abs(err), 3),
                    prev_delta_steps=prev_delta_steps,
                )

            # Direction-inference recovery checks. The previous iteration is
            # "substantive" only if it commanded ≥ regression_threshold_deg
            # worth of steps — protects against noise-triggered false alarms
            # on tiny corrections that finished convergence.
            if (
                prev_abs_err is not None
                and abs(prev_delta_steps)
                >= max(1, int(est_spd * regression_threshold_deg))
            ):
                angle_delta = abs(angle - prev_angle) if prev_angle is not None else 0.0
                err_grew = (
                    not crossed
                    and abs(err) > prev_abs_err + regression_threshold_deg
                )
                angle_stuck = angle_delta < min_response_deg

                if err_grew:
                    # |err| grew after a real correction → direction is wrong.
                    # Don't wait — flip on the first occurrence. This is
                    # qualitatively different from "stuck" (which might just be
                    # backlash): here we actually moved, and it made things
                    # worse.
                    if not flipped_in_phase2:
                        old_dir = direction_sign
                        direction_sign = -direction_sign
                        flipped_in_phase2 = True
                        logger.warning(
                            "auto_calibrator.phase2_direction_flip",
                            cam_id=cam_id,
                            reason="err_regression",
                            prev_abs_err=round(prev_abs_err, 3),
                            current_abs_err=round(abs(err), 3),
                            prev_delta_steps=prev_delta_steps,
                            angle_delta=round(angle_delta, 3),
                            old_direction_sign=old_dir,
                            new_direction_sign=direction_sign,
                        )
                    else:
                        logger.error(
                            "auto_calibrator.phase2_abort_err_regression_after_flip",
                            cam_id=cam_id,
                            current_position=drive.current_position,
                            start_pos=start_pos,
                            current_angle=round(angle, 3),
                            reference_angle_deg=reference_angle_deg,
                            current_abs_err=round(abs(err), 3),
                            hint=(
                                "Both directions regress — IMU axis mismatch "
                                "or camera mounting inverted. Verify active_angle "
                                "config matches mounting and reference_angle_deg "
                                "sign is correct for this cam."
                            ),
                        )
                        await _move_and_settle(
                            drive, start_pos,
                            speed=settle_speed, settle_ms=settle_ms,
                        )
                        iterations += 1
                        return AutoCalResult(
                            cam_id, axis, False, None, None, None, None, iterations,
                            "phase2_both_directions_err_regression",
                        )
                elif angle_stuck:
                    # Commanded a real move, IMU didn't respond → we just
                    # pushed into a physical end-stop (or the mechanism
                    # decoupled). Escalating the step size would just grind
                    # harder into the stop; instead we back off to start_pos
                    # and try the opposite motor direction once. If stuck
                    # again after the flip, the camera is pinned between two
                    # stops (or the mechanism is broken) — abort cleanly.
                    logger.warning(
                        "auto_calibrator.phase2_angle_stuck",
                        cam_id=cam_id,
                        flipped_in_phase2=flipped_in_phase2,
                        prev_delta_steps=prev_delta_steps,
                        angle_delta=round(angle_delta, 3),
                        current_abs_err=round(abs(err), 3),
                        current_position=drive.current_position,
                    )
                    if not flipped_in_phase2:
                        old_dir = direction_sign
                        direction_sign = -direction_sign
                        flipped_in_phase2 = True
                        logger.warning(
                            "auto_calibrator.phase2_direction_flip",
                            cam_id=cam_id,
                            reason="angle_stuck_end_stop",
                            old_direction_sign=old_dir,
                            new_direction_sign=direction_sign,
                            current_position=drive.current_position,
                            current_abs_err=round(abs(err), 3),
                            hint=(
                                "Interpreted as end-stop in the previous "
                                "motor direction. Backing off to start_pos "
                                "and trying the opposite direction."
                            ),
                        )
                        # Back off from the end-stop before the next attempt.
                        await _move_and_settle(
                            drive, start_pos,
                            speed=settle_speed, settle_ms=settle_ms,
                        )
                        iterations += 1
                        # Reset delta history so the next iteration's
                        # recovery logic doesn't fire off the pre-flip move.
                        prev_angle = None
                        prev_abs_err = None
                        prev_err_sign = 0
                        prev_delta_steps = 0
                        continue
                    else:
                        logger.error(
                            "auto_calibrator.phase2_abort_both_directions_stuck",
                            cam_id=cam_id,
                            current_position=drive.current_position,
                            start_pos=start_pos,
                            travel_from_start=drive.current_position - start_pos,
                            current_angle=round(angle, 3),
                            reference_angle_deg=reference_angle_deg,
                            current_abs_err=round(abs(err), 3),
                            hint=(
                                "Camera appears pinned against a physical "
                                "end-stop in both motor directions. Manually "
                                "center the camera near the reference angle "
                                "and restart, or relax the soft envelope so "
                                "the motor can clear the stop."
                            ),
                        )
                        await _move_and_settle(
                            drive, start_pos,
                            speed=settle_speed, settle_ms=settle_ms,
                        )
                        iterations += 1
                        return AutoCalResult(
                            cam_id, axis, False, None, None, None, None, iterations,
                            "phase2_both_directions_angle_stuck",
                        )

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
                logger.error(
                    "auto_calibrator.phase2_abort_safety_travel",
                    cam_id=cam_id,
                    current_position=pre_pos,
                    attempted_next=next_pos,
                    start_pos=start_pos,
                    safety_travel=safety_travel,
                    current_angle=round(angle, 3),
                    reference_angle_deg=reference_angle_deg,
                    current_abs_err=round(abs(err), 3),
                )
                return AutoCalResult(
                    cam_id, axis, False, None, None, None, None, iterations,
                    "safety_travel_exceeded",
                )

            await _move_and_settle(
                drive, next_pos,
                speed=settle_speed, settle_ms=settle_ms,
            )
            iterations += 1

            prev_angle = angle
            prev_abs_err = abs(err)
            prev_err_sign = err_sign
            prev_delta_steps = delta_steps
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

        # ───────────── Phase 4: return to origin ─────────────
        await _move_and_settle(
            drive, start_pos,
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
