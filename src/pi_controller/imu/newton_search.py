"""
Shared Newton-style angle-convergence loop.

Used by:
  * auto_calibrator.py Phase 2 — converge on the ±90° reference angle at
    service startup (discovers reference_position).
  * drift_detector.py converge()  — manually-requested angle moves and
    sequence waypoints (user or MoveCommand supplies target_angle_deg).

Both callers run the same closed-loop IMU-feedback algorithm:

  1. Sample angle, compute signed err = angle - target_angle_deg.
  2. Re-estimate steps_per_degree (est_spd) from the last substantive move,
     smoothed with EMA (smoothing=0.1 — trust measurement quickly since
     Phase 1's probe often underestimates the true ratio by 2-5×).
  3. Detect crossover (err sign flipped → overshoot, skip recovery).
  4. Detect err regression (|err| grew after a real correction, no
     crossover) → flip direction once. Second flip → abort.
  5. Detect angle_stuck (commanded a real move, IMU didn't respond) →
     back off to start_pos, flip direction once. Second stuck → abort.
  6. If |err| <= threshold_deg, done.
  7. Otherwise, step `delta = -err * est_spd * direction_sign`, clamped
     to ±max_step_per_iter and to the safety_travel envelope.

Direction sign convention
-------------------------
`direction_sign = +1` iff a positive motor step produces a positive IMU
angle delta. cam2 mounts the bricklet inverted, so its direction_sign is
normally -1. The helper never needs to know about physical mounting; the
caller supplies the correct direction_sign and the signed target.

Error-model convention
----------------------
Signed raw error (`angle - target_angle_deg`). The magnitude-based error
model used by the old converge() is *not* supported — cam-mount inversion
must be handled by `direction_sign`, not by taking absolute values. This
lets both cams use the same loop body.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Awaitable, Callable

import asyncio
import structlog

if TYPE_CHECKING:
    from src.pi_controller.drives.tinkerforge_drive import TinkerforgeDrive


logger = structlog.get_logger()


@dataclass
class NewtonResult:
    converged: bool
    iterations: int            # iterations consumed inside this call
    final_angle: float | None  # last IMU reading sampled
    final_err: float | None    # last angle - target_angle_deg
    final_position: int        # drive.current_position at return
    error: str | None          # error code on failure, else None
    direction_sign: int        # possibly flipped during recovery
    est_spd: float             # possibly updated by EMA


async def _move_and_settle(
    drive: TinkerforgeDrive,
    target: float,
    *,
    speed: float,
    settle_ms: int,
) -> None:
    await drive.move_to(target, speed=speed)
    if settle_ms > 0:
        await asyncio.sleep(settle_ms / 1000.0)


async def newton_angle_search(
    *,
    cam_id: str,
    drive: "TinkerforgeDrive",
    read_angle: Callable[[str], Awaitable[float | None]],
    target_angle_deg: float,
    threshold_deg: float,
    motion_threshold_deg: float,
    max_iterations: int,
    max_step_per_iter: int,
    est_spd_init: float,
    direction_sign: int,
    settle_speed: float,
    settle_ms: int,
    safety_travel: int | None,
    start_pos: int,
    log_prefix: str = "newton",
    on_direction_sign_change: Callable[[int], None] | None = None,
    allow_direction_flip: bool = True,
    stuck_tolerance: int = 1,
    regression_tolerance: int = 1,
) -> NewtonResult:
    """Drive `drive` so the IMU reads `target_angle_deg` within `threshold_deg`.

    `read_angle(label)` is an async closure that performs the IMU fetch; the
    caller controls whether it is a passive cache read or a request/response.
    `log_prefix` is prepended to structlog event names so the auto-cal and
    manual paths stay distinguishable in the log stream.
    `safety_travel=None` disables the "drifted too far from start_pos" abort
    — appropriate for manual moves that may legitimately traverse the full
    calibrated envelope.
    `allow_direction_flip=False` treats any err-regression or angle-stuck as
    an abort condition (no direction flip). Use this for manual moves where
    direction is known from auto-cal — a stuck iteration then means the
    drive hit a physical end-stop and must not keep pushing in the wrong
    direction. Auto-cal leaves this True to auto-recover from a bad Phase 1
    probe.

    `stuck_tolerance` / `regression_tolerance` require N *consecutive*
    violations before aborting. Only meaningful when `allow_direction_flip`
    is False — auto-cal flips direction on the first bad iteration. For
    manual moves, settling noise and a tight threshold_deg relative to IMU
    noise can cause false-positive stuck/regression on a single iteration;
    tolerance > 1 lets the loop keep searching through transients.
    """

    min_response_deg = 0.2
    regression_threshold_deg = max(threshold_deg, motion_threshold_deg)
    spd_smoothing = 0.1  # trust measured spd quickly (see module docstring)
    spd_min = 1.0
    est_spd = max(spd_min, est_spd_init)

    prev_angle: float | None = None
    prev_abs_err: float | None = None
    prev_err_sign: int = 0
    prev_delta_steps: int = 0
    flipped = False
    stuck_streak = 0
    regression_streak = 0

    iterations = 0
    angle: float | None = None
    err: float | None = None

    for _ in range(max_iterations):
        angle = await read_angle(f"{log_prefix}:search")
        if angle is None:
            return NewtonResult(
                converged=False,
                iterations=iterations,
                final_angle=None,
                final_err=None,
                final_position=drive.current_position,
                error="imu_timeout",
                direction_sign=direction_sign,
                est_spd=est_spd,
            )

        err = angle - target_angle_deg
        err_sign = 1 if err > 0 else (-1 if err < 0 else 0)

        # ── Re-estimate est_spd from last substantive iteration ─────────
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
                    f"{log_prefix}.spd_update",
                    cam_id=cam_id,
                    prev_spd=round(est_spd, 3),
                    measured_spd=round(measured_spd, 3),
                    new_spd=round(new_spd, 3),
                    prev_delta_steps=prev_delta_steps,
                    angle_delta=round(angle - prev_angle, 3),
                )
            est_spd = new_spd

        # ── Crossover (overshoot — expected, not a direction fault) ─────
        crossed = (
            prev_err_sign != 0 and err_sign != 0 and err_sign != prev_err_sign
        )
        if crossed:
            logger.info(
                f"{log_prefix}.crossover",
                cam_id=cam_id,
                prev_abs_err=round(prev_abs_err, 3) if prev_abs_err else None,
                current_abs_err=round(abs(err), 3),
                prev_delta_steps=prev_delta_steps,
            )

        # ── Recovery: err-regression and angle-stuck ────────────────────
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
                regression_streak += 1
                stuck_streak = 0
                if allow_direction_flip and not flipped:
                    old_dir = direction_sign
                    direction_sign = -direction_sign
                    flipped = True
                    if on_direction_sign_change is not None:
                        on_direction_sign_change(direction_sign)
                    logger.warning(
                        f"{log_prefix}.direction_flip",
                        cam_id=cam_id,
                        reason="err_regression",
                        prev_abs_err=round(prev_abs_err, 3),
                        current_abs_err=round(abs(err), 3),
                        prev_delta_steps=prev_delta_steps,
                        angle_delta=round(angle_delta, 3),
                        old_direction_sign=old_dir,
                        new_direction_sign=direction_sign,
                    )
                elif (
                    not allow_direction_flip
                    and regression_streak < regression_tolerance
                ):
                    # Manual mode: tolerate transient regression (IMU noise or
                    # residual settling from the previous step). Keep searching.
                    logger.info(
                        f"{log_prefix}.err_regression_tolerated",
                        cam_id=cam_id,
                        streak=regression_streak,
                        tolerance=regression_tolerance,
                        prev_abs_err=round(prev_abs_err, 3),
                        current_abs_err=round(abs(err), 3),
                        prev_delta_steps=prev_delta_steps,
                        angle_delta=round(angle_delta, 3),
                    )
                else:
                    # Either flipped already or flips disabled and streak
                    # exceeded tolerance (persistent wrong-direction motion).
                    err_code = (
                        "both_directions_err_regression"
                        if allow_direction_flip
                        else "err_regression_no_flip"
                    )
                    logger.error(
                        f"{log_prefix}.abort_err_regression",
                        cam_id=cam_id,
                        allow_direction_flip=allow_direction_flip,
                        flipped=flipped,
                        regression_streak=regression_streak,
                        regression_tolerance=regression_tolerance,
                        current_position=drive.current_position,
                        start_pos=start_pos,
                        current_angle=round(angle, 3),
                        target_angle_deg=target_angle_deg,
                        current_abs_err=round(abs(err), 3),
                    )
                    await _move_and_settle(
                        drive, start_pos,
                        speed=settle_speed, settle_ms=settle_ms,
                    )
                    iterations += 1
                    return NewtonResult(
                        converged=False,
                        iterations=iterations,
                        final_angle=angle,
                        final_err=err,
                        final_position=drive.current_position,
                        error=err_code,
                        direction_sign=direction_sign,
                        est_spd=est_spd,
                    )
            elif angle_stuck:
                stuck_streak += 1
                regression_streak = 0
                logger.warning(
                    f"{log_prefix}.angle_stuck",
                    cam_id=cam_id,
                    allow_direction_flip=allow_direction_flip,
                    flipped=flipped,
                    stuck_streak=stuck_streak,
                    stuck_tolerance=stuck_tolerance,
                    prev_delta_steps=prev_delta_steps,
                    angle_delta=round(angle_delta, 3),
                    current_abs_err=round(abs(err), 3),
                    current_position=drive.current_position,
                )
                if allow_direction_flip and not flipped:
                    old_dir = direction_sign
                    direction_sign = -direction_sign
                    flipped = True
                    if on_direction_sign_change is not None:
                        on_direction_sign_change(direction_sign)
                    logger.warning(
                        f"{log_prefix}.direction_flip",
                        cam_id=cam_id,
                        reason="angle_stuck_end_stop",
                        old_direction_sign=old_dir,
                        new_direction_sign=direction_sign,
                        current_position=drive.current_position,
                        current_abs_err=round(abs(err), 3),
                    )
                    await _move_and_settle(
                        drive, start_pos,
                        speed=settle_speed, settle_ms=settle_ms,
                    )
                    iterations += 1
                    prev_angle = None
                    prev_abs_err = None
                    prev_err_sign = 0
                    prev_delta_steps = 0
                    stuck_streak = 0
                    regression_streak = 0
                    continue
                elif (
                    not allow_direction_flip
                    and stuck_streak < stuck_tolerance
                ):
                    # Manual mode: tolerate transient stuck (IMU still
                    # settling, small correction below noise floor). Keep
                    # searching — the main loop will take another Newton
                    # step below.
                    logger.info(
                        f"{log_prefix}.angle_stuck_tolerated",
                        cam_id=cam_id,
                        streak=stuck_streak,
                        tolerance=stuck_tolerance,
                        prev_delta_steps=prev_delta_steps,
                        angle_delta=round(angle_delta, 3),
                        current_abs_err=round(abs(err), 3),
                    )
                else:
                    # Flips disabled and streak exceeded tolerance (persistent
                    # no-response → physical end stop), OR already flipped
                    # once. Back off and abort; pressing further grinds the
                    # hardware.
                    err_code = (
                        "both_directions_angle_stuck"
                        if allow_direction_flip
                        else "end_stop_reached"
                    )
                    logger.error(
                        f"{log_prefix}.abort_end_stop",
                        cam_id=cam_id,
                        allow_direction_flip=allow_direction_flip,
                        flipped=flipped,
                        stuck_streak=stuck_streak,
                        stuck_tolerance=stuck_tolerance,
                        current_position=drive.current_position,
                        start_pos=start_pos,
                        travel_from_start=drive.current_position - start_pos,
                        current_angle=round(angle, 3),
                        target_angle_deg=target_angle_deg,
                        current_abs_err=round(abs(err), 3),
                    )
                    await _move_and_settle(
                        drive, start_pos,
                        speed=settle_speed, settle_ms=settle_ms,
                    )
                    iterations += 1
                    return NewtonResult(
                        converged=False,
                        iterations=iterations,
                        final_angle=angle,
                        final_err=err,
                        final_position=drive.current_position,
                        error=err_code,
                        direction_sign=direction_sign,
                        est_spd=est_spd,
                    )
            else:
                # Clean iteration — substantive move produced a
                # substantive, non-regressing response. Reset streaks.
                stuck_streak = 0
                regression_streak = 0

        # ── Convergence check ───────────────────────────────────────────
        if abs(err) <= threshold_deg:
            return NewtonResult(
                converged=True,
                iterations=iterations,
                final_angle=angle,
                final_err=err,
                final_position=drive.current_position,
                error=None,
                direction_sign=direction_sign,
                est_spd=est_spd,
            )

        # ── Newton step, clamped ────────────────────────────────────────
        delta_steps = int(-err * est_spd * direction_sign)
        if delta_steps > max_step_per_iter:
            delta_steps = max_step_per_iter
        elif delta_steps < -max_step_per_iter:
            delta_steps = -max_step_per_iter

        pre_pos = drive.current_position
        next_pos = pre_pos + delta_steps

        if safety_travel is not None and abs(next_pos - start_pos) > safety_travel:
            logger.error(
                f"{log_prefix}.abort_safety_travel",
                cam_id=cam_id,
                current_position=pre_pos,
                attempted_next=next_pos,
                start_pos=start_pos,
                safety_travel=safety_travel,
                current_angle=round(angle, 3),
                target_angle_deg=target_angle_deg,
                current_abs_err=round(abs(err), 3),
            )
            return NewtonResult(
                converged=False,
                iterations=iterations,
                final_angle=angle,
                final_err=err,
                final_position=pre_pos,
                error="safety_travel_exceeded",
                direction_sign=direction_sign,
                est_spd=est_spd,
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

    # Exhausted max_iterations
    return NewtonResult(
        converged=False,
        iterations=iterations,
        final_angle=angle,
        final_err=err,
        final_position=drive.current_position,
        error="max_iterations_exceeded",
        direction_sign=direction_sign,
        est_spd=est_spd,
    )
