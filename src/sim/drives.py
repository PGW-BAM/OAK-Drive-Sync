"""Simulated drives: drop-in ``BaseDrive`` replacements that move the ``RigModel``.

``SimLinearDrive`` (axis a) mirrors ``GPIODrive`` position semantics (position in *units*,
steps = units x steps_per_unit); ``SimRadialDrive`` (axis b) mirrors ``TinkerforgeDrive``
(position = raw steps). Both write the *physical* position (motor steps) into the shared
``RigModel`` so pose / simulated IMU follow the motion.

Counter vs. physical position
-----------------------------
The drive's ``current_position`` is the controller's *step counter* (what the real drive
reports). The physical position lives in ``_phys`` (steps) and in the RigModel. They only
diverge through injected step loss / backlash, or when the counter is overwritten without
motion (``set_current_position`` / ``seed_position`` / ``set_zero``) -- exactly like on the
real hardware, which is what the closed-loop angle logic has to cope with.

Virtual limit switches (``AxisConfig.switch_min_pos`` / ``switch_max_pos``) act on the
*physical* position: the move stops at the switch, state becomes FAULT and
``triggered_switch`` is ``"min"`` / ``"max"`` (physical unit sense, independent of
``direction_sign``). A following ``move_to`` clears the fault; moving away from the switch
works, pushing further into it faults again immediately.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import time
from typing import TYPE_CHECKING, Any

import structlog

from src.pi_controller.drives.base import BaseDrive, DriveState
from src.sim import kinematics as kin
from src.sim.rig_model import RigModel

if TYPE_CHECKING:
    from src.sim.rig_config import AxisConfig

logger = structlog.get_logger()

# --- shared rig (create_drive() builds drives from plain config dicts) --------------------------

_shared_rig: RigModel | None = None


def get_shared_rig(config_path: str | None = None) -> RigModel:
    """Process-wide RigModel used by all sim drives and the SimImuResponder.

    Created lazily from ``config/sim_rig.yaml`` (or ``config_path`` on first call).
    """
    global _shared_rig
    if _shared_rig is None:
        _shared_rig = RigModel.from_yaml(config_path)
    return _shared_rig


def set_shared_rig(rig: RigModel | None) -> None:
    """Install (or with ``None`` reset) the shared rig, e.g. in tests."""
    global _shared_rig
    _shared_rig = rig


# --- motion profile ------------------------------------------------------------------------------


class MotionProfile:
    """Distance-vs-time for one move: constant velocity, or trapezoid/triangle with ramps."""

    def __init__(
        self,
        distance: float,
        velocity: float,
        accel: float | None = None,
        decel: float | None = None,
    ) -> None:
        self.distance_total = distance
        v = max(velocity, 1e-9)
        self._v = v
        self._a = accel if accel and accel > 0 else None
        self._d = (decel if decel and decel > 0 else self._a) if self._a else None
        if self._a is None or self._d is None:
            self._vp, self._ta, self._tc, self._td = v, 0.0, distance / v, 0.0
            self._da = 0.0
        else:
            da, dd = v * v / (2 * self._a), v * v / (2 * self._d)
            if da + dd <= distance:
                vp, tc = v, (distance - da - dd) / v
            else:  # triangle: never reaches max speed
                vp = math.sqrt(2 * distance * self._a * self._d / (self._a + self._d))
                tc = 0.0
            self._vp, self._tc = vp, tc
            self._ta, self._td = vp / self._a, vp / self._d
            self._da = vp * vp / (2 * self._a)
        self.duration = self._ta + self._tc + self._td

    def distance(self, t: float) -> float:
        if t <= 0:
            return 0.0
        if t >= self.duration:
            return self.distance_total
        if self._a is None or self._d is None:
            return self._v * t
        if t <= self._ta:
            return 0.5 * self._a * t * t
        if t <= self._ta + self._tc:
            return self._da + self._vp * (t - self._ta)
        tt = t - self._ta - self._tc
        return self._da + self._vp * self._tc + self._vp * tt - 0.5 * self._d * tt * tt


# --- common base ---------------------------------------------------------------------------------


class _SimAxisDrive(BaseDrive):
    """Shared sim mechanics; subclasses define unit semantics, clamping and homing."""

    #: physical motor steps per native position unit (linear: steps_per_unit, radial: 1)
    _unit_steps: float = 1.0

    def __init__(
        self,
        cam_id: str,
        axis: str,
        *,
        rig: RigModel | None,
        max_speed: float,
        acceleration: float | None,
        deceleration: float | None,
        update_interval_s: float,
        step_loss_fraction: float,
        backlash_steps: float,
        post_move_delay: float,
        dir_setup_delay: float,
    ) -> None:
        super().__init__(cam_id, axis)
        if axis not in ("a", "b"):
            raise ValueError(f"axis must be 'a' or 'b', got {axis!r}")
        if not 0.0 <= step_loss_fraction < 1.0:
            raise ValueError("step_loss_fraction must be in [0, 1)")
        self.rig = rig if rig is not None else get_shared_rig()
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.update_interval_s = update_interval_s
        self.step_loss_fraction = step_loss_fraction
        self.backlash_steps = backlash_steps
        self.post_move_delay = post_move_delay
        self.dir_setup_delay = dir_setup_delay

        self.triggered_switch: str | None = None
        self._simulated = False  # functional simulation (closed loop works), not a GPIO fallback
        self._stop_event = asyncio.Event()
        self._stop_at: float = 0.0
        self._last_dir: int = 0
        self._slack: float = 0.0

        self._axis_cfg: AxisConfig = (
            self.rig.cam(cam_id).axis_a if axis == "a" else self.rig.cam(cam_id).axis_b
        )
        self._phys: float = float(self.rig.get_steps(cam_id, axis))  # type: ignore[arg-type]
        self._current_position = self._phys / self._unit_steps
        # Virtual switches + soft envelope in motor steps, ordered (low, high)
        sw = sorted(
            [
                (float(kin.units_to_steps(self._axis_cfg, self._axis_cfg.switch_min_pos)), "min"),
                (float(kin.units_to_steps(self._axis_cfg, self._axis_cfg.switch_max_pos)), "max"),
            ]
        )
        self._sw_lo, self._sw_lo_name = sw[0]
        self._sw_hi, self._sw_hi_name = sw[1]
        soft = sorted(
            kin.units_to_steps(self._axis_cfg, u)
            for u in (self._axis_cfg.soft_min, self._axis_cfg.soft_max)
        )
        self._soft_lo_steps, self._soft_hi_steps = float(soft[0]), float(soft[1])

    # -- hooks for subclasses --------------------------------------------------------------------

    def _clamp_target(self, position: float) -> float:
        raise NotImplementedError

    def _plan_steps(self, delta: float) -> float | None:
        """Commanded step count for a counter delta, or None if the move is a no-op."""
        raise NotImplementedError

    # -- lifecycle -------------------------------------------------------------------------------

    async def setup(self) -> None:
        self._phys = float(self.rig.get_steps(self.cam_id, self.axis))  # type: ignore[arg-type]
        logger.info("sim_drive.setup", key=self.key, phys_steps=self._phys)

    async def cleanup(self) -> None:
        self.emergency_stop()

    def emergency_stop(self) -> None:
        self._stop_at = time.monotonic()
        self._stop_event.set()
        self._target_position = None
        self._state = DriveState.IDLE
        logger.warning("sim_drive.emergency_stop", key=self.key)

    def clear_fault(self) -> None:
        self.triggered_switch = None
        if self._state == DriveState.FAULT:
            self._state = DriveState.IDLE

    @property
    def physical_steps(self) -> float:
        """True mechanical position in motor steps (counter may differ after step loss)."""
        return self._phys

    # -- physics ---------------------------------------------------------------------------------

    def _apply_motion(self, d_cmd: float, direction: int) -> tuple[float, str | None]:
        """Advance the physical position by ``d_cmd`` commanded steps.

        Returns (fraction of ``d_cmd`` that was consumed, triggered switch name or None).
        """
        if d_cmd <= 0.0:
            return 1.0, None
        if self.backlash_steps > 0 and self._last_dir not in (0, direction):
            self._slack = self.backlash_steps  # play must be taken up again after a reversal
        self._last_dir = direction
        slack_eat = min(self._slack, d_cmd)
        eff = 1.0 - self.step_loss_fraction
        d_eff = (d_cmd - slack_eat) * eff
        room = (self._sw_hi - self._phys) if direction > 0 else (self._phys - self._sw_lo)
        if d_eff > 0.0 and d_eff >= room:
            room = max(room, 0.0)
            self._slack -= slack_eat
            self._phys += direction * room
            name = self._sw_hi_name if direction > 0 else self._sw_lo_name
            used = slack_eat + room / eff
            return min(1.0, used / d_cmd), name
        self._slack -= slack_eat
        self._phys += direction * d_eff
        return 1.0, None

    def _publish_to_rig(self) -> None:
        self.rig.set_steps(self.cam_id, self.axis, round(self._phys))  # type: ignore[arg-type]

    # -- move ------------------------------------------------------------------------------------

    async def move_to(self, position: float, speed: float = 1.0) -> None:
        async with self._move_lock:
            target = self._clamp_target(position)
            self._target_position = target
            self._state = DriveState.MOVING
            self._stop_event.clear()
            self.triggered_switch = None

            start = self._current_position
            delta = target - start
            n_steps = self._plan_steps(delta)
            if n_steps is None:
                self._current_position = target
                self._state = DriveState.REACHED
                return

            # Pick up external rig changes (e.g. GUI teleport) so physics stay consistent
            rig_steps = self.rig.get_steps(self.cam_id, self.axis)  # type: ignore[arg-type]
            if rig_steps != round(self._phys):
                self._phys = float(rig_steps)

            speed = max(0.01, min(1.0, speed))
            profile = MotionProfile(
                n_steps, self.max_speed * speed, self.acceleration, self.deceleration
            )
            direction = 1 if delta > 0 else -1
            logger.info(
                "sim_drive.move_start", key=self.key, current=start, target=target, steps=n_steps
            )
            t0 = time.monotonic()
            lead = self.dir_setup_delay
            done = 0.0  # commanded steps consumed so far

            try:
                while True:
                    stopped = self._stop_event.is_set()
                    now = self._stop_at if stopped else time.monotonic()
                    dist = profile.distance(now - t0 - lead)
                    frac_step, switch = self._apply_motion(dist - done, direction)
                    done += (dist - done) * frac_step
                    self._publish_to_rig()
                    self._current_position = start + delta * (done / n_steps)

                    if switch is not None:
                        self.triggered_switch = switch
                        self._state = DriveState.FAULT
                        logger.warning("sim_drive.switch_triggered", key=self.key, switch=switch)
                        return
                    if stopped:
                        logger.warning("sim_drive.stopped_mid_move", key=self.key, done=done)
                        return  # emergency_stop() already set state IDLE
                    if dist >= n_steps:
                        break
                    remaining = lead + profile.duration - (time.monotonic() - t0)
                    await self._sleep_or_stop(min(self.update_interval_s, max(remaining, 0.0)))

                self._current_position = target
                if self.post_move_delay > 0:
                    await self._sleep_or_stop(self.post_move_delay)
                if self._stop_event.is_set():
                    return
                self._state = DriveState.REACHED
                logger.info("sim_drive.move_complete", key=self.key, position=target)
            except asyncio.CancelledError:
                self._stop_event.set()
                self._stop_at = time.monotonic()
                self._state = DriveState.IDLE
                raise

    async def _sleep_or_stop(self, seconds: float) -> None:
        with contextlib.suppress(TimeoutError):
            await asyncio.wait_for(self._stop_event.wait(), timeout=seconds)


# --- axis a --------------------------------------------------------------------------------------


class SimLinearDrive(_SimAxisDrive):
    """Simulated GPIO linear drive. Position in units; steps = units x ``steps_per_unit``.

    Same bounds semantics as ``GPIODrive``: lower bound (``home_position``) is ALWAYS enforced,
    upper bound (``max_position``) only outside ``calibration_mode``. Defaults: the rig's
    soft limits for this axis. Unused real-hardware keys (``step_pin``, ``dir_pin``,
    ``pulse_delay``, ``invert_direction``, ``min_move_interval``, ``direction_change_delay``)
    are accepted and ignored so a real config block works with only ``type`` changed.
    """

    def __init__(
        self,
        cam_id: str,
        axis: str = "a",
        *,
        rig: RigModel | None = None,
        steps_per_unit: float | None = None,
        max_speed: float = 1000.0,
        acceleration: float | None = None,
        deceleration: float | None = None,
        max_position: float | None = None,
        home_position: float | None = None,
        update_interval_s: float = 0.02,
        step_loss_fraction: float = 0.0,
        backlash_steps: float = 0.0,
        post_move_delay: float = 0.0,
        dir_setup_delay: float = 0.0,
        **hw_params: Any,
    ) -> None:
        rig = rig if rig is not None else get_shared_rig()
        spu = steps_per_unit
        if spu is None:
            cam = rig.cam(cam_id)
            spu = (cam.axis_a if axis == "a" else cam.axis_b).steps_per_unit
        self.steps_per_unit = float(spu)
        self._unit_steps = self.steps_per_unit
        super().__init__(
            cam_id,
            axis,
            rig=rig,
            max_speed=max_speed,
            acceleration=acceleration,
            deceleration=deceleration,
            update_interval_s=update_interval_s,
            step_loss_fraction=step_loss_fraction,
            backlash_steps=backlash_steps,
            post_move_delay=post_move_delay,
            dir_setup_delay=dir_setup_delay,
        )
        self._home_position = (
            home_position if home_position is not None else self._soft_lo_steps / self._unit_steps
        )
        self._max_position = (
            max_position if max_position is not None else self._soft_hi_steps / self._unit_steps
        )
        if hw_params:
            logger.debug("sim_linear.ignored_params", key=self.key, params=sorted(hw_params))

    def _clamp_target(self, position: float) -> float:
        position = max(self._home_position, position)
        if not self.calibration_mode:
            position = min(self._max_position, position)
        return position

    def _plan_steps(self, delta: float) -> float | None:
        if abs(delta) < 1e-6:
            return None
        steps = int(abs(delta) * self.steps_per_unit)
        return float(steps) if steps > 0 else None

    async def home(self) -> None:
        self._state = DriveState.HOMING
        if self._current_position != self._home_position:
            await self.move_to(self._home_position, speed=0.3)
        self._current_position = self._home_position
        if self._state != DriveState.FAULT:
            self._state = DriveState.IDLE

    def get_min_position(self) -> float:
        return self._home_position

    def get_max_position(self) -> float:
        return self._max_position

    def set_min_position(self, value: float) -> None:
        self._home_position = value

    def set_max_position(self, value: float) -> None:
        self._max_position = value


# --- axis b --------------------------------------------------------------------------------------


class SimRadialDrive(_SimAxisDrive):
    """Simulated Tinkerforge tilt drive. Position = raw motor steps.

    Same bounds semantics as ``TinkerforgeDrive``: clamped to [min, max] unless
    ``calibration_mode`` (then unclamped). Defaults: the rig's soft limits for this axis.
    ``max_velocity`` (steps/s, x ``speed``), ``acceleration`` / ``deceleration`` (steps/s^2)
    shape the move. Other real-hardware keys (``uid``, ``host``, ``port``,
    ``step_resolution``) are accepted and ignored.
    """

    def __init__(
        self,
        cam_id: str,
        axis: str = "b",
        *,
        rig: RigModel | None = None,
        min_position: int | None = None,
        max_position: int | None = None,
        max_velocity: float = 2000.0,
        acceleration: float | None = 1000.0,
        deceleration: float | None = 1000.0,
        motor_current: int = 1200,
        update_interval_s: float = 0.02,
        step_loss_fraction: float = 0.0,
        backlash_steps: float = 0.0,
        **hw_params: Any,
    ) -> None:
        super().__init__(
            cam_id,
            axis,
            rig=rig,
            max_speed=max_velocity,
            acceleration=acceleration,
            deceleration=deceleration,
            update_interval_s=update_interval_s,
            step_loss_fraction=step_loss_fraction,
            backlash_steps=backlash_steps,
            post_move_delay=0.0,
            dir_setup_delay=0.0,
        )
        self.max_velocity = max_velocity
        self.motor_current = motor_current
        self._min_position = int(min_position if min_position is not None else self._soft_lo_steps)
        self._max_position = int(max_position if max_position is not None else self._soft_hi_steps)
        if hw_params:
            logger.debug("sim_radial.ignored_params", key=self.key, params=sorted(hw_params))

    @property
    def supports_angle_feedback(self) -> bool:
        return True

    def _clamp_target(self, position: float) -> float:
        if self.calibration_mode:
            return float(int(position))
        return float(int(max(self._min_position, min(self._max_position, position))))

    def _plan_steps(self, delta: float) -> float | None:
        return None if abs(delta) < 1 else abs(delta)

    async def home(self) -> None:
        self._state = DriveState.HOMING
        home_pos = float(self._min_position) if self.min_calibrated else 0.0
        await self.move_to(home_pos, speed=0.5)
        self._current_position = home_pos
        if self._state != DriveState.FAULT:
            self._state = DriveState.IDLE

    def get_min_position(self) -> float:
        return float(self._min_position)

    def get_max_position(self) -> float:
        return float(self._max_position)

    def set_min_position(self, value: float) -> None:
        self._min_position = int(value)

    def set_max_position(self, value: float) -> None:
        self._max_position = int(value)

    def set_current_position(self, value: float) -> None:
        """Counter re-anchor only -- the physical position (rig) is untouched."""
        if self._state not in (DriveState.IDLE, DriveState.REACHED):
            raise RuntimeError(
                f"set_current_position refused: drive {self.key} state={self._state}"
            )
        self._current_position = float(int(value))
        self._target_position = self._current_position

    def get_motor_current_mA(self) -> int | None:  # noqa: N802 (matches TinkerforgeDrive)
        return self.motor_current

    def set_motor_current_mA(self, value: int) -> None:  # noqa: N802
        self.motor_current = int(value)
