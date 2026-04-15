"""GPIO stepper drive controller using gpiozero.

Controls a linear motor via PUL (pulse) + DIR (direction) GPIO pins on the
Raspberry Pi 5, matching the proven working script from the project reference.

Pin assignments (from drive_config.yaml):
  cam1:a  — PUL=GPIO 14, DIR=GPIO 15
  cam2:a  — PUL=GPIO 16, DIR=GPIO 17

Uses a blocking _time.sleep() pulse loop in a thread — the same approach as the
reference script that physically moves the motors.
"""

from __future__ import annotations

import asyncio
import threading
import time as _time

import structlog

from src.pi_controller.drives.base import BaseDrive, DriveState

logger = structlog.get_logger()

try:
    from gpiozero import Device, LED
    from gpiozero.exc import GPIOZeroError
    try:
        from gpiozero.pins.lgpio import LGPIOFactory
        Device.pin_factory = LGPIOFactory()
    except Exception:
        pass  # gpiozero will auto-select the correct factory for this platform
except ImportError:
    Device = None  # type: ignore[assignment,misc]
    LED = None  # type: ignore[assignment,misc]
    GPIOZeroError = Exception  # type: ignore[assignment,misc]
    logger.warning("gpiozero not available — GPIO drive will run in simulation mode")


# Pulse half-period: 10µs high + 10µs low = 20µs per step
DEFAULT_PULSE_DELAY = 0.00001


class GPIODrive(BaseDrive):
    """Linear motor controlled via PUL + DIR GPIO pins."""

    def __init__(
        self,
        cam_id: str,
        axis: str,
        *,
        step_pin: int,
        dir_pin: int,
        steps_per_unit: float = 200.0,
        max_speed: float = 1000.0,
        max_position: float = 100000.0,
        home_position: float = 0.0,
        invert_direction: bool = False,
        pulse_delay: float = DEFAULT_PULSE_DELAY,
        # ── Driver-safety timing ──────────────────────────────────────────
        # These prevent the physical stepper driver from triggering its
        # built-in over-speed / direction-reversal fault latch.
        dir_setup_delay: float = 0.010,
        # 10 ms between DIR pin change and first step pulse.
        # Datasheet minimum is typically 5 µs; 10 ms is a large safety margin.
        post_move_delay: float = 0.050,
        # 50 ms rest after every move finishes — lets the coil discharge.
        direction_change_delay: float = 0.200,
        # 200 ms extra gap when the new move direction is opposite to the last.
        min_move_interval: float = 0.050,
        # 50 ms minimum gap between any two consecutive moves (same direction).
    ) -> None:
        super().__init__(cam_id, axis)
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.steps_per_unit = steps_per_unit
        self.max_speed = max_speed
        self._max_position = max_position
        self._home_position = home_position
        self.invert_direction = invert_direction
        self.pulse_delay = pulse_delay

        self.dir_setup_delay = dir_setup_delay
        self.post_move_delay = post_move_delay
        self.direction_change_delay = direction_change_delay
        self.min_move_interval = min_move_interval

        self._pul: LED | None = None
        self._dir: LED | None = None
        self._stop_flag = threading.Event()
        self._async_stop = asyncio.Event()
        self._simulated = LED is None
        self._setup_error: str | None = None

        # Tracks whether the _step_loop thread is still running.
        # Prevents two threads from pulsing the same GPIO pin simultaneously.
        self._thread_done = threading.Event()
        self._thread_done.set()  # no thread running initially

        # Inter-move safety state
        self._last_direction: int | None = None
        self._last_move_end_ts: float = 0.0

    # ── Properties ────────────────────────────────────────────────────────

    @property
    def is_simulated(self) -> bool:
        return self._simulated

    @property
    def setup_error(self) -> str | None:
        return self._setup_error

    # ── Lifecycle ──────────────────────────────────────────────────────────

    async def setup(self) -> None:
        if self._simulated and self._setup_error is None:
            # gpiozero not importable on this platform
            logger.info("gpio_drive.setup_simulated", key=self.key)
            return

        # Close existing pins if re-initialising
        for attr in ("_pul", "_dir"):
            dev = getattr(self, attr, None)
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass
                setattr(self, attr, None)

        try:
            self._pul = LED(self.step_pin)
            self._dir = LED(self.dir_pin)
            self._simulated = False
            self._setup_error = None
        except Exception as exc:
            error_msg = f"{type(exc).__name__}: {exc}"
            self._setup_error = error_msg
            self._simulated = True
            logger.error(
                "gpio_drive.setup_failed",
                key=self.key,
                error=error_msg,
            )
            return

        pin_factory = type(Device.pin_factory).__name__ if Device is not None else "unknown"
        logger.info(
            "gpio_drive.setup_ok",
            key=self.key,
            step_pin=self.step_pin,
            dir_pin=self.dir_pin,
            pulse_delay=self.pulse_delay,
            pin_factory=pin_factory,
        )

    async def cleanup(self) -> None:
        self.emergency_stop()
        # Wait for the pulse thread to finish before closing GPIO objects
        if not self._thread_done.is_set():
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, self._thread_done.wait, 1.0)
        if self._pul is not None:
            self._pul.off()
            self._pul.close()
            self._pul = None
        if self._dir is not None:
            self._dir.off()
            self._dir.close()
            self._dir = None
        logger.info("gpio_drive.cleanup", key=self.key)

    # ── Step loop (runs in thread) ─────────────────────────────────────────

    def _step_loop(self, steps: int, direction: int, delta: float) -> int:
        """Blocking pulse loop — identical to the working reference script.

        dir.on/off → for loop: pul.on, sleep, pul.off, sleep

        Always sets _thread_done in finally so move_to can serialise threads.
        """
        try:
            pul = self._pul
            dir_dev = self._dir
            delay = self.pulse_delay
            position_per_step = delta / steps
            start_pos = self._current_position

            if pul is None:
                logger.error("gpio_drive._step_loop: pul is None", key=self.key)
                return 0

            # Set direction once before pulsing — same as reference script.
            # Sleep dir_setup_delay afterwards so the driver has time to latch
            # the new direction before the first step pulse arrives.
            forward = direction > 0
            if self.invert_direction:
                forward = not forward
            if dir_dev is not None:
                if forward:
                    dir_dev.on()
                else:
                    dir_dev.off()
            _time.sleep(self.dir_setup_delay)

            for i in range(steps):
                if self._stop_flag.is_set():
                    return i

                pul.on()
                _time.sleep(delay)
                pul.off()
                _time.sleep(delay)

                self._current_position = start_pos + (i + 1) * position_per_step

            # Post-move rest — lets the driver coil discharge before the next
            # command.  Skipped if the stop flag was set (emergency stop).
            if not self._stop_flag.is_set():
                _time.sleep(self.post_move_delay)

            return steps

        finally:
            # Always signal completion so the next move_to doesn't start a
            # second thread while this one is still pulsing.
            self._thread_done.set()

    # ── Move logic ─────────────────────────────────────────────────────────

    async def move_to(self, position: float, speed: float = 1.0) -> None:
        async with self._move_lock:
            # Hard lower-bound — ALWAYS enforced, even in calibration mode.
            # The iHSS60 drives fault-latch at their physical lowest endpoint
            # (where gravity deposits them when unpowered).  The startup
            # position is auto-seeded as the minimum; going below it is not
            # permitted under any circumstance to keep the drive out of the
            # fault zone.
            position = max(self._home_position, position)
            # Upper bound only clamped outside calibration mode, so the user
            # can still jog up during calibration to discover the max.
            if not self.calibration_mode:
                position = min(self._max_position, position)

            self._target_position = position
            self._state = DriveState.MOVING
            self._stop_flag.clear()
            self._async_stop.clear()

            delta = position - self._current_position
            if abs(delta) < 1e-6:
                self._state = DriveState.REACHED
                return

            steps = int(abs(delta) * self.steps_per_unit)
            if steps == 0:
                self._current_position = position
                self._state = DriveState.REACHED
                return

            direction = 1 if delta > 0 else -1

            logger.info(
                "gpio_drive.move_start",
                key=self.key,
                current=self._current_position,
                target=position,
                steps=steps,
                direction=direction,
            )

            if self._simulated:
                await self._simulate_move(position, steps)
                return

            loop = asyncio.get_running_loop()

            # ── Inter-move safety gap ─────────────────────────────────────
            # Enforce a minimum delay between moves and a longer delay when
            # the direction reverses, to prevent driver fault-latch.
            if self._last_direction is not None:
                needed = (
                    self.direction_change_delay
                    if direction != self._last_direction
                    else self.min_move_interval
                )
                elapsed = _time.monotonic() - self._last_move_end_ts
                gap = needed - elapsed
                if gap > 0:
                    await asyncio.sleep(gap)

            # Wait for any previous thread to exit before starting a new one.
            # Without this, rapid jogs or cancelled moves leave a ghost thread
            # pulsing the same pin — causing the motor to stall or behave
            # erratically.  The thread always sets _thread_done in its finally.
            if not self._thread_done.is_set():
                logger.debug("gpio_drive.waiting_for_prev_thread", key=self.key)
                self._stop_flag.set()   # ask old thread to exit early
                await loop.run_in_executor(None, self._thread_done.wait, 2.0)
                self._stop_flag.clear()  # ready for the new thread

            self._thread_done.clear()  # mark new thread as in-flight

            try:
                steps_done = await loop.run_in_executor(
                    None, self._step_loop, steps, direction, delta
                )
            except asyncio.CancelledError:
                # NiceGUI cancels button handlers on disconnect / tab navigation.
                # Signal the thread to stop, but do NOT re-raise — re-raising
                # propagates the cancellation to NiceGUI and leaves the drive
                # in a broken state for subsequent jog presses.
                self._stop_flag.set()
                self._state = DriveState.IDLE
                logger.warning("gpio_drive.move_cancelled", key=self.key)
                return
            except Exception as exc:
                self._stop_flag.set()
                self._state = DriveState.FAULT
                logger.error("gpio_drive.move_error", key=self.key, error=str(exc))
                raise

            if self._stop_flag.is_set():
                logger.warning(
                    "gpio_drive.stopped_mid_move",
                    key=self.key,
                    steps_done=steps_done,
                    steps_total=steps,
                )
                self._last_move_end_ts = _time.monotonic()
                self._state = DriveState.IDLE
                return

            self._current_position = position
            self._last_direction = direction
            self._last_move_end_ts = _time.monotonic()
            self._state = DriveState.REACHED
            logger.info("gpio_drive.move_complete", key=self.key, position=position)

    # ── Simulated movement ─────────────────────────────────────────────────

    async def _simulate_move(self, target: float, steps: int) -> None:
        total_time = steps * self.pulse_delay * 2
        sim_time = min(total_time, 3.0)
        elapsed = 0.0
        start_pos = self._current_position
        delta = target - start_pos

        while elapsed < sim_time:
            if self._async_stop.is_set():
                self._state = DriveState.IDLE
                return
            await asyncio.sleep(0.05)
            elapsed += 0.05
            self._current_position = start_pos + delta * min(elapsed / sim_time, 1.0)

        self._current_position = target
        self._state = DriveState.REACHED
        logger.info("gpio_drive.simulated_move_complete", key=self.key, position=target)

    # ── Homing ─────────────────────────────────────────────────────────────

    async def home(self) -> None:
        self._state = DriveState.HOMING
        logger.info("gpio_drive.homing", key=self.key)
        if self._current_position != self._home_position:
            await self.move_to(self._home_position, speed=0.3)
        self._current_position = self._home_position
        self._state = DriveState.IDLE
        logger.info("gpio_drive.homed", key=self.key)

    # ── Emergency stop ─────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        self._stop_flag.set()
        self._async_stop.set()
        if self._pul is not None:
            self._pul.off()
        self._target_position = None
        self._state = DriveState.IDLE
        logger.warning("gpio_drive.emergency_stop", key=self.key)

    # ── Calibration helpers ────────────────────────────────────────────────

    def get_min_position(self) -> float:
        return self._home_position

    def get_max_position(self) -> float:
        return self._max_position

    def set_min_position(self, value: float) -> None:
        self._home_position = value

    def set_max_position(self, value: float) -> None:
        self._max_position = value
