"""GPIO stepper drive controller using gpiozero.

Controls a linear motor via PUL (pulse) + DIR (direction) GPIO pins on the
Raspberry Pi 5, matching the proven working script from the project reference.

Pin assignments (from drive_config.yaml):
  cam1:a  — PUL=GPIO 14, DIR=GPIO 15
  cam2:a  — PUL=GPIO 16, DIR=GPIO 17

Uses a blocking time.sleep() pulse loop in a thread — the same approach as the
reference script that physically moves the motors.
"""

from __future__ import annotations

import asyncio
import threading
import time

import structlog

from src.pi_controller.drives.base import BaseDrive, DriveState

logger = structlog.get_logger()

try:
    from gpiozero import Device, LED
    from gpiozero.exc import GPIOZeroError
    # On Pi 5, gpiozero 2.x auto-detects lgpio as the default factory because
    # RPi.GPIO does not support Pi 5. Forcing LGPIOFactory() explicitly can
    # crash the import if the GPIO chip is unavailable at import time, causing
    # all drives to silently fall into simulation mode.
    # We still try to set lgpio explicitly for clarity, but any failure is
    # ignored — gpiozero's auto-detection handles it correctly on Pi 5.
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


# Pulse half-period matching the proven working reference script:
# 10µs high + 10µs low = 20µs per step
DEFAULT_PULSE_DELAY = 0.00001

# Log a heartbeat every N steps so the Pi log shows real motion progress.
_PROGRESS_LOG_INTERVAL = 10_000


class GPIODrive(BaseDrive):
    """Linear motor controlled via PUL + DIR GPIO pins.

    Generates step pulses using a blocking for-loop with time.sleep()
    running in a separate thread, identical to the working reference script.
    """

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

        self._pul: LED | None = None
        self._dir_pin_dev: LED | None = None
        self._stop_flag = threading.Event()
        self._async_stop = asyncio.Event()
        self._simulated = LED is None
        self._setup_error: str | None = None

        # _thread_done is SET when no thread is running (or the last thread
        # has finished).  move_to waits on it before starting a new thread,
        # which prevents two _step_loop threads from pulsing simultaneously.
        self._thread_done = threading.Event()
        self._thread_done.set()

    # ── Public properties ──────────────────────────────────────────────────

    @property
    def is_simulated(self) -> bool:
        return self._simulated

    @property
    def setup_error(self) -> str | None:
        """Human-readable reason why this drive is in simulation mode, or None."""
        return self._setup_error

    # ── Lifecycle ──────────────────────────────────────────────────────────

    async def setup(self) -> None:
        if self._simulated and self._setup_error is None:
            # gpiozero not importable — nothing to retry
            logger.info("gpio_drive.setup_simulated", key=self.key)
            return

        # Close any existing pins before (re-)initialising
        if self._pul is not None:
            try:
                self._pul.close()
            except Exception:
                pass
            self._pul = None
        if self._dir_pin_dev is not None:
            try:
                self._dir_pin_dev.close()
            except Exception:
                pass
            self._dir_pin_dev = None

        try:
            self._pul = LED(self.step_pin)
            self._dir_pin_dev = LED(self.dir_pin)
            self._simulated = False
            self._setup_error = None
        except Exception as exc:
            error_msg = f"{type(exc).__name__}: {exc}"
            self._setup_error = error_msg
            self._simulated = True
            logger.error(
                "gpio_drive.setup_failed — falling back to SIMULATION",
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
        # Wait for any running thread to finish before closing the GPIO objects.
        if not self._thread_done.is_set():
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, self._thread_done.wait, 1.0)
        if self._pul is not None:
            self._pul.off()
            self._pul.close()
            self._pul = None
        if self._dir_pin_dev is not None:
            self._dir_pin_dev.off()
            self._dir_pin_dev.close()
            self._dir_pin_dev = None
        logger.info("gpio_drive.cleanup", key=self.key)

    # ── Step loop (runs in thread) ─────────────────────────────────────────

    def _step_loop(self, steps: int, direction: int, delta: float) -> int:
        """Blocking step pulse loop — runs in a thread via run_in_executor.

        Returns the number of steps actually completed.
        Always sets _thread_done on exit so move_to can serialise threads.
        """
        try:
            pul = self._pul
            dir_dev = self._dir_pin_dev
            delay = self.pulse_delay
            position_per_step = delta / steps
            start_pos = self._current_position

            if pul is None:
                logger.error("gpio_drive._step_loop called but pul is None", key=self.key)
                return 0

            # Set direction before pulsing
            forward = direction > 0
            if self.invert_direction:
                forward = not forward
            if dir_dev is not None:
                if forward:
                    dir_dev.on()
                else:
                    dir_dev.off()

            for i in range(steps):
                if self._stop_flag.is_set():
                    logger.info(
                        "gpio_drive._step_loop.stopped",
                        key=self.key,
                        steps_done=i,
                        steps_total=steps,
                    )
                    return i

                pul.on()
                time.sleep(delay)
                pul.off()
                time.sleep(delay)

                self._current_position = start_pos + (i + 1) * position_per_step

                # Periodic heartbeat so the Pi log shows real motion
                if (i + 1) % _PROGRESS_LOG_INTERVAL == 0:
                    logger.debug(
                        "gpio_drive._step_loop.progress",
                        key=self.key,
                        steps_done=i + 1,
                        steps_total=steps,
                        position=self._current_position,
                    )

            return steps

        finally:
            # Always signal that this thread has exited — even on exception.
            self._thread_done.set()

    # ── Move logic ─────────────────────────────────────────────────────────

    async def move_to(self, position: float, speed: float = 1.0) -> None:
        async with self._move_lock:
            if not self.calibration_mode:
                position = max(self._home_position, min(self._max_position, position))

            self._target_position = position

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
                self._state = DriveState.MOVING
                await self._simulate_move(position, steps)
                return

            # ── Ensure the previous thread has exited before pulsing again ──
            # This prevents two threads from toggling the same GPIO pin
            # simultaneously, which caused erratic behaviour.
            loop = asyncio.get_running_loop()
            if not self._thread_done.is_set():
                logger.debug("gpio_drive.waiting_for_previous_thread", key=self.key)
                self._stop_flag.set()  # ask old thread to stop
                await loop.run_in_executor(None, self._thread_done.wait, 1.0)
                if not self._thread_done.is_set():
                    logger.error("gpio_drive.previous_thread_timeout", key=self.key)

            self._thread_done.clear()   # this thread is starting
            self._stop_flag.clear()
            self._async_stop.clear()
            self._state = DriveState.MOVING

            try:
                steps_done = await loop.run_in_executor(
                    None, self._step_loop, steps, direction, delta
                )
            except asyncio.CancelledError:
                # NiceGUI may cancel button-handler tasks on disconnect / navigation.
                # Signal the thread to stop, but do NOT re-raise — re-raising propagates
                # the cancel up to NiceGUI which can put the drive in an unrecoverable
                # state from the GUI's perspective.  Just treat it as a soft stop.
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
                self._state = DriveState.IDLE
                return

            self._current_position = position
            self._state = DriveState.REACHED
            logger.info("gpio_drive.move_complete", key=self.key, position=position)

    # ── Simulated movement ─────────────────────────────────────────────────

    async def _simulate_move(self, target: float, steps: int) -> None:
        """Simulated movement for development without GPIO hardware."""
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
            progress = min(elapsed / sim_time, 1.0)
            self._current_position = start_pos + delta * progress

        self._current_position = target
        self._state = DriveState.REACHED
        logger.info("gpio_drive.simulated_move_complete", key=self.key, position=target)

    # ── Homing ─────────────────────────────────────────────────────────────

    async def home(self) -> None:
        """Home to position 0. No limit switches — software only."""
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
