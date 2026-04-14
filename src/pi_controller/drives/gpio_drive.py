"""GPIO stepper drive controller using lgpio.

Controls a linear motor via a single GPIO pin on the Raspberry Pi 5.
Supports single-pin mode (step_pin only, no dir_pin) for simple linear actuators,
or step+dir mode for stepper drivers.
Uses a blocking time.sleep() pulse loop in a thread — the same proven approach
that works with the motors directly.
No limit switches for now — position tracked in software, starts at 0 (bottom).
"""

from __future__ import annotations

import asyncio
import threading
import time

import structlog

from src.pi_controller.drives.base import BaseDrive, DriveState

logger = structlog.get_logger()

try:
    import lgpio
except ImportError:
    lgpio = None  # type: ignore[assignment]
    logger.warning("lgpio not available — GPIO drive will run in simulation mode")


# Pulse half-period: 10µs high + 10µs low = 20µs per step = 50kHz
DEFAULT_PULSE_DELAY = 0.00001


class GPIODrive(BaseDrive):
    """Stepper motor controlled via GPIO pin.

    Generates step pulses using a blocking for-loop with time.sleep()
    running in a separate thread, matching the proven working approach.
    """

    def __init__(
        self,
        cam_id: str,
        axis: str,
        *,
        step_pin: int,
        dir_pin: int | None = None,
        enable_pin: int | None = None,
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
        self.enable_pin = enable_pin
        self.steps_per_unit = steps_per_unit
        self.max_speed = max_speed
        self._max_position = max_position
        self._home_position = home_position
        self.invert_direction = invert_direction
        self.pulse_delay = pulse_delay

        self._gpio_handle: int | None = None
        self._stop_flag = threading.Event()
        self._async_stop = asyncio.Event()
        self._simulated = lgpio is None

    async def setup(self) -> None:
        if self._simulated:
            logger.info("gpio_drive.setup_simulated", key=self.key)
            return

        h = lgpio.gpiochip_open(0)
        self._gpio_handle = h

        lgpio.gpio_claim_output(h, self.step_pin, 0)
        if self.dir_pin is not None:
            lgpio.gpio_claim_output(h, self.dir_pin, 0)
        if self.enable_pin is not None:
            lgpio.gpio_claim_output(h, self.enable_pin, 1)  # disabled by default

        logger.info(
            "gpio_drive.setup",
            key=self.key,
            step_pin=self.step_pin,
            dir_pin=self.dir_pin,
            pulse_delay=self.pulse_delay,
        )

    async def cleanup(self) -> None:
        self.emergency_stop()
        if self._gpio_handle is not None:
            lgpio.gpio_write(self._gpio_handle, self.step_pin, 0)
            if self.enable_pin is not None:
                lgpio.gpio_write(self._gpio_handle, self.enable_pin, 1)
            lgpio.gpiochip_close(self._gpio_handle)
            self._gpio_handle = None
        logger.info("gpio_drive.cleanup", key=self.key)

    def _enable(self) -> None:
        if self._gpio_handle and self.enable_pin is not None:
            lgpio.gpio_write(self._gpio_handle, self.enable_pin, 0)

    def _disable(self) -> None:
        if self._gpio_handle and self.enable_pin is not None:
            lgpio.gpio_write(self._gpio_handle, self.enable_pin, 1)

    def _step_loop(self, steps: int, direction: int, delta: float) -> int:
        """Blocking step pulse loop — runs in a thread.

        This is the same pattern as the proven working script:
        for loop with lgpio.gpio_write HIGH, time.sleep, LOW, time.sleep.

        Returns the number of steps actually completed.
        """
        h = self._gpio_handle
        pin = self.step_pin
        delay = self.pulse_delay
        position_per_step = delta / steps
        start_pos = self._current_position

        for i in range(steps):
            if self._stop_flag.is_set():
                return i

            lgpio.gpio_write(h, pin, 1)
            time.sleep(delay)
            lgpio.gpio_write(h, pin, 0)
            time.sleep(delay)

            # Update position after each step
            self._current_position = start_pos + (i + 1) * position_per_step

        return steps

    async def move_to(self, position: float, speed: float = 1.0) -> None:
        async with self._move_lock:
            if not self.calibration_mode:
                position = max(self._home_position, min(self._max_position, position))

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
            if self.invert_direction:
                direction = -direction

            logger.info(
                "gpio_drive.move_start",
                key=self.key,
                target=position,
                steps=steps,
                pulse_delay=self.pulse_delay,
            )

            if self._simulated:
                await self._simulate_move(position, steps)
                return

            self._enable()

            # Set direction (only if dir_pin is configured)
            if self.dir_pin is not None:
                dir_val = 1 if direction > 0 else 0
                lgpio.gpio_write(self._gpio_handle, self.dir_pin, dir_val)

            try:
                # Run the blocking step loop in a thread so we don't
                # block the async event loop (GUI, MQTT, heartbeat).
                loop = asyncio.get_event_loop()
                steps_done = await loop.run_in_executor(
                    None, self._step_loop, steps, direction, delta
                )

                if self._stop_flag.is_set():
                    logger.warning(
                        "gpio_drive.stopped_mid_move",
                        key=self.key,
                        steps_done=steps_done,
                        steps_total=steps,
                    )
                    self._state = DriveState.IDLE
                    return

                # Snap to target to avoid float drift
                self._current_position = position
                self._state = DriveState.REACHED
                logger.info("gpio_drive.move_complete", key=self.key, position=position)

            except Exception as exc:
                self._state = DriveState.FAULT
                self._disable()
                logger.error("gpio_drive.move_error", key=self.key, error=str(exc))
                raise

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

    async def home(self) -> None:
        """Home to position 0 (bottom). No limit switches — software only."""
        self._state = DriveState.HOMING
        logger.info("gpio_drive.homing", key=self.key)

        if self._current_position != self._home_position:
            self._state = DriveState.HOMING
            await self.move_to(self._home_position, speed=0.3)

        self._current_position = self._home_position
        self._state = DriveState.IDLE
        logger.info("gpio_drive.homed", key=self.key)

    def emergency_stop(self) -> None:
        self._stop_flag.set()
        self._async_stop.set()
        if self._gpio_handle is not None and lgpio is not None:
            lgpio.gpio_write(self._gpio_handle, self.step_pin, 0)
        self._disable()
        self._target_position = None
        self._state = DriveState.IDLE
        logger.warning("gpio_drive.emergency_stop", key=self.key)

    def get_min_position(self) -> float:
        return self._home_position

    def get_max_position(self) -> float:
        return self._max_position

    def set_min_position(self, value: float) -> None:
        self._home_position = value

    def set_max_position(self, value: float) -> None:
        self._max_position = value
