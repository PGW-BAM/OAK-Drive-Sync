"""Tinkerforge Silent Stepper Bricklet 2.0 drive controller.

Uses the Tinkerforge Python bindings to communicate with the bricklet
via Brick Daemon (brickd), typically running on the Pi over USB.
"""

from __future__ import annotations

import asyncio

import structlog

from src.pi_controller.drives.base import BaseDrive, DriveState

logger = structlog.get_logger()

try:
    from tinkerforge.ip_connection import IPConnection
    from tinkerforge.bricklet_silent_stepper_v2 import BrickletSilentStepperV2
except ImportError:
    IPConnection = None  # type: ignore[assignment,misc]
    BrickletSilentStepperV2 = None  # type: ignore[assignment,misc]
    logger.warning(
        "tinkerforge not available — Tinkerforge drive will run in simulation mode"
    )


class TinkerforgeDrive(BaseDrive):
    """Drive controlled via Tinkerforge Silent Stepper Bricklet 2.0."""

    def __init__(
        self,
        cam_id: str,
        axis: str,
        *,
        uid: str,
        host: str = "localhost",
        port: int = 4223,
        min_position: int = 1500,
        max_position: int = 9500,
        max_velocity: int = 2000,
        acceleration: int = 1000,
        deceleration: int = 1000,
        step_resolution: int = 8,  # microstepping: 1,2,4,8,16,32,64,128,256
    ) -> None:
        super().__init__(cam_id, axis)
        self.uid = uid
        self.host = host
        self.port = port
        self._min_position = min_position
        self._max_position = max_position
        self.max_velocity = max_velocity
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.step_resolution = step_resolution

        self._ipcon: IPConnection | None = None
        self._stepper: BrickletSilentStepperV2 | None = None
        self._stop_event = asyncio.Event()
        self._simulated = BrickletSilentStepperV2 is None

        # Start at min position (bottom)
        self._current_position = float(min_position)

    async def setup(self) -> None:
        if self._simulated:
            logger.info("tinkerforge_drive.setup_simulated", key=self.key, uid=self.uid)
            return

        self._ipcon = IPConnection()
        self._stepper = BrickletSilentStepperV2(self.uid, self._ipcon)

        # Connect in a thread to avoid blocking the event loop
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None, self._ipcon.connect, self.host, self.port
        )

        # Configure stepper
        self._stepper.set_motor_current(800)  # mA — adjust per motor
        self._stepper.set_step_configuration(
            self.step_resolution, True  # interpolation
        )
        self._stepper.set_max_velocity(self.max_velocity)
        self._stepper.set_speed_ramping(self.acceleration, self.deceleration)
        self._stepper.set_enabled(True)

        # Register position reached callback
        self._stepper.register_callback(
            BrickletSilentStepperV2.CALLBACK_POSITION_REACHED,
            self._on_position_reached,
        )

        logger.info(
            "tinkerforge_drive.setup",
            key=self.key,
            uid=self.uid,
            host=self.host,
            port=self.port,
        )

    def _on_position_reached(self, position: int) -> None:
        """Callback from Tinkerforge when target position is reached."""
        self._current_position = float(position)
        if self._state == DriveState.MOVING:
            self._state = DriveState.REACHED
        elif self._state == DriveState.HOMING:
            self._state = DriveState.IDLE
        logger.info(
            "tinkerforge_drive.position_reached",
            key=self.key,
            position=position,
        )

    async def cleanup(self) -> None:
        self.emergency_stop()
        if self._stepper is not None:
            self._stepper.set_enabled(False)
        if self._ipcon is not None:
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._ipcon.disconnect)
            self._ipcon = None
            self._stepper = None
        logger.info("tinkerforge_drive.cleanup", key=self.key)

    async def move_to(self, position: float, speed: float = 1.0) -> None:
        async with self._move_lock:
            target = int(max(self._min_position, min(self._max_position, position)))
            self._target_position = float(target)
            self._state = DriveState.MOVING
            self._stop_event.clear()

            if abs(target - self._current_position) < 1:
                self._current_position = float(target)
                self._state = DriveState.REACHED
                return

            speed = max(0.01, min(1.0, speed))
            velocity = int(self.max_velocity * speed)

            logger.info(
                "tinkerforge_drive.move_start",
                key=self.key,
                target=target,
                velocity=velocity,
            )

            if self._simulated:
                await self._simulate_move(float(target))
                return

            self._stepper.set_max_velocity(velocity)
            self._stepper.set_target_position(target)

            # Poll until position reached or stopped
            while self._state == DriveState.MOVING:
                if self._stop_event.is_set():
                    self._state = DriveState.IDLE
                    return
                await asyncio.sleep(0.05)
                try:
                    pos = self._stepper.get_current_position()
                    self._current_position = float(pos)
                except Exception:
                    pass

            logger.info(
                "tinkerforge_drive.move_complete",
                key=self.key,
                position=self._current_position,
            )

    async def _simulate_move(self, target: float) -> None:
        """Simulated movement for development without Tinkerforge hardware."""
        start = self._current_position
        delta = target - start
        total_time = abs(delta) / self.max_velocity
        sim_time = min(total_time, 3.0)
        elapsed = 0.0

        while elapsed < sim_time:
            if self._stop_event.is_set():
                self._state = DriveState.IDLE
                return
            await asyncio.sleep(0.05)
            elapsed += 0.05
            progress = min(elapsed / sim_time, 1.0) if sim_time > 0 else 1.0
            self._current_position = start + delta * progress

        self._current_position = target
        self._state = DriveState.REACHED
        logger.info(
            "tinkerforge_drive.simulated_move_complete",
            key=self.key,
            position=target,
        )

    async def home(self) -> None:
        """Home to minimum position."""
        self._state = DriveState.HOMING
        logger.info("tinkerforge_drive.homing", key=self.key)
        await self.move_to(float(self._min_position), speed=0.5)
        self._current_position = float(self._min_position)
        self._state = DriveState.IDLE
        logger.info("tinkerforge_drive.homed", key=self.key)

    def emergency_stop(self) -> None:
        self._stop_event.set()
        if self._stepper is not None:
            try:
                self._stepper.stop()
            except Exception:
                pass
        self._target_position = None
        self._state = DriveState.IDLE
        logger.warning("tinkerforge_drive.emergency_stop", key=self.key)

    def get_min_position(self) -> float:
        return float(self._min_position)

    def get_max_position(self) -> float:
        return float(self._max_position)
