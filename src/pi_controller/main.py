"""
Pi Drive Controller — Main Entry Point

Manages 4 mechanical drives (2 per camera) via GPIO,
coordinated through MQTT commands from the Windows controller.
"""

from __future__ import annotations

import asyncio
from pathlib import Path

import click
import structlog
import yaml

from src.shared.models import (
    DriveError,
    DriveErrorType,
    DrivePosition,
    DriveState,
    HomeCommand,
    MoveCommand,
    PiHealth,
    StopCommand,
)
from src.shared.mqtt_client import MQTTClient
from src.shared.mqtt_topics import (
    CMD_DRIVE_HOME,
    CMD_DRIVE_MOVE,
    CMD_DRIVE_STOP,
    ERROR_DRIVE,
    HEALTH_PI,
    STATUS_DRIVE_POSITION,
    SUB_ALL_DRIVE_CMDS,
    topic,
)

logger = structlog.get_logger()


class DriveManager:
    """Manages all drives for one or more cameras."""

    def __init__(self, config: dict, mqtt: MQTTClient) -> None:
        self.config = config
        self.mqtt = mqtt
        self.drives: dict[str, dict] = {}  # "cam1:a" → drive config + state
        self._init_drives()

    def _init_drives(self) -> None:
        """Initialize drive state from config."""
        for cam_id, axes in self.config["drives"].items():
            for axis_name, axis_cfg in axes.items():
                if axis_name.startswith("axis_"):
                    axis_letter = axis_name.split("_")[1]
                    key = f"{cam_id}:{axis_letter}"
                    self.drives[key] = {
                        **axis_cfg,
                        "cam_id": cam_id,
                        "axis": axis_letter,
                        "current_position": 0.0,
                        "state": DriveState.IDLE,
                    }
        logger.info("drives.initialized", count=len(self.drives))

    async def handle_move(self, topic_str: str, payload: dict) -> None:
        """Handle a move command from MQTT."""
        # Extract cam_id from topic: cmd/drives/{cam_id}/move
        parts = topic_str.split("/")
        cam_id = parts[2]
        cmd = MoveCommand.model_validate(payload)
        key = f"{cam_id}:{cmd.drive_axis}"

        drive = self.drives.get(key)
        if not drive:
            logger.error("drive.unknown", key=key)
            return

        logger.info(
            "drive.move_start",
            key=key,
            target=cmd.target_position,
            speed=cmd.speed,
            seq=cmd.sequence_id,
        )

        # Update state → moving
        drive["state"] = DriveState.MOVING
        await self._publish_position(cam_id, cmd.drive_axis, drive, cmd.sequence_id)

        # TODO: Replace with real GPIO step/dir logic
        # Simulate movement duration based on distance and speed
        distance = abs(cmd.target_position - drive["current_position"])
        steps = distance * drive["steps_per_unit"]
        duration = steps / (drive["max_speed"] * cmd.speed) if cmd.speed > 0 else 0
        await asyncio.sleep(duration)

        # Update position
        drive["current_position"] = cmd.target_position
        drive["state"] = DriveState.REACHED

        # Wait settling time
        settling_ms = self.config.get("settling_delay_ms", 150)
        await asyncio.sleep(settling_ms / 1000.0)

        drive["state"] = DriveState.IDLE
        await self._publish_position(cam_id, cmd.drive_axis, drive, cmd.sequence_id)
        logger.info("drive.move_complete", key=key, position=cmd.target_position)

    async def handle_home(self, topic_str: str, payload: dict) -> None:
        """Handle a home command."""
        parts = topic_str.split("/")
        cam_id = parts[2]
        cmd = HomeCommand.model_validate(payload)
        key = f"{cam_id}:{cmd.drive_axis}"

        drive = self.drives.get(key)
        if not drive:
            return

        logger.info("drive.homing", key=key)
        drive["state"] = DriveState.HOMING
        await self._publish_position(cam_id, cmd.drive_axis, drive, cmd.sequence_id)

        # TODO: Real homing logic — move toward limit switch at homing_speed
        await asyncio.sleep(2.0)  # Simulate homing

        drive["current_position"] = 0.0
        drive["state"] = DriveState.IDLE
        await self._publish_position(cam_id, cmd.drive_axis, drive, cmd.sequence_id)
        logger.info("drive.homed", key=key)

    async def handle_stop(self, topic_str: str, payload: dict) -> None:
        """Handle emergency stop."""
        parts = topic_str.split("/")
        cam_id = parts[2]
        cmd = StopCommand.model_validate(payload)

        if cmd.drive_axis:
            keys = [f"{cam_id}:{cmd.drive_axis}"]
        else:
            keys = [k for k in self.drives if k.startswith(f"{cam_id}:")]

        for key in keys:
            drive = self.drives.get(key)
            if drive:
                # TODO: Immediately disable GPIO step output
                drive["state"] = DriveState.IDLE
                axis = key.split(":")[1]
                await self._publish_position(cam_id, axis, drive, cmd.sequence_id)
                logger.warning("drive.stopped", key=key)

    async def _publish_position(
        self, cam_id: str, axis: str, drive: dict, sequence_id: str | None = None
    ) -> None:
        pos = DrivePosition(
            sequence_id=sequence_id,
            drive_axis=axis,  # type: ignore[arg-type]
            current_position=drive["current_position"],
            target_position=None,
            state=drive["state"],
        )
        await self.mqtt.publish(
            topic(STATUS_DRIVE_POSITION, cam_id=cam_id),
            pos,
            qos=1,
            retain=True,
        )

    def get_drive_states(self) -> dict[str, str]:
        """Return current state of all drives for health beacon."""
        return {k: str(v["state"]) for k, v in self.drives.items()}


async def heartbeat_loop(mqtt: MQTTClient, drive_mgr: DriveManager) -> None:
    """Publish Pi health beacon every 2 seconds."""
    import os

    while True:
        try:
            # Read CPU temperature (Linux/RPi)
            cpu_temp = 0.0
            temp_path = Path("/sys/class/thermal/thermal_zone0/temp")
            if temp_path.exists():
                cpu_temp = int(temp_path.read_text().strip()) / 1000.0

            uptime = int(float(Path("/proc/uptime").read_text().split()[0])) if Path("/proc/uptime").exists() else 0

            health = PiHealth(
                online=True,
                cpu_temp_c=cpu_temp,
                uptime_s=uptime,
                drive_states=drive_mgr.get_drive_states(),
            )
            await mqtt.publish(HEALTH_PI, health, qos=0, retain=False)
        except Exception:
            logger.exception("heartbeat.error")

        await asyncio.sleep(2.0)


async def run_controller(config_path: str, broker_host: str, broker_port: int) -> None:
    """Main async entry point."""
    with open(config_path) as f:
        config = yaml.safe_load(f)

    mqtt = MQTTClient(
        broker_host=broker_host,
        broker_port=broker_port,
        client_id="oak-pi-drive-controller",
        lwt_topic=HEALTH_PI,
        lwt_payload={"online": False},
    )

    drive_mgr = DriveManager(config, mqtt)

    # Register MQTT command handlers
    async def dispatch_command(topic_str: str, payload: dict) -> None:
        if topic_str.endswith("/move"):
            await drive_mgr.handle_move(topic_str, payload)
        elif topic_str.endswith("/home"):
            await drive_mgr.handle_home(topic_str, payload)
        elif topic_str.endswith("/stop"):
            await drive_mgr.handle_stop(topic_str, payload)

    mqtt.subscribe(SUB_ALL_DRIVE_CMDS, dispatch_command)

    # Run MQTT client and heartbeat concurrently
    async with asyncio.TaskGroup() as tg:
        tg.create_task(mqtt.run())
        tg.create_task(heartbeat_loop(mqtt, drive_mgr))


@click.group()
def cli() -> None:
    """OAK-Drive-Sync Pi Drive Controller."""
    structlog.configure(
        processors=[
            structlog.dev.ConsoleRenderer(),
        ],
    )


@cli.command()
@click.option("--config", default="config/drive_pinmap.yaml", help="Drive config file")
@click.option("--broker-host", default="localhost", help="MQTT broker hostname")
@click.option("--broker-port", default=1883, type=int, help="MQTT broker port")
def run(config: str, broker_host: str, broker_port: int) -> None:
    """Start the drive controller."""
    logger.info("pi_controller.starting", config=config, broker=broker_host)
    asyncio.run(run_controller(config, broker_host, broker_port))


if __name__ == "__main__":
    cli()
