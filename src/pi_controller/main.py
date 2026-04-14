"""
Pi Drive Controller — Main Entry Point

Manages 4 mechanical drives (2 GPIO linear + 2 Tinkerforge steppers),
coordinated through MQTT commands from the Windows controller.
Includes a NiceGUI web interface for non-programmers.
"""

from __future__ import annotations

import asyncio
from pathlib import Path

import click
import structlog
import yaml

from src.pi_controller.drives.base import BaseDrive, DriveState
from src.pi_controller.drives.gpio_drive import GPIODrive
from src.pi_controller.drives.tinkerforge_drive import TinkerforgeDrive
from src.pi_controller.imu.drift_detector import DriftDetector
from src.shared.models import (
    DriveError,
    DrivePosition,
    PiHealth,
    PositionPreset,
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
    SUB_ALL_IMU_TELEMETRY,
    topic,
)

logger = structlog.get_logger()


# ──────────────────────────────────────────────
# Drive Factory
# ──────────────────────────────────────────────

def create_drive(cam_id: str, axis: str, cfg: dict) -> BaseDrive:
    """Create a drive instance from config."""
    drive_type = cfg.pop("type")
    if drive_type == "gpio":
        return GPIODrive(cam_id, axis, **cfg)
    elif drive_type == "tinkerforge":
        return TinkerforgeDrive(cam_id, axis, **cfg)
    else:
        raise ValueError(f"Unknown drive type: {drive_type}")


# ──────────────────────────────────────────────
# Drive Manager
# ──────────────────────────────────────────────

class DriveManager:
    """Manages all drives and dispatches MQTT commands."""

    def __init__(self, config: dict, mqtt: MQTTClient) -> None:
        self.config = config
        self.mqtt = mqtt
        self.drives: dict[str, BaseDrive] = {}
        self.settling_delay_ms: int = config.get("settling_delay_ms", 150)
        self.drift_detector: DriftDetector | None = None
        self._sequence_running = False
        self._sequence_stop = asyncio.Event()
        self._init_drives()

    def _init_drives(self) -> None:
        for cam_id, axes in self.config["drives"].items():
            for axis_key, axis_cfg in axes.items():
                # Support both "axis_a" and "a" formats
                axis = axis_key.split("_")[-1] if "_" in axis_key else axis_key
                cfg = dict(axis_cfg)  # copy so we don't mutate config
                drive = create_drive(cam_id, axis, cfg)
                self.drives[drive.key] = drive
        logger.info("drives.initialized", count=len(self.drives), keys=list(self.drives.keys()))

    async def setup_all(self) -> None:
        failed = []
        for drive in list(self.drives.values()):
            try:
                await drive.setup()
            except Exception as exc:
                logger.error(
                    "drive.setup_failed",
                    key=drive.key,
                    error=str(exc),
                )
                failed.append(drive.key)
        if failed:
            logger.warning("drives.some_failed", failed=failed, msg="These drives will be unavailable")

    async def cleanup_all(self) -> None:
        for drive in self.drives.values():
            try:
                await drive.cleanup()
            except Exception as exc:
                logger.error("drive.cleanup_failed", key=drive.key, error=str(exc))

    async def handle_move(self, topic_str: str, payload: dict) -> None:
        parts = topic_str.split("/")
        cam_id = parts[2]
        axis = payload.get("drive_axis", "a")
        key = f"{cam_id}:{axis}"
        drive = self.drives.get(key)
        if not drive:
            logger.error("drive.unknown", key=key)
            return

        sequence_id = payload.get("sequence_id")
        target = payload["target_position"]
        speed = payload.get("speed", 1.0)

        logger.info("drive.move_start", key=key, target=target, speed=speed, seq=sequence_id)

        # Publish moving state
        await self._publish_position(drive, sequence_id)

        try:
            await drive.move_to(target, speed)

            # Wait settling time
            await asyncio.sleep(self.settling_delay_ms / 1000.0)

            # IMU drift check — axis_b Tinkerforge radial drives only
            if (
                axis == "b"
                and self.drift_detector is not None
                and payload.get("checkpoint_name")
                and isinstance(drive, TinkerforgeDrive)
            ):
                await self.drift_detector.check_and_correct(
                    cam_id, drive, payload["checkpoint_name"]
                )

            # Publish reached state
            await self._publish_position(drive, sequence_id)
            logger.info("drive.move_complete", key=key, position=drive.current_position)

        except Exception as exc:
            logger.error("drive.move_error", key=key, error=str(exc))
            await self.mqtt.publish(
                topic(ERROR_DRIVE, cam_id=cam_id),
                DriveError(
                    sequence_id=sequence_id,
                    drive_axis=axis,
                    error_type="stall",
                    message=str(exc),
                ),
                qos=1,
            )
            await self._publish_position(drive, sequence_id)

    async def handle_home(self, topic_str: str, payload: dict) -> None:
        parts = topic_str.split("/")
        cam_id = parts[2]
        axis = payload.get("drive_axis", "a")
        key = f"{cam_id}:{axis}"
        drive = self.drives.get(key)
        if not drive:
            return

        sequence_id = payload.get("sequence_id")
        logger.info("drive.homing", key=key)

        await self._publish_position(drive, sequence_id)

        try:
            await drive.home()
            await self._publish_position(drive, sequence_id)
            logger.info("drive.homed", key=key)
        except Exception as exc:
            logger.error("drive.home_error", key=key, error=str(exc))
            await self.mqtt.publish(
                topic(ERROR_DRIVE, cam_id=cam_id),
                DriveError(
                    sequence_id=sequence_id,
                    drive_axis=axis,
                    error_type="stall",
                    message=str(exc),
                ),
                qos=1,
            )

    async def handle_stop(self, topic_str: str, payload: dict) -> None:
        parts = topic_str.split("/")
        cam_id = parts[2]
        axis = payload.get("drive_axis")
        sequence_id = payload.get("sequence_id")

        if axis:
            keys = [f"{cam_id}:{axis}"]
        else:
            keys = [k for k in self.drives if k.startswith(f"{cam_id}:")]

        for key in keys:
            drive = self.drives.get(key)
            if drive:
                drive.emergency_stop()
                await self._publish_position(drive, sequence_id)
                logger.warning("drive.stopped", key=key)

    async def _publish_position(self, drive: BaseDrive, sequence_id: str | None = None) -> None:
        from src.pi_controller.position_log import append_position_entry
        pos = DrivePosition(
            sequence_id=sequence_id,
            drive_axis=drive.axis,
            current_position=drive.current_position,
            target_position=drive.target_position,
            state=drive.state.value,
        )
        await self.mqtt.publish(
            topic(STATUS_DRIVE_POSITION, cam_id=drive.cam_id),
            pos,
            qos=1,
            retain=True,
        )
        append_position_entry(self.drives)

    def get_drive_states(self) -> dict[str, str]:
        return {k: v.state.value for k, v in self.drives.items()}

    # ──────────────────────────────────────────
    # Position-based sequence (GUI-driven)
    # ──────────────────────────────────────────

    async def move_all_to_position(self, preset: PositionPreset) -> None:
        """Move all 4 drives to a position preset in parallel."""
        targets = preset.as_drive_targets()
        tasks = []
        for key, target_pos in targets.items():
            drive = self.drives.get(key)
            if drive:
                tasks.append(drive.move_to(target_pos))
        if tasks:
            await asyncio.gather(*tasks)

    async def run_sequence(
        self,
        positions: list[PositionPreset],
        dwell_time_ms: int = 500,
        repeat_count: int = 1,
    ) -> None:
        """Run a cyclic capture sequence through positions.

        For each position: move all drives → settle → notify cameras → dwell → next.
        repeat_count=0 means infinite.
        """
        self._sequence_running = True
        self._sequence_stop.clear()
        cycle = 0

        logger.info(
            "sequence.start",
            positions=len(positions),
            repeats=repeat_count,
            dwell_ms=dwell_time_ms,
        )

        try:
            while repeat_count == 0 or cycle < repeat_count:
                if self._sequence_stop.is_set():
                    break
                cycle += 1
                logger.info("sequence.cycle", cycle=cycle)

                for i, preset in enumerate(positions):
                    if self._sequence_stop.is_set():
                        break

                    logger.info("sequence.moving_to", position=preset.name, step=i + 1)

                    # Move all drives in parallel
                    await self.move_all_to_position(preset)

                    # Settling delay
                    await asyncio.sleep(self.settling_delay_ms / 1000.0)

                    # Publish "reached" for each drive → Windows cameras capture
                    targets = preset.as_drive_targets()
                    for key in targets:
                        drive = self.drives.get(key)
                        if drive:
                            await self._publish_position(drive, f"seq-cycle{cycle}-step{i}")

                    logger.info("sequence.position_reached", position=preset.name)

                    # Dwell time (pause at position)
                    await asyncio.sleep(dwell_time_ms / 1000.0)

        finally:
            self._sequence_running = False
            logger.info("sequence.complete", cycles=cycle)

    def stop_sequence(self) -> None:
        self._sequence_stop.set()
        for drive in self.drives.values():
            drive.emergency_stop()

    @property
    def sequence_running(self) -> bool:
        return self._sequence_running


# ──────────────────────────────────────────────
# Heartbeat
# ──────────────────────────────────────────────

async def heartbeat_loop(mqtt: MQTTClient, drive_mgr: DriveManager) -> None:
    """Publish Pi health beacon every 2 seconds."""
    while True:
        try:
            cpu_temp = _read_cpu_temp()
            uptime = _read_uptime()

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


def _read_cpu_temp() -> float:
    temp_path = Path("/sys/class/thermal/thermal_zone0/temp")
    if temp_path.exists():
        return int(temp_path.read_text().strip()) / 1000.0
    return 0.0


def _read_uptime() -> int:
    uptime_path = Path("/proc/uptime")
    if uptime_path.exists():
        return int(float(uptime_path.read_text().split()[0]))
    return 0


# ──────────────────────────────────────────────
# Main entry point
# ──────────────────────────────────────────────

def _load_config(config_path: str, broker_host: str | None, broker_port: int | None) -> tuple[dict, str, int]:
    """Load config and resolve broker settings."""
    with open(config_path) as f:
        config = yaml.safe_load(f)
    broker = config.get("broker", {})
    host = broker_host or broker.get("host", "localhost")
    port = broker_port or broker.get("port", 1883)
    return config, host, port


async def _setup_controller(config: dict, host: str, port: int) -> tuple[DriveManager, MQTTClient]:
    """Create and wire up drives + MQTT. Returns (drive_mgr, mqtt)."""
    mqtt = MQTTClient(
        broker_host=host,
        broker_port=port,
        client_id="oak-pi-drive-controller",
        lwt_topic=HEALTH_PI,
        lwt_payload={"online": False},
    )

    drive_mgr = DriveManager(config, mqtt)
    await drive_mgr.setup_all()

    drift_detector = DriftDetector(mqtt, Path("config/imu_calibration.yaml"))
    drive_mgr.drift_detector = drift_detector
    mqtt.subscribe(SUB_ALL_IMU_TELEMETRY, drift_detector.on_imu_message)

    async def dispatch_command(topic_str: str, payload: dict) -> None:
        if topic_str.endswith("/move"):
            await drive_mgr.handle_move(topic_str, payload)
        elif topic_str.endswith("/home"):
            await drive_mgr.handle_home(topic_str, payload)
        elif topic_str.endswith("/stop"):
            await drive_mgr.handle_stop(topic_str, payload)

    mqtt.subscribe(SUB_ALL_DRIVE_CMDS, dispatch_command)
    return drive_mgr, mqtt


async def run_controller_headless(
    config_path: str,
    broker_host: str | None,
    broker_port: int | None,
) -> None:
    """Run without GUI — MQTT + heartbeat only."""
    from src.pi_controller.gui import apply_calibration
    from src.pi_controller.position_log import rotate_log, seed_positions_from_log

    rotate_log()
    config, host, port = _load_config(config_path, broker_host, broker_port)
    drive_mgr, mqtt = await _setup_controller(config, host, port)
    seed_positions_from_log(drive_mgr)
    apply_calibration(drive_mgr)
    from src.pi_controller.gui import auto_seed_min_calibration
    auto_seed_min_calibration(drive_mgr)

    try:
        async with asyncio.TaskGroup() as tg:
            tg.create_task(mqtt.run())
            tg.create_task(heartbeat_loop(mqtt, drive_mgr))
    finally:
        await drive_mgr.cleanup_all()


def run_controller_with_gui(
    config_path: str,
    broker_host: str | None,
    broker_port: int | None,
) -> None:
    """Run with NiceGUI. NiceGUI owns the event loop; MQTT runs as background tasks."""
    from nicegui import app as nicegui_app
    from src.pi_controller.gui import setup_gui, run_gui

    config, host, port = _load_config(config_path, broker_host, broker_port)

    # NiceGUI needs pages registered before ui.run(), but drives need async setup.
    # We create drive_mgr synchronously (no GPIO init yet), register pages,
    # then do async setup in on_startup.

    mqtt = MQTTClient(
        broker_host=host,
        broker_port=port,
        client_id="oak-pi-drive-controller",
        lwt_topic=HEALTH_PI,
        lwt_payload={"online": False},
    )

    # Pre-create DriveManager (drives not yet initialized)
    drive_mgr = DriveManager(config, mqtt)

    # DriftDetector is constructed synchronously (no asyncio objects in __init__)
    drift_detector = DriftDetector(mqtt, Path("config/imu_calibration.yaml"))
    drive_mgr.drift_detector = drift_detector

    # Register all GUI pages
    gui_cfg = setup_gui(drive_mgr, config, config.get("gui", {}), drift_detector=drift_detector)

    # Wire up MQTT command dispatch
    async def dispatch_command(topic_str: str, payload: dict) -> None:
        if topic_str.endswith("/move"):
            await drive_mgr.handle_move(topic_str, payload)
        elif topic_str.endswith("/home"):
            await drive_mgr.handle_home(topic_str, payload)
        elif topic_str.endswith("/stop"):
            await drive_mgr.handle_stop(topic_str, payload)

    mqtt.subscribe(SUB_ALL_DRIVE_CMDS, dispatch_command)

    # Async startup: init GPIO/Tinkerforge, subscribe IMU, start MQTT + heartbeat
    async def on_startup() -> None:
        from src.pi_controller.gui import apply_calibration, auto_seed_min_calibration
        from src.pi_controller.position_log import rotate_log, seed_positions_from_log

        rotate_log()
        await drive_mgr.setup_all()
        seed_positions_from_log(drive_mgr)
        apply_calibration(drive_mgr)
        auto_seed_min_calibration(drive_mgr)
        mqtt.subscribe(SUB_ALL_IMU_TELEMETRY, drift_detector.on_imu_message)
        asyncio.create_task(mqtt.run())
        asyncio.create_task(heartbeat_loop(mqtt, drive_mgr))
        logger.info("background_tasks.started")

    async def on_shutdown() -> None:
        await mqtt.stop()
        await drive_mgr.cleanup_all()
        logger.info("background_tasks.stopped")

    nicegui_app.on_startup(on_startup)
    nicegui_app.on_shutdown(on_shutdown)

    # This blocks — NiceGUI owns the event loop
    run_gui(gui_cfg)


# ──────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────

@click.group()
def cli() -> None:
    """OAK-Drive-Sync Pi Drive Controller."""
    structlog.configure(
        processors=[
            structlog.dev.ConsoleRenderer(),
        ],
    )


@cli.command()
@click.option("--config", default="config/drive_config.yaml", help="Drive config file")
@click.option("--broker-host", default=None, help="MQTT broker hostname (overrides config)")
@click.option("--broker-port", default=None, type=int, help="MQTT broker port (overrides config)")
@click.option("--no-gui", is_flag=True, default=False, help="Disable web GUI")
def run(config: str, broker_host: str | None, broker_port: int | None, no_gui: bool) -> None:
    """Start the drive controller."""
    logger.info("pi_controller.starting", config=config)
    if no_gui:
        asyncio.run(run_controller_headless(config, broker_host, broker_port))
    else:
        run_controller_with_gui(config, broker_host, broker_port)


if __name__ == "__main__":
    cli()
