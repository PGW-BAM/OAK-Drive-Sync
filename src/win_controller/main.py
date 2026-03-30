"""
Windows Camera Controller — Main Entry Point

Orchestrates the move→confirm→capture workflow for OAK-D 4 Pro cameras
coordinated with the Pi drive controller via MQTT.
"""

from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from enum import StrEnum
from pathlib import Path
from typing import Any
from uuid import uuid4

import click
import structlog
import yaml

from src.shared.models import (
    CameraHealth,
    CameraState,
    CameraStatus,
    CaptureSequence,
    CaptureStep,
    DrivePosition,
    DriveState,
    MoveCommand,
    OrchestrationError,
    WinControllerHealth,
)
from src.shared.mqtt_client import MQTTClient
from src.shared.mqtt_topics import (
    CMD_DRIVE_MOVE,
    ERROR_ORCHESTRATION,
    HEALTH_CAMERA,
    HEALTH_WIN_CONTROLLER,
    STATUS_CAMERA_STATE,
    STATUS_DRIVE_POSITION,
    SUB_ALL_DRIVE_STATUS,
    SUB_ALL_HEALTH,
    topic,
)

logger = structlog.get_logger()


class OrchestratorState(StrEnum):
    IDLE = "idle"
    MOVING = "moving"
    SETTLING = "settling"
    CAPTURING = "capturing"
    ERROR = "error"
    PAUSED = "paused"


class CameraManager:
    """Manages DepthAI camera connections and captures."""

    def __init__(self, config: dict) -> None:
        self.config = config
        self.devices: dict[str, Any] = {}  # cam_id → dai.Device (lazy init)

    async def initialize(self) -> list[str]:
        """Discover and initialize cameras. Returns list of connected cam_ids."""
        connected = []
        for cam_id, cam_cfg in self.config["cameras"].items():
            try:
                # TODO: Replace with real DepthAI initialization
                # import depthai as dai
                # device = dai.Device(dai.Pipeline(), dai.DeviceInfo(cam_cfg["mx_id"]))
                logger.info("camera.init", cam_id=cam_id, mx_id=cam_cfg.get("mx_id", ""))
                self.devices[cam_id] = None  # Placeholder for dai.Device
                connected.append(cam_id)
            except Exception as e:
                logger.error("camera.init_failed", cam_id=cam_id, error=str(e))
        return connected

    async def capture(self, cam_id: str, sequence_id: str, metadata: dict) -> Path | None:
        """Trigger a still capture and save to disk."""
        cam_cfg = self.config["cameras"][cam_id]
        capture_dir = Path(self.config["storage"]["capture_dir"])
        ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
        fmt = cam_cfg["capture"]["format"]
        out_path = capture_dir / cam_id / sequence_id / f"{ts}.{fmt}"
        out_path.parent.mkdir(parents=True, exist_ok=True)

        try:
            # TODO: Replace with real DepthAI capture
            # queue = device.getOutputQueue("still", maxSize=1, blocking=True)
            # frame = queue.get()
            # cv2.imwrite(str(out_path), frame.getCvFrame())
            logger.info("camera.captured", cam_id=cam_id, path=str(out_path))

            # Write placeholder for testing
            out_path.write_text(f"placeholder capture {ts}")
            return out_path

        except Exception as e:
            logger.error("camera.capture_failed", cam_id=cam_id, error=str(e))
            return None

    def is_connected(self, cam_id: str) -> bool:
        return cam_id in self.devices

    def get_frame(self, cam_id: str):
        """Get a preview frame for the streaming overlay. Returns numpy array or None."""
        # TODO: Implement DepthAI preview queue read
        return None


class Orchestrator:
    """State machine for the move→confirm→capture workflow."""

    def __init__(
        self,
        mqtt: MQTTClient,
        camera_mgr: CameraManager,
        config: dict,
    ) -> None:
        self.mqtt = mqtt
        self.camera_mgr = camera_mgr
        self.config = config
        self.state = OrchestratorState.IDLE
        self.current_sequence: CaptureSequence | None = None
        self.current_step_index: int = 0
        self.total_steps: int = 0

        # Drive position tracking (updated from MQTT)
        self.drive_positions: dict[str, DrivePosition] = {}
        self._position_events: dict[str, asyncio.Event] = {}

    async def handle_drive_status(self, topic_str: str, payload: dict) -> None:
        """Handle incoming drive position updates."""
        parts = topic_str.split("/")
        cam_id = parts[2]
        pos = DrivePosition.model_validate(payload)
        key = f"{cam_id}:{pos.drive_axis}"

        self.drive_positions[key] = pos

        # Signal waiting orchestrator if drive reached target
        if pos.state in (DriveState.REACHED, DriveState.IDLE) and key in self._position_events:
            self._position_events[key].set()

    async def run_sequence(self, sequence: CaptureSequence) -> None:
        """Execute a full capture sequence."""
        self.current_sequence = sequence
        self.total_steps = len(sequence.steps)
        logger.info(
            "sequence.start",
            seq_id=sequence.sequence_id,
            name=sequence.name,
            steps=self.total_steps,
        )

        orch_cfg = self.config.get("orchestration", {})
        move_timeout = orch_cfg.get("move_timeout_s", 30)
        capture_timeout = orch_cfg.get("capture_timeout_s", 10)
        retry_count = orch_cfg.get("retry_count", 1)

        for i, step in enumerate(sequence.steps):
            if self.state == OrchestratorState.PAUSED:
                logger.info("sequence.paused", step=i)
                # TODO: Wait for resume signal
                continue

            self.current_step_index = i
            logger.info("sequence.step", index=i, cam=step.cam_id, pos=step.position)

            try:
                # ── Move both axes ───────────────────
                self.state = OrchestratorState.MOVING
                await self._move_drive(
                    step.cam_id, "a", step.position.drive_a, sequence.sequence_id, move_timeout
                )
                await self._move_drive(
                    step.cam_id, "b", step.position.drive_b, sequence.sequence_id, move_timeout
                )

                # ── Settling delay ───────────────────
                self.state = OrchestratorState.SETTLING
                settling_s = step.settling_delay_ms / 1000.0
                await asyncio.sleep(settling_s)

                # ── Capture ──────────────────────────
                self.state = OrchestratorState.CAPTURING
                result = await asyncio.wait_for(
                    self.camera_mgr.capture(
                        step.cam_id,
                        sequence.sequence_id,
                        {"step": i, "position": step.position.model_dump()},
                    ),
                    timeout=capture_timeout,
                )

                if result is None:
                    # Retry once
                    for attempt in range(retry_count):
                        logger.warning("capture.retry", attempt=attempt + 1)
                        result = await asyncio.wait_for(
                            self.camera_mgr.capture(
                                step.cam_id, sequence.sequence_id, {"step": i, "retry": attempt + 1}
                            ),
                            timeout=capture_timeout,
                        )
                        if result:
                            break

                self.state = OrchestratorState.IDLE

            except asyncio.TimeoutError:
                self.state = OrchestratorState.ERROR
                err = OrchestrationError(
                    event="timeout",
                    sequence_id=sequence.sequence_id,
                    cam_id=step.cam_id,
                    message=f"Step {i} timed out",
                )
                await self.mqtt.publish(
                    topic(ERROR_ORCHESTRATION, event="timeout"), err, qos=1
                )
                logger.error("sequence.timeout", step=i)
                break

            except Exception as e:
                self.state = OrchestratorState.ERROR
                err = OrchestrationError(
                    event="sequence_abort",
                    sequence_id=sequence.sequence_id,
                    cam_id=step.cam_id,
                    message=str(e),
                )
                await self.mqtt.publish(
                    topic(ERROR_ORCHESTRATION, event="sequence_abort"), err, qos=1
                )
                logger.exception("sequence.error", step=i)
                break

        self.state = OrchestratorState.IDLE
        logger.info("sequence.complete", seq_id=sequence.sequence_id)

    async def _move_drive(
        self,
        cam_id: str,
        axis: str,
        target: float,
        sequence_id: str,
        timeout_s: float,
    ) -> None:
        """Send move command and wait for confirmation."""
        key = f"{cam_id}:{axis}"
        event = asyncio.Event()
        self._position_events[key] = event

        cmd = MoveCommand(
            sequence_id=sequence_id,
            drive_axis=axis,  # type: ignore[arg-type]
            target_position=target,
        )
        await self.mqtt.publish(topic(CMD_DRIVE_MOVE, cam_id=cam_id), cmd, qos=1)

        try:
            await asyncio.wait_for(event.wait(), timeout=timeout_s)
        finally:
            self._position_events.pop(key, None)


async def heartbeat_loop(mqtt: MQTTClient, camera_mgr: CameraManager) -> None:
    """Publish Windows controller health every 2 seconds."""
    while True:
        health = WinControllerHealth(
            online=True,
            cameras_connected=list(camera_mgr.devices.keys()),
        )
        await mqtt.publish(HEALTH_WIN_CONTROLLER, health, qos=0)

        # Per-camera health
        for cam_id in camera_mgr.devices:
            cam_health = CameraHealth(
                cam_id=cam_id,
                online=camera_mgr.is_connected(cam_id),
            )
            await mqtt.publish(topic(HEALTH_CAMERA, cam_id=cam_id), cam_health, qos=0)

        await asyncio.sleep(2.0)


async def run_app(config_path: str, sequence_path: str | None) -> None:
    """Main async entry point for the Windows camera controller."""
    with open(config_path) as f:
        config = yaml.safe_load(f)

    mqtt_cfg = config.get("mqtt", {})
    mqtt = MQTTClient(
        broker_host=mqtt_cfg.get("broker_host", "localhost"),
        broker_port=mqtt_cfg.get("broker_port", 1883),
        client_id=mqtt_cfg.get("client_id", "oak-win-controller"),
        lwt_topic=HEALTH_WIN_CONTROLLER,
        lwt_payload={"online": False},
    )

    camera_mgr = CameraManager(config)
    orchestrator = Orchestrator(mqtt, camera_mgr, config)

    # Register MQTT handlers
    mqtt.subscribe(SUB_ALL_DRIVE_STATUS, orchestrator.handle_drive_status)

    # Initialize cameras
    connected = await camera_mgr.initialize()
    logger.info("cameras.ready", connected=connected)

    # Load sequence if provided
    sequence = None
    if sequence_path:
        with open(sequence_path) as f:
            seq_data = yaml.safe_load(f)
        if "sequence_id" not in seq_data:
            seq_data["sequence_id"] = str(uuid4())[:8]
        sequence = CaptureSequence.model_validate(seq_data)

    async with asyncio.TaskGroup() as tg:
        tg.create_task(mqtt.run())
        tg.create_task(heartbeat_loop(mqtt, camera_mgr))
        if sequence:
            # Wait for MQTT to connect first
            await mqtt._connected.wait()
            tg.create_task(orchestrator.run_sequence(sequence))


@click.group()
def cli() -> None:
    """OAK-Drive-Sync Camera Controller (Windows)."""
    structlog.configure(
        processors=[
            structlog.dev.ConsoleRenderer(),
        ],
    )


@cli.command()
@click.option("--config", default="config/camera_config.yaml", help="Camera config file")
@click.option("--sequence", default=None, help="Capture sequence YAML to run immediately")
def run(config: str, sequence: str | None) -> None:
    """Start the camera controller and optionally run a capture sequence."""
    logger.info("cam_controller.starting", config=config, sequence=sequence)
    asyncio.run(run_app(config, sequence))


if __name__ == "__main__":
    cli()
