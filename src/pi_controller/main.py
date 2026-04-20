"""
Pi Drive Controller — Main Entry Point

Manages 4 mechanical drives (2 GPIO linear + 2 Tinkerforge steppers),
coordinated through MQTT commands from the Windows controller.
Includes a NiceGUI web interface for non-programmers.
"""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Callable

import click
import structlog
import yaml

from src.pi_controller.drives.base import BaseDrive, DriveState
from src.pi_controller.drives.gpio_drive import GPIODrive
from src.pi_controller.drives.tinkerforge_drive import TinkerforgeDrive
from src.pi_controller.imu.auto_calibrator import auto_calibrate
from src.pi_controller.imu.drift_detector import DriftDetector
from src.shared.models import (
    DriveError,
    DrivePosition,
    PiHealth,
    PositionPreset,
    SequenceStep,
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
        # Set once startup auto-calibration finishes (success OR graceful
        # failure). handle_move's angle-driven branch waits briefly on this
        # before computing a coarse target from reference_position.
        self.autocal_complete: asyncio.Event = asyncio.Event()
        # Live reference to the GUI positions list so run_sequence can write
        # back updated motor-position seeds after successful angle convergence.
        self.positions: list[PositionPreset] = []
        self.save_positions_fn: Callable[[], None] | None = None
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

        target_angle_deg = payload.get("target_angle_deg")
        active_angle = payload.get("active_angle")
        resync_position = payload.get("resync_position")

        angle_driven = (
            axis == "b"
            and self.drift_detector is not None
            and isinstance(drive, TinkerforgeDrive)
            and target_angle_deg is not None
            and active_angle in ("roll", "pitch")
        )

        try:
            if angle_driven:
                # Angle-first: skip any open-loop coarse move and let converge()
                # handle everything. converge() has angle-stuck, motor-stuck, and
                # error-regression monitoring built in, so the first iteration
                # always moves toward the target; if it grows the error (wrong
                # sign), it flips once and retries. This replaces the earlier
                # "probe + remaining_delta" coarse that repeatedly misread IMU
                # noise near ±90° and persisted spurious sign flips.
                result = await self.drift_detector.converge(
                    cam_id,
                    drive,
                    target_angle_deg=float(target_angle_deg),
                    active_angle=active_angle,
                    checkpoint_name=payload.get("checkpoint_name") or "",
                    resync_position=(
                        float(resync_position) if resync_position is not None else None
                    ),
                )
                if not result.converged:
                    drive._state = DriveState.FAULT
                    logger.warning(
                        "drive.converge_failed",
                        key=key,
                        iterations=result.iterations,
                        final_error_deg=result.final_error_deg,
                    )
            else:
                await drive.move_to(target, speed)

                # Wait settling time
                await asyncio.sleep(self.settling_delay_ms / 1000.0)

                if (
                    axis == "b"
                    and self.drift_detector is not None
                    and payload.get("checkpoint_name")
                    and isinstance(drive, TinkerforgeDrive)
                ):
                    # Legacy one-shot drift check (no target_angle_deg supplied).
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
        steps: list[PositionPreset | SequenceStep],
        dwell_time_ms: int = 500,
        repeat_count: int = 1,
    ) -> None:
        """Run a cyclic capture sequence through a mixed list of steps.

        Each step is either:
          - SequenceStep(type="position")  → open-loop move of all 4 drives
          - SequenceStep(type="checkpoint") → closed-loop angle converge of cam:b

        Backwards compatible: plain PositionPreset items are normalised to
        position steps automatically.

        repeat_count=0 means infinite.
        """
        # Normalise: wrap bare PositionPreset items into SequenceStep
        normalised: list[SequenceStep] = []
        for s in steps:
            if isinstance(s, PositionPreset):
                normalised.append(SequenceStep(type="position", preset=s))
            else:
                normalised.append(s)

        self._sequence_running = True
        self._sequence_stop.clear()
        cycle = 0

        logger.info(
            "sequence.start",
            steps=len(normalised),
            repeats=repeat_count,
            dwell_ms=dwell_time_ms,
        )

        try:
            while repeat_count == 0 or cycle < repeat_count:
                if self._sequence_stop.is_set():
                    break
                cycle += 1
                logger.info("sequence.cycle", cycle=cycle)

                for i, step in enumerate(normalised):
                    if self._sequence_stop.is_set():
                        break

                    seq_id = f"seq-cycle{cycle}-step{i}"

                    if step.type == "position" and step.preset is not None:
                        preset = step.preset
                        logger.info("sequence.moving_to", position=preset.name, step=i + 1)

                        # Axis-a drives: always open-loop in parallel
                        a_tasks = []
                        for cam_id in ("cam1", "cam2"):
                            drive_a = self.drives.get(f"{cam_id}:a")
                            if drive_a:
                                a_tasks.append(
                                    drive_a.move_to(getattr(preset, f"{cam_id}_a"))
                                )
                        if a_tasks:
                            await asyncio.gather(*a_tasks)

                        await asyncio.sleep(self.settling_delay_ms / 1000.0)

                        # Axis-b drives: converge to saved angle if target stored,
                        # else open-loop. Sequential (IMU responses must not cross-wire).
                        for cam_id in ("cam1", "cam2"):
                            drive_b = self.drives.get(f"{cam_id}:b")
                            if drive_b is None:
                                continue
                            angle_target = getattr(preset, f"{cam_id}_b_angle_deg", None)
                            motor_seed = int(getattr(preset, f"{cam_id}_b", 0))

                            if (
                                angle_target is not None
                                and self.drift_detector is not None
                                and isinstance(drive_b, TinkerforgeDrive)
                            ):
                                active_angle = self.drift_detector.get_active_angle(cam_id)
                                logger.info(
                                    "sequence.angle_converge",
                                    cam_id=cam_id,
                                    target_angle_deg=angle_target,
                                    seed_position=motor_seed,
                                    position=preset.name,
                                )
                                result = await self.drift_detector.converge(
                                    cam_id=cam_id,
                                    drive=drive_b,
                                    target_angle_deg=angle_target,
                                    active_angle=active_angle,
                                    checkpoint_name=f"pos:{preset.name}",
                                    seed_position=motor_seed,
                                )
                                if result.converged:
                                    new_seed = int(drive_b.current_position)
                                    # Write updated motor position back so next
                                    # sequence run seeds closer to the right spot.
                                    setattr(preset, f"{cam_id}_b", float(new_seed))
                                    for p in self.positions:
                                        if p.name == preset.name:
                                            setattr(p, f"{cam_id}_b", float(new_seed))
                                            break
                                    if self.save_positions_fn is not None:
                                        try:
                                            self.save_positions_fn()
                                        except Exception:
                                            logger.exception(
                                                "sequence.save_positions_failed"
                                            )
                                else:
                                    logger.warning(
                                        "sequence.angle_converge_failed",
                                        cam_id=cam_id,
                                        position=preset.name,
                                        iterations=result.iterations,
                                        final_error_deg=result.final_error_deg,
                                    )
                            else:
                                await drive_b.move_to(getattr(preset, f"{cam_id}_b"))

                        await asyncio.sleep(self.settling_delay_ms / 1000.0)

                        for key in preset.as_drive_targets():
                            drive = self.drives.get(key)
                            if drive:
                                await self._publish_position(drive, seq_id)

                        logger.info("sequence.position_reached", position=preset.name)

                    elif step.type == "checkpoint" and step.checkpoint is not None:
                        cp_ref = step.checkpoint
                        cam_id = cp_ref.cam_id
                        name = cp_ref.name

                        if self.drift_detector is None:
                            logger.warning(
                                "sequence.checkpoint_skipped_no_detector",
                                cam_id=cam_id,
                                name=name,
                            )
                        else:
                            cp = self.drift_detector._find_checkpoint(cam_id, name)
                            if cp is None:
                                logger.warning(
                                    "sequence.checkpoint_not_found",
                                    cam_id=cam_id,
                                    name=name,
                                )
                            else:
                                drive = self.drives.get(f"{cam_id}:b")
                                if drive is None or not isinstance(drive, TinkerforgeDrive):
                                    logger.warning(
                                        "sequence.checkpoint_no_drive",
                                        cam_id=cam_id,
                                        name=name,
                                    )
                                else:
                                    active_angle = cp["active_angle"]
                                    target_angle_deg = float(
                                        cp[f"expected_{active_angle}_deg"]
                                    )
                                    seed_position = int(cp.get("drive_position", 0))

                                    logger.info(
                                        "sequence.checkpoint_converge",
                                        cam_id=cam_id,
                                        name=name,
                                        target_angle_deg=target_angle_deg,
                                        seed_position=seed_position,
                                        step=i + 1,
                                    )

                                    result = await self.drift_detector.converge(
                                        cam_id=cam_id,
                                        drive=drive,
                                        target_angle_deg=target_angle_deg,
                                        active_angle=active_angle,
                                        checkpoint_name=name,
                                        seed_position=seed_position,
                                    )

                                    if not result.converged:
                                        logger.warning(
                                            "sequence.checkpoint_converge_failed",
                                            cam_id=cam_id,
                                            name=name,
                                            iterations=result.iterations,
                                            final_error_deg=result.final_error_deg,
                                        )

                                    await self._publish_position(drive, seq_id)

                    # Dwell at this step
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
# Startup auto-calibration
# ──────────────────────────────────────────────

async def run_startup_autocal(
    drive_mgr: DriveManager,
    drift_detector: DriftDetector,
) -> None:
    """Discover fresh reference_position + reference_angle + steps_per_degree
    for each cam:b drive.

    Runs once per service boot AFTER MQTT has connected and is receiving IMU
    telemetry. Sets `drive_mgr.autocal_complete` when done (success or failure).
    Angle-driven moves that arrive before this event is set fall back to
    pure-converge (legacy behavior).
    """
    try:
        # Load the auto_calibration block from the drift detector's YAML cache.
        ac_cfg: dict = (
            drift_detector._calibration.get("auto_calibration", {}) or {}
        )
        if not ac_cfg.get("enabled", True):
            logger.info("auto_calibrator.disabled_by_config")
            return

        # Wait for MQTT client to be connected (up to 15s). Auto-cal needs
        # request/response IMU traffic which only works post-subscribe.
        try:
            await asyncio.wait_for(drift_detector._mqtt._connected.wait(), timeout=15.0)
        except asyncio.TimeoutError:
            logger.warning("auto_calibrator.mqtt_not_connected")
            return

        # Brief settle so retained IMU telemetry / the first publish lands.
        await asyncio.sleep(1.0)

        active_angle_map: dict = ac_cfg.get("active_angle", {}) or {}

        # Only Tinkerforge radial (axis-b) drives get auto-cal. Sequential so
        # IMU responses don't get cross-wired between cameras.
        for key, drive in list(drive_mgr.drives.items()):
            if not key.endswith(":b"):
                continue
            if not isinstance(drive, TinkerforgeDrive):
                continue
            if getattr(drive, "_simulated", False):
                logger.info("auto_calibrator.skip_simulated", key=key)
                continue

            cam_id = key.split(":", 1)[0]
            active_angle = active_angle_map.get(f"{cam_id}_b", "roll")

            logger.info("auto_calibrator.start", cam_id=cam_id, active_angle=active_angle)
            result = await auto_calibrate(
                drift_detector,
                drive,
                cam_id,
                active_angle=active_angle,
                config=ac_cfg,
            )

            if result.success:
                drift_detector.set_reference_position(
                    cam_id, result.reference_position, axis="b"
                )
                drift_detector.set_reference_angle_deg(
                    cam_id, result.reference_angle_deg, axis="b"
                )
                drift_detector.set_steps_per_degree(cam_id, result.steps_per_degree)
                drift_detector.set_target_sign(cam_id, result.direction_sign, axis="b")
                # Prune angle_history entries whose stored motor position
                # is now incompatible with the fresh reference. Keeps the
                # close-to-correct entries (drift was small), drops far
                # ones so they re-learn via a fresh Newton search.
                conv_cfg = (
                    drift_detector._calibration.get("convergence", {}) or {}
                )
                tolerance_steps = int(
                    conv_cfg.get("stale_seed_tolerance_steps", 2000)
                )
                pruned = drift_detector.angle_history.invalidate_stale(
                    cam_id=cam_id,
                    reference_position=int(result.reference_position),
                    reference_angle_deg=float(result.reference_angle_deg),
                    steps_per_degree=float(result.steps_per_degree),
                    direction_sign=int(result.direction_sign),
                    tolerance_steps=tolerance_steps,
                )
                remaining = drift_detector.angle_history.count(cam_id)
                logger.info(
                    "auto_calibrator.success",
                    cam_id=cam_id,
                    reference_angle_deg=result.reference_angle_deg,
                    reference_position=result.reference_position,
                    steps_per_degree=round(result.steps_per_degree, 3),
                    direction_sign=result.direction_sign,
                    iterations=result.iterations,
                    angle_history_pruned=len(pruned),
                    angle_history_remaining=remaining,
                )
                print(
                    f"\n✓ Angle calibration SUCCESSFUL for {cam_id}:b — "
                    f"reference {result.reference_angle_deg:+.1f}° "
                    f"at step {result.reference_position:.0f}, "
                    f"{result.steps_per_degree:.2f} steps/° "
                    f"(direction {result.direction_sign:+d}, "
                    f"{result.iterations} iter; "
                    f"history: {remaining} kept, {len(pruned)} pruned)\n",
                    flush=True,
                )
            else:
                logger.warning(
                    "auto_calibrator.failed",
                    cam_id=cam_id,
                    error=result.error,
                    iterations=result.iterations,
                )
                print(
                    f"\n✗ Angle calibration FAILED for {cam_id}:b — "
                    f"{result.error} (after {result.iterations} iter)\n",
                    flush=True,
                )
    finally:
        drive_mgr.autocal_complete.set()
        logger.info("auto_calibrator.complete")


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
    _hist_count = drift_detector.angle_history.count()
    logger.info("angle_history.loaded", total_entries=_hist_count)
    print(
        f"Angle history loaded — {_hist_count} saved angle→position entries",
        flush=True,
    )
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
            tg.create_task(run_startup_autocal(drive_mgr, drive_mgr.drift_detector))
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
    _hist_count = drift_detector.angle_history.count()
    logger.info("angle_history.loaded", total_entries=_hist_count)
    print(
        f"Angle history loaded — {_hist_count} saved angle→position entries",
        flush=True,
    )

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
        asyncio.create_task(run_startup_autocal(drive_mgr, drift_detector))
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
