"""
Monitoring System — Connectivity Tracking, Streaming Overlay, Email Alerts

Subscribes to all health/status/error topics, maintains a real-time
connectivity model, renders an overlay on camera preview streams,
and sends email alerts on failures.
"""

from __future__ import annotations

import asyncio
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import aiosqlite
import click
import structlog
import yaml
from jinja2 import Environment, FileSystemLoader

from src.shared.models import (
    AlertEvent,
    AlertSeverity,
    CameraState,
    ConnectivityState,
    DriveState,
)
from src.shared.mqtt_client import MQTTClient
from src.shared.mqtt_topics import (
    MONITORING_CONNECTIVITY,
    SUB_ALL_ERRORS,
    SUB_ALL_HEALTH,
    SUB_ALL_DRIVE_STATUS,
    SUB_ALL_CAMERA_STATUS,
)

logger = structlog.get_logger()


class ConnectivityTracker:
    """Tracks the online/offline state of all system components."""

    def __init__(self, config: dict) -> None:
        self.config = config
        thresholds = config.get("connectivity", {}).get("alert_thresholds", {})
        self.pi_offline_threshold = thresholds.get("pi_offline_s", 10)
        self.camera_offline_threshold = thresholds.get("camera_offline_s", 15)
        self.broker_offline_threshold = thresholds.get("broker_offline_s", 10)

        self.last_pi_heartbeat: float = 0.0
        self.last_camera_heartbeats: dict[str, float] = {}
        self.camera_states: dict[str, CameraState] = {}
        self.drive_states: dict[str, DriveState] = {}
        self.broker_connected: bool = False

    @property
    def pi_online(self) -> bool:
        if self.last_pi_heartbeat == 0.0:
            return False
        offline_threshold = self.config.get("connectivity", {}).get("offline_threshold_s", 6)
        return (time.monotonic() - self.last_pi_heartbeat) < offline_threshold

    def camera_online(self, cam_id: str) -> bool:
        last = self.last_camera_heartbeats.get(cam_id, 0.0)
        if last == 0.0:
            return False
        return (time.monotonic() - last) < 6.0

    def get_state(self) -> ConnectivityState:
        cameras = {}
        for cam_id in self.last_camera_heartbeats:
            if self.camera_online(cam_id):
                cameras[cam_id] = self.camera_states.get(cam_id, CameraState.ONLINE)
            else:
                cameras[cam_id] = CameraState.OFFLINE

        return ConnectivityState(
            pi_online=self.pi_online,
            broker_connected=self.broker_connected,
            cameras=cameras,
            drives=self.drive_states.copy(),
        )

    async def handle_health(self, topic_str: str, payload: dict) -> None:
        """Process incoming health beacons."""
        now = time.monotonic()

        if topic_str == "health/pi":
            self.last_pi_heartbeat = now
            drive_states = payload.get("drive_states", {})
            for k, v in drive_states.items():
                self.drive_states[k] = DriveState(v)

        elif topic_str.startswith("health/cameras/"):
            cam_id = topic_str.split("/")[2]
            self.last_camera_heartbeats[cam_id] = now

        elif topic_str == "health/win_controller":
            pass  # Tracked implicitly by broker connection

    async def handle_drive_status(self, topic_str: str, payload: dict) -> None:
        parts = topic_str.split("/")
        cam_id = parts[2]
        axis = payload.get("drive_axis", "a")
        state = payload.get("state", "idle")
        self.drive_states[f"{cam_id}:{axis}"] = DriveState(state)

    async def handle_camera_status(self, topic_str: str, payload: dict) -> None:
        parts = topic_str.split("/")
        cam_id = parts[2]
        state = payload.get("state", "online")
        self.camera_states[cam_id] = CameraState(state)


class AlertManager:
    """Manages email alerts with deduplication and rate limiting."""

    def __init__(self, config: dict) -> None:
        self.config = config
        alert_cfg = config.get("alerts", {})
        self.enabled = alert_cfg.get("enabled", False)
        self.email = alert_cfg.get("email", "")
        self.dedup_window = alert_cfg.get("dedup_window_s", 300)
        self.max_per_hour = alert_cfg.get("max_alerts_per_hour", 20)

        self.smtp_cfg = alert_cfg.get("smtp", {})
        self._last_sent: dict[str, float] = {}  # alert_type → last send timestamp
        self._sent_count_hour: int = 0
        self._hour_start: float = time.monotonic()

        # Jinja2 templates
        template_dir = Path("config/email_templates")
        if template_dir.exists():
            self.jinja_env = Environment(loader=FileSystemLoader(str(template_dir)))
        else:
            self.jinja_env = None

        self.fallback_log = Path(config.get("fallback_log", "logs/unsent_alerts.jsonl"))

    def should_send(self, alert_type: str) -> bool:
        if not self.enabled or not self.email:
            return False

        now = time.monotonic()

        # Rate limit
        if now - self._hour_start > 3600:
            self._sent_count_hour = 0
            self._hour_start = now
        if self._sent_count_hour >= self.max_per_hour:
            logger.warning("alerts.rate_limited")
            return False

        # Dedup
        last = self._last_sent.get(alert_type, 0.0)
        if now - last < self.dedup_window:
            logger.debug("alerts.dedup_suppressed", alert_type=alert_type)
            return False

        return True

    async def send_alert(self, alert: AlertEvent) -> None:
        if not self.should_send(alert.alert_type):
            return

        self._last_sent[alert.alert_type] = time.monotonic()
        self._sent_count_hour += 1

        try:
            body = self._render_email(alert)
            await self._send_smtp(
                subject=f"[OAK-Drive-Sync] {alert.severity.value.upper()}: "
                        f"{alert.alert_type} — {alert.component}",
                body=body,
            )
            logger.info("alert.sent", type=alert.alert_type, to=self.email)

        except Exception as e:
            logger.error("alert.send_failed", error=str(e))
            await self._log_fallback(alert)

    def _render_email(self, alert: AlertEvent) -> str:
        if self.jinja_env:
            try:
                template = self.jinja_env.get_template("alert.txt.j2")
                return template.render(
                    severity=alert.severity.value,
                    alert_type=alert.alert_type,
                    component=alert.component,
                    message=alert.message,
                    timestamp=alert.timestamp.isoformat(),
                    system_state=alert.system_state,
                )
            except Exception:
                pass

        # Fallback plain text
        return (
            f"Alert: {alert.alert_type}\n"
            f"Severity: {alert.severity.value}\n"
            f"Component: {alert.component}\n"
            f"Message: {alert.message}\n"
            f"Time: {alert.timestamp.isoformat()}\n"
        )

    async def _send_smtp(self, subject: str, body: str) -> None:
        import os
        import aiosmtplib

        password = self.smtp_cfg.get("password") or os.environ.get("OAK_SMTP_PASSWORD", "")

        await aiosmtplib.send(
            message=self._build_mime(subject, body),
            hostname=self.smtp_cfg.get("host", ""),
            port=self.smtp_cfg.get("port", 587),
            username=self.smtp_cfg.get("username", ""),
            password=password,
            use_tls=self.smtp_cfg.get("use_tls", True),
        )

    def _build_mime(self, subject: str, body: str):
        from email.mime.text import MIMEText

        msg = MIMEText(body)
        msg["Subject"] = subject
        msg["From"] = self.smtp_cfg.get("username", "noreply@oak-drive-sync.local")
        msg["To"] = self.email
        return msg

    async def _log_fallback(self, alert: AlertEvent) -> None:
        self.fallback_log.parent.mkdir(parents=True, exist_ok=True)
        with open(self.fallback_log, "a") as f:
            f.write(alert.model_dump_json() + "\n")
        logger.warning("alert.logged_to_fallback", path=str(self.fallback_log))


class OverlayRenderer:
    """Renders connectivity overlay onto camera preview frames."""

    def __init__(self, config: dict) -> None:
        self.config = config.get("overlay", {})
        self.enabled = self.config.get("enabled", True)
        self.opacity = self.config.get("opacity", 0.6)
        self.font_scale = self.config.get("font_scale", 0.5)
        self.visible = True
        self._last_error: str = ""
        self._last_error_time: float = 0.0

    def set_error(self, msg: str) -> None:
        self._last_error = msg
        self._last_error_time = time.monotonic()

    def render(self, frame, state: ConnectivityState, step_info: str = "") -> Any:
        """Overlay connectivity info onto an OpenCV frame. Returns modified frame."""
        if not self.enabled or not self.visible or frame is None:
            return frame

        try:
            import cv2
            import numpy as np

            h, w = frame.shape[:2]
            overlay = frame.copy()

            # Status bar background
            bar_h = 60
            cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
            frame = cv2.addWeighted(overlay, self.opacity, frame, 1 - self.opacity, 0)

            y = 20
            x = 10

            # Connection dots
            indicators = [
                ("Pi", state.pi_online),
                ("MQTT", state.broker_connected),
            ]
            for cam_id, cam_state in state.cameras.items():
                indicators.append((cam_id, cam_state != CameraState.OFFLINE))

            for label, online in indicators:
                color = (0, 200, 0) if online else (0, 0, 200)
                cv2.circle(frame, (x + 6, y), 6, color, -1)
                cv2.putText(
                    frame, label, (x + 16, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (255, 255, 255), 1,
                )
                x += 90

            # Drive positions
            y2 = 45
            drive_text = "  ".join(
                f"{k}={v.value}" for k, v in state.drives.items()
            )
            cv2.putText(
                frame, f"Drives: {drive_text}", (10, y2),
                cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (200, 200, 200), 1,
            )

            # Sequence progress
            if step_info:
                cv2.putText(
                    frame, step_info, (w - 200, y2),
                    cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (200, 200, 200), 1,
                )

            # Error bar (bottom, auto-dismiss after 30s)
            if self._last_error and (time.monotonic() - self._last_error_time < 30):
                err_y = h - 30
                cv2.rectangle(overlay, (0, err_y - 5), (w, h), (0, 0, 80), -1)
                frame_bottom = cv2.addWeighted(overlay, self.opacity, frame, 1 - self.opacity, 0)
                frame[err_y - 5:h, :] = frame_bottom[err_y - 5:h, :]
                cv2.putText(
                    frame, f"⚠ {self._last_error}", (10, err_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (100, 100, 255), 1,
                )

            return frame

        except ImportError:
            logger.warning("overlay.opencv_not_available")
            return frame

    def toggle(self) -> None:
        self.visible = not self.visible


class MonitoringDatabase:
    """SQLite-backed connectivity and alert history."""

    def __init__(self, db_path: str) -> None:
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._db: aiosqlite.Connection | None = None

    async def initialize(self) -> None:
        self._db = await aiosqlite.connect(str(self.db_path))
        await self._db.executescript("""
            CREATE TABLE IF NOT EXISTS connectivity_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                component TEXT NOT NULL,
                state TEXT NOT NULL,
                timestamp DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
                duration_s REAL
            );
            CREATE TABLE IF NOT EXISTS alert_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                alert_type TEXT NOT NULL,
                component TEXT NOT NULL,
                message TEXT,
                email_sent BOOLEAN DEFAULT FALSE,
                timestamp DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
            );
            CREATE INDEX IF NOT EXISTS idx_conn_comp ON connectivity_log(component, timestamp);
            CREATE INDEX IF NOT EXISTS idx_alert_ts ON alert_log(timestamp);
        """)
        await self._db.commit()

    async def log_state_change(self, component: str, new_state: str) -> None:
        if self._db is None:
            return
        await self._db.execute(
            "INSERT INTO connectivity_log (component, state, timestamp) VALUES (?, ?, ?)",
            (component, new_state, datetime.now(timezone.utc).isoformat()),
        )
        await self._db.commit()

    async def log_alert(self, alert: AlertEvent, email_sent: bool) -> None:
        if self._db is None:
            return
        await self._db.execute(
            "INSERT INTO alert_log (alert_type, component, message, email_sent, timestamp) "
            "VALUES (?, ?, ?, ?, ?)",
            (
                alert.alert_type,
                alert.component,
                alert.message,
                email_sent,
                alert.timestamp.isoformat(),
            ),
        )
        await self._db.commit()

    async def close(self) -> None:
        if self._db:
            await self._db.close()


async def monitoring_loop(
    mqtt: MQTTClient,
    tracker: ConnectivityTracker,
    alert_mgr: AlertManager,
    overlay: OverlayRenderer,
    db: MonitoringDatabase,
) -> None:
    """Main monitoring tick — runs every second."""
    prev_state: dict[str, bool] = {}

    while True:
        state = tracker.get_state()

        # Publish aggregated state
        await mqtt.publish(MONITORING_CONNECTIVITY, state, qos=1, retain=True)

        # Check for state transitions → alerts
        checks = [
            ("pi", state.pi_online, tracker.pi_offline_threshold, "pi_offline", AlertSeverity.CRITICAL),
            ("broker", state.broker_connected, tracker.broker_offline_threshold, "broker_offline", AlertSeverity.CRITICAL),
        ]
        for cam_id, cam_state in state.cameras.items():
            online = cam_state != CameraState.OFFLINE
            checks.append(
                (cam_id, online, tracker.camera_offline_threshold, "camera_offline", AlertSeverity.HIGH)
            )

        for comp, is_online, threshold, alert_type, severity in checks:
            was_online = prev_state.get(comp, True)
            if was_online and not is_online:
                # Went offline
                await db.log_state_change(comp, "offline")
                alert = AlertEvent(
                    alert_type=alert_type,
                    severity=severity,
                    component=comp,
                    message=f"{comp} went offline",
                    system_state=state,
                )
                await alert_mgr.send_alert(alert)
                await db.log_alert(alert, alert_mgr.should_send(alert_type))
                overlay.set_error(f"{comp} OFFLINE")

            elif not was_online and is_online:
                await db.log_state_change(comp, "online")

            prev_state[comp] = is_online

        # Check drive faults
        for drive_key, drive_state in state.drives.items():
            if drive_state == DriveState.FAULT:
                alert = AlertEvent(
                    alert_type="drive_fault",
                    severity=AlertSeverity.HIGH,
                    component=drive_key,
                    message=f"Drive {drive_key} reports fault",
                    system_state=state,
                )
                await alert_mgr.send_alert(alert)
                overlay.set_error(f"Drive {drive_key} FAULT")

        await asyncio.sleep(1.0)


async def error_handler(
    topic_str: str,
    payload: dict,
    alert_mgr: AlertManager,
    overlay: OverlayRenderer,
    tracker: ConnectivityTracker,
) -> None:
    """Handle error topic messages."""
    msg = payload.get("message", "Unknown error")
    component = topic_str.replace("error/", "")
    overlay.set_error(f"{component}: {msg}")

    alert = AlertEvent(
        alert_type=payload.get("error_type", "unknown"),
        severity=AlertSeverity.HIGH,
        component=component,
        message=msg,
        system_state=tracker.get_state(),
    )
    await alert_mgr.send_alert(alert)


async def email_setup_prompt(config: dict, config_path: str) -> str:
    """Prompt for email address if not configured."""
    alerts_cfg = config.get("alerts", {})
    email = alerts_cfg.get("email", "")

    if not email:
        print("\n╔══════════════════════════════════════════════╗")
        print("║  OAK-Drive-Sync — Email Alert Setup          ║")
        print("╚══════════════════════════════════════════════╝")
        email_input = input("Enter email for error notifications (or Enter to skip): ").strip()

        if email_input and "@" in email_input:
            # Save to config
            config.setdefault("alerts", {})["email"] = email_input
            with open(config_path, "w") as f:
                yaml.dump(config, f, default_flow_style=False)
            print(f"✓ Email saved: {email_input}")
            return email_input
        else:
            print("→ Email alerts disabled. You can configure later in monitoring_config.yaml")
            return ""

    return email


async def run_monitoring(config_path: str) -> None:
    """Main async entry point."""
    with open(config_path) as f:
        config = yaml.safe_load(f)

    # Email setup on first run
    email = await email_setup_prompt(config, config_path)
    if email:
        config.setdefault("alerts", {})["email"] = email

    # Components
    mqtt_host = config.get("mqtt", {}).get("broker_host", "localhost")
    mqtt_port = config.get("mqtt", {}).get("broker_port", 1883)

    mqtt = MQTTClient(
        broker_host=mqtt_host,
        broker_port=mqtt_port,
        client_id="oak-monitor",
        lwt_topic="health/monitor",
        lwt_payload={"online": False},
    )

    tracker = ConnectivityTracker(config)
    alert_mgr = AlertManager(config)
    overlay = OverlayRenderer(config)
    db = MonitoringDatabase(config.get("database", {}).get("path", "data/monitoring.db"))

    await db.initialize()

    # MQTT subscriptions
    mqtt.subscribe(SUB_ALL_HEALTH, tracker.handle_health)
    mqtt.subscribe(SUB_ALL_DRIVE_STATUS, tracker.handle_drive_status)
    mqtt.subscribe(SUB_ALL_CAMERA_STATUS, tracker.handle_camera_status)
    mqtt.subscribe(
        SUB_ALL_ERRORS,
        lambda t, p: error_handler(t, p, alert_mgr, overlay, tracker),
    )

    # Set broker connected state based on MQTT client
    async def track_broker_state() -> None:
        while True:
            tracker.broker_connected = mqtt.connected
            await asyncio.sleep(1.0)

    try:
        async with asyncio.TaskGroup() as tg:
            tg.create_task(mqtt.run())
            tg.create_task(track_broker_state())
            tg.create_task(monitoring_loop(mqtt, tracker, alert_mgr, overlay, db))
    finally:
        await db.close()


@click.group()
def cli() -> None:
    """OAK-Drive-Sync Monitoring System."""
    structlog.configure(
        processors=[
            structlog.dev.ConsoleRenderer(),
        ],
    )


@cli.command()
@click.option("--config", default="config/monitoring_config.yaml", help="Monitoring config file")
def run(config: str) -> None:
    """Start the monitoring system."""
    logger.info("monitoring.starting", config=config)
    asyncio.run(run_monitoring(config))


if __name__ == "__main__":
    cli()
