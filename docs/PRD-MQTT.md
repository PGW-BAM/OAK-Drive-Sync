# PRD: MQTT-Based Drive/Camera Synchronization System

**Document Version**: 1.0  
**Status**: Draft  
**Owner**: oak-drive-sync project  
**Last Updated**: 2026-03-13

---

## 1. Problem Statement

Two Luxonis OAK-D 4 Pro cameras are mounted on mechanical positioning drives (4 drives total, 2 per camera) controlled by a Raspberry Pi 5 via GPIO. The cameras are connected to a Windows 11 machine via a PoE++ network switch. An image capture application on the Windows machine records images at defined intervals and positions.

Currently there is no coordination layer between the drive positioning (Pi) and the camera capture (Windows). The system needs a reliable, low-latency communication channel to synchronize "move to position → confirm arrival → capture image" workflows, track the health of all components, and alert operators when something goes wrong.

## 2. Goals

### Primary Goals
- **G1**: Reliable command/response coordination between Windows camera controller and Pi drive controller with <50ms messaging latency on LAN
- **G2**: Automated capture sequences — define a list of positions, system works through them autonomously
- **G3**: Real-time connectivity monitoring visible in the camera streaming interface
- **G4**: Email notifications when errors or disconnects occur

### Non-Goals (Phase 1)
- Web-based remote control (future Phase 2)
- Multi-broker clustering or WAN operation
- Machine vision / automated crack detection (separate project)
- Recording video streams (capture is still-image based)

## 3. System Architecture

### 3.1 Component Overview

| Component            | Platform   | Role                                    |
|----------------------|------------|-----------------------------------------|
| Mosquitto Broker     | RPi 5      | MQTT message routing                    |
| Drive Controller     | RPi 5      | GPIO drive control, MQTT client         |
| Camera Controller    | Win 11     | DepthAI capture, orchestration, MQTT    |
| Connectivity Monitor | Win 11     | Health tracking, overlay, email alerts  |

### 3.2 Why MQTT

- **Decoupled architecture**: Pi and Windows communicate asynchronously via topics, no direct socket management
- **QoS guarantees**: QoS 1 ensures commands are delivered at-least-once even through brief network hiccups
- **Retained messages**: New or reconnecting clients immediately get the last known state
- **Last Will and Testament**: Instant disconnect detection without polling
- **Lightweight**: Mosquitto runs comfortably on a Pi 5 alongside the drive control code
- **Battle-tested**: Proven in industrial IoT with similar requirements
- **Existing expertise**: Operator has MQTT experience from ioBroker/Victron ESS energy automation

### 3.3 Network Topology

All devices on the same LAN subnet. The PoE++ switch provides power to cameras and network connectivity to all devices.

```
Windows 11 ──┐
OAK-D #1 ────┤── PoE++ Switch ──── Raspberry Pi 5 (Mosquitto + Drive Controller)
OAK-D #2 ────┘
```

MQTT traffic flows over TCP port 1883 (or 8883 with TLS). Camera data flows over the DepthAI/XLink protocol directly between cameras and Windows PC.

## 4. MQTT Topic Schema

### 4.1 Topic Tree

```yaml
# Commands (Windows → Pi)
cmd/
  drives/
    cam1/
      move          # Payload: MoveCommand
      home          # Payload: HomeCommand
      stop          # Payload: StopCommand
    cam2/
      move
      home
      stop

# Status (Pi → Windows)
status/
  drives/
    cam1/
      position      # Payload: DrivePosition (retained)
    cam2/
      position

# Camera State (Windows internal, published for monitoring)
status/
  cameras/
    cam1/
      state          # Payload: CameraState (retained)
    cam2/
      state

# Health Beacons (bidirectional)
health/
  pi                 # Payload: PiHealth (QoS 0, LWT)
  win_controller     # Payload: WinHealth (QoS 0, LWT)
  cameras/
    cam1             # Payload: CameraHealth (QoS 0)
    cam2

# Error Events
error/
  drives/
    cam1             # Payload: DriveError
    cam2
  cameras/
    cam1             # Payload: CameraError
    cam2
  orchestration/
    timeout          # Payload: OrchestrationError
    sequence_abort

# Monitoring (aggregated by monitor)
monitoring/
  connectivity       # Payload: ConnectivityState (retained)

# Configuration
config/
  sequence/
    active           # Payload: CaptureSequence (retained)
```

### 4.2 QoS Strategy

| Topic Pattern          | QoS | Retained | Rationale                                    |
|------------------------|-----|----------|----------------------------------------------|
| `cmd/drives/+/move`   | 1   | No       | Commands must arrive; not idempotent to retain |
| `cmd/drives/+/home`   | 1   | No       | Same as move                                 |
| `cmd/drives/+/stop`   | 1   | No       | Safety-critical, must arrive                 |
| `status/drives/+/pos` | 1   | Yes      | Late joiners need current position           |
| `status/cameras/+/*`  | 1   | Yes      | Late joiners need current camera state       |
| `health/*`            | 0   | No       | Frequent, loss-tolerant, LWT covers gaps     |
| `error/**`            | 1   | No       | Errors must arrive for alerting              |
| `monitoring/*`        | 1   | Yes      | Dashboard needs current state on connect     |

### 4.3 Message Payloads (Pydantic Models)

```python
# Core command
class MoveCommand(BaseModel):
    sequence_id: str          # UUID for correlation
    drive_axis: Literal["a", "b"]  # Which drive (each cam has 2)
    target_position: float    # Target in drive-native units
    speed: float = 1.0        # 0.0–1.0 normalized
    timestamp: datetime

# Position status
class DrivePosition(BaseModel):
    sequence_id: str | None   # Correlates to command, None if manual
    drive_axis: Literal["a", "b"]
    current_position: float
    target_position: float | None
    state: Literal["idle", "moving", "reached", "fault", "homing"]
    timestamp: datetime

# Health beacon
class PiHealth(BaseModel):
    online: bool
    cpu_temp_c: float
    uptime_s: int
    drive_states: dict[str, str]  # cam_id:axis → state
    timestamp: datetime

# Error event
class DriveError(BaseModel):
    sequence_id: str | None
    drive_axis: Literal["a", "b"]
    error_type: Literal["stall", "limit_switch", "timeout", "gpio_fault"]
    message: str
    timestamp: datetime

# Connectivity aggregate
class ConnectivityState(BaseModel):
    pi_online: bool
    broker_connected: bool
    cameras: dict[str, Literal["online", "offline", "capturing", "error"]]
    drives: dict[str, Literal["idle", "moving", "fault"]]
    last_update: datetime
```

## 5. Core Workflow: Move → Confirm → Capture

### 5.1 Sequence Diagram

```
Windows Controller                    MQTT Broker                    Pi Drive Controller
       │                                  │                                  │
       │  PUBLISH cmd/drives/cam1/move    │                                  │
       │  (QoS 1, MoveCommand)           │                                  │
       │─────────────────────────────────►│                                  │
       │                                  │  DELIVER cmd/drives/cam1/move    │
       │                                  │─────────────────────────────────►│
       │                                  │                                  │
       │                                  │          [GPIO: activate drive]  │
       │                                  │                                  │
       │                                  │  PUBLISH status/drives/cam1/pos  │
       │                                  │  state="moving"                  │
       │                                  │◄─────────────────────────────────│
       │  DELIVER status (moving)         │                                  │
       │◄─────────────────────────────────│                                  │
       │                                  │                                  │
       │          ... drive moving ...    │          ... drive moving ...    │
       │                                  │                                  │
       │                                  │  PUBLISH status/drives/cam1/pos  │
       │                                  │  state="reached"                 │
       │                                  │◄─────────────────────────────────│
       │  DELIVER status (reached)        │                                  │
       │◄─────────────────────────────────│                                  │
       │                                  │                                  │
       │  [wait settling_delay: 150ms]    │                                  │
       │                                  │                                  │
       │  [TRIGGER: DepthAI capture]      │                                  │
       │  [STORE: image + metadata]       │                                  │
       │                                  │                                  │
       │  → Next position in sequence     │                                  │
```

### 5.2 Timeout and Error Handling

| Condition                  | Timeout | Action                                              |
|----------------------------|---------|------------------------------------------------------|
| Move command no ACK        | 5s      | Retry once, then escalate to `error/orchestration`  |
| Drive not reached target   | 30s     | Publish stop, report to `error/drives/{cam_id}`     |
| Camera capture failure     | 10s     | Retry once, then skip position, log error           |
| Pi heartbeat lost          | 6s      | Monitor flags offline, alert if >10s                |
| Camera health lost         | 6s      | Monitor flags offline, alert if >15s                |
| Broker connection lost     | —       | Auto-reconnect with exponential backoff (1s–30s)    |

### 5.3 Settling Time

Mechanical drives introduce vibration. After the Pi reports `state="reached"`, the Windows controller waits a **configurable settling delay** (default: 150ms, range: 50–500ms) before triggering the camera capture. This value should be tuned per-installation based on drive mass and speed.

## 6. Camera-MQTT Integration Strategy

### 6.1 How DepthAI and MQTT Coexist

The DepthAI SDK manages the camera pipeline (color stream, depth stream, encoding, capture trigger) entirely on the Windows machine over the PoE link. MQTT has nothing to do with the camera data path — it only coordinates *when* to capture.

Architecture within the Windows controller:

```
┌─────────────────────────────────────────────┐
│  CameraController (asyncio)                 │
│                                             │
│  ┌──────────────┐    ┌───────────────────┐  │
│  │ MQTTClient   │    │ DepthAI Pipeline  │  │
│  │              │    │                   │  │
│  │ subscribes:  │    │ cam1 = dai.Device  │  │
│  │  status/+    │    │ cam2 = dai.Device  │  │
│  │  health/+    │    │                   │  │
│  │              │    │ .getOutputQueue()  │  │
│  │ publishes:   │    │ .capture()         │  │
│  │  cmd/+       │    │                   │  │
│  └──────┬───────┘    └───────┬───────────┘  │
│         │                    │              │
│         └──────┬─────────────┘              │
│                │                            │
│         ┌──────▼───────┐                    │
│         │ Orchestrator │                    │
│         │ State Machine│                    │
│         └──────────────┘                    │
│                                             │
│  ┌──────────────┐    ┌───────────────────┐  │
│  │ StreamViewer  │    │ MonitorOverlay    │  │
│  │ (preview)     │◄───│ (connectivity)    │  │
│  └──────────────┘    └───────────────────┘  │
└─────────────────────────────────────────────┘
```

### 6.2 Key Integration Points

1. **Camera discovery**: DepthAI discovers cameras by MxID or IP on the PoE subnet. The system maps `cam_id` (e.g., "cam1") to DepthAI MxID in `config/camera_config.yaml`. This mapping is also used in all MQTT topics.

2. **Capture trigger**: The orchestrator calls `queue.get()` on the DepthAI still capture queue only after MQTT confirms the drive has reached position and the settling delay has elapsed. This is a software trigger — the pipeline is always running, but frames are only saved at the right moment.

3. **Camera health → MQTT**: The Windows controller monitors DepthAI device connectivity (ping, `dai.Device.getConnectedCameras()`) and publishes to `health/cameras/{cam_id}`. If a camera disconnects at the USB/PoE level, this is detected and reported via MQTT to the monitoring system.

4. **Preview stream + overlay**: DepthAI provides preview frames via output queues. The monitoring overlay composites connectivity indicators onto these frames before displaying them in an OpenCV window. This is purely local to the Windows machine — the overlay is not transmitted over MQTT.

### 6.3 Dual-Camera Concurrency

The two cameras can operate concurrently if their drives are independent (which they are — each camera has its own 2 drives). The orchestrator can:

- **Sequential mode**: Complete all positions for cam1, then cam2 (simpler, safer)
- **Parallel mode**: Move both camera drives simultaneously, capture whichever is ready first (faster, requires careful state tracking)

The mode is configurable in the capture sequence definition.

## 7. Connectivity Monitoring & Overlay

### 7.1 What Is Tracked

| Component       | Detection Method              | Healthy Threshold | Alert Threshold |
|-----------------|-------------------------------|-------------------|-----------------|
| Pi controller   | MQTT heartbeat (2s interval)  | <6s since last    | >10s            |
| MQTT broker     | Client connection state       | Connected         | >10s disconnect |
| Camera 1        | DepthAI device status + MQTT  | Online            | >15s offline    |
| Camera 2        | DepthAI device status + MQTT  | Online            | >15s offline    |
| Drive cam1/a    | Position status messages      | idle or moving    | fault state     |
| Drive cam1/b    | Position status messages      | idle or moving    | fault state     |
| Drive cam2/a    | Position status messages      | idle or moving    | fault state     |
| Drive cam2/b    | Position status messages      | idle or moving    | fault state     |

### 7.2 Streaming Overlay Layout

```
┌──────────────────────────────────────────────┐
│ Camera 1 Preview                             │
│                                              │
│  ┌─ Status Bar (top) ──────────────────────┐ │
│  │ 🟢 Pi  🟢 Broker  🟢 Cam1  🟡 Cam2     │ │
│  │ Drives: A=45.2° B=12.0mm               │ │
│  │ Sequence: 14/50  ▶ Running              │ │
│  └─────────────────────────────────────────┘ │
│                                              │
│           [camera image content]             │
│                                              │
│  ┌─ Error Bar (bottom, shown on error) ────┐ │
│  │ ⚠ Drive cam2/a: stall detected (12s ago)│ │
│  └─────────────────────────────────────────┘ │
└──────────────────────────────────────────────┘
```

- Status indicators: filled circles with color (green/yellow/red)
- Semi-transparent background (alpha ~0.6) so camera content is still visible
- Toggle with `H` key
- Updates at 1 Hz (not every frame — performance)

### 7.3 Uptime History

SQLite database (`data/monitoring.db`) stores:

```sql
CREATE TABLE connectivity_log (
    id INTEGER PRIMARY KEY,
    component TEXT NOT NULL,         -- 'pi', 'broker', 'cam1', 'cam2', 'drive_cam1_a', etc.
    state TEXT NOT NULL,             -- 'online', 'offline', 'fault', etc.
    timestamp DATETIME NOT NULL,
    duration_s REAL                  -- filled on state change (how long was previous state)
);

CREATE TABLE alert_log (
    id INTEGER PRIMARY KEY,
    alert_type TEXT NOT NULL,
    component TEXT NOT NULL,
    message TEXT,
    email_sent BOOLEAN DEFAULT FALSE,
    timestamp DATETIME NOT NULL
);
```

## 8. Email Notification System

### 8.1 Configuration

In `config/monitoring_config.yaml`:

```yaml
alerts:
  enabled: true
  email: ""  # Operator fills this in, or sets via UI prompt at first run
  smtp:
    host: "smtp.example.com"
    port: 587
    use_tls: true
    username: "alerts@example.com"
    password: ""   # Or use env var SMTP_PASSWORD
  thresholds:
    pi_offline_s: 10
    camera_offline_s: 15
    broker_offline_s: 10
    drive_fault: true
    sequence_abort: true
  dedup_window_s: 300  # Don't send same alert type more than once per 5 min
```

### 8.2 Alert Types

| Alert                | Trigger                           | Severity |
|----------------------|-----------------------------------|----------|
| `pi_offline`         | Pi heartbeat lost > threshold     | Critical |
| `camera_offline`     | Camera unreachable > threshold    | High     |
| `broker_offline`     | MQTT connection lost > threshold  | Critical |
| `drive_fault`        | Any drive reports fault state     | High     |
| `sequence_aborted`   | Capture sequence stopped on error | Medium   |
| `capture_failure`    | Camera capture failed after retry | Medium   |

### 8.3 Email Content

Subject: `[OAK-Drive-Sync] {severity}: {alert_type} — {component}`

Body (Jinja2 template) includes:
- Alert description
- Component affected
- Timestamp
- Current system state snapshot (all components)
- Last 5 error log entries
- Suggested remediation steps

### 8.4 First-Run Email Setup

On first startup, if `alerts.email` is empty, the monitoring system:
1. Prints a prompt to console: "Enter email address for error notifications (or press Enter to skip):"
2. Validates the input (basic format check)
3. Sends a test email: "OAK-Drive-Sync alert system configured successfully"
4. Saves to config file

## 9. Reliability Requirements

| Requirement                          | Target         |
|--------------------------------------|----------------|
| MQTT message delivery (QoS 1)       | 99.9% on LAN   |
| Disconnect detection latency         | <6s             |
| Auto-reconnect after network glitch  | <5s             |
| False positive alert rate            | <1/day          |
| Capture sequence completion rate     | >99% (with retries) |
| Overlay rendering overhead           | <2ms/frame      |

## 10. Phasing

### Phase 1 (MVP)
- MQTT broker on Pi with basic config
- Drive controller with GPIO + MQTT
- Camera controller with DepthAI + MQTT orchestration
- Connectivity monitoring with streaming overlay
- Email alerts for critical errors
- Single-camera sequential capture sequences

### Phase 2
- Dual-camera parallel mode
- NiceGUI web dashboard for monitoring
- TLS encryption on MQTT
- Capture sequence editor UI
- Historical uptime analytics

### Phase 3
- Integration with fatigue crack analysis pipeline
- REST API for remote control
- Multi-broker failover
- Webhook notifications (Slack, Matrix)
