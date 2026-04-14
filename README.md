# OAK-Drive-Sync

Synchronized capture system for **2x Luxonis OAK-D 4 Pro** cameras with MQTT-coordinated mechanical positioning drives on a Raspberry Pi 5.

## Architecture

```
Windows 11 (Camera Control + Orchestrator + Monitor)
    │
    │  MQTT (TCP 1883)
    ▼
PoE++ Switch ──► OAK-D 4 Pro × 2
    │
    ▼
Raspberry Pi 5 (Mosquitto Broker + Drive Controller + Web GUI)
    │
    ├──► 2× GPIO linear motors (pins 15, 17)
    └──► 2× Tinkerforge Silent Stepper Bricklet 2.0
```

### Drive Hardware

| Drive   | Type          | Control                     | Range        |
|---------|---------------|-----------------------------|--------------|
| cam1:a  | GPIO linear   | PUL=GPIO 14, DIR=GPIO 15    | 0 – configurable |
| cam1:b  | Tinkerforge   | Silent Stepper (UID: 2eoB)  | 0 – calibrated   |
| cam2:a  | GPIO linear   | PUL=GPIO 15, DIR=GPIO 17    | 0 – configurable |
| cam2:b  | Tinkerforge   | Silent Stepper (UID: 2eoz)  | 0 – calibrated   |

## Quick Start — Raspberry Pi

### One-Click Install

```bash
git clone https://github.com/PGW-BAM/OAK-Drive-Sync.git
cd OAK-Drive-Sync
chmod +x install.sh
./install.sh
```

This installs everything: system packages, Mosquitto, Tinkerforge Brick Daemon, Python dependencies, GPIO permissions, and the systemd service.

### Run Manually

```bash
uv run oak-pi run
```

The web GUI is then available at `http://<pi-ip>:8080`.

### Run as System Service (auto-start on boot)

```bash
sudo systemctl start oak-drive-controller
sudo systemctl status oak-drive-controller
journalctl -u oak-drive-controller -f
```

### Run Without GUI

```bash
uv run oak-pi run --no-gui
```

## Web GUI

Open `http://<pi-ip>:8080` in any browser on the same network.

| Tab           | Purpose                                              |
|---------------|------------------------------------------------------|
| **Dashboard** | Live drive states, positions, MQTT status             |
| **Manual**    | Jog drives, go-to position, home, emergency stop     |
| **Positions** | Define named presets, capture current position        |
| **Sequences** | Select positions, set dwell time, run cyclic captures |
| **Calibration** | Jog to physical endpoints, set min/max, save        |
| **Settings**  | Drive limits, MQTT status, settling delay             |

### Calibration Workflow

Since there are no limit switches yet, calibrate manually:

1. Go to the **Calibration** tab
2. For each drive, jog to the **lower physical endpoint** → click **Set as Min**
3. Jog to the **upper physical endpoint** → click **Set as Max**
4. Click **Save Calibration** — saved to `config/calibration.yaml`, auto-loaded on next startup

### Capture Sequences

1. Define positions on the **Positions** tab (or capture current drive positions)
2. Go to **Sequences**, select the positions, set dwell time and repeat count
3. Click **Start Sequence** — drives move to each position, settle, notify cameras via MQTT, pause, repeat

## Quick Start — Windows

```bash
cd oak-drive-sync
uv sync --extra camera

# Edit config/camera_config.yaml with your camera MxIDs and Pi IP address
uv run oak-cam run --config config/camera_config.yaml
```

### Monitoring (Windows)

```bash
uv run oak-monitor run --config config/monitoring_config.yaml
```

## MQTT Protocol

The Windows side sends drive commands, the Pi executes them and reports back:

| Direction | Topic                           | Payload         | QoS |
|-----------|---------------------------------|-----------------|-----|
| Win → Pi  | `cmd/drives/{cam_id}/move`      | MoveCommand     | 1   |
| Win → Pi  | `cmd/drives/{cam_id}/home`      | HomeCommand     | 1   |
| Win → Pi  | `cmd/drives/{cam_id}/stop`      | StopCommand     | 1   |
| Pi → Win  | `status/drives/{cam_id}/position` | DrivePosition | 1 (retained) |
| Pi → Win  | `health/pi`                     | PiHealth        | 0   |
| Pi → Win  | `error/drives/{cam_id}`         | DriveError      | 1   |

**Key flow:** Windows sends move → Pi moves drives → Pi publishes `state="reached"` → Windows captures image.

## Configuration Files

| File                            | Purpose                                |
|---------------------------------|----------------------------------------|
| `config/drive_config.yaml`      | Drive hardware: GPIO pins, Tinkerforge UIDs, speeds, limits |
| `config/calibration.yaml`       | Saved drive min/max from calibration (auto-generated) |
| `config/positions.yaml`         | Named position presets (auto-generated from GUI) |
| `config/camera_config.yaml`     | Camera MxIDs, capture settings         |
| `config/monitoring_config.yaml` | Alert thresholds, SMTP, overlay        |
| `config/mosquitto.conf`         | Mosquitto broker drop-in config        |
| `config/mqtt_topics.yaml`       | Topic schema documentation             |
| `config/sequences/*.yaml`       | Capture sequence definitions           |

## Project Structure

```
src/
├── pi_controller/              # Raspberry Pi side
│   ├── main.py                 # Entry point, DriveManager, MQTT, CLI
│   ├── gui.py                  # NiceGUI web interface
│   └── drives/
│       ├── base.py             # Abstract drive interface
│       ├── gpio_drive.py       # GPIO stepper (time.sleep pulse loop)
│       └── tinkerforge_drive.py # Tinkerforge Silent Stepper Bricklet 2.0
├── win_controller/             # Windows camera side
│   └── main.py                 # DepthAI orchestrator (stubbed)
├── shared/                     # Shared between Pi and Windows
│   ├── models.py               # Pydantic v2 payload models
│   ├── mqtt_client.py          # Async MQTT client with auto-reconnect
│   └── mqtt_topics.py          # Canonical topic definitions
└── monitoring/                 # Connectivity monitoring + alerts
    └── main.py                 # Overlay, email alerts, SQLite logging
```

## Documentation

- **[PRD-MQTT.md](docs/PRD-MQTT.md)** — Full product requirements
- **[RASPBERRY_PI_IMPLEMENTATION.md](https://github.com/PGW-BAM/OAK-Drive-Sync/wiki)** — Pi implementation guide (in companion repo)

## Claude Code Agents

| Agent              | Command                           | Domain                        |
|--------------------|-----------------------------------|-------------------------------|
| `pi-controller`    | `claude --agent pi-controller`    | GPIO drives, Pi MQTT client   |
| `cam-controller`   | `claude --agent cam-controller`   | DepthAI, orchestration        |
| `mqtt-infra`       | `claude --agent mqtt-infra`       | Broker, topics, shared models |
| `monitoring`       | `claude --agent monitoring`       | Overlay, alerts, dashboard    |
| `integration-test` | `claude --agent integration-test` | E2E tests, simulators         |
