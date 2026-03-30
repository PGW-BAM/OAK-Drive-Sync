# OAK-Drive-Sync

Synchronized capture system for **Luxonis OAK-D 4 Pro** cameras with MQTT-coordinated mechanical positioning drives on a Raspberry Pi 5.

## Architecture

```
Windows 11 (Camera + Orchestrator + Monitor)
    │
    │  MQTT (TCP 1883)
    ▼
PoE++ Switch ──► OAK-D 4 Pro × 2
    │
    ▼
Raspberry Pi 5 (Mosquitto Broker + Drive Controller)
    │
    └──► 4× GPIO-driven positioning drives (2 per camera)
```

## Quick Start

### 1. Raspberry Pi Setup

```bash
# Install Mosquitto
sudo apt install mosquitto mosquitto-clients
sudo cp config/mosquitto.conf /etc/mosquitto/conf.d/oak-drive-sync.conf
sudo systemctl restart mosquitto

# Install drive controller
cd ~/oak-drive-sync
uv sync --extra pi
uv run oak-pi run --broker-host localhost
```

### 2. Windows Setup

```bash
cd oak-drive-sync
uv sync --extra camera

# Edit config/camera_config.yaml with your camera MxIDs and Pi IP address

# Start camera controller with a capture sequence
uv run oak-cam run --config config/camera_config.yaml --sequence config/sequences/example_grid_scan.yaml
```

### 3. Monitoring (Windows)

```bash
# First run will prompt for alert email address
uv run oak-monitor run --config config/monitoring_config.yaml
```

## Claude Code Agents

This project is designed for multi-agent development with Claude Code:

| Agent              | Command                               | Domain                        |
|--------------------|---------------------------------------|-------------------------------|
| `pi-controller`    | `claude --agent pi-controller`        | GPIO drives, Pi MQTT client   |
| `cam-controller`   | `claude --agent cam-controller`       | DepthAI, orchestration        |
| `mqtt-infra`       | `claude --agent mqtt-infra`           | Broker, topics, shared models |
| `monitoring`       | `claude --agent monitoring`           | Overlay, alerts, dashboard    |
| `integration-test` | `claude --agent integration-test`     | E2E tests, simulators         |

Memory is managed via the `thedotmack/claude-mem` plugin — no `memory.md` files.

## Configuration Files

| File                              | Purpose                              |
|-----------------------------------|--------------------------------------|
| `config/drive_pinmap.yaml`        | GPIO pin assignments per drive       |
| `config/camera_config.yaml`       | Camera MxIDs, capture settings       |
| `config/monitoring_config.yaml`   | Alert thresholds, SMTP, overlay      |
| `config/mosquitto.conf`           | Mosquitto broker settings            |
| `config/mqtt_topics.yaml`         | Topic schema documentation           |
| `config/sequences/*.yaml`         | Capture sequence definitions         |

## Documentation

- **[PRD-MQTT.md](docs/PRD-MQTT.md)** — Full product requirements for the MQTT coordination system
