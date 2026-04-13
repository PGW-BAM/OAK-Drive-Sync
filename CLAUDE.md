# OAK-Drive-Sync — Claude Code Project Instructions

## Project Overview

Synchronized control system for **2× Luxonis OAK-D 4 Pro** cameras with **4× mechanical positioning drives** controlled via Raspberry Pi 5, coordinated over MQTT.

### System Topology

```
┌─────────────────────┐       PoE++ Switch        ┌──────────────────────────┐
│  Windows 11 PC      │◄──────────────────────────►│ OAK-D 4 Pro #1          │
│  - Camera capture    │◄──────────────────────────►│ OAK-D 4 Pro #2          │
│  - Orchestration     │       Ethernet             │                          │
│  - Monitoring UI     │◄──────────────────────────►│ Raspberry Pi 5           │
│  - Email alerts      │       (MQTT over TCP)      │  - 2× GPIO linear motors │
└─────────────────────┘                             │  - 2× Tinkerforge stepper│
                                                    │  - NiceGUI web UI (:8080)│
                                                    │  - Mosquitto broker      │
                                                    │  - MQTT client           │
                                                    └──────────────────────────┘
```

### Drive Hardware

| Drive   | Type          | Control Method                          | Pin/UID |
|---------|---------------|-----------------------------------------|---------|
| cam1:a  | GPIO linear   | Blocking step pulse loop via lgpio      | GPIO 15 |
| cam1:b  | Tinkerforge   | Silent Stepper Bricklet 2.0 via brickd  | UID 2eoB |
| cam2:a  | GPIO linear   | Blocking step pulse loop via lgpio      | GPIO 17 |
| cam2:b  | Tinkerforge   | Silent Stepper Bricklet 2.0 via brickd  | UID 2eoz |

**GPIO motors** use a `time.sleep(0.00001)` pulse loop (10µs HIGH + 10µs LOW) running in a thread. No direction pin — single pin per motor. No limit switches yet — calibration is manual via the GUI.

**Tinkerforge motors** use `set_target_position()` with position range 1500–9500. Bricklet position counter is synced on startup via `set_current_position()`.

### Communication Stack

- **MQTT 5.0** via Mosquitto broker (runs on Pi)
- **DepthAI SDK** for OAK-D camera control (Windows side)
- **lgpio** for GPIO step pulse generation (Pi side)
- **Tinkerforge Python bindings** for Silent Stepper control (Pi side)
- **NiceGUI** for Pi web interface (drive control, calibration, sequences)
- **Email notifications** via SMTP (aiosmtplib)

## Tech Stack

| Component         | Technology                              | Platform   |
|-------------------|-----------------------------------------|------------|
| MQTT Broker       | Mosquitto 2.x                           | RPi 5      |
| Pi Drive Control  | Python 3.11+, lgpio, tinkerforge        | RPi 5      |
| Pi Web GUI        | NiceGUI (port 8080)                     | RPi 5      |
| Camera Control    | Python 3.11+, depthai (stubbed)         | Windows 11 |
| Monitoring UI     | OpenCV overlay + NiceGUI (Phase 2)      | Windows 11 |
| Email Alerts      | aiosmtplib, jinja2 templates            | Windows 11 |
| Package Manager   | uv                                      | Both       |

## Agent Architecture (Claude Code)

This project uses **specialized agents** invoked via `claude --agent <name>`:

| Agent              | Scope                                              |
|--------------------|-----------------------------------------------------|
| `pi-controller`    | Raspberry Pi GPIO drive code, MQTT client, health   |
| `cam-controller`   | DepthAI capture pipeline, MQTT integration          |
| `mqtt-infra`       | Broker config, topic schema, QoS, TLS, reliability  |
| `monitoring`       | Connectivity dashboard, streaming overlay, alerts    |
| `integration-test` | End-to-end test harness, simulation, CI             |

## Memory

This project uses the **`thedotmack/claude-mem`** plugin for persistent memory.
Do NOT create or use `memory.md` files — all persistent context is managed via claude-mem.

Key memory namespaces:
- `project:architecture` — system design decisions
- `project:mqtt-topics` — canonical topic tree
- `project:hardware` — pin mappings, camera serial numbers, network addresses
- `project:issues` — known bugs and workarounds

## Coding Conventions

- Python 3.11+ with type hints everywhere
- `uv` for dependency management (pyproject.toml)
- asyncio-first design; blocking GPIO work runs in threads via `run_in_executor`
- NiceGUI owns the event loop when GUI is enabled; MQTT runs as background task
- Pydantic v2 models for all MQTT message payloads
- Structured logging via `structlog`
- All MQTT topics documented in `config/mqtt_topics.yaml`
- Tests with pytest + pytest-asyncio

## Key Configuration Files

| File                          | Purpose                                      |
|-------------------------------|----------------------------------------------|
| `config/drive_config.yaml`    | Drive hardware: pins, UIDs, speeds, limits   |
| `config/calibration.yaml`     | Saved min/max from manual calibration        |
| `config/positions.yaml`       | Named position presets for sequences         |
| `config/mosquitto.conf`       | Mosquitto drop-in (listener + auth only)     |
| `config/mqtt_topics.yaml`     | Topic schema documentation                   |

## Key Design Decisions

1. **MQTT QoS 1** for drive commands (at-least-once delivery, idempotent moves)
2. **MQTT QoS 0** for health/heartbeat beacons (frequent, loss-tolerant)
3. **Retained messages** for last-known drive positions
4. **Last Will and Testament (LWT)** on both Pi and Windows clients for disconnect detection
5. **JSON payloads** with Pydantic validation on both ends
6. **Settling delay** (configurable, default 150ms) between drive-reached and capture-trigger
7. **Sequence IDs** on every command for correlation and out-of-order detection
8. **NiceGUI as event loop owner** — MQTT and heartbeat run as `app.on_startup` background tasks
9. **GPIO pulses via blocking thread** — `time.sleep(10µs)` loop in `run_in_executor`, not asyncio
10. **Manual calibration** — no limit switches yet; min/max set via GUI and saved to YAML
11. **Resilient drive setup** — one failing drive doesn't crash the whole controller
