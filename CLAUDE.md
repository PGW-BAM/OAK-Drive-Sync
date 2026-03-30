# OAK-Drive-Sync — Claude Code Project Instructions

## Project Overview

Synchronized control system for **2× Luxonis OAK-D 4 Pro** cameras with **4× mechanical positioning drives** controlled via Raspberry Pi 5 GPIO, coordinated over MQTT.

### System Topology

```
┌─────────────────────┐       PoE++ Switch        ┌──────────────────┐
│  Windows 11 PC      │◄──────────────────────────►│ OAK-D 4 Pro #1  │
│  - Camera capture    │◄──────────────────────────►│ OAK-D 4 Pro #2  │
│  - Orchestration     │       Ethernet             │                  │
│  - Monitoring UI     │◄──────────────────────────►│ Raspberry Pi 5   │
│  - Email alerts      │       (MQTT over TCP)      │  - 4× GPIO drives│
└─────────────────────┘                             │  - MQTT client   │
                                                    │  - Health beacon │
                                                    └──────────────────┘
```

### Communication Stack

- **MQTT 5.0** via Mosquitto broker (runs on Pi or dedicated host)
- **DepthAI SDK** for OAK-D camera control (Windows side)
- **gpiozero / lgpio** for drive control (Pi side)
- **Email notifications** via SMTP (aiosmtplib)
- **Connectivity overlay** rendered into the camera streaming UI

## Tech Stack

| Component         | Technology                        | Platform   |
|-------------------|-----------------------------------|------------|
| MQTT Broker       | Mosquitto 2.x                     | RPi 5      |
| Pi Drive Control  | Python 3.11+, gpiozero, paho-mqtt | RPi 5      |
| Camera Control    | Python 3.11+, depthai, paho-mqtt  | Windows 11 |
| Monitoring UI     | NiceGUI or PyQt overlay           | Windows 11 |
| Email Alerts      | aiosmtplib, jinja2 templates      | Windows 11 |
| Package Manager   | uv                                | Both       |

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
- asyncio-first design (both Pi and Windows sides)
- Pydantic v2 models for all MQTT message payloads
- Structured logging via `structlog`
- All MQTT topics documented in `config/mqtt_topics.yaml`
- Tests with pytest + pytest-asyncio

## Key Design Decisions

1. **MQTT QoS 1** for drive commands (at-least-once delivery, idempotent moves)
2. **MQTT QoS 0** for health/heartbeat beacons (frequent, loss-tolerant)
3. **Retained messages** for last-known drive positions
4. **Last Will and Testament (LWT)** on both Pi and Windows clients for disconnect detection
5. **JSON payloads** with Pydantic validation on both ends
6. **Settling delay** (configurable, default 150ms) between drive-reached and capture-trigger
7. **Sequence IDs** on every command for correlation and out-of-order detection
