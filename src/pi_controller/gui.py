"""
NiceGUI Web Interface for OAK-Drive-Sync Pi Controller.

Lightweight browser-based GUI for non-programmers to:
- View live drive status
- Manually jog drives
- Define and save position presets
- Run cyclic capture sequences
"""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import TYPE_CHECKING

import structlog
import yaml
from nicegui import ui, app

from src.shared.models import PositionPreset

if TYPE_CHECKING:
    from src.pi_controller.main import DriveManager

logger = structlog.get_logger()

POSITIONS_FILE = Path("config/positions.yaml")


# ──────────────────────────────────────────────
# Position Persistence
# ──────────────────────────────────────────────

def load_positions() -> list[PositionPreset]:
    if POSITIONS_FILE.exists():
        data = yaml.safe_load(POSITIONS_FILE.read_text()) or {}
        return [PositionPreset(**p) for p in data.get("positions", [])]
    return []


def save_positions(positions: list[PositionPreset]) -> None:
    data = {"positions": [p.model_dump() for p in positions]}
    POSITIONS_FILE.write_text(yaml.dump(data, default_flow_style=False))


# ──────────────────────────────────────────────
# GUI Application
# ──────────────────────────────────────────────

async def start_gui(drive_mgr: DriveManager, config: dict, gui_cfg: dict) -> None:
    """Start the NiceGUI web server."""

    positions: list[PositionPreset] = load_positions()
    sequence_task: asyncio.Task | None = None

    # ── Dashboard Tab ──────────────────────────

    @ui.page("/")
    def dashboard_page():
        ui.page_title("OAK Drive Controller")

        with ui.header().classes("bg-blue-900 text-white"):
            ui.label("OAK Drive Controller").classes("text-xl font-bold")
            ui.space()
            with ui.row().classes("gap-2"):
                ui.link("Dashboard", "/").classes("text-white")
                ui.link("Manual", "/manual").classes("text-white")
                ui.link("Positions", "/positions").classes("text-white")
                ui.link("Sequences", "/sequences").classes("text-white")
                ui.link("Settings", "/settings").classes("text-white")

        with ui.card().classes("w-full"):
            ui.label("Drive Status").classes("text-lg font-bold")

            drive_cards = {}
            with ui.row().classes("w-full flex-wrap gap-4"):
                for key, drive in drive_mgr.drives.items():
                    with ui.card().classes("w-64") as card:
                        drive_cards[key] = {}
                        ui.label(f"Drive {key}").classes("font-bold text-blue-700")
                        drive_cards[key]["state"] = ui.label(f"State: {drive.state.value}")
                        drive_cards[key]["position"] = ui.label(
                            f"Position: {drive.current_position:.1f}"
                        )
                        drive_cards[key]["range"] = ui.label(
                            f"Range: {drive.get_min_position():.0f} – {drive.get_max_position():.0f}"
                        )
                        drive_cards[key]["progress"] = ui.linear_progress(value=0).props(
                            "color=blue"
                        )

        with ui.card().classes("w-full mt-4"):
            ui.label("System").classes("text-lg font-bold")
            mqtt_label = ui.label("MQTT: connecting...")
            seq_label = ui.label("Sequence: idle")

        # Live update timer
        def update_dashboard():
            for key, drive in drive_mgr.drives.items():
                if key in drive_cards:
                    cards = drive_cards[key]
                    cards["state"].text = f"State: {drive.state.value}"
                    cards["position"].text = f"Position: {drive.current_position:.1f}"
                    min_p = drive.get_min_position()
                    max_p = drive.get_max_position()
                    range_size = max_p - min_p
                    if range_size > 0:
                        progress = (drive.current_position - min_p) / range_size
                        cards["progress"].value = max(0.0, min(1.0, progress))

            mqtt_label.text = f"MQTT: {'connected' if drive_mgr.mqtt.connected else 'disconnected'}"
            seq_label.text = f"Sequence: {'running' if drive_mgr.sequence_running else 'idle'}"

        ui.timer(0.5, update_dashboard)

    # ── Manual Control Tab ─────────────────────

    @ui.page("/manual")
    def manual_page():
        ui.page_title("Manual Control")

        with ui.header().classes("bg-blue-900 text-white"):
            ui.label("OAK Drive Controller").classes("text-xl font-bold")
            ui.space()
            with ui.row().classes("gap-2"):
                ui.link("Dashboard", "/").classes("text-white")
                ui.link("Manual", "/manual").classes("text-white")
                ui.link("Positions", "/positions").classes("text-white")
                ui.link("Sequences", "/sequences").classes("text-white")
                ui.link("Settings", "/settings").classes("text-white")

        speed_slider = ui.slider(min=0.1, max=1.0, value=0.5, step=0.1).props(
            "label-always"
        ).classes("w-64")
        ui.label("Speed").classes("text-sm")

        for key, drive in drive_mgr.drives.items():
            with ui.card().classes("w-full mt-2"):
                ui.label(f"Drive {key}").classes("font-bold text-blue-700")

                with ui.row().classes("items-center gap-2"):
                    pos_label = ui.label(f"Position: {drive.current_position:.1f}")
                    state_label = ui.label(f"({drive.state.value})")

                with ui.row().classes("gap-2"):
                    target_input = ui.number(
                        label="Target",
                        value=drive.current_position,
                        format="%.1f",
                    ).classes("w-32")

                    async def go_to(d=drive, inp=target_input):
                        await d.move_to(inp.value, speed_slider.value)

                    ui.button("Go", on_click=go_to).props("color=primary")

                    async def jog_up(d=drive, amount=100):
                        target = min(d.current_position + amount, d.get_max_position())
                        await d.move_to(target, speed_slider.value)

                    async def jog_down(d=drive, amount=100):
                        target = max(d.current_position - amount, d.get_min_position())
                        await d.move_to(target, speed_slider.value)

                    ui.button("+100", on_click=jog_up).props("color=teal")
                    ui.button("-100", on_click=jog_down).props("color=teal")

                    async def home_drive(d=drive):
                        await d.home()

                    ui.button("Home", on_click=home_drive).props("color=orange")

                    def stop_drive(d=drive):
                        d.emergency_stop()

                    ui.button("STOP", on_click=stop_drive).props("color=red")

                def update_labels(pl=pos_label, sl=state_label, d=drive):
                    pl.text = f"Position: {d.current_position:.1f}"
                    sl.text = f"({d.state.value})"

                ui.timer(0.3, update_labels)

        with ui.row().classes("mt-4 gap-2"):
            async def home_all():
                tasks = [d.home() for d in drive_mgr.drives.values()]
                await asyncio.gather(*tasks)

            ui.button("Home All", on_click=home_all).props("color=orange size=lg")

            def stop_all():
                for d in drive_mgr.drives.values():
                    d.emergency_stop()

            ui.button("STOP ALL", on_click=stop_all).props("color=red size=lg")

    # ── Positions Tab ──────────────────────────

    @ui.page("/positions")
    def positions_page():
        ui.page_title("Position Presets")

        with ui.header().classes("bg-blue-900 text-white"):
            ui.label("OAK Drive Controller").classes("text-xl font-bold")
            ui.space()
            with ui.row().classes("gap-2"):
                ui.link("Dashboard", "/").classes("text-white")
                ui.link("Manual", "/manual").classes("text-white")
                ui.link("Positions", "/positions").classes("text-white")
                ui.link("Sequences", "/sequences").classes("text-white")
                ui.link("Settings", "/settings").classes("text-white")

        positions_container = ui.column().classes("w-full")

        def refresh_positions_list():
            positions_container.clear()
            with positions_container:
                if not positions:
                    ui.label("No positions defined yet.").classes("text-gray-500")
                    return

                for i, preset in enumerate(positions):
                    with ui.card().classes("w-full"):
                        with ui.row().classes("items-center w-full"):
                            ui.label(preset.name).classes("font-bold text-lg flex-grow")

                            with ui.row().classes("gap-1 text-sm"):
                                ui.label(f"cam1:a={preset.cam1_a:.0f}")
                                ui.label(f"cam1:b={preset.cam1_b:.0f}")
                                ui.label(f"cam2:a={preset.cam2_a:.0f}")
                                ui.label(f"cam2:b={preset.cam2_b:.0f}")

                            async def goto_pos(p=preset):
                                await drive_mgr.move_all_to_position(p)

                            ui.button("Go To", on_click=goto_pos).props("color=primary size=sm")

                            def capture_pos(idx=i):
                                positions[idx] = PositionPreset(
                                    name=positions[idx].name,
                                    cam1_a=drive_mgr.drives.get("cam1:a", _DummyDrive()).current_position,
                                    cam1_b=drive_mgr.drives.get("cam1:b", _DummyDrive()).current_position,
                                    cam2_a=drive_mgr.drives.get("cam2:a", _DummyDrive()).current_position,
                                    cam2_b=drive_mgr.drives.get("cam2:b", _DummyDrive()).current_position,
                                )
                                save_positions(positions)
                                refresh_positions_list()

                            ui.button("Capture Current", on_click=capture_pos).props(
                                "color=teal size=sm"
                            )

                            def delete_pos(idx=i):
                                positions.pop(idx)
                                save_positions(positions)
                                refresh_positions_list()

                            ui.button("Delete", on_click=delete_pos).props(
                                "color=red size=sm flat"
                            )

        refresh_positions_list()

        # Add new position form
        ui.separator().classes("mt-4")
        ui.label("Add New Position").classes("text-lg font-bold")

        with ui.row().classes("items-end gap-2"):
            name_input = ui.input(label="Name", placeholder="e.g. Top Left").classes("w-48")

            cam1a_input = ui.number(label="cam1:a", value=0, format="%.0f").classes("w-24")
            cam1b_input = ui.number(label="cam1:b", value=1500, format="%.0f").classes("w-24")
            cam2a_input = ui.number(label="cam2:a", value=0, format="%.0f").classes("w-24")
            cam2b_input = ui.number(label="cam2:b", value=1500, format="%.0f").classes("w-24")

            def add_position():
                if not name_input.value:
                    ui.notify("Please enter a name", type="warning")
                    return
                preset = PositionPreset(
                    name=name_input.value,
                    cam1_a=cam1a_input.value or 0,
                    cam1_b=cam1b_input.value or 1500,
                    cam2_a=cam2a_input.value or 0,
                    cam2_b=cam2b_input.value or 1500,
                )
                positions.append(preset)
                save_positions(positions)
                name_input.value = ""
                refresh_positions_list()
                ui.notify(f"Position '{preset.name}' added", type="positive")

            ui.button("Add", on_click=add_position).props("color=primary")

            def add_from_current():
                if not name_input.value:
                    ui.notify("Please enter a name", type="warning")
                    return
                preset = PositionPreset(
                    name=name_input.value,
                    cam1_a=drive_mgr.drives.get("cam1:a", _DummyDrive()).current_position,
                    cam1_b=drive_mgr.drives.get("cam1:b", _DummyDrive()).current_position,
                    cam2_a=drive_mgr.drives.get("cam2:a", _DummyDrive()).current_position,
                    cam2_b=drive_mgr.drives.get("cam2:b", _DummyDrive()).current_position,
                )
                positions.append(preset)
                save_positions(positions)
                name_input.value = ""
                refresh_positions_list()
                ui.notify(f"Position '{preset.name}' captured from current", type="positive")

            ui.button("Add from Current Position", on_click=add_from_current).props(
                "color=teal"
            )

    # ── Sequences Tab ──────────────────────────

    @ui.page("/sequences")
    def sequences_page():
        nonlocal sequence_task
        ui.page_title("Capture Sequences")

        with ui.header().classes("bg-blue-900 text-white"):
            ui.label("OAK Drive Controller").classes("text-xl font-bold")
            ui.space()
            with ui.row().classes("gap-2"):
                ui.link("Dashboard", "/").classes("text-white")
                ui.link("Manual", "/manual").classes("text-white")
                ui.link("Positions", "/positions").classes("text-white")
                ui.link("Sequences", "/sequences").classes("text-white")
                ui.link("Settings", "/settings").classes("text-white")

        if not positions:
            ui.label(
                "No positions defined. Go to the Positions tab first."
            ).classes("text-orange-600 text-lg")
            return

        ui.label("Select Positions for Sequence").classes("text-lg font-bold")

        # Checkboxes for each position
        selected: dict[int, ui.checkbox] = {}
        for i, preset in enumerate(positions):
            selected[i] = ui.checkbox(
                f"{preset.name} (cam1:a={preset.cam1_a:.0f}, cam1:b={preset.cam1_b:.0f}, "
                f"cam2:a={preset.cam2_a:.0f}, cam2:b={preset.cam2_b:.0f})",
                value=True,
            )

        ui.separator()

        with ui.row().classes("items-end gap-4"):
            dwell_input = ui.number(
                label="Dwell time (ms)", value=500, min=0, max=60000, step=100
            ).classes("w-40")

            repeat_input = ui.number(
                label="Repeat count (0=infinite)", value=1, min=0, max=10000, step=1
            ).classes("w-48")

        ui.separator()
        status_label = ui.label("Status: idle").classes("text-lg")
        cycle_label = ui.label("")

        async def start_sequence():
            nonlocal sequence_task
            if drive_mgr.sequence_running:
                ui.notify("Sequence already running", type="warning")
                return

            selected_positions = [
                positions[i] for i, cb in selected.items() if cb.value
            ]
            if not selected_positions:
                ui.notify("Select at least one position", type="warning")
                return

            status_label.text = "Status: RUNNING"
            status_label.classes(replace="text-lg text-green-700")
            ui.notify("Sequence started", type="positive")

            sequence_task = asyncio.create_task(
                drive_mgr.run_sequence(
                    selected_positions,
                    dwell_time_ms=int(dwell_input.value or 500),
                    repeat_count=int(repeat_input.value or 1),
                )
            )

            try:
                await sequence_task
                status_label.text = "Status: completed"
                status_label.classes(replace="text-lg text-blue-700")
                ui.notify("Sequence completed", type="positive")
            except asyncio.CancelledError:
                status_label.text = "Status: stopped"
                status_label.classes(replace="text-lg text-orange-700")
            except Exception as exc:
                status_label.text = f"Status: error — {exc}"
                status_label.classes(replace="text-lg text-red-700")

        def stop_sequence():
            nonlocal sequence_task
            drive_mgr.stop_sequence()
            if sequence_task and not sequence_task.done():
                sequence_task.cancel()
            status_label.text = "Status: stopped"
            status_label.classes(replace="text-lg text-orange-700")
            ui.notify("Sequence stopped", type="warning")

        with ui.row().classes("gap-2"):
            ui.button("Start Sequence", on_click=start_sequence).props(
                "color=green size=lg"
            )
            ui.button("Stop Sequence", on_click=stop_sequence).props(
                "color=red size=lg"
            )

    # ── Settings Tab ───────────────────────────

    @ui.page("/settings")
    def settings_page():
        ui.page_title("Settings")

        with ui.header().classes("bg-blue-900 text-white"):
            ui.label("OAK Drive Controller").classes("text-xl font-bold")
            ui.space()
            with ui.row().classes("gap-2"):
                ui.link("Dashboard", "/").classes("text-white")
                ui.link("Manual", "/manual").classes("text-white")
                ui.link("Positions", "/positions").classes("text-white")
                ui.link("Sequences", "/sequences").classes("text-white")
                ui.link("Settings", "/settings").classes("text-white")

        with ui.card().classes("w-full"):
            ui.label("Drive Limits").classes("text-lg font-bold")

            for key, drive in drive_mgr.drives.items():
                with ui.row().classes("items-center gap-2"):
                    ui.label(f"{key}:").classes("w-20 font-bold")
                    ui.label(
                        f"Min: {drive.get_min_position():.0f}  "
                        f"Max: {drive.get_max_position():.0f}"
                    )
                    ui.label(f"Current: {drive.current_position:.1f}")

        with ui.card().classes("w-full mt-4"):
            ui.label("MQTT").classes("text-lg font-bold")
            broker = config.get("broker", {})
            ui.label(f"Broker: {broker.get('host', 'localhost')}:{broker.get('port', 1883)}")
            mqtt_status = ui.label("Status: checking...")

            def update_mqtt():
                mqtt_status.text = (
                    f"Status: {'connected' if drive_mgr.mqtt.connected else 'disconnected'}"
                )

            ui.timer(1.0, update_mqtt)

        with ui.card().classes("w-full mt-4"):
            ui.label("Settling Delay").classes("text-lg font-bold")
            settling = ui.number(
                label="Settling delay (ms)",
                value=drive_mgr.settling_delay_ms,
                min=0,
                max=5000,
                step=10,
            )

            def update_settling():
                drive_mgr.settling_delay_ms = int(settling.value or 150)
                ui.notify("Settling delay updated", type="positive")

            ui.button("Apply", on_click=update_settling).props("color=primary")

    # ── Start server ───────────────────────────

    host = gui_cfg.get("host", "0.0.0.0")
    port = gui_cfg.get("port", 8080)

    logger.info("gui.starting", host=host, port=port)
    ui.run(
        host=host,
        port=port,
        title=gui_cfg.get("title", "OAK Drive Controller"),
        reload=False,
        show=False,  # don't open browser on headless Pi
    )


class _DummyDrive:
    """Fallback for missing drives when reading positions."""

    current_position: float = 0.0
