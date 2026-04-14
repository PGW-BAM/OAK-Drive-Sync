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
    from src.pi_controller.imu.drift_detector import DriftDetector
    from src.pi_controller.main import DriveManager

logger = structlog.get_logger()

POSITIONS_FILE = Path("config/positions.yaml")
CALIBRATION_FILE = Path("config/calibration.yaml")


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
# Calibration Persistence
# ──────────────────────────────────────────────

def load_calibration() -> dict[str, dict[str, float]]:
    """Load saved calibration: { "cam1:a": {"min": 0, "max": 5000}, ... }"""
    if CALIBRATION_FILE.exists():
        data = yaml.safe_load(CALIBRATION_FILE.read_text()) or {}
        return data.get("drives", {})
    return {}


def save_calibration(cal: dict[str, dict[str, float]]) -> None:
    data = {"drives": cal}
    CALIBRATION_FILE.write_text(yaml.dump(data, default_flow_style=False))


def apply_calibration(drive_mgr: DriveManager) -> None:
    """Load calibration from file and apply to drives."""
    cal = load_calibration()
    for key, limits in cal.items():
        drive = drive_mgr.drives.get(key)
        if drive:
            if "min" in limits:
                drive.set_min_position(limits["min"])
            if "max" in limits:
                drive.set_max_position(limits["max"])
            logger.info("calibration.applied", key=key, limits=limits)


# ──────────────────────────────────────────────
# GUI Application
# ──────────────────────────────────────────────

def setup_gui(
    drive_mgr: DriveManager,
    config: dict,
    gui_cfg: dict,
    drift_detector: DriftDetector | None = None,
) -> dict:
    """Register all NiceGUI pages. Call before ui.run().

    Returns gui_cfg for use by run_gui().
    """

    positions: list[PositionPreset] = load_positions()
    sequence_task: asyncio.Task | None = None

    # Session-only calibration — no auto-load from file.
    # All 4 drives must be calibrated manually via the Calibration tab each startup.

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
                ui.link("Calibration", "/calibration").classes("text-white")
                ui.link("Settings", "/settings").classes("text-white")

        # Calibration banner
        cal_banner = ui.card().classes("w-full bg-red-100 border-l-4 border-red-500")
        with cal_banner:
            with ui.row().classes("items-center gap-2"):
                ui.icon("warning").classes("text-red-600 text-2xl")
                cal_banner_text = ui.label(
                    "Drives not calibrated! Go to the Calibration tab before running sequences."
                ).classes("text-red-700 font-bold")
                ui.link("Go to Calibration", "/calibration").classes(
                    "ml-4 text-red-800 underline font-bold"
                )

        with ui.card().classes("w-full"):
            ui.label("Drive Status").classes("text-lg font-bold")

            drive_cards = {}
            with ui.row().classes("w-full flex-wrap gap-4"):
                for key, drive in drive_mgr.drives.items():
                    with ui.card().classes("w-64") as card:
                        drive_cards[key] = {}
                        ui.label(f"Drive {key}").classes("font-bold text-blue-700")
                        drive_cards[key]["cal"] = ui.label("NOT CALIBRATED").classes(
                            "text-red-600 font-bold text-sm"
                        )
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
            all_calibrated = True
            for key, drive in drive_mgr.drives.items():
                if key in drive_cards:
                    cards = drive_cards[key]
                    cards["state"].text = f"State: {drive.state.value}"
                    cards["position"].text = f"Position: {drive.current_position:.1f}"

                    if drive.calibrated:
                        cards["cal"].text = "CALIBRATED"
                        cards["cal"].classes(replace="text-green-600 font-bold text-sm")
                        min_p = drive.get_min_position()
                        max_p = drive.get_max_position()
                        cards["range"].text = f"Range: {min_p:.0f} – {max_p:.0f}"
                        range_size = max_p - min_p
                        if range_size > 0:
                            progress = (drive.current_position - min_p) / range_size
                            cards["progress"].value = max(0.0, min(1.0, progress))
                    else:
                        all_calibrated = False
                        cards["cal"].text = "NOT CALIBRATED"
                        cards["cal"].classes(replace="text-red-600 font-bold text-sm")
                        cards["range"].text = "Range: — (calibrate first)"
                        cards["progress"].value = 0

            # Show/hide calibration banner
            cal_banner.set_visibility(not all_calibrated)

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
                ui.link("Calibration", "/calibration").classes("text-white")
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
                ui.link("Calibration", "/calibration").classes("text-white")
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
            cam1b_input = ui.number(label="cam1:b", value=0, format="%.0f").classes("w-24")
            cam2a_input = ui.number(label="cam2:a", value=0, format="%.0f").classes("w-24")
            cam2b_input = ui.number(label="cam2:b", value=0, format="%.0f").classes("w-24")

            def add_position():
                if not name_input.value:
                    ui.notify("Please enter a name", type="warning")
                    return
                preset = PositionPreset(
                    name=name_input.value,
                    cam1_a=cam1a_input.value or 0,
                    cam1_b=cam1b_input.value or 0,
                    cam2_a=cam2a_input.value or 0,
                    cam2_b=cam2b_input.value or 0,
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
                ui.link("Calibration", "/calibration").classes("text-white")
                ui.link("Settings", "/settings").classes("text-white")

        all_calibrated = all(d.calibrated for d in drive_mgr.drives.values())
        if not all_calibrated:
            uncal = [k for k, d in drive_mgr.drives.items() if not d.calibrated]
            with ui.card().classes("w-full bg-red-100 border-l-4 border-red-500"):
                with ui.row().classes("items-center gap-2"):
                    ui.icon("warning").classes("text-red-600 text-2xl")
                    ui.label(
                        f"Cannot run sequences — drives not calibrated: {', '.join(uncal)}"
                    ).classes("text-red-700 font-bold")
                    ui.link("Go to Calibration", "/calibration").classes(
                        "ml-4 text-red-800 underline font-bold"
                    )
            return

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

    # ── Calibration Tab ────────────────────────

    @ui.page("/calibration")
    def calibration_page():
        ui.page_title("Drive Calibration")

        with ui.header().classes("bg-blue-900 text-white"):
            ui.label("OAK Drive Controller").classes("text-xl font-bold")
            ui.space()
            with ui.row().classes("gap-2"):
                ui.link("Dashboard", "/").classes("text-white")
                ui.link("Manual", "/manual").classes("text-white")
                ui.link("Positions", "/positions").classes("text-white")
                ui.link("Sequences", "/sequences").classes("text-white")
                ui.link("Calibration", "/calibration").classes("text-white")
                ui.link("Settings", "/settings").classes("text-white")

        # Enable calibration mode (bypasses position clamping).
        # Set here for the initial state and also inside each jog handler,
        # because app.on_disconnect fires on tab navigation and would reset it.
        for d in drive_mgr.drives.values():
            d.calibration_mode = True

        ui.label("Drive Calibration").classes("text-2xl font-bold")
        ui.markdown(
            "**Calibration is required every startup.** For each of the 4 drives:\n\n"
            "1. Jog to the **lower physical endpoint** → click **Set as Min**\n"
            "2. Jog to the **upper physical endpoint** → click **Set as Max**\n\n"
            "All 4 drives must be calibrated before running sequences."
        ).classes("text-gray-600 mb-4")

        # Overall status
        cal_status = ui.label("").classes("text-lg font-bold mb-2")

        def update_overall_status():
            done = sum(1 for d in drive_mgr.drives.values() if d.calibrated)
            total = len(drive_mgr.drives)
            if done == total:
                cal_status.text = f"All {total} drives calibrated — ready to go!"
                cal_status.classes(replace="text-lg font-bold mb-2 text-green-700")
            else:
                cal_status.text = f"Calibrated: {done}/{total} drives"
                cal_status.classes(replace="text-lg font-bold mb-2 text-orange-700")

        update_overall_status()

        for key, drive in drive_mgr.drives.items():
            with ui.card().classes("w-full mt-2"):
                with ui.row().classes("items-center gap-4 w-full"):
                    ui.label(f"Drive {key}").classes("font-bold text-blue-700 text-lg w-24")

                    cal_label = ui.label(
                        "CALIBRATED" if drive.calibrated else "NOT CALIBRATED"
                    ).classes(
                        "font-bold text-sm w-32 " +
                        ("text-green-600" if drive.calibrated else "text-red-600")
                    )

                    pos_label = ui.label(f"Position: {drive.current_position:.1f}").classes(
                        "w-40"
                    )
                    state_label = ui.label(f"({drive.state.value})").classes("w-20")

                    min_label = ui.label(
                        f"Min: {drive.get_min_position():.0f}" if drive.min_calibrated else "Min: —"
                    ).classes("text-orange-700 font-bold w-28")
                    max_label = ui.label(
                        f"Max: {drive.get_max_position():.0f}" if drive.max_calibrated else "Max: —"
                    ).classes("text-orange-700 font-bold w-28")
                    range_label = ui.label("").classes("text-purple-700 w-32")

                # Jog controls
                with ui.row().classes("items-center gap-2 mt-1"):
                    ui.label("Jog:").classes("text-sm w-10")

                    step_sizes = [1, 10, 50, 100, 500, 1000]
                    for step in step_sizes:
                        async def jog_down(d=drive, s=step):
                            d.calibration_mode = True  # re-assert: on_disconnect can clear it
                            target = d.current_position - s
                            await d.move_to(target, 0.3)

                        ui.button(f"-{step}", on_click=jog_down).props(
                            "color=grey-7 size=sm dense"
                        ).classes("px-2")

                    ui.label("|").classes("text-gray-400")

                    for step in step_sizes:
                        async def jog_up(d=drive, s=step):
                            d.calibration_mode = True  # re-assert: on_disconnect can clear it
                            target = d.current_position + s
                            await d.move_to(target, 0.3)

                        ui.button(f"+{step}", on_click=jog_up).props(
                            "color=grey-7 size=sm dense"
                        ).classes("px-2")

                # Calibration actions
                with ui.row().classes("items-center gap-2 mt-2"):
                    def set_min(
                        k=key, d=drive, ml=min_label, rl=range_label, cl=cal_label
                    ):
                        d.set_min_position(d.current_position)
                        d.min_calibrated = True
                        ml.text = f"Min: {d.get_min_position():.0f}"
                        if d.max_calibrated:
                            rl.text = f"Range: {d.get_max_position() - d.get_min_position():.0f}"
                        if d.calibrated:
                            cl.text = "CALIBRATED"
                            cl.classes(replace="font-bold text-sm w-32 text-green-600")
                        update_overall_status()
                        ui.notify(f"{k} MIN set to {d.current_position:.1f}", type="positive")

                    ui.button("Set as Min", on_click=set_min).props("color=deep-orange")

                    def set_max(
                        k=key, d=drive, xl=max_label, rl=range_label, cl=cal_label
                    ):
                        d.set_max_position(d.current_position)
                        d.max_calibrated = True
                        xl.text = f"Max: {d.get_max_position():.0f}"
                        if d.min_calibrated:
                            rl.text = f"Range: {d.get_max_position() - d.get_min_position():.0f}"
                        if d.calibrated:
                            cl.text = "CALIBRATED"
                            cl.classes(replace="font-bold text-sm w-32 text-green-600")
                        update_overall_status()
                        ui.notify(f"{k} MAX set to {d.current_position:.1f}", type="positive")

                    ui.button("Set as Max", on_click=set_max).props("color=deep-orange")

                    def set_zero(d=drive, k=key):
                        d.set_zero()
                        ui.notify(f"{k} position reset to 0", type="info")

                    ui.button("Set Zero Here", on_click=set_zero).props("color=blue-grey")

                    def stop_drive(d=drive):
                        d.emergency_stop()

                    ui.button("STOP", on_click=stop_drive).props("color=red")

                # Live position update
                def update_cal_labels(pl=pos_label, sl=state_label, d=drive):
                    pl.text = f"Position: {d.current_position:.1f}"
                    sl.text = f"({d.state.value})"

                ui.timer(0.2, update_cal_labels)

        # ── IMU Calibration Section ──────────────
        ui.separator().classes("mt-8")
        ui.label("IMU Drift Calibration").classes("text-2xl font-bold mt-4")
        ui.markdown(
            "Define up to 4 named checkpoints per radial drive (axis **b**). "
            "When a `MoveCommand` includes the checkpoint name, the Pi requests "
            "a fresh IMU angle from the Windows controller and auto-corrects any drift.\n\n"
            "**Requires the Windows controller to be running and connected.**"
        ).classes("text-gray-600 mb-2")

        if drift_detector is None:
            ui.label("DriftDetector not initialized — restart with GUI mode.").classes(
                "text-red-500 font-bold"
            )
        else:
            # Global settings card
            with ui.card().classes("w-full mt-2"):
                ui.label("Global Settings").classes("font-bold text-lg")
                with ui.row().classes("items-end gap-4 mt-1"):
                    threshold_input = ui.number(
                        label="Drift threshold (°)",
                        value=drift_detector.get_drift_threshold(),
                        min=0.1,
                        max=45.0,
                        step=0.1,
                        format="%.1f",
                    ).classes("w-44")

                    def save_threshold(inp=threshold_input):
                        drift_detector.set_drift_threshold(float(inp.value or 2.0))
                        ui.notify("Drift threshold saved", type="positive")

                    ui.button("Save", on_click=save_threshold).props("color=primary size=sm")

            # Per-camera IMU calibration (cam1:b and cam2:b only)
            for cam_id in ("cam1", "cam2"):
                drive_key = f"{cam_id}:b"
                drive = drive_mgr.drives.get(drive_key)
                if drive is None:
                    continue

                with ui.card().classes("w-full mt-4"):
                    ui.label(f"Checkpoints — {drive_key}").classes(
                        "font-bold text-blue-700 text-lg"
                    )

                    # Live IMU angle display
                    with ui.row().classes("gap-6 items-center mt-1"):
                        roll_label = ui.label("Roll: —").classes("font-mono text-sm")
                        pitch_label = ui.label("Pitch: —").classes("font-mono text-sm")
                        imu_status = ui.label("(no data)").classes(
                            "text-xs text-gray-400"
                        )

                    def update_imu_display(
                        rl=roll_label, pl=pitch_label, sl=imu_status, cid=cam_id
                    ):
                        angle = drift_detector.get_latest_imu(cid)
                        if angle:
                            rl.text = f"Roll: {angle.roll_deg:.2f}°"
                            pl.text = f"Pitch: {angle.pitch_deg:.2f}°"
                            sl.text = f"(updated {angle.timestamp.strftime('%H:%M:%S')})"
                        else:
                            rl.text = "Roll: —"
                            pl.text = "Pitch: —"
                            sl.text = "(no data — is Windows controller running?)"

                    ui.timer(0.5, update_imu_display)

                    # Steps-per-degree
                    with ui.row().classes("items-end gap-4 mt-3"):
                        spd_input = ui.number(
                            label="Steps per degree",
                            value=drift_detector.get_steps_per_degree(cam_id),
                            min=0.1,
                            max=10000.0,
                            step=1.0,
                            format="%.1f",
                        ).classes("w-48")
                        ui.label(
                            "Move between two positions, note the step delta and angle "
                            "delta, then enter: steps ÷ degrees."
                        ).classes("text-xs text-gray-500 max-w-xs")

                        def save_spd(cid=cam_id, inp=spd_input):
                            drift_detector.set_steps_per_degree(cid, float(inp.value or 120.0))
                            ui.notify(
                                f"Steps/degree saved for {cid}:b", type="positive"
                            )

                        ui.button("Save", on_click=save_spd).props("color=primary size=sm")

                    # Checkpoint table
                    ui.separator().classes("mt-3")
                    ui.label("Defined Checkpoints").classes("font-bold text-sm mt-1")
                    checkpoints_col = ui.column().classes("w-full")

                    def refresh_table(cid=cam_id, col=checkpoints_col):
                        col.clear()
                        cps = drift_detector.get_checkpoints(cid)
                        with col:
                            if not cps:
                                ui.label("No checkpoints yet.").classes(
                                    "text-gray-400 text-sm"
                                )
                                return
                            with ui.grid(columns=5).classes("w-full gap-1 text-sm"):
                                for header in (
                                    "Name", "Drive Position", "Expected Angle", "Active", ""
                                ):
                                    ui.label(header).classes("font-bold text-xs text-gray-600")
                                for cp in cps:
                                    active = cp["active_angle"]
                                    expected = cp.get(f"expected_{active}_deg", "—")
                                    ui.label(cp["name"]).classes("text-sm")
                                    ui.label(str(cp["drive_position"])).classes("text-sm font-mono")
                                    ui.label(f"{expected}°").classes("text-sm font-mono")
                                    ui.label(active).classes("text-sm")

                                    def delete_cp(n=cp["name"], cid2=cid, col2=col):
                                        drift_detector.delete_checkpoint(cid2, n)
                                        refresh_table(cid2, col2)
                                        ui.notify(f"Deleted '{n}'", type="warning")

                                    ui.button("Delete", on_click=delete_cp).props(
                                        "color=red size=xs flat dense"
                                    )

                    refresh_table()

                    # Add checkpoint form
                    ui.separator().classes("mt-3")
                    ui.label("Add / Update Checkpoint").classes("font-bold text-sm mt-1")
                    with ui.row().classes("items-end gap-2 mt-1 flex-wrap"):
                        cp_name = ui.input(
                            label="Name",
                            placeholder="e.g. top_view",
                        ).classes("w-36")
                        active_angle_sel = ui.select(
                            label="Active angle",
                            options=["roll", "pitch"],
                            value="roll",
                        ).classes("w-28")

                        async def capture_checkpoint(
                            cid=cam_id,
                            d=drive,
                            name_inp=cp_name,
                            ang_sel=active_angle_sel,
                            col=checkpoints_col,
                        ):
                            name = name_inp.value.strip()
                            if not name:
                                ui.notify(
                                    "Please enter a checkpoint name", type="warning"
                                )
                                return
                            angle = drift_detector.get_latest_imu(cid)
                            if angle is None:
                                ui.notify(
                                    "No IMU data available — is the Windows controller "
                                    "running and connected?",
                                    type="negative",
                                )
                                return
                            drift_detector.calibrate_checkpoint(
                                cid,
                                name,
                                d.current_position,
                                angle,
                                active_angle=ang_sel.value,
                            )
                            name_inp.value = ""
                            refresh_table(cid, col)
                            ui.notify(
                                f"Checkpoint '{name}' saved at position "
                                f"{d.current_position:.0f} "
                                f"({ang_sel.value}: "
                                f"{angle.roll_deg if ang_sel.value == 'roll' else angle.pitch_deg:.2f}°)",
                                type="positive",
                            )

                        ui.button(
                            "Capture Here", on_click=capture_checkpoint
                        ).props("color=teal")

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
                ui.link("Calibration", "/calibration").classes("text-white")
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

    logger.info("gui.setup_complete", host=host, port=port)
    return gui_cfg


def run_gui(gui_cfg: dict) -> None:
    """Start the NiceGUI server. This blocks and owns the event loop."""
    host = gui_cfg.get("host", "0.0.0.0")
    port = gui_cfg.get("port", 8080)
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
