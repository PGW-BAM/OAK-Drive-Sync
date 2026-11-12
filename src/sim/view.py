"""Interactive 3D view + control panel of the digital twin (NiceGUI ``ui.scene``).

Two modes (see docs/SIMULATION.md):

* standalone / offline: ``python -m src.sim.view`` or ``oak-twin`` -- sliders set the
  ``RigModel`` directly.
* live: ``oak-pi run --sim`` registers the same page as ``/twin`` on the Pi GUI app
  (``register_twin_page(rig, drive_mgr=...)``); moves are sent through the ``DriveManager``'s
  sim drives, the view only reads the shared ``RigModel`` (10 Hz timer).

Scene = specimen frame of ``src/sim`` (mm, z up, x toward cam1, y across the width).
"""

from __future__ import annotations

import asyncio
import math
import struct
from typing import TYPE_CHECKING, Any

import click
from fastapi import Response
from nicegui import app, ui

from src.sim import kinematics as kin
from src.sim.rig_model import RigModel
from src.sim.targets import Target, TargetResult, parse_targets, solve_targets

if TYPE_CHECKING:
    from src.pi_controller.main import DriveManager
    from src.sim.specimen import Mesh

STL_URL = "/twin-specimen.stl"
CAM_COLORS = {"cam1": "#2563eb", "cam2": "#f97316"}
_FALLBACK_COLORS = ["#7c3aed", "#0d9488", "#db2777"]
WELD_COLOR = "#eab308"
GOOD, BAD, WARN = "#16a34a", "#dc2626", "#d97706"

BODY = (100.0, 50.0, 60.0)  # camera housing: width (image-right), height (image-up), depth (mm)
RAIL_OFFSET = 140.0  # rail sits this far behind the pivot (away from the specimen)
ARC_RADIUS = 100.0  # drawn radius of the tilt-range arc around the pivot
FRUSTUM_FALLBACK_LEN = 400.0

DEFAULT_TARGETS = """[
  {"cam_id": "cam1", "z": 0.0},
  {"cam_id": "cam1", "z": 12.0, "theta_deg": 10.0},
  {"cam_id": "cam2", "z": -8.0},
  {"cam_id": "cam2", "z": 0.0, "normal": [-0.97, 0.0, 0.24]},
  {"cam_id": "cam1", "z": 200.0}
]"""

VIOLATION_DE = {
    "a_min": "Höhe unter Soft-Min",
    "a_max": "Höhe über Soft-Max",
    "b_min": "Tilt unter Soft-Min",
    "b_max": "Tilt über Soft-Max",
    "normal_y": "Normale hat y-Anteil (keine Gier-Achse)",
    "normal_direction": "Normale zeigt von der Kamera weg",
    "unknown_cam": "unbekannte Kamera",
}
MODE_DE = {"perpendicular": "senkrecht", "normal": "Normale", "fixed": "fester Winkel"}

# --- helpers --------------------------------------------------------------------------------


def _cam_color(rig: RigModel, cam_id: str) -> str:
    if cam_id in CAM_COLORS:
        return CAM_COLORS[cam_id]
    return _FALLBACK_COLORS[sorted(rig.config.cameras).index(cam_id) % len(_FALLBACK_COLORS)]


def _stl_bytes(mesh: Mesh) -> bytes:
    """Binary STL with facet normals (three.js needs them for shading)."""
    out = bytearray(80) + struct.pack("<I", len(mesh.faces))
    for i, j, k in mesh.faces:
        a, b, c = mesh.vertices[i], mesh.vertices[j], mesh.vertices[k]
        n = kin.vcross(
            (b[0] - a[0], b[1] - a[1], b[2] - a[2]), (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        )
        ln = math.sqrt(kin.vdot(n, n)) or 1.0
        out += struct.pack("<12fH", n[0] / ln, n[1] / ln, n[2] / ln, *a, *b, *c, 0)
    return bytes(out)


def _explain(rig: RigModel, cam_id: str, res: kin.IKResult) -> str:
    """German reason why an IK result is not reachable (which limit, which value)."""
    cam = rig.cam(cam_id)
    parts = []
    for v in res.violations:
        axis = cam.axis_a if v[0] == "a" else cam.axis_b
        val, unit = (res.height_mm, "mm") if v[0] == "a" else (res.tilt_deg, "°")
        lim = axis.soft_min if v.endswith("min") else axis.soft_max
        parts.append(f"{VIOLATION_DE[v]}: {val:.1f} {unit} (Grenze {lim:.1f} {unit})")
    return "; ".join(parts)


def _violations_de(res: TargetResult) -> str:
    return ", ".join(VIOLATION_DE.get(v, v) for v in res.violations)


_stl_route_ready = False


def _ensure_stl_route(rig: RigModel) -> None:
    """Serve the specimen mesh (``Specimen.mesh()``) as STL for ``ui.scene.stl``."""
    global _stl_route_ready
    if _stl_route_ready:
        return
    data = _stl_bytes(rig.specimen.mesh())

    @app.get(STL_URL)
    def _specimen_stl() -> Response:
        return Response(content=data, media_type="model/stl")

    _stl_route_ready = True


def _rot_cols(x: kin.Vec3, y: kin.Vec3, z: kin.Vec3) -> list[list[float]]:
    """Rotation matrix whose columns are the local axes x, y, z expressed in the world frame."""
    return [[x[i], y[i], z[i]] for i in range(3)]


# --- the page ---------------------------------------------------------------------------------


class TwinPage:
    """One browser client: 3D scene, per-camera panels, sweep and Soll/Ist tables."""

    def __init__(self, rig: RigModel, drive_mgr: DriveManager | None) -> None:
        self.rig = rig
        self.drive_mgr = drive_mgr
        self.cams = sorted(rig.config.cameras)
        self.scene: ui.scene
        self.w: dict[str, dict[str, Any]] = {c: {} for c in self.cams}
        self.groups: dict[str, Any] = {}
        self.dyn: dict[str, list[Any]] = {c: [] for c in self.cams}
        self.last: dict[str, tuple[int, int]] = {}

    @property
    def live(self) -> bool:
        return self.drive_mgr is not None

    # --- layout ---------------------------------------------------------------------------

    def build(self) -> None:
        ui.page_title("OAK Digital Twin")
        with ui.header().classes("bg-blue-900 text-white items-center"):
            ui.label("OAK Digital Twin").classes("text-xl font-bold")
            ui.label("Live (Sim-Antriebe)" if self.live else "Offline-Modell").classes(
                "text-sm bg-blue-700 rounded px-2"
            )
            ui.space()
            if self.live:
                ui.link("Dashboard", "/").classes("text-white")
        with ui.row().classes("w-full no-wrap items-start gap-4 p-2"):
            with ui.column().classes("flex-grow gap-2"):
                self._build_scene_card()
                self._build_sweep_card()
            with ui.column().classes("gap-2").style("width: 560px; flex-shrink: 0"):
                for cam in self.cams:
                    self._build_cam_card(cam)
        with ui.row().classes("w-full p-2"):
            self._build_targets_card()
        self.refresh(force=True)
        ui.timer(0.1, self.refresh)

    def _build_scene_card(self) -> None:
        with ui.card().classes("w-full p-2"):
            with ui.row().classes("items-center gap-2"):
                ui.label("3D-Ansicht").classes("font-bold")
                ui.button("Seite", on_click=lambda: self._view(0, -1000, 0)).props("dense outline")
                ui.button("Vorne", on_click=lambda: self._view(1000, 0, 100)).props("dense outline")
                ui.button("Iso", on_click=lambda: self._view(750, -800, 420)).props("dense outline")
            camera = ui.scene.perspective_camera(fov=40, near=10, far=10000)
            self.scene = ui.scene(
                width=760, height=560, grid=False, camera=camera, background_color="#f1f5f9"
            ).classes("w-full")
            self._view(750, -800, 420, duration=0)
            self._build_static()
            with ui.row().classes("gap-3 text-xs items-center"):
                for color, text in (
                    ("#9ca3af", "Probe"),
                    (WELD_COLOR, "Schweißnaht"),
                    (CAM_COLORS["cam1"], "cam1 (vorne)"),
                    (CAM_COLORS["cam2"], "cam2 (hinten)"),
                    (GOOD, "Naht im Bild"),
                    (BAD, "Naht nicht im Bild"),
                    (WARN, "Soft-Limit"),
                    ("#991b1b", "Endschalter"),
                ):
                    with ui.row().classes("items-center gap-1"):
                        ui.element("div").style(f"width:12px;height:12px;background:{color}")
                        ui.label(text)

    def _view(self, x: float, y: float, z: float, duration: float = 0.6) -> None:
        self.scene.move_camera(
            x,
            y,
            z,
            look_at_x=0,
            look_at_y=0,
            look_at_z=0,
            up_x=0,
            up_y=0,
            up_z=1,
            duration=duration,
        )

    def _build_cam_card(self, cam: str) -> None:
        c = self.rig.cam(cam)
        w = self.w[cam]
        color = _cam_color(self.rig, cam)
        side = "Vorderseite" if c.side > 0 else "Rückseite"
        with ui.card().classes("w-full p-3 gap-1").style(f"border-left: 6px solid {color}"):
            ui.label(f"{cam} – {side}").classes("font-bold text-lg").style(f"color:{color}")
            for key, name, ax, step in (
                ("h", "Höhe h [mm]", c.axis_a, 0.5),
                ("t", "Tilt θ [°]", c.axis_b, 0.1),
            ):
                with ui.row().classes("w-full items-center no-wrap gap-2"):
                    ui.label(name).classes("w-24 text-sm whitespace-nowrap")
                    w[key + "_slider"] = (
                        ui.slider(
                            min=ax.travel_min,
                            max=ax.travel_max,
                            step=step,
                            value=0.0,
                            on_change=lambda e, k=key, cm=cam: self._on_demand(cm, k, e.value),
                        )
                        .props("label")
                        .classes("flex-grow")
                    )
                    w[key + "_num"] = (
                        ui.number(
                            value=0.0,
                            step=step,
                            format="%.2f",
                            on_change=lambda e, k=key, cm=cam: self._on_demand(cm, k, e.value),
                        )
                        .props("dense outlined")
                        .classes("w-28")
                    )
                w[key + "_steps"] = ui.label("").classes("text-xs text-gray-500 -mt-2 ml-24")
            if self.live:
                ui.button(
                    "Fahren (über Antriebe)",
                    on_click=lambda cm=cam: self._go_demand(cm),
                ).props("dense color=primary")
            with ui.grid(columns="auto 1fr").classes("gap-x-4 gap-y-0 w-full mt-1"):
                for key, name in (
                    ("hit", "Treffer z"),
                    ("dist", "Abstand Linse→Treffer"),
                    ("inc", "Einfallswinkel"),
                    ("fp", "Footprint B × H"),
                    ("gsd", "GSD (h × v)"),
                    ("weld", "Naht im Bild"),
                    ("imu", "IMU Roll / Pitch"),
                    ("lim_a", "Limit Höhe (a)"),
                    ("lim_b", "Limit Tilt (b)"),
                ) + ((("drv", "Antriebe"),) if self.live else ()):
                    ui.label(name).classes("text-gray-500 text-sm")
                    w[key] = ui.label("–").classes("text-sm font-mono")
            ui.separator()
            with ui.row().classes("w-full items-center no-wrap gap-2"):
                w["ik_z"] = (
                    ui.number("Ziel z (mm)", value=0.0, step=1.0, format="%.1f")
                    .props("dense outlined")
                    .classes("w-24")
                )
                w["ik_mode"] = (
                    ui.select(
                        {"normal": "senkrecht zur Oberfläche", "fixed": "fester Winkel"},
                        value="normal",
                    )
                    .props("dense outlined")
                    .classes("flex-grow")
                )
                w["ik_angle"] = (
                    ui.number("θ (°)", value=0.0, step=1.0, format="%.1f")
                    .props("dense outlined")
                    .classes("w-20")
                )
                w["ik_angle"].bind_visibility_from(w["ik_mode"], "value", lambda v: v == "fixed")
                ui.button("Anfahren", on_click=lambda cm=cam: self._ik_go(cm)).props("dense")
            w["ik_msg"] = ui.label("").classes("text-sm")

    def _build_sweep_card(self) -> None:
        with ui.card().classes("w-full p-3"):
            ui.label("Sweep (beide Kameras, senkrechte Sicht)").classes("font-bold")
            with ui.row().classes("items-center gap-2"):
                self.sweep_in = ui.input("Ziel-z-Werte (mm)", value="-20, -10, 0, 10, 20").classes(
                    "w-64"
                )
                self.sweep_btn = ui.button("Sweep abfahren", on_click=self._sweep)
            cols = [
                {"name": n, "label": lab, "field": n, "align": "left"}
                for n, lab in (
                    ("z", "Ziel z"),
                    ("cam", "Kamera"),
                    ("h", "h (mm)"),
                    ("t", "θ (°)"),
                    ("ok", "erreichbar"),
                )
            ]
            self.sweep_table = (
                ui.table(columns=cols, rows=[], row_key="id").props("dense flat").classes("w-full")
            )

    def _build_targets_card(self) -> None:
        with ui.card().classes("w-full p-3"):
            ui.label("Soll/Ist – Ziele (JSON, provisorisch)").classes("font-bold")
            self.tg_text = (
                ui.textarea(value=DEFAULT_TARGETS)
                .props("outlined dense rows=7")
                .classes("w-full font-mono")
            )
            with ui.row().classes("items-center gap-2"):
                ui.button("Berechnen", on_click=self._targets_compute)
                ui.upload(
                    on_upload=self._targets_upload, auto_upload=True, label="JSON laden"
                ).props("accept=.json flat dense").classes("w-48")
            self.tg_msg = ui.label("").classes("text-sm text-red-600")
            cols = [
                {"name": n, "label": lab, "field": n, "align": "left"}
                for n, lab in (
                    ("cam", "Kamera"),
                    ("z", "z"),
                    ("mode", "Modus"),
                    ("h", "Soll h"),
                    ("t", "Soll θ"),
                    ("ih", "Ist h"),
                    ("it", "Ist θ"),
                    ("dh", "Δh"),
                    ("dt", "Δθ"),
                    ("ok", "erreichbar"),
                )
            ]
            self.tg_table = (
                ui.table(columns=cols, rows=[], row_key="id").props("dense flat").classes("w-full")
            )

    # --- static scene -----------------------------------------------------------------------

    def _build_static(self) -> None:
        spec = self.rig.specimen
        cfg = spec.cfg
        t, bh = cfg.thickness, cfg.bead.height
        with self.scene:
            self.scene.stl(STL_URL).material("#9ca3af", side="both")
            # weld bead highlighted as a band across both faces
            self.scene.box(t + 2 * bh + 0.4, cfg.gauge_width, cfg.bead.width).material(
                WELD_COLOR, 0.85
            )
            if cfg.clamp_length > 0:
                zc = cfg.length / 2 - cfg.clamp_length / 2
                for sz in (-1, 1):
                    self.scene.box(t + 40, cfg.grip_width + 30, cfg.clamp_length).move(
                        0, 0, sz * zc
                    ).material("#475569", 0.35)
            for cam in self.cams:
                self._build_cam_static(cam)

    def _build_cam_static(self, cam: str) -> None:
        """Parts that only move with the height h: pivot, tilt arc, arm, carriage, rail."""
        c = self.rig.cam(cam)
        s = c.side
        color = _cam_color(self.rig, cam)
        spec = self.rig.specimen
        px = s * (spec.half_thickness + c.pivot_distance)
        rail_x = px + s * RAIL_OFFSET
        ab = c.axis_b

        def ray(theta: float, r0: float, r1: float) -> list[list[float]]:
            d = (-s * math.cos(math.radians(theta)), 0.0, math.sin(math.radians(theta)))
            return [[d[0] * r0, 0, d[2] * r0], [d[0] * r1, 0, d[2] * r1]]

        with self.scene.group() as g:
            self.groups[cam] = g
            # pivot axis (horizontal, parallel to y): three.js cylinders point along scene y
            self.scene.cylinder(7, 7, 80, 20).material("#334155")
            # arm to the rail + carriage
            self.scene.box(RAIL_OFFSET, 12, 12).move(s * RAIL_OFFSET / 2, 0, 0).material("#64748b")
            self.scene.box(30, 60, 40).move(s * RAIL_OFFSET, 0, 0).material(color)
            # tilt range: arc of the optical axis + limit rays (soft = amber, switch = red)
            n = 40
            arc = [
                ray(ab.travel_min + (ab.travel_max - ab.travel_min) * i / n, ARC_RADIUS, 0)[0]
                for i in range(n + 1)
            ]
            self.scene.polyline(arc).material(color)
            self.scene.polyline(ray(0.0, 0, ARC_RADIUS)).material("#94a3b8")
            for lim in (ab.soft_min, ab.soft_max):
                self.scene.polyline(ray(lim, 0, ARC_RADIUS * 1.12)).material(WARN)
            for lim in (ab.switch_min_pos, ab.switch_max_pos):
                self.scene.polyline(ray(lim, ARC_RADIUS * 0.9, ARC_RADIUS * 1.2)).material(
                    "#991b1b"
                )
            self.scene.text(cam, f"color:{color};font-weight:bold;font-size:14px").move(
                s * 40, 0, 80
            )
        # vertical linear axis (fixed): rail with soft-limit and switch markers
        aa = c.axis_a
        lo, hi = aa.switch_min_pos, aa.switch_max_pos
        self.scene.box(10, 20, hi - lo).move(rail_x, 0, (lo + hi) / 2).material("#94a3b8")
        for z, col in ((aa.soft_min, WARN), (aa.soft_max, WARN)):
            self.scene.box(16, 60, 2).move(rail_x, 0, z).material(col)
        for z in (lo, hi):
            self.scene.box(16, 60, 3).move(rail_x, 0, z).material("#991b1b")

    # --- dynamic scene ----------------------------------------------------------------------

    def _redraw(self, cam: str) -> None:
        """Rebuild what depends on (h, theta): body, axis ray, hit point, frustum, footprint."""
        rig = self.rig
        c = rig.cam(cam)
        s = c.side
        color = _cam_color(rig, cam)
        pose = rig.pose(cam)
        hit = rig.hit(cam)
        fp = rig.footprint(cam)
        self.groups[cam].move(pose.pivot[0], 0, pose.pivot[2])
        for obj in self.dyn[cam]:
            obj.delete()
        objs: list[Any] = []
        d = pose.direction
        center = kin.vadd(pose.lens, kin.vscale(d, -BODY[2] / 2))
        with self.scene:
            objs.append(
                self.scene.box(*BODY)
                .move(*center)
                .rotate_R(_rot_cols(pose.right, pose.up, kin.vscale(d, -1)))
                .material(color, 0.9)
            )
            objs.append(self.scene.sphere(9).move(*pose.lens).material("#0f172a"))
            end = hit.point if hit.hit else kin.vadd(pose.lens, kin.vscale(d, FRUSTUM_FALLBACK_LEN))
            objs.append(self.scene.polyline([list(pose.lens), list(end)]).material("#111827"))
            if hit.hit:
                objs.append(self.scene.sphere(3.5).move(*hit.point).material(BAD))
            if fp.valid:
                ok = fp.weld_in_view
                col = GOOD if ok else BAD
                px = s * (rig.specimen.half_thickness + rig.specimen.cfg.bead.height + 0.3)
                corners = [[px, y, z] for y, z in fp.polygon]
                star = [corners[0], list(pose.lens), corners[1], list(pose.lens), corners[2]]
                star += [list(pose.lens), corners[3]]
                objs.append(self.scene.polyline(star).material(color))
                objs.append(self.scene.polyline([*corners, corners[0]]).material(col))
                fill = (
                    self.scene.extrusion([[y, z] for y, z in fp.polygon], 0.2)
                    .move(px, 0, 0)
                    .rotate_R(_rot_cols((0, 1, 0), (0, 0, 1), (1, 0, 0)))
                    .material(col, 0.18, side="both")
                )
                objs.append(fill)
        self.dyn[cam] = objs

    # --- updating ---------------------------------------------------------------------------

    def refresh(self, force: bool = False) -> None:
        for cam in self.cams:
            steps = (self.rig.get_steps(cam, "a"), self.rig.get_steps(cam, "b"))
            if force or steps != self.last.get(cam):
                self.last[cam] = steps
                self._redraw(cam)
                self._update_readouts(cam)
                if self.live and force:
                    self._push_demand(cam, self.rig.height(cam), self.rig.tilt(cam))
            if self.live:
                self._update_drive_status(cam)

    @staticmethod
    def _style(label: ui.label, text: str, level: str = "ok") -> None:
        color = {"ok": "text-green-700", "warn": "text-amber-600", "bad": "text-red-600"}[level]
        label.set_text(text)
        label.classes(replace=f"text-sm font-mono font-bold {color}")

    def _limit_text(self, st: kin.LimitStatus, label: ui.label) -> None:
        if st.switch_triggered:
            self._style(label, "ENDSCHALTER " + ("min" if st.switch_min else "max"), "bad")
        elif not st.within_soft:
            self._style(label, "Soft-Limit " + ("min" if st.soft_min else "max"), "warn")
        else:
            self._style(label, "OK", "ok")

    def _update_readouts(self, cam: str) -> None:
        rig, w = self.rig, self.w[cam]
        h, th = rig.height(cam), rig.tilt(cam)
        hit, fp = rig.hit(cam), rig.footprint(cam)
        lim = rig.limits(cam)
        w["h_steps"].set_text(f"Ist {h:.2f} mm = {rig.get_steps(cam, 'a')} Schritte (Achse a)")
        w["t_steps"].set_text(f"Ist {th:.2f}° = {rig.get_steps(cam, 'b')} Schritte (Achse b)")
        if hit.hit:
            w["hit"].set_text(f"{hit.z:+.2f} mm" + ("" if hit.on_specimen else " (neben Probe)"))
            w["dist"].set_text(f"{hit.distance:.1f} mm")
            w["inc"].set_text(f"{hit.incidence_deg:.2f}°")
        else:
            for k in ("hit", "dist", "inc"):
                w[k].set_text("kein Treffer")
        if fp.valid:
            w["fp"].set_text(f"{fp.width:.0f} × {fp.height:.0f} mm")
            w["gsd"].set_text(f"{fp.gsd_h_mm_px:.3f} × {fp.gsd_v_mm_px:.3f} mm/px")
            ok = fp.weld_in_view
            self._style(
                w["weld"],
                ("ja" if ok else "nein") + f" ({fp.weld_coverage:.0%})",
                "ok" if ok else "bad",
            )
        else:
            w["fp"].set_text("–")
            w["gsd"].set_text("–")
            self._style(w["weld"], "nein (FOV trifft Fläche nicht)", "bad")
        roll, pitch = rig.imu_angles(cam)
        w["imu"].set_text(f"{roll:+.2f}° / {pitch:+.2f}°")
        self._limit_text(lim["a"], w["lim_a"])
        self._limit_text(lim["b"], w["lim_b"])

    def _update_drive_status(self, cam: str) -> None:
        parts, level = [], "ok"
        for axis in ("a", "b"):
            drv = self.drive_mgr.drives.get(f"{cam}:{axis}") if self.drive_mgr else None
            if drv is None:
                continue
            txt = f"{axis}: {drv.state.value}"
            sw = getattr(drv, "triggered_switch", None)
            if sw:
                txt += f" (Endschalter {sw})"
            if drv.state.value == "fault":
                level = "bad"
            elif drv.state.value == "moving" and level == "ok":
                level = "warn"
            parts.append(txt)
        self._style(self.w[cam]["drv"], " · ".join(parts) or "–", level)

    # --- moving -----------------------------------------------------------------------------

    def _push_demand(self, cam: str, h: float, theta: float) -> None:
        """Set the Soll widgets (sliders + number fields)."""
        w = self.w[cam]
        for key, val in (("h", round(h, 3)), ("t", round(theta, 3))):
            for suffix in ("_slider", "_num"):
                if w[key + suffix].value != val:
                    w[key + suffix].value = val

    def _on_demand(self, cam: str, key: str, value: float | None) -> None:
        if value is None:
            return
        self._push_demand(
            cam,
            value if key == "h" else self.w[cam]["h_slider"].value,
            value if key == "t" else self.w[cam]["t_slider"].value,
        )
        if not self.live:  # offline: the slider IS the model
            if key == "h":
                self.rig.set_pose(cam, h=float(value))
            else:
                self.rig.set_pose(cam, theta_deg=float(value))

    async def _move(self, cam: str, h: float, theta: float) -> None:
        """Offline: set the model. Live: real moves through the sim drives (move_to)."""
        if self.drive_mgr is None:
            self.rig.set_pose(cam, h=h, theta_deg=theta)
            return
        c = self.rig.cam(cam)
        jobs = []
        for axis, ax, val in (("a", c.axis_a, h), ("b", c.axis_b, theta)):
            drv = self.drive_mgr.drives.get(f"{cam}:{axis}")
            if drv is None:
                continue
            # drive position = steps / steps_per_unit (axis a) or raw steps (axis b)
            pos = kin.units_to_steps(ax, val) / getattr(drv, "steps_per_unit", 1.0)
            jobs.append(drv.move_to(pos))
        try:
            await asyncio.gather(*jobs)
        except Exception as exc:
            ui.notify(f"{cam}: Fahrt fehlgeschlagen: {exc}", type="negative")

    async def _go_demand(self, cam: str) -> None:
        w = self.w[cam]
        await self._move(cam, float(w["h_slider"].value), float(w["t_slider"].value))

    async def _ik_go(self, cam: str) -> None:
        w = self.w[cam]
        z = float(w["ik_z"].value or 0.0)
        theta = float(w["ik_angle"].value or 0.0) if w["ik_mode"].value == "fixed" else None
        res = self.rig.solve(cam, z, theta_deg=theta)
        if not res.reachable:
            w["ik_msg"].set_text("Nicht erreichbar: " + _explain(self.rig, cam, res))
            w["ik_msg"].classes(replace="text-sm text-red-600 font-bold")
            return
        w["ik_msg"].set_text(f"→ h = {res.height_mm:.2f} mm, θ = {res.tilt_deg:.2f}°")
        w["ik_msg"].classes(replace="text-sm text-green-700")
        self._push_demand(cam, res.height_mm, res.tilt_deg)
        await self._move(cam, res.height_mm, res.tilt_deg)

    # --- sweep + Soll/Ist -------------------------------------------------------------------

    async def _sweep(self) -> None:
        try:
            zs = [float(x) for x in self.sweep_in.value.replace(";", ",").split(",") if x.strip()]
        except ValueError:
            ui.notify("Ungültige z-Liste", type="negative")
            return
        self.sweep_btn.disable()
        rows: list[dict[str, Any]] = []
        self.sweep_table.rows = []
        try:
            for z in zs:
                results = solve_targets(self.rig, [Target(cam_id=c, z=z) for c in self.cams])
                for r in results:
                    ok = r.reachable
                    rows.append(
                        {
                            "id": f"{len(rows)}",
                            "z": f"{z:+.1f}",
                            "cam": r.target.cam_id,
                            "h": f"{r.height_mm:.2f}" if r.height_mm is not None else "–",
                            "t": f"{r.tilt_deg:.2f}" if r.tilt_deg is not None else "–",
                            "ok": "ja" if ok else "nein: " + _violations_de(r),
                        }
                    )
                    if ok and r.height_mm is not None and r.tilt_deg is not None:
                        self._push_demand(r.target.cam_id, r.height_mm, r.tilt_deg)
                await asyncio.gather(
                    *(
                        self._move(r.target.cam_id, r.height_mm, r.tilt_deg)
                        for r in results
                        if r.reachable and r.height_mm is not None and r.tilt_deg is not None
                    )
                )
                self.sweep_table.rows = list(rows)
                self.sweep_table.update()
                await asyncio.sleep(0.6)
        finally:
            self.sweep_btn.enable()

    def _set_targets_msg(self, text: str) -> None:
        self.tg_msg.set_text(text)

    async def _targets_upload(self, e: Any) -> None:
        file = getattr(e, "file", None)  # NiceGUI >= 3: e.file ; 2.x: e.content
        raw = await file.read() if file is not None else e.content.read()
        self.tg_text.value = raw.decode("utf-8", errors="replace")
        self._targets_compute()

    def _targets_compute(self) -> None:
        try:
            targets = parse_targets(self.tg_text.value)
        except ValueError as exc:
            self.tg_table.rows = []
            self.tg_table.update()
            self._set_targets_msg(f"Fehler: {str(exc).splitlines()[0]}")
            return
        self._set_targets_msg("")
        rows = []
        for i, r in enumerate(solve_targets(self.rig, targets)):
            t = r.target
            known = t.cam_id in self.rig.config.cameras
            ih = self.rig.height(t.cam_id) if known else None
            it = self.rig.tilt(t.cam_id) if known else None

            def f(v: float | None, fmt: str = ".2f") -> str:
                return "–" if v is None else format(v, fmt)

            dh = r.height_mm - ih if r.height_mm is not None and ih is not None else None
            dt = r.tilt_deg - it if r.tilt_deg is not None and it is not None else None
            rows.append(
                {
                    "id": str(i),
                    "cam": t.cam_id,
                    "z": f"{t.z:+.1f}",
                    "mode": MODE_DE[t.mode],
                    "h": f(r.height_mm),
                    "t": f(r.tilt_deg),
                    "ih": f(ih),
                    "it": f(it),
                    "dh": f(dh, "+.2f"),
                    "dt": f(dt, "+.2f"),
                    "ok": "ja" if r.reachable else "nein: " + _violations_de(r),
                }
            )
        self.tg_table.rows = rows
        self.tg_table.update()


# --- registration / entry points -----------------------------------------------------------------


def register_twin_page(rig: RigModel, drive_mgr: DriveManager | None = None, path: str = "/twin"):
    """Register the twin page on the NiceGUI app (call before ``ui.run``).

    ``drive_mgr=None``: offline, sliders set the rig directly. With a ``DriveManager`` the panel
    sends real ``drive.move_to`` calls and only reads the (shared) rig for display.
    """
    _ensure_stl_route(rig)

    @ui.page(path)
    def _twin_page() -> None:
        TwinPage(rig, drive_mgr).build()


@click.command()
@click.option(
    "--rig-config",
    type=click.Path(exists=True, dir_okay=False),
    default=None,
    help="sim_rig.yaml (default: config/sim_rig.yaml)",
)
@click.option("--host", default="127.0.0.1", show_default=True)
@click.option(
    "--port", default=8081, show_default=True, type=int, help="HTTP port (8080 is the Pi GUI)"
)
def cli(rig_config: str | None, host: str, port: int) -> None:
    """Digital twin 3D view, standalone (offline, no MQTT, no drives)."""
    register_twin_page(RigModel.from_yaml(rig_config), path="/")
    ui.run(host=host, port=port, title="OAK Digital Twin", reload=False, show=False)


if __name__ in {"__main__", "__mp_main__"}:
    cli()
