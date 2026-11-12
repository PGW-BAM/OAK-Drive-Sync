"""Parametric dogbone specimen with weld bead (pure ``math``).

Specimen frame (mm): origin at weld centre on the mid-plane, z up along the length,
x = front-face normal (toward cam1), y across the width. Front face x=+t/2, back x=-t/2.

Width profile: clamped ends (full width B, ``clamp_length`` each) -> circular fillet radius R
-> parallel gauge (width b). Gauge half-length follows from L, R and clamp_length.
Bead: PLACEHOLDER profile (cos2 / ellipse) on both faces, centred at z=0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path

    from src.sim.rig_config import BeadConfig, SpecimenConfig

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class Mesh:
    vertices: list[Vec3]
    faces: list[tuple[int, int, int]]


class Specimen:
    def __init__(self, cfg: SpecimenConfig) -> None:
        self.cfg = cfg
        self._delta = (cfg.grip_width - cfg.gauge_width) / 2
        r = cfg.fillet_radius
        # axial length of the fillet arc: sqrt(R^2 - (R - delta)^2)
        self.fillet_length = math.sqrt(max(r * r - (r - self._delta) ** 2, 0.0))
        self.gauge_half_length = cfg.length / 2 - cfg.clamp_length - self.fillet_length
        if self.gauge_half_length < 0:
            raise ValueError("specimen too short for clamp_length + fillet")

    @property
    def gauge_length(self) -> float:
        return 2 * self.gauge_half_length

    @property
    def half_thickness(self) -> float:
        return self.cfg.thickness / 2

    # --- width -----------------------------------------------------------
    def half_width(self, z: float) -> float:
        c = self.cfg
        az = abs(z)
        if az <= self.gauge_half_length:
            return c.gauge_width / 2
        s = az - self.gauge_half_length
        if s >= self.fillet_length:
            return c.grip_width / 2
        r = c.fillet_radius
        return c.gauge_width / 2 + r - math.sqrt(r * r - s * s)

    # --- bead ------------------------------------------------------------
    def bead_height(self, z: float) -> float:
        b: BeadConfig = self.cfg.bead
        hw = b.width / 2
        if abs(z) >= hw:
            return 0.0
        if b.profile == "cos2":
            return b.height * math.cos(math.pi * z / b.width) ** 2
        return b.height * math.sqrt(1.0 - (z / hw) ** 2)

    def bead_slope(self, z: float) -> float:
        """d(bead height)/dz."""
        b = self.cfg.bead
        hw = b.width / 2
        if abs(z) >= hw:
            return 0.0
        if b.profile == "cos2":
            return -b.height * (math.pi / b.width) * math.sin(2 * math.pi * z / b.width)
        return -b.height * z / (hw * hw * math.sqrt(max(1.0 - (z / hw) ** 2, 1e-12)))

    # --- surface (side=+1 front, -1 back) ----------------------------------
    def surface_x(self, z: float, side: int) -> float:
        return side * (self.half_thickness + self.bead_height(z))

    def surface_normal(self, z: float, side: int) -> Vec3:
        """Outward unit normal of the face at z (in the x-z plane)."""
        fp = self.bead_slope(z)
        n = math.hypot(1.0, fp)
        return (side / n, 0.0, -fp / n)

    # --- mesh --------------------------------------------------------------
    def mesh(self, nz: int = 61, ny: int = 9) -> Mesh:
        """Coarse triangle mesh (front + back faces, side walls; ends left open)."""
        half_l = self.cfg.length / 2
        bw = self.cfg.bead.width
        zs = sorted(
            {-half_l + i * self.cfg.length / (nz - 1) for i in range(nz)}
            | {-bw / 2 + i * bw / 24 for i in range(25)}
            | {
                s * (self.gauge_half_length + k * self.fillet_length / 8)
                for s in (-1, 1)
                for k in range(9)
            }
        )
        verts: list[Vec3] = []
        faces: list[tuple[int, int, int]] = []
        grids: dict[int, list[list[int]]] = {}
        for side in (1, -1):
            rows: list[list[int]] = []
            for z in zs:
                hw = self.half_width(z)
                row = []
                for j in range(ny):
                    y = -hw + 2 * hw * j / (ny - 1)
                    row.append(len(verts))
                    verts.append((self.surface_x(z, side), y, z))
                rows.append(row)
            grids[side] = rows
            for i in range(len(zs) - 1):
                for j in range(ny - 1):
                    a, b = rows[i][j], rows[i][j + 1]
                    c, d = rows[i + 1][j], rows[i + 1][j + 1]
                    # wind so that normals point outward (+x for front, -x for back)
                    if side == 1:
                        faces += [(a, b, d), (a, d, c)]
                    else:
                        faces += [(a, d, b), (a, c, d)]
        for j in (0, ny - 1):  # side walls
            for i in range(len(zs) - 1):
                f0, f1 = grids[1][i][j], grids[1][i + 1][j]
                b0, b1 = grids[-1][i][j], grids[-1][i + 1][j]
                if j == 0:
                    faces += [(f0, b0, b1), (f0, b1, f1)]
                else:
                    faces += [(f0, b1, b0), (f0, f1, b1)]
        return Mesh(verts, faces)


def load_specimen_mesh(path: str | Path) -> Mesh:
    """Load a scanned specimen mesh (STL/PLY/GLB) in the specimen frame.

    Interface stub for later scan data. Uses ``trimesh`` if installed
    (``pip install trimesh``); otherwise raises NotImplementedError. Not yet used by the
    kinematics (which use the parametric bead profile).
    """
    try:
        import trimesh  # type: ignore[import-not-found]
    except ImportError as exc:
        raise NotImplementedError("mesh loading needs the optional 'trimesh' package") from exc
    m = trimesh.load(str(path), force="mesh")
    return Mesh(
        [(float(v[0]), float(v[1]), float(v[2])) for v in m.vertices],
        [(int(f[0]), int(f[1]), int(f[2])) for f in m.faces],
    )
