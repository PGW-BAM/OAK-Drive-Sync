# Digital Twin — Kinematik-Kern (Schritt 1)

Headless, reines Python (`math`), keine Hardware, kein MQTT. Dient als Grundlage für
simulierte Drives, simulierte IMU und eine spätere 3D-Ansicht.

## Koordinatensystem (Probenkoordinaten, mm / Grad)

Ursprung in der Schweißnaht-Mitte (Mittelebene), z nach oben entlang der Probe,
x = Normale der Vorderseite (zeigt zu cam1), y über die Breite.
Vorderseite bei x = +t/2, Rückseite bei x = −t/2. Seite s = +1 (cam1) / −1 (cam2).

```
 Seitenansicht (x–z), cam1 (s=+1)

   z ↑
     |        Probe        Pivot P      Linse
     |        |#|             o----------[cam]      Tilt θ > 0 = optische Achse nach oben
     |   Naht →|#|<--- D_pivot --->|<-38->|
     +--------|#|------------------------→ x
            -t/2 +t/2
```

- Pivot `P = (s·(t/2 + D_pivot), 0, h)`, `D_pivot = working_distance − lens_to_pivot`
- Optische Achse `d = (−s·cosθ, 0, sinθ)`, Linse `= P − lens_to_pivot·d`
- Achse a = Höhe `h` (mm), Achse b = Tilt `θ` (Grad) um die horizontale Pivot-Achse (∥ y)
- Exakte IK: `h = z₀ − (D_pivot − f(z₀))·tanθ`; Senkrechtblick: `tanθ = f′(z₀)` (Wulst-Steigung)

## Module

| Datei | Inhalt |
|---|---|
| `src/sim/rig_config.py` | Pydantic-Modelle + `load_rig_config()` für `config/sim_rig.yaml` |
| `src/sim/specimen.py` | Hundeknochen: `half_width(z)`, `surface_x`, `surface_normal`, `mesh()`, `load_specimen_mesh()` (Stub, optional `trimesh`) |
| `src/sim/kinematics.py` | Steps↔mm/Grad, `forward`, `intersect_surface` (flat/bead), `solve_view`, `check_limits` |
| `src/sim/fov.py` | Frustum-Footprint auf der Fläche, Schweißnaht-Abdeckung, GSD |
| `src/sim/rig_model.py` | `RigModel`: Config + aktuelle Steps je Kamera; `pose`, `hit`, `footprint`, `solve`, `move_to`, `imu_angles` |

## Config-Schlüssel (`config/sim_rig.yaml`)

- `specimen`: `length`, `grip_width`, `gauge_width`, `thickness`, `fillet_radius`, `clamp_length`, `bead.{profile,height,width}`
- `cameras.<cam>`: `side`, `working_distance`, `lens_to_pivot`
  - `axis_a` / `axis_b`: `steps_per_unit`, `direction_sign`, `zero_offset_steps`, `travel_min/max`,
    `soft_margin` (Soft-Limit innerhalb des Hubs), `switch_overtravel` bzw. `switch_min/max` (virtuelle Endschalter, Default knapp außerhalb des Hubs)
  - `fov`: `hfov_deg`, `vfov_deg`, `width_px`, `height_px`
  - `imu`: `roll_sign`, `roll_offset_deg`, `pitch_offset_deg` (`roll = sign·θ + offset`)

## Platzhalter / Schätzwerte (nicht verifiziert)

- Fillet-Radius R = 200 mm (Schätzwert aus Zeichnung)
- Schweißwulst: cos²-Profil, 2 mm × 19 mm — bis Scan-Daten vorliegen
- Achse a: `steps_per_unit` (200 steps/mm), Hub ±150 mm, Soft-Margin, Überlauf
- Achse b: `steps_per_unit` = 2000 (aus Getriebe, wird durch Kalibrierung überschrieben); Soft-Margin/Überlauf
- FOV 69° × 55°, 1920×1080 — gegen Datenblatt OAK-D 4 Pro und aktiven Sensor/Auflösung prüfen
- IMU-Einbau-Offset (0) und Endschalter-Positionen (Design offen)

## Tests

```bash
python -m pytest tests/sim -q
```
