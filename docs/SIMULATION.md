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

## Schritt 2: Pi-Steuerung gegen die Simulation

Die bestehende Pi-Logik (`DriveManager`, `DriftDetector.converge`, Auto-Kalibrierung) läuft unverändert
gegen simulierte Antriebe und eine simulierte Kamera-IMU. Die Simulation schreibt die Schrittpositionen
in das gemeinsame `RigModel`, daher folgen Pose und IMU-Winkel der Bewegung.

### Starten

Voraussetzung: ein Mosquitto-Broker (z. B. `mosquitto -p 1883` lokal), Host/Port in der Config.

```bash
oak-pi run --sim --no-gui            # nutzt config/drive_config.sim.yaml
OAK_SIM=1 oak-pi run                 # gleichwertig (Umgebungsvariable)
oak-pi run --config config/drive_config.sim.yaml
```

- `--sim` / `OAK_SIM=1` setzt nur den Config-Default auf `config/drive_config.sim.yaml`.
  Der Sim-Modus selbst wird durch den Top-Level-Schlüssel `sim:` in der Config aktiviert.
- Drive-Typen: `type: sim_linear` (Achse a, wie `GPIODrive`: Position in *Einheiten*,
  Schritte = Einheiten × `steps_per_unit`) und `type: sim_radial` (Achse b, wie `TinkerforgeDrive`:
  Position = rohe Schritte). Hardware-Schlüssel (`step_pin`, `uid`, ...) werden akzeptiert und ignoriert.
- Der Prozess startet zusätzlich einen `SimImuResponder` (eigene MQTT-Verbindung `oak-sim-imu`), der die
  Windows-Kamera-IMU ersetzt: 2 Hz Telemetrie auf `telemetry/cam/{cam}/imu` und Antworten auf
  `cmd/cam/{cam}/imu_check` (mit `request_id`).
- **Achtung Arbeitsverzeichnis:** `data/drive_positions.jsonl`, `config/calibration.yaml`,
  `config/imu_calibration.yaml` und `config/positions.yaml` sind relativ zum CWD fest verdrahtet und werden
  auch im Sim-Modus beschrieben. Sim deshalb aus einem Scratch-Verzeichnis mit Kopie von `config/` starten.
- Die Auto-Kalibrierung erwartet reale Einbaulagen (cam1 ≈ −90°, cam2 ≈ +90° bei Tilt 0). Dafür in einer
  Kopie von `sim_rig.yaml` `imu.roll_offset_deg: -90` (cam1) bzw. `90` (cam2) setzen und per
  `sim.rig_config` verwenden; dann läuft sie in der Sim erfolgreich durch (geprüft). Mit Offset 0 schlägt sie
  erwartungsgemäß fehl (Ziel ±90° nicht erreichbar), die Steuerung läuft trotzdem weiter.

### Was simuliert wird

- Bewegungsdauer aus `max_speed` bzw. `max_velocity` (Schritte/s, skaliert mit `speed`), optional Trapez-Profil
  (`acceleration`/`deceleration`), asynchron, Position alle `update_interval_s` (Default 20 ms) aktualisiert.
- `emergency_stop()` mitten in der Bewegung (Position friert zum Stopp-Zeitpunkt ein, State IDLE).
- Soft-Min/Max und `calibration_mode` wie in den echten Treibern (GPIO: unteres Limit immer, oberes nur
  außerhalb `calibration_mode`; Tinkerforge: beide, in `calibration_mode` keine). Defaults = Soft-Limits aus
  `sim_rig.yaml`, überschreibbar mit `home_position`/`max_position` bzw. `min_position`/`max_position`.
- Virtuelle Endschalter (`switch_min/max` bzw. Überlauf aus `sim_rig.yaml`) wirken auf die *physische*
  Position: Stopp am Schalter, State `FAULT`, `drive.triggered_switch` = `"min"`/`"max"` (physikalischer Sinn,
  unabhängig von `direction_sign`). Das nächste `move_to` quittiert den Fehler.
- Fehlerinjektion (Default aus): `step_loss_fraction` (Anteil verlorener Schritte; der Zähler merkt nichts) und
  `backlash_steps` (Spiel bei jeder Richtungsumkehr). `set_current_position`/`seed_position` ändern nur den
  Zähler, nicht die physische Position (wie beim Resync der echten Hardware).
- `SimImuResponder`: `noise_deg` (Gauß), `settle_tau_s` (Tiefpass-Nachschwingen), `rate_hz`.
- `FakeBroker`/`FakeMQTTClient` (`src/sim/fake_mqtt.py`) für Tests ohne Mosquitto: Wildcards, Retained,
  `dispatch="serial"` (wie der echte `MQTTClient`: Handler werden inline awaited) oder `"concurrent"`.

### Fähigkeits-Property statt `isinstance`

`BaseDrive.supports_angle_feedback` (Default `False`; `True` für `TinkerforgeDrive` und `SimRadialDrive`)
ersetzt in `main.py` die Prüfungen `isinstance(drive, TinkerforgeDrive)`. Der `_simulated`-Check der
Auto-Kalibrierung bleibt (Tinkerforge ohne Bricklet-Bibliothek wird weiter übersprungen); Sim-Radialantriebe
setzen `_simulated = False`.

### Bekannte Einschränkungen

- **Bekannter Bug (nicht behoben):** `MQTTClient.run` awaited jeden Handler inline. `handle_move` → `converge`
  wartet auf die `imu_check`-Antwort, die derselbe Empfangsloop liefern müsste → Timeout, `converge` schlägt
  fehl (FAULT). Im Test: `test_e2e_angle_move_serial_dispatch` (`xfail strict`) vs.
  `test_e2e_angle_move_concurrent_dispatch` (grün). Mit `--sim` gegen einen echten Broker tritt der Fehler
  daher ebenfalls auf, bis der Client Handler nebenläufig ausführt.
- Kein Stall/Motorstrom-Modell, kein Mechanik-Nachschwingen (nur der IMU-Tiefpass), keine Thermik.
- Reale GPIO-Sicherheitszeiten (`direction_change_delay`, `min_move_interval`) und `pulse_delay` werden nicht
  nachgebildet; `speed` wirkt in der Sim (beim echten `GPIODrive` wird es ignoriert).
- `DriveManager` meldet bei Endschalter-FAULT im offenen Pfad keinen `DriveError` (`limit_switch`); der State
  `fault` steht nur im Positions-Status.
- Der Fake-Broker simuliert weder QoS noch LWT noch Reconnects; `stop()` beendet `run()` sofort.
- `_setup_controller` erzeugt einen echten `MQTTClient` (nicht injizierbar); der E2E-Test baut die gleiche
  Verdrahtung daher nach.

## Schritt 4: 3D-Ansicht und Bedienfeld (`src/sim/view.py`)

Interaktive Browser-Ansicht des Aufbaus (NiceGUI `ui.scene`, three.js) plus Bedienfeld. Szene in
Probenkoordinaten (mm, z nach oben): Probe (`Specimen.mesh()`, grau), Schweißnaht (gelbes Band),
Spannbacken (transparent, 125 mm je Ende), je Kamera Gehäuse an der Linsenpose, Pivot-Achse, Bogen des
Tilt-Bereichs (Soft-Limits amber, Endschalter rot), vertikale Schiene mit Soft-/Endschalter-Markern,
optische Achse bis zum Treffpunkt, FOV-Frustum und Footprint auf der Fläche (grün = Schweißnaht im Bild,
rot = nicht im Bild). Kamerapresets: Seite, Vorne, Iso (Maus: drehen/zoomen).

### Starten

```bash
pip install nicegui                    # bzw. uv pip install -e ".[sim]"
python -m src.sim.view                 # Offline: http://127.0.0.1:8081  (Option --port, --host, --rig-config)
oak-twin --port 8081                   # gleichwertig (Console-Script)

oak-pi run --sim                       # Live: zusätzlich Seite http://<pi>:8080/twin
```

- **Offline:** Slider/Zahlenfelder setzen das `RigModel` direkt (sofort wirksam), kein MQTT, keine Antriebe.
  Standard-Port 8081, damit er nicht mit der Pi-GUI (8080) kollidiert.
- **Live (`--sim` mit GUI):** `main.py` registriert die Seite `/twin` an der bestehenden NiceGUI-App und bindet
  sie an `get_shared_rig()`; ein 10-Hz-Timer liest die Pose, während die Sim-Antriebe fahren. Die Fahr-Buttons
  („Fahren“, „Anfahren“, Sweep) rufen `drive.move_to(...)` über den `DriveManager` auf (Soft-Limits, Endschalter,
  Bewegungsprofil der Sim-Antriebe wirken). Das ist der direkte Drive-Aufruf, nicht der MQTT-Pfad
  `DriveManager.handle_move`. Zusätzliche Zeile „Antriebe“ zeigt State (moving/reached/fault) und ausgelösten Endschalter.

### Bedienfeld je Kamera

- Slider + Zahlenfeld für Höhe h (mm) und Tilt θ (°) innerhalb des Hubs, darunter Ist-Wert und Schritte
  je Achse (a/b). Offline wirken sie sofort; live nur über „Fahren (über Antriebe)“.
- Anzeigen: Treffer-z, Abstand Linse→Treffer, Einfallswinkel zur Flächennormalen, Footprint B × H, GSD (h × v),
  Naht im Bild (Abdeckung der Nahtlinie), IMU Roll/Pitch, Limit je Achse (OK / Soft-Limit amber /
  ENDSCHALTER rot).
- IK: Ziel-z (Default 0 = Naht), Modus „senkrecht zur Oberfläche“ oder „fester Winkel“, Button „Anfahren“
  (`RigModel.solve`). Nicht erreichbar → rote Meldung mit Grenze und Wert, es wird nicht gefahren.
- Sweep: Liste von Ziel-z-Werten (Default −20…+20), fährt beide Kameras nacheinander an (senkrechte Sicht) und
  füllt eine Tabelle (Ziel-z, Kamera, h, θ, erreichbar).

### Soll/Ist-Ziele (provisorisches JSON-Format, `src/sim/targets.py`)

Liste von Zielen; Textfeld oder JSON-Datei-Upload. „Berechnen“ zeigt Soll (IK), Ist (aktuelle Pose), Δ und
Erreichbarkeit. Headless nutzbar: `parse_targets(text)` und `solve_targets(rig, targets)`.

```json
[
  {"cam_id": "cam1", "z": 0.0},
  {"cam_id": "cam1", "z": 12.0, "theta_deg": 10.0},
  {"cam_id": "cam2", "z": 0.0, "normal": [-0.97, 0.0, 0.24]}
]
```

| Feld | Bedeutung |
|---|---|
| `cam_id` | Kamera (`cam1`, `cam2`) — Pflicht |
| `z` | Zielhöhe auf der Probenoberfläche in mm (Default 0 = Naht) |
| `normal` | optional `[nx, ny, nz]`: Flächennormale (Probenkoordinaten, aus der Fläche heraus), zu der die Kamera antiparallel blicken soll. `ny` muss ≈ 0 sein (keine Gier-Achse), `nx` muss zur Kamera zeigen (cam1 > 0, cam2 < 0), sonst „nicht erreichbar“ |
| `theta_deg` | optional fester Tilt (nur Höhe wird gelöst); schließt `normal` aus |

Ohne `normal`/`theta_deg`: senkrecht zur lokalen Oberfläche (inkl. Wulstneigung). Das Format ist provisorisch
und noch nicht mit einer realen Soll/Ist-Quelle abgestimmt.
