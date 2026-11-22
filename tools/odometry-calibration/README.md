# Odometry vs. Calibration-Board — Arbeitspraktiken

Wie man den Roboter fährt und die **interne (kaputte) STM32-Odometry** gegen die
**genaue Calibration-Board-Odometry (Ground Truth)** misst.

Hintergrund: Die On-Board-Odometry stimmt nicht, weil der PAA-Optical-Flow-Sensor
nicht im Rotationszentrum sitzt — Drehungen erzeugen Schein-Translation, und schon
auf der Geraden gibt es einen Skalenfehler. Das **Calibration-Board** (separater
STM32 + PAA + ICM-IMU, am Roboter montiert) liefert die genaue Referenz.

---

## 0. Setup / Voraussetzungen

- **Pi:** `pi@192.168.178.150` (Port 8421). SSH-Key vorhanden.
- **Services müssen laufen** (laufen normalerweise per systemd, `enabled`):
  - `raccoon-calib-bridge` — liest `/dev/ttyACM0`, publiziert `hardware/calib_board/*`
  - `stm32_data_reader` — interne Odometry/Motoren über SPI, publiziert `hardware/odometry/*`
  ```bash
  ssh pi@192.168.178.150 'systemctl is-active raccoon-calib-bridge stm32_data_reader'
  ```
- **Board-Gesundheit prüfen** (siehe Schritt 2): `state/board = connected`,
  `state/paa = connected`, `paa/state/cal_valid = 1`, `squal` möglichst hoch (>~60).
- **Projekt:** `.../competition/Ecer2026/packingbot`, gefahren mit der dortigen
  **uv-Instanz**: `uv run raccoon run`.

---

## 1. Fahren — `uv run raccoon run`

```bash
cd .../competition/Ecer2026/packingbot
uv run raccoon run            # sync -> codegen -> führt src.main auf dem Pi aus
```

- Reihenfolge: **Setup-Missions zuerst** (`config/missions.yml`, `M000SetupMission`
  ist als `setup` getaggt), dann **Light-Start-Gate**, dann reguläre Missions, dann
  Shutdown. Die Setup-Mission (aktuell `drive_forward(20)`) fährt also **sofort**,
  noch vor dem Gate.
- **Non-interaktiv:** ohne physischen Light-Start hängt der Lauf nach dem Setup am
  Gate. Das ist ok — die Setup-Fahrt ist da schon passiert. Mit Timeout starten und
  danach beenden:
  ```bash
  timeout 60 uv run --no-sync raccoon --no-validate run
  ```
  `uv run` ignoriert SIGTERM teils → der Prozess wird hart beendet. Danach prüfen,
  dass auf dem Pi nichts mehr läuft (Motoren stehen nach `drive_forward` ohnehin):
  ```bash
  ssh pi@192.168.178.150 'pgrep -fa src.main'   # sollte leer sein
  ```
- `--dev` = Button statt Light. `--no-calibrate` nutzt gespeicherte Kalibrierung.

---

## 2. Live-Daten lesen (Ground Truth + intern)

> `rring.py` abonniert die Kanäle über das Python-`raccoon_transport` und merkt sich den
> letzten Frame jedes Kanals. Es läuft **auf dem Pi**, wo der Transport liegt.

```bash
scp rring.py odolog.py pi@192.168.178.150:/tmp/
ssh pi@192.168.178.150 'python3 /tmp/rring.py'   # Health + beide Odometry-Quellen
```

Wichtige Channels:

| Channel | Einheit | Bedeutung |
|---|---|---|
| `hardware/calib_board/odom/state/pos_x` / `pos_y` / `heading` | cm / cm / deg | **Ground Truth** (PAA + IMU fusioniert) |
| `hardware/calib_board/state/board`, `state/paa` | string | `connected` |
| `hardware/calib_board/paa/state/cal_valid`, `paa/state/squal` | int | Kal. gültig (1), Oberflächenqualität (0..169) |
| `hardware/odometry/state/pos_x` / `pos_y` / `heading` | m / m / rad | **interne** STM32-Dead-Reckoning (die kaputte) |
| `hardware/odometry/state/vx` / `vy` / `wz` | m/s, rad/s | interne Geschwindigkeiten |

Calib-Odometry läuft **frei** (kein Reset im deployten Stand) → immer **Deltas**
über das Bewegungsfenster auswerten, nicht Absolutwerte. Interne Odometry wird vom
Motion-Step zu Beginn auf 0 gesetzt (robot frame: x=vorwärts, y=seitlich).

---

## 3. Messlauf (fahren + beide Quellen mitloggen + auswerten)

```bash
# 1) Logger auf dem Pi starten (z.B. 75 s Fenster)
scp rring.py odolog.py pi@192.168.178.150:/tmp/
ssh pi@192.168.178.150 'nohup python3 /tmp/odolog.py 75 /tmp/drive_log.csv >/tmp/odolog.out 2>&1 & echo PID $!'

# 2) Fahren
cd .../packingbot && timeout 60 uv run --no-sync raccoon --no-validate run

# 3) CSV holen + auswerten
scp pi@192.168.178.150:/tmp/drive_log.csv .
python3 analyze.py drive_log.csv
```

`analyze.py` findet das Bewegungsfenster über die interne Odometry und gibt
Netto-Verschiebung + Pfadlänge beider Quellen sowie den **Skalenfehler**
(intern / Ground-Truth) aus.

**Referenzergebnis** (`drive_forward(20)`, 2026-06-14): kommandiert 20 cm,
intern ~19,5 cm, real (Ground Truth) **~16,9 cm** → interne Odometry überschätzt
um ~15–18 %, schon auf der Geraden. Frames sind gegeneinander verdreht
(Calib-Heading ~4,7°) → Beträge/straight-line vergleichen, nicht rohe x/y.

> Der **große** Rotationszentrum-Fehler zeigt sich bei **Drehungen**
> (`turn_left/right`) — dann ist `heading change` groß und die Ground-Truth-
> Translation sollte ~0 sein, während die interne Odometry driftet.

---

## 4. Sauber integriert (optional, braucht Deploy)

Die uncommitteten raccoon-lib-Änderungen (`OdometrySource`, Auto-Switch in
`WombatOdometry`, `get_internal_*`-Bindings) machen den Vergleich direkt über
`robot.odometry` möglich — statt der Transport-Abos in `rring.py`. Dazu deployen:

```bash
cd .../raccoon-lib && RPI_HOST=192.168.178.150 bash deploy.sh   # Docker ARM64 cross-build
```

### Automatischer BEMF→rad-Tuner

Die manuelle Messung oben ist jetzt als vollautomatischer DSL-Step verfügbar:
`auto_tune_bemf_velocity()` (siehe `docs/bemf-velocity-autotune.md`). Er fährt einen
PWM-Sweep, misst `ticks_to_rad` pro Motor gegen das Calib-Board, prüft Linearität und
legt Daten + Plots unter `.raccoon/auto_tune/bemf_velocity/<ts>/` ab.

### Direkte Odometry-API (nach Deploy)

Danach in einer Mission/Step: `robot.odometry.get_distance_from_origin()` (aktive
Quelle = Calib-Board, wenn erkannt) vs. `robot.odometry.get_internal_distance_from_origin()`
(immer STM32) und `robot.odometry.get_active_source()`. Aktuell auf dem Pi **nicht**
deployt (lib 1.0.0), daher der Weg über `rring.py` oben.
```
