# RaccoonLib — Developer-Experience & API Review

**Datum:** 2026-07-12
**Reviewer:** externe, strenge DX-/API-Durchsicht (Fokus: Code-Schönheit, API-Ergonomie, Erweiterbarkeit)
**Evidenzbasis:** zwei reale Wettbewerbs-Bots auf RaccoonLib — `cube-bot` (leicht, Mission-lastig, lib 1.0.139) und `drumbot` (~10k LOC, Services/Custom-Steps/Vision/UI, lib 1.0.135) — plus die Public-API-Stubs (`raccoon/**/*.pyi`, lib 1.0.161) und die Doku (`raccoon-docs`).

Dieses Dokument ist bewusst **kritisch**. Es soll RaccoonLib besser machen, nicht loben. Was gut ist, steht kurz in [§1](#1-was-wirklich-gut-ist); der Rest sind Baustellen.

> **Leitgedanke des Frameworks:** *„Raise the Floor, Don't Lower the Ceiling."* Genau daran wird hier gemessen. RaccoonLib hebt den Floor mit einem starken deklarativen Kern und einer guten Service/UI/DI-Schicht — aber **API-Wildwuchs** und **unrobuste Rand-Subsysteme** senken den Floor an vielen Stellen wieder.

---

## Inhalt
- [1. Was wirklich gut ist](#1-was-wirklich-gut-ist)
- [2. Die drei Kernprobleme](#2-die-drei-kernprobleme)
- [2b. Idiome, die in Wahrheit Workarounds sind](#2b-idiome-die-in-wahrheit-workarounds-sind)
- [2c. Erweiterte Idiom-/Gap-Funde (5-Agent-Sweep)](#2c-erweiterte-idiom-gap-funde-5-agent-sweep)
- [3. API-Oberfläche: Wildwuchs](#3-api-oberfläche-wildwuchs)
- [4. Mission-Ergonomie: statischer Baum & Nesting](#4-mission-ergonomie-statischer-baum--nesting)
- [5. Custom-Steps erweitern: Zeremonie & zwei Modelle](#5-custom-steps-erweitern-zeremonie--zwei-modelle)
- [6. Rand-Subsysteme sind nicht robust (Beweis: Monkeypatches)](#6-rand-subsysteme-sind-nicht-robust)
- [7. Vision, Hardware, Config](#7-vision-hardware-config)
- [8. Doku & Distribution](#8-doku--distribution)
- [9. Priorisierte Cleanup-Roadmap (Top 12)](#9-priorisierte-cleanup-roadmap)

---

## 1. Was wirklich gut ist

Nicht wegräumen — das trägt das Framework:

- **`get_service(ServiceClass)`-DI** (`robot/service.py`). Der komplette Vertrag ist `RobotService.__init__(self, robot)`, Services werden pro Robot gecacht. Das ist die Schicht, die drumbot 682 Zeilen Drum-Logik aus dem Mission-Baum ziehen ließ. Sauber, keine Reibung.
- **`condition.py`** ist vorbildlich: komposable `StopCondition` mit strenger Typvalidierung, guten Fehlermeldungen, One-Shot-Logging und sehr guter Doku zu `after_cm` vs. `after_forward_cm`.
- **`MotionStep` (`on_start`/`on_update`/`on_stop`)** ist der *richtig* designte Erweiterungspunkt: fester Loop, dt-Handling, `hard_stop`-Cleanup (`motion_step.py`). `scan_sweep_step`/`turn_to_peak_step` erben ihn sauber.
- **UI-Kit + `UIScreen[T]`/`UIStep`**: deklaratives `build() -> Widget` (Row/Column/Card/…), `@on_click("id")`/`@on_change("id")`, `self.close(result)`/`self.refresh()`, One-Liner `self.confirm/message/input_number/choose`. drumbots Kalibrier-UIs beweisen: das ist produktiv und low-ceremony. Die große Widget-Zahl ist dadurch gerechtfertigt.
- **Auto-Tune** (Relay-Feedback statt Raten) und das **`seq()/parallel()/background()`-Missionsmodell** für den Happy-Path.
- **Hardware-Config `yaml → defs.py`-Codegen** ist ein solides Source-of-Truth-Muster.

---

## 2. Die drei Kernprobleme

Beide Bots — der leichte und der schwere — stoßen auf **dieselben drei** Framework-Schwächen. Nur die Symptome eskalieren mit der Komplexität:

| # | Problem | cube-bot (leicht) | drumbot (schwer) |
|---|---------|-------------------|------------------|
| A | **Kein Wertetransport/Verzweigung zwischen Steps** | `nonlocal`-Closures in `defer` | **modul-`global`** State + 9× `defer` in einer Datei |
| B | **Custom-Steps sind zeremoniell & nicht simulierbar** | wenige, inline; `RACCOON_SIM=1`-Handbau | **29 Klassen → 62 `@dsl`-Sites**, 0 `to_simulation_step` |
| C | **Rand-Subsysteme unrobust** | `drive_to_analog_target` → `trash/`, 729-Z.-Eigenbau | **2× Monkeypatch** von Lib-Internals |

Dazu kommt der **API-Oberflächen-Wildwuchs** (§3), der unabhängig von der Komplexität jeden trifft.

---

## 2b. Idiome, die in Wahrheit Workarounds sind 🔴

Die aussagekräftigsten User-Pains sind **nicht** die, über die geklagt wird — sondern die *Idiome, die zur Gewohnheit geworden sind, weil eine Fähigkeit fehlt*. Ein Idiom, das überall auftaucht, ist oft ein fehlendes Feature in Verkleidung. Hier die belegten Fälle aus beiden Bots:

| Idiom (was man tippt) | Verborgene Lücke | Beleg | Status |
|---|---|---|---|
| `background(Defs.arm_claw.grab())` um einen **einzelnen** Servo/Arm-Move | **Servo-Steps blockieren immer**, kein `blocking=False` | Quelle: `SetServoPosition._execute_step` macht `await asyncio.sleep(estimate_servo_move_time(...))` (steps.py:126–130). cube-bot m070:55, m090:144, m060:45,74, m030:25; drumbot m040:15 | ✅ **`blocking=False` in Planung** — deckt genau das ab |
| `wait_for_seconds(0.1) # make sure the servo movement is done` **direkt nach** einem (schon blockierenden) Move | **Open-Loop-Servos: der geschätzte `estimate_servo_move_time` ≠ Realität unter Last** — es gibt kein „wirklich am Ziel angekommen"-Signal | cube-bot m070:43, m050:75 („definatly on his right posission") | ❌ offen — `blocking=False` hilft hier *nicht*; braucht Positions-Feedback oder einen `until_servo_settled()`-Ersatz |
| `fully_disable_servos() # don't press down on the cube too hard` | **Keine Pro-Servo-Drehmoment-/Compliance-Kontrolle** — der einzige Weg, „nicht mehr drücken" ist *alle* Servos abschalten | cube-bot m070:44; 7 Vorkommen | ❌ offen — braucht „halte mit niedrigem Torque" / „geh auf diesem einen Servo limp" |
| `wait_for_seconds(0.3) # make sure we are still / so front doesn't lift / jerky stop` | **Kein Settle-/Jerk-Limit-Primitiv** — nach einer Motion schwingt/kippt der Bot nach, kein `wait_until_settled()` | cube-bot m070:53,97,101; m060:67 | ❌ offen — ein „still"-Condition oder jerk-limitierter Stop würde die blinden Sleeps ersetzen |
| `mark_heading_reference()` **erneut** aufrufen | **IMU-Heading driftet statisch, keine Auto-Kompensation** | drumbot m020:121 `# re-mark heading reference (because of static imu drift)` + `drum_pipe_heading_mark_one/two` mit Toleranz-Gate | ❌ offen — Drift-Kompensation oder ein sauberes „re-zero heading"-Primitiv |
| Turn-Strategie umbauen `# instead of force dir cause its broken` | **`force_direction` bei Turns ist kaputt** | drumbot m030:52 | ❌ Bug — fixen, dann Workaround entfernen |
| `run(lambda robot: None)` / `seq([])` | **Kein `noop()`** (siehe §4.3) | mehrfach | ❌ offen |
| `settle_duration=9999, grace_period=9998` | **Kein `None`/`disabled` für Limits** (siehe §4.3) | drumbot m020 | ❌ offen |
| `background(..., name="x")` + `wait_for_background("x")` | **Keine typisierten Nebenläufigkeits-Handles** (siehe §4.3) | beide Bots | ❌ offen |

**Kernbeobachtung für die Servo-/Aktuator-Ecke:** Es sind **drei getrennte Lücken**, die leicht als eine erscheinen. `blocking=False` löst nur die erste (Fire-and-Forget). Die zweite (blindes `wait_for_seconds` nach dem Move) und dritte (`fully_disable_servos` als Holzhammer) kommen aus demselben Wurzelproblem: **Servos sind open-loop, das Framework kann Bewegungszeit nur *schätzen* und Torque nur *ganz oder gar nicht*.** Solange es kein Positions-/Last-Feedback (oder wenigstens ein pro-Servo-Torque-Flag) gibt, werden die Teams weiter blind sleepen und pauschal disablen — auch mit `blocking=False`.

**Methodik-Tipp für RaccoonLib:** Ein `background()` um einen Einzel-Actuator-Move, ein `wait_for_seconds` mit „make sure …"-Kommentar, ein `seq([])`, eine `9999`-Konstante — das sind **Symptom-Marker**. Ein Grep danach über die Wettbewerbs-Bots (`grep -rn "background(" src/missions | …`) ist ein billiger, wiederholbarer „fehlende-Features-Detektor" für jede Release.

---

## 2c. Erweiterte Idiom-/Gap-Funde (5-Agent-Sweep)

Ein systematischer Sweep mit fünf parallelen Analysen (je eine Linse: Motion, Sensorik, Control-Flow, Custom-Steps, Config) über beide Bots. Hier die **neuen**, oben noch nicht genannten Funde, nach Linse gruppiert. Format: *Idiom → verborgene Lücke → Beleg*.

### Bewegung / Ausrichtung
- **Overshoot-then-correct-Zentrierung** → das Strafe-Lineup-Primitive braucht **zwei** IR-Sensoren, das Chassis hat nur einen → *jede* laterale Ausrichtung wird als „vorbeifahren + zurück" von Hand gebaut. **~6×** (cube m050:63, m070:102, m000:96 auf beiden Achsen, m030:33). *Kein Single-Sensor-Centering-Primitive.*
- **`heading=0` 57× / `after_cm` 39×** (viele als Fudge-Nudge mit „overshoot/avoid the white dot"-Kommentar) → kein Heading-Frame-Scope (§4.3) und **kein „Stop N cm nach Condition"-Offset-Primitive**.
- **`over_line + after_cm(N) + over_line`** (gleicher Sensor) = „überquere die erste Linie, fahr N cm blind, fang die zweite" → **kein Line-Counting-/„Nte Überquerung"-Condition** (m080:67,83,127).
- **Opposing-Jiggle** `drive_backward(cm=5); drive_forward(cm=5)` zum Losrütteln eines verklemmten Teils → **kein Unstick-/Recovery-Motion-Primitive** (m030:70, m040:28, m080:110).
- **`switch_calibration_set("upper"/"default")`** nicht an Map-Region/Scope gebunden → globaler Modus, asymmetrisch gesetzt (m060:90) und woanders gecleart (m080:105) — leicht im falschen Zustand zu hinterlassen.
- **`DriveUntilImpact` als Präsenz-Probe mit totem `else`-Zweig** (m080:37-59) → **kein „Objekt in Distanz da?"-Query**; Präsenz aus „wie weit kam die Impact-Fahrt" erschlossen.

### Sensorik / Perception / Kalibrierung
- **Kein Flank-/Hysterese-Analog-Condition** (nur statische `on_analog_above/below`) → `on_analog_flank` als 150-Z.-Custom-Condition + `RangeFinder` Median-Filter + Schmitt-Trigger von Hand.
- **Kein Heading-„weggedrückt/hängt"-Condition** → **zweimal unabhängig nachgebaut** (`OnImuHang` aus Odometry, `_heading_stuck` aus IMU). `stall_detected` gibt's nur für Motor-Ticks.
- **Kalibrierung ist mean-only, kein Quality/Verify-Contract** → 729-Z.-Rebuild + pro Kalibrator erfundene SNR-/Replay-Gates (`MIN_DETECTION_RATE=0.9`, `amplitude ≥ 3·noise`).
- **Kein `on_analog_near`/Range-Query** → `is_sensor_in_calibration_range` hand-rolled, **mit latentem Bug** (fehlendes `abs()`, m090:56).
- **Kein gefilterter/percentil/Median-Read** → robuste Statistik **3× unterschiedlich** hand-codiert (trimmed-percentile / median / p95).
- **Kalibrier-Store nicht an Sensor-Lifecycle gebunden** → bei jedem Reconnect manuell re-pushen (`_apply_stored_calibration`).
- **Vision unzuverlässig** → `learned_timeout`, `guess_color()`→„blue"-Default, „stuck-drum"-Erkennung aus Detection-Timing → open-loop-Notausgänge (direkt in `todo` gewünscht).

### Control-Flow / Concurrency — die zwei Wurzel-Lücken
- 🔴 **Kein Step-zu-Step-Wertekanal** → treibt **16 `defer`-Sites**, **7 `global`/Closure-Kanäle**, ein **Step-Objekt-als-Return-Channel** (`approach.impact_result`, m080:38) und **`os.environ` als Missions-übergreifender Kanal** (`DRUMBOT_NO_POSITION_HOLD`).
- 🔴 **Kein Supervised-Concurrency/Monitored-Loop-Konstrukt** → `collect_drums_step.py` sind **450 Z. rohes `asyncio`** (Watchdog-Tasks, manuelle `for`-Loop mit `await run_step`, Emergency-Transitions); `drum_lineup_step.py` ist eine **6-`defer`-State-Machine** — dieselbe Lücke, andersherum gelöst.
- **`parallel(motion, servo)` 13× nur zum Überlappen** → kein `overlap()`/`with_arm()`-Kombinator.
- **`wait_until_distance(15)` in `parallel`** zum Timing eines Aktuators gegen einen Drive — **nur 1× im ganzen Code benutzt** (Primitive zu umständlich) → **kein „bei Distanz X → tu Y"-Event**.
- **`wait_for_checkpoint(60+18) # don't collide with drum-bot`** → **kein Inter-Roboter-Sync**, nur Magic-Sekunden gegen die Wall-Clock.

### Custom-Steps / Services (Framework-Lückenfüller)
- **`RobotService` = 23 Zeilen, keine Lifecycle-Hooks** (kein `on_start`/`on_stop`/`on_shutdown`) → `camera_lifecycle_step`, manuelles asyncio-Task-Teardown, und **keine Service-Komposition** → ein Mixin (`DrumMotorCalibrationMixin`) nur um das 682-Z.-God-Object zu zerteilen.
- **Kein Progress-Stall+Retry-mit-Backoff-Motion-Primitive** → `_make_stall_checker`/`_retry_on_stall` (Watchdog ist „between-step only").
- **Kein Motion+Sampler-Kombinator** → `TurnMotion(...)` pro Step neu verdrahtet (`scan_sweep`, `turn_to_peak`).
- **Keine State-Machine-Fazilität** → `eject_mode`/`motor_locked`-Flags (drum) + Ring-Buffer-Allocator (`SortingService`, 208 Z.).
- **`step/sim/` ist ein leeres `__init__`** → Simulation per **Fake-Service-Tausch** (`FakeColorDetectionService` 137 Z., `fake_camera_step`), nicht per Step-Sim-Hook. Bestätigt §5.3 auf Framework-Ebene.
- **Keine Managed-Background-Task-Utility** → `_track_position_loop` von Hand (create_task/cancel/drain) — wiederkehrendes Muster (vgl. Memory `localization-never-on-hotpath`).
- **Keine Common-Screen-Library** → 12 bespoke `UIScreen`-Subklassen für wiederkehrende „Badge + Text + Confirm"-Muster.

### Config / Hardware / Lifecycle
- 🔴 **YAML↔`defs.py`-Drift mit stillem Datenverlust:** die aktuell getunten Werte leben **nur** im gitignorierten `defs.py`; ein `raccoon run`-Regen **überschreibt sie aus der committeten YAML** — **8 bestätigte Drift-Punkte** (z. B. `lift_drums_servo.eject_position` 113→**126**, `arm_base._0deg` 100→**105**, `arm_elbow.offset` 5.8→**-4.5**; `arm_claw.cube_stack_regrab_open` in defs **gedroppt**). Kein Round-Trip, kein Drift-Check.
- 🔴 **`robot.defs.et_range_finder` referenziert ein Device, das in KEINER Config existiert** → der ganze `RangeFinderService`/`range_finder/*`-Baum instanziiert gegen einen nicht-existenten Def, sichtbar erst zur Laufzeit als `AttributeError`. Ungetypter `defs.*`-Zugriff: **73 (drum) / 142 (cube)** Sites; `.value`-Position-Access **20** Sites.
- **`# ── Link lengths — mirror of config/servos.yml arm_geometry ──`** → der gespiegelte YAML-Key **existiert gar nicht**; der Kommentar dokumentiert eine Sync-Pflicht, die das Framework nicht einlösen kann (kein Arm-Geometrie-Schema).
- **Zwei Quellen für denselben Knopf:** `DRUMBOT_NO_POSITION_HOLD` in `run-configurations.yml` **und** zur Laufzeit gesetzt; `first_cube_line_gap` code-default **26.0** ≠ persisted **30.0** in `racoon.calibration.yml`.
- **Placeholder-`MotorCalibration(1.0, 1.0)`** wird still akzeptiert (siehe §6.3) — kein „uncalibrated"-State.
- **`racoon.calibration.yml`** (ein „c") — ein framework-diktierter, **load-bearing Tippfehler** (per `.gitignore`-Glob gematcht), lokal nicht fixbar. `raccoon.project.yml` hat zwei „c".
- **`SensorGroup(left=…)` einelementig akzeptiert** (`rear`, 36× genutzt) — keine Arity-Validierung.

---

## 3. API-Oberfläche: Wildwuchs

### 3.1 Viele explizite Richtungsnamen — gewollt, aber inkonsistent umgesetzt 🟠
> **Klarstellung (Autoren-Feedback):** Die Teams beider Bots **mögen die expliziten Namen** — `strafe_left(5)`, `forward_lineup_on_black`, `turn_left(...)`. Am Call-Site sind sie lesbarer, per Autocomplete auffindbar und vermeiden Vorzeichen-Verwirrung (`drive(strafe=-5)` — welches Vorzeichen ist „links"?). **Diese explizite Oberfläche ist ein bewusstes Design und soll bleiben.** Der Kritikpunkt ist also *nicht* „ersetzt das durch signierte Parameter", sondern die zwei Probleme unten.

Der Bestand an expliziten Namen ist groß:
- **Drive/Strafe:** `drive_forward` / `drive_backward` / `strafe_left` / `strafe_right` (+ 4 Builder).
- **Lineup:** 8 Funktionen als Kreuzprodukt `{forward,backward,strafe_left,strafe_right} × {on_black,on_white}`, plus `forward_single_lineup`/`backward_single_lineup`.
- **Wall-Align:** 4. **Arcs:** 4. **Turn:** `turn_left/right` + `turn_to_heading_left/right`. **Motor-Stop:** `motor_off/brake/passive_brake/stop_motor`. **Auto-Tune:** 7.

**Problem 1 — gemischte Philosophie (der eigentliche Smell) 🟠.** Das Framework ist *nicht durchgängig* explizit: dieselbe Codebasis nutzt an anderer Stelle signierte Parameter — `directional_follow_line(heading_speed=…, strafe_speed=…)`, `line_follow().move(forward=1, strafe=-0.3)`, `drive_angle(-140)`. Wenn „explizite Namen" der Hausstil ist, sind *diese* die Ausreißer. Ein Entwickler muss pro Subsystem raten, welche Konvention gilt. → **Eine Oberflächen-Philosophie wählen und überall anwenden** (hier: explizit), die signed-param-Ecken angleichen.

**Problem 2 — die Vervielfachung ist nicht die Richtung, sondern Class × dsl × Builder pro Name.** Jeder explizite Name kostet nicht 1, sondern bis zu 3 Symbole (`StrafeLeft` Klasse + `strafe_left` dsl + `StrafeLeftBuilder`). *Das* ist die Redundanz, die es zu killen gilt — unabhängig davon, ob der Name explizit oder parametrisiert ist (siehe §3.2). Die expliziten Call-Site-Namen bleiben; nur das Backing dahinter wird eins.

**Empfehlung (revidiert):** Expliziten Namen behalten. Aber (a) *eine* Impl-Basis pro Familie, aus der alle Richtungsvarianten **generiert** werden (damit ein Bugfix/Feature einen Ort trifft, nicht vier), und (b) die verbleibenden signed-param-APIs an den expliziten Hausstil angleichen. Die `.pyi`-Header („Auto-generated … Source: drive.py") zeigen, dass der Generator schon existiert — er muss nur konsistent für *alle* Familien laufen und die Class/Builder-Doppelung nicht mit vervielfachen.

### 3.2 Zwei bis drei parallele APIs für dieselbe Sache 🔴
Line-Following existiert als **8 flache Funktionen** (`follow_line`, `follow_line_single`, `directional_follow_line`, `strafe_follow_line`, …) **und** als **Fluent-Builder** `line_follow().single().move().correct_lateral().pid().until()` — plus 5 `*Config`-Dataclasses + 3 Enums. **~20 Namen für eine Fähigkeit.** Die Doku/Docstrings zeigen die Funktions-Form, beide realen Bots nutzen fast ausschließlich den Builder. Ein Neuling weiß nicht, welche „die richtige" ist.

Als Struktur des ganzen `__all__`: fast jeder Step existiert als `PascalCase`-Klasse **und** `snake_case`-DSL-Funktion (`WaitForSeconds`/`wait_for_seconds`, `IfThen`/`if_then`, alle `SetMotor*`/`set_motor_*` …), oft plus Builder. **Über die Hälfte der 360 `__all__`-Namen ist diese 1:1(:1)-Doppelung.**

### 3.3 Flacher 360-Namen-Namespace erzwingt `from raccoon import *` 🔴
`__init__.pyi` flacht **125 Submodule** in einen Root-Namespace mit **360 Einträgen**. Real macht dadurch *jede* Datei in beiden Bots `from raccoon import *` (cube-bot sogar als „project convention" kommentiert). Autocomplete unbrauchbar, Namenskollisionen vorprogrammiert (`servo` Funktion vs. `Servo` Klasse vs. `ServoPreset`), Herkunft nie sichtbar.

**Handfester Beleg für fehlende Sorgfalt:** `__all__` enthält **10 doppelte Einträge** (`line_follow`, `ServoPreset`, `Defer`, `defer`, `Run`, `run`, `TableMap`, `MapSegment`, `UIStep`, `UIScreen`) — die Liste wird aus `_step_all`/`_ui_all`/`_robot_all` **ohne Dedup** zusammengeklebt.

### 3.4 Ein Konzept, viele Namen — teils mit stillen Einheiten-Wechseln 🟠
- **Geschwindigkeit:** `speed`, `speed_scale`, `forward_speed`, `strafe_speed`, `heading_speed`, `velocity` — 6 Namen. Schlimmer: **gleicher Name ≠ gleiche Einheit.** `drive_forward(speed=)` ist ein *Bruchteil 0–1*, `wall_align_forward(speed=)` laut Docstring *m/s*.
- **Distanz:** `cm` (`drive_forward`) vs. `distance_cm` (`directional_follow_line`).
- **`forward` doppeldeutig:** `drive_forward(cm=)` → *Richtung*; `line_follow().move(forward=1)` → *Speed-Bruchteil*. Selbes Wort, orthogonale Bedeutung, selbes Subsystem.

### 3.5 Doppelte Operatoren & doppelte `until` 🟠
- `.until()` gibt es als **Kwarg** *und* als **Chain-Methode** auf praktisch jedem Motion-Primitive.
- Bedingungs-Verkettung „dann" existiert als `>` **und** `+`. `+` wurde nur nachgerüstet, um Pythons chained-comparison-Falle von `>` zu umgehen (samt `__bool__`-TypeError als Pflaster). `on_black(s) + after_cm(6)` liest sich wie „und beides", meint aber „schwarz, *dann* 6 cm". Semantisch überraschend.

### 3.6 Altlast im Public-API 🟡
`WaitForLightLegacy` / `wait_for_light_legacy` neben der neuen Variante exportiert. Überlappende Pfad-APIs `goto`/`goto_relative`/`spline`/`smooth_path`/`optimize`/`drive_arc_segment`. Das README nennt sogar `drive_strafe`/`turn_right`, geliefert werden `strafe_left/right` — Inkonsistenz schon im Schaufenster.

---

## 4. Mission-Ergonomie: statischer Baum & Nesting

Der Happy-Path (`seq/parallel/background/.until`) liest sich gut. Alles *drumherum* — Resilienz, Verzweigung, Nebenläufigkeit, Heading — hat **keine flachen Primitiven**, also weicht man auf Wrapper-Verschachtelung, Python-Closures und Magic-Strings aus.

### 4.1 Fallback/Retry = Pyramid of Doom 🔴
Es gibt **keinen flachen Retry-/„try in Reihenfolge"-Kombinator**. cube-bot `m080` staffelt 6 Ebenen:
```python
timeout_or(step=_follow()…, fallback=
  optimize([ timeout_or(step=drive_backward()…, fallback=
    seq([ timeout_or(step=_follow()…, fallback=
      optimize([ timeout(step=drive_backward()…) ]) ),
      strafe_left(…) ]) ) ]) )
```
Jede Absicherungsstufe kostet `timeout_or → optimize([…]) → seq([…])` = drei Indents. **Gebraucht:** `attempt(step).or_else(step)` / `first_of([a,b,c])` / `retry(step, times=, timeout=)`.

### 4.2 Statischer Baum → jede Laufzeit-Entscheidung wird zur `defer`-Closure 🔴
`Mission.sequence()` wird einmal vorab gebaut. Alles, was live State/Sensor lesen muss, wird in `defer(lambda robot: …)` gewickelt, das einen Step *zurückgibt*. drumbot `m020`: `after_collect`, `drum_pipe_heading_mark_one/two` — alle die Form `def _build(robot): …; return defer(_build)`. `drum_lineup_step.py` hat **9** `defer()`.

Und weil `defer` keinen Wert an den nächsten Step übergeben kann, fließt State über **globale Variablen**:
- cube-bot: `nonlocal cube_is_there` zwischen `run(check…)` und `defer(position…)`.
- drumbot: **modul-`global _was_first_heading_valid`** zwischen zwei `defer`-Steps; `_block_start_time` zwischen `BlockTimerStartStep` und `BlockTimerCheckStep`.

**Gebraucht:** First-Class-`when(measure, then=…, otherwise=…)` und ein Weg, einen gemessenen Wert typisiert an den Folge-Step zu reichen.

### 4.3 Weitere Reibungen, die sich summieren 🟠
- **Kein `noop()`:** „nichts tun" = `seq([])` oder `run(lambda robot: None)` (in `m020` dreimal). Ein leeres `seq` als Kontrollfluss ist ein Geruch.
- **Sentinel-Magie statt „aus":** `wall_align_forward(settle_duration=9999, max_duration=9999, grace_period=9998, accel_threshold=99)` — riesige Zahlen = „nie abbrechen", weil die API kein `None`/`disabled` hat. `grace_period=9998` kodiert sogar Reihenfolge.
- **Stringly-typed Nebenläufigkeit:** `background(…, name="x")` + `wait_for_background("x")`. In cube-bot `m090` wartet der Code auf `"clap"`, dessen Background auskommentiert ist → stiller Hänger/No-Op. Tippfehler = undetektierbarer Deadlock. **Gebraucht:** typisierte Handles (`h = background(...)` → `wait(h)`).
- **`heading=0`-Boilerplate auf jeder Bewegungszeile** + verstreutes manuelles `mark_heading_reference()`. **Gebraucht:** ein Block-/Mission-Heading-Kontext.
- **`step=` mal benannt, mal positional** (`timeout_or(step=…)` vs. `timeout(step, seconds=)` vs. `background(arm.move_angles(…))`).
- **`optimize([...]).cut_corners(7, cut_until=True)`** hängt den Pfad-Optimizer per Chain-Methode an eine Step-Liste, mit Magic-Zahlen — und ist in beiden Bots mehrfach **auskommentiert** („TODO: enable teh optimize"). Eine API, die man aus Unsicherheit auskommentiert, ist zu unvorhersehbar.

**Fair:** drumbots Service/Step-Split macht die *Mission* deutlich flacher als cube-bot — der Nesting-Schmerz ist real, aber gute Architektur entkommt ihm größtenteils. Der Rest sind die `defer`/`global`-Conditionals.

---

## 5. Custom-Steps erweitern: Zeremonie & zwei Modelle

### 5.1 Jeder Step wird ZWEIMAL geschrieben 🔴
Erweiterungspunkt zwingt zu Klasse **+** Factory. `go_to_slot_step.py` ist repräsentativ für alle 14 cube-/29 drumbot-Steps:
```python
@dsl(hidden=True)
class GoToSlotStep(Step):
    def __init__(self, slot, *, stall_retries=None, tolerate_stall=False): ...
    async def _execute_step(self, robot): ...

@dsl()
def go_to_slot(slot, *, stall_retries=None, tolerate_stall=False) -> GoToSlotStep:
    return GoToSlotStep(slot=slot, stall_retries=stall_retries, tolerate_stall=tolerate_stall)
```
Die Factory wiederholt **jeden Parameter dreimal** (Signatur, Weitergabe, Docstring) und tut sonst nichts. In drumbot: **~28 Step-Subklassen → ~36 `@dsl`-Factory-Sites**, alle von Hand.

**Korrektur/Nuance (wichtig):** Es ist *nicht* so, dass das Framework keine Abkürzung bietet — `@dsl_step` (`annotation.py:36-84`) generiert Builder **und** `snake_case()`-Factory automatisch aus der `__init__`-Introspektion und setzt `hidden=True`. Das Problem ist: **`@dsl_step` wird in beiden Bots 0× verwendet.** Die Teams zahlen die Boilerplate freiwillig — mutmaßlich um den Stubgen-Build-Schritt zu vermeiden, den `@dsl_step` mitbringt. **Gebraucht:** `@dsl_step` friktionsfrei zum *Default*-Pfad machen (kein separater Stubgen-Build nötig), damit der ergonomische Weg auch der bequemste ist. Der Fix ist Adoption/Ergonomie, nicht ein fehlendes Feature.

### 5.2 Zwei Authoring-Modelle, keine Anleitung 🟠
Basis-`Step` bietet `async _execute_step(self, robot)` (coroutine, gut). Die Lib-*Motion*-Steps nutzen `on_start`/`on_update`/`to_simulation_step` (Tick). Es ist nirgends dokumentiert, wann man welches nimmt.

### 5.3 Kein Custom-Step ist simulierbar 🟠
`grep` über beide Bots: **kein einziger** `to_simulation_step`. Der Basis-`Step` bietet nur `_execute_step` + `lower_to_segments` (Optimizer-IR). Die 29 drumbot-Steps sind im Simulator **opake Barrieren**. Genau deshalb baut cube-bot handverdrahtete `RACCOON_SIM=1`-Fallbacks in eigene Steps — der Sim-Hook ist so umständlich, dass ihn niemand bedient.

### 5.4 Keine „Motion + Sensor"-Komposition 🟠
Um „drehen *und* dabei sampeln" zu bauen, konstruiert `scan_sweep_step` von Hand `TurnMotion(robot.drive, robot.odometry, robot.motion_pid_config, cfg)`, ruft `.start()` und pumpt `.update(dt)`/`.is_finished()` im `on_update`. `turn_to_peak_step` dupliziert denselben `_make_turn`-Helper. Das Framework hat keinen Weg, einen Sampler an eine bestehende Motion zu hängen → der Motion-Treiber wird pro Step re-implementiert.

---

## 6. Rand-Subsysteme sind nicht robust

Der stärkste Befund des ganzen Reviews: **das fortgeschrittenste Team musste das Framework zweimal monkeypatchen.**

### 6.1 `transport_wire_patch.py` — Framework-Codegen spricht nicht mit sich selbst 🔴
154 Zeilen, die `encode`/`decode` von `cam_blob_t`/`cam_frame_t`/`cam_detections_t` ersetzen. Grund (Originaldoc): die lcm-gen'ten Python-Klassen schreiben 8-Byte-Fingerprint + `[len+1][utf8][\0]`, aber der Dart-Decoder (botui) und der C++-Codec erwarten *kein* Fingerprint + `[len][utf8]` → **`RangeError` bei jedem Frame mit Detections**. Ein **gebrochener Cross-Language-Wire-Contract** im eigenen Codegen, den *jeder* Python-Prozess selbst reparieren muss. Bonus: der Stub sagt `area: int32`, das Wire-Format ist `float`.

### 6.2 `heading_frame.py` — Korrektheits-Bug im Kern-Primitiv 🔴
Monkeypatch von `heading_reference._world_heading`, weil `turn_to_heading` das Turn-Delta im **Localization-Frame** rechnete, während die C++-`TurnMotion` auf **Odometry** regelt → Endheading um den loc/odom-Offset daneben. Kommentar: *„Fix in raccoon-lib bereits gemacht; dieser Monkeypatch zieht ihn vor dem nächsten Lib-Deploy."* → Das **Wheel-Pinning/Deploy-Cadence** ist so träge, dass Teams lieber Interna patchen als auf den Fix zu warten. Das ist ein Distributions-DX-Problem, nicht nur ein Bug.

### 6.3 Motor-API failt still auf unkalibrierten Mechanik-Motoren 🔴
`DRUM_KALIBRIERUNG_FIX.md`: der generierte `drum_motor` bekam `MotorCalibration(ticks_to_rad=1.0, vel_lpf_alpha=1.0)` ohne `bemf_offset`/`static_friction_pct`. Folge: die Velocity-PID kommandiert nur Bruchteil-PWM, überwindet die Haftreibung nicht — **`set_velocity(1700)` erzeugt null Bewegung ohne Fehler**. Kostete eine dokumentierte Debugging-Session; Fix war der Wechsel auf Open-Loop `set_speed`. Eine geschlossene Regelung, die auf Nicht-Antriebsmotoren still no-opt, muss **laut fehlschlagen**.

### 6.4 Kein Stop-Semantik-Vertrag für Velocity-Kommandos 🟠
`terminate_leftover_velocity.py` existiert nur, weil „the wombat firmware holds that velocity-PID target for the whole collection (**the same persistence behind the wall_align leftover-velocity bug**) … Without this the robot keeps driving forward". Ein Velocity-Kommando ohne definierte Stop-Semantik zwingt defensive `_stop_drive`-Steps.

---

## 7. Vision, Hardware, Config

- **`CamSensor` unbrauchbar für den Wettbewerb** 🔴 — der native `CamSensor` (Polling `is_detected/get_blob_x`, keine Lifecycle) überlebt keinen User-Programm-Neustart. drumbot baute stattdessen einen **kompletten Out-of-Process-Vision-Stack**: `daemons/vision.py` (736 Z.) + `hardware/usb_camera.py` (590 Z.) + 411-Z.-Pub/Sub-Client. Das Framework braucht eine **Daemon-Service-Infrastruktur** (Lifecycle, Reconnect, Request/Response-RPC, Kalibrier-Persistenz) — drumbot hat sie per `uuid4`+`threading.Event`-Korrelation von Hand nachgebaut.
- **Fremd-Sensoren = Abstraktion von Null bauen** 🟠 — für einen Range-Finder wrappt `hardware/range_finder.py` den rohen `ETSensor` und baut Median-Filter, benannte Kalibrier-Profile und Enter/Exit-Thresholds selbst. Nichts davon bietet das Framework für Analog-Sensoren.
- **`self.robot.defs.<name>` ist ungetypt** 🟠 — Services greifen via `robot.defs.drum_motor` zu; ein Rename im Projekt-YAML bricht das zur Laufzeit per `AttributeError`. Ein `robot.require_motor("drum")` mit lautem Fehler wäre sicherer.
- **Config-per-`getenv`-Wildwuchs** 🟡 — `vision.py` hat 9 `DRUMBOT_*`-Env-Knöpfe mit Inline-Parsing, weil das Projekt-Config-System keinen projekteigenen Daemon erreicht. cube-bot: `RACCOON_SIM`, `DRUMBOT_NO_POSITION_HOLD` mitten im Mission-Code.

---

## 8. Doku & Distribution

Die Content-Seiten (`raccoon-docs/content/**`) wurden am 2026-06-18 überarbeitet und sind weitgehend korrekt — die Restprobleme leben in **generierten Artefakten**:

- **KI-Onboarding-Fläche ist rott** 🔴 — `public/llms-full.txt`/`llms.txt` sind vom 2026-05-28 (vor dem Rewrite) und lehren jede entfernte/crashende API: `from libstp import *` (**78 Treffer**), `class M00SetupMission(Mission)`, `FusedOdometry(...)`, `turn_to_heading(0)`, `follow_line(...)` als Primär-Beispiel. Das sind die Dateien, die Cursor/Claude einlesen — also der *primäre* Onboarding-Kanal für Schüler mit KI. Der Generator lief nach dem Content-Merge nicht. **Fix:** `llms-*.txt` in den CI-Regen ziehen (oder `.gitignore`).
- **Zwei konkurrierende „Quick Start"-Seiten**, beide `weight: 1`, verschiedene Autoren, kein Querverweis.
- **Doku widerspricht sich** — `lineup.md:148` „there is no public `forward_speed` parameter" vs. `03-missions.md:95` `forward_speed=1.0` (real existiert der Param).
- **Tote „libstp"-Marke leakt zur Laufzeit** in Log-Dateinamen (`libstp-2026-07-03_*.log`).

Interne Audit-Reports existieren bereits (`DOCS_AUDIT_REPORT_2026-06-18.md`: 66 P0-Crash-Issues) — diese Punkte sind dort dokumentiert, aber der `llms-*.txt`-Regen fehlt.

---

## 9. Priorisierte Cleanup-Roadmap

Nach Impact/Aufwand. Die ersten fünf haben den größten DX-Hebel.

1. **Velocity-Stop-Semantik & lauter Fehler bei unkalibrierten Motoren fixen** (§6.3, §6.4). Sicherheits-/Korrektheitsrelevant, kostete beide Teams Debug-Zeit. Eine geschlossene Regelung darf nie still no-opt.
2. **LCM-Codegen über Python/Dart/C++ vereinheitlichen** (§6.1) und den `turn_to_heading` loc/odom-Fix deployen (§6.2) — damit die zwei Monkeypatches sterben. Danach: schnellerer Lib-Release-Kanal, damit Teams nicht mehr patchen müssen.
2b. **Servo-/Aktuator-Feedback-Familie** (§2b) — die häufigsten User-Idiome sind hier Workarounds. Reihenfolge: (a) `blocking=False` an Servo-Steps *(schon geplant)* killt das `background(einzel-servo)`-Idiom; (b) ein „Aktuator wirklich am Ziel"-Signal statt blindem `wait_for_seconds` (Positions-Feedback oder kalibrierte, last-bewusste Move-Zeit); (c) Pro-Servo-Torque/Compliance-Flag, damit `fully_disable_servos()` nicht mehr der einzige Weg ist, „nicht mehr drücken" zu sagen; (d) ein `wait_until_settled()`/jerk-limitierter Stop für die „so the bot is fully still"-Sleeps.
3. **Flache Kontrollfluss-Primitiven ergänzen** (§4.1, §4.2): `first_of([...])`/`attempt().or_else()`/`retry(...)`, `when(measure, then=, otherwise=)`, ein echtes `noop()`, und typisierte Background-Handles statt Magic-Strings. Löst das Nesting *und* die `global`-State-Fädelung auf einen Schlag.
4. **API-Konsistenz statt -Reduktion** (§3.1) — die expliziten Richtungsnamen **behalten** (die Bot-Autoren wollen sie ausdrücklich). Stattdessen: (a) *eine* Oberflächen-Philosophie durchziehen, d. h. die verbleibenden signed-param-Ecken (`directional_follow_line(heading_speed=…)`, `drive_angle(-140)`) an den expliziten Hausstil angleichen; (b) alle Richtungsvarianten aus *einer* Impl-Basis pro Familie generieren, damit ein Fix einen Ort trifft. Kein Kollaps der Namensfläche.
5. **Eine API pro Fähigkeit** (§3.2) — Line-Follow: Builder *oder* Funktion, nicht beides; Doku an die reale Wahl (Builder) angleichen, Funktions-Form zur Migrations-Notiz degradieren.
6. **Namespace kuratieren** (§3.3): schlankes Top-Level, gruppierte Submodule (`raccoon.motion`, `raccoon.steps`, `raccoon.ui`), `__all__` deduplizieren, `*Legacy` entfernen, Star-Import als Konvention beenden.
7. **Custom-Step-Boilerplate eliminieren** (§5.1): DSL-Factory aus der Klasse ableiten, statt sie von Hand zu verlangen.
8. **Ein dokumentiertes Step-Modell + funktionierender Sim-Hook für Custom-Steps** (§5.2, §5.3). Ohne Simulierbarkeit ist der Simulator für ernsthafte Bots wertlos.
9. **`sensor + motion`-Komposition** (§5.4): einen Sampler/Callback an eine laufende Motion hängen können, statt `TurnMotion` pro Step neu zu verdrahten.
10. **Daemon-Service-Infrastruktur** (§7): Lifecycle, Reconnect, Request/Response-RPC, Kalibrier-Persistenz — damit Vision/Custom-Daemons nicht from scratch gebaut werden.
11. **Block-/Mission-Heading-Kontext** (§4.3) statt `heading=0` pro Zeile; `None`/`disabled` statt Sentinel-`9999`.
12. **Doku-Distribution reparieren** (§8): `llms-*.txt` in CI regenerieren, doppelte Quick-Starts mergen, „libstp"-Leaks tilgen.

### Ergänzungen aus dem 5-Agent-Sweep (§2c)

Neue High-Impact-Punkte, die sich in die Roadmap einreihen:

- **A. Zwei fehlende Kontrollfluss-Konstrukte schließen** (§2c, größter Hebel nach den Safety-Fixes): (1) ein **Step-zu-Step-Wertekanal** (ein Step liefert einen Wert, der nächste liest ihn typisiert) — eliminiert 16 `defer` + 7 `global`-Kanäle auf einen Schlag; (2) ein **Supervised-Concurrency/Monitored-Loop-Step** (Schleife mit nebenläufigen Watchdogs + Zustandsübergängen) — ersetzt die 450-Z.-asyncio-Steps und die defer-State-Machines. Gehört mit Roadmap #3 zusammen.
- **B. `RobotService`-Lifecycle** — `on_start`/`on_stop`/`on_shutdown`, eine Managed-Background-Task-Utility und Sub-Komponenten-Komposition. Killt `camera_lifecycle_step`, das Hand-Teardown und den Mixin-Workaround.
- **C. YAML↔Codegen-Round-Trip / kein stiller Datenverlust** — getunte Werte dürfen bei `raccoon run` nicht überschrieben werden (Drift-Check oder Rück-Propagation), plus ein **typisierter `defs`-Accessor** (der `et_range_finder`-Fehler wäre zur Build-Zeit aufgefallen).
- **D. Fehlende Framework-Primitiven als Bibliothek** — statt Consumer-Eigenbauten: Flank/Hysterese-Analog-Condition, Heading-„weggedrückt"-Condition, gefilterter/Median-Analog-Read, Progress-Stall+Retry-mit-Backoff, Motion+Sampler-Kombinator, „bei Distanz X → Event", `overlap()`-Kombinator, Kalibrier-Quality-Contract, Single-Sensor-Centering.
- **E. `@dsl_step` zum friktionsfreien Default machen** (ersetzt Roadmap #7 präziser) — der Auto-Generator existiert, wird aber 0× genutzt; den Stubgen-Build-Schritt eliminieren, damit der bequeme Weg der Standardweg ist.
- **F. Step-Level-Sim-Contract** (`step/sim/` ist leer) — damit Simulation nicht per Fake-Service-Duplikat läuft. Verschärft Roadmap #8.

---

### Schlusswort
RaccoonLib hat einen **echten Kern** (DI, `condition.py`, `MotionStep`, UI) — drumbot beweist, dass man darauf ein 10k-LOC-System bauen *kann*. Aber je tiefer ein Team geht, desto klarer werden die Ränder: **zwei Monkeypatches**, ein **komplett selbstgebauter Vision-Stack**, ein **still failender Motor**, **nicht-simulierbare Custom-Steps**, **kein Step-zu-Step-Kanal** (→ `global`), ein **statischer Mission-Baum**, der jede Laufzeit-Entscheidung besteuert — und darüber eine **API-Oberfläche, die sich selbst dreifach dupliziert**. Das damning­ste Einzelartefakt ist `transport_wire_patch.py`: ein fortgeschrittenes Team musste die Serialisierung reverse-engineeren, damit das Framework mit sich selbst reden kann.

Der größte Hebel ist nicht mehr Features, sondern **Konsolidierung & Robustheit der Ränder**.
