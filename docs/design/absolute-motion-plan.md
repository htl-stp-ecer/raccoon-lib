# Absolute Motion + Localization — Designvorschlag (Strawman)

**Status:** Diskussion · **Verwandt:** [Issue #13](https://github.com/htl-stp-ecer/raccoon-lib/issues/13)

## TL;DR

Heute fährt der Roboter eine Mission als Folge **relativer**, **open-loop** Bewegungen. Drift wird per `WorldCorrectionMiddleware` korrigiert — aber nur innerhalb von `flow()`/`smooth_path` Blöcken. Sobald die Mission rausspringt, ist der Welt-Bezug wieder weg.

Vorschlag: drei Schichten, sauber getrennt.

| Schicht          | Verantwortung                                                  | Sprache | Heute                                  |
|------------------|----------------------------------------------------------------|---------|----------------------------------------|
| **DSL/Plan**     | Mission ausdrücken, Plan-IR aus relativer DSL desugarn          | Python  | Implizit über relative DSL-Aufrufe      |
| **Optimizer**    | Redundante Knoten faltenlöschen, co-lineare Goto's mergen, Heading aus nächstem Target ableiten | C++ (mit Py-Bindings) | – |
| **Localization** | Wo *bin* ich wirklich — kontinuierlich, nie zurückgesetzt       | **C++**, Hintergrund-Thread | Odometrie wird bei jedem Motion-Start zurückgesetzt |
| **Execution**    | Regle auf die nächste Welt-Pose, nicht auf "fahre 70 cm"       | C++ (Motion-Klassen) + Py (Executor) | Jeder Step macht sein eigenes Ding |

**Leitprinzip:** alles auf dem Hot-Path läuft in C++. Python nur für Plan-Authoring und Executor-Schleife (die ohnehin nur `await asyncio.sleep`-driven ist). Pi3 hat keine Reserven zu verschenken.

Plus: explizite **Resync-Steps** als Observation-Spikes für einen Particle Filter, der Sensoren am Roboter mit der bekannten Map-Geometrie kreuzt.

---

## Warum das eigentliche Problem nicht der Reset ist

`resetVelocityControllers()` in `start()` ist nur ein Symptom. Die wirkliche Wurzel:

1. **`odometry.reset()` läuft bei jedem Cold-Start** → eine konsistente Welt-Pose über mehrere Motions hinweg gibt es nicht.
2. **Steps sind relativ** → `drive_forward(70)` heißt "ab hier 70 cm geradeaus", nicht "endpunkt = (x=2.30, y=1.10, θ=90°)". Wenn der Turn davor 7° überschießt, fährt der Drive die 70 cm in die falsche Richtung.
3. **Die Korrektur ist ein Aufsatz** → `WorldPoseTracker` rekonstruiert den Welt-Frame über Akkumulator-Hacks, weil der eigentliche Träger (Odometrie) zwischen Motions stirbt.

Ein Pflaster auf der Arterie, wie du sagst. Wir wollen die Arterie zunähen.

---

## Architektur

### 1. Localization Service (`robot.localization`)

Ein einziger, persistenter Welt-Pose-Schätzer auf dem Roboter. Lebt von Mission-Start bis Mission-Ende. Wird **nie** von einer Motion zurückgesetzt. **Komplett in C++**, läuft in einem eigenen Hintergrund-Thread, sampelt Sensoren und propagiert die Pose auch wenn keine Motion aktiv ist (Stichwort: jemand schubst den Roboter).

```cpp
class Localization {
public:
    Pose getPose() const;                 // thread-safe snapshot
    Covariance getCovariance() const;
    void observe(const Observation&);     // Resync-Punkt — von Resync-Steps gerufen
    // tickt intern @ ≥100 Hz, eigener std::thread
};
```

Python-Bindings:

```python
robot.localization.get_pose() -> Pose
robot.localization.observe(observation)
```

**Sensor-Realität (Pi3 + Wombat):**
- **Keine Wheel-Encoder**, aber STM32 liefert bereits eine fertige Odometrie (`Stm32Odometry`): Full-Speed Back-EMF aller Motoren + IMU-Slip-Correction, integrierte Pose und ein **drift-armes Heading** (~6 Monate Tuning, das ist der mit Abstand beste Sensor-Kanal). Pro-Motor-kalibrierte Konstanten kommen demnächst.
- **Accelerometer-Rohwert** ist sehr verrauscht — als Position-Quelle nicht direkt nutzbar; bestenfalls als Stoß-Detektor (Spikes signalisieren externe Krafteinwirkung).
- **IR-Reflektanz-Sensoren** sind unsere einzige *positionsbezogene* Observation-Quelle.

Damit ist die Localization-Struktur:

- **Prediction-Step:** **kein Re-Implement der Kinematik**. `Stm32Odometry` liefert kumulierte Pose; Localization speichert das letzte gesehene Sample, bildet `delta = current - last` pro Tick und propagiert jedes Partikel mit diesem Delta + Prozess-Rauschen. Die ganze Schwerarbeit (Back-EMF, Slip-Correction, IMU-Fusion) sitzt schon im STM32; doppelt machen wäre Verschwendung. Keine STM32-API-Änderung nötig.
- **Disturbance-Modell:** Accel-Spike ohne Motor-Kommando → Prozess-Rauschen für N Ticks hochfahren (Filter spreizt sich, lässt sich vom nächsten IR-Read wieder einfangen).
- **Observation-Step:** IR-Reflektanz pro Partikel. Sensor an Body-Position aus `RobotGeometry`, Welt-Position aus Partikel-Pose berechnen, in der **Vector-Map** den erwarteten Region-Wert (Linie / nicht-Linie / Wall) nachschlagen, mit gemessenem IR-Wert vergleichen, Partikel gewichten.
- **Resampling:** Low-Variance. `N_eff` als Health-Metrik in Telemetrie.
- **Partikel-Anzahl:** Ziel 100–200 auf Pi3. Mit C++ und kompaktem Memory-Layout (SoA, Floats) bei 100 Hz Update realistisch.

#### Vorbedingung: Vector-Map + RobotGeometry nach C++ portieren

Das ist die heute schmerzhafteste Stelle. Der Particle-Filter macht pro Tick `N_partikel × N_sensoren` Map-Lookups — das **kann nicht in Python passieren**.

Heutiger Stand:
- `modules/libstp-robot/python/raccoon/robot/table_map.py` (338 Zeilen Python) — `TableMap`, `MapSegment`, `_point_to_segment_distance`, `is_on_line`, `is_on_wall`, `sensor_field_position`, `sensor_is_on_line`, …
- `modules/libstp-sim/include/libstp/sim/WorldMap.hpp` (C++) — schon teilweise da, aber **nur in der Sim** verwendet, parst dasselbe ftmap-Format. **Doppelimplementierung.**
- `modules/libstp-robot/python/raccoon/robot/geometry.py` (253 Zeilen Python) — `RobotGeometry`, `SensorPosition`, `WheelPosition`, Distanzberechnungen.
- `modules/libstp-robot/python/raccoon/robot/map_corrected_odometry.py` (123+ Zeilen Python) — auch hot-path-relevant.

**Vorschlag:** ein neues `libstp-map` C++ Modul, das die heutige `WorldMap`-Implementierung aus libstp-sim übernimmt und um die fehlenden `TableMap`-Queries erweitert (`is_on_line`/`is_on_wall`, segment-Nearest-Distance, sensor-projected reads). `libstp-sim` und der Roboter ziehen beide von dort. **Python-Klassen `TableMap` und `RobotGeometry` werden gelöscht** — Bindings exposen die C++-Typen direkt. Damit existiert das Map-Format nur noch einmal, mit einer kanonischen Performance-getuneten Implementierung, und der Particle-Filter kann ohne FFI-Overhead pro Lookup darauf zugreifen.

Performance-Kandidaten innerhalb des C++-Ports:
- **Räumlicher Index** (z.B. statisches Grid oder R-Tree) über die Liniensegmente — einmalig beim Laden gebaut, danach O(log n) oder O(1) pro Lookup.
- **Batch-API** `is_on_line(span<Point>) -> span<bool>` für vektorisierte Particle-Filter-Updates.
- **Pre-Computed Reflektanz-Grid** als optionaler Cache (1 cm Auflösung): wenn Map sich nicht ändert, einmal raster, danach O(1) pro Sample.

**Warum Particle Filter und nicht EKF:**
- IR-Reflektanz ist diskret/multi-modal (Partikel auf weißem vs schwarzem Bereich): nicht-gauss.
- Multi-Modal nach Schub/Slip — ein EKF würde fälschlich konvergieren.
- Mit so schwachem Motion-Modell (Back-EMF statt Encoder) ist Hypothesen-Vielfalt überlebenswichtig bis ein Resync sie kollabiert.

**Always-on:** der Localization-Thread läuft unabhängig vom Motion-State. Wenn der Roboter steht, läuft Prediction trotzdem (Velocity = 0 + Rauschen) und Observations werden weiter integriert. Schubst jemand: Accel-Spike → erhöhtes Prozess-Rauschen → Filter spreizt → nächster IR-Read korrigiert.

### 2. Plan IR

Ein Plan ist eine Liste von Knoten. Jeder Knoten ist eines von:

```python
@dataclass
class Goto:                # bewege dich zu absoluter Welt-Pose
    target: Pose
    via: Literal["forward", "lateral", "arc", "auto"] = "auto"
    speed_scale: float = 1.0
    heading_lock: float | None = None   # falls nur Position, nicht Orientierung relevant

@dataclass
class TurnTo:
    target_heading: float  # absolut, im Welt-Frame

@dataclass
class Resync:              # explizite Welt-Korrektur
    method: Literal["wall_align", "find_line", "marker", "io_button"]
    expected_pose: Pose | None = None    # Vorwissen: ich erwarte mich hier zu sein
    snap_axes: tuple[bool, bool, bool] = (True, True, True)  # x, y, θ

@dataclass
class Action:              # nicht-Bewegung: Servo, Wait, beliebige Substep
    step: Step
    blocking: bool = True   # blockt Plan oder läuft parallel zur nächsten Bewegung?
```

Plan = `list[PlanNode]`. Wird vom Compiler in eine Segment-Liste plus Side-Action-Schedule überführt — wie heute, aber mit absoluten Targets statt relativen Deltas.

### 3. Executor — "regle auf das absolute Target"

Bei jedem Segment-Start:

1. Lies aktuelle Pose aus `robot.localization.get_pose()`.
2. Berechne Delta zum absoluten Segment-Target.
3. Konfiguriere Motion mit diesem Delta. **Dadurch absorbiert jede Bewegung automatisch jeden vorangegangenen Fehler** — ohne separaten Korrektur-Layer.
4. Während der Motion: Heading-PID regelt nicht mehr auf "Anfangs-Heading + relativer Offset", sondern auf das absolute Welt-Heading des Segments.
5. Bei Segment-Ende: nichts. Keine Korrektur-Berechnung, keine `advance_expected`. Der Welt-Frame existiert in `localization`, nicht im Tracker.

**Kein `odometry.reset()` zwischen Motions mehr.** Lokale Achsen leben in der Motion-Klasse selbst (Profile-Position startet bei 0), aber die Welt-Pose wird aus `localization` gelesen und nicht angefasst.

### 4. Resync-Steps

Explizite Punkte, an denen die Mission der Localization eine **stark gewichtete Observation** zuschiebt — *nicht* eine harte Pose-Setzung. Begründung: die Roboter sind mechanisch nie genau genug, um eine Wand-Berührung als Wahrheit zu deklarieren. Heading ist die einzige Größe, die hart genug ist, dass ein Snap vertretbar wäre — aber weiches Snappen funktioniert da auch, mit kleiner Varianz.

Soft-Snap-Modell: jeder Resync-Step erzeugt eine `Observation` mit explizitem Mess-Rauschen pro Achse. Der Filter integriert sie wie jede andere Messung, nur mit sehr kleinem σ entlang der gesnappten Achse(n). Wenn die Realität widerspricht (anderer Sensor sagt deutlich was anderes), wird der Snap aufgeweicht statt fälschlich übernommen.

```python
# σ_x klein (Wand gibt x), σ_y groß (lateral nicht gemessen), σ_θ klein (Wand-Tangente gibt θ)
align_to_wall(side="right", expected_x=30)

# σ_y klein (Linie gibt y), σ_x groß, σ_θ klein
find_line(direction="forward", expected_y=85, max_distance=10)

# Pflicht-Start, alle σ klein (Roboter wurde positioniert hingestellt)
resync_at_start_pose(x=10, y=10, theta_deg=0)
```

```python
# Wand auf der rechten Seite ankoppeln → x und θ werden aus dem Wand-Kontakt
# rekonstruiert (y bleibt vom Filter)
align_to_wall(side="right", expected_x=0.30)

# Schwarze Linie suchen → y wird aus Linien-Position rekonstruiert
find_line(direction="forward", expected_y=0.85, max_distance=0.10)

# Startposition vom Lichtsensor / Button-Press
resync_at_start_pose(Pose(x=0.10, y=0.10, theta=0.0))
```

Jeder dieser Steps ruft am Ende `robot.localization.observe(Observation(...))` mit einer hochgewichteten Messung auf. Der Filter snappt dann auf die neue Pose — Drift weg.

---

## DSL für die Mission

Der heutige relative DSL bleibt für simple Sachen erhalten. Für absolute Sachen kommen drei neue Builder-Familien.

### Niedrige Ebene — explizite Posen

API-Konvention: **Python-Aufrufe sind kwargs, keine verschachtelten Konstrukte.** Innen wandelt das Binding zu C++-Structs (`Pose`, `Goto`, `TurnTo`) für die IR.

```python
from raccoon.step.motion import goto, turn_to, resync_at

mission = seq(
    resync_at(x=10, y=10, theta_deg=0),
    goto(x=80, y=10, via="forward"),         # absolute cm/grad
    turn_to(theta_deg=90),
    goto(x=80, y=100),
    align_to_wall(side="front", expected_y=105),
    goto(x=20, y=100),
)
```

Pose-Eingaben durchgängig in **cm und Grad** auf der Python-Seite (User-friendly, konsistent mit der bestehenden DSL). Konvertierung nach Meter/rad passiert beim Eintritt in die C++-IR.

### Mittlere Ebene — Pfad mit Side-Actions

```python
mission = path(
    start=Pose(x=0.10, y=0.10, theta=0),
    nodes=[
        goto(x=0.80, y=0.10),
        action(servo_open("claw"), blocking=False),   # läuft parallel zur Fahrt davor
        turn_to(deg=90),
        goto(x=0.80, y=1.00, speed=0.5),
        align_to_wall("front"),
        goto(x=0.20, y=1.00),
    ],
)
```

`path()` kompiliert in dieselbe IR wie `flow()` heute, nur mit absoluten Posen. Smooth-Übergänge / Corner-Cuts / Spline-Passes funktionieren weiter — sie operieren auf der absoluten IR.

### Hohe Ebene — relative DSL desugart auf absolut (das ist der Hauptpfad)

In der Praxis schreibt **niemand** `goto(Pose(0.80, 0.10))` — alle nutzen `drive_forward(70)`. Heißt: das Desugaring ist nicht Bonus, sondern **der** Pfad. Es muss bombenfest sein, und genau hier kommen Optimizer-Passes ins Spiel.

#### Schritt 1 — dummes Desugaring (1:1)

Reine mechanische Übersetzung, keine Klugheit. Jeder relative Knoten bekommt ein absolutes Target ausgerechnet, indem er auf der "intended pose chain" addiert wird.

```python
# Eingabe (relative DSL)
seq(drive_forward(70), turn(90), drive_forward(50))

# Schritt 1: rohes Desugaring
[Goto(target=Pose(0.70, 0.00, 0)),
 TurnTo(target_heading=π/2),
 Goto(target=Pose(0.70, 0.50, π/2))]
```

Du hast völlig Recht: das `TurnTo` dazwischen ist redundant — der zweite `Goto` *impliziert* das Heading bereits. Die Trennung "dumm desugarn → klug optimieren" ist genau Sinn der Sache: das Desugaring darf naiv und audithierbar sein, der Optimizer arbeitet auf einer wohldefinierten IR.

#### Schritt 2 — Optimizer-Passes

Plug-bare Passes, jede mit klarer Invariante. Reihenfolge zählt.

**Der Optimizer ist absichtlich konservativ.** Lieber zu wenig optimieren als eine Mission stillschweigend in der Bedeutung verändern. Aggressive Passes (Action-Lifting, Heading-Inference) bleiben standardmäßig **aus** und werden pro Pass per Flag freigeschaltet, sobald wir Vertrauen aufgebaut haben.

| Pass                     | Was                                                                          | Default |
|--------------------------|------------------------------------------------------------------------------|---------|
| `validate_reachable`     | Posen außerhalb der Map werfen Compile-Fehler                                | **on**  |
| `fold_implicit_turns`    | `TurnTo(θ)` vor `Goto(target_heading=θ)` löschen — **nur** wenn dazwischen keine `Action` und kein `parallel`-Block lief | **on**  |
| `merge_collinear_gotos`  | Zwei `Goto`s mit selber Richtung → ein `Goto` zum letzten Target             | off     |
| `infer_heading`          | `Goto(x, y)` ohne Heading bekommt Heading vom Vektor zum Target              | off     |
| `lift_actions`           | `Action(blocking=False)` an die früheste Stelle ziehen, an der sie passt     | off     |
| `insert_resync_hints`    | An langen Geraden vor Wand: optional `align_to_wall` einfügen               | off     |

Jeder Pass ist eine Funktion `IR -> IR`. Tests prüfen Idempotenz und semantische Äquivalenz (gleiche End-Pose nach Ausführung gegen Referenz-Trajektorie).

```python
# Schritt 2: nach fold_implicit_turns + merge_collinear_gotos
[Goto(target=Pose(0.70, 0.00, 0)),
 Goto(target=Pose(0.70, 0.50, π/2))]
```

#### Wo das läuft

- **Desugaring + Optimizer in C++** mit Python-Bindings. Die IR ist ein POD-Struct, Passes sind reine Funktionen — perfekt für C++. Python ruft `compile_plan(nodes) -> CompiledPlan` und kriegt eine ausführbare IR zurück. Vorteile: schneller, testbar in beiden Sprachen, gleiche Implementierung in Sim und auf dem Roboter.
- **Executor-Schleife darf Python bleiben** — sie ist `await asyncio.sleep`-driven mit `update_rate` Cadence, kein CPU-Hot-Path.

Mission-Author schreibt wie heute, kriegt Drift-Resistenz + Optimierung geschenkt.

---

## Was an heutigem Code überlebt

- **PathExecutor + Compiler:** überleben, IR wird auf absolute Targets umgestellt.
- **`smooth_path` / Splines / Corner-Cut:** überleben — operieren weiter auf der IR.
- **Motion-Klassen (LinearMotion, TurnMotion, ArcMotion):** überleben weitgehend; bekommen einen "absolute target" Konfig-Pfad. Profile-Berechnung bleibt lokal.
- **`startWarm()`:** bleibt, ist orthogonal — Velocity-Kontinuität an Segment-Übergängen ist von Welt-Pose-Tracking unabhängig.

## Was wegfällt

- `WorldCorrectionMiddleware` und `WorldPoseTracker` — komplett gelöscht.
- `odometry.reset()` aus den Motion-`start()`-Methoden raus. Reset gibt es nur noch über `localization.observe()`.
- `getAbsoluteHeading()` als separate API — die Welt-Pose ist jetzt erste Bürgerin, kein Workaround.
- Die "expected pose chain" Akkumulator-Hacks im Tracker.

---

## Migration in Phasen

1. **`libstp-map` C++-Modul.** `WorldMap` aus libstp-sim als Basis, `TableMap`-Queries aus Python portiert, räumlicher Index, Batch-API. Python-Bindings ersetzen `TableMap` 1:1. **Python-Datei wird gelöscht.** `RobotGeometry` mit raus. **`MapCorrectedOdometry` wird hart gedroppt** — keine Migration, keine Adapter, weg. Bestehende Tests laufen unverändert (oder werden gelöscht, wo sie nur die Korrekturschicht testen).
2. **`Localization` C++-Service** mit Pass-Through (kein Filter, nimmt `Stm32Odometry` direkt durch). Eigener Thread, `observe()`-API. Wird auf `robot.localization` exposed. Motions ignorieren ihn noch. Sofortiger Gewinn: stabile Welt-Pose über Motion-Grenzen hinweg. Da `Stm32Odometry` nur kumulierte Pose liefert, berechnet Localization das Delta selbst (`current - last_seen`) — kein STM32-Firmware-Change nötig.
3. **`Pose`-Type + IR mit absoluten Targets** (auch C++, mit Bindings). Compiler kann beides erzeugen, Executor kann beides ausführen. Snapshot-Tests gegen heutige Mission-Outputs.
4. **Motions bekommen `absolute_target`-Pfad — harter Umstieg.** Kein Adapter, kein Doppel-Pfad. Alle Motions werden auf das absolute Target umgezogen, `WorldCorrectionMiddleware` wird in derselben Phase ersatzlos entfernt. Drumbot-Toleranzen aus `374cd5b` werden hier zurückgedreht. Mehr Risiko in einer Phase, aber kein Code-Doppel mit Lifetime > 0.
5. **DSL desugart** auf absolute Plan-IR. Konservativer Optimizer (nur `validate_reachable` + `fold_implicit_turns`). Default flippt für neue Code-Pfade; alte DSL bleibt funktional.
6. **Particle Filter** ersetzt den Pass-Through in Localization. Resync-Steps (`align_to_wall`, `find_line`, `resync_at_start_pose`) werden funktional. Performance-Profil auf Pi3.
7. **Cleanup:** `WorldPoseTracker`-Reste, `odometry.reset()` aus Motion-Starts, alte relative-only Code-Pfade in Tests/Examples.

Jede Phase ist deploybar.

---

## Tradeoffs / Risiken (ehrlich)

- **Particle Filter auf Pi3 ist eng kalkuliert.** 100–200 Partikel × 100 Hz × IR-Sensoren × Vector-Map-Lookup. Mit C++ und gutem Memory-Layout machbar, aber nicht großzügig. Profiling vor Phase 5; falls zu langsam, fallen Optionen: weniger Partikel, niedrigere Update-Rate (50 Hz), oder Adaptive-KLD-Sampling.
- **Schwaches Motion-Modell.** Ohne Encoder ist die Prediction zwischen Resyncs deutlich schlechter als bei Robotern mit Wheel-Odometry. Der Filter *braucht* regelmäßige Observations — d.h. Missionen müssen häufig genug über Linien fahren oder Wand-Resyncs machen, sonst spreizt der Filter sich tot. Das ist eine Mission-Design-Disziplin, kein Code-Fix.
- **Heading-Drift ohne Magnetometer.** Über 2 min Match-Dauer i.d.R. ok, aber pathologische Manöver (viele schnelle Drehungen) können Gyro-Drift bis zu mehreren Grad akkumulieren. Wand-Align ist die einzige Heading-Korrektur — das muss fester Bestandteil des Mission-Design-Vokabulars sein.
- **Ohne Resync-Steps wird's nicht besser, nur anders.** Reine Drift bleibt; Filter degeneriert dann zur klassischen Dead-Reckoning. Das System wird nicht *schlechter*, aber der Gewinn kommt erst mit den Observations.
- **Vector-Map als Observation-Source.** Existiert bereits — gut. Was fehlt: dokumentierte Sensor-Body-Positionen (`(dx_s, dy_s)` je IR-Sensor) als Roboter-Konfiguration und ein performanter Map-Lookup für "an Welt-Punkt P, welcher Bereich/Helligkeit?". Räumlicher Index (z.B. R-Tree) lohnt wenn die Map nicht trivial ist.
- **Multi-Modal Filter am Start.** Mitigation: erste `resync_at_start_pose()` ist Pflicht (Roboter wird positioniert hingestellt → Single-Hypothesis-Start mit kleinem Rauschen).
- **Debugging wird anders.** "Warum ist der Roboter da?" wird zu "Filter-Pose zeigt X, Dead-Reckoning-Pose zeigt Y, IR-Reads sagen Z, N_eff war 12". Gute Telemetrie + Replay-Tooling ist Pflicht.
- **DSL-Migration ist ein Trojaner-Pferd.** Wenn die Desugaring/Optimizer-Logik subtil falsch ist (z.B. bei `parallel(...)`-Blöcken oder Kondition-basierten Segments ohne bekanntes Endpoint), brechen alte Missionen schweigend. Snapshot-Tests gegen heutige Mission-Recordings sind Pflicht in Phase 4. Jeder Optimizer-Pass kriegt einen Property-Test "Plan-Endpose unverändert nach Pass".

---

## Relative API darf neben Absolute leben

Wie der `flow()` Step, soll es fürs Absolute Mode eine option geben oder so - relativ soll weiterhin der Default sein; in manchen Situationen ist relativ immer noch besser- Smarter ist nicht immer gleich gut für gewisse Situationen.

## Offen / zu klären

konkretem Plan + Tests pro Schritt.
