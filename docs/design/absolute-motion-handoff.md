# Absolute-Motion Migration — Hand-off für Codex-Orchestrator

**Datum:** 2026-05-01
**Branch:** `absolute-motion`
**Repo:** `/media/tobias/TobiasSSD1/projects/Botball/raccoon/raccoon-lib`
**Vorgänger-Orchestrator:** Claude Opus 4.7 (Usage-Limit gehittet)

---

## Wer du bist und wie du arbeitest

Du bist der Orchestrator. Der User hat ausdrücklich gesagt: **„halte deinen Kontext sauber und mach nichts weiter als an weitere Agenten auszulagern!"** — du delegierst, du committest, du fragst beim User nach wenn ein Architektur-Entscheid ansteht. Du programmierst nicht selbst.

Default-Pattern pro Phase / pro Commit:

1. **Plan-Agent** (Sub-Agent vom Typ Plan oder eine codex-rescue-Instanz) — produziert einen Commit-für-Commit-Plan mit konkreten Datei-Operationen.
2. **Executor-Agent** (general-purpose oder codex-rescue) pro Commit — schreibt den Code, **führt KEINEN Build/Test aus** (User-Direktive: „dont run tests, they take too much time right now"), liefert `git status` + `git diff --stat`.
3. **Du committest selbst** mit Conventional-Commit-Stil und der pre-commit-Hook-Discipline (siehe unten).
4. Nach einer kompletten Phase ODER bei Verdacht auf Regression: **Build/Test-Validator-Agent** mit zeitlichem Budget (~10 min, fokussierter Test-Filter).

**Bei Architektur-Abweichungen vom Doc-Plan:** advisor() Aufruf vor Commit, dann User-Bestätigung, dann ausführen.

**Beim User-Sprachstil:** Antwort auf Deutsch (du-Form), knapp, konkret. Keine ASCII-Substitutionen für Umlaute.

---

## Stand bei Übergabe

### Doc-Plan
`docs/design/absolute-motion-plan.md` — Phasen 1-7 in der Migrations-Liste, jede mit Status-Note. Plan-Doc ist die einzige Wahrheit für Scope. Die „Implementation Notes"-Sektion am Ende dokumentiert Plan-Abweichungen.

### Erledigt: Phasen 1-4 ✅

```
4e5848d fix: phase-4 multi-segment + spline regressions
9199992 test: restore strict drumbot tolerances (phase-4 verification)
b7c9bd4 refactor: route python heading reads through localization
d0d5638 refactor(motion.path): drop AbsolutePathExecutor
75694fc refactor(motion.path): delete WorldCorrectionMiddleware + Correction
6d2c941 refactor(motion.path): supply absolute target_heading_rad per segment
1a229d0 refactor(libstp-motion): motions go absolute-only, no more reset()
c769a24 feat(motion.path): AbsolutePathExecutor — thin dispatcher
704622c feat(motion.path): compile_plan for absolute IR (1:1 desugaring)
0d18f51 feat(motion.path): absolute plan IR types + cm/deg factories
7175c91 feat(libstp-localization): reset resilience + cross-motion wiring
1f33998 feat(raccoon.localization): pybind11 bindings + robot.localization
110916c feat(libstp-localization): pass-through world-pose service
2104dbd refactor: drop MapCorrectedOdometry
527270a refactor(raccoon.map): drop Python TableMap, alias to WorldMap binding
dc6e2ef refactor(libstp-sim): retire WorldMap copy, alias to libstp::map
4aef856 feat(libstp-map): canonical ftmap module with sensor-projected queries
```

### Validierungs-Stand

- **Phase 1-3 Validation** (ein Validator-Agent vor Phase 4): Build clean, alle Phase-1-3-Tests grün (46/46), Sim-Tests grün soweit gelaufen. Zwei Caveats:
  - `MockSimIntegrationTest.*` (5 Tests): Test-Logik PASS, aber Process-Teardown SegFault. Vermutlich pre-existing (oder durch Phase 1 WorldMap-Aliasing). Bisect war nicht im Budget.
  - Fehlende `.pyi`-Stubs für `raccoon.map` und `raccoon.localization`. User: **„stubs sind überflüssig in the near future"** — überspringen.
- **Phase 4 Validation 1**: 3 echte Regressionen (Multi-Segment-Distanz, Post-Turn-Hang, Spline-Sign).
- **Phase 4 Fix** (Codex-Rescue): 3 Bugs gefixt, Verifikation `pytest test_smooth_path test_path_optimizer test_spline_path` → 89 passed in 59s. Commit `4e5848d`.
- **Phase 4 Validation 2 (FÄLLIG)**: nach dem Fix wurde bisher nur das Sub-Set verifiziert. Bevor Phase 5 startet, wäre eine zweite gezielte Sweep sinnvoll — vor allem ob die straffen Drumbot-Toleranzen aus `9199992` gegen die fixed Phase-4-Welt halten.

### Untracked / Workdir

`.codex/` ist gitignored (Codex-Rescue-Worktree-Output, gehört nicht ins Repo).

---

## Was als nächstes ansteht: Phasen 5-7

### Phase 5 — DSL desugart auf absolute Plan-IR

**Doc-Zeile:** „DSL desugart auf absolute Plan-IR. Konservativer Optimizer (nur `validate_reachable` + `fold_implicit_turns`). Default flippt für neue Code-Pfade; alte DSL bleibt funktional."

**Vorhandene Munition aus Phase 3:**
- `modules/libstp-motion/python/raccoon/step/motion/path/abs_ir.py` — Goto/TurnTo/Resync/Action/AbsoluteNode
- `modules/libstp-motion/python/raccoon/step/motion/path/abs_factory.py` — `goto/turn_to/resync/action` (cm/deg → m/rad)
- `modules/libstp-motion/python/raccoon/step/motion/path/abs_compiler.py` — `compile_plan(nodes) -> CompiledAbsolutePlan` (1:1, keine Optimizer-Passes)

**Was zu bauen ist:**

1. **Desugaring** der heutigen relativen DSL (`drive_forward(70)`, `turn_left(90)`, `flow(...)`, `smooth_path(...)`) auf eine absolute IR-Liste. Nur ein Compile-Time-Pass — Runtime-Pose wird zur Compile-Zeit nicht bekannt sein, also ist die "Pose-Chain" *intended* (was der Plan-Doc-Schritt 1 unter "dummes Desugaring" zeigt). Der echte Welt-Heading wird erst beim Executor pro Segment aus `localization.get_pose()` gelesen — Phase 4 macht das schon.
2. **Optimizer-Passes** (alle als `IR -> IR`-Funktionen):
   - `validate_reachable` (default **on**) — Posen außerhalb der Map → CompileError.
   - `fold_implicit_turns` (default **on**) — `TurnTo(θ)` vor `Goto(target_heading=θ)` löschen, **nur** wenn dazwischen keine Action / kein parallel-Block lief.
   - Weitere Passes (`merge_collinear_gotos`, `infer_heading`, `lift_actions`, `insert_resync_hints`) — default **off**, jede pro Pass per Flag freischaltbar.
3. **Executor-Anschluss**: PathExecutor (in Phase 4 schon absolute-aware) muss die desugarte IR konsumieren. Pragmatischer Pfad: ein `compile_relative_to_absolute(steps) -> CompiledAbsolutePlan`-Helper, den `flow()`/`smooth_path()` aufruft, dann den heutigen PathExecutor mit absoluten Targets füttern. Kein neuer Executor.
4. **Snapshot-Tests**: Plan-Doc verlangt „Snapshot-Tests gegen heutige Mission-Outputs". Phase 3 hat Trajektorien-Snapshots auf Phase 4 verschoben — Phase 5 erbt diese Pflicht. Konkret: einige typische Missionen aus `tests/python/sim/` (smooth_path, path_optimizer) als Endpose-Snapshot speichern, dann verifizieren dass Desugaring nichts dreht.
5. **Default-Flip**: User-Code soll weiter funktionieren (relative DSL bleibt User-API), aber unter der Haube läuft alles durch absolute IR. Plan-Doc sagt "Default flippt für neue Code-Pfade; alte DSL bleibt funktional" — heißt: kein Bruch der User-API.

**Plan-Anweisung wiederholt im Plan-Doc (Implementation Notes):**

> "Phase 3 — Python-IR statt C++-IR: compile_plan ist Mission-Startup, kein Hot-Path. Action-Knoten wickeln Python-Steps; ein C++-IR mit py::object würde Bindings-Komplexität ohne Performance-Gewinn bringen. Folge: Phase 5 (Optimizer-Passes) bleibt ebenfalls Python."

Halte dich daran. Optimizer in Python.

**User-Wunsch (zentral!):** *„lass die relative api leben! So wie zB flow() steps ... ABER: lass die relative api leben!"* — relative DSL ist User-Default, absolute IR ist Compile-Target unter der Haube. Plan-Doc-Sektion „Relative API darf neben Absolute leben" steht explizit drin.

**Risiken (aus dem Plan-Doc, „Tradeoffs / Risiken"):**
- Snapshot-Tests gegen heutige Mission-Recordings sind PFLICHT. Subtile Bugs im Desugaring (parallel(...)-Blöcke, Conditions ohne bekanntes Endpoint) brechen alte Missionen schweigend.
- Jeder Optimizer-Pass kriegt einen Property-Test "Plan-Endpose unverändert nach Pass".

**Aufteilung Empfehlung (vom Vorgänger-Orchestrator):**
- Commit A — Desugaring relative → absolute IR (`compile_relative_to_absolute`), keine Passes. Snapshot-Tests gegen 3-5 typische Missionen.
- Commit B — `validate_reachable` Pass + Tests.
- Commit C — `fold_implicit_turns` Pass + Property-Tests.
- Commit D — PathExecutor / flow() / smooth_path() konsumieren intern den absoluten Pfad. User-API unverändert. Snapshot-Equivalenz gegen Commit A.
- (Optional, nur falls Bedarf) Commit E — weitere Optimizer-Passes (off-by-default).

### Phase 6 — Particle Filter

Plan-Doc-Zeile: „Particle Filter ersetzt den Pass-Through in Localization. Resync-Steps (`align_to_wall`, `find_line`, `resync_at_start_pose`) werden funktional. Performance-Profil auf Pi3."

Vorbereitung steht: `libstp-map` hat sensor-projected queries (`sensorIsOnLine`, `sensorIsOnWall`, `sensorFieldPosition`), `Localization::observe(Observation)` API existiert. Filter selbst (Prediction = Stm32-Delta-Propagation, Observation = IR-Reflektanz, Resampling = Low-Variance) muss im `libstp-localization`-Modul ergänzt werden — der heutige Pass-Through wird durch den Filter ersetzt.

Resync-Steps gehören in `libstp-motion` oder `libstp-step` (entscheide pragmatisch — sie sind Steps, also vermutlich `libstp-step` oder eigenes Modul).

Performance: 100-200 Partikel × 100 Hz × N_Sensoren × Map-Lookup. Plan-Doc: "Profiling vor Phase 5; falls zu langsam, fallen Optionen: weniger Partikel, niedrigere Update-Rate (50 Hz), oder Adaptive-KLD-Sampling."

**Plan-Anpassung nötig:** der ursprüngliche Plan-Doc nennt RobotGeometry als Vorbedingung-für-C++-Port. Phase 1 hat das *nicht* gemacht (RobotGeometry blieb Python-Mixin, weil Sensor-Keys Python-Instanzen sind). Phase 6 muss entscheiden, ob der Particle-Filter die Sensor-Offsets als reine POD-Liste in C++ konsumiert (RobotGeometry baut die Liste an einem Punkt vor Filter-Init) oder ob doch ein C++-RobotGeometry-Refactor kommt. Empfehlung des Vorgänger-Orchestrators: erste Variante, weil sie minimal-invasiv und Phase-6-lokal ist.

### Phase 7 — Cleanup

Plan-Doc: „WorldPoseTracker-Reste, odometry.reset() aus Motion-Starts, alte relative-only Code-Pfade in Tests/Examples."

WorldPoseTracker und Correction sind durch Phase 4 schon weg. odometry.reset() aus Motion-Starts ebenfalls. Phase 7 ist dadurch dünner als ursprünglich erwartet — vermutlich nur:
- Reste des Reset-Detection-Heuristik-Pfads in `Localization::tickLoop()` aufräumen wenn Phase 4-Garantie hält.
- Alte relative-only Tests, Examples, Docs anschauen — was braucht ein Update?
- Telemetry / Replay-Tooling (Plan-Doc-Tradeoff "Debugging wird anders" verlangt das).

---

## Repo-Spielregeln

### Build-Pfad (lokal, schnell)

- Inkremental C++: `cmake --build build-test-local -j` — ~15s wenn cache warm
- Voller editable install (subprocess-Sim-Tests brauchen das, sonst alter Code): `pip install -e . --break-system-packages --no-build-isolation` — ~17s
- Stub-Generation überspringbar: `cmake -D LIBSTP_RUN_MYPY=OFF` (es gibt ~838 pre-existing mypy-Errors in unrelated code).
- Driver-Bundle für Sim: `--config-settings=cmake.define.DRIVER_BUNDLE=mock` falls neu konfiguriert wird.

### Test-Filter (zügig)

- C++: `ctest -R "Motion|Sim|Localization|Map|Collision" --output-on-failure -j` aus `build-test-local/`.
- Python (Phase-Hot-Path): `pytest tests/python/test_map_binding.py tests/python/test_localization_passthrough.py tests/python/test_abs_ir_construction.py tests/python/test_abs_compiler.py -x --tb=short`.
- Python Sim-Regressionen: `timeout 300 pytest tests/python/sim/test_localization_cross_motion.py tests/python/sim/test_drive_mission.py tests/python/sim/test_smooth_path_composite.py tests/python/sim/test_path_optimizer.py tests/python/sim/test_sim_bindings.py -x --tb=short`.
- pyproject hat `filterwarnings=error` — bei Bedarf `-o filterwarnings=ignore::pytest.PytestAssertRewriteWarning`.
- **Niemals** `pytest tests/` (full suite) ohne Timeout — `test_auto_tune` ist slow.

### pre-commit-Hook (commit MUSS zweimal versucht werden)

Hooks fixen automatisch:
- `end-of-file-fixer` (newline am Datei-Ende)
- `ruff` (auto-fix für viele Lints; manche `EM101`/`EM102` sind manual: f-strings in Exception → in `msg = ...` Variable extrahieren, dann `raise ...(msg)`)
- `ruff-format` (Reformatierung)
- `cmake-format`

**Standard-Pattern**: ersten `git commit` versuchen → Hook fixt automatisch → die fixierten Files sind unstaged → `git add -A && git commit -m ...` erneut. Beim zweiten Mal sind alle Hooks grün.

`EM101`/`EM102` (string in raise): umschreiben `raise X("...")` → `msg = "..."; raise X(msg)`. Das ist ein expliziter Manual-Fix.

### Conventional Commits

```
<type>(<scope>): <subject>

<body — was, warum, wie verifiziert>

Part of phase N of docs/design/absolute-motion-plan.md.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
```

Co-Author-Trailer ggf. anpassen — wenn Codex der Executor war: zusätzlich `Co-Authored-By: Codex <noreply@openai.com>`.

### Branch-Hygiene

- Niemals force-pushen.
- Niemals `git reset --hard` ohne explizite User-Erlaubnis.
- Hook-Fails: NEUEN Commit machen, niemals `--amend`.

---

## Sub-Agent-Catalog (was ich genutzt habe)

- **Plan** (Sub-Agent-Type) — für Phase-Plans. Bekommt Plan-Doc + Recherche-Auftrag, liefert Commit-Aufteilung mit konkreten Dateien.
- **general-purpose** — für Executor-Aufträge. Bekommt einen Commit-Brief mit Datei-Liste, erlaubten/verbotenen Bereichen, Stop-Punkten.
- **codex:codex-rescue** — für tiefere Diagnose / Root-Cause / parallele Implementation. Achtung: schreibt in `.codex/`-Worktree, kann manchmal nicht selbst committen (read-only `.git` in Sandbox) → du committest dann selbst. Sehr gut für Bug-Hunting.
- **advisor()** (kein Sub-Agent, sondern Tool im Orchestrator) — bei Plan-Abweichungen, Architektur-Entscheidungen, Reibungspunkten. Frage konkret.

### Brief-Template Executor

```
Commit X der Phase-N-Migration in <repo> (Branch absolute-motion).
Lies docs/design/absolute-motion-plan.md (Phase N + relevante Sektionen)
und git log -<n> für Kontext.

ZIEL Commit X: <eine Sache>

KEIN BUILD, KEINE TESTS AUSFÜHREN.

KONKRETE SCHRITTE:
1. <recherche-schritt>
2. <datei-änderung mit pfad>
...

WICHTIG / SCOPE:
- KEIN Anfassen von <out-of-scope-Bereich>
- ...

KONFLIKT-FALL: <wann melden statt fixen>

LIEFERUNG: Bericht (max. 400 Wörter):
- Geänderte Dateien
- Reibungspunkte
- git status + git diff --stat
- nicht selbst committen
```

---

## Erwartetes nächstes Vorgehen

1. **Phase 4 Validation-2** als Validator-Agent triggern (Test-Sweep gegen den fixed Stand) — verifizieren ob die strikten Drumbot-Toleranzen aus `9199992` halten und ob alle Sim-Tests durchlaufen.
2. Falls grün: **Phase 5 Plan-Agent** für die relative-zu-absolute Desugaring + Optimizer-Passes.
3. Pro Commit ein Executor-Agent.
4. Nach Phase-5-Abschluss: nochmal Validator-Agent.
5. Phase 6 ist substanziell (Particle Filter) — vor Beginn unbedingt advisor() um Architektur abzusichern.

**User-Charakter:** liebt Pace und Klarheit, mag keine Kontext-Verschwendung. Wenn ein Architektur-Entscheid ansteht, **eine** klare Frage mit 2-3 Optionen + Empfehlung. Niemals "soll ich X, Y oder Z?" mit 5 Bullets — das frisst seinen Kontext.

**User-Sprache:** deutsch, du-Form. Keine Emojis. Knapp.

Viel Erfolg.
