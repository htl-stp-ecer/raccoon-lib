# Arm IK — Design & Implementierungsplan

**Status:** Design · **Scope:** raccoon-lib + toolchain

---

## TL;DR

Roboterarm-IK löst bei Botball zwei Probleme: Koordinaten-zu-Servo-Mapping und Arm-Kalibrierung. Der Ansatz hier: **IK läuft ausschließlich am Laptop zur Codegen-Zeit** — auf dem Wombat landen nur fertige Servo-Winkel. Keine IK-Library, kein scipy, kein numpy auf dem Roboter.

Named positions (der 99%-Fall im Wettkampf) kosten zur Laufzeit null Overhead. Dynamische Positionen (`arm.to(x,y,z)`) sind optional und benötigen ikpy nur wenn explizit gebraucht.

---

## Architektur-Überblick

```
Dev-Laptop (raccoon codegen):
  raccoon.project.yml
    → ArmChainGenerator (toolchain)
    → ikpy löst IK pro named position
    → joint angles → servo_deg via 2-Punkt-Map
    → defs.py  (ArmPreset mit eingebetteten Winkel-Literalen)
    → defs.pyi (type-safe Methoden pro Position)

Wombat (runtime):
  ArmPreset.grab_cone()
    → lookup vorberechnete Winkel
    → seq([servo(j, a), ...])   ← existierende Steps, null IK
```

**Invariante:** `ArmPreset` und alle zugehörigen Runtime-Klassen haben **keine** Abhängigkeit auf ikpy, numpy oder scipy. Diese Libs leben nur im toolchain als dev-dependency.

---

## YAML-Syntax

Alle Felder außer `type`, `joints[].servo`, `joints[].length_cm` und `positions` sind **optional**. Man kann minimal einsteigen und Features nach Bedarf hinzufügen.

```yaml
# raccoon.project.yml → definitions:

arm:
  type: ArmChain

  # ── Optionaler Workspace-Guard ─────────────────────────────────────────────
  # Codegen prüft ob gelöste Kartesian-Position im erlaubten Bereich liegt.
  # Fehler zur Codegen-Zeit, nicht zur Laufzeit.
  workspace:                        # optional
    z_min_cm: 2                     # Arm darf nicht unter 2 cm (Chassis-Schutz)
    z_max_cm: 40                    # optionale Obergrenze
    reach_max_cm: 25                # max. Radius vom Ursprung des ersten Gelenks

  # ── Gelenke (Reihenfolge = kinematische Kette, Basis → Spitze) ────────────
  joints:
    - servo: shoulder_servo         # Referenz auf ein anderes definitions-Entry
      length_cm: 12.5               # Linklänge für die Kinematik

      # Mapping: IK rechnet in joint_range_deg, Servo bekommt servo_range_deg.
      # Lineare Interpolation zwischen den zwei Punkten.
      # Invertierung: servo_range_deg: [130, 10] (hi zuerst).
      joint_range_deg: [0, 90]      # optional; default [0, 180]
      servo_range_deg: [10, 130]    # optional; default = joint_range_deg

    - servo: elbow_servo
      length_cm: 10.0
      joint_range_deg: [-45, 45]
      servo_range_deg: [150, 30]    # invertierter Servo

  # ── Verbotene Zonen (optional) ─────────────────────────────────────────────
  # Compound-Bedingungen über Gelenkwinkel. Codegen lehnt Positionen ab,
  # die in eine forbidden zone fallen. Nützlich für Selbstkollision oder
  # Kollision mit dem Chassis bei bestimmten Gelenk-Kombinationen.
  forbidden_zones:                  # optional
    - name: hits_chassis            # nur für Fehlermeldungen
      condition: "shoulder_deg > 75 and elbow_deg < -30"

    - name: cable_tension
      condition: "shoulder_deg < 10 and elbow_deg > 40"

  # ── Named Positions ────────────────────────────────────────────────────────
  # Kartesische Ziele (cm, relativ zum Ursprung des ersten Gelenks).
  # IK wird zur Codegen-Zeit gelöst. Ergebnis: vorberechnete Servo-Winkel.
  positions:
    grab_cone:   {x: 18, y: 0,  z: 5}
    drop_zone:   {x: 10, y: 12, z: 20}
    home:        {x: 0,  y: 0,  z: 25}
```

### Servo-Referenz-Prinzip

Gelenke referenzieren existierende `Servo`-Einträge aus `definitions:`. Der Servo ist damit weiterhin direkt über `Defs.shoulder_servo` zugänglich — keine Doppelkonfiguration.

```yaml
definitions:
  shoulder_servo:
    type: Servo
    port: 0
    positions:          # normale ServoPreset-Positionen bleiben erhalten
      home: 90

  arm:
    type: ArmChain
    joints:
      - servo: shoulder_servo   # ← Referenz, kein Duplikat
        ...
```

---

## Generiertes Code

### defs.py

```python
# Auto-generated — do not edit
class Defs:
    shoulder_servo = ServoPreset(Servo(port=0), positions={"home": 90})
    elbow_servo    = ServoPreset(Servo(port=1), positions={"home": 90})

    arm = ArmPreset(
        joints=[shoulder_servo.device, elbow_servo.device],
        positions={
            "grab_cone": [45.2, -30.1],   # Servo-Grad, bereits konvertiert
            "drop_zone": [67.8,  12.3],   # inkl. Offset, Invertierung, Scale
            "home":      [90.0,   0.0],
        },
    )
```

### defs.pyi

```python
# Auto-generated type stub
from raccoon.step.arm import ArmPreset
from raccoon.step import Step

class _ArmPreset(ArmPreset):
    def grab_cone(self, speed: float | None = None) -> Step: ...
    def drop_zone(self, speed: float | None = None) -> Step: ...
    def home(self, speed: float | None = None) -> Step: ...
    # .to() kommt von ArmPreset base class

class Defs:
    ...
    arm: _ArmPreset
```

---

## Runtime-API (ArmPreset)

### Named positions — zero overhead

```python
# Instant (parallele servo()-Calls)
Defs.arm.grab_cone()

# Eased (parallele slow_servo()-Calls)
Defs.arm.grab_cone(speed=60)

# In einer Sequenz
seq([
    Defs.arm.home(),
    drive_forward(20),
    Defs.arm.grab_cone(speed=80),
    Defs.claw_servo.closed(),
])
```

### Dynamische Positionen (optional, runtime-IK)

```python
# Nur verfügbar wenn ikpy installiert ist
# Für den seltenen Fall wo Koordinaten erst zur Laufzeit bekannt sind
Defs.arm.to(x=18, y=0, z=5)
Defs.arm.to(x=18, y=0, z=5, speed=60)
```

---

## Codegen-Fehler (Beispiele)

Alle Probleme werden beim `raccoon codegen` auf dem Laptop gefangen:

```
ERROR  arm.positions.too_low
       Gelöste Position z=0.5 cm verletzt workspace.z_min_cm=2.0
       IK-Ergebnis: shoulder=87.3°, elbow=-31.2° → x=18.0, y=0.0, z=0.5 cm

ERROR  arm.positions.grab_far
       IK konnte nicht innerhalb joint_range_deg konvergieren (max_iter=500)
       shoulder_servo: benötigt 95.2° aber joint_range_deg=[0, 90]
       → Position außerhalb der Reichweite oder hinter einem Limit

ERROR  arm.positions.bad_config
       Trifft forbidden_zone "hits_chassis"
       shoulder_deg=78.1 > 75 AND elbow_deg=-33.4 < -30
       → Arm würde mit Chassis kollidieren
```

---

## Implementierungsplan

### Phase 1 — raccoon-lib: `ArmPreset` Runtime-Klasse

**Neues Modul:** `modules/libstp-arm/`

Analoges Muster zu `libstp-servo/`. Keine externen Dependencies.

**Dateien:**

```
modules/libstp-arm/python/raccoon/step/arm/
  __init__.py
  preset.py          ← ArmPreset + _ArmPosition
  steps.py           ← ArmToPosition Step (für .to())
```

**`preset.py` — Kernklasse:**

```python
class _ArmPosition:
    """Callable analog zu _PresetPosition in ServoPreset."""

    def __init__(self, joints: list[Servo], angles_deg: list[float], name: str):
        self._joints = joints
        self._angles = angles_deg
        self.__name__ = name

    def __call__(self, speed: float | None = None) -> "Step":
        from raccoon.step.servo.steps import servo as _servo, SlowServo
        from raccoon.step.step import seq
        steps = [
            _servo(j, a) if speed is None else SlowServo(j, a, speed=speed)
            for j, a in zip(self._joints, self._angles)
        ]
        return seq(steps)


class ArmPreset:
    def __init__(self, joints: list[Servo], positions: dict[str, list[float]]):
        self._joints = joints
        self._positions = positions
        for name, angles in positions.items():
            setattr(self, name, _ArmPosition(joints, angles, name))

    def to(self, x: float, y: float, z: float, speed: float | None = None) -> "Step":
        """Runtime-IK. Benötigt ikpy (optional dependency)."""
        try:
            import ikpy.chain  # noqa: PLC0415
        except ImportError:
            msg = "arm.to() benötigt ikpy: pip install ikpy"
            raise ImportError(msg) from None
        # ... IK solve + _ArmPosition(...)()
```

**Exports in `raccoon/__init__.py`:** `ArmPreset` neben `ServoPreset`.

### Phase 2 — toolchain: `ArmChainGenerator`

**Dateien:**

```
toolchain/raccoon_cli/codegen/generators/
  arm_generator.py        ← ArmChainGenerator
  arm_stub_generator.py   ← generiert _ArmPreset Stub-Klasse
```

**Ablauf in `ArmChainGenerator.generate_body()`:**

1. Joints aus YAML lesen, Servo-Referenzen auflösen
2. ikpy-Chain aufbauen (URDFLink aus `length_cm` + `axis`)
3. Pro named position:
   a. Kartesische Ziel-Matrix aufbauen
   b. `chain.inverse_kinematics(target)` aufrufen
   c. Gelenkwinkel (rad) → Grad → Servo-Grad via linearer 2-Punkt-Map
   d. Optional: workspace-Bounds prüfen
   e. Optional: forbidden_zones prüfen
4. `ArmPreset(joints=[...], positions={...})` Literal emittieren

**Hilfsfunktion (codegen-only, nicht auf Roboter):**

```python
def joint_to_servo(joint_deg, joint_range, servo_range):
    j_lo, j_hi = joint_range
    s_lo, s_hi = servo_range
    t = (joint_deg - j_lo) / (j_hi - j_lo)
    return round(s_lo + t * (s_hi - s_lo), 2)
```

**toolchain `pyproject.toml` — dev dependency:**

```toml
[project.optional-dependencies]
arm = ["ikpy>=3.3"]
# raccoon codegen zieht diese automatisch wenn ArmChain erkannt
```

**`DefsGenerator` — Integration:**

```python
# in defs_generator.py, generate_body():
if type_name == "ArmChain":
    from .arm_generator import ArmChainGenerator
    arm_gen = ArmChainGenerator(field_name, hw_cfg, all_definitions)
    hw_expr = arm_gen.build_expr(self.imports)
    attributes.append((field_name, hw_expr))
    continue
```

**`DefsStubGenerator` — Integration:**

```python
# analog zu _ConeArmServoPreset:
elif type_name == "ArmChain":
    class_name = f"_{_to_camel(field_name)}Preset"
    preset_classes.append(_build_arm_preset_class(class_name, hw_cfg))
    fields.append((field_name, class_name))
```

### Phase 3 — Dokumentation (User-facing)

Zielort: Hugo-Doku-Site (htl-stp-ecer/documentation), analog zu bestehenden DSL-Katalog-Einträgen.

**Zu schreiben:**

- `docs/arm-chain.md` — Konzept & Quickstart (minimal working example)
- `docs/arm-chain-reference.md` — vollständige YAML-Referenz mit allen optionalen Feldern
- `docs/arm-calibration-guide.md` — Workflow: 2-Punkte-Kalibrierung, wie misst man servo_range

**DSL-Katalog-Eintrag** für `arm.to()` via `generate_dsl_catalog.py`.

---

## Abhängigkeiten & Scope

| Komponente | Dependency | Wo |
|---|---|---|
| `ArmPreset` (runtime) | keine | raccoon-lib |
| `ArmPreset.to()` | ikpy (optional import) | raccoon-lib |
| `ArmChainGenerator` | ikpy | toolchain dev-dep |
| `raccoon codegen` mit ArmChain | ikpy auto-installiert | toolchain |

ikpy kommt nie in die raccoon-lib Runtime-Dependencies. Der Wombat braucht es nicht.

---

## Offene Entscheidungen

- **`forbidden_zones` Syntax:** String-Expression (`"shoulder_deg > 75 and elbow_deg < -30"`) ist einfach zu schreiben aber `eval()`-basiert im Codegen. Alternative: strukturierte Form `{joint: shoulder, gt: 75}`. String ist für jetzt ok da Codegen auf dem Dev-Laptop läuft.
- **`arm.to()` Runtime-IK für 2-DOF:** Geschlossene Formel (kein ikpy, kein scipy) wäre sauberer als optionaler Import. Implementierung nach Phase 1+2.
- **Parallelität der Servo-Calls:** `_ArmPosition.__call__` gibt heute `seq([servo, servo])` zurück. Sollte `parallel([servo, servo])` sein — alle Gelenke bewegen sich gleichzeitig. Prüfen ob `parallel()` existiert und passt.
