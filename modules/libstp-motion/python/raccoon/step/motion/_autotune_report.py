"""Persist per-phase auto-tune calibration data + plots under
``.raccoon/auto_tune/<phase>/<timestamp>/``.

Mirrors :mod:`raccoon.step.motion._bemf_report` (the BEMF→velocity reporter):
each phase always writes ``data.csv``, ``summary.json`` and a human-readable
``findings.md`` plus a self-contained ``plot.py``. PNG plots are rendered inline
only when matplotlib is importable (optional ``plots`` extra); otherwise the
data + ``plot.py`` let you render them anywhere matplotlib is available.

Phases covered by ``write_report(phase_name, result, log=print)``:

- ``vel_lpf``          — VelLpfResult map {port: VelLpfResult}
- ``static_friction``  — StaticFrictionResult map {port: StaticFrictionResult}
- ``characterize``     — reads accel_*/decel_*.csv from a dump dir (env path)
- ``velocity``         — VelocityTuneResult map {axis: VelocityTuneResult}

Each phase is independent: a failure in one writer is logged and swallowed by
the calling DSL step so reporting never aborts a tune.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import subprocess
import sys
import time
from pathlib import Path

from raccoon.project_yaml import find_project_root


def _phase_dir(phase_name: str, log) -> "Path | None":
    """Create and return ``<project>/.raccoon/auto_tune/<phase>/<ts>/`` or None."""
    root = find_project_root()
    if root is None:
        log("  [report] no project root found — skipping autotune report")
        return None
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = Path(root) / ".raccoon" / "auto_tune" / phase_name / ts
    out.mkdir(parents=True, exist_ok=True)
    return out


def _render(out: "Path", plot_script: str, log) -> bool:
    """Write plot.py and render PNGs when matplotlib is available."""
    (out / "plot.py").write_text(plot_script)
    if importlib.util.find_spec("matplotlib") is None:  # optional `plots` extra
        log(
            "  [report] matplotlib not installed (pip install 'raccoon-library[plots]') "
            "— wrote data + plot.py; render plots where matplotlib is available"
        )
        return False
    proc = subprocess.run(
        [sys.executable, "plot.py"],
        cwd=str(out),
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        log(f"  [report] plot.py failed: {proc.stderr.strip()[:200]}")
        return False
    return True


# ===========================================================================
# Phase: vel_lpf
# ===========================================================================


def _write_vel_lpf(out: "Path", results: dict) -> None:
    """results: dict[port, VelLpfResult]."""
    motors = []
    for port, r in sorted(results.items()):
        motors.append(
            {
                "port": int(port),
                "initial_alpha": float(r.initial_alpha),
                "tuned_alpha": float(r.tuned_alpha),
                "min_score": float(r.min_score),
                "applied": bool(r.applied),
                "n_raw": len(list(r.raw_bemf)),
                "n_sweep": len(list(r.sweep)),
            }
        )

    # data.csv — long format: one row per (port, kind, x, y...).
    with (out / "data.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["port", "kind", "index", "x", "variance", "lag_change_rate", "score"])
        for port, r in sorted(results.items()):
            for i, b in enumerate(r.raw_bemf):
                w.writerow([port, "raw", i, b, "", "", ""])
            for sp in r.sweep:
                w.writerow(
                    [
                        port,
                        "sweep",
                        "",
                        f"{sp.alpha:.6g}",
                        sp.variance,
                        sp.lag_change_rate,
                        sp.score,
                    ]
                )

    (out / "summary.json").write_text(json.dumps({"phase": "vel_lpf", "motors": motors}, indent=2))

    lines = [
        "# Velocity-LPF alpha tuning findings",
        "",
        "Per-motor IIR `vel_lpf_alpha` chosen to minimise a weighted noise+lag score "
        "over the captured raw BEMF series.",
        "",
        "| port | initial alpha | tuned alpha | min score | applied | raw samples |",
        "|---|---|---|---|---|---|",
    ]
    for m in motors:
        lines.append(
            f"| {m['port']} | {m['initial_alpha']:.3f} | {m['tuned_alpha']:.3f} | "
            f"{m['min_score']:.6g} | {'yes' if m['applied'] else 'no'} | {m['n_raw']} |"
        )
    lines += [
        "",
        "See `raw_bemf.png` (raw trace per motor), `score_vs_alpha.png` (sweep curve, "
        "chosen alpha marked) and `raw_vs_filtered.png` (overlay at the chosen alpha). "
        "Run `python plot.py` here if they are missing.",
        "",
    ]
    (out / "findings.md").write_text("\n".join(lines))


# ===========================================================================
# Phase: static_friction
# ===========================================================================


def _write_static_friction(out: "Path", results: dict) -> None:
    """results: dict[port, StaticFrictionResult]."""
    motors = []
    for port, r in sorted(results.items()):
        motors.append(
            {
                "port": int(port),
                "ks_positive_pct": int(r.ks_positive_pct),
                "ks_negative_pct": int(r.ks_negative_pct),
                "ks_avg_pct": int(r.ks_avg_pct),
                "measured": bool(r.measured),
                "motion_threshold": int(r.motion_threshold),
            }
        )

    with (out / "data.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["port", "direction", "pwm_pct", "median_bemf"])
        for port, r in sorted(results.items()):
            for s in r.forward_sweep:
                w.writerow([port, "forward", s.pwm_pct, s.median_bemf])
            for s in r.reverse_sweep:
                w.writerow([port, "reverse", s.pwm_pct, s.median_bemf])

    (out / "summary.json").write_text(
        json.dumps({"phase": "static_friction", "motors": motors}, indent=2)
    )

    lines = [
        "# Static-friction (kS) findings",
        "",
        "Per-motor PWM duty swept upward in both directions; kS is the first PWM "
        "percent whose median |BEMF| crosses the motion threshold.",
        "",
        "| port | kS+ (%) | kS- (%) | kS avg (%) | measured | threshold |",
        "|---|---|---|---|---|---|",
    ]
    for m in motors:
        lines.append(
            f"| {m['port']} | {m['ks_positive_pct']} | {m['ks_negative_pct']} | "
            f"{m['ks_avg_pct']} | {'yes' if m['measured'] else 'no'} | "
            f"{m['motion_threshold']} |"
        )
    lines += [
        "",
        "See `bemf_vs_pwm.png` (BEMF vs PWM per motor, both directions, kS threshold "
        "and crossing marked). Run `python plot.py` here if missing.",
        "",
    ]
    (out / "findings.md").write_text("\n".join(lines))


# ===========================================================================
# Phase: characterize  (reads the accel_*/decel_*.csv dumped by C++)
# ===========================================================================


def _write_characterize(out: "Path", payload: dict) -> None:
    """payload: {"csv_dir": str, "axes": [...], "results": {axis: {...}}}.

    Copies the C++-dumped ``accel_<axis>.csv`` / ``decel_<axis>.csv`` (columns
    t,pos,vel) into the report folder and records max_v / accel / decel.
    """
    csv_dir = Path(payload["csv_dir"])
    axes = list(payload.get("axes", []))
    results = payload.get("results", {})

    axis_meta = []
    with (out / "data.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["axis", "phase", "t", "pos", "vel"])
        for axis in axes:
            n_accel = 0
            n_decel = 0
            for phase in ("accel", "decel"):
                src = csv_dir / f"{phase}_{axis}.csv"
                if not src.exists():
                    continue
                with src.open() as sf:
                    reader = csv.DictReader(sf)
                    for row in reader:
                        w.writerow([axis, phase, row["t"], row["pos"], row["vel"]])
                        if phase == "accel":
                            n_accel += 1
                        else:
                            n_decel += 1
            res = results.get(axis, {})
            axis_meta.append(
                {
                    "axis": axis,
                    "angular": axis == "angular",
                    "max_velocity": float(res.get("max_velocity", 0.0)),
                    "acceleration": float(res.get("acceleration", 0.0)),
                    "deceleration": float(res.get("deceleration", 0.0)),
                    "n_accel": n_accel,
                    "n_decel": n_decel,
                }
            )

    (out / "summary.json").write_text(
        json.dumps({"phase": "characterize", "axes": axis_meta}, indent=2)
    )

    lines = [
        "# Drive-characterization findings",
        "",
        "Open-loop full-power trials per axis: acceleration ramp + coast-down.",
        "",
        "| axis | max_v | accel | decel | accel samples | decel samples |",
        "|---|---|---|---|---|---|",
    ]
    for a in axis_meta:
        unit = "rad/s" if a["angular"] else "m/s"
        au = "rad/s^2" if a["angular"] else "m/s^2"
        lines.append(
            f"| {a['axis']} | {a['max_velocity']:.4f} {unit} | "
            f"{a['acceleration']:.4f} {au} | {a['deceleration']:.4f} {au} | "
            f"{a['n_accel']} | {a['n_decel']} |"
        )
    lines += [
        "",
        "See `velocity_vs_time_<axis>.png` (accel + coast-down, max_v / accel-slope / "
        "decel-slope annotated). Run `python plot.py` here if missing.",
        "",
    ]
    (out / "findings.md").write_text("\n".join(lines))


# ===========================================================================
# Phase: velocity (PID step response)
# ===========================================================================


def _write_velocity(out: "Path", results: dict) -> None:
    """results: dict[axis, VelocityTuneResult]."""
    axes_meta = []
    with (out / "data.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["axis", "phase", "t", "commanded", "measured"])
        for axis, r in results.items():
            for phase, resp in (("baseline", r.baseline_response), ("tuned", r.tuned_response)):
                times = list(resp.times)
                cmd = list(resp.commanded)
                meas = list(resp.measured)
                for i in range(len(times)):
                    w.writerow([axis, phase, times[i], cmd[i], meas[i]])
            axes_meta.append(
                {
                    "axis": axis,
                    "accepted": bool(r.accepted),
                    "Ks": float(r.plant.Ks),
                    "Tu": float(r.plant.Tu),
                    "Tg": float(r.plant.Tg),
                    "method": r.plant.method,
                    "kp": float(r.pid.kp),
                    "ki": float(r.pid.ki),
                    "kd": float(r.pid.kd),
                    "baseline_ise": float(r.baseline_ise),
                    "tuned_ise": float(r.tuned_ise),
                    "n_baseline": len(list(r.baseline_response.times)),
                    "n_tuned": len(list(r.tuned_response.times)),
                }
            )

    (out / "summary.json").write_text(
        json.dumps({"phase": "velocity", "axes": axes_meta}, indent=2)
    )

    lines = [
        "# Velocity-PID step-response findings",
        "",
        "Per-axis FOPDT plant identified from a baseline step response; CHR gains "
        "applied and validated by ISE.",
        "",
        "| axis | accepted | Ks | Tu (s) | Tg (s) | kp | ki | kd | ISE base→tuned |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for a in axes_meta:
        lines.append(
            f"| {a['axis']} | {'yes' if a['accepted'] else 'no'} | {a['Ks']:.4g} | "
            f"{a['Tu']:.4g} | {a['Tg']:.4g} | {a['kp']:.4g} | {a['ki']:.4g} | "
            f"{a['kd']:.4g} | {a['baseline_ise']:.4g}→{a['tuned_ise']:.4g} |"
        )
    lines += [
        "",
        "See `step_response_<axis>.png` (commanded vs measured, baseline & tuned, with "
        "FOPDT fit). Run `python plot.py` here if missing.",
        "",
    ]
    (out / "findings.md").write_text("\n".join(lines))


# ===========================================================================
# Dispatch
# ===========================================================================

_WRITERS = {
    "vel_lpf": (_write_vel_lpf, lambda: _VEL_LPF_PLOT),
    "static_friction": (_write_static_friction, lambda: _STATIC_FRICTION_PLOT),
    "characterize": (_write_characterize, lambda: _CHARACTERIZE_PLOT),
    "velocity": (_write_velocity, lambda: _VELOCITY_PLOT),
}


def write_report(phase_name: str, result, log=print) -> "Path | None":
    """Write the per-phase report folder; return its path (or None).

    ``result`` is whatever the phase produces (a {port/axis: Result} map, or the
    characterize payload dict). Unknown phases are ignored with a log line.
    """
    entry = _WRITERS.get(phase_name)
    if entry is None:
        log(f"  [report] no writer for phase '{phase_name}' — skipping")
        return None
    out = _phase_dir(phase_name, log)
    if out is None:
        return None

    writer, plot_getter = entry
    writer(out, result)
    made = _render(out, plot_getter(), log)
    log(f"  [report] {phase_name} report -> {out}" + (" (with plots)" if made else ""))
    return out


# ===========================================================================
# Self-contained plotting scripts (written into each report folder).
# Each reads data.csv + summary.json from its own directory; no raccoon imports.
# ===========================================================================

_VEL_LPF_PLOT = r'''#!/usr/bin/env python3
"""Render vel_lpf plots from data.csv + summary.json. Standalone (matplotlib only)."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
summary = json.loads((HERE / "summary.json").read_text())
motors = summary["motors"]
by_port_alpha = {m["port"]: m["tuned_alpha"] for m in motors}

raw = {}     # port -> [bemf...]
sweep = {}   # port -> [(alpha, score)...]
with open(HERE / "data.csv") as f:
    for r in csv.DictReader(f):
        port = int(r["port"])
        if r["kind"] == "raw":
            raw.setdefault(port, []).append(float(r["x"]))
        elif r["kind"] == "sweep":
            sweep.setdefault(port, []).append((float(r["x"]), float(r["score"])))

made = []

# 1) raw BEMF trace per motor
if any(raw.values()):
    plt.figure(figsize=(9, 6))
    for port, series in sorted(raw.items()):
        plt.plot(range(len(series)), series, linewidth=0.8, label=f"port {port}")
    plt.xlabel("sample index")
    plt.ylabel("raw BEMF (ADC counts)")
    plt.title("Quiescent raw BEMF trace per motor")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(HERE / "raw_bemf.png", dpi=130)
    plt.close()
    made.append("raw_bemf.png")

# 2) score vs alpha (chosen alpha marked)
if any(sweep.values()):
    plt.figure(figsize=(9, 6))
    for port, pts in sorted(sweep.items()):
        pts = sorted(pts)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        line, = plt.plot(xs, ys, "o-", markersize=3, label=f"port {port}")
        a = by_port_alpha.get(port)
        if a is not None:
            plt.axvline(a, color=line.get_color(), linestyle=":", linewidth=1)
    plt.xlabel("alpha")
    plt.ylabel("score (lower is better)")
    plt.title("Score vs alpha  (dotted = chosen alpha)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(HERE / "score_vs_alpha.png", dpi=130)
    plt.close()
    made.append("score_vs_alpha.png")

# 3) raw vs filtered overlay at the chosen alpha (per motor)
def iir(series, alpha):
    out = []
    y = series[0] if series else 0.0
    for v in series:
        y = (1.0 - alpha) * y + alpha * v
        out.append(y)
    return out

ports_with_raw = [p for p in raw if raw[p]]
if ports_with_raw:
    n = len(ports_with_raw)
    fig, axes = plt.subplots(n, 1, figsize=(9, 3 * n), squeeze=False)
    for ax, port in zip([a[0] for a in axes], ports_with_raw):
        series = raw[port]
        a = by_port_alpha.get(port, 0.5)
        ax.plot(range(len(series)), series, linewidth=0.7, alpha=0.5, label="raw")
        ax.plot(range(len(series)), iir(series, a), linewidth=1.4,
                label=f"filtered (alpha={a:.2f})")
        ax.set_title(f"port {port}")
        ax.set_xlabel("sample index")
        ax.set_ylabel("BEMF")
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(HERE / "raw_vs_filtered.png", dpi=130)
    plt.close(fig)
    made.append("raw_vs_filtered.png")

print("wrote:", ", ".join(made) if made else "(no data to plot)")
'''


_STATIC_FRICTION_PLOT = r'''#!/usr/bin/env python3
"""Render static-friction plot from data.csv + summary.json. Standalone."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
summary = json.loads((HERE / "summary.json").read_text())
motors = {m["port"]: m for m in summary["motors"]}

curves = {}  # (port, direction) -> [(pwm, bemf)...]
with open(HERE / "data.csv") as f:
    for r in csv.DictReader(f):
        key = (int(r["port"]), r["direction"])
        curves.setdefault(key, []).append((int(r["pwm_pct"]), int(r["median_bemf"])))

ports = sorted({p for (p, _) in curves})
made = []
if ports:
    n = len(ports)
    fig, axes = plt.subplots(n, 1, figsize=(9, 3 * n), squeeze=False)
    threshold = next((m["motion_threshold"] for m in motors.values()), None)
    for ax, port in zip([a[0] for a in axes], ports):
        for direction, marker in (("forward", "o-"), ("reverse", "s-")):
            pts = sorted(curves.get((port, direction), []), key=lambda p: abs(p[0]))
            if not pts:
                continue
            xs = [abs(p[0]) for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(xs, ys, marker, markersize=3, label=direction)
        m = motors.get(port, {})
        if threshold is not None:
            ax.axhline(threshold, color="k", linestyle=":", linewidth=1,
                       label=f"threshold={threshold}")
        if m.get("ks_positive_pct"):
            ax.axvline(m["ks_positive_pct"], color="tab:blue", linestyle="--",
                       linewidth=1, label=f"kS+={m['ks_positive_pct']}%")
        if m.get("ks_negative_pct"):
            ax.axvline(m["ks_negative_pct"], color="tab:orange", linestyle="--",
                       linewidth=1, label=f"kS-={m['ks_negative_pct']}%")
        ax.set_title(f"port {port}")
        ax.set_xlabel("|PWM| (%)")
        ax.set_ylabel("median |BEMF|")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(HERE / "bemf_vs_pwm.png", dpi=130)
    plt.close(fig)
    made.append("bemf_vs_pwm.png")

print("wrote:", ", ".join(made) if made else "(no data to plot)")
'''


_CHARACTERIZE_PLOT = r'''#!/usr/bin/env python3
"""Render drive-characterization plots from data.csv + summary.json. Standalone."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
summary = json.loads((HERE / "summary.json").read_text())
meta = {a["axis"]: a for a in summary["axes"]}

# axis -> phase -> [(t, vel)...]
series = {}
with open(HERE / "data.csv") as f:
    for r in csv.DictReader(f):
        series.setdefault(r["axis"], {}).setdefault(r["phase"], []).append(
            (float(r["t"]), float(r["vel"]))
        )

made = []
for axis, phases in series.items():
    a = meta.get(axis, {})
    angular = a.get("angular", False)
    unit = "rad/s" if angular else "m/s"
    plt.figure(figsize=(9, 6))
    t_offset = 0.0
    for phase, style in (("accel", "tab:blue"), ("decel", "tab:red")):
        pts = sorted(phases.get(phase, []))
        if not pts:
            continue
        ts = [p[0] + t_offset for p in pts]
        vs = [abs(p[1]) for p in pts]
        plt.plot(ts, vs, color=style, linewidth=1, label=phase)
        if pts:
            t_offset = ts[-1]
    mv = a.get("max_velocity", 0.0)
    if mv > 0:
        plt.axhline(mv, color="k", linestyle=":", linewidth=1, label=f"max_v={mv:.3f} {unit}")
    title = (f"{axis}: max_v={a.get('max_velocity', 0):.3f} {unit}, "
             f"accel={a.get('acceleration', 0):.3f}, decel={a.get('deceleration', 0):.3f}")
    plt.title(title)
    plt.xlabel("time (s)  [accel then coast-down]")
    plt.ylabel(f"|velocity| ({unit})")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    fname = f"velocity_vs_time_{axis}.png"
    plt.savefig(HERE / fname, dpi=130)
    plt.close()
    made.append(fname)

print("wrote:", ", ".join(made) if made else "(no data to plot)")
'''


_VELOCITY_PLOT = r'''#!/usr/bin/env python3
"""Render velocity-PID step-response plots from data.csv + summary.json. Standalone."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
summary = json.loads((HERE / "summary.json").read_text())
meta = {a["axis"]: a for a in summary["axes"]}

# axis -> phase -> [(t, commanded, measured)...]
series = {}
with open(HERE / "data.csv") as f:
    for r in csv.DictReader(f):
        series.setdefault(r["axis"], {}).setdefault(r["phase"], []).append(
            (float(r["t"]), float(r["commanded"]), float(r["measured"]))
        )

made = []
for axis, phases in series.items():
    a = meta.get(axis, {})
    plt.figure(figsize=(9, 6))
    base = sorted(phases.get("baseline", []))
    if base:
        ts = [p[0] for p in base]
        plt.plot(ts, [p[1] for p in base], "k--", linewidth=1, label="commanded")
        plt.plot(ts, [p[2] for p in base], linewidth=1.2, label="measured (baseline)")
    tuned = sorted(phases.get("tuned", []))
    if tuned:
        plt.plot([p[0] for p in tuned], [p[2] for p in tuned], linewidth=1.2,
                 label="measured (tuned)")
    title = (f"{axis}: {'ACCEPTED' if a.get('accepted') else 'reverted'} | "
             f"Ks={a.get('Ks', 0):.3g} Tu={a.get('Tu', 0):.3g}s Tg={a.get('Tg', 0):.3g}s | "
             f"ISE {a.get('baseline_ise', 0):.3g}->{a.get('tuned_ise', 0):.3g}")
    plt.title(title)
    plt.xlabel("time since step (s)")
    plt.ylabel("velocity (m/s or rad/s)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    fname = f"step_response_{axis}.png"
    plt.savefig(HERE / fname, dpi=130)
    plt.close()
    made.append(fname)

print("wrote:", ", ".join(made) if made else "(no data to plot)")
'''
