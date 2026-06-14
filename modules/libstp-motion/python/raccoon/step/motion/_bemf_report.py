"""Persist BEMF→velocity calibration data + plots under
``.raccoon/auto_tune/bemf_velocity/<timestamp>/``.

Always writes ``data.csv``, ``summary.json`` and a human-readable
``findings.md`` plus a self-contained ``plot.py``. PNG plots are rendered
inline only when matplotlib is importable (optional ``plots`` extra); otherwise
the data + ``plot.py`` let you render them anywhere matplotlib is available.
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


def _motor_dicts(result):
    out = []
    for idx, m in enumerate(result.motors):
        if m.n_points == 0:
            continue
        out.append(
            {
                "index": idx,  # column index in data.csv (dticks<index>/bemf<index>/ttr<index>)
                "port": m.port,
                "n_points": m.n_points,
                "ticks_to_rad_median": m.ticks_to_rad_median,
                "ticks_to_rad_mean": m.ticks_to_rad_mean,
                "ticks_to_rad_cv": m.ticks_to_rad_cv,
                "bemf_omega_slope": m.bemf_omega_slope,
                "bemf_omega_intercept": m.bemf_omega_intercept,
                "bemf_omega_r2": m.bemf_omega_r2,
                "linear": bool(m.linear),
            }
        )
    return out


def _findings_md(result, motors) -> str:
    lines = [
        "# BEMF→velocity calibration findings",
        "",
        f"- success: **{result.success}**",
        f"- overall linear: **{result.linear_overall}**",
        f"- valid speed points: **{sum(1 for p in result.points if p.valid)}**",
        "",
        "## Per-motor results",
        "",
        "| port | ticks_to_rad (median) | CV over speeds | ω = a·bemf + b | R² | verdict |",
        "|---|---|---|---|---|---|",
    ]
    for m in motors:
        lines.append(
            f"| {m['port']} | {m['ticks_to_rad_median']:.6g} | "
            f"{100 * m['ticks_to_rad_cv']:.1f}% | "
            f"ω={m['bemf_omega_slope']:.4g}·bemf{m['bemf_omega_intercept']:+.4g} | "
            f"{m['bemf_omega_r2']:.4f} | {'LINEAR' if m['linear'] else 'NON-LINEAR'} |"
        )
    lines += [
        "",
        "## Interpretation",
        "",
        "`ticks_to_rad` is the BEMF-tick → wheel-angle scale, derived per speed as "
        "`(ground_truth_distance / wheel_radius) / Δticks`. A small **CV** across speeds "
        "means a single scale describes tick↔angle well (linear). The **ω-vs-BEMF** fit "
        "characterises the instantaneous ADC-BEMF↔wheel-speed relation: a near-zero "
        "intercept means BEMF is proportional to speed; a non-zero intercept exposes a "
        "**BEMF offset at standstill** (the firmware integrates phantom ticks with no "
        "dead-zone), which a single linear scale cannot remove and which dominates the "
        "residual odometry error at low speed / during acceleration.",
        "",
        "See `omega_vs_bemf.png`, `ticks_to_rad_vs_omega.png`, `residuals_vs_omega.png`, "
        "`omega_vs_pwm.png` (run `python plot.py` here if they are missing).",
        "",
    ]
    return "\n".join(lines)


def write_report(result, log=print) -> "Path | None":
    """Write the calibration report folder; return its path (or None)."""
    root = find_project_root()
    if root is None:
        log("  [report] no project root found — skipping autotune report")
        return None

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = Path(root) / ".raccoon" / "auto_tune" / "bemf_velocity" / ts
    out.mkdir(parents=True, exist_ok=True)

    n = len(list(result.ticks_to_rad))

    with (out / "data.csv").open("w", newline="") as f:
        w = csv.writer(f)
        cols = ["pwm", "distance_m", "time_s", "speed_mps", "omega_rad_s", "valid"]
        for i in range(n):
            cols += [f"dticks{i}", f"bemf{i}", f"ttr{i}"]
        w.writerow(cols)
        for p in result.points:
            row = [
                p.pwm_percent,
                p.ground_truth_distance_m,
                p.window_time_s,
                p.body_speed_mps,
                p.wheel_omega_rad_s,
                int(p.valid),
            ]
            for i in range(n):
                row += [p.delta_ticks[i], p.median_bemf[i], p.ticks_to_rad[i]]
            w.writerow(row)

    motors = _motor_dicts(result)
    summary = {
        "success": bool(result.success),
        "linear_overall": bool(result.linear_overall),
        "failure_reason": result.failure_reason,
        "ticks_to_rad": list(result.ticks_to_rad),
        "wheel_radius_note": "ticks_to_rad = (distance/wheel_radius)/delta_ticks per speed",
        "motors": motors,
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    (out / "findings.md").write_text(_findings_md(result, motors))
    (out / "plot.py").write_text(_PLOT_SCRIPT)

    made = False
    if importlib.util.find_spec("matplotlib") is not None:  # optional `plots` extra
        proc = subprocess.run(
            [sys.executable, "plot.py"],
            cwd=str(out),
            capture_output=True,
            text=True,
            check=False,
        )
        made = proc.returncode == 0
        if not made:
            log(f"  [report] plot.py failed: {proc.stderr.strip()[:200]}")
    else:
        log(
            "  [report] matplotlib not installed (pip install 'raccoon-library[plots]') "
            "— wrote data + plot.py; render plots where matplotlib is available"
        )

    log(f"  [report] autotune report -> {out}" + (" (with plots)" if made else ""))
    return out


# Self-contained plotting script written into each report folder. Reads
# data.csv + summary.json from its own directory; no raccoon imports.
_PLOT_SCRIPT = r'''#!/usr/bin/env python3
"""Render BEMF→velocity calibration plots from data.csv + summary.json.
Standalone: requires only matplotlib (+ csv/json). Run from this folder.
"""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
summary = json.loads((HERE / "summary.json").read_text())
motors = summary["motors"]
ports = [m["port"] for m in motors]

rows = []
with open(HERE / "data.csv") as f:
    for r in csv.DictReader(f):
        if int(r["valid"]) == 1:
            rows.append(r)
rows.sort(key=lambda r: float(r["omega_rad_s"]))
omega = [float(r["omega_rad_s"]) for r in rows]
pwm = [float(r["pwm"]) for r in rows]


def col(i, key):
    return [float(r[f"{key}{i}"]) for r in rows]


# 1) omega vs BEMF (instantaneous) + fit line
plt.figure(figsize=(8, 6))
for m in motors:
    i = m["index"]
    bemf = col(i, "bemf")
    plt.scatter(bemf, omega, s=24, label=f"port {m['port']} (R²={m['bemf_omega_r2']:.4f})")
    if bemf:
        xs = [min(bemf), max(bemf)]
        ys = [m["bemf_omega_slope"] * x + m["bemf_omega_intercept"] for x in xs]
        plt.plot(xs, ys, "--", linewidth=1)
plt.xlabel("median |BEMF| (ADC counts)")
plt.ylabel("wheel speed ω (rad/s, ground truth)")
plt.title("BEMF → wheel speed  (dashed = linear fit)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(HERE / "omega_vs_bemf.png", dpi=130)
plt.close()

# 2) ticks_to_rad vs omega (constancy => linear tick<->angle)
plt.figure(figsize=(8, 6))
for m in motors:
    ttr = col(m["index"], "ttr")
    plt.scatter(omega, ttr, s=24,
                label=f"port {m['port']} (CV={100*m['ticks_to_rad_cv']:.1f}%)")
    plt.axhline(m["ticks_to_rad_median"], linestyle=":", linewidth=1)
plt.xlabel("wheel speed ω (rad/s)")
plt.ylabel("ticks_to_rad  = (D/r)/Δticks")
plt.title("Per-speed ticks_to_rad  (flat = single scale holds)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(HERE / "ticks_to_rad_vs_omega.png", dpi=130)
plt.close()

# 3) residuals of the omega-vs-bemf fit (structure => nonlinearity)
plt.figure(figsize=(8, 6))
for m in motors:
    bemf = col(m["index"], "bemf")
    resid = [o - (m["bemf_omega_slope"] * b + m["bemf_omega_intercept"])
             for o, b in zip(omega, bemf)]
    plt.plot(omega, resid, "o-", linewidth=1, label=f"port {m['port']}")
plt.axhline(0.0, color="k", linewidth=0.8)
plt.xlabel("wheel speed ω (rad/s)")
plt.ylabel("residual ω − fit (rad/s)")
plt.title("ω-vs-BEMF fit residuals  (trend = nonlinearity / offset)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(HERE / "residuals_vs_omega.png", dpi=130)
plt.close()

# 4) omega vs PWM (sanity: open-loop speed curve)
plt.figure(figsize=(8, 6))
plt.plot(pwm, omega, "o-")
plt.xlabel("PWM (%)")
plt.ylabel("wheel speed ω (rad/s, ground truth)")
plt.title("Open-loop speed vs PWM")
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(HERE / "omega_vs_pwm.png", dpi=130)
plt.close()

print("wrote: omega_vs_bemf.png, ticks_to_rad_vs_omega.png, "
      "residuals_vs_omega.png, omega_vs_pwm.png")
'''
