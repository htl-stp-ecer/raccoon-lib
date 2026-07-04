#!/usr/bin/env python3
"""Render the to_absolute drift-correction demo as one overlay PNG.

Runs ``tests/python/sim/_to_absolute_drift_runner.py`` three ways (ref / baseline
/ absolute) in subprocesses, captures each ground-truth trajectory, and draws
them on one figure: the no-drift ``ref`` target path, the drift-blown
``baseline`` path, and the landmark-corrected ``absolute`` path. The point is
visible at a glance — under identical injected odometry drift the plain baseline
walks off the target while ``optimize(...).to_absolute()`` stays on it.

    .venv-test/bin/python tools/render_drift_demo.py
    .venv-test/bin/python tools/render_drift_demo.py --out drift_demo/

Wall-clock (NOT fast-time): the localization worker integrates on the real clock.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNNER = REPO_ROOT / "tests" / "python" / "sim" / "_to_absolute_drift_runner.py"

STYLE = {
    "ref": ("#2ca02c", "ref: no drift (intended path)", "-"),
    "baseline": ("#9e9e9e", "baseline + drift (drifts off target)", "--"),
    "absolute": ("#ff7f0e", "to_absolute + drift + landmarks (corrected)", "-"),
}


def _run(config: str) -> dict:
    env = os.environ.copy()
    env["RACCOON_SIM"] = "1"
    env.setdefault("LIBSTP_LOG_LEVEL", "error")
    env.setdefault("LIBSTP_NO_CALIBRATE", "1")
    env["LIBSTP_TIMING_ENABLED"] = "0"
    env.pop("RACCOON_SIM_FASTTIME", None)
    proc = subprocess.run(
        [sys.executable, str(RUNNER), "--config", config],
        capture_output=True,
        text=True,
        timeout=120,
        env=env,
        check=False,
    )
    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:") :])[config]
    sys.stderr.write(proc.stdout[-1000:] + "\n" + proc.stderr[-1000:] + "\n")
    msg = f"runner emitted no RESULTS for {config}"
    raise SystemExit(msg)


def _err(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(REPO_ROOT / "drift_demo"))
    args = ap.parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Running ref / baseline / absolute (wall-clock subprocesses)...")
    runs = {c: _run(c) for c in ("ref", "baseline", "absolute")}
    (out_dir / "drift_runs.json").write_text(json.dumps(runs))

    ref_end = runs["ref"]["end_gt"]
    base_err = _err(runs["baseline"]["end_gt"], ref_end)
    abs_err = _err(runs["absolute"]["end_gt"], ref_end)

    import matplotlib as mpl

    mpl.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 7))
    for cfg in ("ref", "baseline", "absolute"):
        color, label, ls = STYLE[cfg]
        traj = runs[cfg].get("traj") or []
        if traj:
            xs = [p[0] for p in traj]
            ys = [p[1] for p in traj]
            ax.plot(xs, ys, ls, color=color, lw=2.2, label=label, zorder=3)
            ax.plot(xs[-1], ys[-1], "o", color=color, ms=9, zorder=4)
    sx, sy = runs["ref"]["traj"][0][:2] if runs["ref"].get("traj") else (50, 50)
    ax.plot(sx, sy, "k*", ms=16, label="start", zorder=5)
    # target marker
    ax.plot(ref_end[0], ref_end[1], "P", color="#2ca02c", ms=14, zorder=5)

    ax.set_aspect("equal", "box")
    ax.set_xlabel("table x (cm)")
    ax.set_ylabel("table y (cm)")
    ax.set_title(
        "to_absolute() corrects injected odometry drift\n"
        f"baseline ends {base_err:.1f} cm off target   |   "
        f"to_absolute ends {abs_err:.1f} cm off target",
        fontsize=12,
        fontweight="bold",
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    out = out_dir / "drift_demo.png"
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"baseline drift = {base_err:.1f} cm   to_absolute error = {abs_err:.1f} cm")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
