#!/usr/bin/env python3
"""Plot accel/decel CSVs dumped by characterize_drive.cpp.

Set LIBSTP_AUTOTUNE_CSV=<dir> before running the runner, then:
    python3 tools/plot_autotune_csv.py <dir>

Produces a PNG per axis showing position + velocity over time.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib as mpl

mpl.use("Agg")
import matplotlib.pyplot as plt


def _load(path: Path) -> tuple[list[float], list[float], list[float]]:
    t, p, v = [], [], []
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            p.append(float(row["pos"]))
            v.append(float(row["vel"]))
    return t, p, v


def _plot_axis(csv_dir: Path, axis: str) -> None:
    accel = csv_dir / f"accel_{axis}.csv"
    decel = csv_dir / f"decel_{axis}.csv"
    if not accel.exists() or not decel.exists():
        print(f"skip {axis}: missing csv")
        return

    at, ap, av = _load(accel)
    dt, dp, dv = _load(decel)

    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex="col")
    fig.suptitle(f"axis = {axis}")

    axes[0, 0].plot(at, ap, "b-")
    axes[0, 0].set_title("accel: position")
    axes[0, 0].set_ylabel("pos")
    axes[0, 0].grid(True)

    axes[1, 0].plot(at, av, "g-")
    axes[1, 0].set_title("accel: velocity")
    axes[1, 0].set_xlabel("t [s]")
    axes[1, 0].set_ylabel("vel")
    axes[1, 0].grid(True)

    axes[0, 1].plot(dt, dp, "b-")
    axes[0, 1].set_title("decel: position")
    axes[0, 1].grid(True)

    axes[1, 1].plot(dt, dv, "g-")
    axes[1, 1].set_title("decel: velocity")
    axes[1, 1].set_xlabel("t [s]")
    axes[1, 1].grid(True)

    # Annotate peak and 90/10 thresholds on decel velocity.
    if dv:
        peak = max(abs(x) for x in dv)
        axes[1, 1].axhline(0.9 * peak, color="orange", linestyle=":", label="90%")
        axes[1, 1].axhline(0.1 * peak, color="red", linestyle=":", label="10%")
        axes[1, 1].legend(loc="upper right", fontsize=8)

    out = csv_dir / f"plot_{axis}.png"
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    plt.close(fig)
    print(f"wrote {out}")

    # Numeric summary.
    if av:
        print(f"  accel: pos {ap[0]:.4f} -> {ap[-1]:.4f}, v_peak={max(abs(x) for x in av):.4f}")
    if dv:
        print(f"  decel: pos {dp[0]:.4f} -> {dp[-1]:.4f}, v_peak={max(abs(x) for x in dv):.4f}")


def main() -> None:
    if len(sys.argv) < 2:
        print("usage: plot_autotune_csv.py <csv_dir>")
        sys.exit(1)
    csv_dir = Path(sys.argv[1])
    for axis in ("forward", "lateral", "angular"):
        _plot_axis(csv_dir, axis)


if __name__ == "__main__":
    main()
