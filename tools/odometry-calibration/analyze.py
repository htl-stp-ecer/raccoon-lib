"""Analyze a drive_log.csv produced by odolog.py.

Detects the movement window from the internal odometry (which is reset to 0 at
the start of a motion step), then reports net displacement and path length for
both the internal estimate and the calibration-board ground truth.

Usage (anywhere, after fetching the CSV):
    python3 analyze.py [drive_log.csv]
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "drive_log.csv"
    rows = []
    with Path(path).open() as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})

    def internal_disp(r):
        return math.hypot(r["ix"], r["iy"])

    moving = [i for i, r in enumerate(rows) if internal_disp(r) > 0.005]  # >5 mm
    if not moving:
        print("no internal movement detected — did the robot drive?")
        return

    i0 = max(0, moving[0] - 5)
    i1 = min(len(rows) - 1, moving[-1] + 5)
    a, b = rows[i0], rows[i1]

    print(f"window t={a['t']:.1f}..{b['t']:.1f}s ({i1 - i0} samples)")
    print("--- INTERNAL STM32 (robot frame, reset at start; cm) ---")
    fwd, lat = (b["ix"] - a["ix"]) * 100, (b["iy"] - a["iy"]) * 100
    print(f"  forward = {fwd:.2f}  lateral = {lat:.2f}  straight-line = {math.hypot(fwd, lat):.2f}")
    print("--- CALIB BOARD / GROUND TRUTH (board frame; cm) ---")
    dcx, dcy = b["cx"] - a["cx"], b["cy"] - a["cy"]
    print(f"  dx = {dcx:.2f}  dy = {dcy:.2f}  straight-line = {math.hypot(dcx, dcy):.2f}")
    print(f"  heading change = {b['ch'] - a['ch']:.2f} deg")

    p_int = p_gt = 0.0
    for j in range(i0 + 1, i1 + 1):
        p_int += (
            math.hypot(rows[j]["ix"] - rows[j - 1]["ix"], rows[j]["iy"] - rows[j - 1]["iy"]) * 100
        )
        p_gt += math.hypot(rows[j]["cx"] - rows[j - 1]["cx"], rows[j]["cy"] - rows[j - 1]["cy"])
    print(f"--- path length: internal = {p_int:.2f} cm   ground-truth = {p_gt:.2f} cm")
    gt_sl = math.hypot(dcx, dcy)
    if gt_sl > 1e-6:
        print(f"--- scale error (internal/GT straight-line) = {math.hypot(fwd, lat) / gt_sl:.3f}x")


if __name__ == "__main__":
    main()
