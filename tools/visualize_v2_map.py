#!/usr/bin/env python3
"""Render a v2 multi-layer ftmap as ONE PANEL PER PLANE, old sim_run style.

Two stacked plots — Ebene 1 (Ground, z=0) and Ebene 2 (Ramp) — each drawn with
the rich sim_run_chain table style (filled oriented line/wall rectangles, robot
footprint ghosts). The robot trajectory is split by the plane the robot is on:
on each panel the path is drawn only while the robot rides THAT plane and breaks
(discontinues) the instant it switches away — so the two panels' paths are
complementary and together form the full run.

    .venv-test/bin/python tools/visualize_v2_map.py \
        --map .../2026-game-table-v2.ftmap --trajectory /tmp/v2_traj.json --out /tmp/v2_run.png
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))
# Reuse the old renderer's drawing helpers.
import sim_run_chain as SRC  # noqa: E402


def _flip_seg(s, h):
    """ftmap top-down → sim bottom-up; emit the {x0,y0,x1,y1,w,wall} dict
    sim_run_chain._draw_table expects."""
    return {
        "x0": s["startX"], "y0": h - s["startY"],
        "x1": s["endX"], "y1": h - s["endY"],
        "w": s.get("widthCm", 1.0), "wall": s.get("kind") == "wall",
    }


# Minimal robot geometry for the footprint ghosts (cube-bot defaults, cm).
_GEOM = {"length_cm": 26.0, "width_cm": 23.5, "rc_fwd": 0.0, "rc_strafe": 0.0,
         "wheels": [], "sensors": []}


def _draw_transitions(ax, transitions, h):
    for t in transitions:
        e = t.get("from")
        if not e:
            continue
        ax.plot([e["startX"], e["endX"]],
                [h - e["startY"], h - e["endY"]],
                "--", color="#e377c2", lw=2.2, alpha=0.95, zorder=7,
                label=f"transition {t.get('id','')}")


def _draw_layer_path(ax, traj, layer_idx, color):
    """Draw the trajectory ONLY where robot.layer == layer_idx; insert breaks
    (None) elsewhere so the polyline discontinues when on the other plane."""
    import numpy as np

    xs, ys = [], []
    for p in traj:
        if p.get("layer", 0) == layer_idx:
            xs.append(p["x"]); ys.append(p["y"])
        else:
            xs.append(np.nan); ys.append(np.nan)
    ax.plot(xs, ys, "-", color=color, lw=2.2, alpha=0.95, zorder=6,
            label="robot path (this plane)")
    # Robot footprint ghosts at sparse on-plane points.
    on = [p for p in traj if p.get("layer", 0) == layer_idx]
    if on:
        step = max(1, len(on) // 8)
        for k in range(0, len(on), step):
            p = on[k]
            import math
            SRC._draw_robot(ax, p["x"], p["y"], math.radians(p.get("theta", 0.0)),
                            _GEOM, alpha=0.4, body_color=color)
        # entry/exit onto this plane.
        ax.plot(on[0]["x"], on[0]["y"], "o", color="#2ca02c", ms=11, mec="black",
                mew=1.0, zorder=9, label="enters this plane")
        ax.plot(on[-1]["x"], on[-1]["y"], "s", color="#9467bd", ms=11, mec="black",
                mew=1.0, zorder=9, label="leaves this plane")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--trajectory", default=None)
    args = ap.parse_args()

    data = json.loads(Path(args.map).read_text())
    if data.get("version") != 2:
        raise SystemExit(f"{args.map} is not a v2 ftmap (version={data.get('version')})")
    tw = data["table"]["widthCm"]
    th = data["table"]["heightCm"]
    table = (tw, th)
    layers = data.get("layers", [])
    transitions = data.get("transitions", [])

    traj = []
    if args.trajectory:
        traj = json.loads(Path(args.trajectory).read_text()).get("traj", [])

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(layers)
    fig, axes = plt.subplots(n, 1, figsize=(15, 6.4 * n))
    if n == 1:
        axes = [axes]

    path_colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]
    for idx, (ax, layer) in enumerate(zip(axes, layers)):
        segs = [_flip_seg(s, th) for s in layer.get("lines", [])]
        SRC._draw_table(ax, segs, table)  # old filled-rectangle table style
        _draw_transitions(ax, transitions, th)
        if traj:
            _draw_layer_path(ax, traj, idx, path_colors[idx % len(path_colors)])
        lid = layer.get("id", f"layer{idx}")
        zcm = layer.get("zCm", 0)
        on_n = sum(1 for p in traj if p.get("layer", 0) == idx)
        ax.set_title(
            f"Ebene {idx + 1} — {lid}  (z={zcm} cm)"
            + (f"   ·   {on_n} trajectory pts on this plane" if traj else ""),
            fontsize=13, fontweight="bold",
        )
        # Dedup legend.
        hs, ls = ax.get_legend_handles_labels()
        seen, h2, l2 = set(), [], []
        for hh, ll in zip(hs, ls):
            if ll and ll not in seen:
                seen.add(ll); h2.append(hh); l2.append(ll)
        if h2:
            ax.legend(h2, l2, loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)

    fig.suptitle(
        f"v2 ftmap: {Path(args.map).name}  ({tw:.0f}×{th:.0f} cm) — one panel per plane; "
        "the path breaks when the robot switches plane and continues on the other",
        fontsize=12, fontweight="bold",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.99))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
