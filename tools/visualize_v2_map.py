#!/usr/bin/env python3
"""Render a v2 multi-layer ftmap in the SIM frame, to check ramp alignment.

The sim works in a bottom-up frame (origin bottom-left, +Y up); the ftmap stores
Y top-down, so every line/transition is flipped on ingest. This tool applies the
SAME flip and draws each layer (ground vs ramp in distinct colours) + the
transition seams, overlaid with reference robot poses observed in the sim. That
makes it visually obvious whether the ramp footprint sits where the robot
actually drives — i.e. whether the authored map frame matches the sim frame.

    .venv-test/bin/python tools/visualize_v2_map.py \
        --map /path/to/2026-game-table-v2.ftmap --out /path/to/v2_map.png

Optionally overlay a trajectory JSON ({"traj":[{"x":..,"y":..},..]} from
sim_run_chain) with --trajectory, and extra "(x,y,label)" markers with --marker.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

# Reference robot poses seen in the FLAT-sim M070 trace (sim/internal frame, cm).
# These are where the robot actually drives today; the ramp lines must overlap
# these for M070's sensors to fire.
DEFAULT_MARKERS = [
    (42.8, 25.63, "START"),
    (218.5, 12.3, "M070 entry"),
    (210.7, 50.2, "M070 pre-grab"),
    (222.9, 50.6, "M070 stall (E wall)"),
]

LAYER_COLORS = {
    "ground": ("#444444", "#888888"),  # (line, wall)
    "ramp": ("#e8731a", "#b5651d"),
}
_FALLBACK = ("#1f77b4", "#7f7f7f")


def _flip_seg(s, h):
    """ftmap top-down (startY..) → sim bottom-up (h - startY)."""
    return (s["startX"], h - s["startY"], s["endX"], h - s["endY"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="v2 ftmap path")
    ap.add_argument("--out", required=True, help="output PNG path")
    ap.add_argument("--trajectory", default=None, help="optional sim_run trajectory.json")
    ap.add_argument("--marker", action="append", default=[], help="extra 'x,y,label' marker")
    args = ap.parse_args()

    data = json.loads(Path(args.map).read_text())
    if data.get("version") != 2:
        raise SystemExit(f"{args.map} is not a v2 ftmap (version={data.get('version')})")
    tw = data["table"]["widthCm"]
    th = data["table"]["heightCm"]

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(15, 7.0))
    # Table border.
    from matplotlib.patches import Rectangle

    ax.add_patch(Rectangle((0, 0), tw, th, fill=False, ec="#222", lw=1.5, zorder=1))

    # Layers.
    for layer in data.get("layers", []):
        lid = layer.get("id", "?")
        zcm = layer.get("zCm", 0)
        line_c, wall_c = LAYER_COLORS.get(lid, _FALLBACK)
        for s in layer.get("lines", []):
            x0, y0, x1, y1 = _flip_seg(s, th)
            is_wall = s.get("kind") == "wall"
            ax.plot(
                [x0, x1], [y0, y1],
                "-",
                color=wall_c if is_wall else line_c,
                lw=max(1.5, s.get("widthCm", 2) * 0.8),
                alpha=0.5 if is_wall else 0.9,
                solid_capstyle="round",
                zorder=3 if is_wall else 4,
                label=f"{lid} (z={zcm}cm)" if s is layer["lines"][0] else None,
            )

    # Transition seams.
    for t in data.get("transitions", []):
        for edge_key, mark in (("from", "o"), ("to", "x")):
            e = t.get(edge_key)
            if not e:
                continue
            x0, y0, x1, y1 = _flip_seg(e, th)
            ax.plot(
                [x0, x1], [y0, y1],
                "--", color="#d62728", lw=2.2, alpha=0.85, zorder=6,
                label=f"transition {t.get('id','')}" if edge_key == "from" else None,
            )

    # Optional trajectory overlay.
    if args.trajectory:
        traj = json.loads(Path(args.trajectory).read_text()).get("traj", [])
        if traj:
            ax.plot([p["x"] for p in traj], [p["y"] for p in traj],
                    "-", color="#1f77b4", lw=1.2, alpha=0.7, zorder=5, label="sim trajectory")

    # Reference markers (where the robot drives in the flat sim today).
    markers = list(DEFAULT_MARKERS)
    for m in args.marker:
        x, y, *lbl = m.split(",")
        markers.append((float(x), float(y), lbl[0] if lbl else ""))
    for x, y, lbl in markers:
        ax.plot(x, y, "*", color="#2ca02c", ms=15, mec="black", mew=0.6, zorder=8)
        ax.annotate(lbl, (x, y), fontsize=8, color="#145214", zorder=8,
                    xytext=(4, 4), textcoords="offset points")

    ax.set_aspect("equal")
    ax.set_xlim(-5, tw + 5)
    ax.set_ylim(-5, th + 5)
    ax.set_xlabel("table x (cm)  — sim/internal frame, +Y up")
    ax.set_ylabel("table y (cm)")
    ax.grid(True, ls=":", alpha=0.3)
    ax.set_title(
        f"v2 ftmap: {Path(args.map).name}  ({tw:.0f}×{th:.0f}cm)\n"
        "ground=grey  ramp=orange  transition seams=red dashed  "
        "green ★ = where the robot actually drives in sim",
        fontsize=11, fontweight="bold",
    )
    # Dedup legend.
    handles, labels = ax.get_legend_handles_labels()
    seen, h2, l2 = set(), [], []
    for hh, ll in zip(handles, labels):
        if ll and ll not in seen:
            seen.add(ll); h2.append(hh); l2.append(ll)
    ax.legend(h2, l2, loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
    fig.tight_layout()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
