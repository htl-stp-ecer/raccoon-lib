#!/usr/bin/env python3
"""Small-multiples view of how ``optimize([...])`` transforms a mission's path —
one panel per pass option, same axes, so each pass's effect is obvious:

    base              optimize()            decompose + merge (always-on)
    cut_corners       .cut_corners(cm)      arcs replace right-angle corners
    splinify          .splinify()           whole path → one Catmull-Rom spline
    absolute_heading  .absolute_heading()   headings normalised to a reference
    time_optimal      .time_optimal()       speed profile (geometry unchanged) —
                                            green seam = flow-through, red = stop

Reuses the geometry tracer from visualize_optimizer (no duplicate forward
kinematics). One PNG per mission.

    RACCOON_SIM=1 .venv-test/bin/python tools/render_optimize_passes.py \
        --out /tmp/optpasses --cut-corners 8
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[0]
sys.path.insert(0, str(REPO))
import visualize_optimizer as V  # noqa: E402

M = 100.0  # m → cm


def _nodes(opt, lib):
    return lib.PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes


def _draw_items(ax, items, color, lw=2.0):
    drew = False
    for it in items:
        if it.pts and len(it.pts) >= 2:
            xs = [p[0] * M for p in it.pts]
            ys = [p[1] * M for p in it.pts]
            ax.plot(xs, ys, "-", color=color, lw=lw, alpha=0.9, ls=":" if it.preview else "-")
            drew = True
        if it.marker:
            ax.plot(it.marker[0] * M, it.marker[1] * M, "x", color=color, ms=5, alpha=0.6)
    return drew


def _seam_carry(seq, lib, preview_m):
    """Walk optimize().time_optimal() nodes; return (polyline_items, seam_pts).

    seam_pts = list of (x_cm, y_cm, carries) at each boundary between motion
    segments — carries True when the velocity profile flows through (entry>0)."""
    opt = lib.optimize(seq).time_optimal()
    nodes = _nodes(opt, lib)
    items = V.trace_relative(nodes, lib, preview_m)
    # Re-walk to pair each motion segment's end pose with its carry flag.
    p, env = V.Pose(), V.Env()
    seams, prev_end = [], None
    for node in nodes:
        if not isinstance(node, lib.Segment):
            continue
        item, p = V.advance_segment(p, node, lib, preview_m, env)
        if node.kind == "turn":
            prev_end = (p.x, p.y)
            continue
        if prev_end is not None:
            carries = (node.entry_speed_mps or 0.0) > 1e-6
            seams.append((prev_end[0] * M, prev_end[1] * M, carries))
        prev_end = (p.x, p.y)
    return items, seams


def render_mission(label, seq, lib, out_dir, preview_m, cut_cm):
    import matplotlib.pyplot as plt

    base = V.trace_relative(_nodes(lib.optimize(seq), lib), lib, preview_m)
    cut_items, cut_status = V.compile_cut_corners(seq, cut_cm, lib, preview_m)
    # Compact the verbose cut status to just the arc count for the panel title.
    n_arc = sum(
        1
        for n in _nodes(lib.optimize(seq).cut_corners(cut_cm), lib)
        if isinstance(n, lib.Segment) and n.kind == "arc"
    )
    cut_status = f"{n_arc} arc(s)" + ("" if n_arc else " — no eligible corners")
    ctrl, dense = V.spline_preview(base, lib)
    try:
        ah = V.trace_relative(_nodes(lib.optimize(seq).absolute_heading(), lib), lib, preview_m)
        ah_ok = True
    except Exception:
        ah, ah_ok = base, False
    to_items, seams = _seam_carry(seq, lib, preview_m)
    n_flow = sum(1 for *_, c in seams if c)

    panels = [
        ("optimize()  ·  decompose + merge", base, "#1f77b4", None),
        (f".cut_corners({cut_cm:.0f}cm)  ·  {cut_status}", cut_items, "#d62728", None),
        (".splinify()  ·  whole path → one spline", base, "#bbbbbb", dense),
        (f".absolute_heading()  ·  {'OK' if ah_ok else 'n/a'}", ah, "#9467bd", None),
        (f".time_optimal()  ·  {n_flow} flow-through seam(s)", to_items, "#e8731a", "seams"),
    ]

    # Shared bounds across all panels.
    allx, ally = [], []
    for _, items, _, _ in panels:
        for it in items:
            for x, y in it.pts:
                allx.append(x * M)
                ally.append(y * M)
    if dense:
        allx += [x * M for x, y in dense]
        ally += [y * M for x, y in dense]
    pad = 5
    xlim = (min(allx) - pad, max(allx) + pad) if allx else (-10, 50)
    ylim = (min(ally) - pad, max(ally) + pad) if ally else (-10, 50)

    fig, axes = plt.subplots(1, len(panels), figsize=(4.6 * len(panels), 5.0), squeeze=False)
    for ax, (title, items, color, extra) in zip(axes[0], panels, strict=False):
        # Ghost the base path for reference (light grey) in every panel.
        _draw_items(ax, base, "#dddddd", lw=1.2)
        if extra == "seams":
            _draw_items(ax, items, color)
            for x, y, carries in seams:
                ax.plot(
                    x,
                    y,
                    "o",
                    ms=8,
                    color="#2a9d3a" if carries else "#d62728",
                    mec="black",
                    mew=0.5,
                    zorder=5,
                )
        elif extra is not None:  # spline dense curve
            xs = [x * M for x, y in extra]
            ys = [y * M for x, y in extra]
            ax.plot(xs, ys, "-", color="#2a9d3a", lw=2.2)
        else:
            _draw_items(ax, items, color)
        ax.plot(0, 0, "ko", ms=7)
        ax.set_title(title, fontsize=8.5, fontweight="bold", pad=6)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, ls=":", alpha=0.3)
        ax.tick_params(labelsize=7)
    fig.suptitle(
        f"{label} — how optimize() builds the path, pass by pass "
        "(green seam = flow-through, red = full stop)",
        fontsize=12,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.01,
        "forward (cm) →   ·   up = left (cm)   ·   grey ghost = optimize() base path  "
        "·   dotted = unknown sensor endpoint",
        ha="center",
        fontsize=8,
        color="#555",
    )
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    out = out_dir / f"{label}.passes.png"
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"  wrote {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", default=V.DEFAULT_PROJECT)
    ap.add_argument("--mission", default=None, help="substring filter")
    ap.add_argument("--out", default="/tmp/optpasses")
    ap.add_argument("--cut-corners", type=float, default=8.0)
    ap.add_argument("--preview-cm", type=float, default=V.DEFAULT_PREVIEW_CM)
    args = ap.parse_args()

    import matplotlib as mpl

    mpl.use("Agg")

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    project = Path(args.project).resolve()
    sys.path.insert(0, str(project))
    os.chdir(project)
    lib = V.load_lib()
    preview_m = args.preview_cm / 100.0

    missions = V.discover_missions(lib)
    if args.mission:
        missions = [(n, c) for n, c in missions if args.mission.lower() in n.lower()]
    ok = 0
    for name, cls in missions:
        try:
            seq = cls().sequence()
        except Exception as e:
            print(f"- {name}\n  [skip] {type(e).__name__}: {e}")
            continue
        print(f"- {name}")
        try:
            render_mission(name, seq, lib, out_dir, preview_m, args.cut_corners)
            ok += 1
        except Exception as e:
            print(f"  [fail] {type(e).__name__}: {e}")
    print(f"\nDone: {ok}/{len(missions)} → {out_dir}")


if __name__ == "__main__":
    main()
