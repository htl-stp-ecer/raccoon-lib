#!/usr/bin/env python3
"""Demonstrate cut_corners at COMPILE time: a corner becomes an arc.

Compiles ``drive_forward(50) + turn_right(90) + drive_forward(50)`` with and
without ``.cut_corners(R)`` and prints the node list, showing the sharp
``linear + turn + linear`` corner replaced by a smooth ``linear + arc + linear``
(legs trimmed by R, arc radius R/tan(θ/2)). Also shows that a concurrent
background arm move does not block the cut. No robot/sim needed.

    .venv-test/bin/python tools/cut_corners_demo.py
"""

from __future__ import annotations

import math
import os

os.environ.setdefault("RACCOON_SIM", "1")


def _fmt(nodes):
    from raccoon.step.motion.path.ir import Segment

    out = []
    for n in nodes:
        if isinstance(n, Segment):
            if n.kind == "arc":
                out.append(f"arc(R={n.radius_m * 100:.1f}cm,θ={math.degrees(n.arc_angle_rad):.0f}°)")
            elif n.kind == "turn":
                a = f"{math.degrees(n.angle_rad):.0f}°" if n.angle_rad is not None else "?"
                out.append(f"turn({a})")
            elif n.kind == "linear":
                d = f"{n.distance_m * 100:.0f}cm" if n.distance_m is not None else "until"
                out.append(f"drive({d})")
            else:
                out.append(n.kind)
        else:
            out.append("‖bg" if getattr(n, "is_background", False) else "‖inline")
    return "  →  ".join(out)


def main():
    from raccoon import background, drive_forward, optimize, turn_right, wait_for_seconds
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.passes import flatten_steps

    def compile_nodes(builder):
        opt = builder
        return PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes

    corner = [drive_forward(cm=50), turn_right(90), drive_forward(cm=50)]

    print("=== plain corner (merge only) ===")
    print("  " + _fmt(compile_nodes(optimize(list(corner)))))
    for R in (5.0, 10.0, 15.0):
        print(f"=== cut_corners({R:.0f}cm) ===")
        print("  " + _fmt(compile_nodes(optimize(list(corner)).cut_corners(R))))

    print("\n=== corner with a concurrent background arm move (see-through) ===")
    arm = [drive_forward(cm=50), background(wait_for_seconds(2.0)), turn_right(90), drive_forward(cm=50)]
    raw_nodes, _ = flatten_steps(arm)
    print("  raw:        " + _fmt(raw_nodes))
    print("  cut_corners: " + _fmt(compile_nodes(optimize(list(arm)).cut_corners(10.0))))


if __name__ == "__main__":
    main()
