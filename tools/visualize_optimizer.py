#!/usr/bin/env python3
"""Visualize the path-optimizer's *generated segments* for a raccoon project.

This is a diagnostic tool for the ``optimize([...])`` pipeline (lowering →
decompose → merge → optional ``to_absolute`` / ``splinify`` …).  For every
mission in a target project it lowers the step tree into the IR and draws three
overlaid traces in the robot's start-relative frame so you can eyeball them:

    * RELATIVE   (blue, solid)  — the IR segments integrated geometrically
                                  exactly as ``segments_to_spline_waypoints`` /
                                  the executor would dead-reckon them.
    * ABSOLUTE   (orange, dashed) — the path reconstructed from what
                                  ``.to_absolute()`` emits (``GotoWaypoints`` /
                                  ``AbsoluteHoldMove`` world targets).  If the
                                  pass is correct this lies exactly on top of
                                  the relative trace — that's the "do the
                                  absolute things look like the relatives?"
                                  check.
    * SPLINE     (green)        — the centripetal Catmull-Rom curve through the
                                  translation waypoints (the SAME algorithm the
                                  C++ ``SplineMotion`` follows), so you can see
                                  whether the spline is "proper".

Frame convention (matches the library):
    +x = robot's initial forward, +y = left (90° CCW), heading CCW-positive,
    origin (0, 0) = pose at path start.  Plotted in centimetres.

Because cube-bot missions are heavily sensor-/condition-driven, unknown
endpoints are previewed with a fixed stub length (dashed, ``--preview-cm``) and
``to_absolute`` / ``splinify`` are best-effort: when a path can't be converted
the reason is printed on the figure instead of crashing.

Run it with an interpreter that has a working ``raccoon`` import plus numpy +
matplotlib.  On this machine that is the raccoon-lib mock test venv::

    .venv-test/bin/python tools/visualize_optimizer.py
    .venv-test/bin/python tools/visualize_optimizer.py --mission m007 --show
    .venv-test/bin/python tools/visualize_optimizer.py --demo
    .venv-test/bin/python tools/visualize_optimizer.py --list

Outputs PNGs (+ a .txt explain() dump) into ``--out`` (default ./optimizer_viz).
"""

from __future__ import annotations

import argparse
import importlib
import inspect
import math
import os
import pkgutil
import sys
from dataclasses import dataclass
from pathlib import Path

DEFAULT_PROJECT = "/media/tobias/TobiasSSD/projects/Botball/competition/Ecer2026/cube-bot"
DEFAULT_PREVIEW_CM = 25.0  # stub length drawn for unknown (sensor) endpoints

M = 100.0  # metres → centimetres for plotting


# ---------------------------------------------------------------------------
# Lazy raccoon imports (resolved after the project is on sys.path)
# ---------------------------------------------------------------------------


@dataclass
class Lib:
    """Bundle of the raccoon symbols the tool needs."""

    optimize: object
    PathCompiler: object
    Segment: type
    SideAction: type
    LinearAxis: object
    arc_samples: object
    seg_to_spline_wps: object
    sample_catmull: object
    GotoWaypoints: type
    AbsoluteHoldMove: type
    SplineFollow: type | None
    Mission: type


def load_lib() -> Lib:
    from raccoon.mission.api import Mission
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.goto import AbsoluteHoldMove, GotoWaypoints

    try:
        from raccoon.step.motion.goto import SplineFollow
    except Exception:  # pragma: no cover - older builds
        SplineFollow = None
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes.spline import (
        _arc_samples,
        sample_centripetal_catmull_rom,
        segments_to_spline_waypoints,
    )

    try:
        from raccoon.step.motion import optimize
    except Exception:
        from raccoon.step.motion.path.optimize import optimize

    return Lib(
        optimize=optimize,
        PathCompiler=PathCompiler,
        Segment=Segment,
        SideAction=SideAction,
        LinearAxis=LinearAxis,
        arc_samples=_arc_samples,
        seg_to_spline_wps=segments_to_spline_waypoints,
        sample_catmull=sample_centripetal_catmull_rom,
        GotoWaypoints=GotoWaypoints,
        AbsoluteHoldMove=AbsoluteHoldMove,
        SplineFollow=SplineFollow,
        Mission=Mission,
    )


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


@dataclass
class Pose:
    x: float = 0.0  # metres, +forward(initial)
    y: float = 0.0  # metres, +left
    h: float = 0.0  # radians, CCW+


@dataclass
class Env:
    """Active heading reference from the most recent mark_heading_reference().

    ``heading=`` on a drive / ``turn_to_heading`` target is expressed relative
    to this reference, NOT the start frame: ``θ = o_rad + sign·radians(deg)``
    (mirrors ``_abs_heading_rad`` in the to_absolute pass).  Before any mark,
    ``o_rad=0`` / ``sign=+1`` so degrees are taken in the start frame.
    """

    o_rad: float = 0.0
    sign: float = 1.0
    marked: bool = False

    def abs_heading(self, deg: float) -> float:
        return self.o_rad + self.sign * math.radians(deg)


def _maybe_mark(step, env: Env) -> None:
    """If ``step`` is a MarkHeadingReference, update ``env`` in place."""
    if type(step).__name__ != "MarkHeadingReference":
        return
    env.o_rad = math.radians(getattr(step, "_origin_offset_deg", 0.0) or 0.0)
    env.sign = 1.0 if getattr(step, "_positive_direction", "left") == "left" else -1.0
    env.marked = True


@dataclass
class TraceItem:
    """One drawable piece of a path (a segment or a side-effect marker)."""

    kind: str  # linear/turn/arc/diagonal/follow_line/spline/side/deferred/goto/hold
    pts: list[tuple[float, float]]  # polyline in metres (may be empty)
    label: str = ""
    preview: bool = False  # endpoint was unknown → stub-drawn
    marker: tuple[float, float] | None = None  # point marker (side actions / turns)
    heading_end: float | None = None  # heading after this item (for an arrow)


def _linear_points(p: Pose, axis, d: float, lib: Lib, n: int = 12, travel_h: float | None = None):
    """Sample a linear leg from ``p`` along ``axis`` for signed distance ``d``.

    ``travel_h`` overrides the heading used for the travel direction (used for
    heading-holding drives, which travel along their held heading regardless of
    the running pose heading); defaults to ``p.h``.
    """
    h = p.h if travel_h is None else travel_h
    pts = [(p.x, p.y)]
    for i in range(1, n + 1):
        f = i / n
        if axis == lib.LinearAxis.Forward:
            x = p.x + (d * f) * math.cos(h)
            y = p.y + (d * f) * math.sin(h)
        else:  # Lateral, +left
            x = p.x + (d * f) * (-math.sin(h))
            y = p.y + (d * f) * math.cos(h)
        pts.append((x, y))
    return pts


def advance_segment(p: Pose, seg, lib: Lib, preview_m: float, env: Env) -> tuple[TraceItem, Pose]:
    """Integrate one IR ``Segment`` from pose ``p``; return its draw item + new pose."""
    k = seg.kind
    if k in ("linear", "follow_line"):
        known = seg.has_known_endpoint and seg.distance_m is not None
        d = seg.distance_m if known else (seg.sign or 1.0) * preview_m
        axis = seg.axis or lib.LinearAxis.Forward
        # A heading-holding drive (heading=…) travels along its held heading
        # (relative to the active heading reference), not the running pose
        # heading — mirror what the to_absolute pass does.
        travel_h = env.abs_heading(seg.heading_deg) if seg.heading_deg is not None else None
        pts = _linear_points(p, axis, d, lib, travel_h=travel_h)
        end_h = travel_h if travel_h is not None else p.h
        end = Pose(pts[-1][0], pts[-1][1], end_h)
        label = f"{k} {abs(d) * M:.0f}cm" + ("" if known else " ·until")
        if seg.heading_deg is not None:
            label += f" h{seg.heading_deg:.0f}°"
        if seg.condition is not None and known:
            label += " (cond)"
        return TraceItem(k, pts, label, preview=not known, heading_end=end_h), end

    if k == "diagonal":
        fwd = seg.forward_m or 0.0
        left = seg.left_m or 0.0
        ex = p.x + fwd * math.cos(p.h) - left * math.sin(p.h)
        ey = p.y + fwd * math.sin(p.h) + left * math.cos(p.h)
        end = Pose(ex, ey, p.h)
        return (
            TraceItem(
                "diagonal",
                [(p.x, p.y), (ex, ey)],
                f"diag f{fwd*M:.0f} l{left*M:.0f}",
                heading_end=p.h,
            ),
            end,
        )

    if k == "arc":
        samples, ex, ey, eh = lib.arc_samples(
            p.x, p.y, p.h, seg.radius_m or 0.0, seg.arc_angle_rad or 0.0, seg.lateral
        )
        pts = [(p.x, p.y)] + list(samples)
        deg = math.degrees(seg.arc_angle_rad or 0.0)
        return (
            TraceItem("arc", pts, f"arc r{(seg.radius_m or 0)*M:.0f} {deg:+.0f}°", heading_end=eh),
            Pose(ex, ey, eh),
        )

    if k == "turn":
        target_deg = getattr(seg.opaque_step, "_target_deg", None) if seg.opaque_step else None
        if target_deg is not None:  # turn_to_heading → absolute reference target
            eh = env.abs_heading(target_deg)
            label = f"turn→{target_deg:.0f}° (ref)"
            preview = False
        elif seg.angle_rad is not None:  # plain relative turn
            eh = p.h + seg.angle_rad
            label = f"turn {math.degrees(seg.angle_rad):+.0f}°"
            preview = False
        else:  # condition turn — final heading unknown
            eh = p.h
            label = "turn ·until"
            preview = True
        return (
            TraceItem("turn", [], label, preview=preview, marker=(p.x, p.y), heading_end=eh),
            Pose(p.x, p.y, eh),
        )

    if k == "spline":
        return TraceItem("spline", [], "spline(opaque)", marker=(p.x, p.y), heading_end=p.h), p

    # Unknown kind — leave pose unchanged.
    return TraceItem(k, [], k, marker=(p.x, p.y), heading_end=p.h), p


def trace_relative(nodes, lib: Lib, preview_m: float) -> list[TraceItem]:
    """Walk the lowered IR node list and produce drawable items (relative frame)."""
    p = Pose()
    env = Env()
    items: list[TraceItem] = []
    for node in nodes:
        if node is None:
            items.append(TraceItem("deferred", [], "deferred(?)", marker=(p.x, p.y)))
            continue
        if isinstance(node, lib.SideAction):
            _maybe_mark(node.step, env)
            mode = "bg" if node.is_background else "inline"
            items.append(
                TraceItem("side", [], f"{type(node.step).__name__} [{mode}]", marker=(p.x, p.y))
            )
            continue
        if isinstance(node, lib.Segment):
            item, p = advance_segment(p, node, lib, preview_m, env)
            items.append(item)
    return items


class _Anchor:
    """Minimal pose object exposing ``.position`` / ``.heading`` for the helper."""

    def __init__(self, p: Pose):
        self.position = (p.x, p.y)
        self.heading = p.h


def trace_absolute(abs_nodes, lib: Lib, preview_m: float) -> list[TraceItem]:
    """Reconstruct the path from the post-``to_absolute`` node list."""
    from raccoon.step.motion.goto import _waypoints_to_world_targets

    p = Pose()
    env = Env()
    items: list[TraceItem] = []
    for node in abs_nodes:
        if node is None:
            items.append(TraceItem("deferred", [], "deferred", marker=(p.x, p.y)))
            continue
        if isinstance(node, lib.SideAction):
            step = node.step
            _maybe_mark(step, env)
            if isinstance(step, lib.GotoWaypoints):
                targets = _waypoints_to_world_targets(_Anchor(p), step._waypoints)
                pts = [(p.x, p.y)] + [(tx, ty) for (tx, ty, _th) in targets]
                tx, ty, th = targets[-1]
                p = Pose(tx, ty, th)
                items.append(
                    TraceItem("goto", pts, f"GotoWaypoints n={len(targets)}", heading_end=th)
                )
                continue
            if isinstance(step, lib.AbsoluteHoldMove):
                axis = step._free_axis
                d = (step._sign or 1.0) * preview_m
                pts = _linear_points(p, axis, d, lib)
                p = Pose(pts[-1][0], pts[-1][1], p.h)
                items.append(
                    TraceItem("hold", pts, "AbsoluteHoldMove ·until", preview=True, heading_end=p.h)
                )
                continue
            if lib.SplineFollow is not None and isinstance(step, lib.SplineFollow):
                a = _Anchor(p)
                ctrl = [
                    (
                        a.position[0] + fwd * math.cos(a.heading) - left * math.sin(a.heading),
                        a.position[1] + fwd * math.sin(a.heading) + left * math.cos(a.heading),
                    )
                    for (fwd, left) in step._waypoints
                ]
                pts = [(p.x, p.y)] + ctrl
                if ctrl:
                    p = Pose(ctrl[-1][0], ctrl[-1][1], p.h)
                items.append(TraceItem("spline", pts, "SplineFollow", heading_end=p.h))
                continue
            mode = "bg" if node.is_background else "inline"
            items.append(
                TraceItem("side", [], f"{type(step).__name__} [{mode}]", marker=(p.x, p.y))
            )
            continue
        if isinstance(node, lib.Segment):
            item, p = advance_segment(p, node, lib, preview_m, env)
            items.append(item)
    return items


def spline_preview(items: list[TraceItem], lib: Lib):
    """Build centripetal Catmull-Rom control points + dense samples from a trace.

    Control points = the endpoint of every translation item (linear / diagonal /
    follow_line) plus all arc sample points.  Turns / side actions contribute no
    waypoint — the curve rounds them, exactly like ``segments_to_spline_waypoints``.
    Returns ``(control_pts_m, dense_pts_m)`` or ``(None, None)`` if < 2 waypoints.
    """
    ctrl: list[tuple[float, float]] = []
    for it in items:
        if it.kind in ("linear", "follow_line", "diagonal") and it.pts:
            ctrl.append(it.pts[-1])
        elif it.kind == "arc":
            ctrl.extend(it.pts[1:])  # skip the shared start point
    if len(ctrl) < 2:
        return None, None
    dense = lib.sample_catmull(ctrl, 0.03)
    return ctrl, dense


# ---------------------------------------------------------------------------
# Optimizer status (best-effort to_absolute / splinify compile)
# ---------------------------------------------------------------------------


def _short(msg: str, n: int = 84) -> str:
    """One-line, length-capped status for the on-figure info box."""
    msg = " ".join(str(msg).split())
    return msg if len(msg) <= n else msg[: n - 1] + "…"


def compile_absolute(seq, lib: Lib):
    """Return (abs_nodes, status_str). Never raises."""
    try:
        opt = lib.optimize(seq).to_absolute()
        plan = lib.PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
        n_goto = sum(
            1
            for n in plan.nodes
            if isinstance(n, lib.SideAction) and isinstance(n.step, lib.GotoWaypoints)
        )
        n_hold = sum(
            1
            for n in plan.nodes
            if isinstance(n, lib.SideAction) and isinstance(n.step, lib.AbsoluteHoldMove)
        )
        return plan.nodes, f"OK ({n_goto} GotoWaypoints, {n_hold} AbsoluteHoldMove)"
    except Exception as e:
        return None, _short(f"{type(e).__name__}: {e}")


def compile_cut_corners(seq, radius_cm: float, lib: Lib, preview_m: float):
    """Compile ``optimize(seq).cut_corners(radius_cm)`` → (items, status_str).

    Never raises.  Returns the traced node list (arcs included) plus a one-line
    summary of how many corners were turned into arcs.
    """
    try:
        opt = lib.optimize(seq).cut_corners(radius_cm)
        plan = lib.PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
        items = trace_relative(plan.nodes, lib, preview_m)
        n_arc = sum(1 for n in plan.nodes if isinstance(n, lib.Segment) and n.kind == "arc")
        n_turn = sum(1 for n in plan.nodes if isinstance(n, lib.Segment) and n.kind == "turn")
        status = f"OK (cut {radius_cm:.0f}cm → {n_arc} arc(s), {n_turn} turn(s) left)"
        if n_arc == 0:
            status += " — no eligible linear+turn+linear corners"
        return items, status
    except Exception as e:
        return None, _short(f"{type(e).__name__}: {e}")


# ---------------------------------------------------------------------------
# IR → DSL code generation (the inverse of lowering)
# ---------------------------------------------------------------------------
#
# Emits readable step-DSL source from a compiled IR node list so the optimizer
# output can be verified code-wise alongside explain().  Fidelity:
#   * motion geometry (linear/turn/arc/diagonal/strafe) → exact DSL calls;
#   * conditions → exact for numeric (after_cm…), placeholder sensor refs;
#   * side actions / GotoWaypoints / opaque steps → commented signature (their
#     constructor source isn't recoverable from the instance).


def _num(v: float) -> str:
    """Format a number compactly: ints without a trailing .0."""
    r = round(v, 2)
    return str(int(r)) if abs(r - round(r)) < 1e-9 else f"{r:g}"


def _cond_src(cond, lib: Lib, depth: int = 0) -> str:
    """Best-effort reconstruction of a StopCondition as DSL source."""
    if cond is None:
        return ""
    cls = type(cond).__name__

    # Numeric distance/time/angle conditions — read the stored target.
    if cls in ("after_cm", "after_forward_cm", "after_lateral_cm"):
        m = getattr(cond, "_target_m", None)
        s = f"{cls}({_num(m * 100)})" if m is not None else f"{cls}(?)"
        if getattr(cond, "_absolute", False):
            s += "  # absolute"
        return s
    if cls == "after_degrees":
        r = getattr(cond, "_target_rad", getattr(cond, "_target_deg", None))
        if r is not None:
            deg = math.degrees(r) if abs(r) < 7 else r  # heuristic rad vs deg
            return f"after_degrees({_num(deg)})"
    if cls == "after_seconds":
        s = getattr(cond, "_target_s", getattr(cond, "_seconds", None))
        return f"after_seconds({_num(s)})" if s is not None else "after_seconds(?)"

    # Sensor conditions — type is known, the sensor variable is not.
    if cls in (
        "on_black",
        "on_white",
        "on_digital",
        "on_analog_above",
        "on_analog_below",
        "over_line",
        "stall_detected",
    ):
        needs_sensor = cls not in ("over_line", "stall_detected")
        return f"{cls}(<sensor>)" if needs_sensor else f"{cls}()"

    # Combinators: a + b (sequential) or a | b (any-of). Find child conditions.
    children = []
    for attr in ("_conditions", "_conds", "_parts"):
        seq = getattr(cond, attr, None)
        if isinstance(seq, (list, tuple)) and seq:
            children = list(seq)
            break
    if not children:
        pair = [
            getattr(cond, a, None) for a in ("_left", "_right", "_a", "_b", "_first", "_second")
        ]
        children = [c for c in pair if c is not None]
    if children:
        joiner = " | " if "Any" in cls or "Or" in cls else " + "
        return joiner.join(_cond_src(c, lib, depth + 1) for c in children)

    return f"<cond:{cls}>"


def _until(seg, lib: Lib) -> str:
    if seg.condition is None:
        return ""
    return f".until({_cond_src(seg.condition, lib)})"


def _call(fac: str, *args: str) -> str:
    """Assemble ``fac(a, b, …)`` dropping empty argument fragments."""
    return f"{fac}({', '.join(a for a in args if a)})"


def _seg_src(seg, lib: Lib) -> str:
    """Map one geometry-bearing Segment back to a DSL factory call."""
    k = seg.kind
    speed = "" if abs(seg.speed_scale - 1.0) < 1e-9 else f"speed={_num(seg.speed_scale)}"
    head = "" if seg.heading_deg is None else f"heading={_num(seg.heading_deg)}"

    if k in ("linear", "follow_line"):
        d = seg.distance_m
        cm = "" if (d is None or not seg.has_known_endpoint) else _num(abs(d) * 100)
        if k == "follow_line":
            sig = seg.opaque_step._generate_signature() if seg.opaque_step else "follow_line(...)"
            return f"follow_line(...){_until(seg, lib)}   # {sig}"
        if seg.axis == lib.LinearAxis.Lateral:
            fac = "strafe_left" if (d is None or d >= 0) else "strafe_right"
        else:
            fac = "drive_forward" if (d is None or d >= 0) else "drive_backward"
        return _call(fac, cm, head, speed) + _until(seg, lib)

    if k == "turn":
        target = getattr(seg.opaque_step, "_target_deg", None) if seg.opaque_step else None
        if target is not None:
            fd = getattr(seg.opaque_step, "_force_direction", None)
            side = fd if fd in ("left", "right") else ("left" if (seg.sign or 1) >= 0 else "right")
            return _call(f"turn_to_heading_{side}", _num(target)) + _until(seg, lib)
        if seg.angle_rad is not None:
            deg = math.degrees(seg.angle_rad)
            fac = "turn_left" if seg.angle_rad >= 0 else "turn_right"
            return _call(fac, _num(abs(deg)), speed) + _until(seg, lib)
        fac = "turn_left" if (seg.sign or 1) >= 0 else "turn_right"
        return _call(fac) + _until(seg, lib)

    if k == "arc":
        ang = seg.arc_angle_rad or 0.0
        r_cm = _num((seg.radius_m or 0.0) * 100)
        deg = _num(abs(math.degrees(ang)))
        if seg.lateral:
            fac = "strafe_arc_left" if ang >= 0 else "strafe_arc_right"
        else:
            fac = "drive_arc_left" if ang >= 0 else "drive_arc_right"
        return _call(fac, f"radius_cm={r_cm}", f"degrees={deg}", speed)

    if k == "diagonal":
        fwd = seg.forward_m or 0.0
        left = seg.left_m or 0.0
        angle_deg = _num(math.degrees(math.atan2(left, fwd)))
        cm = _num(math.hypot(fwd, left) * 100)
        return _call("drive_angle", f"angle_deg={angle_deg}", f"cm={cm}", head, speed) + _until(
            seg, lib
        )

    if k == "spline":
        wps = getattr(seg.opaque_step, "_waypoints", None)
        if wps:
            pts = ", ".join(f"({_num(a)}, {_num(b)})" for a, b in (w[:2] for w in wps))
            return f"spline({pts})"
        return "spline(...)"

    return f"<segment:{k}>"


def _side_src(step, is_bg: bool, lib: Lib) -> str:
    """Render a side-action step — pass outputs get readable pseudo-calls."""
    if isinstance(step, lib.GotoWaypoints):
        legs = []
        for rf, rl, adx, ady, kind, hv in step._waypoints:
            fwd, left = (rf + adx) * 100, (rl + ady) * 100
            tag = "/abs" if kind == "abs" else ""
            legs.append(f"({_num(fwd)}, {_num(left)}, {_num(math.degrees(hv))}°{tag})")
        sp = "" if abs(step._speed - 1.0) < 1e-9 else f", speed={_num(step._speed)}"
        return (
            f"goto_waypoints([{', '.join(legs)}]{sp}),"
            f"  # closed-loop nav, {len(legs)} absolute target(s)"
        )
    if lib.SplineFollow is not None and isinstance(step, lib.SplineFollow):
        pts = ", ".join(f"({_num(f * 100)}, {_num(l * 100)})" for f, l in step._waypoints)
        return f"spline_follow([{pts}], speed={_num(step._speed)}),  # absolute pure-pursuit spline"
    if isinstance(step, lib.AbsoluteHoldMove):
        axis = "Forward" if step._free_axis == lib.LinearAxis.Forward else "Lateral"
        u = _until_obj(step._condition, lib) if getattr(step, "_condition", None) else "..."
        return f"absolute_hold_move(free={axis}, sign={_num(step._sign)}).until({u}),  # 2 DOF held abs"
    sig = (
        step._generate_signature() if hasattr(step, "_generate_signature") else type(step).__name__
    )
    mode = "background" if is_bg else "inline side-action"
    return f"# ⟨{mode}⟩ {sig}"


def _until_obj(cond, lib: Lib) -> str:
    return _cond_src(cond, lib)


def nodes_to_dsl(nodes, lib: Lib, call: str = "optimize", suffix: str = "") -> str:
    """Render a compiled IR node list as a step-DSL block."""
    lines: list[str] = []
    for node in nodes:
        if node is None:
            lines.append("    ...,  # ⟨deferred⟩ resolved at runtime")
            continue
        if isinstance(node, lib.SideAction):
            lines.append(f"    {_side_src(node.step, node.is_background, lib)}")
            continue
        if isinstance(node, lib.Segment):
            lines.append(f"    {_seg_src(node, lib)},")
    body = "\n".join(lines) if lines else "    # (empty)"
    return f"{call}([\n{body}\n]){suffix}"


def combo_codegen_text(seq, lib: Lib, cut_cm: float = 8.0) -> str:
    """The death test: generate DSL for every pass combination on one path.

    Compiles the path through cut_corners / absolute_heading / to_absolute /
    splinify and their combinations, rendering each result back to step-DSL so
    you can verify every transform code-wise. Combos that the optimizer rejects
    (composition rules) are shown with their error.
    """
    c = _num(cut_cm)
    combos = [
        ("optimize([...])", lambda: lib.optimize(seq)),
        (f"...).cut_corners({c})", lambda: lib.optimize(seq).cut_corners(cut_cm)),
        ("...).absolute_heading()", lambda: lib.optimize(seq).absolute_heading()),
        ("...).to_absolute()", lambda: lib.optimize(seq).to_absolute()),
        (
            f"...).cut_corners({c}).to_absolute()",
            lambda: lib.optimize(seq).cut_corners(cut_cm).to_absolute(),
        ),
        (
            "...).absolute_heading().to_absolute()",
            lambda: lib.optimize(seq).absolute_heading().to_absolute(),
        ),
        (
            f"...).cut_corners({c}).absolute_heading().to_absolute()",
            lambda: lib.optimize(seq).cut_corners(cut_cm).absolute_heading().to_absolute(),
        ),
        ("...).splinify()", lambda: lib.optimize(seq).splinify()),
        (
            f"...).cut_corners({c}).splinify()",
            lambda: lib.optimize(seq).cut_corners(cut_cm).splinify(),
        ),
        (
            "...).to_absolute().splinify()  # whole path → ONE absolute spline",
            lambda: lib.optimize(seq).to_absolute().splinify(),
        ),
    ]

    out = [
        "# ============================================================",
        "# DEATH TEST — every optimizer pass combination, as step-DSL.",
        "# Motion exact; GotoWaypoints/SplineFollow/AbsoluteHoldMove shown as",
        "# readable pseudo-calls; conditions best-effort. NOT executable as-is.",
        "# ============================================================",
    ]
    for label, build in combos:
        out.append(f"\n# === {label} ===")
        try:
            opt = build()
            plan = lib.PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
            out.append(nodes_to_dsl(plan.nodes, lib, call="optimize"))
        except Exception as e:
            out.append(f"#   ✗ rejected: {type(e).__name__}: {e}")
    return "\n".join(out) + "\n"


def combo_sequence(lib: Lib):
    """A clean pure-motion path (no side actions) so all combos are valid."""
    from raccoon.step.motion import drive_forward, turn_left, turn_right

    return [
        drive_forward(60),
        turn_right(90),
        drive_forward(45),
        turn_left(90),
        drive_forward(40),
        turn_left(90),
        drive_forward(30),
    ]


def codegen_text(seq, lib: Lib, cut_cm: float | None = None) -> str:
    """Produce DSL source for the raw lowering and key optimizer pass stages."""
    from raccoon.step.motion.path.passes import flatten_steps

    raw = seq if isinstance(seq, list) else [seq]
    sections: list[str] = [
        "# Generated step-DSL reconstructed from the optimizer IR.",
        "# Motion geometry is exact; conditions best-effort (<sensor> = unrecoverable",
        "# sensor ref); side-actions/opaque steps shown as comments. NOT executable as-is.",
        "",
    ]

    nodes, _ = flatten_steps(raw)
    sections.append("# === raw lowering (seq, no passes) ===")
    sections.append(nodes_to_dsl(nodes, lib, call="seq"))

    try:
        opt = lib.optimize(seq)  # decompose + merge always-on
        plan = lib.PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
        sections.append("\n# === after optimize() [decompose + merge] ===")
        sections.append(nodes_to_dsl(plan.nodes, lib, call="optimize"))
    except Exception as e:
        sections.append(f"\n# optimize() failed: {type(e).__name__}: {e}")

    if cut_cm is not None:
        try:
            opt = lib.optimize(seq).cut_corners(cut_cm)
            plan = lib.PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
            sections.append(f"\n# === after .cut_corners({_num(cut_cm)}) ===")
            sections.append(
                nodes_to_dsl(plan.nodes, lib, call="optimize")
                + f"  # ...).cut_corners({_num(cut_cm)})"
            )
        except Exception as e:
            sections.append(f"\n# cut_corners failed: {type(e).__name__}: {e}")

    return "\n".join(sections) + "\n"


def splinify_status(seq, lib: Lib) -> str:
    """Whether ``.splinify()`` would compile this path (the strict library check)."""
    try:
        opt = lib.optimize(seq).splinify()
        lib.PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
        return "OK (whole path → one spline)"
    except Exception as e:
        return _short(f"{type(e).__name__}: {e}")


def explain_text(seq, lib: Lib) -> str:
    try:
        return lib.optimize(seq).explain()
    except Exception as e:
        return f"explain() failed: {type(e).__name__}: {e}"


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

_REL_COLOR = "#1f77b4"
_ABS_COLOR = "#ff7f0e"
_SPL_COLOR = "#2ca02c"
_CUT_COLOR = "#9467bd"


def _arrow(ax, x, y, h, color, scale=4.0):
    ax.annotate(
        "",
        xy=(x + scale * math.cos(h), y + scale * math.sin(h)),
        xytext=(x, y),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.2, alpha=0.8),
    )


def plot_with_spline(
    title,
    rel_items,
    abs_items,
    abs_status,
    spl_status,
    summary,
    ctrl,
    dense,
    out_png,
    show,
    cut_items=None,
    cut_status=None,
):
    """Variant that also draws the Catmull-Rom spline overlay."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(13, 7))

    first_seg = True
    for it in rel_items:
        if it.pts:
            xs = [x * M for x, _ in it.pts]
            ys = [y * M for _, y in it.pts]
            ls = "--" if it.preview else "-"
            ax.plot(
                xs,
                ys,
                ls,
                color=_REL_COLOR,
                lw=2.2,
                alpha=0.9,
                label="relative" if first_seg else None,
            )
            first_seg = False
            ax.plot(xs[-1], ys[-1], "o", color=_REL_COLOR, ms=3.5)
            if it.heading_end is not None:
                _arrow(ax, xs[-1], ys[-1], it.heading_end, _REL_COLOR)
            mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
            ax.annotate(
                it.label,
                (mx, my),
                fontsize=6.5,
                color=_REL_COLOR,
                alpha=0.9,
                ha="center",
                va="bottom",
            )
        elif it.marker is not None:
            mx, my = it.marker[0] * M, it.marker[1] * M
            if it.kind == "turn":
                ax.plot(mx, my, "^", color=_REL_COLOR, ms=7, alpha=0.7)
                if it.heading_end is not None:
                    _arrow(ax, mx, my, it.heading_end, _REL_COLOR)
            elif it.kind in ("side", "deferred"):
                ax.plot(mx, my, "s", color="#888888", ms=6, alpha=0.7)
                ax.annotate(
                    it.label,
                    (mx, my),
                    fontsize=6,
                    color="#555555",
                    ha="left",
                    va="top",
                    rotation=15,
                )
            else:
                ax.plot(mx, my, "*", color=_REL_COLOR, ms=8, alpha=0.7)
                ax.annotate(it.label, (mx, my), fontsize=6, color=_REL_COLOR)

    if abs_items:
        first = True
        for it in abs_items:
            if it.kind in ("goto", "hold", "spline") and it.pts:
                xs = [x * M for x, _ in it.pts]
                ys = [y * M for _, y in it.pts]
                ax.plot(
                    xs,
                    ys,
                    "--",
                    color=_ABS_COLOR,
                    lw=1.8,
                    alpha=0.85,
                    label="absolute (to_absolute)" if first else None,
                )
                first = False
                ax.plot(xs[1:], ys[1:], "x", color=_ABS_COLOR, ms=5, alpha=0.85)

    if ctrl and dense:
        dx = [x * M for x, _ in dense]
        dy = [y * M for _, y in dense]
        ax.plot(dx, dy, "-", color=_SPL_COLOR, lw=1.6, alpha=0.85, label="catmull-rom spline")
        cx = [x * M for x, _ in ctrl]
        cy = [y * M for _, y in ctrl]
        ax.plot(
            cx, cy, "o", color=_SPL_COLOR, ms=4, alpha=0.7, mfc="none", label="spline control pts"
        )

    if cut_items:
        first = True
        for it in cut_items:
            if not it.pts:
                continue
            xs = [x * M for x, _ in it.pts]
            ys = [y * M for _, y in it.pts]
            lbl = "cut_corners" if first else None
            first = False
            if it.kind == "arc":
                ax.plot(xs, ys, "-", color=_CUT_COLOR, lw=3.0, alpha=0.95, label=lbl)
                mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
                ax.annotate(
                    it.label, (mx, my), fontsize=6.5, color=_CUT_COLOR, ha="left", va="center"
                )
            else:
                ax.plot(xs, ys, "-", color=_CUT_COLOR, lw=1.4, alpha=0.6, label=lbl)

    ax.plot(0, 0, "o", color="black", ms=9, label="start (0,0)")
    _arrow(ax, 0, 0, 0.0, "black", scale=6.0)

    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, ls=":", alpha=0.4)
    ax.set_xlabel("forward (cm)")
    ax.set_ylabel("left (cm)")
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.legend(loc="best", fontsize=8)

    info = f"{summary}\nto_absolute: {abs_status}\nsplinify:    {spl_status}"
    if cut_status:
        info += f"\ncut_corners: {cut_status}"
    ax.text(
        0.01,
        0.99,
        info,
        transform=ax.transAxes,
        fontsize=8,
        va="top",
        ha="left",
        family="monospace",
        bbox=dict(boxstyle="round", fc="#fffbe6", ec="#cccccc", alpha=0.95),
    )

    fig.tight_layout()
    if out_png:
        fig.savefig(out_png, dpi=130)
        print(f"  wrote {out_png}")
    if show:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Mission discovery
# ---------------------------------------------------------------------------


def discover_missions(lib: Lib):
    """Return [(label, sequence_builder_callable)] for every mission in src.missions."""
    import src.missions as M_pkg

    found = []
    for mi in pkgutil.iter_modules(M_pkg.__path__):
        try:
            mod = importlib.import_module(f"src.missions.{mi.name}")
        except Exception as e:
            print(f"[skip module {mi.name}] {type(e).__name__}: {e}")
            continue
        for nm, obj in inspect.getmembers(mod, inspect.isclass):
            if issubclass(obj, lib.Mission) and obj.__module__ == mod.__name__:
                found.append((nm, obj))
    found.sort(key=lambda t: t[0])
    return found


def node_summary(nodes, lib: Lib) -> str:
    kinds: dict[str, int] = {}
    for n in nodes:
        if n is None:
            k = "deferred"
        elif isinstance(n, lib.SideAction):
            k = "side"
        else:
            k = n.kind
        kinds[k] = kinds.get(k, 0) + 1
    parts = ", ".join(f"{k}×{v}" for k, v in sorted(kinds.items()))
    return f"{len(nodes)} nodes: {parts}" if parts else "0 nodes"


# ---------------------------------------------------------------------------
# Demo path (a clean, fully-convertible path so abs/spline always show)
# ---------------------------------------------------------------------------


def demo_sequence(lib: Lib):
    from raccoon.step.motion import (
        drive_forward,
        mark_heading_reference,
        turn_left,
        turn_right,
    )

    return [
        mark_heading_reference(),
        drive_forward(50),
        turn_right(90),
        drive_forward(40),
        turn_left(90),
        drive_forward(30),
        turn_left(90),
        drive_forward(25),
    ]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def render_one(
    label,
    seq,
    lib: Lib,
    out_dir: Path,
    preview_m: float,
    show: bool,
    cut_cm: float | None = None,
    codegen: bool = False,
):
    from raccoon.step.motion.path.passes import flatten_steps

    raw = seq if isinstance(seq, list) else [seq]
    nodes, _deferred = flatten_steps(raw)
    summary = node_summary(nodes, lib)

    rel_items = trace_relative(nodes, lib, preview_m)

    abs_nodes, abs_status = compile_absolute(seq, lib)
    abs_items = trace_absolute(abs_nodes, lib, preview_m) if abs_nodes is not None else []

    spl_status = splinify_status(seq, lib)
    ctrl, dense = spline_preview(rel_items, lib)

    cut_items = cut_status = None
    if cut_cm is not None:
        cut_items, cut_status = compile_cut_corners(seq, cut_cm, lib, preview_m)

    out_png = out_dir / f"{label}.png" if out_dir else None
    plot_with_spline(
        f"{label}  —  optimizer segments (relative vs absolute vs spline)",
        rel_items,
        abs_items,
        abs_status,
        spl_status,
        summary,
        ctrl,
        dense,
        out_png,
        show,
        cut_items=cut_items,
        cut_status=cut_status,
    )

    code = codegen_text(seq, lib, cut_cm=cut_cm)
    if out_dir:
        cut_line = f"cut_corners: {cut_status}\n" if cut_status else ""
        (out_dir / f"{label}.txt").write_text(
            f"{label}\n{'=' * len(label)}\n\n"
            f"{summary}\n\nto_absolute: {abs_status}\nsplinify:    {spl_status}\n{cut_line}\n"
            f"{explain_text(seq, lib)}\n"
        )
        (out_dir / f"{label}.codegen.py").write_text(code)
    if codegen:
        print(f"\n{'─' * 70}\n# {label}\n{'─' * 70}\n{code}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--project", default=DEFAULT_PROJECT, help="raccoon project root")
    ap.add_argument("--mission", default=None, help="substring filter on mission class name")
    ap.add_argument("--out", default="optimizer_viz", help="output directory for PNGs/TXT")
    ap.add_argument(
        "--preview-cm",
        type=float,
        default=DEFAULT_PREVIEW_CM,
        help="stub length drawn for unknown (sensor) endpoints",
    )
    ap.add_argument(
        "--demo", action="store_true", help="render a clean built-in demo path (always convertible)"
    )
    ap.add_argument(
        "--cut-corners",
        type=float,
        default=None,
        metavar="CM",
        help="overlay optimize().cut_corners(CM) — arcs replace right-angle corners",
    )
    ap.add_argument(
        "--codegen", action="store_true", help="also print the IR→DSL code reconstruction to stdout"
    )
    ap.add_argument(
        "--combo",
        action="store_true",
        help="death test: print DSL for every pass combination "
        "(uses a clean built-in path unless --mission is given)",
    )
    ap.add_argument("--list", action="store_true", help="list discovered missions and exit")
    ap.add_argument("--show", action="store_true", help="open figures interactively")
    args = ap.parse_args()

    if not args.show:
        import matplotlib

        matplotlib.use("Agg")

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    project = Path(args.project).resolve()
    if not (project / "src").is_dir():
        sys.exit(f"no src/ under {project} — is --project correct?")
    sys.path.insert(0, str(project))
    os.chdir(project)

    lib = load_lib()
    preview_m = args.preview_cm / 100.0

    if args.combo:
        cut_cm = args.cut_corners if args.cut_corners is not None else 8.0
        if args.mission:
            missions = [
                (n, c) for (n, c) in discover_missions(lib) if args.mission.lower() in n.lower()
            ]
            if not missions:
                sys.exit("no missions matched")
            label = missions[0][0]
            seq = missions[0][1]().sequence()
        else:
            label, seq = "death_test_clean_path", combo_sequence(lib)
        code = combo_codegen_text(seq, lib, cut_cm=cut_cm)
        print(f"\n{'─' * 70}\n# COMBO / DEATH TEST — {label}\n{'─' * 70}\n{code}")
        (out_dir / f"{label}.combo.py").write_text(code)
        print(f"wrote {out_dir / f'{label}.combo.py'}")
        return

    if args.demo:
        # The demo path is clean linear+turn+linear corners — show cut_corners by
        # default (8 cm) so the arc-rounding is visible without extra flags.
        cut_cm = args.cut_corners if args.cut_corners is not None else 8.0
        print(f"Rendering built-in demo path (cut_corners={cut_cm:.0f}cm)…")
        render_one(
            "demo_clean_path",
            demo_sequence(lib),
            lib,
            out_dir,
            preview_m,
            args.show,
            cut_cm=cut_cm,
            codegen=args.codegen,
        )
        return

    missions = discover_missions(lib)
    if args.mission:
        missions = [(n, c) for (n, c) in missions if args.mission.lower() in n.lower()]

    if args.list:
        print(f"{len(missions)} mission(s):")
        for nm, _ in missions:
            print(f"  {nm}")
        return

    if not missions:
        sys.exit("no missions matched")

    print(f"Rendering {len(missions)} mission(s) → {out_dir}")
    ok = 0
    for nm, cls in missions:
        print(f"- {nm}")
        try:
            seq = cls().sequence()
        except Exception as e:
            print(f"  [skip] sequence() failed: {type(e).__name__}: {e}")
            continue
        try:
            render_one(
                nm,
                seq,
                lib,
                out_dir,
                preview_m,
                args.show,
                cut_cm=args.cut_corners,
                codegen=args.codegen,
            )
            ok += 1
        except Exception as e:
            import traceback

            traceback.print_exc()
            print(f"  [error] {type(e).__name__}: {e}")
    print(f"\nDone: {ok}/{len(missions)} rendered into {out_dir}")


if __name__ == "__main__":
    main()
