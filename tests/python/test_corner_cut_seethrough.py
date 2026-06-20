"""Corner-cut pass: geometry + see-through of non-blocking side actions.

A ``drive + turn + drive`` corner becomes ``drive' + arc + drive'``. Background /
ephemeral side actions between the legs run concurrently with the motion, so
they must NOT block the cut — they're collected and re-emitted around the arc.
Blocking side actions, deferred placeholders, opaque (unknown-angle) turns and
condition-bounded legs stay barriers.
"""

from __future__ import annotations

import importlib.util
import math

import pytest


def _has_raccoon() -> bool:
    return importlib.util.find_spec("raccoon") is not None


pytestmark = pytest.mark.skipif(not _has_raccoon(), reason="raccoon not importable")


def _lin(dist_m=0.5, sign=1.0, axis=None, cond=None, known=True):
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path.ir import Segment

    return Segment(
        kind="linear",
        axis=axis if axis is not None else LinearAxis.Forward,
        sign=sign,
        distance_m=dist_m,
        condition=cond,
        has_known_endpoint=known,
    )


def _turn(deg=90.0, known=True):
    from raccoon.step.motion.path.ir import Segment

    return Segment(
        kind="turn",
        angle_rad=(math.radians(deg) if deg is not None else None),
        has_known_endpoint=known,
    )


def _side(background=True, ephemeral=False):
    from raccoon.step.motion.path.ir import SideAction

    return SideAction(step=object(), is_background=background, ephemeral=ephemeral)


def _kinds(nodes):
    from raccoon.step.motion.path.ir import Segment

    return [n.kind if isinstance(n, Segment) else "side" for n in nodes]


def test_basic_corner_cuts_to_arc():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_lin(0.5), _turn(90), _lin(0.5)], cut_m=0.1)
    assert _kinds(out) == ["linear", "arc", "linear"]
    arc = out[1]
    # R = cut / tan(theta/2); 90° → R = cut.
    assert math.isclose(arc.radius_m, 0.1, rel_tol=1e-6)
    assert math.isclose(arc.arc_angle_rad, math.radians(90), rel_tol=1e-6)
    # Legs trimmed by cut_m.
    assert math.isclose(out[0].distance_m, 0.4, rel_tol=1e-6)
    assert math.isclose(out[2].distance_m, 0.4, rel_tol=1e-6)


def test_background_side_action_does_not_block_cut():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    nodes = [_lin(0.5), _side(background=True), _turn(90), _lin(0.5)]
    out = run_corner_cut(nodes, cut_m=0.1)
    # Cut still happens; the background side action is kept (concurrent w/ arc).
    assert _kinds(out) == ["linear", "side", "arc", "linear"]


def test_ephemeral_branch_does_not_block_cut():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    nodes = [_lin(0.5), _turn(90), _side(background=True, ephemeral=True), _lin(0.5)]
    out = run_corner_cut(nodes, cut_m=0.1)
    assert _kinds(out) == ["linear", "side", "arc", "linear"]


def test_blocking_side_action_is_a_barrier():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    nodes = [_lin(0.5), _side(background=False), _turn(90), _lin(0.5)]
    out = run_corner_cut(nodes, cut_m=0.1)
    # An inline (blocking) action stops the robot — not a smooth corner.
    assert _kinds(out) == ["linear", "side", "turn", "linear"]


def test_opaque_turn_not_cut():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    # turn_to_heading lowers to angle_rad=None (resolved at runtime).
    nodes = [_lin(0.5), _turn(deg=None), _lin(0.5)]
    out = run_corner_cut(nodes, cut_m=0.1)
    assert _kinds(out) == ["linear", "turn", "linear"]


def test_condition_bounded_leg_not_cut():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    # A sensor-bounded drive has no known geometry to trim.
    nodes = [_lin(dist_m=None, cond=object(), known=False), _turn(90), _lin(0.5)]
    out = run_corner_cut(nodes, cut_m=0.1)
    assert _kinds(out) == ["linear", "turn", "linear"]


def test_cross_axis_not_cut():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    nodes = [_lin(0.5, axis=LinearAxis.Forward), _turn(90), _lin(0.5, axis=LinearAxis.Lateral)]
    out = run_corner_cut(nodes, cut_m=0.1)
    assert _kinds(out) == ["linear", "turn", "linear"]


def test_short_leg_not_cut():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    # Leg shorter than the cut can't accommodate it.
    nodes = [_lin(0.05), _turn(90), _lin(0.5)]
    out = run_corner_cut(nodes, cut_m=0.1)
    assert _kinds(out) == ["linear", "turn", "linear"]
