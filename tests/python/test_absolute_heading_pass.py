"""Tests for ``AbsoluteHeadingPass``.

Build IR nodes from real motion factories via ``flatten_steps`` and assert the
pass stamps an integrated path-start heading onto straight legs, respects
explicit user headings, and resets at barriers.

The frame caveat (``target_heading_rad`` is consumed as an ABSOLUTE world
heading at runtime while this pass emits PATH-START-relative values) is
documented in ``absolute_heading.py`` and the change report; these tests only
verify the compile-time integration the pass is responsible for.
"""

from __future__ import annotations

import math

from raccoon.motion import LinearAxis
from raccoon.step.motion import drive_forward, strafe_left, turn_left, turn_right
from raccoon.step.motion.path.ir import Segment, SideAction
from raccoon.step.motion.path.passes import flatten_steps
from raccoon.step.motion.path.passes.absolute_heading import AbsoluteHeadingPass


def _flatten(steps):
    nodes, _deferred = flatten_steps(steps)
    return nodes


def _linears(nodes):
    return [n for n in nodes if isinstance(n, Segment) and n.kind == "linear"]


def _turns(nodes):
    return [n for n in nodes if isinstance(n, Segment) and n.kind == "turn"]


def test_pass_has_name():
    assert AbsoluteHeadingPass().name == "absolute_heading"


def test_drive_turn_drive_integration():
    # turn_right(90) has angle_rad = -pi/2 (sign = -1), so the second leg
    # holds heading -pi/2; the first leg holds the start heading 0.
    nodes = _flatten([drive_forward(50), turn_right(90), drive_forward(30)])

    # Sanity: lowered turn angle sign matches our expectation.
    (turn,) = _turns(nodes)
    assert math.isclose(turn.angle_rad, -math.pi / 2, abs_tol=1e-9)

    out = AbsoluteHeadingPass().run(nodes)
    lin1, lin2 = _linears(out)
    assert math.isclose(lin1.target_heading_rad, 0.0, abs_tol=1e-9)
    assert math.isclose(lin2.target_heading_rad, -math.pi / 2, abs_tol=1e-9)


def test_turn_left_positive_integration():
    nodes = _flatten([drive_forward(20), turn_left(90), drive_forward(20)])
    out = AbsoluteHeadingPass().run(nodes)
    lin1, lin2 = _linears(out)
    assert math.isclose(lin1.target_heading_rad, 0.0, abs_tol=1e-9)
    assert math.isclose(lin2.target_heading_rad, math.pi / 2, abs_tol=1e-9)


def test_running_heading_accumulates_across_two_turns():
    nodes = _flatten(
        [
            drive_forward(10),
            turn_left(90),
            drive_forward(10),
            turn_right(45),
            drive_forward(10),
        ]
    )
    out = AbsoluteHeadingPass().run(nodes)
    lin1, lin2, lin3 = _linears(out)
    assert math.isclose(lin1.target_heading_rad, 0.0, abs_tol=1e-9)
    assert math.isclose(lin2.target_heading_rad, math.pi / 2, abs_tol=1e-9)
    # +90 then -45 -> +45 deg.
    assert math.isclose(lin3.target_heading_rad, math.pi / 4, abs_tol=1e-9)


def test_turn_gets_post_turn_heading():
    nodes = _flatten([drive_forward(10), turn_left(90), drive_forward(10)])
    out = AbsoluteHeadingPass().run(nodes)
    (turn,) = _turns(out)
    # Matches _lower_absolute_turn convention: post-turn absolute heading.
    assert math.isclose(turn.target_heading_rad, math.pi / 2, abs_tol=1e-9)


def test_strafe_leg_gets_stamped():
    nodes = _flatten([strafe_left(20)])
    out = AbsoluteHeadingPass().run(nodes)
    (lin,) = _linears(out)
    assert lin.axis == LinearAxis.Lateral
    assert math.isclose(lin.target_heading_rad, 0.0, abs_tol=1e-9)


def test_explicit_user_heading_preserved():
    # drive_forward supports heading=; it lowers to heading_deg, NOT
    # target_heading_rad, so the pass must leave target_heading_rad unset
    # (None) rather than overwrite the user's relative-heading intent.
    nodes = _flatten([drive_forward(50, heading=45), turn_right(90), drive_forward(30)])
    out = AbsoluteHeadingPass().run(nodes)
    lin1, lin2 = _linears(out)
    assert lin1.heading_deg == 45
    assert lin1.target_heading_rad is None  # not overwritten
    # The leg after the turn still gets the integrated heading.
    assert math.isclose(lin2.target_heading_rad, -math.pi / 2, abs_tol=1e-9)


def test_pre_set_target_heading_not_overwritten():
    seg = Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        sign=1.0,
        distance_m=0.5,
        target_heading_rad=1.234,
        has_known_endpoint=True,
    )
    out = AbsoluteHeadingPass().run([seg])
    assert math.isclose(out[0].target_heading_rad, 1.234, abs_tol=1e-9)


def test_side_action_resets_and_stops_until_next_clean_node():
    side = SideAction(step=object(), is_background=False)
    nodes = [
        Segment(kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.5),
        Segment(kind="turn", sign=1.0, angle_rad=math.pi / 2, has_known_endpoint=True),
        side,
        Segment(kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.5),
    ]
    out = AbsoluteHeadingPass().run(nodes)
    lin_before = out[0]
    lin_after = out[3]
    # Before the side action: stamped at start heading 0.
    assert math.isclose(lin_before.target_heading_rad, 0.0, abs_tol=1e-9)
    # The side action is preserved as-is.
    assert out[2] is side
    # After the side action: frame reset to 0, so the leg is stamped at 0 again
    # (NOT pi/2 from the pre-barrier turn).
    assert math.isclose(lin_after.target_heading_rad, 0.0, abs_tol=1e-9)


def test_none_deferred_placeholder_resets():
    nodes = [
        Segment(kind="turn", sign=1.0, angle_rad=math.pi / 2, has_known_endpoint=True),
        Segment(kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.5),
        None,
        Segment(kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.5),
    ]
    out = AbsoluteHeadingPass().run(nodes)
    assert out[2] is None
    # Leg before None holds the post-turn heading pi/2.
    assert math.isclose(out[1].target_heading_rad, math.pi / 2, abs_tol=1e-9)
    # Leg after None is reset to 0.
    assert math.isclose(out[3].target_heading_rad, 0.0, abs_tol=1e-9)


def test_unknown_endpoint_turn_stops_stamping():
    # A condition-only turn (angle_rad None, has_known_endpoint False) makes
    # every downstream absolute heading unpredictable.
    nodes = [
        Segment(kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.5),
        Segment(kind="turn", sign=-1.0, angle_rad=None, has_known_endpoint=False),
        Segment(kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.3),
    ]
    out = AbsoluteHeadingPass().run(nodes)
    lins = _linears(out)
    # First leg (before the unknown turn) is stamped at start heading 0.
    assert math.isclose(lins[0].target_heading_rad, 0.0, abs_tol=1e-9)
    # The unknown-endpoint turn: not stamped.
    unknown_turn = out[1]
    assert unknown_turn.has_known_endpoint is False
    assert unknown_turn.target_heading_rad is None
    # Downstream straight leg: NOT stamped (frame is unpredictable).
    assert lins[1].target_heading_rad is None


def test_arc_passes_through_but_advances_heading():
    arc = Segment(
        kind="arc",
        radius_m=0.2,
        arc_angle_rad=math.pi / 2,  # +90 CCW
        has_known_endpoint=True,
    )
    nodes = [
        Segment(kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.5),
        arc,
        Segment(kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.5),
    ]
    out = AbsoluteHeadingPass().run(nodes)
    # Arc is untouched (no heading stamped onto it).
    assert out[1].kind == "arc"
    assert out[1].target_heading_rad is None
    # Leg before the arc: heading 0.
    assert math.isclose(out[0].target_heading_rad, 0.0, abs_tol=1e-9)
    # Leg after the arc: advanced by the arc angle.
    assert math.isclose(out[2].target_heading_rad, math.pi / 2, abs_tol=1e-9)


def test_pass_is_pure_does_not_mutate_input():
    nodes = _flatten([drive_forward(50), turn_right(90), drive_forward(30)])
    in_lin = _linears(nodes)
    AbsoluteHeadingPass().run(nodes)
    # Inputs untouched (replace() produced new segments).
    for lin in in_lin:
        assert lin.target_heading_rad is None
