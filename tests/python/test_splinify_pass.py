"""Tests for the terminal ``SplinifyPass`` optimizer pass.

``SplinifyPass`` collapses the whole path into ONE continuous spline.  Two
render modes (the builder sets ``absolute`` from whether ``to_absolute()`` is
chained):

- relative (default) → ``Segment(kind="spline")`` (continuous C++ SplineMotion).
- absolute → ``SideAction(SplineFollow)`` (continuous pure-pursuit on the
  localization particle filter).

Validation (via ``build_spline_step``) is shared by both modes: defers, side
actions, conditions, and <2 control waypoints raise; arcs/diagonals are accepted
and become control waypoints ("everything is one spline").
"""

from __future__ import annotations

import importlib.util

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


@requires_libstp
def test_splinify_relative_emits_one_spline_segment():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass
    from raccoon.step.motion.turn_dsl import turn_right

    nodes, _ = flatten_steps([drive_forward(50), turn_right(90), drive_forward(30)])

    result = SplinifyPass().run(nodes)  # default: relative

    assert len(result) == 1
    seg = result[0]
    assert isinstance(seg, Segment)
    assert seg.kind == "spline"
    assert not any(isinstance(n, SideAction) for n in result)


@requires_libstp
def test_splinify_absolute_emits_one_spline_follow():
    from raccoon.step.motion import SplineFollow
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass
    from raccoon.step.motion.turn_dsl import turn_right

    nodes, _ = flatten_steps([drive_forward(50), turn_right(90), drive_forward(30)])

    result = SplinifyPass(absolute=True).run(nodes)

    assert len(result) == 1
    side = result[0]
    assert isinstance(side, SideAction)
    assert side.is_background is False
    assert isinstance(side.step, SplineFollow)
    assert not any(isinstance(n, Segment) for n in result)


@requires_libstp
def test_splinify_pass_is_terminal():
    from raccoon.step.motion.path.passes.spline import SplinifyPass

    assert SplinifyPass.terminal is True
    assert SplinifyPass().terminal is True
    assert SplinifyPass.name == "splinify"


@requires_libstp
def test_splinify_rejects_side_action():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass
    from raccoon.step.motion.turn_dsl import turn_right
    from raccoon.step.wait_for_seconds_dsl import wait_for_seconds

    nodes, _ = flatten_steps(
        [drive_forward(50), wait_for_seconds(1), turn_right(90), drive_forward(30)]
    )

    with pytest.raises(ValueError):
        SplinifyPass().run(nodes)


@requires_libstp
def test_splinify_accepts_arc():
    # arcs are no longer rejected — they become sampled control waypoints the
    # Catmull-Rom traces, so the whole path is ONE spline.
    from raccoon.step.motion.arc_dsl import drive_arc_left
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass

    nodes, _ = flatten_steps(
        [drive_forward(50), drive_arc_left(radius_cm=30, degrees=90), drive_forward(30)]
    )

    result = SplinifyPass().run(nodes)

    assert len(result) == 1
    assert isinstance(result[0], Segment)
    assert result[0].kind == "spline"


@requires_libstp
def test_splinify_rejects_single_linear():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass

    nodes, _ = flatten_steps([drive_forward(50)])

    with pytest.raises(ValueError):
        SplinifyPass().run(nodes)
