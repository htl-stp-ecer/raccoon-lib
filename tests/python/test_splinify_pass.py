"""Tests for the terminal ``SplinifyPass`` optimizer pass.

``SplinifyPass`` collapses a relative linear/turn path into a single
``Segment(kind="spline")`` node by reusing ``build_spline_step`` (validate +
build the ``SplinePath``) and ``SplinePath.lower_to_segments`` (produce the
single spline segment the unified executor drives).
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
def test_splinify_collapses_to_single_spline_segment():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass
    from raccoon.step.motion.spline_path import SplinePath
    from raccoon.step.motion.turn_dsl import turn_right

    nodes, _deferred = flatten_steps([drive_forward(50), turn_right(90), drive_forward(30)])

    result = SplinifyPass().run(nodes)

    assert len(result) == 1
    seg = result[0]
    assert isinstance(seg, Segment)
    assert seg.kind == "spline"
    assert seg.has_known_endpoint is True
    assert isinstance(seg.opaque_step, SplinePath)


@requires_libstp
def test_splinify_waypoints_match_segments_to_spline_waypoints():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import (
        SplinifyPass,
        segments_to_spline_waypoints,
    )
    from raccoon.step.motion.turn_dsl import turn_right

    nodes, _deferred = flatten_steps([drive_forward(50), turn_right(90), drive_forward(30)])

    segs = [n for n in nodes if isinstance(n, Segment)]
    expected = segments_to_spline_waypoints(segs)

    result = SplinifyPass().run(nodes)
    spline_path = result[0].opaque_step

    assert spline_path._waypoints == expected
    # Sanity: the run actually involved a turn, so the spline curves.
    assert len(expected) == 2


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

    nodes, _deferred = flatten_steps(
        [drive_forward(50), wait_for_seconds(1), turn_right(90), drive_forward(30)]
    )

    with pytest.raises(ValueError):
        SplinifyPass().run(nodes)


@requires_libstp
def test_splinify_rejects_arc():
    from raccoon.step.motion.arc_dsl import drive_arc_left
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass

    nodes, _deferred = flatten_steps(
        [drive_forward(50), drive_arc_left(radius_cm=30, degrees=90), drive_forward(30)]
    )

    with pytest.raises(ValueError):
        SplinifyPass().run(nodes)


@requires_libstp
def test_splinify_rejects_single_linear():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass

    nodes, _deferred = flatten_steps([drive_forward(50)])

    with pytest.raises(ValueError):
        SplinifyPass().run(nodes)
