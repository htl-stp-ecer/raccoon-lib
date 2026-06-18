"""Tests for the terminal ``SplinifyPass`` optimizer pass.

``SplinifyPass`` collapses a relative linear/turn path into a single
ABSOLUTE ``SideAction(GotoWaypoints)`` node: it reuses ``build_spline_step``
(validate + obtain the control waypoints), densely samples the centripetal
Catmull-Rom curve through them, and emits one closed-loop ``GotoWaypoints``
side-action.  The validation/rejection behaviour is unchanged; only the OUTPUT
node type changed (was ``Segment(kind="spline")``).
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
def test_splinify_collapses_to_single_gotowaypoints_side_action():
    # CHANGED: SplinifyPass now emits ONE SideAction(GotoWaypoints) (absolute,
    # closed-loop) instead of a relative Segment(kind="spline").
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.goto import GotoWaypoints
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass
    from raccoon.step.motion.turn_dsl import turn_right

    nodes, _deferred = flatten_steps([drive_forward(50), turn_right(90), drive_forward(30)])

    result = SplinifyPass().run(nodes)

    assert len(result) == 1
    side = result[0]
    assert isinstance(side, SideAction)
    assert side.is_background is False
    assert isinstance(side.step, GotoWaypoints)
    assert not any(isinstance(n, Segment) for n in result)


@requires_libstp
def test_splinify_dense_samples_span_control_waypoints():
    # CHANGED: the output is now GotoWaypoints carrying DENSE Catmull-Rom samples
    # (metres, +fwd/+left) rather than a SplinePath carrying the raw control
    # waypoints (centimetres). The dense samples span the control waypoints.
    import math

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
    control_cm = segments_to_spline_waypoints(segs)
    # Sanity: the run actually involved a turn, so the spline curves.
    assert len(control_cm) == 2
    control_m = [(fwd / 100.0, left / 100.0) for (fwd, left) in control_cm]

    result = SplinifyPass().run(nodes)
    wps = result[0].step._waypoints  # (fwd_m, left_m, dtheta) dense samples
    assert len(wps) >= 10

    # First/last dense sample coincide with the first/last control waypoint.
    assert math.isclose(wps[0][0], control_m[0][0], abs_tol=1e-6)
    assert math.isclose(wps[0][1], control_m[0][1], abs_tol=1e-6)
    assert math.isclose(wps[-1][0], control_m[-1][0], abs_tol=1e-6)
    assert math.isclose(wps[-1][1], control_m[-1][1], abs_tol=1e-6)


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
