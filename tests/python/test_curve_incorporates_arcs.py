"""Tests for arcs/diagonals incorporated into the single Catmull-Rom curve.

``segments_to_spline_waypoints`` / ``build_spline_step`` no longer reject arc
segments.  Instead an arc becomes several SAMPLED control waypoints (one every
~18° of sweep) the centripetal Catmull-Rom traces, so a path with arcs (from
``cut_corners`` or an explicit ``drive_arc_*``) is ONE continuous spline.  A
diagonal becomes a single body-frame-rotated control waypoint, and a plain
linear/turn sequence behaves exactly as before (corners rounded by the curve).

Arc geometry reproduces ``ArcMotion`` (``arc_motion.cpp``): the body-frame
velocity is forward (drive arc) or lateral (strafe arc), the centre lies 90°
toward the inside of the turn at ``radius_m``, the heading sweeps by the signed
``arc_angle_rad`` (>0 = CCW/left), and the sampled points lie on that circle.
"""

from __future__ import annotations

import importlib.util
import math

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


@requires_libstp
def test_linear_turn_linear_waypoints_unchanged():
    # A plain drive/turn/drive path still emits exactly one waypoint per linear
    # endpoint (corners rounded by the Catmull-Rom), and build_spline_step works.
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import (
        build_spline_step,
        segments_to_spline_waypoints,
    )
    from raccoon.step.motion.turn_dsl import turn_right

    nodes, _ = flatten_steps([drive_forward(50), turn_right(90), drive_forward(40)])
    segs = [n for n in nodes if isinstance(n, Segment)]

    wps = segments_to_spline_waypoints(segs)
    assert len(wps) == 2  # one per linear; turn produces none

    # Forward 50 cm along +x.
    assert math.isclose(wps[0][0], 50.0, abs_tol=1e-6)
    assert math.isclose(wps[0][1], 0.0, abs_tol=1e-6)
    # After turning right 90° (heading -> -90°), drive 40 cm -> -y.
    assert math.isclose(wps[1][0], 50.0, abs_tol=1e-6)
    assert math.isclose(wps[1][1], -40.0, abs_tol=1e-6)

    spline = build_spline_step(nodes)
    assert len(spline._waypoints) == 2


@requires_libstp
def test_arc_left_incorporated_into_curve():
    # drive_forward + drive_arc_left + drive_forward now SUCCEEDS: the arc is
    # sampled into several control points that trace the circle, and the post-arc
    # linear integrates from the arc's true endpoint/heading.
    from raccoon.step.motion.arc_dsl import drive_arc_left
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import (
        build_spline_step,
        segments_to_spline_waypoints,
    )

    radius_cm = 20.0
    nodes, _ = flatten_steps(
        [drive_forward(50), drive_arc_left(radius_cm=radius_cm, degrees=90), drive_forward(30)]
    )
    segs = [n for n in nodes if isinstance(n, Segment)]
    wps = segments_to_spline_waypoints(segs)

    # 1 (first linear) + arc samples (90° / 18° = 5) + 1 (last linear).
    assert len(wps) >= 5

    # The first waypoint is the forward-50 endpoint.
    assert math.isclose(wps[0][0], 50.0, abs_tol=1e-6)
    assert math.isclose(wps[0][1], 0.0, abs_tol=1e-6)

    # The arc starts at (50, 0) heading 0 (facing +x). A left (CCW) arc curves
    # toward +y; its centre is at (50, +radius). Every arc sample must lie on the
    # circle of that radius about the centre.
    cx, cy = 50.0, radius_cm
    arc_samples = wps[1:-1]
    assert len(arc_samples) >= 2
    for sx, sy in arc_samples:
        r = math.hypot(sx - cx, sy - cy)
        assert math.isclose(r, radius_cm, abs_tol=1e-6)

    # The arc endpoint (last sampled point before the final linear) is a
    # quarter-circle CCW from (50, 0) about (50, radius): -> (50 + radius, radius)
    # with heading +90° (now facing +y/left).
    arc_end = arc_samples[-1]
    assert math.isclose(arc_end[0], 50.0 + radius_cm, abs_tol=1e-6)
    assert math.isclose(arc_end[1], radius_cm, abs_tol=1e-6)

    # The final linear (30 cm forward) integrates from the arc endpoint at
    # heading +90°, so it advances +y by 30 cm.
    last = wps[-1]
    assert math.isclose(last[0], 50.0 + radius_cm, abs_tol=1e-6)
    assert math.isclose(last[1], radius_cm + 30.0, abs_tol=1e-6)

    # And the whole thing builds into a SplinePath.
    spline = build_spline_step(nodes)
    assert len(spline._waypoints) == len(wps)


@requires_libstp
def test_arc_right_curves_toward_negative_y():
    # A right (CW) arc curves toward -y: centre at (0, -radius), endpoint a
    # quarter-circle CW -> (radius, -radius), heading -90°.
    from raccoon.step.motion.arc_dsl import drive_arc_right
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import segments_to_spline_waypoints

    radius_cm = 15.0
    nodes, _ = flatten_steps([drive_arc_right(radius_cm=radius_cm, degrees=90)])
    segs = [n for n in nodes if isinstance(n, Segment)]
    wps = segments_to_spline_waypoints(segs)

    cx, cy = 0.0, -radius_cm
    for sx, sy in wps:
        assert math.isclose(math.hypot(sx - cx, sy - cy), radius_cm, abs_tol=1e-6)

    end = wps[-1]
    assert math.isclose(end[0], radius_cm, abs_tol=1e-6)
    assert math.isclose(end[1], -radius_cm, abs_tol=1e-6)


@requires_libstp
def test_single_arc_alone_succeeds():
    # A single arc on its own yields enough sampled control points (>= 2) for a
    # valid spline — no "at least 2 linear segments" rejection anymore.
    from raccoon.step.motion.arc_dsl import drive_arc_left
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import build_spline_step

    nodes, _ = flatten_steps([drive_arc_left(radius_cm=20, degrees=90)])
    spline = build_spline_step(nodes)
    assert len(spline._waypoints) >= 2


@requires_libstp
def test_rejects_side_action():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import build_spline_step
    from raccoon.step.wait_for_seconds_dsl import wait_for_seconds

    nodes, _ = flatten_steps([drive_forward(50), wait_for_seconds(1), drive_forward(30)])
    with pytest.raises(ValueError):
        build_spline_step(nodes)


@requires_libstp
def test_rejects_deferred():
    from raccoon.step.motion.path.passes.spline import build_spline_step

    # A None placeholder models a deferred (runtime-resolved) step.
    with pytest.raises(ValueError):
        build_spline_step([None])


class _FakeIRSensor:
    """Minimal IRSensor stand-in for ``on_black``."""

    def probabilityOfBlack(self) -> float:
        return 0.0

    def probabilityOfWhite(self) -> float:
        return 1.0


@requires_libstp
def test_rejects_until_sensor_condition():
    from raccoon.step.condition import on_black
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import build_spline_step

    nodes, _ = flatten_steps(
        [
            drive_forward(50),
            drive_forward(40).until(on_black(_FakeIRSensor())),
        ]
    )
    with pytest.raises(ValueError):
        build_spline_step(nodes)


@requires_libstp
def test_optimize_cut_corners_splinify_composes_with_arc():
    # cut_corners() inserts an arc; splinify() must NOT raise the old arc
    # ValueError — the arc composes into the single curve. We drive the full
    # pass pipeline via explain() (which runs every pass) and assert it does not
    # raise and yields a node.
    from raccoon.step.motion import optimize
    from raccoon.step.motion.arc_dsl import drive_arc_left
    from raccoon.step.motion.drive_dsl import drive_forward

    opt = (
        optimize([drive_forward(50), drive_arc_left(radius_cm=20, degrees=90), drive_forward(30)])
        .cut_corners(5)
        .splinify()
    )

    # explain() runs the full pipeline (decompose, merge, cut_corners, splinify)
    # without touching a robot — it must not raise the arc ValueError.
    report = opt.explain()
    assert "splinify" in report
