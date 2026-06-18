"""Tests for the ABSOLUTE ``SplinifyPass`` output.

``SplinifyPass`` no longer emits a relative ``Segment(kind="spline")``.  It
validates the path via ``build_spline_step`` (keeping its ValueError behaviour),
densely samples the SAME centripetal Catmull-Rom curve through the control
waypoints, and emits ONE inline ``SideAction(GotoWaypoints)`` that follows the
dense samples CLOSED-LOOP on the localization particle filter.

The relative ``SplineMotion`` / ``smooth_path(spline=True)`` path uses
``build_spline_step`` directly and is unaffected (covered by
``test_spline_unified.py`` / ``test_smooth_path_composite.py``).
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
def test_splinify_emits_single_gotowaypoints_side_action():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.goto import GotoWaypoints
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import SplinifyPass
    from raccoon.step.motion.turn_dsl import turn_right

    nodes, _deferred = flatten_steps([drive_forward(50), turn_right(90), drive_forward(30)])

    result = SplinifyPass().run(nodes)

    # Exactly ONE node, and it is a SideAction wrapping GotoWaypoints — NOT a
    # Segment(kind="spline") anymore.
    assert len(result) == 1
    side = result[0]
    assert isinstance(side, SideAction)
    assert side.is_background is False
    assert isinstance(side.step, GotoWaypoints)
    assert not any(isinstance(n, Segment) for n in result)

    # Many dense waypoints (the sampled curve), each a 3-tuple with dtheta=None
    # (hold-heading translation along the absolute curve). The control path here
    # spans 0.30 m, so ~3 cm spacing yields ~10 samples.
    wps = side.step._waypoints
    assert len(wps) >= 10
    for fwd_m, left_m, dtheta in wps:
        assert isinstance(fwd_m, float)
        assert isinstance(left_m, float)
        assert dtheta is None


@requires_libstp
def test_dense_samples_lie_on_catmull_rom_through_control_points():
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
    control_m = [(fwd / 100.0, left / 100.0) for (fwd, left) in control_cm]
    assert len(control_m) == 2  # the run curves through 2 control points

    wps = SplinifyPass().run(nodes)[0].step._waypoints
    samples = [(w[0], w[1]) for w in wps]

    # First sample ~ first control point, last sample ~ last control point.
    first, last = control_m[0], control_m[-1]
    assert math.isclose(samples[0][0], first[0], abs_tol=1e-6)
    assert math.isclose(samples[0][1], first[1], abs_tol=1e-6)
    assert math.isclose(samples[-1][0], last[0], abs_tol=1e-6)
    assert math.isclose(samples[-1][1], last[1], abs_tol=1e-6)

    # Reasonable count for ~3 cm spacing over the control-point path length.
    path_len = math.hypot(last[0] - first[0], last[1] - first[1])
    approx = path_len / 0.03
    assert 0.4 * approx <= len(samples) <= 4.0 * approx


@requires_libstp
def test_sampler_matches_cpp_endpoint_reflection():
    """A 3-point S-curve: samples pass through every control point.

    Centripetal Catmull-Rom with reflected virtual endpoints (the C++
    convention) passes through ALL control points, including interior ones.
    """
    from raccoon.step.motion.path.passes.spline import (
        sample_centripetal_catmull_rom,
    )

    cps = [(0.0, 0.0), (0.5, 0.15), (0.8, 0.0)]
    samples = sample_centripetal_catmull_rom(cps, spacing_m=0.03)

    # Endpoints exact.
    assert math.isclose(samples[0][0], cps[0][0], abs_tol=1e-9)
    assert math.isclose(samples[0][1], cps[0][1], abs_tol=1e-9)
    assert math.isclose(samples[-1][0], cps[-1][0], abs_tol=1e-9)
    assert math.isclose(samples[-1][1], cps[-1][1], abs_tol=1e-9)

    # Interior control point is hit by some sample (it's a segment boundary).
    assert any(
        math.isclose(x, cps[1][0], abs_tol=1e-9) and math.isclose(y, cps[1][1], abs_tol=1e-9)
        for (x, y) in samples
    )


@requires_libstp
def test_optimize_splinify_compiles_to_single_gotowaypoints():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.goto import GotoWaypoints
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.optimize import optimize
    from raccoon.step.motion.turn_dsl import turn_right

    opt = optimize([drive_forward(50), turn_right(90), drive_forward(30)]).splinify()

    plan = PathCompiler(opt._effective_passes()).compile(opt._raw_steps)

    side_actions = [n for n in plan.nodes if isinstance(n, SideAction)]
    assert len(side_actions) == 1
    assert isinstance(side_actions[0].step, GotoWaypoints)
    assert not any(isinstance(n, Segment) for n in plan.nodes)


@requires_libstp
def test_cut_corners_then_splinify_still_value_error():
    """cut_corners() stays RELATIVE → splinify reaches compile and the arc
    rejection ValueError still names splinify(), not smooth_path.
    """
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.optimize import optimize
    from raccoon.step.motion.turn_dsl import turn_right

    opt = optimize([drive_forward(50), turn_right(90), drive_forward(40)]).cut_corners(5).splinify()
    with pytest.raises(ValueError) as excinfo:
        PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
    msg = str(excinfo.value)
    assert "splinify()" in msg
    assert "smooth_path" not in msg


@requires_libstp
def test_to_absolute_then_splinify_still_path_build_error():
    """to_absolute() produces ABSOLUTE; splinify() requires RELATIVE → the
    representation guard still rejects the composition at BUILD time.
    """
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.optimize import PathBuildError, optimize

    opt = optimize([drive_forward(50), drive_forward(30)]).to_absolute()
    with pytest.raises(PathBuildError) as excinfo:
        opt.splinify()
    msg = str(excinfo.value)
    assert "splinify" in msg
    assert "RELATIVE" in msg
    assert "ABSOLUTE" in msg
