"""Builder coupling for splinify — relative vs absolute, and arc composition.

- ``optimize(p).splinify()`` → ONE relative ``Segment(kind="spline")`` (the
  continuous C++ SplineMotion).
- ``optimize(p).to_absolute().splinify()`` → ONE ``SideAction(SplineFollow)``:
  to_absolute turns on absolute mode, so the whole path becomes one absolute
  spline followed closed-loop on the localization filter (the per-leg
  ToAbsolutePass is subsumed). NO PathBuildError — the combination is allowed.
- ``optimize(p).cut_corners(5).splinify()`` → ONE relative spline that
  INCORPORATES the arc cut_corners inserted ("everything is one spline").

Plus the pure centripetal Catmull-Rom sampler. smooth_path(spline=True) uses
build_spline_step directly and is unaffected (test_spline_unified / composite).
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


def _compiled_nodes(opt):
    from raccoon.step.motion.path.compiler import PathCompiler

    return PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes


@requires_libstp
def test_splinify_relative_compiles_to_one_spline_segment():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.optimize import optimize
    from raccoon.step.motion.turn_dsl import turn_right

    nodes = _compiled_nodes(
        optimize([drive_forward(50), turn_right(90), drive_forward(30)]).splinify()
    )

    segs = [n for n in nodes if isinstance(n, Segment)]
    assert len(segs) == 1
    assert segs[0].kind == "spline"
    assert not any(isinstance(n, SideAction) for n in nodes)


@requires_libstp
def test_to_absolute_then_splinify_is_one_absolute_spline():
    """to_absolute() turns on absolute mode; chained with splinify() the path
    becomes ONE absolute SplineFollow — allowed (no PathBuildError), per-leg
    ToAbsolutePass subsumed."""
    from raccoon.step.motion import SplineFollow
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.optimize import optimize
    from raccoon.step.motion.turn_dsl import turn_right

    opt = optimize([drive_forward(50), turn_right(90), drive_forward(30)]).to_absolute().splinify()
    nodes = _compiled_nodes(opt)

    side = [n for n in nodes if isinstance(n, SideAction)]
    assert len(side) == 1
    assert isinstance(side[0].step, SplineFollow)
    assert not any(isinstance(n, Segment) for n in nodes)


@requires_libstp
def test_cut_corners_then_splinify_incorporates_the_arc():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.optimize import optimize
    from raccoon.step.motion.turn_dsl import turn_right

    opt = optimize([drive_forward(50), turn_right(90), drive_forward(40)]).cut_corners(5).splinify()
    nodes = _compiled_nodes(opt)

    # cut_corners inserts an arc; splinify eats it into one relative spline (no
    # to_absolute → relative). One Segment(kind="spline"), no error.
    segs = [n for n in nodes if isinstance(n, Segment)]
    assert len(segs) == 1
    assert segs[0].kind == "spline"
    assert not any(isinstance(n, SideAction) for n in nodes)


@requires_libstp
def test_sampler_matches_cpp_endpoint_reflection():
    """A 3-point S-curve: samples pass through every control point (centripetal
    Catmull-Rom with reflected virtual endpoints, the C++ convention)."""
    from raccoon.step.motion.path.passes.spline import sample_centripetal_catmull_rom

    cps = [(0.0, 0.0), (0.5, 0.15), (0.8, 0.0)]
    samples = sample_centripetal_catmull_rom(cps, spacing_m=0.03)

    assert math.isclose(samples[0][0], cps[0][0], abs_tol=1e-9)
    assert math.isclose(samples[0][1], cps[0][1], abs_tol=1e-9)
    assert math.isclose(samples[-1][0], cps[-1][0], abs_tol=1e-9)
    assert math.isclose(samples[-1][1], cps[-1][1], abs_tol=1e-9)
    assert any(
        math.isclose(x, cps[1][0], abs_tol=1e-9) and math.isclose(y, cps[1][1], abs_tol=1e-9)
        for (x, y) in samples
    )
