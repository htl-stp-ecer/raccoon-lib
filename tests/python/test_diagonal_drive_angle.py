"""Tests for first-class diagonal ``drive_angle`` optimizer support.

A true-diagonal ``drive_angle`` (e.g. ``drive_angle(45, cm=30)``) has a 100%
known body-frame travel direction, so it lowers to a ``"diagonal"`` segment
(NOT an opaque barrier) carrying ``forward_m`` / ``left_m``, runs via the
``DriveAngleAdapter``, and converts under ``to_absolute()`` into a single
``GotoWaypoints`` waypoint.

Geometry (robot-centric angle: 0 = forward, +90 = right, -90 = left):
``forward_m = d·cos(θ)``, ``left_m = -d·sin(θ)`` (right = +sin → left = -sin).
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


def _resolve(step):
    """Resolve a DSL builder to its concrete Step (mirrors lowering.resolve_step)."""
    return step.resolve() if hasattr(step, "resolve") else step


# ---------------------------------------------------------------------------
# lower_to_segments → a single "diagonal" segment with known displacement
# ---------------------------------------------------------------------------


@requires_libstp
def test_drive_angle_45_lowers_to_diagonal():
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    step = _resolve(drive_angle(angle_deg=45, cm=30))
    segs = step.lower_to_segments()

    assert isinstance(segs, list) and len(segs) == 1
    seg = segs[0]
    assert seg.kind == "diagonal"
    # forward-right: forward = +0.2121, left = -0.2121
    assert seg.forward_m == pytest.approx(0.30 * math.cos(math.radians(45)))
    assert seg.forward_m == pytest.approx(0.2121, abs=1e-4)
    assert seg.left_m == pytest.approx(-0.30 * math.sin(math.radians(45)))
    assert seg.left_m == pytest.approx(-0.2121, abs=1e-4)
    assert seg.distance_m == pytest.approx(0.30)
    assert seg.has_known_endpoint is True
    assert seg.opaque_step is step


@requires_libstp
def test_drive_angle_30_lowers_to_diagonal():
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    seg = _resolve(drive_angle(angle_deg=30, cm=20)).lower_to_segments()[0]
    assert seg.kind == "diagonal"
    assert seg.forward_m == pytest.approx(0.20 * math.cos(math.radians(30)))
    assert seg.forward_m == pytest.approx(0.1732, abs=1e-4)
    assert seg.left_m == pytest.approx(-0.10)
    assert seg.distance_m == pytest.approx(0.20)


# ---------------------------------------------------------------------------
# flatten → a single diagonal Segment (NOT a SideAction/barrier), no crash
# ---------------------------------------------------------------------------


@requires_libstp
def test_flatten_diagonal_yields_single_segment():
    from raccoon.step.motion.drive_angle_dsl import drive_angle
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes.lowering import flatten_steps

    nodes, deferred = flatten_steps([drive_angle(angle_deg=45, cm=30)])
    assert deferred == []
    assert len(nodes) == 1
    assert isinstance(nodes[0], Segment)
    assert not isinstance(nodes[0], SideAction)
    assert nodes[0].kind == "diagonal"


# ---------------------------------------------------------------------------
# Regression: axis-aligned angles still lower to linear
# ---------------------------------------------------------------------------


@requires_libstp
def test_axis_aligned_still_linear():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    seg = _resolve(drive_angle(angle_deg=90, cm=20)).lower_to_segments()[0]
    assert seg.kind == "linear"
    assert seg.axis == LinearAxis.Lateral


# ---------------------------------------------------------------------------
# until-only diagonal: unknown endpoint, stays a Segment after to_absolute
# ---------------------------------------------------------------------------


@requires_libstp
def test_until_only_diagonal_unknown_endpoint():
    from raccoon.step.condition import after_cm
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    seg = _resolve(drive_angle(angle_deg=45).until(after_cm(50))).lower_to_segments()[0]
    assert seg.kind == "diagonal"
    # known_distance only promotes linear / follow_line, not diagonal, so the
    # endpoint stays unknown and forward_m / left_m stay None.
    assert seg.has_known_endpoint is False
    assert seg.forward_m is None
    assert seg.left_m is None


@requires_libstp
def test_until_only_diagonal_stays_segment_after_to_absolute():
    from raccoon.step.condition import after_cm
    from raccoon.step.motion.drive_angle_dsl import drive_angle
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes import ToAbsolutePass, flatten_steps

    nodes, _ = flatten_steps([drive_angle(angle_deg=45).until(after_cm(50))])
    result = ToAbsolutePass().run(nodes)
    # Unknown endpoint -> does not qualify -> stays a Segment, no GotoWaypoints.
    segs = [n for n in result if isinstance(n, Segment)]
    assert len(segs) == 1
    assert segs[0].kind == "diagonal"
    assert not any(
        isinstance(n, SideAction) and type(n.step).__name__ == "GotoWaypoints" for n in result
    )


# ---------------------------------------------------------------------------
# Integration: to_absolute converts a known-endpoint diagonal to GotoWaypoints
# ---------------------------------------------------------------------------


@requires_libstp
def test_diagonal_converts_to_goto_waypoint():
    from raccoon.step.motion import GotoWaypoints
    from raccoon.step.motion.drive_angle_dsl import drive_angle
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.ir import SideAction
    from raccoon.step.motion.path.optimize import optimize

    opt = optimize([drive_angle(angle_deg=45, cm=30)]).to_absolute()
    nodes = PathCompiler(opt._passes).compile(opt._raw_steps).nodes

    gotos = [
        n.step for n in nodes if isinstance(n, SideAction) and isinstance(n.step, GotoWaypoints)
    ]
    assert len(gotos) == 1
    wps = gotos[0]._waypoints
    assert len(wps) == 1
    forward, left, dtheta = wps[0]
    # heading unchanged by a diagonal -> body == world delta.
    assert forward == pytest.approx(0.2121, abs=1e-4)
    assert left == pytest.approx(-0.2121, abs=1e-4)
    assert dtheta == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# is_same_type: two diagonals never warm-start
# ---------------------------------------------------------------------------


@requires_libstp
def test_two_diagonals_not_same_type():
    from raccoon.step.motion.drive_angle_dsl import drive_angle
    from raccoon.step.motion.path.passes.lowering import is_same_type

    a = _resolve(drive_angle(angle_deg=45, cm=30)).lower_to_segments()[0]
    b = _resolve(drive_angle(angle_deg=30, cm=20)).lower_to_segments()[0]
    assert is_same_type(a, b) is False
