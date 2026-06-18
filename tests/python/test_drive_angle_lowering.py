"""Unit tests for ``DriveAngle.lower_to_segments()``.

``DriveAngle`` holds heading fixed while translating along a robot-centric
vector at ``angle_deg``. The path IR ``linear`` segment can only express travel
along the ``Forward`` or ``Lateral`` axis, so:

- axis-aligned angles (0 / 180 / +90 / -90) lower to a single ``linear`` segment,
- any true diagonal lowers to a single ``diagonal`` segment carrying the known
  body-frame displacement (it runs via an opaque adapter, not a barrier).

This is the regression guard for the m070 crash where ``drive_angle(...)``
flowed through the optimizer with no ``lower_to_segments`` and broke flattening.
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


def _resolve(step):
    """Resolve a DSL builder to its concrete Step (mirrors lowering.resolve_step)."""
    return step.resolve() if hasattr(step, "resolve") else step


# ---------------------------------------------------------------------------
# Axis-aligned drive_angle lowers to a linear segment
# ---------------------------------------------------------------------------


@requires_libstp
def test_drive_angle_forward_lowers_to_forward_linear():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    segs = _resolve(drive_angle(angle_deg=0, cm=30)).lower_to_segments()

    assert isinstance(segs, list) and len(segs) == 1
    seg = segs[0]
    assert seg.kind == "linear"
    assert seg.axis == LinearAxis.Forward
    assert seg.sign == pytest.approx(1.0)
    assert seg.distance_m == pytest.approx(0.30)
    assert seg.has_known_endpoint is True


@requires_libstp
def test_drive_angle_backward_lowers_to_negative_forward():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    seg = _resolve(drive_angle(angle_deg=180, cm=20)).lower_to_segments()[0]
    assert seg.kind == "linear"
    assert seg.axis == LinearAxis.Forward
    assert seg.sign == pytest.approx(-1.0)
    assert seg.distance_m == pytest.approx(-0.20)


@requires_libstp
def test_drive_angle_right_lowers_to_lateral():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    seg = _resolve(drive_angle(angle_deg=90, cm=15)).lower_to_segments()[0]
    assert seg.kind == "linear"
    assert seg.axis == LinearAxis.Lateral
    assert seg.sign == pytest.approx(1.0)
    assert seg.distance_m == pytest.approx(0.15)


@requires_libstp
def test_drive_angle_left_lowers_to_negative_lateral():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    seg = _resolve(drive_angle(angle_deg=-90, cm=15)).lower_to_segments()[0]
    assert seg.kind == "linear"
    assert seg.axis == LinearAxis.Lateral
    assert seg.sign == pytest.approx(-1.0)
    assert seg.distance_m == pytest.approx(-0.15)


@requires_libstp
def test_drive_angle_speed_is_carried():
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    seg = _resolve(drive_angle(angle_deg=0, cm=10, speed=0.4)).lower_to_segments()[0]
    assert seg.speed_scale == pytest.approx(0.4)


# ---------------------------------------------------------------------------
# True diagonal angles lower to a "diagonal" segment
# ---------------------------------------------------------------------------


@requires_libstp
@pytest.mark.parametrize("angle", [45, -45, 30, 135, -120])
def test_drive_angle_diagonal_lowers_to_diagonal_segment(angle):
    import math

    from raccoon.step.motion.drive_angle_dsl import drive_angle

    segs = _resolve(drive_angle(angle_deg=angle, cm=30)).lower_to_segments()
    assert isinstance(segs, list) and len(segs) == 1
    seg = segs[0]
    assert seg.kind == "diagonal"
    theta = math.radians(angle)
    assert seg.forward_m == pytest.approx(0.30 * math.cos(theta))
    assert seg.left_m == pytest.approx(-0.30 * math.sin(theta))
    assert seg.distance_m == pytest.approx(0.30)
    assert seg.has_known_endpoint is True


# ---------------------------------------------------------------------------
# Condition-only mode has an unknown endpoint
# ---------------------------------------------------------------------------


@requires_libstp
def test_drive_angle_conditional_has_unknown_endpoint():
    from raccoon.step.condition import StopCondition
    from raccoon.step.motion.drive_angle_dsl import drive_angle

    class _AlwaysFalse(StopCondition):
        def reset(self, robot):  # pragma: no cover - trivial
            pass

        def is_met(self, robot):  # pragma: no cover - trivial
            return False

    seg = _resolve(drive_angle(angle_deg=90, speed=0.5).until(_AlwaysFalse())).lower_to_segments()[
        0
    ]
    assert seg.kind == "linear"
    assert seg.distance_m is None
    assert seg.has_known_endpoint is False
    assert seg.condition is not None


# ---------------------------------------------------------------------------
# Integration: flatten no longer crashes / falls through to a side action
# ---------------------------------------------------------------------------


@requires_libstp
def test_flatten_axis_aligned_drive_angle_yields_segment():
    from raccoon.step.motion.drive_angle_dsl import drive_angle
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps

    nodes, deferred = flatten_steps([drive_angle(angle_deg=90, cm=20)])
    assert deferred == []
    assert len(nodes) == 1
    assert isinstance(nodes[0], Segment)
    assert nodes[0].kind == "linear"


@requires_libstp
def test_flatten_diagonal_drive_angle_yields_diagonal_segment():
    from raccoon.step.motion.drive_angle_dsl import drive_angle
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps

    # A true diagonal lowers to a "diagonal" Segment (not a barrier), no crash.
    nodes, deferred = flatten_steps([drive_angle(angle_deg=45, cm=20)])
    assert deferred == []
    assert len(nodes) == 1
    assert isinstance(nodes[0], Segment)
    assert nodes[0].kind == "diagonal"


@requires_libstp
def test_flatten_mixed_drive_and_axis_aligned_drive_angle():
    from raccoon.step.motion.drive_angle_dsl import drive_angle
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.passes.lowering import flatten_steps

    nodes, _ = flatten_steps([drive_forward(50), drive_angle(angle_deg=90, cm=20)])
    kinds = [n.kind for n in nodes]
    assert kinds == ["linear", "linear"]
