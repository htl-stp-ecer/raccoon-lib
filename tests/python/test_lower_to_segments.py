"""Unit tests for the ``lower_to_segments()`` self-describing protocol.

Each optimizer-supported step lowers itself into IR ``Segment``s; every other
step returns ``None`` (opaque) and is preserved by the pipeline as a side
action / barrier.  The optimizer's ``extract_segment`` delegates to this
protocol instead of a hardcoded ``isinstance`` switch.
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
# Leaf motion steps lower to the expected segment
# ---------------------------------------------------------------------------


@requires_libstp
def test_drive_forward_lowers_to_linear_segment():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.drive_dsl import drive_forward

    segs = _resolve(drive_forward(20)).lower_to_segments()

    assert isinstance(segs, list) and len(segs) == 1
    seg = segs[0]
    assert seg.kind == "linear"
    assert seg.axis == LinearAxis.Forward
    assert seg.distance_m == pytest.approx(0.20)
    assert seg.has_known_endpoint is True


@requires_libstp
def test_drive_backward_lowers_with_negative_distance():
    from raccoon.step.motion.drive_dsl import drive_backward

    seg = _resolve(drive_backward(20)).lower_to_segments()[0]
    assert seg.kind == "linear"
    # backward = negative signed distance
    assert seg.distance_m == pytest.approx(-0.20)


@requires_libstp
def test_turn_lowers_to_turn_segment():
    from raccoon.step.motion.turn_dsl import turn_right

    seg = _resolve(turn_right(90)).lower_to_segments()[0]
    assert seg.kind == "turn"
    assert abs(seg.angle_rad) == pytest.approx(math.radians(90))
    assert seg.has_known_endpoint is True


@requires_libstp
def test_spline_lowers_to_spline_segment_carrying_itself():
    from raccoon.step.motion.spline_path import spline

    step = _resolve(spline((50.0, 0.0), (100.0, 20.0)))
    seg = step.lower_to_segments()[0]
    assert seg.kind == "spline"
    assert seg.has_known_endpoint is True
    # the opaque step is carried so the executor can drive the real motion
    assert seg.opaque_step is step


# ---------------------------------------------------------------------------
# Conditional (.until) drive has an unknown endpoint
# ---------------------------------------------------------------------------


@requires_libstp
def test_conditional_drive_has_unknown_endpoint():
    from raccoon.step.condition import StopCondition
    from raccoon.step.motion.drive_dsl import drive_forward

    class _AlwaysFalse(StopCondition):
        def reset(self, robot):  # pragma: no cover - trivial
            pass

        def is_met(self, robot):  # pragma: no cover - trivial
            return False

    seg = _resolve(drive_forward(speed=0.5).until(_AlwaysFalse())).lower_to_segments()[0]
    assert seg.kind == "linear"
    assert seg.has_known_endpoint is False
    assert seg.condition is not None


# ---------------------------------------------------------------------------
# Opaque steps return None
# ---------------------------------------------------------------------------


@requires_libstp
def test_non_motion_step_is_opaque():
    from raccoon.step import wait_for_seconds

    assert _resolve(wait_for_seconds(0.1)).lower_to_segments() is None


@requires_libstp
def test_base_step_default_is_opaque():
    """A user step that does not override the protocol is opaque by default."""
    from raccoon.step import Step

    class _CustomStep(Step):
        async def _execute_step(self, robot):  # pragma: no cover - trivial
            pass

        def _generate_signature(self) -> str:
            return "CustomStep()"

    assert _CustomStep().lower_to_segments() is None


# ---------------------------------------------------------------------------
# extract_segment delegates to the protocol
# ---------------------------------------------------------------------------


@requires_libstp
def test_extract_segment_returns_segment_for_motion_step():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.passes.lowering import extract_segment

    seg = extract_segment(_resolve(drive_forward(30)))
    assert seg.kind == "linear"
    assert seg.distance_m == pytest.approx(0.30)


@requires_libstp
def test_extract_segment_rejects_opaque_step():
    from raccoon.step import wait_for_seconds
    from raccoon.step.motion.path.passes.lowering import extract_segment

    with pytest.raises(TypeError):
        extract_segment(_resolve(wait_for_seconds(0.1)))


# ---------------------------------------------------------------------------
# Integration: flatten a mixed sequence
# ---------------------------------------------------------------------------


@requires_libstp
def test_flatten_pure_motion_sequence():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.turn_dsl import turn_right

    nodes, deferred = flatten_steps([drive_forward(50), turn_right(90), drive_forward(30)])
    assert deferred == []
    kinds = [n.kind for n in nodes]
    assert kinds == ["linear", "turn", "linear"]


@requires_libstp
def test_flatten_mixed_motion_and_side_action():
    from raccoon.step import wait_for_seconds
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes.lowering import flatten_steps

    nodes, _ = flatten_steps([drive_forward(50), wait_for_seconds(0.1), drive_forward(30)])
    assert isinstance(nodes[0], Segment) and nodes[0].kind == "linear"
    assert isinstance(nodes[1], SideAction)  # opaque step pinned as side action
    assert isinstance(nodes[2], Segment) and nodes[2].kind == "linear"
