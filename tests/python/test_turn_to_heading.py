"""Unit tests for ``turn_to_heading_right/left`` as a KNOWN-endpoint turn.

``turn_to_heading_*`` used to return a ``Defer`` (an opaque, runtime-only
barrier the optimizer cannot plan). They now return a real
:class:`~raccoon.step.motion.heading_reference.TurnToHeading` MotionStep that
lowers to a single ``Segment(kind="turn", has_known_endpoint=True)`` carrying
itself as ``opaque_step`` (``angle_rad is None`` — the target is an ABSOLUTE
reference heading resolved at ``on_start``).

Consequences the optimizer must respect:

- It is NOT a Defer barrier: ``flatten_steps`` produces a real turn ``Segment``
  with no ``None`` placeholder and an empty ``deferred`` list.
- It targets an absolute heading, NOT a per-run relative delta, so the
  ``to_absolute`` run-folding must BREAK the run around it (it cannot
  ``heading += angle_rad`` a ``None``) — it stays a standalone ``Segment``
  between two re-anchoring ``GotoWaypoints``.
- ``splinify()`` cannot fold an absolute turn into the curve → ``ValueError``.

Runtime motion can't be exercised without a robot/sim, so these tests cover
construction/signature, lowering, and optimizer-pass integration only.
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
# Construction / signature
# ---------------------------------------------------------------------------


@requires_libstp
def test_right_maps_to_negative_target_deg():
    from raccoon.step.motion.heading_reference import TurnToHeading, turn_to_heading_right

    step = _resolve(turn_to_heading_right(90))
    assert isinstance(step, TurnToHeading)
    assert step._target_deg == pytest.approx(-90.0)
    assert step._speed == pytest.approx(1.0)
    assert step._force_direction is None


@requires_libstp
def test_left_maps_to_positive_target_deg():
    from raccoon.step.motion.heading_reference import TurnToHeading, turn_to_heading_left

    step = _resolve(turn_to_heading_left(90, speed=0.5, force_direction="right"))
    assert isinstance(step, TurnToHeading)
    assert step._target_deg == pytest.approx(90.0)
    assert step._speed == pytest.approx(0.5)
    assert step._force_direction == "right"


@requires_libstp
def test_signature():
    from raccoon.step.motion.heading_reference import turn_to_heading_right

    step = _resolve(turn_to_heading_right(90))
    assert step._generate_signature() == "TurnToHeading(target=-90.0°)"


# ---------------------------------------------------------------------------
# Lowering — one known-endpoint turn segment carrying itself, angle_rad None
# ---------------------------------------------------------------------------


@requires_libstp
def test_lowers_to_known_endpoint_turn_segment():
    from raccoon.step.motion.heading_reference import turn_to_heading_right

    step = _resolve(turn_to_heading_right(90))
    segs = step.lower_to_segments()
    assert len(segs) == 1
    seg = segs[0]
    assert seg.kind == "turn"
    assert seg.has_known_endpoint is True
    assert seg.opaque_step is step
    assert seg.angle_rad is None


# ---------------------------------------------------------------------------
# Not a Defer barrier — flatten yields a real Segment, no None / no deferred
# ---------------------------------------------------------------------------


@requires_libstp
def test_flatten_has_no_defer_barrier():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.heading_reference import turn_to_heading_right
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps

    nodes, deferred = flatten_steps(
        [drive_forward(20), turn_to_heading_right(0), drive_forward(30)]
    )

    assert len(nodes) == 3
    assert all(n is not None for n in nodes)
    assert isinstance(nodes[0], Segment) and nodes[0].kind == "linear"
    assert isinstance(nodes[1], Segment) and nodes[1].kind == "turn"
    assert nodes[1].opaque_step is not None
    assert nodes[1].angle_rad is None
    assert isinstance(nodes[2], Segment) and nodes[2].kind == "linear"

    # No deferred barriers — the heading turn is planned, not deferred.
    assert deferred == []


# ---------------------------------------------------------------------------
# to_absolute — the heading turn BREAKS the run and stays a Segment between
# two re-anchoring GotoWaypoints
# ---------------------------------------------------------------------------


@requires_libstp
def test_to_absolute_keeps_heading_turn_as_segment_between_gotos():
    from raccoon.step.motion import GotoWaypoints
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.heading_reference import turn_to_heading_right
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.optimize import optimize

    opt = optimize([drive_forward(50), turn_to_heading_right(90), drive_forward(30)]).to_absolute()
    nodes = PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes

    # Expect: GotoWaypoints (SideAction), turn Segment, GotoWaypoints (SideAction).
    kinds = []
    for n in nodes:
        if isinstance(n, SideAction) and isinstance(n.step, GotoWaypoints):
            kinds.append("goto")
        elif isinstance(n, Segment):
            kinds.append(f"seg:{n.kind}")
        else:
            kinds.append(type(n).__name__)

    assert kinds == ["goto", "seg:turn", "goto"]

    turn_seg = next(n for n in nodes if isinstance(n, Segment))
    assert turn_seg.kind == "turn"
    assert turn_seg.opaque_step is not None
    assert turn_seg.angle_rad is None

    # Each GotoWaypoints anchors its own single forward leg (0.5 m / 0.3 m).
    gotos = [
        n.step for n in nodes if isinstance(n, SideAction) and isinstance(n.step, GotoWaypoints)
    ]
    assert len(gotos) == 2
    assert len(gotos[0]._waypoints) == 1
    assert len(gotos[1]._waypoints) == 1
    assert gotos[0]._waypoints[0][0] == pytest.approx(0.5)
    assert gotos[1]._waypoints[0][0] == pytest.approx(0.3)


# ---------------------------------------------------------------------------
# splinify — cannot fold an absolute heading turn
# ---------------------------------------------------------------------------


@requires_libstp
def test_splinify_rejects_heading_turn():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.heading_reference import turn_to_heading_right
    from raccoon.step.motion.path.passes.lowering import flatten_steps
    from raccoon.step.motion.path.passes.spline import build_spline_step

    nodes, _ = flatten_steps([drive_forward(50), turn_to_heading_right(90), drive_forward(30)])
    with pytest.raises(ValueError, match="turn_to_heading"):
        build_spline_step(nodes)


@requires_libstp
def test_splinify_builder_rejects_heading_turn():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.heading_reference import turn_to_heading_right
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.optimize import optimize

    opt = optimize([drive_forward(50), turn_to_heading_right(90), drive_forward(30)]).splinify()
    with pytest.raises(ValueError, match="turn_to_heading"):
        PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
