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
- It targets an absolute heading. Because localization 0° = start heading and
  the ``mark_heading_reference`` offset is the compile-time difference to the
  reference, ``to_absolute`` resolves the turn to an absolute localization
  heading at COMPILE time and FOLDS it into ONE continuous ``GotoWaypoints``
  (no run break, no runtime service read). Without a ``mark_heading_reference``
  in the optimized steps it raises a clear error.
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
    sig = step._generate_signature()
    assert "TurnToHeading" in sig
    assert "-90" in sig  # right(90) -> target -90 deg


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
# to_absolute — the heading turn FOLDS into ONE GotoWaypoints (compile-time)
# ---------------------------------------------------------------------------


@requires_libstp
def test_to_absolute_folds_heading_turn_into_one_goto():
    from raccoon.step.motion import GotoWaypoints
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.heading_reference import turn_to_heading_right
    from raccoon.step.motion.heading_reference_dsl import mark_heading_reference
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.optimize import optimize

    # mark offset 90 + turn_to_heading_right(90) -> localization heading 0.
    opt = optimize(
        [
            mark_heading_reference(origin_offset_deg=90),
            drive_forward(50),
            turn_to_heading_right(90),
            drive_forward(30),
        ]
    ).to_absolute()
    nodes = PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes

    # The mark passes through; drive + turn_to_heading + drive fold into ONE
    # GotoWaypoints (no leftover turn Segment) — the absolute heading is resolved
    # at COMPILE time via the mark offset.
    assert not any(isinstance(n, Segment) for n in nodes)
    gotos = [
        n.step for n in nodes if isinstance(n, SideAction) and isinstance(n.step, GotoWaypoints)
    ]
    assert len(gotos) == 1
    wps = gotos[0]._waypoints
    assert len(wps) == 2

    # Leg 1 (before the turn): RELATIVE, 0.5 m forward.
    assert wps[0][4] == "rel"
    assert wps[0][0] == pytest.approx(0.5)

    # Leg 2 (after turn_to_heading_right(90), offset 90 -> localization heading 0):
    # ABSOLUTE, 0.3 m along heading 0 (abs_dx = 0.3, abs_dy = 0).
    assert wps[1][4] == "abs"
    assert wps[1][5] == pytest.approx(0.0, abs=1e-9)
    assert wps[1][2] == pytest.approx(0.3)
    assert wps[1][3] == pytest.approx(0.0, abs=1e-9)


@requires_libstp
def test_to_absolute_heading_turn_without_mark_raises():
    """A heading turn needs the compile-time mark offset to align the reference
    with localization — without a mark_heading_reference() in the steps, the
    to_absolute fold raises a clear error."""
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.heading_reference import turn_to_heading_right
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.optimize import optimize

    opt = optimize([drive_forward(50), turn_to_heading_right(90), drive_forward(30)]).to_absolute()
    with pytest.raises(ValueError, match="mark_heading_reference"):
        PathCompiler(opt._effective_passes()).compile(opt._raw_steps)


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
def test_splinify_builder_splits_at_heading_turn():
    # A turn_to_heading targets an absolute reference heading and cannot be
    # folded into a relative Catmull-Rom, so it acts as a barrier: splinify
    # splits the motion into one spline on each side and keeps the heading turn
    # in place — it no longer raises through the optimize() builder.
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.heading_reference import turn_to_heading_right
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.optimize import optimize
    from raccoon.step.motion.turn_dsl import turn_left

    opt = optimize(
        [
            drive_forward(50),
            turn_left(90),
            drive_forward(40),
            turn_to_heading_right(90),
            drive_forward(30),
            turn_left(90),
            drive_forward(20),
        ]
    ).splinify()
    nodes = PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes

    splines = [n for n in nodes if isinstance(n, Segment) and n.kind == "spline"]
    heading_turns = [
        n
        for n in nodes
        if isinstance(n, Segment) and n.kind == "turn" and n.opaque_step is not None
    ]
    assert len(splines) == 2  # one curve on each side of the barrier
    assert len(heading_turns) == 1  # turn_to_heading preserved in place
