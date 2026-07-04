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
def test_to_absolute_heading_turn_without_mark_degrades_gracefully():
    """A heading turn needs the compile-time mark offset to align the reference
    with localization. Without a ``mark_heading_reference()`` in the steps the
    offset is unknown (common when ``optimize()`` wraps a single mission whose
    mark was set earlier), so the fold DEGRADES GRACEFULLY: the run passes
    through as ordinary relative legs instead of raising — to_absolute is
    best-effort and must never break a build. The only cost is no
    drift-correction on that run, so no absolute (``GotoWaypoints``) fold is
    emitted for it."""
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.heading_reference import turn_to_heading_right
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.optimize import optimize

    opt = optimize([drive_forward(50), turn_to_heading_right(90), drive_forward(30)]).to_absolute()
    # Must NOT raise — it compiles, degrading the unresolvable heading run.
    plan = PathCompiler(opt._effective_passes()).compile(opt._raw_steps)
    assert plan is not None
    # No GotoWaypoints carrying an ABSOLUTE-heading waypoint was emitted (the
    # run stayed relative). GotoWaypoints store waypoints whose 5th field is the
    # heading kind ("rel"/"abs"); a degraded run produces none with "abs".
    from raccoon.step.motion.goto import GotoWaypoints

    def _iter_steps(p):
        for node in getattr(p, "nodes", getattr(p, "steps", [])) or []:
            yield getattr(node, "step", node)

    abs_waypoints = [
        s
        for s in _iter_steps(plan)
        if isinstance(s, GotoWaypoints) and any(wp[4] == "abs" for wp in s._waypoints)
    ]
    assert not abs_waypoints, "unresolvable heading run should degrade to relative, not fold to abs"


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


# ---------------------------------------------------------------------------
# force_direction — enum, validation, and the on_start direction-preservation
# regression (a forced direction must NOT be re-normalized to shortest path).
# ---------------------------------------------------------------------------


class _FakePose:
    def __init__(self, heading: float) -> None:
        self.heading = heading


class _FakeOdometry:
    def __init__(self, heading: float = 0.0) -> None:
        self._heading = heading

    def get_pose(self) -> _FakePose:
        return _FakePose(self._heading)


class _FakeRobot:
    """Minimal robot exposing only what TurnToHeading.on_start touches."""

    def __init__(self, heading: float = 0.0) -> None:
        self.odometry = _FakeOdometry(heading)
        self.localization = None
        self.drive = object()
        self.motion_pid_config = object()
        self._service = None

    def get_service(self, _cls):
        return self._service


def _robot_with_marked_reference(heading: float = 0.0):
    from raccoon.robot.heading_reference import HeadingReferenceService

    robot = _FakeRobot(heading=heading)
    service = HeadingReferenceService(robot)
    service.mark()  # reference == current heading, positive_direction="left"
    robot._service = service
    return robot, service


@requires_libstp
def test_turn_direction_enum_validates_and_coerces():
    from raccoon.robot.heading_reference import TurnDirection

    assert TurnDirection.coerce(None) is None
    assert TurnDirection.coerce("left") is TurnDirection.LEFT
    assert TurnDirection.coerce("RIGHT") is TurnDirection.RIGHT
    assert TurnDirection.coerce(TurnDirection.LEFT) is TurnDirection.LEFT
    # str-enum keeps string equality so legacy comparisons still work.
    assert TurnDirection.RIGHT == "right"
    with pytest.raises(ValueError, match="Invalid turn direction"):
        TurnDirection.coerce("sideways")


@requires_libstp
def test_factory_coerces_force_direction_string_to_enum():
    from raccoon.robot.heading_reference import TurnDirection
    from raccoon.step.motion.heading_reference import turn_to_heading_left

    step = _resolve(turn_to_heading_left(45, force_direction="right"))
    assert step._force_direction is TurnDirection.RIGHT


@requires_libstp
def test_factory_rejects_invalid_force_direction():
    from raccoon.step.motion.heading_reference import turn_to_heading_right

    with pytest.raises(ValueError, match="Invalid turn direction"):
        _resolve(turn_to_heading_right(30, force_direction="up"))


@requires_libstp
def test_compute_turn_forced_direction_extends_past_180():
    from raccoon.robot.heading_reference import TurnDirection

    _, service = _robot_with_marked_reference(heading=0.0)

    # Shortest path to +10° is +10° left; forcing RIGHT must go the long way.
    assert service.compute_turn(10.0) == pytest.approx(10.0)
    assert service.compute_turn(10.0, force_direction=TurnDirection.RIGHT) == pytest.approx(-350.0)
    # And vice-versa: shortest path to -10° is right; forcing LEFT wraps to +350.
    assert service.compute_turn(-10.0) == pytest.approx(-10.0)
    assert service.compute_turn(-10.0, force_direction="left") == pytest.approx(350.0)


@requires_libstp
def test_on_start_preserves_forced_direction_target(monkeypatch):
    """Regression: on_start must hand the forced (possibly >180°) delta to
    TurnMotion verbatim. The previous ``math.remainder`` normalization silently
    folded a forced 350° right turn back into a 10° left turn."""
    import math

    import raccoon.step.motion.heading_reference as hr
    from raccoon.robot.heading_reference import TurnDirection

    robot, _ = _robot_with_marked_reference(heading=0.0)

    captured = {}

    class _FakeMotion:
        def __init__(self, _drive, _odo, _pid, config):
            captured["config"] = config

        def start(self) -> None:
            pass

    monkeypatch.setattr(hr, "TurnMotion", _FakeMotion)

    # turn_to_heading_left(10) -> target +10°; forcing RIGHT -> -350° physical.
    step = _resolve(hr.turn_to_heading_left(10, force_direction=TurnDirection.RIGHT))
    step.on_start(robot)

    assert "config" in captured  # not skipped as "already at target"
    assert captured["config"].target_angle_rad == pytest.approx(math.radians(-350.0))
    # Guard against the old bug: must NOT have collapsed to the +10° shortest path.
    assert captured["config"].target_angle_rad < -math.pi
