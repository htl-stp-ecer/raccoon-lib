"""Tests for the ``decompose`` pass.

``DecomposePass`` splits a conditional ``linear`` / ``follow_line`` leg whose
stop condition is a sequential ``_Then`` chain at EVERY bare relative
``after_cm`` boundary — leading, trailing, or interleaved — emitting one
known-distance leg per ``after_cm`` leaf and one grouped unknown sensor leg per
run of consecutive non-``after_cm`` leaves, so each known part becomes
optimizable (absolutizable / splinifiable).

No simulator or C++ runtime is required beyond the raccoon package.
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


def _imports():
    from raccoon.motion import LinearAxis
    from raccoon.step.condition import _Then, after_cm, on_black, over_line
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes import DecomposePass, flatten_steps
    from raccoon.step.wait_for_seconds_dsl import wait_for_seconds

    return {
        "LinearAxis": LinearAxis,
        "_Then": _Then,
        "after_cm": after_cm,
        "on_black": on_black,
        "over_line": over_line,
        "drive_forward": drive_forward,
        "Segment": Segment,
        "SideAction": SideAction,
        "DecomposePass": DecomposePass,
        "flatten_steps": flatten_steps,
        "wait_for_seconds": wait_for_seconds,
    }


class _FakeIRSensor:
    """Minimal IRSensor stand-in for ``over_line``."""

    def probabilityOfBlack(self) -> float:
        return 0.0

    def probabilityOfWhite(self) -> float:
        return 1.0


def _segments(nodes, Segment):
    return [n for n in nodes if isinstance(n, Segment)]


def _decompose(steps, imp):
    nodes, _deferred = imp["flatten_steps"](steps)
    return imp["DecomposePass"]().run(nodes)


# ---------------------------------------------------------------------------
# Decompose — after_cm + sensor
# ---------------------------------------------------------------------------


@requires_libstp
def test_after_cm_plus_over_line_splits_in_two():
    imp = _imports()
    sensor = _FakeIRSensor()
    cond = imp["after_cm"](12) + imp["over_line"](sensor)
    nodes = _decompose([imp["drive_forward"]().until(cond)], imp)
    segs = _segments(nodes, imp["Segment"])

    assert len(segs) == 2

    known, rest = segs

    # First leg: the known after_cm distance, promoted to a known endpoint.
    assert known.kind == "linear"
    assert known.axis == imp["LinearAxis"].Forward
    assert known.distance_m == pytest.approx(0.12)
    assert known.has_known_endpoint is True
    assert isinstance(known.condition, imp["after_cm"])
    assert known.condition._absolute is False

    # Second leg: the remaining over_line (a _Then led by a sensor),
    # unknown endpoint.
    assert rest.kind == "linear"
    assert rest.distance_m is None
    assert rest.has_known_endpoint is False
    assert isinstance(rest.condition, imp["_Then"])
    assert not isinstance(rest.condition._first, imp["after_cm"])


@requires_libstp
def test_after_cm_chain_peels_recursively():
    imp = _imports()
    sensor = _FakeIRSensor()
    cond = imp["after_cm"](20) + imp["after_cm"](10) + imp["over_line"](sensor)
    nodes = _decompose([imp["drive_forward"]().until(cond)], imp)
    segs = _segments(nodes, imp["Segment"])

    assert len(segs) == 3

    first, second, third = segs

    # 20 cm known.
    assert first.distance_m == pytest.approx(0.20)
    assert first.has_known_endpoint is True
    assert isinstance(first.condition, imp["after_cm"])

    # 10 cm known — measured from its own segment start.
    assert second.distance_m == pytest.approx(0.10)
    assert second.has_known_endpoint is True
    assert isinstance(second.condition, imp["after_cm"])

    # over_line — unknown endpoint, not decomposed further.
    assert third.distance_m is None
    assert third.has_known_endpoint is False
    assert isinstance(third.condition, imp["_Then"])
    assert not isinstance(third.condition._first, imp["after_cm"])


@requires_libstp
def test_trailing_after_cm_splits_in_two():
    # on_black + after_cm(5) → [sensor(on_black), known(0.05)].  The trailing
    # after_cm measures from where the preceding (sensor) leg ended, so it is a
    # real known distance once split off.
    imp = _imports()
    sensor = _FakeIRSensor()
    cond = imp["on_black"](sensor) + imp["after_cm"](5)
    nodes = _decompose([imp["drive_forward"]().until(cond)], imp)
    segs = _segments(nodes, imp["Segment"])

    assert len(segs) == 2
    sensor_leg, known = segs

    # First leg: the on_black sensor, unknown endpoint, condition is the bare
    # on_black (single-leaf group → no _Then wrapper).
    assert sensor_leg.distance_m is None
    assert sensor_leg.has_known_endpoint is False
    assert isinstance(sensor_leg.condition, imp["on_black"])

    # Second leg: the trailing known 5 cm.
    assert known.distance_m == pytest.approx(0.05)
    assert known.has_known_endpoint is True
    assert isinstance(known.condition, imp["after_cm"])
    assert known.condition._absolute is False


@requires_libstp
def test_interleaved_sensor_and_after_cm_splits_into_four():
    # on_black + after_cm + on_black + after_cm → [sensor, known, sensor, known].
    imp = _imports()
    sensor = _FakeIRSensor()
    cond = (
        imp["on_black"](sensor) + imp["after_cm"](7) + imp["on_black"](sensor) + imp["after_cm"](3)
    )
    nodes = _decompose([imp["drive_forward"]().until(cond)], imp)
    segs = _segments(nodes, imp["Segment"])

    assert len(segs) == 4
    s1, k1, s2, k2 = segs

    assert s1.has_known_endpoint is False
    assert isinstance(s1.condition, imp["on_black"])

    assert k1.distance_m == pytest.approx(0.07)
    assert k1.has_known_endpoint is True
    assert isinstance(k1.condition, imp["after_cm"])

    assert s2.has_known_endpoint is False
    assert isinstance(s2.condition, imp["on_black"])

    assert k2.distance_m == pytest.approx(0.03)
    assert k2.has_known_endpoint is True
    assert isinstance(k2.condition, imp["after_cm"])


@requires_libstp
def test_over_line_grouped_then_trailing_after_cm():
    # over_line + after_cm(5) = _Then(_Then(on_black, on_white), after_cm(5))
    # → [sensor(_Then(on_black, on_white)), known(0.05)].  The two over_line
    # leaves stay grouped as one sensor leg.
    imp = _imports()
    sensor = _FakeIRSensor()
    cond = imp["over_line"](sensor) + imp["after_cm"](5)
    nodes = _decompose([imp["drive_forward"]().until(cond)], imp)
    segs = _segments(nodes, imp["Segment"])

    assert len(segs) == 2
    sensor_leg, known = segs

    # over_line preserved as a grouped _Then sensor leg (on_black then on_white).
    assert sensor_leg.distance_m is None
    assert sensor_leg.has_known_endpoint is False
    assert isinstance(sensor_leg.condition, imp["_Then"])
    assert isinstance(sensor_leg.condition._first, imp["on_black"])
    assert not isinstance(sensor_leg.condition._second, imp["after_cm"])

    # Trailing known 5 cm.
    assert known.distance_m == pytest.approx(0.05)
    assert known.has_known_endpoint is True
    assert isinstance(known.condition, imp["after_cm"])


# ---------------------------------------------------------------------------
# Guards — never decompose when the endpoint isn't actually known
# ---------------------------------------------------------------------------


@requires_libstp
def test_over_line_alone_not_decomposed():
    imp = _imports()
    sensor = _FakeIRSensor()
    # over_line is a _Then led by on_black (a sensor), NOT after_cm.
    cond = imp["over_line"](sensor)
    nodes = _decompose([imp["drive_forward"]().until(cond)], imp)
    segs = _segments(nodes, imp["Segment"])

    assert len(segs) == 1
    seg = segs[0]
    assert seg.distance_m is None
    assert seg.has_known_endpoint is False
    assert isinstance(seg.condition, imp["_Then"])


@requires_libstp
def test_bare_after_cm_unchanged():
    imp = _imports()
    # Bare after_cm — already promoted at lowering, not a _Then; left as one leg.
    nodes = _decompose([imp["drive_forward"]().until(imp["after_cm"](30))], imp)
    segs = _segments(nodes, imp["Segment"])

    assert len(segs) == 1
    seg = segs[0]
    assert seg.distance_m == pytest.approx(0.30)
    assert seg.has_known_endpoint is True
    assert isinstance(seg.condition, imp["after_cm"])


@requires_libstp
def test_plain_drive_and_side_action_untouched():
    imp = _imports()
    nodes = _decompose([imp["drive_forward"](30), imp["wait_for_seconds"](0.1)], imp)
    segs = _segments(nodes, imp["Segment"])

    assert len(segs) == 1
    seg = segs[0]
    assert seg.distance_m == pytest.approx(0.30)
    assert seg.has_known_endpoint is True
    assert seg.condition is None

    # The wait survives as a SideAction, untouched.
    assert any(isinstance(n, imp["SideAction"]) for n in nodes)


# ---------------------------------------------------------------------------
# Integration via the builder — decompose then to_absolute
# ---------------------------------------------------------------------------


@requires_libstp
def test_builder_decompose_then_to_absolute():
    from raccoon.step.motion import AbsoluteHoldMove, GotoWaypoints, optimize
    from raccoon.step.motion.path.compiler import PathCompiler

    imp = _imports()
    sensor = _FakeIRSensor()
    cond = imp["after_cm"](12) + imp["over_line"](sensor)

    # decompose is ALWAYS-ON, so we only chain to_absolute(); the effective
    # pipeline (decompose + merge prepended) still splits the leg first.
    opt = optimize([imp["drive_forward"]().until(cond)]).to_absolute()
    nodes = PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes

    # The known (after_cm 12) leg became a GotoWaypoints SideAction.
    goto_actions = [
        n for n in nodes if isinstance(n, imp["SideAction"]) and isinstance(n.step, GotoWaypoints)
    ]
    assert len(goto_actions) == 1

    # Behavior change: the SENSOR leg (unknown endpoint, over_line condition)
    # used to stay a Segment; to_absolute now converts it into an
    # AbsoluteHoldMove (cross-axis position + heading held absolute, free axis
    # driven until the sensor fires). No Segment survives.
    assert not _segments(nodes, imp["Segment"])
    holds = [
        n.step
        for n in nodes
        if isinstance(n, imp["SideAction"]) and isinstance(n.step, AbsoluteHoldMove)
    ]
    assert len(holds) == 1


@requires_libstp
def test_builder_decompose_trailing_then_to_absolute():
    # Trailing after_cm: on_black + after_cm(20).  decompose splits into a
    # sensor leg (on_black, unknown endpoint) + a known 20 cm leg; to_absolute
    # converts the known leg into a GotoWaypoints SideAction and the sensor leg
    # into an AbsoluteHoldMove. No Segment survives.
    from raccoon.step.motion import AbsoluteHoldMove, GotoWaypoints, optimize
    from raccoon.step.motion.path.compiler import PathCompiler

    imp = _imports()
    sensor = _FakeIRSensor()
    cond = imp["on_black"](sensor) + imp["after_cm"](20)

    opt = optimize([imp["drive_forward"]().until(cond)]).to_absolute()
    nodes = PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes

    # The trailing known (after_cm 20) leg became a GotoWaypoints SideAction.
    goto_actions = [
        n for n in nodes if isinstance(n, imp["SideAction"]) and isinstance(n.step, GotoWaypoints)
    ]
    assert len(goto_actions) == 1

    # The on_black sensor leg became an AbsoluteHoldMove; no Segment survives.
    assert not _segments(nodes, imp["Segment"])
    holds = [
        n.step
        for n in nodes
        if isinstance(n, imp["SideAction"]) and isinstance(n.step, AbsoluteHoldMove)
    ]
    assert len(holds) == 1
