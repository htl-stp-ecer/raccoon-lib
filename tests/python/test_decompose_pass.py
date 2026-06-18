"""Tests for the ``decompose`` pass.

``DecomposePass`` splits a conditional ``linear`` / ``follow_line`` leg whose
stop condition is a sequential ``_Then`` LED by a bare relative ``after_cm``
into a known-distance leg + the remaining-condition leg, so the known part
becomes optimizable (absolutizable / splinifiable).  A chain of leading
``after_cm`` conditions peels recursively.

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
    from raccoon.step.condition import _Then, after_cm, over_line
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes import DecomposePass, flatten_steps
    from raccoon.step.wait_for_seconds_dsl import wait_for_seconds

    return {
        "LinearAxis": LinearAxis,
        "_Then": _Then,
        "after_cm": after_cm,
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
    from raccoon.step.motion import GotoWaypoints, optimize
    from raccoon.step.motion.path.compiler import PathCompiler

    imp = _imports()
    sensor = _FakeIRSensor()
    cond = imp["after_cm"](12) + imp["over_line"](sensor)

    opt = optimize([imp["drive_forward"]().until(cond)]).decompose().to_absolute()
    nodes = PathCompiler(opt._passes).compile(opt._raw_steps).nodes

    # The known leg became a GotoWaypoints SideAction.
    goto_actions = [
        n for n in nodes if isinstance(n, imp["SideAction"]) and isinstance(n.step, GotoWaypoints)
    ]
    assert len(goto_actions) == 1

    # The sensor leg stayed a Segment (unknown endpoint, over_line condition).
    segs = _segments(nodes, imp["Segment"])
    assert len(segs) == 1
    assert segs[0].has_known_endpoint is False
    assert isinstance(segs[0].condition, imp["_Then"])
