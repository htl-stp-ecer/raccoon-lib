"""Tests for always-on known-distance recovery at lowering time.

A ``.until(after_cm(N))`` stop condition stores a KNOWN travel distance.
``recover_known_distance`` (applied unconditionally inside ``extract_segment``)
fills in ``distance_m`` / ``has_known_endpoint`` so the optimizer never sees a
hidden known distance.  It is NOT an opt-in pass — there is no reason to hide a
known distance — so it runs for every lowered segment.

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
    from raccoon.step.condition import after_cm, over_line
    from raccoon.step.motion.drive_dsl import drive_forward, strafe_left
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes import flatten_steps
    from raccoon.step.motion.turn_dsl import turn_right
    from raccoon.step.wait_for_seconds_dsl import wait_for_seconds

    return {
        "LinearAxis": LinearAxis,
        "after_cm": after_cm,
        "over_line": over_line,
        "drive_forward": drive_forward,
        "strafe_left": strafe_left,
        "turn_right": turn_right,
        "Segment": Segment,
        "SideAction": SideAction,
        "flatten_steps": flatten_steps,
        "wait_for_seconds": wait_for_seconds,
    }


class _FakeIRSensor:
    """Minimal IRSensor stand-in for ``over_line`` / ``on_black``."""

    def probabilityOfBlack(self) -> float:
        return 0.0

    def probabilityOfWhite(self) -> float:
        return 1.0


def _segments(nodes, Segment):
    return [n for n in nodes if isinstance(n, Segment)]


def _lower(steps, imp):
    nodes, _deferred = imp["flatten_steps"](steps)
    return nodes


# ---------------------------------------------------------------------------
# Promotion — bare relative after_cm (always-on, no pass)
# ---------------------------------------------------------------------------


@requires_libstp
def test_drive_after_cm_promoted_at_lowering():
    imp = _imports()
    seg = _segments(
        _lower([imp["drive_forward"]().until(imp["after_cm"](30))], imp), imp["Segment"]
    )[0]
    assert seg.kind == "linear"
    assert seg.axis == imp["LinearAxis"].Forward
    assert seg.distance_m == pytest.approx(0.30)
    assert seg.has_known_endpoint is True
    assert seg.condition is not None  # runtime odometer stop kept


@requires_libstp
def test_strafe_after_cm_promoted_with_direction():
    imp = _imports()
    seg = _segments(
        _lower([imp["strafe_left"](heading=0).until(imp["after_cm"](12))], imp), imp["Segment"]
    )[0]
    assert seg.axis == imp["LinearAxis"].Lateral
    assert seg.distance_m == pytest.approx(-0.12)
    assert seg.has_known_endpoint is True


# ---------------------------------------------------------------------------
# Guards — never promote when the endpoint isn't actually known
# ---------------------------------------------------------------------------


@requires_libstp
def test_combined_condition_not_promoted():
    imp = _imports()
    cond = imp["after_cm"](1) + imp["over_line"](_FakeIRSensor())
    seg = _segments(_lower([imp["drive_forward"]().until(cond)], imp), imp["Segment"])[0]
    assert seg.has_known_endpoint is False
    assert seg.distance_m is None


@requires_libstp
def test_absolute_after_cm_not_promoted():
    imp = _imports()
    seg = _segments(
        _lower([imp["drive_forward"]().until(imp["after_cm"](12, absolute=True))], imp),
        imp["Segment"],
    )[0]
    assert seg.has_known_endpoint is False


@requires_libstp
def test_plain_drive_unchanged():
    imp = _imports()
    seg = _segments(_lower([imp["drive_forward"](30)], imp), imp["Segment"])[0]
    assert seg.distance_m == pytest.approx(0.30)
    assert seg.has_known_endpoint is True
    assert seg.condition is None


@requires_libstp
def test_side_action_untouched():
    imp = _imports()
    nodes = _lower(
        [imp["drive_forward"]().until(imp["after_cm"](20)), imp["wait_for_seconds"](0.1)], imp
    )
    assert any(isinstance(n, imp["SideAction"]) for n in nodes)
