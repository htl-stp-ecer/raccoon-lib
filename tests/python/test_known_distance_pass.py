"""Unit tests for the ``known_distance`` optimizer pass.

The pass recovers the KNOWN travel distance hidden inside a
``.until(after_cm(N))`` stop condition: lowering marks such a segment as
``distance_m=None, has_known_endpoint=False`` (it never inspects the
condition), so geometry passes can't act on it.  ``KnownDistancePass``
promotes a bare relative ``after_cm`` segment to a known-endpoint segment
while leaving the runtime condition in place.

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


# ---------------------------------------------------------------------------
# Imports — deferred behind requires_libstp so collection doesn't fail
# ---------------------------------------------------------------------------


def _imports():
    from raccoon.step.condition import after_cm, over_line
    from raccoon.step.motion.drive_dsl import drive_forward, strafe_left
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.optimize import optimize
    from raccoon.step.motion.path.passes import (
        KnownDistancePass,
        MergePass,
        flatten_steps,
    )
    from raccoon.step.wait_for_seconds_dsl import wait_for_seconds

    return {
        "after_cm": after_cm,
        "over_line": over_line,
        "drive_forward": drive_forward,
        "strafe_left": strafe_left,
        "Segment": Segment,
        "SideAction": SideAction,
        "optimize": optimize,
        "KnownDistancePass": KnownDistancePass,
        "MergePass": MergePass,
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


def _run_pass(steps, imp):
    """Lower ``steps`` to IR and run the pass; return the node list."""
    nodes, _deferred = imp["flatten_steps"](steps)
    return imp["KnownDistancePass"]().run(nodes)


# ---------------------------------------------------------------------------
# Promotion — bare relative after_cm
# ---------------------------------------------------------------------------


class TestPromotesBareAfterCm:
    @requires_libstp
    def test_strafe_left_lateral_promoted(self):
        imp = _imports()
        from raccoon.motion import LinearAxis

        step = imp["strafe_left"](heading=0).until(imp["after_cm"](12))
        nodes = _run_pass([step], imp)
        segs = _segments(nodes, imp["Segment"])
        assert len(segs) == 1
        seg = segs[0]
        # strafe_left has sign -1, so distance is -0.12 m.
        assert abs(seg.distance_m - (-0.12)) < 1e-9
        assert seg.has_known_endpoint is True
        assert seg.axis == LinearAxis.Lateral
        # Condition is preserved as the canonical runtime stop.
        assert seg.condition is not None
        assert isinstance(seg.condition, imp["after_cm"])

    @requires_libstp
    def test_drive_forward_forward_promoted(self):
        imp = _imports()
        from raccoon.motion import LinearAxis

        step = imp["drive_forward"]().until(imp["after_cm"](30))
        nodes = _run_pass([step], imp)
        segs = _segments(nodes, imp["Segment"])
        assert len(segs) == 1
        seg = segs[0]
        # drive_forward has sign +1.
        assert abs(seg.distance_m - 0.30) < 1e-9
        assert seg.has_known_endpoint is True
        assert seg.axis == LinearAxis.Forward
        assert seg.condition is not None


# ---------------------------------------------------------------------------
# Guards — must NOT promote
# ---------------------------------------------------------------------------


class TestGuards:
    @requires_libstp
    def test_combined_condition_not_promoted(self):
        imp = _imports()
        sensor = _FakeIRSensor()
        # after_cm(1) + over_line(sensor) is a _Then (combined) condition:
        # it may fire EARLY, so the endpoint is NOT known.
        combined = imp["after_cm"](1) + imp["over_line"](sensor)
        # Sanity: the combinator does not produce an after_cm instance.
        assert not isinstance(combined, imp["after_cm"])

        step = imp["drive_forward"]().until(combined)
        nodes = _run_pass([step], imp)
        seg = _segments(nodes, imp["Segment"])[0]
        assert seg.has_known_endpoint is False
        assert seg.distance_m is None

    @requires_libstp
    def test_absolute_after_cm_not_promoted(self):
        imp = _imports()
        step = imp["drive_forward"]().until(imp["after_cm"](12, absolute=True))
        nodes = _run_pass([step], imp)
        seg = _segments(nodes, imp["Segment"])[0]
        assert seg.has_known_endpoint is False
        assert seg.distance_m is None


# ---------------------------------------------------------------------------
# Untouched — already-known segments and side actions
# ---------------------------------------------------------------------------


class TestUntouched:
    @requires_libstp
    def test_plain_drive_already_known_unchanged(self):
        imp = _imports()
        step = imp["drive_forward"](30)
        nodes = _run_pass([step], imp)
        seg = _segments(nodes, imp["Segment"])[0]
        assert seg.has_known_endpoint is True
        assert abs(seg.distance_m - 0.30) < 1e-9
        # No condition was attached, none invented.
        assert seg.condition is None

    @requires_libstp
    def test_side_action_untouched(self):
        imp = _imports()
        step = imp["wait_for_seconds"](1)
        nodes = _run_pass([step], imp)
        # The wait is a side action, not a segment — pass leaves it alone.
        assert any(isinstance(n, imp["SideAction"]) for n in nodes)
        assert not _segments(nodes, imp["Segment"])


# ---------------------------------------------------------------------------
# Integration — pass unlocks downstream geometry reasoning
# ---------------------------------------------------------------------------


class TestIntegration:
    @requires_libstp
    def test_known_distance_then_merge(self):
        imp = _imports()

        def _build():
            return [
                imp["drive_forward"]().until(imp["after_cm"](20)),
                imp["drive_forward"]().until(imp["after_cm"](10)),
            ]

        # Before known_distance: both segments are condition-only, unknown
        # endpoint, so they cannot be reasoned about geometrically.
        from raccoon.step.motion.path.compiler import PathCompiler

        opt = imp["optimize"](_build()).known_distance().merge()
        nodes = PathCompiler(opt._passes).compile(opt._raw_steps).nodes
        segs = _segments(nodes, imp["Segment"])

        # known_distance promotes BOTH segments to known endpoints with the
        # recovered distances.
        assert all(s.has_known_endpoint for s in segs)
        promoted = sorted(s.distance_m for s in segs)
        assert abs(promoted[0] - 0.10) < 1e-9
        assert abs(promoted[1] - 0.20) < 1e-9

        # Limitation: MergePass refuses to combine segments that still carry a
        # stop condition (``can_merge`` rejects any conditional segment), so the
        # two known-endpoint conditional drives remain two segments. The
        # promotion is still what unlocks geometry passes that DON'T require a
        # null condition (e.g. to_absolute waypoint reasoning).
        assert len(segs) == 2
        assert all(s.condition is not None for s in segs)

    @requires_libstp
    def test_explain_reports_known_endpoint(self):
        imp = _imports()
        opt = imp["optimize"]([imp["drive_forward"]().until(imp["after_cm"](20))]).known_distance()
        text = opt.explain()
        assert "known_distance" in text
        assert "has_known_endpoint=True" in text
