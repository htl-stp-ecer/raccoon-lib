"""Unit tests for the ``to_absolute`` optimizer pass.

``ToAbsolutePass`` converts maximal runs of known-endpoint ``linear`` /
``turn`` segments (no live condition) into inline ``SideAction(goto_relative)``
nodes — one navigate-to-pose leg per linear endpoint.  Turns fold into the next
leg's target heading; non-qualifying nodes (side actions, sensor conditions,
arcs, unknown endpoints) pass through untouched.

No simulator or C++ runtime is required beyond the raccoon package.
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


def _imports():
    from raccoon.step.condition import after_cm, over_line
    from raccoon.step.motion import goto_relative
    from raccoon.step.motion.arc_dsl import drive_arc_left
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.goto import GotoRelative
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.optimize import optimize
    from raccoon.step.motion.path.passes import ToAbsolutePass, flatten_steps
    from raccoon.step.motion.turn_dsl import turn_right
    from raccoon.step.wait_for_seconds_dsl import wait_for_seconds

    return {
        "after_cm": after_cm,
        "over_line": over_line,
        "drive_forward": drive_forward,
        "drive_arc_left": drive_arc_left,
        "turn_right": turn_right,
        "goto_relative": goto_relative,
        "GotoRelative": GotoRelative,
        "Segment": Segment,
        "SideAction": SideAction,
        "optimize": optimize,
        "ToAbsolutePass": ToAbsolutePass,
        "flatten_steps": flatten_steps,
        "wait_for_seconds": wait_for_seconds,
    }


class _FakeIRSensor:
    """Minimal IRSensor stand-in for ``over_line``."""

    def probabilityOfBlack(self) -> float:
        return 0.0

    def probabilityOfWhite(self) -> float:
        return 1.0


def _run_pass(steps, imp):
    nodes, _deferred = imp["flatten_steps"](steps)
    return imp["ToAbsolutePass"]().run(nodes)


def _goto_legs(nodes, imp):
    """Return the GotoRelative steps wrapped in inline SideActions, in order."""
    legs = []
    for n in nodes:
        if isinstance(n, imp["SideAction"]):
            assert n.is_background is False
            if isinstance(n.step, imp["GotoRelative"]):
                legs.append(n.step)
    return legs


# ---------------------------------------------------------------------------
# Basic conversion — two forward legs
# ---------------------------------------------------------------------------


class TestForwardRun:
    @requires_libstp
    def test_two_forward_legs(self):
        imp = _imports()
        nodes = _run_pass([imp["drive_forward"](50), imp["drive_forward"](30)], imp)
        # No Segment survives; exactly two inline goto_relative side actions.
        assert not any(isinstance(n, imp["Segment"]) for n in nodes)
        legs = _goto_legs(nodes, imp)
        assert len(legs) == 2
        # Body-frame deltas: cumulative 0.5 m then 0.8 m forward, 0 left.
        assert legs[0]._forward_m == pytest.approx(0.5)
        assert legs[0]._left_m == pytest.approx(0.0, abs=1e-9)
        assert legs[0]._dtheta_rad == pytest.approx(0.0, abs=1e-9)
        assert legs[1]._forward_m == pytest.approx(0.8)
        assert legs[1]._left_m == pytest.approx(0.0, abs=1e-9)
        assert legs[1]._dtheta_rad == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Run with a turn — heading folds into the following leg
# ---------------------------------------------------------------------------


class TestRunWithTurn:
    @requires_libstp
    def test_turn_between_linears(self):
        imp = _imports()
        nodes = _run_pass(
            [
                imp["drive_forward"](50),
                imp["turn_right"](90),
                imp["drive_forward"](30),
            ],
            imp,
        )
        legs = _goto_legs(nodes, imp)
        assert len(legs) == 2

        # Leg 1: 0.5 m forward, heading still 0.
        assert legs[0]._forward_m == pytest.approx(0.5)
        assert legs[0]._left_m == pytest.approx(0.0, abs=1e-9)
        assert legs[0]._dtheta_rad == pytest.approx(0.0, abs=1e-9)

        # turn_right(90) -> heading -90° (CW). The second 0.3 m forward leg is
        # displaced from the run start by 0.5 m forward + 0.3 m along the -90°
        # direction (= 0.3 m to the right, i.e. body-left = -0.3 m).
        assert legs[1]._forward_m == pytest.approx(0.5)
        assert legs[1]._left_m == pytest.approx(-0.3)
        assert legs[1]._dtheta_rad == pytest.approx(math.radians(-90.0))


# ---------------------------------------------------------------------------
# known_distance gating — after_cm only qualifies once promoted
# ---------------------------------------------------------------------------


class TestKnownDistanceGating:
    @requires_libstp
    def test_until_after_cm_needs_known_distance_first(self):
        imp = _imports()
        from raccoon.step.motion.path.compiler import PathCompiler

        def _build():
            return [imp["drive_forward"]().until(imp["after_cm"](20))]

        # Without known_distance(): the condition segment is unknown-endpoint,
        # so to_absolute leaves it as a Segment.
        opt = imp["optimize"](_build()).to_absolute()
        nodes = PathCompiler(opt._passes).compile(opt._raw_steps).nodes
        assert any(isinstance(n, imp["Segment"]) for n in nodes)
        assert not _goto_legs(nodes, imp)

        # With known_distance() first: the segment is promoted to a known
        # endpoint and to_absolute converts it.
        opt2 = imp["optimize"](_build()).known_distance().to_absolute()
        nodes2 = PathCompiler(opt2._passes).compile(opt2._raw_steps).nodes
        legs = _goto_legs(nodes2, imp)
        assert len(legs) == 1
        assert legs[0]._forward_m == pytest.approx(0.20)


# ---------------------------------------------------------------------------
# Non-qualifying nodes pass through untouched
# ---------------------------------------------------------------------------


class TestPassThrough:
    @requires_libstp
    def test_side_action_untouched(self):
        imp = _imports()
        nodes = _run_pass([imp["wait_for_seconds"](1)], imp)
        # The wait stays a SideAction; no goto_relative legs were produced.
        assert any(isinstance(n, imp["SideAction"]) for n in nodes)
        assert not _goto_legs(nodes, imp)

    @requires_libstp
    def test_sensor_condition_segment_untouched(self):
        imp = _imports()
        sensor = _FakeIRSensor()
        step = imp["drive_forward"]().until(imp["over_line"](sensor))
        nodes = _run_pass([step], imp)
        # Live sensor condition -> unknown endpoint -> stays a Segment.
        segs = [n for n in nodes if isinstance(n, imp["Segment"])]
        assert len(segs) == 1
        assert segs[0].condition is not None
        assert not _goto_legs(nodes, imp)

    @requires_libstp
    def test_arc_segment_untouched(self):
        imp = _imports()
        step = imp["drive_arc_left"](radius_cm=20, degrees=90)
        nodes = _run_pass([step], imp)
        segs = [n for n in nodes if isinstance(n, imp["Segment"])]
        assert len(segs) == 1
        assert segs[0].kind == "arc"
        assert not _goto_legs(nodes, imp)

    @requires_libstp
    def test_side_action_splits_runs(self):
        imp = _imports()
        # A wait between two forward legs breaks the run; each side becomes its
        # own single-leg conversion, with the wait preserved in between.
        nodes = _run_pass(
            [
                imp["drive_forward"](50),
                imp["wait_for_seconds"](1),
                imp["drive_forward"](30),
            ],
            imp,
        )
        legs = _goto_legs(nodes, imp)
        assert len(legs) == 2
        # Both anchor at their own run start: each is 0.5 / 0.3 m forward.
        assert legs[0]._forward_m == pytest.approx(0.5)
        assert legs[1]._forward_m == pytest.approx(0.3)
        # The wait survived as a side action between them.
        assert any(
            isinstance(n, imp["SideAction"]) and not isinstance(n.step, imp["GotoRelative"])
            for n in nodes
        )


# ---------------------------------------------------------------------------
# Integration — explain()
# ---------------------------------------------------------------------------


class TestIntegration:
    @requires_libstp
    def test_explain_reports_to_absolute(self):
        imp = _imports()
        opt = imp["optimize"]([imp["drive_forward"](50), imp["drive_forward"](30)]).to_absolute()
        text = opt.explain()
        assert "to_absolute" in text
        assert "GotoRelative" in text
