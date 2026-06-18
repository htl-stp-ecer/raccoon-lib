"""Unit tests for the ``to_absolute`` optimizer pass.

``ToAbsolutePass`` converts each maximal run of known-endpoint ``linear`` /
``turn`` segments (no live condition) into ONE inline
``SideAction(GotoWaypoints)`` node carrying one body-frame waypoint per linear
endpoint.  The single step captures one anchor per run and drives to the
resulting absolute world targets in sequence.  Turns fold into the next
waypoint's target heading; non-qualifying nodes (side actions, sensor
conditions, arcs, unknown endpoints) pass through untouched.

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
    from raccoon.step.motion import GotoWaypoints
    from raccoon.step.motion.arc_dsl import drive_arc_left
    from raccoon.step.motion.drive_dsl import drive_forward
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
        "GotoWaypoints": GotoWaypoints,
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


def _goto_steps(nodes, imp):
    """Return the GotoWaypoints steps wrapped in inline SideActions, in order."""
    steps = []
    for n in nodes:
        if isinstance(n, imp["SideAction"]):
            assert n.is_background is False
            if isinstance(n.step, imp["GotoWaypoints"]):
                steps.append(n.step)
    return steps


# ---------------------------------------------------------------------------
# Basic conversion — two forward legs
# ---------------------------------------------------------------------------


class TestForwardRun:
    @requires_libstp
    def test_two_forward_legs(self):
        imp = _imports()
        nodes = _run_pass([imp["drive_forward"](50), imp["drive_forward"](30)], imp)
        # No Segment survives; exactly ONE inline GotoWaypoints side action with
        # two waypoints.
        assert not any(isinstance(n, imp["Segment"]) for n in nodes)
        steps = _goto_steps(nodes, imp)
        assert len(steps) == 1
        wps = steps[0]._waypoints
        assert len(wps) == 2
        # Body-frame deltas: cumulative 0.5 m then 0.8 m forward, 0 left.
        assert wps[0][0] == pytest.approx(0.5)
        assert wps[0][1] == pytest.approx(0.0, abs=1e-9)
        assert wps[0][2] == pytest.approx(0.0, abs=1e-9)
        assert wps[1][0] == pytest.approx(0.8)
        assert wps[1][1] == pytest.approx(0.0, abs=1e-9)
        assert wps[1][2] == pytest.approx(0.0, abs=1e-9)


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
        # The whole contiguous run becomes ONE GotoWaypoints with TWO waypoints.
        steps = _goto_steps(nodes, imp)
        assert len(steps) == 1
        wps = steps[0]._waypoints
        assert len(wps) == 2

        # Waypoint 1: 0.5 m forward, heading still 0.
        assert wps[0][0] == pytest.approx(0.5)
        assert wps[0][1] == pytest.approx(0.0, abs=1e-9)
        assert wps[0][2] == pytest.approx(0.0, abs=1e-9)

        # turn_right(90) -> heading -90° (CW). The second 0.3 m forward leg is
        # the post-turn displacement: 0.5 m forward + 0.3 m along the -90°
        # direction (= 0.3 m to the right, i.e. body-left = -0.3 m), measured
        # from the SINGLE run-start anchor (not chained).
        assert wps[1][0] == pytest.approx(0.5)
        assert wps[1][1] == pytest.approx(-0.3)
        assert wps[1][2] == pytest.approx(math.radians(-90.0))


# ---------------------------------------------------------------------------
# known_distance gating — after_cm only qualifies once promoted
# ---------------------------------------------------------------------------


class TestKnownDistanceGating:
    @requires_libstp
    def test_until_after_cm_converts_without_extra_pass(self):
        """after_cm distance is recovered at lowering, so to_absolute converts it."""
        imp = _imports()
        from raccoon.step.motion.path.compiler import PathCompiler

        opt = imp["optimize"]([imp["drive_forward"]().until(imp["after_cm"](20))]).to_absolute()
        nodes = PathCompiler(opt._passes).compile(opt._raw_steps).nodes
        steps = _goto_steps(nodes, imp)
        assert len(steps) == 1
        wps = steps[0]._waypoints
        assert len(wps) == 1
        assert wps[0][0] == pytest.approx(0.20)


# ---------------------------------------------------------------------------
# Non-qualifying nodes pass through untouched
# ---------------------------------------------------------------------------


class TestPassThrough:
    @requires_libstp
    def test_side_action_untouched(self):
        imp = _imports()
        nodes = _run_pass([imp["wait_for_seconds"](1)], imp)
        # The wait stays a SideAction; no GotoWaypoints steps were produced.
        assert any(isinstance(n, imp["SideAction"]) for n in nodes)
        assert not _goto_steps(nodes, imp)

    @requires_libstp
    def test_sensor_linear_leg_becomes_absolute_hold_move(self):
        """Behavior change: a SENSOR-bounded single-axis linear leg used to stay
        a Segment; to_absolute now converts it into an inline
        ``SideAction(AbsoluteHoldMove)`` (2 DOF held absolute, free axis until
        the sensor)."""
        from raccoon.step.motion import AbsoluteHoldMove

        imp = _imports()
        sensor = _FakeIRSensor()
        step = imp["drive_forward"]().until(imp["over_line"](sensor))
        nodes = _run_pass([step], imp)
        # No Segment survives; one inline AbsoluteHoldMove side action.
        assert not any(isinstance(n, imp["Segment"]) for n in nodes)
        holds = [
            n.step
            for n in nodes
            if isinstance(n, imp["SideAction"]) and isinstance(n.step, AbsoluteHoldMove)
        ]
        assert len(holds) == 1
        assert not _goto_steps(nodes, imp)

    @requires_libstp
    def test_arc_segment_untouched(self):
        imp = _imports()
        step = imp["drive_arc_left"](radius_cm=20, degrees=90)
        nodes = _run_pass([step], imp)
        segs = [n for n in nodes if isinstance(n, imp["Segment"])]
        assert len(segs) == 1
        assert segs[0].kind == "arc"
        assert not _goto_steps(nodes, imp)

    @requires_libstp
    def test_side_action_splits_runs(self):
        imp = _imports()
        # A wait between two forward legs breaks the run; each side becomes its
        # own single-waypoint GotoWaypoints, with the wait preserved in between.
        nodes = _run_pass(
            [
                imp["drive_forward"](50),
                imp["wait_for_seconds"](1),
                imp["drive_forward"](30),
            ],
            imp,
        )
        steps = _goto_steps(nodes, imp)
        assert len(steps) == 2
        # Each is its own run with one waypoint, anchored at its own run start:
        # 0.5 / 0.3 m forward.
        assert len(steps[0]._waypoints) == 1
        assert len(steps[1]._waypoints) == 1
        assert steps[0]._waypoints[0][0] == pytest.approx(0.5)
        assert steps[1]._waypoints[0][0] == pytest.approx(0.3)
        # The wait survived as a side action between them.
        assert any(
            isinstance(n, imp["SideAction"]) and not isinstance(n.step, imp["GotoWaypoints"])
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
        assert "GotoWaypoints" in text
