"""Pure-Python unit tests for smooth_path() path optimizer.

Tests the algebraic merge pass, corner-cut pass, spline waypoint
computation, and construction-time error handling.  No simulator or
C++ runtime is required beyond the raccoon package being installed.
"""

from __future__ import annotations

import importlib.util
import math

import pytest


def _libstp_available() -> bool:
    """Check raccoon availability without importing the module."""
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


# ---------------------------------------------------------------------------
# Imports — deferred behind requires_libstp so collection doesn't fail
# ---------------------------------------------------------------------------


def _imports():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path.passes import (
        build_spline_step,
        can_merge,
        merge_two,
        optimize_nodes,
        run_corner_cut,
        run_merge,
        segments_to_spline_waypoints,
        try_corner_arc,
    )
    from raccoon.step.motion.smooth_path import _Segment, _SideAction

    return {
        "LinearAxis": LinearAxis,
        "_Segment": _Segment,
        "_SideAction": _SideAction,
        "_can_merge": can_merge,
        "_merge_two": merge_two,
        "_pass_merge": run_merge,
        "_pass_corner_cut": run_corner_cut,
        "_try_corner_arc": try_corner_arc,
        "_optimize_nodes": optimize_nodes,
        "_segments_to_spline_waypoints": segments_to_spline_waypoints,
        "_build_spline_step": build_spline_step,
    }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _fwd(distance_m: float, speed: float = 1.0, condition=None) -> "_Segment":
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.smooth_path import _Segment

    return _Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        sign=1.0 if distance_m >= 0 else -1.0,
        distance_m=distance_m,
        speed_scale=speed,
        condition=condition,
        has_known_endpoint=condition is None,
    )


def _lat(distance_m: float, speed: float = 1.0) -> "_Segment":
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.smooth_path import _Segment

    return _Segment(
        kind="linear",
        axis=LinearAxis.Lateral,
        sign=1.0 if distance_m >= 0 else -1.0,
        distance_m=distance_m,
        speed_scale=speed,
    )


def _turn(deg: float, speed: float = 1.0) -> "_Segment":
    from raccoon.step.motion.smooth_path import _Segment

    rad = math.radians(deg)
    return _Segment(
        kind="turn",
        sign=1.0 if rad >= 0 else -1.0,
        angle_rad=rad,
        speed_scale=speed,
    )


def _side() -> "_SideAction":
    from raccoon.step.motion.smooth_path import _SideAction

    return _SideAction(step=None, is_background=False)


# ===========================================================================
# Merge pass
# ===========================================================================


class TestPassMerge:
    @requires_libstp
    def test_two_forward_drives_merged(self):
        imp = _imports()
        result = imp["_pass_merge"]([_fwd(0.3), _fwd(0.2)])
        assert len(result) == 1
        assert abs(result[0].distance_m - 0.5) < 1e-9

    @requires_libstp
    def test_three_forward_drives_merged(self):
        imp = _imports()
        result = imp["_pass_merge"]([_fwd(0.1), _fwd(0.2), _fwd(0.3)])
        assert len(result) == 1
        assert abs(result[0].distance_m - 0.6) < 1e-9

    @requires_libstp
    def test_two_turns_merged(self):
        imp = _imports()
        result = imp["_pass_merge"]([_turn(30), _turn(20)])
        assert len(result) == 1
        assert abs(math.degrees(result[0].angle_rad) - 50.0) < 1e-6

    @requires_libstp
    def test_opposite_direction_drives_not_merged(self):
        """Conservative: opposite-direction drives are not collapsed."""
        imp = _imports()
        result = imp["_pass_merge"]([_fwd(0.5), _fwd(-0.3)])
        assert len(result) == 2

    @requires_libstp
    def test_different_axes_not_merged(self):
        imp = _imports()
        result = imp["_pass_merge"]([_fwd(0.3), _lat(0.3)])
        assert len(result) == 2

    @requires_libstp
    def test_drive_turn_drive_not_merged(self):
        imp = _imports()
        result = imp["_pass_merge"]([_fwd(0.3), _turn(45), _fwd(0.2)])
        assert len(result) == 3

    @requires_libstp
    def test_side_action_is_barrier(self):
        imp = _imports()
        result = imp["_pass_merge"]([_fwd(0.3), _side(), _fwd(0.2)])
        assert len(result) == 3
        assert abs(result[0].distance_m - 0.3) < 1e-9
        assert abs(result[2].distance_m - 0.2) < 1e-9

    @requires_libstp
    def test_condition_based_not_merged(self):
        """Segments with conditions are never merged."""
        sentinel = object()  # stand-in for a condition
        imp = _imports()
        cond_seg = _fwd(0.5, condition=sentinel)
        result = imp["_pass_merge"]([cond_seg, _fwd(0.2)])
        assert len(result) == 2

    @requires_libstp
    def test_speed_takes_minimum(self):
        imp = _imports()
        result = imp["_pass_merge"]([_fwd(0.3, speed=1.0), _fwd(0.2, speed=0.5)])
        assert abs(result[0].speed_scale - 0.5) < 1e-9

    @requires_libstp
    def test_heading_mismatch_not_merged(self):
        """Two forward drives with different heading targets must not merge."""
        from raccoon.motion import LinearAxis
        from raccoon.step.motion.smooth_path import _Segment

        imp = _imports()
        seg_a = _Segment(
            kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.3, heading_deg=0.0
        )
        seg_b = _Segment(
            kind="linear", axis=LinearAxis.Forward, sign=1.0, distance_m=0.2, heading_deg=10.0
        )
        result = imp["_pass_merge"]([seg_a, seg_b])
        assert len(result) == 2


# ===========================================================================
# Corner-cut pass
# ===========================================================================


class TestCornerCut:
    @requires_libstp
    def test_90deg_right_corner_cut(self):
        imp = _imports()
        lin1, turn, lin2 = _fwd(0.5), _turn(-90), _fwd(0.4)
        cut = imp["_try_corner_arc"](lin1, turn, lin2, cut_m=0.05)
        assert cut is not None
        new_l1, arc, new_l2 = cut

        assert abs(new_l1.distance_m - 0.45) < 1e-9
        assert abs(new_l2.distance_m - 0.35) < 1e-9
        # R = cut / tan(|θ|/2) = 0.05 / tan(45°) = 0.05
        assert abs(arc.radius_m - 0.05) < 1e-9
        assert abs(arc.arc_angle_rad - math.radians(-90)) < 1e-9
        assert arc.kind == "arc"
        assert arc.lateral is False

    @requires_libstp
    def test_45deg_turn_radius(self):
        imp = _imports()
        lin1, turn, lin2 = _fwd(0.5), _turn(45), _fwd(0.4)
        cut = imp["_try_corner_arc"](lin1, turn, lin2, cut_m=0.05)
        assert cut is not None
        _, arc, _ = cut
        expected_r = 0.05 / math.tan(math.radians(22.5))
        assert abs(arc.radius_m - expected_r) < 1e-6

    @requires_libstp
    def test_leg_too_short_skipped(self):
        imp = _imports()
        lin_short = _fwd(0.03)
        cut = imp["_try_corner_arc"](lin_short, _turn(90), _fwd(0.4), cut_m=0.05)
        assert cut is None

    @requires_libstp
    def test_condition_based_skipped(self):
        sentinel = object()
        imp = _imports()
        cond = _fwd(0.5, condition=sentinel)
        cut = imp["_try_corner_arc"](cond, _turn(90), _fwd(0.4), cut_m=0.05)
        assert cut is None

    @requires_libstp
    def test_different_axes_skipped(self):
        imp = _imports()
        cut = imp["_try_corner_arc"](_fwd(0.5), _turn(90), _lat(0.4), cut_m=0.05)
        assert cut is None

    @requires_libstp
    def test_lateral_arc_for_strafe_legs(self):
        """A strafe+turn+strafe corner produces a lateral arc."""
        imp = _imports()
        cut = imp["_try_corner_arc"](_lat(0.5), _turn(90), _lat(0.4), cut_m=0.05)
        assert cut is not None
        _, arc, _ = cut
        assert arc.lateral is True

    @requires_libstp
    def test_pass_corner_cut_full_sequence(self):
        imp = _imports()
        nodes = [_fwd(0.5), _turn(-90), _fwd(0.4)]
        result = imp["_pass_corner_cut"](nodes, cut_m=0.05)
        assert len(result) == 3
        kinds = [n.kind for n in result]
        assert kinds == ["linear", "arc", "linear"]

    @requires_libstp
    def test_side_action_breaks_pattern(self):
        """A side action between segments must block corner cutting."""
        imp = _imports()
        nodes = [_fwd(0.5), _side(), _turn(-90), _fwd(0.4)]
        result = imp["_pass_corner_cut"](nodes, cut_m=0.05)
        # The turn is at index 2 and the first linear at index 0 —
        # the side action breaks the three-consecutive-segment check.
        assert any(isinstance(n, imp["_SideAction"]) for n in result)
        # No arc should appear since the triple was interrupted
        assert not any(isinstance(n, imp["_Segment"]) and n.kind == "arc" for n in result)


# ===========================================================================
# Spline waypoint computation
# ===========================================================================


class TestSplineWaypoints:
    @requires_libstp
    def test_single_forward_segment(self):
        imp = _imports()
        wps = imp["_segments_to_spline_waypoints"]([_fwd(0.5)])
        assert len(wps) == 1
        fwd_cm, left_cm = wps[0]
        assert abs(fwd_cm - 50.0) < 1e-6
        assert abs(left_cm) < 1e-6

    @requires_libstp
    def test_two_forward_segments(self):
        imp = _imports()
        wps = imp["_segments_to_spline_waypoints"]([_fwd(0.5), _fwd(0.3)])
        assert len(wps) == 2
        assert abs(wps[0][0] - 50.0) < 1e-6
        assert abs(wps[1][0] - 80.0) < 1e-6

    @requires_libstp
    def test_right_turn_then_forward(self):
        """drive(50) + turn_right(90) + drive(30): endpoint at (50, -30) in cm."""
        imp = _imports()
        wps = imp["_segments_to_spline_waypoints"]([_fwd(0.5), _turn(-90), _fwd(0.3)])
        assert len(wps) == 2
        fwd_cm, left_cm = wps[1]
        assert abs(fwd_cm - 50.0) < 1e-4
        assert abs(left_cm - (-30.0)) < 1e-4

    @requires_libstp
    def test_left_turn_then_forward(self):
        """drive(50) + turn_left(90) + drive(30): endpoint at (50, +30) in cm."""
        imp = _imports()
        wps = imp["_segments_to_spline_waypoints"]([_fwd(0.5), _turn(90), _fwd(0.3)])
        assert len(wps) == 2
        fwd_cm, left_cm = wps[1]
        assert abs(fwd_cm - 50.0) < 1e-4
        assert abs(left_cm - 30.0) < 1e-4

    @requires_libstp
    def test_turn_does_not_add_waypoint(self):
        """Consecutive turns produce no waypoints."""
        imp = _imports()
        wps = imp["_segments_to_spline_waypoints"]([_turn(45), _turn(45), _fwd(0.4)])
        assert len(wps) == 1

    @requires_libstp
    def test_180_degree_u_turn(self):
        """drive(50) + turn(180°) + drive(30): heads back, ends at (20, 0)."""
        imp = _imports()
        wps = imp["_segments_to_spline_waypoints"]([_fwd(0.5), _turn(180), _fwd(0.3)])
        assert len(wps) == 2
        fwd_cm, left_cm = wps[1]
        assert abs(fwd_cm - 20.0) < 1e-4  # 50 - 30
        assert abs(left_cm) < 1e-4


# ===========================================================================
# Construction-time validation via smooth_path()
# ===========================================================================


class TestSmoothPathConstruction:
    @requires_libstp
    def test_optimize_merges_in_nodes(self):
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import _Segment, smooth_path

        step = smooth_path(drive_forward(20), drive_forward(30), optimize=True)
        seg_nodes = [n for n in step._nodes if isinstance(n, _Segment)]
        assert len(seg_nodes) == 1
        assert abs(seg_nodes[0].distance_m - 0.5) < 1e-6

    @requires_libstp
    def test_corner_cut_inserts_arc(self):
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import _Segment, smooth_path
        from raccoon.step.motion.turn_dsl import turn_right

        step = smooth_path(
            drive_forward(50),
            turn_right(90),
            drive_forward(40),
            corner_cut_cm=5.0,
        )
        seg_nodes = [n for n in step._nodes if isinstance(n, _Segment)]
        kinds = [s.kind for s in seg_nodes]
        assert kinds == ["linear", "arc", "linear"]
        assert abs(seg_nodes[1].arc_angle_rad - math.radians(-90)) < 1e-6

    @requires_libstp
    def test_side_action_preserved_with_optimize(self):
        """A background() between two drives is kept; the drives are NOT merged."""
        from raccoon.step.logic.background import background
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import _Segment, _SideAction, smooth_path

        class _NullStep:
            def required_resources(self):
                return frozenset()

            def collected_resources(self):
                return frozenset()

        # We need a real non-drive step to pass to background().
        # Use the servo stub approach via dynamic class.
        from raccoon.step.base import Step

        class _NullNonDrive(Step):
            def required_resources(self):
                return frozenset({"servo:0"})

            def _generate_signature(self):
                return "Null()"

            async def _execute_step(self, robot):
                pass

        step = smooth_path(
            drive_forward(20),
            background(_NullNonDrive()),
            drive_forward(20),
            optimize=True,
        )
        seg_count = sum(1 for n in step._nodes if isinstance(n, _Segment))
        action_count = sum(1 for n in step._nodes if isinstance(n, _SideAction))
        assert seg_count == 2, "drives must not be merged across side action"
        assert action_count == 1

    @requires_libstp
    def test_spline_true_stores_spline_step(self):
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import smooth_path
        from raccoon.step.motion.spline_path import SplinePath
        from raccoon.step.motion.turn_dsl import turn_right

        step = smooth_path(drive_forward(50), turn_right(90), drive_forward(30), spline=True)
        assert step._spline_step is not None
        assert isinstance(step._spline_step, SplinePath)

    @requires_libstp
    def test_spline_waypoints_correct(self):
        """Check that spline waypoints match hand-computed expected values."""
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import smooth_path
        from raccoon.step.motion.turn_dsl import turn_right

        step = smooth_path(drive_forward(50), turn_right(90), drive_forward(30), spline=True)
        wps = step._spline_step._waypoints
        assert len(wps) == 2
        # First WP: end of drive(50) — 50cm forward, 0 lateral
        assert abs(wps[0][0] - 50.0) < 1e-4
        assert abs(wps[0][1]) < 1e-4
        # Second WP: after turn right 90° and drive 30 — still 50cm forward, 30cm right
        assert abs(wps[1][0] - 50.0) < 1e-4
        assert abs(wps[1][1] - (-30.0)) < 1e-4

    @requires_libstp
    def test_spline_and_corner_cut_mutually_exclusive(self):
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import smooth_path
        from raccoon.step.motion.turn_dsl import turn_right

        with pytest.raises(ValueError, match="mutually exclusive"):
            smooth_path(
                drive_forward(50),
                turn_right(90),
                drive_forward(30),
                spline=True,
                corner_cut_cm=5.0,
            )

    @requires_libstp
    def test_spline_requires_two_linears(self):
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import smooth_path

        with pytest.raises(ValueError, match="at least 2 linear"):
            smooth_path(drive_forward(50), spline=True)

    @requires_libstp
    def test_spline_rejects_condition_based(self):
        from raccoon.step.condition import Condition
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import smooth_path

        class _AlwaysFalse(Condition):
            def start(self, robot):
                pass

            def check(self, robot) -> bool:
                return False

        with pytest.raises(ValueError, match="condition-based"):
            smooth_path(
                drive_forward(speed=0.5).until(_AlwaysFalse()),
                drive_forward(30),
                spline=True,
            )

    @requires_libstp
    def test_spline_rejects_side_action(self):
        from raccoon.step.base import Step
        from raccoon.step.logic.background import background
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import smooth_path

        class _NullNonDrive(Step):
            def required_resources(self):
                return frozenset({"servo:0"})

            def _generate_signature(self):
                return "Null()"

            async def _execute_step(self, robot):
                pass

        with pytest.raises(ValueError, match="side actions"):
            smooth_path(
                drive_forward(30),
                background(_NullNonDrive()),
                drive_forward(30),
                spline=True,
            )

    @requires_libstp
    def test_spline_rejects_arc_segments(self):
        from raccoon.step.motion.arc import DriveArcRight
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import smooth_path

        with pytest.raises(ValueError, match="arc segments"):
            smooth_path(
                drive_forward(30),
                DriveArcRight(radius_cm=10, degrees=45),
                drive_forward(30),
                spline=True,
            )

    @requires_libstp
    def test_signature_reflects_flags(self):
        from raccoon.step.motion.drive_dsl import drive_forward
        from raccoon.step.motion.smooth_path import smooth_path
        from raccoon.step.motion.turn_dsl import turn_right

        sig_opt = smooth_path(
            drive_forward(30), drive_forward(20), optimize=True
        )._generate_signature()
        assert "opt" in sig_opt

        sig_cut = smooth_path(
            drive_forward(50),
            turn_right(90),
            drive_forward(40),
            corner_cut_cm=5.0,
        )._generate_signature()
        assert "cut=5cm" in sig_cut

        sig_spline = smooth_path(
            drive_forward(50),
            turn_right(90),
            drive_forward(40),
            spline=True,
        )._generate_signature()
        assert "spline" in sig_spline
