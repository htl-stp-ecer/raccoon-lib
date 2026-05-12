from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from raccoon.motion import LinearAxis
from raccoon.step.motion.arc_dsl import drive_arc_left
from raccoon.step.motion.drive_dsl import drive_forward
from raccoon.step.motion.path import Goto, Resync, TurnTo
from raccoon.step.motion.path.executor import (
    _apply_resync,
    _current_runtime_pose,
    _lower_absolute_goto,
    _lower_absolute_goto_segments,
    _lower_absolute_turn,
)
from raccoon.step.motion.smooth_path import smooth_path
from raccoon.step.motion.turn_dsl import turn_left


def _pose(x_m: float, y_m: float, heading_rad: float):
    return SimpleNamespace(position=[x_m, y_m, 0.0], heading=heading_rad)


def test_lower_absolute_turn_uses_current_world_heading() -> None:
    seg = _lower_absolute_turn(math.radians(15.0), TurnTo(theta_rad=math.radians(90.0)))

    assert seg.kind == "turn"
    assert seg.angle_rad == pytest.approx(math.radians(75.0))
    assert seg.target_heading_rad == pytest.approx(math.radians(90.0))


def test_lower_absolute_forward_goto_builds_linear_segment() -> None:
    seg = _lower_absolute_goto(
        _pose(1.0, 2.0, math.pi / 2.0),
        Goto(x_m=1.0, y_m=2.3, theta_rad=math.pi / 2.0, via="forward", speed_scale=0.5),
    )

    assert seg.kind == "linear"
    assert seg.axis == LinearAxis.Forward
    assert seg.distance_m == pytest.approx(0.3)
    assert seg.target_heading_rad == pytest.approx(math.pi / 2.0)
    assert seg.speed_scale == pytest.approx(0.5)


def test_lower_absolute_arc_goto_reconstructs_arc_segment() -> None:
    seg = _lower_absolute_goto(
        _pose(0.0, 0.0, 0.0),
        Goto(x_m=0.3, y_m=0.3, theta_rad=math.pi / 2.0, via="arc"),
    )

    assert seg.kind == "arc"
    assert seg.radius_m == pytest.approx(0.3, abs=1e-6)
    assert seg.arc_angle_rad == pytest.approx(math.pi / 2.0)
    assert seg.lateral is False


def test_lower_absolute_forward_goto_splits_non_collinear_world_target() -> None:
    segments = _lower_absolute_goto_segments(
        _pose(0.0, 0.0, 0.0),
        Goto(x_m=0.3, y_m=0.1, theta_rad=0.0, via="forward"),
    )

    assert [seg.kind for seg in segments] == ["turn", "linear", "turn"]
    assert segments[1].distance_m == pytest.approx(math.hypot(0.3, 0.1))
    assert segments[2].target_heading_rad == pytest.approx(0.0)


def test_apply_resync_observes_localization_with_snap_mask() -> None:
    calls = []

    class _Localization:
        def get_pose(self):
            return _pose(0.2, 0.4, 1.0)

        def observe(self, observation):
            calls.append(observation)

    robot = SimpleNamespace(localization=_Localization())

    _apply_resync(
        robot,
        Resync(
            method="io_button",
            expected_x_m=1.2,
            expected_theta_rad=0.5,
            snap_axes=(True, False, True),
        ),
    )

    assert len(calls) == 1
    obs = calls[0]
    assert float(obs.pose.position[0]) == pytest.approx(1.2)
    assert float(obs.pose.position[1]) == pytest.approx(0.4)
    assert float(obs.pose.heading) == pytest.approx(0.5)
    assert tuple(obs.sigma) == pytest.approx((1e-3, math.inf, 1e-3))


def test_current_runtime_pose_prefers_odometry_position_over_stale_localization() -> None:
    robot = SimpleNamespace(
        odometry=SimpleNamespace(get_pose=lambda: _pose(0.4, 0.0, 0.0)),
        localization=SimpleNamespace(get_pose=lambda: _pose(0.1, 0.2, 0.0)),
    )

    pose = _current_runtime_pose(robot)

    assert float(pose.position[0]) == pytest.approx(0.4)
    assert float(pose.position[1]) == pytest.approx(0.0)


def test_workspace_smooth_path_uses_absolute_runtime_for_simple_path() -> None:
    step = smooth_path(turn_left(90), drive_forward(30))

    assert step._absolute_plan is not None
    assert step._absolute_bridge_fallback_reason is None
    assert step._plan.passes_applied[:2] == [
        "absolute:relative_desugar",
        "absolute:validate_reachable",
    ]


def test_workspace_smooth_path_keeps_absolute_runtime_for_arc_path() -> None:
    step = smooth_path(drive_forward(20), drive_arc_left(radius_cm=20, degrees=90))

    assert step._absolute_plan is not None
    assert step._absolute_bridge_fallback_reason is None
