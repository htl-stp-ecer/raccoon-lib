"""Tests for Phase-5 relative DSL desugaring into absolute IR."""

from __future__ import annotations

import math

import pytest

from raccoon.step import background, run
from raccoon.step.condition import StopCondition
from raccoon.step.motion import (
    drive_arc_left,
    drive_forward,
    strafe_left,
    turn_left,
    turn_right,
)
from raccoon.step.motion.path import (
    Action,
    CompileError,
    Goto,
    IntendedPose,
    TurnTo,
    absolute_to_relative_nodes,
    compile_relative_to_absolute,
)
from raccoon.step.motion.path.ir import Segment, SideAction


class _Map:
    width_cm = 100.0
    height_cm = 100.0


class _Never(StopCondition):
    def check(self, robot) -> bool:
        return False


def test_relative_drive_turn_drive_desugars_to_absolute_pose_chain() -> None:
    plan = compile_relative_to_absolute(
        [drive_forward(70), turn_left(90), drive_forward(50)],
        fold_turns=False,
    )

    assert plan.passes_applied == ("relative_desugar", "validate_reachable")
    assert len(plan.nodes) == 3
    first, turn, second = plan.nodes

    assert isinstance(first, Goto)
    assert first.x_m == pytest.approx(0.70)
    assert first.y_m == pytest.approx(0.0)
    assert first.theta_rad == pytest.approx(0.0)

    assert isinstance(turn, TurnTo)
    assert turn.theta_rad == pytest.approx(math.pi / 2.0)

    assert isinstance(second, Goto)
    assert second.x_m == pytest.approx(0.70)
    assert second.y_m == pytest.approx(0.50)
    assert second.theta_rad == pytest.approx(math.pi / 2.0)


def test_fold_implicit_turns_removes_redundant_turn_before_goto() -> None:
    plan = compile_relative_to_absolute([turn_left(90), drive_forward(50)])

    assert plan.passes_applied == (
        "relative_desugar",
        "validate_reachable",
        "fold_implicit_turns",
    )
    assert len(plan.nodes) == 1
    node = plan.nodes[0]
    assert isinstance(node, Goto)
    assert node.x_m == pytest.approx(0.0)
    assert node.y_m == pytest.approx(0.50)
    assert node.theta_rad == pytest.approx(math.pi / 2.0)


def test_action_barrier_prevents_turn_folding() -> None:
    plan = compile_relative_to_absolute(
        [turn_left(90), run(lambda: None), drive_forward(50)],
    )

    assert len(plan.nodes) == 3
    assert isinstance(plan.nodes[0], TurnTo)
    assert isinstance(plan.nodes[1], Action)
    assert isinstance(plan.nodes[2], Goto)


def test_background_step_becomes_non_blocking_action() -> None:
    plan = compile_relative_to_absolute(
        [drive_forward(20), background(run(lambda: None)), drive_forward(20)],
    )

    assert len(plan.nodes) == 3
    action = plan.nodes[1]
    assert isinstance(action, Action)
    assert action.blocking is False


def test_lateral_motion_uses_body_right_coordinate_convention() -> None:
    plan = compile_relative_to_absolute([strafe_left(20)], fold_turns=False)

    node = plan.nodes[0]
    assert isinstance(node, Goto)
    assert node.x_m == pytest.approx(0.0)
    assert node.y_m == pytest.approx(0.20)
    assert node.via == "lateral"


def test_arc_endpoint_matches_motion_simulation_delta() -> None:
    plan = compile_relative_to_absolute([drive_arc_left(radius_cm=30, degrees=90)])

    node = plan.nodes[0]
    assert isinstance(node, Goto)
    assert node.x_m == pytest.approx(0.30)
    assert node.y_m == pytest.approx(0.30)
    assert node.theta_rad == pytest.approx(math.pi / 2.0)
    assert node.via == "arc"


def test_start_pose_offsets_desugared_chain() -> None:
    plan = compile_relative_to_absolute(
        [drive_forward(20)],
        start=IntendedPose(x_m=0.10, y_m=0.20, theta_rad=math.pi / 2.0),
        fold_turns=False,
    )

    node = plan.nodes[0]
    assert isinstance(node, Goto)
    assert node.x_m == pytest.approx(0.10)
    assert node.y_m == pytest.approx(0.40)
    assert node.theta_rad == pytest.approx(math.pi / 2.0)


def test_validate_reachable_rejects_target_outside_table() -> None:
    with pytest.raises(CompileError, match="outside table bounds"):
        compile_relative_to_absolute([drive_forward(150)], world_map=_Map())


def test_condition_based_segments_are_rejected_for_absolute_desugar() -> None:
    step = drive_forward(speed=0.5).until(_Never())
    with pytest.raises(CompileError, match="known endpoint"):
        compile_relative_to_absolute([step])


def test_right_turn_desugars_negative_heading() -> None:
    plan = compile_relative_to_absolute([turn_right(90)], fold_turns=False)

    node = plan.nodes[0]
    assert isinstance(node, TurnTo)
    assert node.theta_rad == pytest.approx(-math.pi / 2.0)


def test_absolute_bridge_preserves_absolute_turn_and_linear_heading_targets() -> None:
    plan = compile_relative_to_absolute(
        [turn_left(90), drive_forward(50)],
        fold_turns=False,
    )

    nodes = absolute_to_relative_nodes(plan.nodes)

    assert len(nodes) == 2
    turn, drive = nodes
    assert isinstance(turn, Segment)
    assert turn.kind == "turn"
    assert turn.angle_rad == pytest.approx(math.pi / 2.0)
    assert turn.target_heading_rad == pytest.approx(math.pi / 2.0)

    assert isinstance(drive, Segment)
    assert drive.kind == "linear"
    assert drive.distance_m == pytest.approx(0.50)
    assert drive.target_heading_rad == pytest.approx(math.pi / 2.0)


def test_absolute_bridge_preserves_side_action_blocking_mode() -> None:
    plan = compile_relative_to_absolute(
        [drive_forward(20), background(run(lambda: None)), drive_forward(20)],
        fold_turns=False,
    )

    nodes = absolute_to_relative_nodes(plan.nodes)

    assert len(nodes) == 3
    assert isinstance(nodes[1], SideAction)
    assert nodes[1].is_background is True
