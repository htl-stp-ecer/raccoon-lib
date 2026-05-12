"""Desugar the relative motion DSL into absolute motion-plan IR."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from raccoon.motion import LinearAxis

from .abs_compiler import CompiledAbsolutePlan
from .abs_ir import AbsoluteNode, Action, Goto, Resync, TurnTo
from .abs_passes import CompileError, fold_implicit_turns, validate_reachable
from .ir import PathNode, Segment, SideAction
from .passes import flatten_steps


@dataclass(frozen=True)
class IntendedPose:
    """Compile-time pose in SI units used by the relative-to-absolute pass."""

    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0


DEFAULT_START = IntendedPose()


def _body_to_world(
    forward_m: float, strafe_right_m: float, theta_rad: float
) -> tuple[float, float]:
    return (
        forward_m * math.cos(theta_rad) + strafe_right_m * math.sin(theta_rad),
        forward_m * math.sin(theta_rad) - strafe_right_m * math.cos(theta_rad),
    )


def _world_to_body(dx_m: float, dy_m: float, theta_rad: float) -> tuple[float, float]:
    return (
        dx_m * math.cos(theta_rad) + dy_m * math.sin(theta_rad),
        dx_m * math.sin(theta_rad) - dy_m * math.cos(theta_rad),
    )


def _linear_endpoint(pose: IntendedPose, seg: Segment) -> IntendedPose:
    if seg.distance_m is None or not seg.has_known_endpoint:
        msg = "compile_relative_to_absolute: condition-based linear segments need a known endpoint"
        raise CompileError(msg)

    if seg.axis == LinearAxis.Forward:
        dx, dy = _body_to_world(seg.distance_m, 0.0, pose.theta_rad)
    elif seg.axis == LinearAxis.Lateral:
        dx, dy = _body_to_world(0.0, seg.distance_m, pose.theta_rad)
    else:
        msg = f"compile_relative_to_absolute: unsupported linear axis {seg.axis!r}"
        raise CompileError(msg)

    theta = pose.theta_rad
    if seg.heading_deg is not None:
        theta = math.radians(seg.heading_deg)
    return IntendedPose(pose.x_m + dx, pose.y_m + dy, theta)


def _arc_endpoint(pose: IntendedPose, seg: Segment) -> IntendedPose:
    if seg.radius_m is None or seg.arc_angle_rad is None:
        msg = "compile_relative_to_absolute: arc segments need radius and angle"
        raise CompileError(msg)

    radius = seg.radius_m
    theta = seg.arc_angle_rad
    if seg.lateral:
        strafe_sign = 1.0 if theta >= 0.0 else -1.0
        forward = radius * (1.0 - math.cos(theta))
        strafe = strafe_sign * radius * math.sin(abs(theta))
    else:
        forward = radius * math.sin(abs(theta))
        strafe = radius * (1.0 - math.cos(theta)) * (-1.0 if theta > 0.0 else 1.0)

    dx, dy = _body_to_world(forward, strafe, pose.theta_rad)
    return IntendedPose(pose.x_m + dx, pose.y_m + dy, pose.theta_rad + theta)


def _segment_to_absolute(seg: Segment, pose: IntendedPose) -> tuple[AbsoluteNode, IntendedPose]:
    if seg.kind == "follow_line":
        msg = (
            "compile_relative_to_absolute: follow_line segments are opaque "
            "and cannot be desugared safely yet"
        )
        raise CompileError(msg)

    if seg.kind == "linear":
        next_pose = _linear_endpoint(pose, seg)
        via = "lateral" if seg.axis == LinearAxis.Lateral else "forward"
        return (
            Goto(
                x_m=next_pose.x_m,
                y_m=next_pose.y_m,
                theta_rad=next_pose.theta_rad,
                via=via,
                speed_scale=seg.speed_scale,
            ),
            next_pose,
        )

    if seg.kind == "turn":
        if seg.angle_rad is None or not seg.has_known_endpoint:
            msg = "compile_relative_to_absolute: condition-based turns need a known endpoint"
            raise CompileError(msg)
        next_pose = IntendedPose(pose.x_m, pose.y_m, pose.theta_rad + seg.angle_rad)
        return TurnTo(theta_rad=next_pose.theta_rad), next_pose

    if seg.kind == "arc":
        next_pose = _arc_endpoint(pose, seg)
        return (
            Goto(
                x_m=next_pose.x_m,
                y_m=next_pose.y_m,
                theta_rad=next_pose.theta_rad,
                via="arc",
                speed_scale=seg.speed_scale,
            ),
            next_pose,
        )

    msg = f"compile_relative_to_absolute: unsupported segment kind {seg.kind!r}"
    raise CompileError(msg)


def nodes_to_absolute(
    nodes: Sequence[PathNode | None],
    *,
    start: IntendedPose = DEFAULT_START,
) -> tuple[AbsoluteNode, ...]:
    """Convert already-lowered relative path nodes into absolute IR nodes."""
    pose = start
    out: list[AbsoluteNode] = []
    for index, node in enumerate(nodes):
        if node is None:
            msg = (
                "compile_relative_to_absolute: Defer nodes cannot be desugared "
                f"at construction time (index {index})"
            )
            raise CompileError(msg)
        if isinstance(node, Segment):
            abs_node, pose = _segment_to_absolute(node, pose)
            out.append(abs_node)
            continue
        if isinstance(node, SideAction):
            out.append(Action(step=node.step, blocking=not node.is_background))
            continue

        msg = f"compile_relative_to_absolute: unsupported node type {type(node).__name__!r}"
        raise CompileError(msg)
    return tuple(out)


def absolute_to_relative_nodes(
    nodes: Sequence[AbsoluteNode],
    *,
    start: IntendedPose = DEFAULT_START,
) -> list[PathNode]:
    """Lower a conservative absolute IR subset back into legacy path nodes.

    This bridge is intentionally narrow: it only accepts the subset that the
    existing executor can run without silently changing semantics.
    """
    pose = start
    out: list[PathNode] = []
    for index, node in enumerate(nodes):
        if isinstance(node, Action):
            out.append(SideAction(step=node.step, is_background=not node.blocking))
            continue

        if isinstance(node, TurnTo):
            angle_rad = math.atan2(
                math.sin(node.theta_rad - pose.theta_rad),
                math.cos(node.theta_rad - pose.theta_rad),
            )
            out.append(
                Segment(
                    kind="turn",
                    sign=1.0 if angle_rad >= 0.0 else -1.0,
                    angle_rad=angle_rad,
                    target_heading_rad=node.theta_rad,
                    has_known_endpoint=True,
                )
            )
            pose = IntendedPose(pose.x_m, pose.y_m, node.theta_rad)
            continue

        if isinstance(node, Goto):
            if node.via == "arc":
                msg = (
                    "absolute_to_relative_nodes: arc Goto nodes cannot be "
                    "lowered back into legacy Segment IR safely"
                )
                raise CompileError(msg)

            dx_m = node.x_m - pose.x_m
            dy_m = node.y_m - pose.y_m
            forward_m, strafe_right_m = _world_to_body(dx_m, dy_m, pose.theta_rad)

            if node.via == "forward":
                if abs(strafe_right_m) > 1e-6:
                    msg = (
                        "absolute_to_relative_nodes: forward Goto is not "
                        f"body-collinear at index {index}"
                    )
                    raise CompileError(msg)
                axis = LinearAxis.Forward
                distance_m = forward_m
            elif node.via == "lateral":
                if abs(forward_m) > 1e-6:
                    msg = (
                        "absolute_to_relative_nodes: lateral Goto is not "
                        f"body-collinear at index {index}"
                    )
                    raise CompileError(msg)
                axis = LinearAxis.Lateral
                distance_m = strafe_right_m
            elif node.via == "auto":
                if abs(strafe_right_m) <= 1e-6:
                    axis = LinearAxis.Forward
                    distance_m = forward_m
                elif abs(forward_m) <= 1e-6:
                    axis = LinearAxis.Lateral
                    distance_m = strafe_right_m
                else:
                    msg = (
                        "absolute_to_relative_nodes: auto Goto needs an "
                        f"unambiguous single-axis delta at index {index}"
                    )
                    raise CompileError(msg)
            else:
                msg = f"absolute_to_relative_nodes: unsupported via={node.via!r}"
                raise CompileError(msg)

            out.append(
                Segment(
                    kind="linear",
                    axis=axis,
                    sign=1.0 if distance_m >= 0.0 else -1.0,
                    distance_m=distance_m,
                    speed_scale=node.speed_scale,
                    target_heading_rad=node.theta_rad,
                    has_known_endpoint=True,
                )
            )
            pose = IntendedPose(
                x_m=node.x_m,
                y_m=node.y_m,
                theta_rad=pose.theta_rad if node.theta_rad is None else node.theta_rad,
            )
            continue

        if isinstance(node, Resync):
            msg = "absolute_to_relative_nodes: Resync nodes are not executable on the legacy path"
            raise CompileError(msg)

        msg = f"absolute_to_relative_nodes: unsupported node type {type(node).__name__!r}"
        raise CompileError(msg)
    return out


def compile_relative_to_absolute(
    steps: Sequence,
    *,
    start: IntendedPose = DEFAULT_START,
    world_map=None,
    validate: bool = True,
    fold_turns: bool = True,
) -> CompiledAbsolutePlan:
    """Compile relative DSL steps into absolute IR.

    This is Phase 5's conservative compile-time path. It does not execute the
    plan and it does not read runtime localization; it only follows the
    intended relative pose chain described in the design document.
    """
    nodes, deferred = flatten_steps(list(steps))
    if deferred:
        msg = "compile_relative_to_absolute: Defer steps are not supported by the absolute IR path yet"
        raise CompileError(msg)

    abs_nodes = nodes_to_absolute(nodes, start=start)
    passes: list[str] = ["relative_desugar"]

    if validate:
        abs_nodes = validate_reachable(abs_nodes, world_map=world_map)
        passes.append("validate_reachable")
    if fold_turns:
        abs_nodes = fold_implicit_turns(abs_nodes)
        passes.append("fold_implicit_turns")

    return CompiledAbsolutePlan(nodes=tuple(abs_nodes), passes_applied=tuple(passes))


__all__ = [
    "IntendedPose",
    "absolute_to_relative_nodes",
    "compile_relative_to_absolute",
    "nodes_to_absolute",
]
