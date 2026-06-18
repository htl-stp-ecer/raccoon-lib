"""``absolute_heading`` pass — pin each straight leg to an integrated heading.

Every straight motion (``linear`` / ``follow_line``) normally re-zeroes its
heading reference on whatever world heading the previous segment happened to
leave behind (executor priority #3 — ``current_world_heading_rad`` read fresh
at each boundary, see ``motion_factory._create_linear_motion``). Across a long
path the per-leg odometry drift accumulates.

This pass instead integrates a single running heading through the node list at
compile time and stamps it onto ``Segment.target_heading_rad`` for every
straight leg, so each leg regulates against ONE coherent heading reference
rather than chaining off the previous leg's drifted frame.

Integration convention (identical to ``to_absolute`` / ``segments_to_spline_waypoints``):

    running_heading starts at 0.0 (the path-start frame)
    each ``turn`` segment adds its signed ``angle_rad``
    each ``arc`` segment adds its signed ``arc_angle_rad``
    straight legs do not change the heading

Frame of ``target_heading_rad`` (path-start-relative; executor applies offset):
    At runtime ``target_heading_rad`` is fed *straight into the PID* as an
    ABSOLUTE world heading (``motion_factory._create_linear_motion`` line ~182:
    ``config.target_heading_rad = seg.target_heading_rad``), and "world heading"
    is the raw, un-zeroed odometry heading (``get_world_heading_rad`` =
    ``robot.odometry.get_pose().heading``). The headings this pass emits are in
    the PATH-START frame (start = 0).

    The executor resolves the frame: ``PathExecutor.run`` captures the
    path-start world heading H0 once at the first motion segment and adds it to
    every stamped ``target_heading_rad`` (via ``_with_heading_offset``) before
    the value reaches ``create_motion`` / position-offset / warm-start, so each
    leg regulates against the correct absolute world heading regardless of the
    robot's heading at path start. This pass therefore emits the relative
    integration only; the offset is an executor concern, now wired in.

Guards (conservative — drift correction must never fight an unknown frame):
    * A ``SideAction`` resets the running heading to 0 and *stops* stamping
      until the next clean node — a side action may reorient the robot
      (it can drive), invalidating the integrated frame.
    * A ``None`` (deferred placeholder) does the same — its motion is unknown.
    * A ``Segment`` with ``has_known_endpoint is False`` (unknown-angle turn /
      condition-only move) makes every downstream absolute heading
      unpredictable: we reset and stop stamping for the remainder of the path.
      (We do not try to resume after a clean run-restart heuristic — there is
      no clean run-restart marker in the IR, so the conservative choice is to
      stop until the list ends. A ``SideAction`` / ``None`` that follows would
      reset anyway; this only differs by also stopping across a later clean
      stretch, which we accept rather than risk stamping a wrong frame.)
    * An explicit user heading already on the segment (``heading_deg`` set, or
      ``target_heading_rad`` already set) is never overwritten — user intent
      wins. We still advance the running heading past such legs.

``arc`` / ``spline`` segments are passed through untouched (we never stamp a
heading onto them), but an ``arc`` with a known ``arc_angle_rad`` still advances
the running heading so following straight legs stay correct.
"""

from __future__ import annotations

from dataclasses import replace

from ..ir import PathNode, Segment, SideAction


def _advance_heading(running: float, seg: Segment) -> float:
    """Advance the running heading by a segment's signed angle, if known."""
    if seg.kind == "turn" and seg.angle_rad is not None:
        return running + seg.angle_rad
    if seg.kind == "arc" and seg.arc_angle_rad is not None:
        return running + seg.arc_angle_rad
    return running


def _has_explicit_heading(seg: Segment) -> bool:
    """Did the user pin a heading on this segment already?

    ``heading_deg`` is the user-facing ``heading=`` kwarg (drive/strafe), and a
    pre-set ``target_heading_rad`` comes from the absolute-plan bridge. Either
    means "don't touch".
    """
    return seg.heading_deg is not None or seg.target_heading_rad is not None


class AbsoluteHeadingPass:
    """Stamp an integrated path-start heading onto every straight leg.

    Pure node→node pass. Walks the node list once, integrating a running
    heading (start 0; turns/arcs add their signed angle) and filling
    ``target_heading_rad`` on each ``linear`` / ``follow_line`` segment that has
    no explicit user heading. Barriers (``SideAction``, ``None``, or an
    unknown-endpoint segment) reset the frame and stop stamping; see the module
    docstring for the full guard rules and the absolute-vs-relative frame
    caveat.
    """

    name = "absolute_heading"

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        result: list[PathNode | None] = []
        running = 0.0
        # Once the absolute frame is invalidated (unknown endpoint) we stop
        # stamping for the rest of the path; SideAction/None also stop but
        # reset `running` so a subsequent clean stretch *could* resume — except
        # we treat an unknown endpoint as terminal for stamping.
        stamping = True

        for node in nodes:
            # --- Barriers: deferred placeholder or side action ---
            if node is None or isinstance(node, SideAction):
                result.append(node)
                running = 0.0
                stamping = True  # a clean motion stretch may follow
                continue

            seg = node

            # --- Unknown-endpoint segment: frame becomes unpredictable ---
            if not seg.has_known_endpoint:
                result.append(seg)
                running = 0.0
                stamping = False  # stop stamping for the rest of the path
                continue

            # --- arc / spline: pass through, but advance heading for arcs ---
            if seg.kind in ("arc", "spline"):
                result.append(seg)
                running = _advance_heading(running, seg)
                continue

            # --- turn: advance heading; optionally stamp post-turn heading ---
            if seg.kind == "turn":
                running = _advance_heading(running, seg)
                if stamping and not _has_explicit_heading(seg):
                    # Matches _lower_absolute_turn: target_heading_rad is the
                    # POST-turn absolute heading.
                    result.append(replace(seg, target_heading_rad=running))
                else:
                    result.append(seg)
                continue

            # --- straight leg (linear / follow_line) ---
            if seg.kind in ("linear", "follow_line"):
                if stamping and not _has_explicit_heading(seg):
                    result.append(replace(seg, target_heading_rad=running))
                else:
                    result.append(seg)
                continue

            # --- anything else: untouched ---
            result.append(seg)

        return result
