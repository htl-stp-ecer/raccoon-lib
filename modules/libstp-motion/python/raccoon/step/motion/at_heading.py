"""
Wait until the robot has turned a certain number of degrees.

Designed to run inside a parallel() branch alongside a turn step:

    parallel(
        turn_left(90),
        seq([wait_until_degrees(45), servo_open()]),
    )

Multiple heading-triggered actions:

    parallel(
        turn_left(90),
        seq([wait_until_degrees(30), arm_lower()]),
        seq([wait_until_degrees(60), servo_open()]),
        seq([wait_until_degrees(85), servo_close()]),
    )

Polls odometry heading at 100 Hz.  Three origin modes are available —
see ``HeadingOrigin`` for details.
"""

from __future__ import annotations

import asyncio
import enum
import math
from typing import TYPE_CHECKING

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


class HeadingOrigin(enum.Enum):
    """Reference point from which ``wait_until_degrees`` measures rotation.

    Choose the origin that matches where your degree target is counted from.
    All three modes use the same 100 Hz polling loop and stall detection —
    only the zero point differs.

    Members
    -------
    STEP_START
        Degrees are counted from the heading at the moment
        ``wait_until_degrees`` *itself* starts executing.

        Use this when ``wait_until_degrees`` is the **first** step in its
        ``seq()`` branch, i.e. there are no prior steps that could consume
        time while the turn is already progressing.  This is the default.

        Example — fires 45° after the step starts::

            parallel(
                [
                    turn_left(90),
                    seq([wait_until_degrees(45), servo(claw, 90)]),
                ]
            )

    TURN_START
        Degrees are counted from the heading at the start of the concurrent
        turn step.  ``TurnMotion.start()`` resets odometry to zero, so this
        origin is always zero regardless of when ``wait_until_degrees``
        begins running.

        Use this when earlier steps in the same ``seq()`` branch may have
        taken time while the turn was already progressing, and you still
        want the threshold to be measured from the turn's own start, not
        from a later point in the branch.

        Example — fires when the turn has reached 45° from its own start,
        even though ``prepare_arm()`` ran first::

            parallel(
                [
                    turn_left(90),
                    seq(
                        [
                            prepare_arm(),
                            wait_until_degrees(45, origin=HeadingOrigin.TURN_START),
                            servo(claw, 90),
                        ]
                    ),
                ]
            )

    HEADING_REFERENCE
        Degrees are counted from the global heading reference set by
        ``mark_heading_reference()``.  Uses the raw IMU heading
        (``get_absolute_heading()``), which is never reset by motion steps.

        Use this when you want to trigger an action at a board-absolute
        heading rather than at an angle relative to the current turn.
        Requires ``mark_heading_reference()`` to have been called earlier
        in the mission.

        Example — fires when the robot faces 45° from its mission-start
        heading, no matter how many turns have happened before::

            mark_heading_reference()
            # ... other motion ...
            parallel(
                [
                    turn_left(90),
                    seq([wait_until_degrees(45, origin=HeadingOrigin.HEADING_REFERENCE), servo(claw, 90)]),
                ]
            )
    """

    STEP_START = "step_start"
    TURN_START = "turn_start"
    HEADING_REFERENCE = "heading_reference"


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


@dsl_step(tags=["motion", "wait"])
class WaitUntilDegrees(Step):
    """Wait until the robot has turned at least the given number of degrees.

    Polls heading at 100 Hz and compares the angular distance from the
    chosen origin against the threshold.  Designed to run inside a
    ``parallel()`` branch alongside a turn step, enabling actions to trigger
    at specific angles during a turn.

    The origin (zero point) is controlled by the ``origin`` parameter — see
    :class:`HeadingOrigin` for the three available modes.

    Args:
        degrees: Heading-change threshold in degrees (always positive).
        origin: Which reference point to count degrees from.
            Defaults to ``HeadingOrigin.STEP_START``.

    Example::

        from raccoon.step import parallel, seq
        from raccoon.step.motion import turn_left, wait_until_degrees, HeadingOrigin
        from raccoon.step.servo import servo

        # Default: fires 45° after this step starts executing
        parallel(
            [
                turn_left(90),
                seq([wait_until_degrees(45), servo(claw, 90)]),
            ]
        )

        # TURN_START: fires at 45° from the turn's own start
        parallel(
            [
                turn_left(90),
                seq(
                    [
                        prepare_arm(),
                        wait_until_degrees(45, origin=HeadingOrigin.TURN_START),
                        servo(claw, 90),
                    ]
                ),
            ]
        )

        # HEADING_REFERENCE: fires at 45° from the global reference
        parallel(
            [
                turn_left(90),
                seq([wait_until_degrees(45, origin=HeadingOrigin.HEADING_REFERENCE), servo(claw, 90)]),
            ]
        )
    """

    def __init__(
        self,
        degrees: float,
        origin: HeadingOrigin = HeadingOrigin.STEP_START,
    ) -> None:
        super().__init__()
        self._threshold_rad = math.radians(abs(degrees))
        self._origin = origin
        self._hz = 100

    def _generate_signature(self) -> str:
        suffix = (
            f", origin={self._origin.value}" if self._origin != HeadingOrigin.STEP_START else ""
        )
        return f"WaitUntilDegrees(degrees={math.degrees(self._threshold_rad):.1f}{suffix})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        interval = 1.0 / self._hz
        stall_threshold_rad = math.radians(0.5)  # 0.5°
        stall_timeout_s = 0.5

        if self._origin == HeadingOrigin.STEP_START:
            # Capture heading now; measure rotation from this point forward.
            _initial = robot.odometry.get_heading()

            def _turned() -> float:
                return abs(robot.odometry.get_heading() - _initial)

        elif self._origin == HeadingOrigin.TURN_START:
            # TurnMotion.start() resets odometry to zero, so the odometry
            # heading IS the rotation from turn-start already.
            def _turned() -> float:
                return abs(robot.odometry.get_heading())

        else:  # HEADING_REFERENCE
            from raccoon.robot.heading_reference import HeadingReferenceService

            service = robot.get_service(HeadingReferenceService)
            if service.reference_deg is None:
                msg = (
                    "HeadingOrigin.HEADING_REFERENCE requires mark_heading_reference() "
                    "to have been called before this step."
                )
                raise RuntimeError(msg)
            ref_rad = math.radians(service.reference_deg)

            def _turned() -> float:
                current = robot.odometry.get_absolute_heading()
                return abs(_normalize_angle(current - ref_rad))

        loop = asyncio.get_event_loop()
        last_progress_rad = 0.0
        last_progress_time = loop.time()

        while True:
            turned_rad = _turned()
            now = loop.time()

            if turned_rad >= self._threshold_rad:
                self.debug(
                    f"Turned {math.degrees(turned_rad):.1f}° >= "
                    f"{math.degrees(self._threshold_rad):.1f}° — done"
                )
                return

            # Stall detection: if heading hasn't meaningfully changed,
            # the turning motion has likely finished or stalled.
            if turned_rad - last_progress_rad > stall_threshold_rad:
                last_progress_rad = turned_rad
                last_progress_time = now
            elif now - last_progress_time > stall_timeout_s:
                self.error(
                    f"Stall detected: heading stuck at "
                    f"{math.degrees(turned_rad):.1f}° for {stall_timeout_s}s, "
                    f"target was {math.degrees(self._threshold_rad):.1f}° — "
                    f"the concurrent motion likely ended early"
                )
                return

            await asyncio.sleep(interval)
