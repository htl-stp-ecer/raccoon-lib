from __future__ import annotations

from typing import TYPE_CHECKING

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["motion", "stop"])
class Stop(Step):
    """Stop all drive motors immediately.

    Use this between motion sequences or at the end of a mission
    to ensure the robot is stationary.

    Args:
        hard: If ``True`` (default), immediately zero motor output.
            If ``False``, decelerate smoothly using the drive controller.

    Example::

        from raccoon.step.motion import drive_forward, stop

        # Drive forward then stop
        seq([drive_forward(50), stop()])
    """

    def __init__(self, hard: bool = True) -> None:
        super().__init__()
        self.hard = hard

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive"})

    def _generate_signature(self) -> str:
        return f"Stop(hard={self.hard})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Stop all drive motors."""
        if self.hard:
            robot.drive.hard_stop()
        else:
            robot.drive.soft_stop()
