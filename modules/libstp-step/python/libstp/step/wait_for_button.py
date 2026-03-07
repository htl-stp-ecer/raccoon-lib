import asyncio

from .base import Step
from .annotation import dsl
from libstp.button import is_pressed
from libstp.ui import UIStep

@dsl(hidden=True)
class WaitForButton(UIStep):
    """UI step that blocks until the operator presses the hardware button."""

    async def _execute_step(self, robot) -> None:
        await self.wait_for_button("Waiting for button press...")


@dsl(tags=["timing", "button"])
def wait_for_button() -> WaitForButton:
    """Wait for the operator to press the hardware button before continuing.

    Blocks the step sequence until the physical button on the robot
    controller is pressed. A "Waiting for button press..." message is
    displayed on the UI while waiting. This is useful for pausing
    between autonomous phases, confirming the robot is correctly
    positioned before starting, or gating destructive actions behind
    human approval.

    Returns:
        A ``WaitForButton`` UI step ready to be scheduled in a step
        sequence.

    Example::

        from libstp.step import wait_for_button

        # Wait for the operator to confirm placement before starting
        sequence(
            wait_for_button(),
            motor_power(robot.motor(0), 100),
            wait(3.0),
            motor_brake(robot.motor(0)),
        )
    """
    return WaitForButton()
