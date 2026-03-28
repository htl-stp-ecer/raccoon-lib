import asyncio

from .base import Step
from .annotation import dsl_step
from libstp.button import is_pressed
from libstp.ui import UIStep


@dsl_step(tags=["timing", "button"])
class WaitForButton(UIStep):
    """Wait for the operator to press the hardware button before continuing.

    Blocks the step sequence until the physical button on the robot
    controller is pressed. A message is displayed on the UI while
    waiting. This is useful for pausing between autonomous phases,
    confirming the robot is correctly positioned before starting, or
    gating destructive actions behind human approval.

    Args:
        message: Text displayed on screen while waiting for the button
            press. Defaults to ``"Waiting for button press..."``.

    Example::

        from libstp.step import wait_for_button

        # Wait for the operator to confirm placement before starting
        sequence(
            wait_for_button("Place robot at starting position"),
            motor_power(robot.motor(0), 100),
            wait(3.0),
            motor_brake(robot.motor(0)),
        )
    """

    def __init__(self, message: str = "Waiting for button press...") -> None:
        super().__init__()
        self._message = message

    def _generate_signature(self) -> str:
        return f"WaitForButton(message={self._message!r})"

    async def _execute_step(self, robot) -> None:
        await self.wait_for_button(self._message)
