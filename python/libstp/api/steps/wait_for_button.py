import asyncio
from typing import Any

from libstp.device import NativeDevice
from libstp.sensor import is_button_clicked

from libstp_helpers.api.steps import Step


class WaitForButton(Step):
    """
    A step that waits for a button click before completing.
    """

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Wait for the specified duration.

        Args:
            device: The device to run on (not used in this step)
            definitions: Additional definitions needed for execution
        """
        await super().run_step(device, definitions)

        self.warn(
            "The robot is waiting for a button click to continue. This is a blocking step and will pause execution until a human operator clicks the button on the robot. If this isnt removed for the competition, it will 100% disqualify you.")
        while is_button_clicked():  # make sure button is not clicked
            await asyncio.sleep(0.1)

        while not is_button_clicked():  # wait until is clicked
            await asyncio.sleep(0.1)


def wait_for_button() -> WaitForButton:
    """Wait for a button click"""
    return WaitForButton()
