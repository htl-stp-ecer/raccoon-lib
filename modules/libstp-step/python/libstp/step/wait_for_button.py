import asyncio

from .base import Step
from .annotation import dsl
from libstp.button import is_pressed
from libstp.ui import UIStep

@dsl(hidden=True)
class WaitForButton(UIStep):
    async def _execute_step(self, robot) -> None:
        await self.wait_for_button("Waiting for button press...")


@dsl(tags=["timing", "button"])
def wait_for_button() -> WaitForButton:
    return WaitForButton()
