import asyncio
from typing import TYPE_CHECKING

from .base import Step
from .annotation import dsl_step

if TYPE_CHECKING:
    from libstp.hal import DigitalSensor


@dsl_step(tags=["timing", "wait", "sensor"])
class WaitForDigital(Step):
    """Block until a digital sensor reads the desired state.

    Polls the sensor at 50 Hz and resumes execution once the reading
    matches the expected value. Useful for waiting until a bumper is
    pressed, a limit switch is triggered, or an external signal arrives.

    Args:
        sensor: The DigitalSensor instance to poll.
        pressed: The target state to wait for. ``True`` (default) waits
            until the sensor reads high; ``False`` waits until it reads
            low.

    Example::

        from libstp.step import wait_for_digital
        from libstp.hal import DigitalSensor

        bumper = DigitalSensor(0)

        # Wait until bumper is pressed
        wait_for_digital(bumper)

        # Wait until bumper is released
        wait_for_digital(bumper, pressed=False)
    """

    def __init__(self, sensor: "DigitalSensor", pressed: bool = True) -> None:
        super().__init__()
        self._sensor = sensor
        self._pressed = pressed

    def _generate_signature(self) -> str:
        state = "pressed" if self._pressed else "released"
        return f"WaitForDigital(port={self._sensor.port}, {state})"

    async def _execute_step(self, robot) -> None:
        while self._sensor.read() != self._pressed:
            await asyncio.sleep(0.02)
