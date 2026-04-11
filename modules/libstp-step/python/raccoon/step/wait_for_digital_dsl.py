"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: wait_for_digital.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .wait_for_digital import WaitForDigital


class WaitForDigitalBuilder(StepBuilder):
    """Builder for WaitForDigital. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._pressed = True

    def sensor(self, value: 'DigitalSensor'):
        self._sensor = value
        return self

    def pressed(self, value: bool):
        self._pressed = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        kwargs['pressed'] = self._pressed
        return WaitForDigital(**kwargs)


@dsl(tags=['timing', 'wait', 'sensor'])
def wait_for_digital(sensor: 'DigitalSensor' = _UNSET, pressed: bool = True):
    """
    Block until a digital sensor reads the desired state.

    Polls the sensor at 50 Hz and resumes execution once the reading
    matches the expected value. Useful for waiting until a bumper is
    pressed, a limit switch is triggered, or an external signal arrives.

    Args:
        sensor: The DigitalSensor instance to poll.
        pressed: The target state to wait for. ``True`` (default) waits until the sensor reads high; ``False`` waits until it reads low.

    Returns:
        A WaitForDigitalBuilder (chainable via ``.sensor()``, ``.pressed()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step import wait_for_digital
        from raccoon.hal import DigitalSensor

        bumper = DigitalSensor(0)

        # Wait until bumper is pressed
        wait_for_digital(bumper)

        # Wait until bumper is released
        wait_for_digital(bumper, pressed=False)
    """
    b = WaitForDigitalBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._pressed = pressed
    return b


__all__ = ['WaitForDigitalBuilder', 'wait_for_digital']
