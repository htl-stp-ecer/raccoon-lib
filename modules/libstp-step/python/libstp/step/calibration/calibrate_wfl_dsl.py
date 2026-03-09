"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: calibrate_wfl.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .calibrate_wfl import CalibrateWaitForLight


class CalibrateWaitForLightBuilder(StepBuilder):
    """Builder for CalibrateWaitForLight. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET

    def sensor(self, value: AnalogSensor):
        self._sensor = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        return CalibrateWaitForLight(**kwargs)


@dsl(tags=['calibration', 'light'])
def calibrate_wait_for_light(sensor: AnalogSensor = _UNSET):
    """
    Calibrate a wait-for-light sensor via interactive measurement.

    Guides the operator through a two-step measurement flow: first cover
    the sensor (dark reading), then expose it to the start lamp (light
    reading). The midpoint threshold is computed and stored so the
    ``wait_for_light_legacy`` step can detect the lamp signal.

    The operator can retry measurements if the values look wrong before
    confirming the calibration.

    Args:
        sensor: The AnalogSensor instance to calibrate.

    Returns:
        A CalibrateWaitForLightBuilder (chainable via ``.sensor()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.calibration import calibrate_wait_for_light

        calibrate_wait_for_light(robot.defs.wait_for_light_sensor)
    """
    b = CalibrateWaitForLightBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    return b


__all__ = ['CalibrateWaitForLightBuilder', 'calibrate_wait_for_light']
