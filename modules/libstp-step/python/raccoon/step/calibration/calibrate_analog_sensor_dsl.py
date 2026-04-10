"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: calibrate_analog_sensor.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .calibrate_analog_sensor import CalibrateAnalogSensor


class CalibrateAnalogSensorBuilder(StepBuilder):
    """Builder for CalibrateAnalogSensor. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._set_name = "default"
        self._sample_duration = 3.0

    def sensor(self, value: "AnalogSensor"):
        self._sensor = value
        return self

    def set_name(self, value: str):
        self._set_name = value
        return self

    def sample_duration(self, value: float):
        self._sample_duration = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs["sensor"] = self._sensor
        kwargs["set_name"] = self._set_name
        kwargs["sample_duration"] = self._sample_duration
        return CalibrateAnalogSensor(**kwargs)


@dsl(tags=["calibration", "sensor"])
def calibrate_analog_sensor(
    sensor: "AnalogSensor" = _UNSET,
    set_name: str = "default",
    sample_duration: float = 3.0,
):
    """
    Capture a reference analog sensor reading at a target robot position.

    Guides the operator to position the robot at a reference location, then
    samples the analog sensor for a short period to derive a stable reference
    raw value.  The result is persisted in ``racoon.calibration.yml`` under the
    ``analog-sensor`` section and consumed at runtime by
    ``drive_to_analog_target()``.

    Typical use: position the robot next to an object, run this step during
    setup, then use ``drive_to_analog_target()`` in the mission to reliably
    drive to that sensor distance every run.

    Supports ``--no-calibrate``: loads the stored reference without running
    the interactive flow when the flag is active and data already exists.

    Prerequisites:
        The sensor must be passed directly.  No auto-discovery from
        ``robot.defs`` is performed.

    Args:
        sensor: The analog sensor to calibrate (e.g. an ``ETSensor``).
        set_name: Label for this calibration point (default ``"default"``). Use different names for multiple reference distances on the same sensor port.
        sample_duration: How long to sample the sensor in seconds. Longer durations produce more stable averages.

    Returns:
        A CalibrateAnalogSensorBuilder (chainable via ``.sensor()``, ``.set_name()``, ``.sample_duration()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.calibration import calibrate_analog_sensor

        # Calibrate ET sensor at the default target position
        calibrate_analog_sensor(robot.defs.et_sensor)

        # Two named positions on the same sensor
        calibrate_analog_sensor(robot.defs.et_sensor, set_name="near")
        calibrate_analog_sensor(robot.defs.et_sensor, set_name="far")
    """
    b = CalibrateAnalogSensorBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._set_name = set_name
    b._sample_duration = sample_duration
    return b


__all__ = ["CalibrateAnalogSensorBuilder", "calibrate_analog_sensor"]
