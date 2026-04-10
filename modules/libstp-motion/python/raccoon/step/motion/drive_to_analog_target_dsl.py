"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: drive_to_analog_target.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .drive_to_analog_target import DriveToAnalogTarget


class DriveToAnalogTargetBuilder(StepBuilder):
    """Builder for DriveToAnalogTarget. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._speed = 0.3
        self._set_name = 'default'
        self._timeout_cm = None

    def sensor(self, value: 'AnalogSensor'):
        self._sensor = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def set_name(self, value: str):
        self._set_name = value
        return self

    def timeout_cm(self, value: Optional[float]):
        self._timeout_cm = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        kwargs['speed'] = self._speed
        kwargs['set_name'] = self._set_name
        kwargs['timeout_cm'] = self._timeout_cm
        return DriveToAnalogTarget(**kwargs)


@dsl(tags=['sensor', 'drive'])
def drive_to_analog_target(sensor: 'AnalogSensor' = _UNSET, speed: float = 0.3, set_name: str = 'default', timeout_cm: Optional[float] = None):
    """
    Drive until an analog sensor reaches its calibrated reference value.

    Reads the reference raw value stored by ``calibrate_analog_sensor()`` for
    the given sensor and set name, then drives forward or backward at the
    specified speed until ``sensor.read()`` crosses that threshold.  Direction
    is chosen automatically: if the current reading is below the target the
    robot drives forward (toward the target); if it is already above the
    target the robot drives backward.

    A ``timeout_cm`` can be provided as a safety backstop — the step stops
    after that distance even if the sensor has not reached the target.

    Prerequisites:
        ``calibrate_analog_sensor(sensor, set_name=...)`` must have been run
        (or a stored calibration must exist) before this step executes.
        Raises ``RuntimeError`` if no calibration data is found.

    Args:
        sensor: The analog sensor used to detect the target position.
        speed: Drive speed as a fraction of maximum (0.0–1.0, default 0.3). Use a slow speed for precise positioning.
        set_name: Which stored calibration point to target (default ``"default"``).
        timeout_cm: Maximum distance to drive in centimetres before giving up.  ``None`` disables the distance limit (default).

    Returns:
        A DriveToAnalogTargetBuilder (chainable via ``.sensor()``, ``.speed()``, ``.set_name()``, ``.timeout_cm()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import drive_to_analog_target

        # Drive to the default calibrated ET-sensor position
        drive_to_analog_target(robot.defs.et_sensor)

        # Slower approach to a named position with 30 cm safety backstop
        drive_to_analog_target(
            robot.defs.et_sensor,
            speed=0.2,
            set_name="near",
            timeout_cm=30,
        )
    """
    b = DriveToAnalogTargetBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._set_name = set_name
    b._timeout_cm = timeout_cm
    return b


__all__ = ['DriveToAnalogTargetBuilder', 'drive_to_analog_target']
