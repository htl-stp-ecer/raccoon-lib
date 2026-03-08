"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: diagonal.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .diagonal import DriveAngleUntilBlack, DriveAngleUntilWhite


class DriveAngleUntilBlackBuilder(StepBuilder):
    """Builder for DriveAngleUntilBlack. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._angle_deg = _UNSET
        self._speed = 0.3
        self._confidence_threshold = 0.7

    def sensor(self, value: Union[IRSensor, list[IRSensor]]):
        self._sensor = value
        return self

    def angle_deg(self, value: float):
        self._angle_deg = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def confidence_threshold(self, value: float):
        self._confidence_threshold = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        if self._angle_deg is not _UNSET:
            kwargs['angle_deg'] = self._angle_deg
        kwargs['speed'] = self._speed
        kwargs['confidence_threshold'] = self._confidence_threshold
        return DriveAngleUntilBlack(**kwargs)


@dsl(tags=['motion', 'sensor'])
def drive_angle_until_black(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, angle_deg: float = _UNSET, speed: float = 0.3, confidence_threshold: float = 0.7):
    """
    Drive at an arbitrary angle until any sensor detects a black surface.

    Decompose the desired travel direction into forward and strafe velocity
    components using trigonometry (``cos`` for forward, ``sin`` for strafe) and
    command them simultaneously. The robot's heading does not change -- only
    its translational direction of travel. Each control cycle, the given IR
    sensor(s) are polled and the step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``.

    The angle convention is: 0 degrees = pure forward, 90 degrees = pure right
    strafe, -90 degrees = pure left strafe, and 180 degrees = pure backward.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects black.
        angle_deg: Travel angle in degrees relative to the robot's forward axis. 0 = forward, 90 = right, -90 = left, 180 = backward.
        speed: Speed magnitude in m/s (positive value). This is the overall speed; it is decomposed into forward and strafe components based on the angle. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for black before the step considers the condition met. Defaults to 0.7.

    Returns:
        A DriveAngleUntilBlackBuilder (chainable via ``.sensor()``, ``.angle_deg()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveAngleUntilBlack

        front_ir = IRSensor(0)

        # Drive diagonally forward-right (45 deg) until black is detected
        DriveAngleUntilBlack(front_ir, angle_deg=45, speed=0.3)

        # Drive purely left (-90 deg) until black -- equivalent to strafe left
        DriveAngleUntilBlack(front_ir, angle_deg=-90, speed=0.2)
    """
    b = DriveAngleUntilBlackBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    if angle_deg is not _UNSET:
        b._angle_deg = angle_deg
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class DriveAngleUntilWhiteBuilder(StepBuilder):
    """Builder for DriveAngleUntilWhite. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._angle_deg = _UNSET
        self._speed = 0.3
        self._confidence_threshold = 0.7

    def sensor(self, value: Union[IRSensor, list[IRSensor]]):
        self._sensor = value
        return self

    def angle_deg(self, value: float):
        self._angle_deg = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def confidence_threshold(self, value: float):
        self._confidence_threshold = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        if self._angle_deg is not _UNSET:
            kwargs['angle_deg'] = self._angle_deg
        kwargs['speed'] = self._speed
        kwargs['confidence_threshold'] = self._confidence_threshold
        return DriveAngleUntilWhite(**kwargs)


@dsl(tags=['motion', 'sensor'])
def drive_angle_until_white(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, angle_deg: float = _UNSET, speed: float = 0.3, confidence_threshold: float = 0.7):
    """
    Drive at an arbitrary angle until any sensor detects a white surface.

    Decompose the desired travel direction into forward and strafe velocity
    components using trigonometry (``cos`` for forward, ``sin`` for strafe) and
    command them simultaneously. The robot's heading does not change -- only
    its translational direction of travel. Each control cycle, the given IR
    sensor(s) are polled and the step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``.

    The angle convention is: 0 degrees = pure forward, 90 degrees = pure right
    strafe, -90 degrees = pure left strafe, and 180 degrees = pure backward.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects white.
        angle_deg: Travel angle in degrees relative to the robot's forward axis. 0 = forward, 90 = right, -90 = left, 180 = backward.
        speed: Speed magnitude in m/s (positive value). This is the overall speed; it is decomposed into forward and strafe components based on the angle. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for white before the step considers the condition met. Defaults to 0.7.

    Returns:
        A DriveAngleUntilWhiteBuilder (chainable via ``.sensor()``, ``.angle_deg()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveAngleUntilWhite

        front_ir = IRSensor(0)

        # Drive diagonally forward-left (-45 deg) until white is detected
        DriveAngleUntilWhite(front_ir, angle_deg=-45, speed=0.3)

        # Drive backward (180 deg) until white -- equivalent to drive backward
        DriveAngleUntilWhite(front_ir, angle_deg=180, speed=0.2)
    """
    b = DriveAngleUntilWhiteBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    if angle_deg is not _UNSET:
        b._angle_deg = angle_deg
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


__all__ = ['DriveAngleUntilBlackBuilder', 'drive_angle_until_black', 'DriveAngleUntilWhiteBuilder', 'drive_angle_until_white']
