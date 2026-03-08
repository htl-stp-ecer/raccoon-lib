"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: drive.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .drive import DriveForwardUntilBlack, DriveForwardUntilWhite, DriveBackwardUntilBlack, DriveBackwardUntilWhite


class DriveForwardUntilBlackBuilder(StepBuilder):
    """Builder for DriveForwardUntilBlack. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._speed = 1.0
        self._confidence_threshold = 0.7

    def sensor(self, value: Union[IRSensor, list[IRSensor]]):
        self._sensor = value
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
        kwargs['speed'] = self._speed
        kwargs['confidence_threshold'] = self._confidence_threshold
        return DriveForwardUntilBlack(**kwargs)


@dsl(tags=['motion', 'sensor'])
def drive_forward_until_black(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 1.0, confidence_threshold: float = 0.7):
    """
    Drive forward until any sensor detects a black surface.

    Commands a constant forward velocity and polls the given IR sensor(s) each
    control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (forward) regardless of the sign passed in.

    This is the recommended way to drive forward until a black line or region
    is detected, for example when approaching a Botball scoring zone bounded
    by black tape.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects black.
        speed: Forward driving speed in m/s. The absolute value is used, so negative inputs are treated as positive. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for black before the step considers the condition met. Lower values make detection more sensitive but increase the risk of false positives. Defaults to 0.7.

    Returns:
        A DriveForwardUntilBlackBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveForwardUntilBlack

        front_ir = IRSensor(0)

        # Drive forward at 0.5 m/s until the front sensor sees black
        DriveForwardUntilBlack(front_ir, speed=0.5)

        # Use two sensors -- stop when either one detects black
        left_ir = IRSensor(1)
        DriveForwardUntilBlack([front_ir, left_ir], speed=0.8)
    """
    b = DriveForwardUntilBlackBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class DriveForwardUntilWhiteBuilder(StepBuilder):
    """Builder for DriveForwardUntilWhite. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._speed = 1.0
        self._confidence_threshold = 0.7

    def sensor(self, value: Union[IRSensor, list[IRSensor]]):
        self._sensor = value
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
        kwargs['speed'] = self._speed
        kwargs['confidence_threshold'] = self._confidence_threshold
        return DriveForwardUntilWhite(**kwargs)


@dsl(tags=['motion', 'sensor'])
def drive_forward_until_white(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 1.0, confidence_threshold: float = 0.7):
    """
    Drive forward until any sensor detects a white surface.

    Commands a constant forward velocity and polls the given IR sensor(s) each
    control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (forward) regardless of the sign passed in.

    Useful for driving across a dark region until the robot reaches a white
    surface, for example crossing black tape to re-enter the playing field.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects white.
        speed: Forward driving speed in m/s. The absolute value is used, so negative inputs are treated as positive. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for white before the step considers the condition met. Defaults to 0.7.

    Returns:
        A DriveForwardUntilWhiteBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveForwardUntilWhite

        front_ir = IRSensor(0)

        # Drive forward at default speed until white is found
        DriveForwardUntilWhite(front_ir)

        # Slower approach with stricter detection
        DriveForwardUntilWhite(front_ir, speed=0.3, confidence_threshold=0.9)
    """
    b = DriveForwardUntilWhiteBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class DriveBackwardUntilBlackBuilder(StepBuilder):
    """Builder for DriveBackwardUntilBlack. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._speed = 1.0
        self._confidence_threshold = 0.7

    def sensor(self, value: Union[IRSensor, list[IRSensor]]):
        self._sensor = value
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
        kwargs['speed'] = self._speed
        kwargs['confidence_threshold'] = self._confidence_threshold
        return DriveBackwardUntilBlack(**kwargs)


@dsl(tags=['motion', 'sensor'])
def drive_backward_until_black(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 1.0, confidence_threshold: float = 0.7):
    """
    Drive backward until any sensor detects a black surface.

    Commands a constant backward velocity and polls the given IR sensor(s)
    each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally, so you should pass a positive value.

    Useful for backing up toward a black boundary line, for example
    repositioning before a scoring action.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects black.
        speed: Backward driving speed in m/s. Pass a positive value; the sign is negated internally. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for black before the step considers the condition met. Defaults to 0.7.

    Returns:
        A DriveBackwardUntilBlackBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveBackwardUntilBlack

        rear_ir = IRSensor(2)

        # Back up at 0.4 m/s until the rear sensor detects black
        DriveBackwardUntilBlack(rear_ir, speed=0.4)
    """
    b = DriveBackwardUntilBlackBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class DriveBackwardUntilWhiteBuilder(StepBuilder):
    """Builder for DriveBackwardUntilWhite. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._speed = 1.0
        self._confidence_threshold = 0.7

    def sensor(self, value: Union[IRSensor, list[IRSensor]]):
        self._sensor = value
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
        kwargs['speed'] = self._speed
        kwargs['confidence_threshold'] = self._confidence_threshold
        return DriveBackwardUntilWhite(**kwargs)


@dsl(tags=['motion', 'sensor'])
def drive_backward_until_white(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 1.0, confidence_threshold: float = 0.7):
    """
    Drive backward until any sensor detects a white surface.

    Commands a constant backward velocity and polls the given IR sensor(s)
    each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally, so you should pass a positive value.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects white.
        speed: Backward driving speed in m/s. Pass a positive value; the sign is negated internally. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for white before the step considers the condition met. Defaults to 0.7.

    Returns:
        A DriveBackwardUntilWhiteBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveBackwardUntilWhite

        rear_ir = IRSensor(2)

        # Back up slowly until the rear sensor sees white
        DriveBackwardUntilWhite(rear_ir, speed=0.3)
    """
    b = DriveBackwardUntilWhiteBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


__all__ = ['DriveForwardUntilBlackBuilder', 'drive_forward_until_black', 'DriveForwardUntilWhiteBuilder', 'drive_forward_until_white', 'DriveBackwardUntilBlackBuilder', 'drive_backward_until_black', 'DriveBackwardUntilWhiteBuilder', 'drive_backward_until_white']
