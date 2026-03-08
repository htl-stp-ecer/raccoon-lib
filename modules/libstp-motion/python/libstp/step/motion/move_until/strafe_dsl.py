"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: strafe.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .strafe import StrafeLeftUntilBlack, StrafeLeftUntilWhite, StrafeRightUntilBlack, StrafeRightUntilWhite


class StrafeLeftUntilBlackBuilder(StepBuilder):
    """Builder for StrafeLeftUntilBlack. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._speed = 0.3
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
        return StrafeLeftUntilBlack(**kwargs)


@dsl(tags=['motion', 'sensor'])
def strafe_left_until_black(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 0.3, confidence_threshold: float = 0.7):
    """
    Strafe left until any sensor detects a black surface.

    Commands a constant leftward lateral velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally to produce leftward motion, so you should pass
    a positive value.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects black.
        speed: Lateral speed in m/s. Pass a positive value; the sign is negated internally to strafe left. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for black before the step considers the condition met. Defaults to 0.7.

    Returns:
        A StrafeLeftUntilBlackBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import StrafeLeftUntilBlack

        left_ir = IRSensor(1)

        # Strafe left at 0.2 m/s until the left sensor finds a black line
        StrafeLeftUntilBlack(left_ir, speed=0.2)
    """
    b = StrafeLeftUntilBlackBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class StrafeLeftUntilWhiteBuilder(StepBuilder):
    """Builder for StrafeLeftUntilWhite. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._speed = 0.3
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
        return StrafeLeftUntilWhite(**kwargs)


@dsl(tags=['motion', 'sensor'])
def strafe_left_until_white(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 0.3, confidence_threshold: float = 0.7):
    """
    Strafe left until any sensor detects a white surface.

    Commands a constant leftward lateral velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally to produce leftward motion, so you should pass
    a positive value.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects white.
        speed: Lateral speed in m/s. Pass a positive value; the sign is negated internally to strafe left. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for white before the step considers the condition met. Defaults to 0.7.

    Returns:
        A StrafeLeftUntilWhiteBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import StrafeLeftUntilWhite

        left_ir = IRSensor(1)

        # Strafe left at default speed until the left sensor finds white
        StrafeLeftUntilWhite(left_ir)
    """
    b = StrafeLeftUntilWhiteBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class StrafeRightUntilBlackBuilder(StepBuilder):
    """Builder for StrafeRightUntilBlack. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._speed = 0.3
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
        return StrafeRightUntilBlack(**kwargs)


@dsl(tags=['motion', 'sensor'])
def strafe_right_until_black(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 0.3, confidence_threshold: float = 0.7):
    """
    Strafe right until any sensor detects a black surface.

    Commands a constant rightward lateral velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (rightward) regardless of the sign passed in.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects black.
        speed: Lateral speed in m/s. The absolute value is used, so negative inputs are treated as positive. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for black before the step considers the condition met. Defaults to 0.7.

    Returns:
        A StrafeRightUntilBlackBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import StrafeRightUntilBlack

        right_ir = IRSensor(3)

        # Strafe right at 0.25 m/s until the right sensor hits a black line
        StrafeRightUntilBlack(right_ir, speed=0.25)
    """
    b = StrafeRightUntilBlackBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class StrafeRightUntilWhiteBuilder(StepBuilder):
    """Builder for StrafeRightUntilWhite. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._speed = 0.3
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
        return StrafeRightUntilWhite(**kwargs)


@dsl(tags=['motion', 'sensor'])
def strafe_right_until_white(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 0.3, confidence_threshold: float = 0.7):
    """
    Strafe right until any sensor detects a white surface.

    Commands a constant rightward lateral velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (rightward) regardless of the sign passed in.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects white.
        speed: Lateral speed in m/s. The absolute value is used, so negative inputs are treated as positive. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for white before the step considers the condition met. Defaults to 0.7.

    Returns:
        A StrafeRightUntilWhiteBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import StrafeRightUntilWhite

        right_ir = IRSensor(3)

        # Strafe right at default speed until the right sensor finds white
        StrafeRightUntilWhite(right_ir)

        # Use multiple sensors with high confidence
        bottom_ir = IRSensor(4)
        StrafeRightUntilWhite(
            [right_ir, bottom_ir], speed=0.2, confidence_threshold=0.85
        )
    """
    b = StrafeRightUntilWhiteBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


__all__ = ['StrafeLeftUntilBlackBuilder', 'strafe_left_until_black', 'StrafeLeftUntilWhiteBuilder', 'strafe_left_until_white', 'StrafeRightUntilBlackBuilder', 'strafe_right_until_black', 'StrafeRightUntilWhiteBuilder', 'strafe_right_until_white']
