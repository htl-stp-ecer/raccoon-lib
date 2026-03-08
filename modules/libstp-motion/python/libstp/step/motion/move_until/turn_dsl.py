"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: turn.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .turn import TurnLeftUntilBlack, TurnLeftUntilWhite, TurnRightUntilBlack, TurnRightUntilWhite


class TurnLeftUntilBlackBuilder(StepBuilder):
    """Builder for TurnLeftUntilBlack. Auto-generated — do not edit."""

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
        return TurnLeftUntilBlack(**kwargs)


@dsl(tags=['motion', 'sensor'])
def turn_left_until_black(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 1.0, confidence_threshold: float = 0.7):
    """
    Turn left (counter-clockwise) until any sensor detects a black surface.

    Commands a constant counter-clockwise angular velocity and polls the given
    IR sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (CCW) regardless of the sign passed in.

    A common use case is sweeping a side-mounted IR sensor across the field to
    locate a black line, then stopping precisely when found.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects black.
        speed: Angular speed in rad/s. The absolute value is used, so negative inputs are treated as positive. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for black before the step considers the condition met. Defaults to 0.7.

    Returns:
        A TurnLeftUntilBlackBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import TurnLeftUntilBlack

        right_ir = IRSensor(3)

        # Sweep left at 0.8 rad/s until the right sensor crosses a black line
        TurnLeftUntilBlack(right_ir, speed=0.8)
    """
    b = TurnLeftUntilBlackBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class TurnLeftUntilWhiteBuilder(StepBuilder):
    """Builder for TurnLeftUntilWhite. Auto-generated — do not edit."""

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
        return TurnLeftUntilWhite(**kwargs)


@dsl(tags=['motion', 'sensor'])
def turn_left_until_white(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 1.0, confidence_threshold: float = 0.7):
    """
    Turn left (counter-clockwise) until any sensor detects a white surface.

    Commands a constant counter-clockwise angular velocity and polls the given
    IR sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (CCW) regardless of the sign passed in.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects white.
        speed: Angular speed in rad/s. The absolute value is used, so negative inputs are treated as positive. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for white before the step considers the condition met. Defaults to 0.7.

    Returns:
        A TurnLeftUntilWhiteBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import TurnLeftUntilWhite

        right_ir = IRSensor(3)

        # Sweep left at 0.6 rad/s until white surface is found
        TurnLeftUntilWhite(right_ir, speed=0.6)
    """
    b = TurnLeftUntilWhiteBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class TurnRightUntilBlackBuilder(StepBuilder):
    """Builder for TurnRightUntilBlack. Auto-generated — do not edit."""

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
        return TurnRightUntilBlack(**kwargs)


@dsl(tags=['motion', 'sensor'])
def turn_right_until_black(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 1.0, confidence_threshold: float = 0.7):
    """
    Turn right (clockwise) until any sensor detects a black surface.

    Commands a constant clockwise angular velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally to produce clockwise rotation, so you should
    pass a positive value.

    A common use case is sweeping a side-mounted IR sensor across the field to
    locate a black line, then stopping precisely when found.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects black.
        speed: Angular speed in rad/s. Pass a positive value; the sign is negated internally to rotate clockwise. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for black before the step considers the condition met. Defaults to 0.7.

    Returns:
        A TurnRightUntilBlackBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import TurnRightUntilBlack

        left_ir = IRSensor(1)

        # Sweep right at 0.8 rad/s until the left sensor crosses a black line
        TurnRightUntilBlack(left_ir, speed=0.8)
    """
    b = TurnRightUntilBlackBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


class TurnRightUntilWhiteBuilder(StepBuilder):
    """Builder for TurnRightUntilWhite. Auto-generated — do not edit."""

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
        return TurnRightUntilWhite(**kwargs)


@dsl(tags=['motion', 'sensor'])
def turn_right_until_white(sensor: Union[IRSensor, list[IRSensor]] = _UNSET, speed: float = 1.0, confidence_threshold: float = 0.7):
    """
    Turn right (clockwise) until any sensor detects a white surface.

    Commands a constant clockwise angular velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally to produce clockwise rotation, so you should
    pass a positive value.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of sensors. The step triggers when **any** sensor in the list detects white.
        speed: Angular speed in rad/s. Pass a positive value; the sign is negated internally to rotate clockwise. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor must report for white before the step considers the condition met. Defaults to 0.7.

    Returns:
        A TurnRightUntilWhiteBuilder (chainable via ``.sensor()``, ``.speed()``, ``.confidence_threshold()``).

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import TurnRightUntilWhite

        left_ir = IRSensor(1)

        # Sweep right slowly until the left sensor finds white
        TurnRightUntilWhite(left_ir, speed=0.5)
    """
    b = TurnRightUntilWhiteBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._speed = speed
    b._confidence_threshold = confidence_threshold
    return b


__all__ = ['TurnLeftUntilBlackBuilder', 'turn_left_until_black', 'TurnLeftUntilWhiteBuilder', 'turn_left_until_white', 'TurnRightUntilBlackBuilder', 'turn_right_until_black', 'TurnRightUntilWhiteBuilder', 'turn_right_until_white']
