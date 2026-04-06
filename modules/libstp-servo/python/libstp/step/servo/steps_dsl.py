"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: steps.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .steps import ShakeServo, SlowServo, FullyDisableServos, Easing, EasingFunc

from libstp.hal import Servo
from .preset import ServoPreset

class ShakeServoBuilder(StepBuilder):
    """Builder for ShakeServo. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._servo = _UNSET
        self._duration = _UNSET
        self._angle_a = _UNSET
        self._angle_b = _UNSET

    def servo(self, value: Servo | ServoPreset):
        self._servo = value
        return self

    def duration(self, value: float):
        self._duration = value
        return self

    def angle_a(self, value: float):
        self._angle_a = value
        return self

    def angle_b(self, value: float):
        self._angle_b = value
        return self

    def _build(self):
        kwargs = {}
        if self._servo is not _UNSET:
            kwargs['servo'] = self._servo
        if self._duration is not _UNSET:
            kwargs['duration'] = self._duration
        if self._angle_a is not _UNSET:
            kwargs['angle_a'] = self._angle_a
        if self._angle_b is not _UNSET:
            kwargs['angle_b'] = self._angle_b
        return ShakeServo(**kwargs)


@dsl(tags=['servo', 'actuator'])
def shake_servo(servo: Servo | ServoPreset = _UNSET, duration: float = _UNSET, angle_a: float = _UNSET, angle_b: float = _UNSET):
    """
    Oscillate a servo back and forth between two angles for a set time.

    Rapidly alternates the servo between ``angle_a`` and ``angle_b`` for
    the given duration. The dwell time at each angle is automatically
    estimated from the angular distance so the servo has time to
    physically reach each endpoint before reversing. Useful for shaking
    objects loose or signalling the operator.

    Args:
        servo: The servo to control, obtained from the robot hardware map (e.g. ``robot.servo(1)``).
        duration: Total oscillation time in seconds. Must be >= 0.
        angle_a: First oscillation endpoint in degrees.
        angle_b: Second oscillation endpoint in degrees.

    Returns:
        A ShakeServoBuilder (chainable via ``.servo()``, ``.duration()``, ``.angle_a()``, ``.angle_b()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.servo import shake_servo

        # Shake a sorting tray for 3 seconds between 60 and 120 degrees
        shake_servo(robot.servo(1), duration=3.0, angle_a=60.0, angle_b=120.0)
    """
    b = ShakeServoBuilder()
    if servo is not _UNSET:
        b._servo = servo
    if duration is not _UNSET:
        b._duration = duration
    if angle_a is not _UNSET:
        b._angle_a = angle_a
    if angle_b is not _UNSET:
        b._angle_b = angle_b
    return b


class SlowServoBuilder(StepBuilder):
    """Builder for SlowServo. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._servo = _UNSET
        self._angle = _UNSET
        self._speed = 60.0
        self._easing = Easing.EASE_IN_OUT

    def servo(self, value: Servo | ServoPreset):
        self._servo = value
        return self

    def angle(self, value: float):
        self._angle = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def easing(self, value: Easing | EasingFunc):
        self._easing = value
        return self

    def _build(self):
        kwargs = {}
        if self._servo is not _UNSET:
            kwargs['servo'] = self._servo
        if self._angle is not _UNSET:
            kwargs['angle'] = self._angle
        kwargs['speed'] = self._speed
        kwargs['easing'] = self._easing
        return SlowServo(**kwargs)


@dsl(tags=['servo', 'actuator'])
def slow_servo(servo: Servo | ServoPreset = _UNSET, angle: float = _UNSET, speed: float = 60.0, easing: Easing | EasingFunc = Easing.EASE_IN_OUT):
    """
    Move a servo to an angle with smooth interpolated motion.

    Instead of commanding the servo to jump straight to the target (as
    ``servo()`` does), this step interpolates through intermediate
    positions using an easing curve. The default is smoothstep
    ease-in-ease-out (3t² − 2t³), which gives gentle acceleration and
    deceleration. Other curves can be selected via the ``easing``
    parameter.

    The total move duration is derived from the angular distance divided
    by ``speed``. Intermediate positions are updated at ~10 Hz.

    Args:
        servo: The servo to control, obtained from the robot hardware map (e.g. ``robot.servo(0)``).
        angle: Target angle in degrees.
        speed: Movement speed in degrees per second. Must be positive. Defaults to 60.0 deg/s.
        easing: Interpolation curve. Pass an :class:`Easing` member or any callable ``(t: float) -> float`` mapping [0, 1] → [0, 1]. Defaults to ``Easing.EASE_IN_OUT``.

    Returns:
        A SlowServoBuilder (chainable via ``.servo()``, ``.angle()``, ``.speed()``, ``.easing()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.servo import slow_servo, Easing

        # Gently lower the arm servo to 20 degrees at 45 deg/s
        slow_servo(robot.servo(0), angle=20.0, speed=45.0)

        # Linear (constant-speed) motion
        slow_servo(robot.servo(0), angle=150.0, easing=Easing.LINEAR)

        # Ease-out only (fast start, slow stop)
        slow_servo(robot.servo(0), angle=0.0, easing=Easing.EASE_OUT)
    """
    b = SlowServoBuilder()
    if servo is not _UNSET:
        b._servo = servo
    if angle is not _UNSET:
        b._angle = angle
    b._speed = speed
    b._easing = easing
    return b


class FullyDisableServosBuilder(StepBuilder):
    """Builder for FullyDisableServos. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()

    def _build(self):
        kwargs = {}
        return FullyDisableServos(**kwargs)


@dsl(tags=['servo', 'actuator'])
def fully_disable_servos():
    """
    Fully disable all servo outputs, removing all power from the servo pins.

    Commands the firmware to enter the fully-disabled servo mode for every
    servo port. In this mode, no PWM signal is sent and the servos can be
    moved freely by hand. This is useful for saving power or when the
    servos should not hold position (e.g. at the end of a run).

    Servos will automatically re-enable when a new position command is
    sent (e.g. via ``servo()`` or ``slow_servo()``).

    Returns:
        A FullyDisableServosBuilder (chainable via , ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.servo import fully_disable_servos

        # Release all servos at the end of a mission
        fully_disable_servos()
    """
    b = FullyDisableServosBuilder()
    return b


__all__ = ['ShakeServoBuilder', 'shake_servo', 'SlowServoBuilder', 'slow_servo', 'FullyDisableServosBuilder', 'fully_disable_servos']
