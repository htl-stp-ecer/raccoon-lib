"""Turn in place until an IR sensor detects a surface color."""
from libstp.sensor_ir import IRSensor
from typing import Union

from ... import dsl, dsl_step
from .core import MoveUntil, MoveUntilConfig, SurfaceColor


@dsl(hidden=True)
def turn_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        angular_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """Turn in place until any sensor detects a black surface.

    Commands a constant angular velocity and polls the given IR sensor(s) each
    control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. This is
    the low-level variant that accepts a signed angular speed; prefer the
    directional helpers ``turn_left_until_black`` / ``turn_right_until_black``
    for most use cases.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            black.
        angular_speed: Rotational speed in rad/s. Positive values rotate
            counter-clockwise (left), negative values rotate clockwise (right).
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for black before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        MoveUntil: A configured motion step that rotates in place and stops
        when black is detected.

    Example::

        from libstp.sensor_ir import IRSensor

        side_ir = IRSensor(1)
        # Rotate CCW at 0.5 rad/s until the sensor finds black
        step = turn_until_black(side_ir, angular_speed=0.5)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        angular_speed=angular_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(hidden=True)
def turn_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        angular_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """Turn in place until any sensor detects a white surface.

    Commands a constant angular velocity and polls the given IR sensor(s) each
    control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. This is
    the low-level variant that accepts a signed angular speed; prefer the
    directional helpers ``turn_left_until_white`` / ``turn_right_until_white``
    for most use cases.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            white.
        angular_speed: Rotational speed in rad/s. Positive values rotate
            counter-clockwise (left), negative values rotate clockwise (right).
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for white before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        MoveUntil: A configured motion step that rotates in place and stops
        when white is detected.

    Example::

        from libstp.sensor_ir import IRSensor

        side_ir = IRSensor(1)
        # Rotate CW at 0.5 rad/s until the sensor finds white
        step = turn_until_white(side_ir, angular_speed=-0.5)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        angular_speed=angular_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl_step(tags=["motion", "sensor"])
class TurnLeftUntilBlack(MoveUntil):
    """Turn left (counter-clockwise) until any sensor detects a black surface.

    Commands a constant counter-clockwise angular velocity and polls the given
    IR sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (CCW) regardless of the sign passed in.

    A common use case is sweeping a side-mounted IR sensor across the field to
    locate a black line, then stopping precisely when found.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            black.
        speed: Angular speed in rad/s. The absolute value is used, so negative
            inputs are treated as positive. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for black before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        TurnLeftUntilBlack: A configured motion step that turns left and stops
        when black is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import TurnLeftUntilBlack

        right_ir = IRSensor(3)

        # Sweep left at 0.8 rad/s until the right sensor crosses a black line
        TurnLeftUntilBlack(right_ir, speed=0.8)
    """

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            speed: float = 1.0,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.BLACK,
            angular_speed=abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"TurnLeftUntilBlack(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class TurnLeftUntilWhite(MoveUntil):
    """Turn left (counter-clockwise) until any sensor detects a white surface.

    Commands a constant counter-clockwise angular velocity and polls the given
    IR sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (CCW) regardless of the sign passed in.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            white.
        speed: Angular speed in rad/s. The absolute value is used, so negative
            inputs are treated as positive. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for white before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        TurnLeftUntilWhite: A configured motion step that turns left and stops
        when white is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import TurnLeftUntilWhite

        right_ir = IRSensor(3)

        # Sweep left at 0.6 rad/s until white surface is found
        TurnLeftUntilWhite(right_ir, speed=0.6)
    """

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            speed: float = 1.0,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.WHITE,
            angular_speed=abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"TurnLeftUntilWhite(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class TurnRightUntilBlack(MoveUntil):
    """Turn right (clockwise) until any sensor detects a black surface.

    Commands a constant clockwise angular velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally to produce clockwise rotation, so you should
    pass a positive value.

    A common use case is sweeping a side-mounted IR sensor across the field to
    locate a black line, then stopping precisely when found.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            black.
        speed: Angular speed in rad/s. Pass a positive value; the sign is
            negated internally to rotate clockwise. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for black before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        TurnRightUntilBlack: A configured motion step that turns right and
        stops when black is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import TurnRightUntilBlack

        left_ir = IRSensor(1)

        # Sweep right at 0.8 rad/s until the left sensor crosses a black line
        TurnRightUntilBlack(left_ir, speed=0.8)
    """

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            speed: float = 1.0,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.BLACK,
            angular_speed=-abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"TurnRightUntilBlack(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class TurnRightUntilWhite(MoveUntil):
    """Turn right (clockwise) until any sensor detects a white surface.

    Commands a constant clockwise angular velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally to produce clockwise rotation, so you should
    pass a positive value.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            white.
        speed: Angular speed in rad/s. Pass a positive value; the sign is
            negated internally to rotate clockwise. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for white before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        TurnRightUntilWhite: A configured motion step that turns right and
        stops when white is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import TurnRightUntilWhite

        left_ir = IRSensor(1)

        # Sweep right slowly until the left sensor finds white
        TurnRightUntilWhite(left_ir, speed=0.5)
    """

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            speed: float = 1.0,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.WHITE,
            angular_speed=-abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"TurnRightUntilWhite(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"
