"""Strafe laterally until an IR sensor detects a surface color.

Prerequisites:
    Strafing requires a mecanum or holonomic drivetrain capable of lateral
    movement. These steps will have no lateral effect on a differential-drive
    robot.
"""
from libstp.sensor_ir import IRSensor
from typing import Union

from ... import dsl, dsl_step
from .core import MoveUntil, MoveUntilConfig, SurfaceColor


@dsl(hidden=True)
def strafe_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        strafe_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """Strafe laterally until any sensor detects a black surface.

    Commands a constant lateral velocity and polls the given IR sensor(s) each
    control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. This is
    the low-level variant that accepts a signed strafe speed; prefer the
    directional helpers ``strafe_left_until_black`` /
    ``strafe_right_until_black`` for most use cases.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            black.
        strafe_speed: Lateral speed in m/s. Positive values strafe left,
            negative values strafe right.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for black before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        MoveUntil: A configured motion step that strafes and stops when black
        is detected.

    Example::

        from libstp.sensor_ir import IRSensor

        side_ir = IRSensor(1)
        # Strafe left at 0.2 m/s until black is found
        step = strafe_until_black(side_ir, strafe_speed=0.2)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        strafe_speed=strafe_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(hidden=True)
def strafe_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        strafe_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """Strafe laterally until any sensor detects a white surface.

    Commands a constant lateral velocity and polls the given IR sensor(s) each
    control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. This is
    the low-level variant that accepts a signed strafe speed; prefer the
    directional helpers ``strafe_left_until_white`` /
    ``strafe_right_until_white`` for most use cases.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            white.
        strafe_speed: Lateral speed in m/s. Positive values strafe left,
            negative values strafe right.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for white before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        MoveUntil: A configured motion step that strafes and stops when white
        is detected.

    Example::

        from libstp.sensor_ir import IRSensor

        side_ir = IRSensor(1)
        # Strafe right at 0.2 m/s until white is found
        step = strafe_until_white(side_ir, strafe_speed=-0.2)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        strafe_speed=strafe_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl_step(tags=["motion", "sensor"])
class StrafeLeftUntilBlack(MoveUntil):
    """Strafe left until any sensor detects a black surface.

    Commands a constant leftward lateral velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally to produce leftward motion, so you should pass
    a positive value.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            black.
        speed: Lateral speed in m/s. Pass a positive value; the sign is
            negated internally to strafe left. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for black before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        StrafeLeftUntilBlack: A configured motion step that strafes left and
        stops when black is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import StrafeLeftUntilBlack

        left_ir = IRSensor(1)

        # Strafe left at 0.2 m/s until the left sensor finds a black line
        StrafeLeftUntilBlack(left_ir, speed=0.2)
    """

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            speed: float = 0.3,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.BLACK,
            strafe_speed=-abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"StrafeLeftUntilBlack(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class StrafeLeftUntilWhite(MoveUntil):
    """Strafe left until any sensor detects a white surface.

    Commands a constant leftward lateral velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally to produce leftward motion, so you should pass
    a positive value.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            white.
        speed: Lateral speed in m/s. Pass a positive value; the sign is
            negated internally to strafe left. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for white before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        StrafeLeftUntilWhite: A configured motion step that strafes left and
        stops when white is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import StrafeLeftUntilWhite

        left_ir = IRSensor(1)

        # Strafe left at default speed until the left sensor finds white
        StrafeLeftUntilWhite(left_ir)
    """

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            speed: float = 0.3,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.WHITE,
            strafe_speed=-abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"StrafeLeftUntilWhite(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class StrafeRightUntilBlack(MoveUntil):
    """Strafe right until any sensor detects a black surface.

    Commands a constant rightward lateral velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (rightward) regardless of the sign passed in.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            black.
        speed: Lateral speed in m/s. The absolute value is used, so negative
            inputs are treated as positive. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for black before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        StrafeRightUntilBlack: A configured motion step that strafes right and
        stops when black is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import StrafeRightUntilBlack

        right_ir = IRSensor(3)

        # Strafe right at 0.25 m/s until the right sensor hits a black line
        StrafeRightUntilBlack(right_ir, speed=0.25)
    """

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            speed: float = 0.3,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.BLACK,
            strafe_speed=abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"StrafeRightUntilBlack(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class StrafeRightUntilWhite(MoveUntil):
    """Strafe right until any sensor detects a white surface.

    Commands a constant rightward lateral velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (rightward) regardless of the sign passed in.

    Prerequisites:
        Requires a mecanum or holonomic drivetrain capable of lateral movement.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            white.
        speed: Lateral speed in m/s. The absolute value is used, so negative
            inputs are treated as positive. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for white before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        StrafeRightUntilWhite: A configured motion step that strafes right and
        stops when white is detected.

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

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            speed: float = 0.3,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.WHITE,
            strafe_speed=abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"StrafeRightUntilWhite(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"
