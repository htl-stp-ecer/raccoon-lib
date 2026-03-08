"""Drive forward or backward until an IR sensor detects a surface color."""
from libstp.sensor_ir import IRSensor
from typing import Union

from ... import dsl, dsl_step
from .core import MoveUntil, MoveUntilConfig, SurfaceColor


@dsl(hidden=True)
def drive_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        forward_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """Drive forward or backward until any sensor detects a black surface.

    Commands a constant forward (or backward) velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. This is
    the low-level variant that accepts a signed speed; prefer the directional
    helpers ``drive_forward_until_black`` / ``drive_backward_until_black`` for
    most use cases.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            the target color.
        forward_speed: Driving speed in m/s. Positive values drive forward,
            negative values drive backward.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for the target color before the step considers the
            condition met. Defaults to 0.7.

    Returns:
        MoveUntil: A configured motion step that can be executed by the
        step runner.

    Example::

        from libstp.sensor_ir import IRSensor

        front_ir = IRSensor(0)
        step = drive_until_black(front_ir, forward_speed=0.5)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        forward_speed=forward_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(hidden=True)
def drive_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        forward_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """Drive forward or backward until any sensor detects a white surface.

    Commands a constant forward (or backward) velocity and polls the given IR
    sensor(s) each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. This is
    the low-level variant that accepts a signed speed; prefer the directional
    helpers ``drive_forward_until_white`` / ``drive_backward_until_white`` for
    most use cases.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            the target color.
        forward_speed: Driving speed in m/s. Positive values drive forward,
            negative values drive backward.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for the target color before the step considers the
            condition met. Defaults to 0.7.

    Returns:
        MoveUntil: A configured motion step that can be executed by the
        step runner.

    Example::

        from libstp.sensor_ir import IRSensor

        front_ir = IRSensor(0)
        step = drive_until_white(front_ir, forward_speed=-0.3)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=forward_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl_step(tags=["motion", "sensor"])
class DriveForwardUntilBlack(MoveUntil):
    """Drive forward until any sensor detects a black surface.

    Commands a constant forward velocity and polls the given IR sensor(s) each
    control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (forward) regardless of the sign passed in.

    This is the recommended way to drive forward until a black line or region
    is detected, for example when approaching a Botball scoring zone bounded
    by black tape.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            black.
        speed: Forward driving speed in m/s. The absolute value is used, so
            negative inputs are treated as positive. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for black before the step considers the condition met.
            Lower values make detection more sensitive but increase the risk of
            false positives. Defaults to 0.7.

    Returns:
        DriveForwardUntilBlack: A configured motion step that drives forward
        and stops when black is detected.

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
            forward_speed=abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"DriveForwardUntilBlack(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class DriveForwardUntilWhite(MoveUntil):
    """Drive forward until any sensor detects a white surface.

    Commands a constant forward velocity and polls the given IR sensor(s) each
    control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is forced positive (forward) regardless of the sign passed in.

    Useful for driving across a dark region until the robot reaches a white
    surface, for example crossing black tape to re-enter the playing field.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            white.
        speed: Forward driving speed in m/s. The absolute value is used, so
            negative inputs are treated as positive. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for white before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        DriveForwardUntilWhite: A configured motion step that drives forward
        and stops when white is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveForwardUntilWhite

        front_ir = IRSensor(0)

        # Drive forward at default speed until white is found
        DriveForwardUntilWhite(front_ir)

        # Slower approach with stricter detection
        DriveForwardUntilWhite(front_ir, speed=0.3, confidence_threshold=0.9)
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
            forward_speed=abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"DriveForwardUntilWhite(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class DriveBackwardUntilBlack(MoveUntil):
    """Drive backward until any sensor detects a black surface.

    Commands a constant backward velocity and polls the given IR sensor(s)
    each control cycle. The step completes as soon as any sensor's
    ``probabilityOfBlack()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally, so you should pass a positive value.

    Useful for backing up toward a black boundary line, for example
    repositioning before a scoring action.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            black.
        speed: Backward driving speed in m/s. Pass a positive value; the sign
            is negated internally. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for black before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        DriveBackwardUntilBlack: A configured motion step that drives backward
        and stops when black is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveBackwardUntilBlack

        rear_ir = IRSensor(2)

        # Back up at 0.4 m/s until the rear sensor detects black
        DriveBackwardUntilBlack(rear_ir, speed=0.4)
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
            forward_speed=-abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"DriveBackwardUntilBlack(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class DriveBackwardUntilWhite(MoveUntil):
    """Drive backward until any sensor detects a white surface.

    Commands a constant backward velocity and polls the given IR sensor(s)
    each control cycle. The step completes as soon as any sensor's
    ``probabilityOfWhite()`` meets or exceeds ``confidence_threshold``. The
    speed is negated internally, so you should pass a positive value.

    Args:
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            white.
        speed: Backward driving speed in m/s. Pass a positive value; the sign
            is negated internally. Defaults to 1.0.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for white before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        DriveBackwardUntilWhite: A configured motion step that drives backward
        and stops when white is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveBackwardUntilWhite

        rear_ir = IRSensor(2)

        # Back up slowly until the rear sensor sees white
        DriveBackwardUntilWhite(rear_ir, speed=0.3)
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
            forward_speed=-abs(speed),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"DriveBackwardUntilWhite(speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"
