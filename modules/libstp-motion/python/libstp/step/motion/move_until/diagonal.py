"""Drive at an arbitrary angle until an IR sensor detects a surface color.

Prerequisites:
    Diagonal driving decomposes the velocity into forward and strafe components,
    so it requires a mecanum or holonomic drivetrain capable of lateral movement.
"""
import math

from libstp.sensor_ir import IRSensor
from typing import Union

from ... import dsl_step
from .core import MoveUntil, MoveUntilConfig, SurfaceColor


@dsl_step(tags=["motion", "sensor"])
class DriveAngleUntilBlack(MoveUntil):
    """Drive at an arbitrary angle until any sensor detects a black surface.

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
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            black.
        angle_deg: Travel angle in degrees relative to the robot's forward
            axis. 0 = forward, 90 = right, -90 = left, 180 = backward.
        speed: Speed magnitude in m/s (positive value). This is the overall
            speed; it is decomposed into forward and strafe components based on
            the angle. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for black before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        DriveAngleUntilBlack: A configured motion step that drives at the
        specified angle and stops when black is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveAngleUntilBlack

        front_ir = IRSensor(0)

        # Drive diagonally forward-right (45 deg) until black is detected
        DriveAngleUntilBlack(front_ir, angle_deg=45, speed=0.3)

        # Drive purely left (-90 deg) until black -- equivalent to strafe left
        DriveAngleUntilBlack(front_ir, angle_deg=-90, speed=0.2)
    """

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            angle_deg: float,
            speed: float = 0.3,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._angle_deg = angle_deg
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        angle_rad = math.radians(angle_deg)
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.BLACK,
            forward_speed=speed * math.cos(angle_rad),
            strafe_speed=speed * math.sin(angle_rad),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"DriveAngleUntilBlack(angle={self._angle_deg:.1f}deg, speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"


@dsl_step(tags=["motion", "sensor"])
class DriveAngleUntilWhite(MoveUntil):
    """Drive at an arbitrary angle until any sensor detects a white surface.

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
        sensor: A single :class:`~libstp.sensor_ir.IRSensor` or a list of
            sensors. The step triggers when **any** sensor in the list detects
            white.
        angle_deg: Travel angle in degrees relative to the robot's forward
            axis. 0 = forward, 90 = right, -90 = left, 180 = backward.
        speed: Speed magnitude in m/s (positive value). This is the overall
            speed; it is decomposed into forward and strafe components based on
            the angle. Defaults to 0.3.
        confidence_threshold: Minimum probability (0.0 -- 1.0) that the sensor
            must report for white before the step considers the condition met.
            Defaults to 0.7.

    Returns:
        DriveAngleUntilWhite: A configured motion step that drives at the
        specified angle and stops when white is detected.

    Example::

        from libstp.sensor_ir import IRSensor
        from libstp.step.motion.move_until import DriveAngleUntilWhite

        front_ir = IRSensor(0)

        # Drive diagonally forward-left (-45 deg) until white is detected
        DriveAngleUntilWhite(front_ir, angle_deg=-45, speed=0.3)

        # Drive backward (180 deg) until white -- equivalent to drive backward
        DriveAngleUntilWhite(front_ir, angle_deg=180, speed=0.2)
    """

    def __init__(
            self,
            sensor: Union[IRSensor, list[IRSensor]],
            angle_deg: float,
            speed: float = 0.3,
            confidence_threshold: float = 0.7,
    ) -> None:
        self._angle_deg = angle_deg
        self._speed = speed
        self._confidence_threshold = confidence_threshold
        angle_rad = math.radians(angle_deg)
        super().__init__(MoveUntilConfig(
            sensor=sensor,
            target=SurfaceColor.WHITE,
            forward_speed=speed * math.cos(angle_rad),
            strafe_speed=speed * math.sin(angle_rad),
            confidence_threshold=confidence_threshold,
        ))

    def _generate_signature(self) -> str:
        return f"DriveAngleUntilWhite(angle={self._angle_deg:.1f}deg, speed={self._speed:.2f}, threshold={self._confidence_threshold:.2f})"
