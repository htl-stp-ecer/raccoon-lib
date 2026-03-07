"""Factory functions for driving at an arbitrary angle until an IR sensor detects a surface color.

Prerequisites:
    Diagonal driving decomposes the velocity into forward and strafe components,
    so it requires a mecanum or holonomic drivetrain capable of lateral movement.
"""
import math

from libstp.sensor_ir import IRSensor
from typing import Union

from ... import dsl
from .core import MoveUntil, MoveUntilConfig, SurfaceColor


@dsl(tags=["motion", "sensor"])
def drive_angle_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        angle_deg: float,
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """Drive at an arbitrary angle until any sensor detects a black surface.

    Decomposes the desired travel direction into forward and strafe velocity
    components using trigonometry (``cos`` for forward, ``sin`` for strafe) and
    commands them simultaneously. The robot's heading does not change -- only
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
        MoveUntil: A configured motion step that drives at the specified angle
        and stops when black is detected.

    Example::

        from libstp.sensor_ir import IRSensor

        front_ir = IRSensor(0)

        # Drive diagonally forward-right (45 deg) until black is detected
        step = drive_angle_until_black(front_ir, angle_deg=45, speed=0.3)

        # Drive purely left (-90 deg) until black -- equivalent to strafe left
        step = drive_angle_until_black(front_ir, angle_deg=-90, speed=0.2)
    """
    angle_rad = math.radians(angle_deg)
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        forward_speed=speed * math.cos(angle_rad),
        strafe_speed=speed * math.sin(angle_rad),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def drive_angle_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        angle_deg: float,
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """Drive at an arbitrary angle until any sensor detects a white surface.

    Decomposes the desired travel direction into forward and strafe velocity
    components using trigonometry (``cos`` for forward, ``sin`` for strafe) and
    commands them simultaneously. The robot's heading does not change -- only
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
        MoveUntil: A configured motion step that drives at the specified angle
        and stops when white is detected.

    Example::

        from libstp.sensor_ir import IRSensor

        front_ir = IRSensor(0)

        # Drive diagonally forward-left (-45 deg) until white is detected
        step = drive_angle_until_white(front_ir, angle_deg=-45, speed=0.3)

        # Drive backward (180 deg) until white -- equivalent to drive backward
        step = drive_angle_until_white(front_ir, angle_deg=180, speed=0.2)
    """
    angle_rad = math.radians(angle_deg)
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=speed * math.cos(angle_rad),
        strafe_speed=speed * math.sin(angle_rad),
        confidence_threshold=confidence_threshold,
    ))
