"""Drive-at-angle until sensor factories."""
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
    """
    Drive at an arbitrary angle until any sensor detects black.

    Decomposes the angle + speed into forward and strafe components.
    Angle convention: 0 = forward, 90 = right, -90 = left, 180 = backward.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        angle_deg: Travel angle in degrees (0=forward, 90=right, -90=left)
        speed: Speed magnitude in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
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
    """
    Drive at an arbitrary angle until any sensor detects white.

    Decomposes the angle + speed into forward and strafe components.
    Angle convention: 0 = forward, 90 = right, -90 = left, 180 = backward.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        angle_deg: Travel angle in degrees (0=forward, 90=right, -90=left)
        speed: Speed magnitude in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    angle_rad = math.radians(angle_deg)
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=speed * math.cos(angle_rad),
        strafe_speed=speed * math.sin(angle_rad),
        confidence_threshold=confidence_threshold,
    ))
