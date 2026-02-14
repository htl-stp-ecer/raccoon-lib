"""Strafe until sensor factories."""
from libstp.sensor_ir import IRSensor
from typing import Union

from ... import dsl
from .core import MoveUntil, MoveUntilConfig, SurfaceColor


@dsl(hidden=True)
def strafe_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        strafe_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        strafe_speed: Speed in m/s (positive = left, negative = right)
        confidence_threshold: Probability threshold for detection (0-1)
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
    """
    Strafe until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        strafe_speed: Speed in m/s (positive = left, negative = right)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        strafe_speed=strafe_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def strafe_left_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe left until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Strafe speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        strafe_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def strafe_left_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe left until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Strafe speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        strafe_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def strafe_right_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe right until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Strafe speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        strafe_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def strafe_right_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe right until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Strafe speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        strafe_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))
