"""Turn until sensor factories."""
from libstp.sensor_ir import IRSensor
from typing import Union

from ... import dsl
from .core import MoveUntil, MoveUntilConfig, SurfaceColor


@dsl(hidden=True)
def turn_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        angular_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        angular_speed: Speed in rad/s (positive = CCW, negative = CW)
        confidence_threshold: Probability threshold for detection (0-1)
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
    """
    Turn until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        angular_speed: Speed in rad/s (positive = CCW, negative = CW)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        angular_speed=angular_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def turn_left_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn left (CCW) until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Angular speed in rad/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        angular_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def turn_left_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn left (CCW) until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Angular speed in rad/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        angular_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def turn_right_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn right (CW) until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Angular speed in rad/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        angular_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def turn_right_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn right (CW) until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Angular speed in rad/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        angular_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))
