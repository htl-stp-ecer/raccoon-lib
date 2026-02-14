"""Drive forward/backward until sensor factories."""
from libstp.sensor_ir import IRSensor
from typing import Union

from ... import dsl
from .core import MoveUntil, MoveUntilConfig, SurfaceColor


@dsl(hidden=True)
def drive_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        forward_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive forward/backward until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        forward_speed: Speed in m/s (positive = forward, negative = backward)
        confidence_threshold: Probability threshold for detection (0-1)
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
    """
    Drive forward/backward until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        forward_speed: Speed in m/s (positive = forward, negative = backward)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=forward_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def drive_forward_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive forward until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Forward speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        forward_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def drive_forward_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive forward until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Forward speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def drive_backward_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive backward until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Backward speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        forward_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def drive_backward_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive backward until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Backward speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))
