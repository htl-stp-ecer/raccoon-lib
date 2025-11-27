from __future__ import annotations

from .constants import (
    SERVO_MAX_ANGLE,
    SERVO_MAX_POSITION,
    SERVO_MIN_ANGLE,
    SERVO_MIN_POSITION,
    SERVO_SPEED_DPS,
)


def clamp_angle(angle: float) -> float:
    """Clamp an angle to the supported servo range."""
    return max(SERVO_MIN_ANGLE, min(angle, SERVO_MAX_ANGLE))


def clamp_position(position: float) -> float:
    """Clamp a servo position to the supported range."""
    return max(SERVO_MIN_POSITION, min(position, SERVO_MAX_POSITION))


def estimate_servo_move_time(start_angle: float, end_angle: float) -> float:
    """
    Estimate time (in seconds) for a servo to move between two angles.
    Returns 0 when the delta is zero to avoid unnecessary sleeps.
    """
    delta = abs(clamp_angle(end_angle) - clamp_angle(start_angle))
    speed = SERVO_SPEED_DPS if SERVO_SPEED_DPS > 0 else 1.0
    return delta / speed


def angle_to_position(angle: float) -> int:
    """
    Convert an angle (degrees) to the HAL position representation.
    Currently this is a 1:1 mapping onto degrees with clamping.
    """
    clamped = clamp_angle(angle)
    scale = SERVO_MAX_POSITION / SERVO_MAX_ANGLE
    return int(round(scale * clamped))


def position_to_angle(position: int) -> float:
    """Convert a HAL position reading to an angle in degrees."""
    clamped = clamp_position(position)
    scale = SERVO_MAX_ANGLE / SERVO_MAX_POSITION
    return scale * clamped


__all__ = [
    "angle_to_position",
    "position_to_angle",
    "estimate_servo_move_time",
    "clamp_angle",
    "clamp_position",
]
