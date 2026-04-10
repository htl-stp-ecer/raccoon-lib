from __future__ import annotations

from .constants import SERVO_SPEED_DPS


def estimate_servo_move_time(start_angle: float, end_angle: float) -> float:
    """
    Estimate time (in seconds) for a servo to move between two angles.
    Returns 0 when the delta is zero to avoid unnecessary sleeps.
    """
    delta = abs(end_angle - start_angle)
    speed = SERVO_SPEED_DPS if SERVO_SPEED_DPS > 0 else 1.0
    return delta / speed


__all__ = [
    "estimate_servo_move_time",
]
