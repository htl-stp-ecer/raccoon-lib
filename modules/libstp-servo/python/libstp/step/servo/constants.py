from __future__ import annotations

# Angular limits in degrees. Keep within the HAL safety checks (0..180).
SERVO_MIN_ANGLE: float = 0.0
SERVO_MAX_ANGLE: float = 180.0

# Approximate free-running speed (degrees per second). This is used only for
# estimating how long to wait after commanding a move so sequences can yield.
# Datasheet reference: ~0.17s / 60° @ 4.8V -> ~353 deg/s. We stay conservative.
SERVO_SPEED_DPS: float = 60.0 / 0.3

# Servo position range matches the HAL expectations (degrees).
SERVO_MIN_POSITION: int = 0
SERVO_MAX_POSITION: int = 180

__all__ = [
    "SERVO_MIN_ANGLE",
    "SERVO_MAX_ANGLE",
    "SERVO_SPEED_DPS",
    "SERVO_MIN_POSITION",
    "SERVO_MAX_POSITION",
]
