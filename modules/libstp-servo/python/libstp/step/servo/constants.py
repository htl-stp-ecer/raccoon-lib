from __future__ import annotations

# Approximate free-running speed (degrees per second). This is used only for
# estimating how long to wait after commanding a move so sequences can yield.
# Datasheet reference: ~0.17s / 60° @ 4.8V -> ~353 deg/s. We stay conservative.
SERVO_SPEED_DPS: float = 60.0 / 0.3

__all__ = [
    "SERVO_SPEED_DPS",
]
