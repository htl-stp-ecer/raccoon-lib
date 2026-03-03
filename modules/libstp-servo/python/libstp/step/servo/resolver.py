from __future__ import annotations

from dataclasses import dataclass
from typing import Union

from libstp.hal import Servo
from libstp.robot.api import GenericRobot


class ServoResolutionError(RuntimeError):
    """Raised when a servo reference cannot be resolved from the robot."""


@dataclass(frozen=True)
class ResolvedServo:
    """Resolved servo lookup result with the chosen display name and object."""

    name: str
    servo: Servo


def resolve_servo(servo_ref: Union[str, Servo], robot: GenericRobot) -> ResolvedServo:
    """
    Resolve a servo reference (instance or defs attribute name) into a concrete Servo.
    """
    if isinstance(servo_ref, Servo):
        label = f"port-{getattr(servo_ref, 'port', 'unknown')}"
        return ResolvedServo(name=label, servo=servo_ref)

    if isinstance(servo_ref, str):
        if not hasattr(robot, "defs"):
            raise ServoResolutionError("Robot has no defs attached for servo lookup")

        defs = robot.defs
        if not hasattr(defs, servo_ref):
            raise ServoResolutionError(f"defs is missing servo '{servo_ref}'")

        candidate = getattr(defs, servo_ref)
        if not isinstance(candidate, Servo):
            raise ServoResolutionError(
                f"defs.{servo_ref} is a {type(candidate).__name__}, expected Servo"
            )

        return ResolvedServo(name=servo_ref, servo=candidate)

    raise ServoResolutionError(
        f"Unsupported servo reference type: {type(servo_ref).__name__}"
    )


__all__ = ["ResolvedServo", "ServoResolutionError", "resolve_servo"]
