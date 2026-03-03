from __future__ import annotations

import asyncio
from typing import Optional, Union

from libstp.hal import Servo
from libstp.robot.api import GenericRobot
from libstp.step import Step
from libstp.step.annotation import dsl

from .constants import SERVO_MAX_ANGLE, SERVO_MIN_ANGLE
from .resolver import resolve_servo
from .utility import (
    angle_to_position,
    estimate_servo_move_time,
    position_to_angle,
)


@dsl(hidden=True)
class SetServoPosition(Step):
    """Set a servo to a target angle and optionally wait for the move to finish."""

    def __init__(
        self, servo: Servo, target_angle: float, duration: Optional[float] = None
    ) -> None:
        super().__init__()
        self._servo_ref = servo
        self._target_angle = float(target_angle)
        self._duration = float(duration) if duration is not None else None
        self._validate_inputs()

    def _validate_inputs(self) -> None:
        if not (SERVO_MIN_ANGLE <= self._target_angle <= SERVO_MAX_ANGLE):
            raise ValueError(
                f"Target angle must be between {SERVO_MIN_ANGLE} and {SERVO_MAX_ANGLE}, "
                f"got {self._target_angle}"
            )

        if self._duration is not None and self._duration < 0:
            raise ValueError("Duration must be >= 0")

    def _generate_signature(self) -> str:
        servo_label = f"port-{getattr(self._servo_ref, 'port', 'na')}"
        return f"SetServoPosition(servo={servo_label},angle={self._target_angle},duration={self._duration})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        target_position = angle_to_position(self._target_angle)

        self._servo_ref.enable()

        duration = self._duration
        if duration is None:
            current_angle = position_to_angle(self._servo_ref.get_position())
            duration = estimate_servo_move_time(current_angle, self._target_angle)

        self._servo_ref.set_position(target_position)
        if duration and duration > 0:
            await asyncio.sleep(duration)

@dsl(tags=["servo", "actuator"])
def servo(servo: Servo, angle: float) -> SetServoPosition:
    """Create a step to set a servo to a specific angle with estimated timing."""
    return SetServoPosition(servo=servo, target_angle=angle, duration=None)



@dsl(hidden=True)
class ShakeServo(Step):
    """Oscillate a servo between two angles for a fixed amount of time."""

    def __init__(
        self, servo: Servo, duration: float, angle_a: float, angle_b: float
    ) -> None:
        super().__init__()
        self._servo_ref = servo
        self._duration = float(duration)
        self._angle_a = float(angle_a)
        self._angle_b = float(angle_b)
        self._validate_inputs()

    def _validate_inputs(self) -> None:
        if self._duration < 0:
            raise ValueError("Duration must be >= 0")

        for label, angle in (("angle_a", self._angle_a), ("angle_b", self._angle_b)):
            if not (SERVO_MIN_ANGLE <= angle <= SERVO_MAX_ANGLE):
                raise ValueError(
                    f"{label} must be between {SERVO_MIN_ANGLE} and {SERVO_MAX_ANGLE}, got {angle}"
                )

    def _generate_signature(self) -> str:
        servo_label = f"port-{getattr(self._servo_ref, 'port', 'na')}"
        return (
            f"ShakeServo(servo={servo_label},duration={self._duration},"
            f"a={self._angle_a},b={self._angle_b})"
        )

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._servo_ref.enable()

        pos_a = angle_to_position(self._angle_a)
        pos_b = angle_to_position(self._angle_b)

        if pos_a == pos_b or self._duration == 0:
            self._servo_ref.set_position(pos_a)
            if self._duration > 0:
                await asyncio.sleep(self._duration)
            return

        move_time = estimate_servo_move_time(self._angle_a, self._angle_b)
        # Guard against a zero-time oscillation; the loop clock resolution makes 10ms safe.
        move_time = max(move_time, 0.01)

        loop = asyncio.get_running_loop()
        end_time = loop.time() + self._duration

        while loop.time() < end_time:
            self._servo_ref.set_position(pos_a)
            await asyncio.sleep(move_time)
            if loop.time() >= end_time:
                break
            self._servo_ref.set_position(pos_b)
            await asyncio.sleep(move_time)


@dsl(tags=["servo", "actuator"])
def shake_servo(servo: Servo, duration: float, angle_a: float, angle_b: float) -> ShakeServo:
    """Create a step to shake a servo between two angles for a fixed duration."""
    return ShakeServo(servo=servo, duration=duration, angle_a=angle_a, angle_b=angle_b)


def _ease_in_out(t: float) -> float:
    """Smoothstep ease-in-ease-out: 3t^2 - 2t^3."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


_EASE_SERVO_TICK = 1 / 10


@dsl(hidden=True)
class EaseServo(Step):
    """Move a servo to a target angle with ease-in/ease-out timing."""

    def __init__(self, servo: Servo, target_angle: float, speed: float) -> None:
        super().__init__()
        self._servo_ref = servo
        self._target_angle = float(target_angle)
        self._speed = float(speed)
        self._validate_inputs()

    def _validate_inputs(self) -> None:
        if not (SERVO_MIN_ANGLE <= self._target_angle <= SERVO_MAX_ANGLE):
            raise ValueError(
                f"Target angle must be between {SERVO_MIN_ANGLE} and {SERVO_MAX_ANGLE}, "
                f"got {self._target_angle}"
            )
        if self._speed <= 0:
            raise ValueError(f"Speed must be > 0, got {self._speed}")

    def _generate_signature(self) -> str:
        servo_label = f"port-{getattr(self._servo_ref, 'port', 'na')}"
        return f"EaseServo(servo={servo_label},angle={self._target_angle},speed={self._speed})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._servo_ref.enable()

        start_angle = position_to_angle(self._servo_ref.get_position())
        delta = self._target_angle - start_angle

        if abs(delta) < 0.5:
            self._servo_ref.set_position(angle_to_position(self._target_angle))
            return

        duration = abs(delta) / self._speed

        loop = asyncio.get_running_loop()
        start_time = loop.time()

        while True:
            elapsed = loop.time() - start_time
            t = min(elapsed / duration, 1.0)
            eased = _ease_in_out(t)
            current_angle = start_angle + delta * eased
            self._servo_ref.set_position(angle_to_position(current_angle))

            if t >= 1.0:
                break

            await asyncio.sleep(_EASE_SERVO_TICK)

        self._servo_ref.set_position(angle_to_position(self._target_angle))


@dsl(tags=["servo", "actuator"])
def slow_servo(servo: Servo, angle: float, speed: float = 60.0) -> EaseServo:
    """Move a servo to an angle with ease-in-ease-out at the given speed (degrees/second)."""
    return EaseServo(servo=servo, target_angle=angle, speed=speed)


__all__ = [
    "SetServoPosition",
    "EaseServo",
    "ShakeServo",
    "servo",
    "slow_servo",
    "shake_servo",
]
