from __future__ import annotations

import asyncio
from typing import Optional

from libstp.hal import Servo
from libstp.robot.api import GenericRobot
from libstp.step import Step
from libstp.step.annotation import dsl, dsl_step

from .resolver import resolve_servo
from .utility import estimate_servo_move_time


def _unwrap_servo(servo_or_preset):
    """Extract the raw Servo from a ServoPreset, or return as-is if already a Servo."""
    from .preset import ServoPreset
    if isinstance(servo_or_preset, ServoPreset):
        return servo_or_preset.device
    return servo_or_preset


@dsl(hidden=True)
class SetServoPosition(Step):
    """Set a servo to a target angle and optionally wait for the move to finish."""

    def __init__(
        self, servo: Servo | ServoPreset, target_angle: float, duration: Optional[float] = None
    ) -> None:
        super().__init__()
        self._servo_ref = _unwrap_servo(servo)
        self._target_angle = float(target_angle)
        self._duration = float(duration) if duration is not None else None
        if self._duration is not None and self._duration < 0:
            raise ValueError("Duration must be >= 0")

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"servo:{self._servo_ref.port}"})

    def _generate_signature(self) -> str:
        servo_label = f"port-{getattr(self._servo_ref, 'port', 'na')}"
        return f"SetServoPosition(servo={servo_label},angle={self._target_angle},duration={self._duration})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._servo_ref.enable()

        duration = self._duration
        if duration is None:
            current_angle = self._servo_ref.get_position()
            duration = estimate_servo_move_time(current_angle, self._target_angle)

        self._servo_ref.set_position(self._target_angle)
        if duration and duration > 0:
            await asyncio.sleep(duration)

@dsl(tags=["servo", "actuator"])
def servo(servo: Servo, angle: float) -> SetServoPosition:
    """Set a servo to a specific angle and wait for the move to finish.

    Commands the servo to the requested angle, then sleeps for an
    estimated duration based on the angular distance the servo needs to
    travel and its approximate speed. This ensures subsequent steps do
    not begin until the servo has physically reached its target.

    Args:
        servo: The servo to control, obtained from the robot hardware map
            (e.g. ``robot.servo(0)``).
        angle: Target angle in degrees.

    Returns:
        A ``SetServoPosition`` step with automatically estimated wait
        duration.

    Example::

        from libstp.step.servo import servo

        # Open a claw by moving servo 0 to 90 degrees
        servo(robot.servo(0), 90.0)

        # Close the claw, then raise the arm
        sequence(
            servo(robot.servo(0), 10.0),
            motor_move_to(robot.motor(2), position=400),
        )
    """
    return SetServoPosition(servo=servo, target_angle=angle, duration=None)



@dsl_step(tags=["servo", "actuator"])
class ShakeServo(Step):
    """Oscillate a servo back and forth between two angles for a set time.

    Rapidly alternates the servo between ``angle_a`` and ``angle_b`` for
    the given duration. The dwell time at each angle is automatically
    estimated from the angular distance so the servo has time to
    physically reach each endpoint before reversing. Useful for shaking
    objects loose or signalling the operator.

    Args:
        servo: The servo to control, obtained from the robot hardware map
            (e.g. ``robot.servo(1)``).
        duration: Total oscillation time in seconds. Must be >= 0.
        angle_a: First oscillation endpoint in degrees.
        angle_b: Second oscillation endpoint in degrees.

    Example::

        from libstp.step.servo import shake_servo

        # Shake a sorting tray for 3 seconds between 60 and 120 degrees
        shake_servo(robot.servo(1), duration=3.0, angle_a=60.0, angle_b=120.0)
    """

    def __init__(
        self, servo: Servo | ServoPreset, duration: float, angle_a: float, angle_b: float
    ) -> None:
        super().__init__()
        self._servo_ref = _unwrap_servo(servo)
        self._duration = float(duration)
        self._angle_a = float(angle_a)
        self._angle_b = float(angle_b)
        if self._duration < 0:
            raise ValueError("Duration must be >= 0")

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"servo:{self._servo_ref.port}"})

    def _generate_signature(self) -> str:
        servo_label = f"port-{getattr(self._servo_ref, 'port', 'na')}"
        return (
            f"ShakeServo(servo={servo_label},duration={self._duration},"
            f"a={self._angle_a},b={self._angle_b})"
        )

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._servo_ref.enable()

        if self._angle_a == self._angle_b or self._duration == 0:
            self._servo_ref.set_position(self._angle_a)
            if self._duration > 0:
                await asyncio.sleep(self._duration)
            return

        move_time = estimate_servo_move_time(self._angle_a, self._angle_b)
        # Guard against a zero-time oscillation; the loop clock resolution makes 10ms safe.
        move_time = max(move_time, 0.01)

        loop = asyncio.get_running_loop()
        end_time = loop.time() + self._duration

        while loop.time() < end_time:
            self._servo_ref.set_position(self._angle_a)
            await asyncio.sleep(move_time)
            if loop.time() >= end_time:
                break
            self._servo_ref.set_position(self._angle_b)
            await asyncio.sleep(move_time)


def _ease_in_out(t: float) -> float:
    """Smoothstep ease-in-ease-out: 3t^2 - 2t^3."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


_EASE_SERVO_TICK = 1 / 10


@dsl_step(tags=["servo", "actuator"])
class SlowServo(Step):
    """Move a servo to an angle with smooth ease-in/ease-out motion.

    Instead of commanding the servo to jump straight to the target (as
    ``servo()`` does), this step interpolates through intermediate
    positions using a smoothstep curve (3t^2 - 2t^3). The result is a
    gentle acceleration and deceleration that avoids mechanical shock and
    reduces jerk on the mechanism.

    The total move duration is derived from the angular distance divided
    by ``speed``. Intermediate positions are updated at ~10 Hz.

    Args:
        servo: The servo to control, obtained from the robot hardware map
            (e.g. ``robot.servo(0)``).
        angle: Target angle in degrees.
        speed: Movement speed in degrees per second. Must be positive.
            Defaults to 60.0 deg/s.

    Example::

        from libstp.step.servo import slow_servo

        # Gently lower the arm servo to 20 degrees at 45 deg/s
        slow_servo(robot.servo(0), angle=20.0, speed=45.0)

        # Use default speed for a smooth open
        slow_servo(robot.servo(0), angle=150.0)
    """

    def __init__(self, servo: Servo | ServoPreset, angle: float, speed: float = 60.0) -> None:
        super().__init__()
        self._servo_ref = _unwrap_servo(servo)
        self._target_angle = float(angle)
        self._speed = float(speed)
        if self._speed <= 0:
            raise ValueError(f"Speed must be > 0, got {self._speed}")

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"servo:{self._servo_ref.port}"})

    def _generate_signature(self) -> str:
        servo_label = f"port-{getattr(self._servo_ref, 'port', 'na')}"
        return f"SlowServo(servo={servo_label},angle={self._target_angle},speed={self._speed})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._servo_ref.enable()

        start_angle = self._servo_ref.get_position()
        delta = self._target_angle - start_angle

        if abs(delta) < 0.5:
            self._servo_ref.set_position(self._target_angle)
            return

        duration = abs(delta) / self._speed

        loop = asyncio.get_running_loop()
        start_time = loop.time()

        while True:
            elapsed = loop.time() - start_time
            t = min(elapsed / duration, 1.0)
            eased = _ease_in_out(t)
            current_angle = start_angle + delta * eased
            self._servo_ref.set_position(current_angle)

            if t >= 1.0:
                break

            await asyncio.sleep(_EASE_SERVO_TICK)

        self._servo_ref.set_position(self._target_angle)


@dsl_step(tags=["servo", "actuator"])
class FullyDisableServos(Step):
    """Fully disable all servo outputs, removing all power from the servo pins.

    Commands the firmware to enter the fully-disabled servo mode for every
    servo port. In this mode, no PWM signal is sent and the servos can be
    moved freely by hand. This is useful for saving power or when the
    servos should not hold position (e.g. at the end of a run).

    Servos will automatically re-enable when a new position command is
    sent (e.g. via ``servo()`` or ``slow_servo()``).

    Example::

        from libstp.step.servo import fully_disable_servos

        # Release all servos at the end of a mission
        fully_disable_servos()
    """

    def required_resources(self) -> frozenset[str]:
        return frozenset({"servo:*"})

    def _generate_signature(self) -> str:
        return "FullyDisableServos()"

    async def _execute_step(self, robot: GenericRobot) -> None:
        Servo.fully_disable_all()


__all__ = [
    "SetServoPosition",
    "SlowServo",
    "ShakeServo",
    "FullyDisableServos",
    "servo",
]
