from __future__ import annotations

import asyncio
import enum
import math
from collections.abc import Callable
from typing import TYPE_CHECKING

from raccoon.hal import Servo
from raccoon.step import Step
from raccoon.step.annotation import dsl, dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

from .utility import estimate_servo_move_time

# ---------------------------------------------------------------------------
# Easing / interpolation helpers
# ---------------------------------------------------------------------------

EasingFunc = Callable[[float], float]
"""Signature for an easing function: maps t in [0, 1] → eased value in [0, 1]."""


def _clamp01(t: float) -> float:
    return max(0.0, min(1.0, t))


def _ease_linear(t: float) -> float:
    """No easing — constant speed."""
    return _clamp01(t)


def _ease_in(t: float) -> float:
    """Quadratic ease-in — slow start, fast end."""
    t = _clamp01(t)
    return t * t


def _ease_out(t: float) -> float:
    """Quadratic ease-out — fast start, slow end."""
    t = _clamp01(t)
    return t * (2.0 - t)


def _ease_in_out_smoothstep(t: float) -> float:
    """Smoothstep ease-in-ease-out: 3t^2 - 2t^3."""
    t = _clamp01(t)
    return t * t * (3.0 - 2.0 * t)


def _ease_in_out_cosine(t: float) -> float:
    """Cosine-based ease-in-ease-out."""
    t = _clamp01(t)
    return (1.0 - math.cos(t * math.pi)) * 0.5


class Easing(enum.Enum):
    """Built-in easing functions for :class:`SlowServo`.

    Each member wraps a callable ``(t: float) -> float`` that maps
    normalised time *t* ∈ [0, 1] to an eased progress value in [0, 1].
    """

    LINEAR = _ease_linear
    """Constant speed — no acceleration or deceleration."""

    EASE_IN = _ease_in
    """Quadratic ease-in — slow start, fast end."""

    EASE_OUT = _ease_out
    """Quadratic ease-out — fast start, slow end."""

    EASE_IN_OUT = _ease_in_out_smoothstep
    """Smoothstep (3t² − 2t³) — gentle acceleration *and* deceleration (default)."""

    EASE_IN_OUT_COSINE = _ease_in_out_cosine
    """Cosine-based ease-in-out — similar feel, slightly different curve shape."""

    def __call__(self, t: float) -> float:
        return self.value(t)


def _unwrap_servo(servo_or_preset):
    """Extract the raw Servo from a ServoPreset, or return as-is if already a Servo.

    Raises TypeError if the argument is neither a Servo nor a ServoPreset.
    """
    from .preset import ServoPreset

    if isinstance(servo_or_preset, ServoPreset):
        return servo_or_preset.device
    if not hasattr(servo_or_preset, "set_position") or not hasattr(servo_or_preset, "port"):
        msg = f"Expected a Servo or ServoPreset, got {type(servo_or_preset).__name__}"
        raise TypeError(msg)
    return servo_or_preset


@dsl(hidden=True)
class SetServoPosition(Step):
    """Set a servo to a target angle and optionally wait for the move to finish."""

    def __init__(
        self, servo: Servo | ServoPreset, target_angle: float, duration: float | None = None
    ) -> None:
        super().__init__()
        self._servo_ref = _unwrap_servo(servo)
        self._target_angle = float(target_angle)
        self._duration = float(duration) if duration is not None else None
        if self._duration is not None and self._duration < 0:
            msg = "Duration must be >= 0"
            raise ValueError(msg)

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"servo:{self._servo_ref.port}"})

    def _generate_signature(self) -> str:
        servo_label = f"port-{getattr(self._servo_ref, 'port', 'na')}"
        return f"SetServoPosition(servo={servo_label},angle={self._target_angle},duration={self._duration})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
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

        from raccoon.step.servo import servo

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

        from raccoon.step.servo import shake_servo

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
            msg = "Duration must be >= 0"
            raise ValueError(msg)

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"servo:{self._servo_ref.port}"})

    def _generate_signature(self) -> str:
        servo_label = f"port-{getattr(self._servo_ref, 'port', 'na')}"
        return (
            f"ShakeServo(servo={servo_label},duration={self._duration},"
            f"a={self._angle_a},b={self._angle_b})"
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
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


_EASE_SERVO_TICK = 1 / 10

_EASING_INT: dict[Easing, int] = {
    Easing.LINEAR: 0,
    Easing.EASE_IN: 1,
    Easing.EASE_OUT: 2,
    Easing.EASE_IN_OUT: 3,
    Easing.EASE_IN_OUT_COSINE: 4,
}


@dsl_step(tags=["servo", "actuator"])
class SlowServo(Step):
    """Move a servo to an angle with smooth interpolated motion.

    Instead of commanding the servo to jump straight to the target (as
    ``servo()`` does), this step interpolates through intermediate
    positions using an easing curve. The default is smoothstep
    ease-in-ease-out (3t² − 2t³), which gives gentle acceleration and
    deceleration. Other curves can be selected via the ``easing``
    parameter.

    The total move duration is derived from the angular distance divided
    by ``speed``. Intermediate positions are updated at ~10 Hz.

    Args:
        servo: The servo to control, obtained from the robot hardware map
            (e.g. ``robot.servo(0)``).
        angle: Target angle in degrees.
        speed: Movement speed in degrees per second. Must be positive.
            Defaults to 60.0 deg/s.
        easing: Interpolation curve. Pass an :class:`Easing` member or
            any callable ``(t: float) -> float`` mapping [0, 1] → [0, 1].
            Defaults to ``Easing.EASE_IN_OUT``.

    Example::

        from raccoon.step.servo import slow_servo, Easing

        # Gently lower the arm servo to 20 degrees at 45 deg/s
        slow_servo(robot.servo(0), angle=20.0, speed=45.0)

        # Linear (constant-speed) motion
        slow_servo(robot.servo(0), angle=150.0, easing=Easing.LINEAR)

        # Ease-out only (fast start, slow stop)
        slow_servo(robot.servo(0), angle=0.0, easing=Easing.EASE_OUT)
    """

    def __init__(
        self,
        servo: Servo | ServoPreset,
        angle: float,
        speed: float = 60.0,
        easing: Easing | EasingFunc = Easing.EASE_IN_OUT,
    ) -> None:
        super().__init__()
        self._servo_ref = _unwrap_servo(servo)
        if not isinstance(angle, int | float):
            msg = f"angle must be a number, got {type(angle).__name__}"
            raise TypeError(msg)
        self._target_angle = float(angle)
        self._speed = float(speed)
        if self._speed <= 0:
            msg = f"Speed must be > 0, got {self._speed}"
            raise ValueError(msg)
        if not callable(easing):
            msg = f"easing must be an Easing member or callable, got {type(easing).__name__}"
            raise TypeError(msg)
        self._easing: EasingFunc = easing if callable(easing) else easing.value

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"servo:{self._servo_ref.port}"})

    def _generate_signature(self) -> str:
        servo_label = f"port-{getattr(self._servo_ref, 'port', 'na')}"
        easing_name = getattr(self._easing, "__name__", None) or getattr(self._easing, "name", "?")
        return f"SlowServo(servo={servo_label},angle={self._target_angle},speed={self._speed},easing={easing_name})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        self._servo_ref.enable()

        start_angle = self._servo_ref.get_position()
        delta = self._target_angle - start_angle

        if abs(delta) < 0.5:
            self._servo_ref.set_position(self._target_angle)
            return

        duration = abs(delta) / self._speed

        easing_int = _EASING_INT.get(self._easing)
        if easing_int is not None:
            self._servo_ref.set_smooth_position(self._target_angle, self._speed, easing_int)
            await asyncio.sleep(duration)
        else:
            loop = asyncio.get_running_loop()
            start_time = loop.time()

            while True:
                elapsed = loop.time() - start_time
                t = min(elapsed / duration, 1.0)
                eased = self._easing(t)
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

        from raccoon.step.servo import fully_disable_servos

        # Release all servos at the end of a mission
        fully_disable_servos()
    """

    def required_resources(self) -> frozenset[str]:
        return frozenset({"servo:*"})

    def _generate_signature(self) -> str:
        return "FullyDisableServos()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        Servo.fully_disable_all()


__all__ = [
    "Easing",
    "SetServoPosition",
    "SlowServo",
    "ShakeServo",
    "FullyDisableServos",
    "servo",
]
