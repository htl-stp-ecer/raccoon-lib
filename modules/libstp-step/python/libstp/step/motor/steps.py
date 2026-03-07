from __future__ import annotations

import asyncio
import math
from enum import Enum
from typing import TYPE_CHECKING, Optional

from libstp.hal import IMotor
from libstp.step import Step
from libstp.step.annotation import dsl

# BEMF sampling rate in Hz (5ms interval on firmware) — must match motor_adapter.cpp
_BEMF_SAMPLE_RATE = 200.0

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class StopMode(Enum):
    """How a motor should be stopped."""

    OFF = "off"
    PASSIVE_BRAKE = "passive_brake"
    ACTIVE_BRAKE = "active_brake"


# ---------------------------------------------------------------------------
# Step classes (hidden from DSL discovery; use factory functions below)
# ---------------------------------------------------------------------------


@dsl(hidden=True)
class SetMotorPower(Step):
    """Set a motor to open-loop percent power (-100..100)."""

    def __init__(self, motor: IMotor, percent: int) -> None:
        super().__init__()
        self._motor = motor
        self._percent = int(percent)
        if not (-100 <= self._percent <= 100):
            raise ValueError(f"percent must be -100..100, got {self._percent}")

    def _generate_signature(self) -> str:
        return f"SetMotorPower(port={self._motor.port},percent={self._percent})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._motor.set_speed(self._percent)


@dsl(hidden=True)
class SetMotorVelocity(Step):
    """Set a motor to closed-loop velocity (firmware BEMF units)."""

    def __init__(self, motor: IMotor, velocity: int) -> None:
        super().__init__()
        self._motor = motor
        self._velocity = int(velocity)

    def _generate_signature(self) -> str:
        return f"SetMotorVelocity(port={self._motor.port},velocity={self._velocity})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._motor.set_velocity(self._velocity)


@dsl(hidden=True)
class SetMotorDps(Step):
    """Set a motor to closed-loop velocity specified in degrees per second."""

    def __init__(self, motor: IMotor, dps: float) -> None:
        super().__init__()
        self._motor = motor
        self._dps = float(dps)

    def _generate_signature(self) -> str:
        return f"SetMotorDps(port={self._motor.port},dps={self._dps})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        ticks_to_rad = self._motor.get_calibration().ticks_to_rad
        self.info(f"Motor {self._motor.port} dps: {self._dps} -> {ticks_to_rad} rad/s")
        rad_per_sec = self._dps * math.pi / 180.0
        self.info(f"Motor {self._motor.port} rad/s: {rad_per_sec}")
        bemf_target = int(round(rad_per_sec / (ticks_to_rad * _BEMF_SAMPLE_RATE)))
        self.info(f"Motor {self._motor.port} bemf: {bemf_target}")
        self._motor.set_velocity(bemf_target)


@dsl(hidden=True)
class MoveMotorTo(Step):
    """Move a motor to an absolute encoder position, waiting for completion."""

    def __init__(
        self,
        motor: IMotor,
        position: int,
        velocity: int = 1000,
        timeout: Optional[float] = None,
    ) -> None:
        super().__init__()
        self._motor = motor
        self._position = int(position)
        self._velocity = int(velocity)
        self._timeout = timeout
        if self._velocity <= 0:
            raise ValueError(f"velocity must be > 0, got {self._velocity}")

    def _generate_signature(self) -> str:
        return (
            f"MoveMotorTo(port={self._motor.port},"
            f"pos={self._position},vel={self._velocity})"
        )

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._motor.move_to_position(self._velocity, self._position)
        loop = asyncio.get_running_loop()
        start = loop.time()
        while not self._motor.is_done():
            if self._timeout is not None and (loop.time() - start) >= self._timeout:
                self.warning(
                    f"MoveMotorTo timed out after {self._timeout:.1f}s "
                    f"(target={self._position}, current={self._motor.get_position()})"
                )
                break
            await asyncio.sleep(0.01)


@dsl(hidden=True)
class MoveMotorRelative(Step):
    """Move a motor by a relative encoder delta, waiting for completion."""

    def __init__(
        self,
        motor: IMotor,
        delta: int,
        velocity: int = 1000,
        timeout: Optional[float] = None,
    ) -> None:
        super().__init__()
        self._motor = motor
        self._delta = int(delta)
        self._velocity = int(velocity)
        self._timeout = timeout
        if self._velocity <= 0:
            raise ValueError(f"velocity must be > 0, got {self._velocity}")

    def _generate_signature(self) -> str:
        return (
            f"MoveMotorRelative(port={self._motor.port},"
            f"delta={self._delta},vel={self._velocity})"
        )

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._motor.move_relative(self._velocity, self._delta)
        loop = asyncio.get_running_loop()
        start = loop.time()
        while not self._motor.is_done():
            if self._timeout is not None and (loop.time() - start) >= self._timeout:
                self.warning(
                    f"MoveMotorRelative timed out after {self._timeout:.1f}s "
                    f"(delta={self._delta}, current={self._motor.get_position()})"
                )
                break
            await asyncio.sleep(0.01)


@dsl(hidden=True)
class StopMotor(Step):
    """Stop a motor using the specified mode."""

    def __init__(
        self, motor: IMotor, mode: StopMode = StopMode.ACTIVE_BRAKE
    ) -> None:
        super().__init__()
        self._motor = motor
        self._mode = mode

    def _generate_signature(self) -> str:
        return f"StopMotor(port={self._motor.port},mode={self._mode.value})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        if self._mode == StopMode.OFF:
            self._motor.off()
        elif self._mode == StopMode.PASSIVE_BRAKE:
            self._motor.set_speed(0)
        elif self._mode == StopMode.ACTIVE_BRAKE:
            self._motor.brake()


# ---------------------------------------------------------------------------
# DSL factory functions
# ---------------------------------------------------------------------------


@dsl(tags=["motor", "actuator"])
def motor_power(motor: IMotor, percent: int) -> SetMotorPower:
    """Set a motor to open-loop percent power (-100..100)."""
    return SetMotorPower(motor=motor, percent=percent)


@dsl(tags=["motor", "actuator"])
def motor_velocity(motor: IMotor, velocity: int) -> SetMotorVelocity:
    """Set a motor to closed-loop velocity (firmware BEMF units)."""
    return SetMotorVelocity(motor=motor, velocity=velocity)


@dsl(tags=["motor", "actuator"])
def motor_move_to(
    motor: IMotor,
    position: int,
    velocity: int = 1000,
    timeout: float | None = None,
) -> MoveMotorTo:
    """Move a motor to an absolute encoder position and wait for completion."""
    return MoveMotorTo(motor=motor, position=position, velocity=velocity, timeout=timeout)


@dsl(tags=["motor", "actuator"])
def motor_move_relative(
    motor: IMotor,
    delta: int,
    velocity: int = 1000,
    timeout: float | None = None,
) -> MoveMotorRelative:
    """Move a motor by a relative encoder delta and wait for completion."""
    return MoveMotorRelative(motor=motor, delta=delta, velocity=velocity, timeout=timeout)


@dsl(tags=["motor", "actuator"])
def motor_dps(motor: IMotor, dps: float) -> SetMotorDps:
    """Set a motor to closed-loop velocity in degrees per second."""
    return SetMotorDps(motor=motor, dps=dps)


@dsl(tags=["motor", "actuator"])
def motor_off(motor: IMotor) -> StopMotor:
    """Turn a motor off (coast / free-spin)."""
    return StopMotor(motor=motor, mode=StopMode.OFF)


@dsl(tags=["motor", "actuator"])
def motor_passive_brake(motor: IMotor) -> StopMotor:
    """Passively brake a motor (zero power, no active hold)."""
    return StopMotor(motor=motor, mode=StopMode.PASSIVE_BRAKE)


@dsl(tags=["motor", "actuator"])
def motor_brake(motor: IMotor) -> StopMotor:
    """Actively brake a motor (engage stop latch, holds position)."""
    return StopMotor(motor=motor, mode=StopMode.ACTIVE_BRAKE)


__all__ = [
    "StopMode",
    "SetMotorPower",
    "SetMotorVelocity",
    "SetMotorDps",
    "MoveMotorTo",
    "MoveMotorRelative",
    "StopMotor",
    "motor_power",
    "motor_velocity",
    "motor_dps",
    "motor_move_to",
    "motor_move_relative",
    "motor_off",
    "motor_passive_brake",
    "motor_brake",
]
