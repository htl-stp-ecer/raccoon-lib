"""
Deadzone calibration step using the UI library.

BEMF is unreliable at low RPM, so we use human observation via the UI
to find the exact motor percentage where the wheel starts turning.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING

from raccoon.step.annotation import dsl_step
from raccoon.ui.screens.basic import MessageScreen
from raccoon.ui.step import UIStep

from .screens import (
    DeadzoneIntroScreen,
    DeadzoneResultsScreen,
    DeadzoneSummaryScreen,
    DeadzoneTestingScreen,
)

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dataclass
class DeadzoneCalibrationResult:
    """Result of deadzone calibration for a single motor."""

    motor_port: int
    motor_name: str
    start_percent_forward: int
    start_percent_reverse: int
    release_percent: int

    @property
    def start_percent(self) -> int:
        """The higher of forward/reverse percentages."""
        return max(self.start_percent_forward, self.start_percent_reverse)


@dsl_step(tags=["calibration", "motor", "deadzone"])
class CalibrateDeadzone(UIStep):
    """Calibrate motor deadzone via UI-based human observation.

    The deadzone is the minimum power percentage required to overcome
    static friction and start the motor turning. BEMF readings are
    unreliable at low RPM, so this step ramps motor power from
    ``start_percent`` upward and asks the operator to confirm when
    the wheel starts spinning. The result is stored as the motor's
    ``ff.kS`` (static friction) feedforward value.

    Args:
        motor_ports: List of motor ports to calibrate. ``None``
            calibrates all drive motors.
        start_percent: Starting power percentage to test.
        max_percent: Maximum power percentage before giving up.
        settle_time: Seconds to wait after setting power before asking
            the operator.

    Example::

        from raccoon.step.calibration import calibrate_deadzone

        # Calibrate all motors with defaults
        calibrate_deadzone()

        # Calibrate only motors 0 and 1
        calibrate_deadzone(motor_ports=[0, 1])
    """

    def __init__(
        self,
        motor_ports: list[int] | None = None,
        start_percent: int = 1,
        max_percent: int = 30,
        settle_time: float = 0.3,
    ):
        super().__init__()
        self.motor_ports = motor_ports
        self.start_percent = start_percent
        self.max_percent = max_percent
        self.settle_time = settle_time
        self.results: list[DeadzoneCalibrationResult] = []

    def _generate_signature(self) -> str:
        ports = self.motor_ports or "all"
        return f"CalibrateDeadzone(ports={ports}, range={self.start_percent}-{self.max_percent}%)"

    async def _find_deadzone_for_direction(
        self,
        robot: "GenericRobot",
        motor,
        motor_name: str,
        motor_port: int,
        direction: int,
        direction_name: str,
    ) -> int:
        """
        Find the minimum power that makes the motor turn in one direction.

        Returns the first percentage where user indicates movement.
        """
        # Show intro screen
        await self.show(
            DeadzoneIntroScreen(
                motor_name=motor_name,
                motor_port=motor_port,
                direction=direction_name,
                start_percent=self.start_percent,
                max_percent=self.max_percent,
            )
        )

        # Test each power level
        for percent in range(self.start_percent, self.max_percent + 1):
            # Apply speed
            motor.set_speed(percent * direction)
            await asyncio.sleep(self.settle_time)

            # Show testing screen and wait for user response
            result = await self.show(
                DeadzoneTestingScreen(
                    motor_name=motor_name,
                    motor_port=motor_port,
                    current_percent=percent,
                    direction=direction_name,
                    max_percent=self.max_percent,
                )
            )

            if result.is_turning:
                motor.brake()
                return percent

        # Hit max without movement
        motor.brake()
        self.warn(f"Motor {motor_name} didn't turn even at {self.max_percent}%")
        return self.max_percent

    async def _calibrate_motor(
        self,
        robot: "GenericRobot",
        motor,
        motor_name: str,
        motor_port: int,
    ) -> DeadzoneCalibrationResult | None:
        """Calibrate a single motor's deadzone."""
        try:
            # Test forward direction
            start_fwd = await self._find_deadzone_for_direction(
                robot, motor, motor_name, motor_port, direction=1, direction_name="FORWARD"
            )
            self.debug(f"{motor_name} forward start: {start_fwd}%")

            await asyncio.sleep(0.5)  # Brief pause between directions

            # Test reverse direction
            start_rev = await self._find_deadzone_for_direction(
                robot, motor, motor_name, motor_port, direction=-1, direction_name="REVERSE"
            )
            self.debug(f"{motor_name} reverse start: {start_rev}%")

        finally:
            motor.brake()

        # Calculate release percent (~70% of average, for hysteresis)
        avg_start = (start_fwd + start_rev) / 2
        release = max(1, int(avg_start * 0.7))

        result = DeadzoneCalibrationResult(
            motor_port=motor_port,
            motor_name=motor_name,
            start_percent_forward=start_fwd,
            start_percent_reverse=start_rev,
            release_percent=release,
        )

        # Show results and ask for confirmation
        confirm = await self.show(
            DeadzoneResultsScreen(
                motor_name=motor_name,
                motor_port=motor_port,
                forward_percent=start_fwd,
                reverse_percent=start_rev,
                release_percent=release,
            )
        )

        if confirm.confirmed:
            return result
        # User wants to retry this motor
        return await self._calibrate_motor(robot, motor, motor_name, motor_port)

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """
        Execute the deadzone calibration.

        Flow for each motor:
        1. Show intro screen with motor info
        2. Incrementally test power levels (FORWARD)
        3. User indicates when wheel starts turning
        4. Repeat for REVERSE direction
        5. Show results, allow retry
        6. After all motors, show summary and apply
        """
        from raccoon.no_calibrate import is_no_calibrate

        if is_no_calibrate():
            self.info("--no-calibrate: skipping deadzone calibration, using stored values")
            return

        from raccoon.hal import Motor

        # Find motors to calibrate
        motors_to_calibrate: list[tuple] = []

        if self.motor_ports:
            # Calibrate specific ports
            for port in self.motor_ports:
                motor_def = robot.defs.get_motor_by_port(port)
                if motor_def:
                    motors_to_calibrate.append((motor_def.name, port, motor_def.inverted))
                else:
                    self.warn(f"No motor definition found for port {port}")
        else:
            # Calibrate all motors in robot definitions
            for motor_def in robot.defs.motors:
                motors_to_calibrate.append((motor_def.name, motor_def.port, motor_def.inverted))

        if not motors_to_calibrate:
            await self.show(
                MessageScreen(
                    "No Motors Found",
                    "No motors found to calibrate. Define motors in your project first.",
                    icon_name="error",
                    icon_color="red",
                )
            )
            return

        self.debug(f"Calibrating {len(motors_to_calibrate)} motor(s)")

        # Calibrate each motor
        self.results = []
        for motor_name, port, inverted in motors_to_calibrate:
            motor = Motor(port=port, inverted=inverted)
            try:
                result = await self._calibrate_motor(robot, motor, motor_name, port)
                if result:
                    self.results.append(result)
            finally:
                motor.brake()

        if not self.results:
            self.warn("No calibration results collected")
            return

        # Show summary and apply
        summary_data = [
            {
                "motor_name": r.motor_name,
                "port": r.motor_port,
                "forward": r.start_percent_forward,
                "reverse": r.start_percent_reverse,
                "kS": r.start_percent / 100.0,  # Normalized for ff.kS
            }
            for r in self.results
        ]

        apply = await self.show(DeadzoneSummaryScreen(summary_data))

        if apply:
            # Apply calibration to each motor
            for result in self.results:
                motor_def = robot.defs.get_motor_by_port(result.motor_port)
                # Update ff.kS (static friction coefficient): the deadzone
                # start_percent (0-100) maps onto kS as a 0-1 normalised
                # gain. Skip motors that lack the nested calibration shape.
                if (
                    motor_def
                    and getattr(motor_def, "calibration", None)
                    and getattr(motor_def.calibration, "ff", None)
                ):
                    cal = motor_def.calibration
                    cal.ff.kS = result.start_percent / 100.0
                    self.debug(f"Applied ff.kS={cal.ff.kS:.4f} to {result.motor_name}")

            self.info(f"ff.kS calibration applied to {len(self.results)} motor(s)")

            await self.show(
                MessageScreen(
                    "Calibration Applied",
                    f"ff.kS (static friction) calibration applied to {len(self.results)} motor(s).",
                    icon_name="check",
                    icon_color="green",
                )
            )
        else:
            self.info("ff.kS calibration cancelled by user")
