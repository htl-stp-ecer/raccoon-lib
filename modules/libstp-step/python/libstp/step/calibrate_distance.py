from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING

from libstp.screen.api import RenderScreen, DistanceCalibrationResult
from libstp.step import Step

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dataclass
class DistanceScaler:
    """Runtime distance scaling factor storage."""
    scale_factor: float = 1.0

    def apply(self, distance_cm: float) -> float:
        """Apply scaling to a distance value."""
        return distance_cm * self.scale_factor


# Global runtime scaler
_distance_scaler = DistanceScaler()


def get_distance_scaler() -> DistanceScaler:
    """Get the global distance scaler instance."""
    return _distance_scaler


def reset_distance_scaler() -> None:
    """Reset the distance scaler to default (no scaling)."""
    _distance_scaler.scale_factor = 1.0


class CalibrateDistance(Step):
    """Step for calibrating robot distance estimation."""

    def __init__(self, calibration_distance_cm: float = 30.0) -> None:
        """
        Initialize the distance calibration step.

        Args:
            calibration_distance_cm: Distance to drive for calibration (default 30cm)
        """
        super().__init__()
        self.calibration_distance_cm = calibration_distance_cm
        self.result: Optional[DistanceCalibrationResult] = None

    def _generate_signature(self) -> str:
        return f"CalibrateDistance(distance_cm={self.calibration_distance_cm})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """
        Execute the distance calibration.

        The calibration flow:
        1. Show prepare screen - user places robot
        2. User presses button - robot drives requested distance
        3. Show measure screen - user enters measured distance
        4. Show confirm screen with scale factor
        5. Apply scale factor to global scaler

        Args:
            robot: The robot instance for driving
        """
        # Import here to avoid circular dependency
        from libstp.step.drive import drive_forward

        screen = RenderScreen([])
        screen.change_screen("calibrate_sensors")

        # Phase 1: Prepare - user places robot
        self.debug("Distance calibration: prepare phase")
        screen.send_state({
            "type": "distanceCalibration",
            "state": "prepare",
            "requested_distance_cm": self.calibration_distance_cm
        })
        await screen._RenderScreen__wait_for_button()

        # Phase 2: Driving - robot moves
        self.debug(f"Distance calibration: driving {self.calibration_distance_cm}cm")
        screen.send_state({
            "type": "distanceCalibration",
            "state": "driving",
            "requested_distance_cm": self.calibration_distance_cm
        })

        # Execute the drive
        drive_step = drive_forward(self.calibration_distance_cm, speed=0.5)
        await drive_step.run_step(robot)

        # Phase 3: Measure - user enters measured distance
        self.debug("Distance calibration: measure phase")
        screen.send_state({
            "type": "distanceCalibration",
            "state": "measure",
            "requested_distance_cm": self.calibration_distance_cm
        })

        # Wait for user to enter measured distance (5 minute timeout)
        response = await screen._RenderScreen__wait_for_lcm_response(timeout=300)

        if response.value == "retry":
            self.warn("Distance calibration timeout during measure phase")
            return

        if response.value == "cancelled":
            self.debug("Distance calibration cancelled")
            return

        # Parse measured distance from response reason
        # Expected format: "measured_distance=28.5"
        measured_cm = self._parse_measured_distance(response.reason)
        if measured_cm is None:
            self.warn(f"Could not parse measured distance from: {response.reason}")
            return

        # Calculate scale factor
        scale_factor = measured_cm / self.calibration_distance_cm
        self.debug(f"Calculated scale factor: {scale_factor}")

        # Phase 4: Confirm - show results
        screen.send_state({
            "type": "distanceCalibration",
            "state": "confirm",
            "requested_distance_cm": self.calibration_distance_cm,
            "measured_distance_cm": measured_cm,
            "scale_factor": scale_factor
        })

        confirm_response = await screen._RenderScreen__wait_for_lcm_response(timeout=120)

        if confirm_response.value == "confirmed":
            # Apply to global scaler
            _distance_scaler.scale_factor = scale_factor
            self.result = DistanceCalibrationResult(
                requested_distance_cm=self.calibration_distance_cm,
                measured_distance_cm=measured_cm,
                scale_factor=scale_factor
            )
            self.debug(f"Distance calibration applied: scale_factor={scale_factor}")
        elif confirm_response.value == "retry":
            # User wants to retry
            self.debug("User requested retry")
            await self._execute_step(robot)
        else:
            self.debug("Distance calibration cancelled at confirm phase")

    def _parse_measured_distance(self, reason: str) -> Optional[float]:
        """Parse measured distance from LCM response reason field."""
        try:
            # Expected format: "measured_distance=28.5"
            if "measured_distance=" in reason:
                value_str = reason.split("measured_distance=")[1].split()[0]
                return float(value_str)
            # Also try just a raw number
            return float(reason)
        except (ValueError, IndexError):
            return None


def calibrate_distance(distance_cm: float = 30.0) -> CalibrateDistance:
    """
    Create a distance calibration step.

    The robot will drive the specified distance, then prompt the user
    to measure the actual distance traveled. A scale factor is computed
    and applied to the global distance scaler.

    Args:
        distance_cm: Distance to drive for calibration (default 30cm)

    Returns:
        CalibrateDistance step instance
    """
    return CalibrateDistance(distance_cm)
