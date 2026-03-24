"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: tune_drive.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .tune_drive import TuneDrive


class TuneDriveBuilder(StepBuilder):
    """Builder for TuneDrive. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._distances_cm = None
        self._speeds = None
        self._csv_dir = "/tmp/drive_telemetry"
        self._axis = "forward"
        self._settle_time = 1.5
        self._timeout = 15.0

    def distances_cm(self, value: list[float]):
        self._distances_cm = value
        return self

    def speeds(self, value: list[float]):
        self._speeds = value
        return self

    def csv_dir(self, value: str):
        self._csv_dir = value
        return self

    def axis(self, value: str):
        self._axis = value
        return self

    def settle_time(self, value: float):
        self._settle_time = value
        return self

    def timeout(self, value: float):
        self._timeout = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["distances_cm"] = self._distances_cm
        kwargs["speeds"] = self._speeds
        kwargs["csv_dir"] = self._csv_dir
        kwargs["axis"] = self._axis
        kwargs["settle_time"] = self._settle_time
        kwargs["timeout"] = self._timeout
        return TuneDrive(**kwargs)


@dsl(tags=["motion", "calibration", "drive"])
def tune_drive(
    distances_cm: list[float] = None,
    speeds: list[float] = None,
    csv_dir: str = "/tmp/drive_telemetry",
    axis: str = "forward",
    settle_time: float = 1.5,
    timeout: float = 15.0,
):
    """
    Run test drives at various distances and speeds, saving telemetry to CSV.

    Executes every combination of (distance, speed) as a real ``LinearMotion``
    drive, collecting per-cycle telemetry at 50 Hz. Each run produces a CSV
    file containing columns such as ``time_s``, ``position_m``,
    ``setpoint_position_m``, ``setpoint_velocity_mps``, ``distance_error_m``,
    ``actual_error_m``, ``filtered_velocity_mps``, ``cmd_vx_mps``,
    ``pid_primary_raw``, ``heading_rad``, ``saturated``, and more (see
    ``CSV_HEADER`` in the module source for the full list).

    The CSV files are intended for offline analysis -- plot position vs.
    setpoint to check tracking, examine PID outputs for saturation, compare
    overshoot across speeds, etc. This is a diagnostic/tuning tool used
    during robot setup, not during competition runs.

    The robot must have enough clear space to drive the longest requested
    distance.

    Args:
        distances_cm: List of distances to test, in centimeters. Negative values drive in reverse. Default ``[10, 25, 50, 100]``.
        speeds: List of speed scales to test (0.0--1.0). Each distance is driven at each speed. Default ``[0.3, 0.6, 1.0]``.
        csv_dir: Directory where CSV files are written. Created automatically if it does not exist. Default ``"/tmp/drive_telemetry"``.
        axis: Drive axis to test: ``"forward"`` or ``"lateral"``. Default ``"forward"``.
        settle_time: Seconds to wait between runs for the robot to come to rest. Default 1.5.
        timeout: Maximum seconds per run before the drive is aborted. Default 15.0.

    Returns:
        A TuneDriveBuilder (chainable via ``.distances_cm()``, ``.speeds()``, ``.csv_dir()``, ``.axis()``, ``.settle_time()``, ``.timeout()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import tune_drive

        # Quick test at a single distance and speed
        tune_drive(
            distances_cm=[50],
            speeds=[0.5],
            csv_dir="/tmp/quick_test",
        )

        # Full sweep for forward axis tuning
        tune_drive(
            distances_cm=[10, 25, 50, 100],
            speeds=[0.3, 0.6, 1.0],
        )

        # Lateral axis characterization
        tune_drive(
            distances_cm=[20, 40],
            speeds=[0.3, 0.6],
            axis="lateral",
        )
    """
    b = TuneDriveBuilder()
    b._distances_cm = distances_cm
    b._speeds = speeds
    b._csv_dir = csv_dir
    b._axis = axis
    b._settle_time = settle_time
    b._timeout = timeout
    return b


__all__ = ["TuneDriveBuilder", "tune_drive"]
