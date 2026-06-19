"""Runtime "has distance calibration run yet?" flag + stored-IR application.

These helpers used to live in ``calibrate_distance``. That step (which rewrote
the odometry ``ticks_to_rad`` baseline) is gone — distance error is now corrected
on the trim layer by the setup :class:`~raccoon.step.calibration.setup.CalibrationGate`.
The lightweight gate flag survives here so distance-based drives can keep warning
when no calibration has run, and the gate marks it done when it finalizes.
"""

from __future__ import annotations

from raccoon.log import warn


class CalibrationRequiredError(Exception):
    """Raised when an operation requires calibration but none has been performed."""


class _CalibrationState:
    """Holds the runtime "have we calibrated yet?" flag."""

    done: bool = False


def is_distance_calibrated() -> bool:
    """Check if distance calibration has been performed."""
    return _CalibrationState.done


def set_distance_calibrated() -> None:
    """Mark distance calibration as performed (called by the calibration gate)."""
    _CalibrationState.done = True


def check_distance_calibration() -> None:
    """Log a warning when distance calibration has not been performed yet."""
    if not _CalibrationState.done:
        warn(
            "Distance calibration highly suggested. Run a setup mission with a "
            "calibration_gate() first."
        )


def reset_distance_calibration() -> None:
    """Reset the calibration flag (for testing)."""
    _CalibrationState.done = False


def load_stored_ir_calibration(
    ir_sensors: list,
    calibration_sets: list[str],
    debug_fn,
    warn_fn,
) -> None:
    """Apply stored IR thresholds from CalibrationStore to each sensor (used under --no-calibrate)."""
    from raccoon import calibration_store as CalibrationStore
    from raccoon.calibration_store import CalibrationType

    for set_name in calibration_sets:
        for sensor in ir_sensors:
            key = f"{set_name}_port{sensor.port}"
            if CalibrationStore.has_readings(CalibrationType.IR_SENSOR, key):
                readings = CalibrationStore.get_readings(CalibrationType.IR_SENSOR, key)
                sensor.setCalibration(readings[1], readings[0])
                debug_fn(
                    f"--no-calibrate: IR port {sensor.port} set '{set_name}': "
                    f"black={readings[1]:.1f} white={readings[0]:.1f}"
                )
            else:
                warn_fn(
                    f"--no-calibrate: no stored readings for IR port {sensor.port} set '{set_name}'"
                )
