"""Setup-time calibration: opportunistic evidence collection + a finalizing gate.

The setup mission scatters :func:`collect_drive` / :func:`collect_ir_set` wrappers
around its motions to gather ground-truth distance and IR samples without any
dedicated calibration drives. A single :func:`calibration_gate` then folds that
evidence into the persisted compensation layers — distance trim via
``MotionTrimService.calibrate_axis`` and IR thresholds via the calibration store —
falling back to a measured drive (and manual measurement) only for axes/sets that
never accumulated enough samples.
"""

from __future__ import annotations

from .session import (
    CalibrationAxis,
    DriveCalibrationSample,
    IrCalibrationSet,
    SetupCalibrationSession,
)
from .steps import (
    CalibrationGate,
    CollectDrive,
    CollectIrSet,
    calibration_gate,
    collect_drive,
    collect_ir_set,
)

__all__ = [
    "CalibrationAxis",
    "CalibrationGate",
    "CollectDrive",
    "CollectIrSet",
    "DriveCalibrationSample",
    "IrCalibrationSet",
    "SetupCalibrationSession",
    "calibration_gate",
    "collect_drive",
    "collect_ir_set",
]
