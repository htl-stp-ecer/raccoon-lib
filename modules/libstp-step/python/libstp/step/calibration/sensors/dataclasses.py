from dataclasses import dataclass


@dataclass
class IRCalibrationChoice:
    """Result from IROverviewScreen."""
    use_existing: bool

@dataclass
class IRConfirmResult:
    """Result from IRConfirmScreen."""
    confirmed: bool
    black_threshold: float
    white_threshold: float

@dataclass
class IRSensorCalibrationResult:
    """Result of IR sensor calibration."""
    white_threshold: float
    black_threshold: float