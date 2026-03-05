from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class IRCalibrationChoice:
    """Result from IROverviewScreen."""
    use_existing: bool


@dataclass
class SensorCalibrationData:
    """Per-sensor calibration data collected during sampling."""
    port: int
    samples: List[float] = field(default_factory=list)
    black_threshold: float = 0.0
    white_threshold: float = 0.0
    black_mean: float = 0.0
    white_mean: float = 0.0
    black_std: float = 0.0
    white_std: float = 0.0

    @property
    def separation(self) -> float:
        return abs(self.white_threshold - self.black_threshold)

    @property
    def is_good(self) -> bool:
        return self.separation > 100


@dataclass
class IRDashboardResult:
    """Result from IRResultsDashboardScreen."""
    confirmed: bool
    sensors: List[SensorCalibrationData] = field(default_factory=list)


@dataclass
class IRConfirmResult:
    """Result from IRConfirmScreen (legacy)."""
    confirmed: bool
    black_threshold: float
    white_threshold: float


@dataclass
class IRSensorCalibrationResult:
    """Result of IR sensor calibration."""
    white_threshold: float
    black_threshold: float
