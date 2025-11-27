from dataclasses import dataclass
from typing import Optional


@dataclass
class AnomalyDetection:
    signature: str
    duration: float
    expected_mean: float
    expected_stddev: float
    threshold_multiplier: float
    deviation_sigma: float
    faster_than_expected: bool
    timestamp: float


@dataclass
class StepStatistics:
    mean: float
    stddev: float
    min: float
    max: float
    count: int
