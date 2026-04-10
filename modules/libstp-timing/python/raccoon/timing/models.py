from dataclasses import dataclass
from typing import Optional


@dataclass
class AnomalyDetection:
    """An outlier execution relative to the recent baseline for one signature."""

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
    """Rolling statistics computed from recent non-anomalous step durations."""

    mean: float
    stddev: float
    min: float
    max: float
    count: int
