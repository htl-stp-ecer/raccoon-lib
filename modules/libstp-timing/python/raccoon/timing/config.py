from __future__ import annotations

from dataclasses import dataclass


@dataclass
class TimingConfig:
    """Runtime knobs for step timing collection and anomaly detection."""

    threshold_multiplier: float = 3.0
    window_size: int = 20
    db_path: str = ".raccoon/step_timing.db"
    enabled: bool = True
    # Floor on the rolling stddev, expressed as a fraction of the mean.
    # Very consistent steps drive the measured stddev toward zero, which
    # would otherwise shrink the anomaly band to nothing and flag ordinary
    # run-to-run jitter as an anomaly. The effective stddev used for the
    # bounds is at least ``min_relative_stddev * abs(mean)``.
    min_relative_stddev: float = 0.10
