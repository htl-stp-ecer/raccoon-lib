from __future__ import annotations

from dataclasses import dataclass


@dataclass
class TimingConfig:
    """Runtime knobs for step timing collection and anomaly detection."""

    threshold_multiplier: float = 3.0
    window_size: int = 20
    db_path: str = "logs/step_timing.db"
    enabled: bool = True
