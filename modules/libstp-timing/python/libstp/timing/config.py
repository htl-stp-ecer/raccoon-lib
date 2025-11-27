from dataclasses import dataclass


@dataclass
class TimingConfig:
    threshold_multiplier: float = 3.0
    window_size: int = 20
    db_path: str = "logs/step_timing.db"
    enabled: bool = True
