from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ChassisCommand:
    vx: float = 0.0  # m/s
    vy: float = 0.0  # m/s (mecanum/omni only)
    w: float = 0.0  # rad/s


@dataclass(frozen=True)
class UpdateDt:
    seconds: float


@dataclass(frozen=True)
class WheelName:
    name: str
