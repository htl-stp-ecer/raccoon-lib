from __future__ import annotations

from typing import Any

from libstp.kinematics import DifferentialKinematics, MecanumKinematics

from .config import RobotConfig
from .errors import UnsupportedDriveError

def build_kinematics(cfg: RobotConfig) -> Any:
    t = cfg.type
    kin = cfg.kinematics

    if t == "differential":
        if DifferentialKinematics is not None:
            return DifferentialKinematics(kin.wheel_diameter, kin.wheelbase_width)
        return MockKinematics("differential")

    if t == "mecanum":
        if MecanumKinematics is not None:
            if kin.wheelbase_length is None:
                raise UnsupportedDriveError("mecanum requires 'wheelbase_length' in kinematics")
            return MecanumKinematics(kin.wheel_diameter, kin.wheelbase_width, kin.wheelbase_length)
        return MockKinematics("mecanum")

    raise UnsupportedDriveError(f"Unsupported drive type: {t}")
