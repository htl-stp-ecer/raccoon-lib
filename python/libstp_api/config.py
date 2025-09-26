from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Any, Optional, List
import yaml

from .errors import ConfigError

DriveType = str  # "differential" | "mecanum" (kept simple; validated at load time)

@dataclass(frozen=True)
class KinematicsConfig:
    wheel_diameter: float
    wheelbase_width: float
    wheelbase_length: Optional[float] = None
    # Motor logical names as defined under definitions:
    left_wheel: Optional[str] = None
    right_wheel: Optional[str] = None
    front_left_wheel: Optional[str] = None
    front_right_wheel: Optional[str] = None
    back_left_wheel: Optional[str] = None
    back_right_wheel: Optional[str] = None

@dataclass(frozen=True)
class ChassisLimits:
    max_velocity: float
    max_angular_velocity: float

@dataclass(frozen=True)
class WheelLimitsCfg:
    max_angular_velocity: float
    max_angular_acceleration: float

@dataclass(frozen=True)
class LimitsConfig:
    chassis: ChassisLimits
    wheels: WheelLimitsCfg

@dataclass(frozen=True)
class RobotConfig:
    type: DriveType
    kinematics: KinematicsConfig
    limits: LimitsConfig
    definitions: Mapping[str, Mapping[str, Any]]


def _require(d: Mapping[str, Any], key: str, ctx: str) -> Any:
    if key not in d:
        raise ConfigError(f"Missing '{key}' in {ctx} configuration")
    return d[key]


def _parse_kinematics(d: Mapping[str, Any]) -> KinematicsConfig:
    wheel_diameter = float(_require(d, "wheel_diameter", "kinematics"))
    wheelbase_width = float(_require(d, "wheelbase_width", "kinematics"))
    wheelbase_length = d.get("wheelbase_length")
    if wheelbase_length is not None:
        wheelbase_length = float(wheelbase_length)

    # optional motor name bindings
    return KinematicsConfig(
        wheel_diameter=wheel_diameter,
        wheelbase_width=wheelbase_width,
        wheelbase_length=wheelbase_length,
        left_wheel=d.get("left_wheel"),
        right_wheel=d.get("right_wheel"),
        front_left_wheel=d.get("front_left_wheel"),
        front_right_wheel=d.get("front_right_wheel"),
        back_left_wheel=d.get("back_left_wheel"),
        back_right_wheel=d.get("back_right_wheel"),
    )


def _parse_limits(d: Mapping[str, Any]) -> LimitsConfig:
    ch = _require(d, "chassis", "limits")
    wh = _require(d, "wheels", "limits")
    chassis = ChassisLimits(
        max_velocity=float(_require(ch, "max_velocity", "limits.chassis")),
        max_angular_velocity=float(_require(ch, "max_angular_velocity", "limits.chassis")),
    )
    wheels = WheelLimitsCfg(
        max_angular_velocity=float(_require(wh, "max_angular_velocity", "limits.wheels")),
        max_angular_acceleration=float(_require(wh, "max_angular_acceleration", "limits.wheels")),
    )
    return LimitsConfig(chassis=chassis, wheels=wheels)


def load_config(source: str | Path | Mapping[str, Any]) -> RobotConfig:
    """Load and validate the YAML or dict configuration.

    Keeping configurable data at the top level, we return an immutable
    dataclass tree. No libstp dependencies here.
    """
    if isinstance(source, (str, Path)):
        p = Path(source)
        if not p.exists():
            raise ConfigError(f"Configuration file not found: {p}")
        with p.open("r") as f:
            raw: Dict[str, Any] = yaml.safe_load(f) or {}
    else:
        raw = dict(source)

    robot = raw.get("robot")
    if not isinstance(robot, dict):
        raise ConfigError("Top-level 'robot' section is required")

    definitions = raw.get("definitions", {})
    if not isinstance(definitions, dict):
        raise ConfigError("Top-level 'definitions' must be a mapping")

    drive_type = str(robot.get("type", "differential")).lower()
    if drive_type not in {"differential", "mecanum"}:
        raise ConfigError(f"Unsupported drive 'type': {drive_type}")

    kin = _parse_kinematics(_require(robot, "kinematics", "robot"))
    lim = _parse_limits(robot.get("limits", {
        "chassis": {"max_velocity": 2.0, "max_angular_velocity": 8.0},
        "wheels": {"max_angular_velocity": 100.0, "max_angular_acceleration": 1000.0},
    }))

    return RobotConfig(
        type=drive_type,
        kinematics=kin,
        limits=lim,
        definitions=definitions,
    )