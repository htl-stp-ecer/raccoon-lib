"""Find and parse a raccoon.project.yml so the pytest plugin can derive a
:class:`~raccoon.testing.sim.SimRobotConfig` automatically.

This is a read-only loader with the minimal ``!include`` / ``!include-merge``
tag support the toolchain uses when splitting a project across multiple YAML
files. It uses PyYAML (already a raccoon dependency) and deliberately does
not share code with the toolchain — the toolchain uses ruamel for
round-trip editing, which is more than we need here.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .sim import SimRobotConfig

PROJECT_FILENAME = "raccoon.project.yml"


@dataclass
class ProjectInfo:
    """Everything the pytest plugin needs about the project under test."""

    root: Path
    project_data: dict[str, Any]
    sim_config: SimRobotConfig

    @property
    def scenes_dir(self) -> Path:
        return self.root / "scenes"


class ProjectNotFoundError(RuntimeError):
    """Raised when the plugin cannot locate a raccoon.project.yml."""


def find_project_root(start: Path | None = None) -> Path:
    """Walk upward from *start* (default: cwd) to locate the project root.

    The project root is the nearest ancestor directory that contains a
    ``raccoon.project.yml`` file.
    """
    here = (start or Path.cwd()).resolve()
    for candidate in (here, *here.parents):
        if (candidate / PROJECT_FILENAME).is_file():
            return candidate
    msg = (
        f"Could not find {PROJECT_FILENAME} by walking upward from {here}. "
        "The raccoon.testing pytest fixtures require a configured project — "
        "run pytest from inside a project directory."
    )
    raise ProjectNotFoundError(msg)


def load_project(root: Path) -> dict[str, Any]:
    """Load ``raccoon.project.yml`` with !include / !include-merge support."""
    return _load_yaml_with_includes(root / PROJECT_FILENAME)


def build_project_info(start: Path | None = None) -> ProjectInfo:
    """Convenience: find + load + derive SimRobotConfig in one call."""
    root = find_project_root(start)
    data = load_project(root)
    cfg = derive_sim_config(data)
    return ProjectInfo(root=root, project_data=data, sim_config=cfg)


def derive_sim_config(data: dict[str, Any]) -> SimRobotConfig:
    """Extract SimRobotConfig fields from a loaded project.yml.

    Missing fields fall back to SimRobotConfig defaults. This is a *best
    effort* — if a specific value matters for a test, override it on the
    config returned by the ``robot_sim_config`` fixture before entering a
    scene.
    """
    cfg = SimRobotConfig()

    robot_cfg = (data.get("robot") or {}) if isinstance(data, dict) else {}
    definitions = (data.get("definitions") or {}) if isinstance(data, dict) else {}

    physical = robot_cfg.get("physical") or {}
    width = _as_float(physical.get("width_cm"))
    length = _as_float(physical.get("length_cm"))
    if width is not None:
        cfg.width_cm = width
    if length is not None:
        cfg.length_cm = length

    rc = physical.get("rotation_center") or {}
    rc_x = _as_float(rc.get("x_cm"))
    rc_y = _as_float(rc.get("y_cm"))
    if rc_x is not None and width is not None:
        cfg.rotation_center_strafe_cm = rc_x - width / 2.0
    if rc_y is not None and length is not None:
        cfg.rotation_center_forward_cm = rc_y - length / 2.0

    kinematics = ((robot_cfg.get("drive") or {}).get("kinematics")) or {}
    wheel_radius = _as_float(kinematics.get("wheel_radius"))
    wheelbase = _as_float(kinematics.get("wheelbase"))
    if wheel_radius is not None:
        cfg.wheel_radius_m = wheel_radius
    if wheelbase is not None:
        cfg.track_width_m = wheelbase
        cfg.wheelbase_m = wheelbase

    left_name = kinematics.get("left_motor")
    right_name = kinematics.get("right_motor")
    left_def = definitions.get(left_name) if isinstance(left_name, str) else None
    right_def = definitions.get(right_name) if isinstance(right_name, str) else None

    left_port = _as_int((left_def or {}).get("port"))
    right_port = _as_int((right_def or {}).get("port"))
    if left_port is not None:
        cfg.left_motor_port = left_port
    if right_port is not None:
        cfg.right_motor_port = right_port

    left_inv = (left_def or {}).get("inverted")
    right_inv = (right_def or {}).get("inverted")
    if isinstance(left_inv, bool):
        cfg.left_motor_inverted = left_inv
    if isinstance(right_inv, bool):
        cfg.right_motor_inverted = right_inv

    ticks_to_rad = _as_float(((left_def or {}).get("calibration") or {}).get("ticks_to_rad"))
    if ticks_to_rad is not None and ticks_to_rad > 0:
        cfg.ticks_to_rad = ticks_to_rad

    motion_pid = robot_cfg.get("motion_pid") or {}
    linear_max = _as_float((motion_pid.get("linear") or {}).get("max_velocity"))
    if linear_max is not None and wheel_radius and wheel_radius > 0:
        # Give the sim a little headroom above the motion PID cap so the
        # PID never gets clamped by the sim's motor saturation.
        cfg.max_wheel_velocity_rad_s = (linear_max / wheel_radius) * 1.5

    return cfg


# ---------------------------------------------------------------------------
# YAML loading with !include / !include-merge
# ---------------------------------------------------------------------------

_MERGE_SENTINEL = object()


class _IncludeLoader(yaml.SafeLoader):
    """SafeLoader subclass that tracks the directory of the file being read.

    The !include / !include-merge constructors resolve their path argument
    relative to this directory, mirroring the toolchain's behavior.
    """

    def __init__(self, stream):
        try:
            self._base_dir = Path(stream.name).resolve().parent
        except AttributeError:
            self._base_dir = Path.cwd()
        super().__init__(stream)


def _include_constructor(loader: _IncludeLoader, node: yaml.Node) -> Any:
    rel = loader.construct_scalar(node)  # type: ignore[arg-type]
    path = (loader._base_dir / str(rel)).resolve()
    return _load_yaml_with_includes(path)


def _include_merge_constructor(loader: _IncludeLoader, node: yaml.Node) -> Any:
    rel = loader.construct_scalar(node)  # type: ignore[arg-type]
    path = (loader._base_dir / str(rel)).resolve()
    data = _load_yaml_with_includes(path)
    if not isinstance(data, dict):
        msg = f"!include-merge at {rel} must resolve to a mapping, " f"got {type(data).__name__}"
        raise ValueError(msg)
    return (_MERGE_SENTINEL, data)


_IncludeLoader.add_constructor("!include", _include_constructor)
_IncludeLoader.add_constructor("!include-merge", _include_merge_constructor)


def _post_process_merges(data: Any) -> Any:
    """Fold !include-merge sentinel tuples into their parent mappings."""
    if not isinstance(data, dict):
        return data

    merges = []
    for key, value in list(data.items()):
        if isinstance(value, tuple) and len(value) == 2 and value[0] is _MERGE_SENTINEL:
            merges.append((key, value[1]))
        else:
            _post_process_merges(value)

    for key, merge_dict in merges:
        del data[key]
        data.update(merge_dict)

    return data


def _load_yaml_with_includes(path: Path) -> Any:
    path = Path(path).resolve()
    with path.open("r", encoding="utf-8") as f:
        data = yaml.load(f, Loader=_IncludeLoader)
    if data is None:
        return {}
    return _post_process_merges(data)


def _as_float(v: Any) -> float | None:
    if v is None or isinstance(v, bool):
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _as_int(v: Any) -> int | None:
    if v is None or isinstance(v, bool):
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


__all__ = [
    "PROJECT_FILENAME",
    "ProjectInfo",
    "ProjectNotFoundError",
    "build_project_info",
    "derive_sim_config",
    "find_project_root",
    "load_project",
]
