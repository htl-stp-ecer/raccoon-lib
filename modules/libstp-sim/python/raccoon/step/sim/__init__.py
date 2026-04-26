"""Deprecated location — use :mod:`raccoon.testing.sim` instead.

This path exists only to keep older test files working while they migrate.
It re-exports the public API from :mod:`raccoon.testing.sim` and emits a
``DeprecationWarning`` on first import. It will be removed in a future
release.
"""

from __future__ import annotations

import warnings

warnings.warn(
    "raccoon.step.sim is deprecated; import from raccoon.testing.sim instead. "
    "The old path will be removed in a future release.",
    DeprecationWarning,
    stacklevel=2,
)

from raccoon.testing.sim import (
    DistanceSensorMount,
    LineSensorMount,
    SimRobotConfig,
    configure,
    detach,
    pose,
    tick,
    use_scene,
    yaw_rate,
)

__all__ = [
    "DistanceSensorMount",
    "LineSensorMount",
    "SimRobotConfig",
    "configure",
    "detach",
    "pose",
    "tick",
    "use_scene",
    "yaw_rate",
]
