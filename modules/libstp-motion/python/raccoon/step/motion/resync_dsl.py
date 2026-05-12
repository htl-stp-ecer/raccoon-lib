"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: resync.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .resync import ResyncAtStartPose


class ResyncAtStartPoseBuilder(StepBuilder):
    """Builder for ResyncAtStartPose. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._expected_x_cm = None
        self._expected_y_cm = None
        self._expected_theta_deg = None
        self._snap_axes = (True, True, True)
        self._sigma_xy_cm = 1.0
        self._sigma_theta_deg = 5.0

    def expected_x_cm(self, value: float | None):
        self._expected_x_cm = value
        return self

    def expected_y_cm(self, value: float | None):
        self._expected_y_cm = value
        return self

    def expected_theta_deg(self, value: float | None):
        self._expected_theta_deg = value
        return self

    def snap_axes(self, value: tuple[bool, bool, bool]):
        self._snap_axes = value
        return self

    def sigma_xy_cm(self, value: float):
        self._sigma_xy_cm = value
        return self

    def sigma_theta_deg(self, value: float):
        self._sigma_theta_deg = value
        return self

    def _build(self):
        kwargs = {}
        kwargs["expected_x_cm"] = self._expected_x_cm
        kwargs["expected_y_cm"] = self._expected_y_cm
        kwargs["expected_theta_deg"] = self._expected_theta_deg
        kwargs["snap_axes"] = self._snap_axes
        kwargs["sigma_xy_cm"] = self._sigma_xy_cm
        kwargs["sigma_theta_deg"] = self._sigma_theta_deg
        return ResyncAtStartPose(**kwargs)


@dsl(tags=["localization", "resync"])
def resync_at_start_pose(
    expected_x_cm: float | None = None,
    expected_y_cm: float | None = None,
    expected_theta_deg: float | None = None,
    snap_axes: tuple[bool, bool, bool] = (True, True, True),
    sigma_xy_cm: float = 1.0,
    sigma_theta_deg: float = 5.0,
):
    """
    Inject a direct absolute pose observation without additional motion.

    Reads the robot's current odometry pose (or uses supplied override values)
    and feeds it as an absolute-pose observation into the localization filter.
    Use this at the start of a run to anchor the EKF to the known start pose.

    Args:
        expected_x_cm: Override for the expected X position in cm. If None, the robot's current odometric X is used.
        expected_y_cm: Override for the expected Y position in cm. If None, the robot's current odometric Y is used.
        expected_theta_deg: Override for the expected heading in degrees. If None, the robot's current odometric heading is used.
        snap_axes: Which axes (x, y, theta) to include in the observation. Defaults to (True, True, True) — all three.
        sigma_xy_cm: Position uncertainty (standard deviation) in cm.
        sigma_theta_deg: Heading uncertainty (standard deviation) in degrees.

    Returns:
        A ResyncAtStartPoseBuilder (chainable via ``.expected_x_cm()``, ``.expected_y_cm()``, ``.expected_theta_deg()``, ``.snap_axes()``, ``.sigma_xy_cm()``, ``.sigma_theta_deg()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import resync_at_start_pose

        resync_at_start_pose(expected_x_cm=0.0, expected_y_cm=0.0, expected_theta_deg=0.0)
    """
    b = ResyncAtStartPoseBuilder()
    b._expected_x_cm = expected_x_cm
    b._expected_y_cm = expected_y_cm
    b._expected_theta_deg = expected_theta_deg
    b._snap_axes = snap_axes
    b._sigma_xy_cm = sigma_xy_cm
    b._sigma_theta_deg = sigma_theta_deg
    return b


__all__ = ["ResyncAtStartPoseBuilder", "resync_at_start_pose"]
