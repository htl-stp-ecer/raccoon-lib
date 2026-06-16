from __future__ import annotations


class MotionTrimService:
    """Per-run residual distance trims applied above the calibrated baseline.

    These trims are intentionally session-scoped and in-memory only. They are
    the last compensation layer after offline autotune has already established
    the motor/odometry baseline.
    """

    def __init__(self, robot) -> None:
        self._robot = robot
        self._axis_scale = {
            "forward": 1.0,
            "lateral": 1.0,
        }

    def set_axis_scale(self, axis: str, scale: float) -> None:
        if axis not in self._axis_scale:
            msg = f"unknown trim axis: {axis!r}"
            raise ValueError(msg)
        if not (scale > 0.0):
            msg = f"axis scale must be > 0, got {scale}"
            raise ValueError(msg)
        self._axis_scale[axis] = float(scale)

    def get_axis_scale(self, axis: str) -> float:
        return float(self._axis_scale.get(axis, 1.0))

    def scale_distance_m(self, axis: str, distance_m: float) -> float:
        return float(distance_m) * self.get_axis_scale(axis)
