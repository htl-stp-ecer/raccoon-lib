from __future__ import annotations

#: Calibration-store section holding the per-axis distance scale factors.
TRIM_STORE_SECTION = "motion-trim"

#: Trim axes that carry a calibrated distance scale factor.
_TRIM_AXES = ("forward", "lateral")


class MotionTrimService:
    """Per-axis distance trim scales applied above the calibrated baseline.

    The forward/lateral scale factors are the last compensation layer after
    offline autotune has already established the motor/odometry baseline. They
    are persisted to the calibration store (section ``motion-trim``) the moment
    they are calibrated, and re-loaded automatically when the service is first
    constructed — including ``--no-calibrate`` runs, which skip the interactive
    trim calibration but must still apply the stored factors.
    """

    def __init__(self, robot) -> None:
        self._robot = robot
        self._axis_scale = {
            "forward": 1.0,
            "lateral": 1.0,
        }
        self._store = self._make_store()
        self._load_stored_scales()

    @staticmethod
    def _make_store():
        from raccoon.step.calibration.store import CalibrationStore

        return CalibrationStore()

    def _load_stored_scales(self) -> None:
        """Apply persisted scale factors; warn when none are available."""
        data = self._store.load(TRIM_STORE_SECTION)
        loaded: list[str] = []
        if data:
            for axis in _TRIM_AXES:
                try:
                    scale = float(data[axis])
                except (KeyError, TypeError, ValueError):
                    continue
                if scale > 0.0:
                    self._axis_scale[axis] = scale
                    loaded.append(f"{axis}={scale:.4f}")

        if loaded:
            self._robot.debug(
                "MotionTrimService: loaded distance scale factors " + ", ".join(loaded)
            )
        else:
            self._robot.warn(
                "MotionTrimService: no stored distance scale factors "
                f"(section '{TRIM_STORE_SECTION}'); using 1.0 for all axes. "
                "Run trim calibration to compensate residual distance error."
            )

    def set_axis_scale(self, axis: str, scale: float) -> None:
        if axis not in self._axis_scale:
            msg = f"unknown trim axis: {axis!r}"
            raise ValueError(msg)
        if not (scale > 0.0):
            msg = f"axis scale must be > 0, got {scale}"
            raise ValueError(msg)
        self._axis_scale[axis] = float(scale)
        self._persist()

    def _persist(self) -> None:
        """Write the current scale factors to the calibration store."""
        self._store.store(TRIM_STORE_SECTION, dict(self._axis_scale))

    def get_axis_scale(self, axis: str) -> float:
        return float(self._axis_scale.get(axis, 1.0))

    def scale_distance_m(self, axis: str, distance_m: float) -> float:
        return float(distance_m) * self.get_axis_scale(axis)
