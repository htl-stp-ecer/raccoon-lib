from __future__ import annotations

from contextlib import contextmanager

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
        # Re-entrant depth counter for :meth:`bypass_scaling`. While > 0, the
        # trim scale is NOT applied to commanded distances (see
        # :meth:`scale_distance_m`). Used to run calibration *measurement* drives
        # raw, so the odom-vs-ground-truth ratio they produce does not depend on
        # the currently-stored (possibly wrong) scale.
        self._bypass_depth = 0
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

    def calibrate_axis(self, axis: str, requested_m: float, measured_m: float) -> float:
        """Fold a fresh calibration measurement into the axis scale factor.

        A calibration drive runs *through* the trim layer: commanding
        ``requested_m`` actually targets ``requested_m * current_scale`` of
        physical distance (see :meth:`scale_distance_m`). The raw correction
        ``requested_m / measured_m`` is therefore relative to the already-scaled
        command, so it must be *composed* with the current factor rather than
        replacing it — otherwise the previously-applied scale is silently
        discarded::

            new_scale = current_scale * (requested_m / measured_m)

        Args:
            axis: Trim axis, ``"forward"`` or ``"lateral"``.
            requested_m: Distance requested for the calibration drive (metres),
                before the trim scale was applied.
            measured_m: Distance the robot actually travelled (metres).

        Returns:
            The newly stored, composed scale factor for ``axis``.
        """
        if not (measured_m > 0.0):
            msg = f"measured distance must be > 0, got {measured_m}"
            raise ValueError(msg)
        correction = requested_m / measured_m
        new_scale = self.get_axis_scale(axis) * correction
        self.set_axis_scale(axis, new_scale)
        return new_scale

    def _persist(self) -> None:
        """Write the current scale factors to the calibration store."""
        self._store.store(TRIM_STORE_SECTION, dict(self._axis_scale))

    def get_axis_scale(self, axis: str) -> float:
        return float(self._axis_scale.get(axis, 1.0))

    @property
    def scaling_bypassed(self) -> bool:
        """True while inside a :meth:`bypass_scaling` context."""
        return self._bypass_depth > 0

    @contextmanager
    def bypass_scaling(self):
        """Run commanded distances *raw* (untrimmed) for the duration.

        A calibration *measurement* drive must not be scaled by the
        currently-stored factor: if it were, the physical distance travelled —
        and therefore the ``odom / ground_truth`` ratio it yields — would depend
        on the (possibly wrong) scale already in effect, so the measurement
        could never correct a bad scale and would not be reproducible run to run.
        Commanding e.g. 70 cm inside this context always targets 70 cm of
        odometry, independent of the stored scale. Re-entrant.
        """
        self._bypass_depth += 1
        try:
            yield
        finally:
            self._bypass_depth -= 1

    def scale_distance_m(self, axis: str, distance_m: float) -> float:
        if self._bypass_depth > 0:
            return float(distance_m)
        return float(distance_m) * self.get_axis_scale(axis)
