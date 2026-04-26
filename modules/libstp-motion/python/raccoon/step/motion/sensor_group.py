"""
SensorGroup: pre-bind IR sensor pairs with default threshold, speed, and
PID gains so mission code doesn't repeat them on every call.

Usage::

    from raccoon.step.motion.sensor_group import SensorGroup

    front = SensorGroup(
        left=front_left_ir,
        right=front_right_ir,
    )

    # In a mission:
    front.lineup_on_black()
    front.drive_until_black()
    front.drive_over_line()
    front.follow_right_edge(cm=125)

    # Override defaults per-call:
    front.lineup_on_black(threshold=0.5)
    front.strafe_left_until_black(speed=0.3, threshold=0.3)

    # Single-sensor access for raw builder calls:
    strafe_left(speed=0.3).until(on_black(front.right))
"""

from __future__ import annotations

from raccoon.step.condition import on_black, on_white


def _any_condition(factory, *sensors, threshold=0.7):
    """Compose stop conditions for multiple sensors with OR."""
    cond = factory(sensors[0], threshold)
    for s in sensors[1:]:
        cond = cond | factory(s, threshold)
    return cond


class SensorGroup:
    """A named sensor pair with pre-bound defaults for threshold, speed, and PID.

    Provides convenience methods for common sensor-triggered operations:
    lineup, drive-until, strafe-until, and line-following. All methods
    return Step builders that can be used directly in ``seq([...])``.

    Args:
        left: IR sensor mounted on the left side (or None).
        right: IR sensor mounted on the right side (or None).
        threshold: Default confidence threshold (0.0--1.0) for black/white
            detection. Defaults to 0.7.
        speed: Default motion speed fraction (0.0--1.0). Defaults to 1.0.
        follow_speed: Default speed for line-following. Defaults to 0.8.
        follow_kp: Proportional gain for line-follow PID. Defaults to 0.5.
        follow_ki: Integral gain for line-follow PID. Defaults to 0.02.
        follow_kd: Derivative gain for line-follow PID. Defaults to 0.0.
    """

    def __init__(
        self,
        left=None,
        right=None,
        threshold: float = 0.7,
        speed: float = 1.0,
        follow_speed: float = 0.8,
        follow_kp: float = 0.5,
        follow_ki: float = 0.02,
        follow_kd: float = 0.0,
    ):
        self.left = left
        self.right = right
        self._both = [s for s in [left, right] if s is not None]
        self._threshold = threshold
        self._speed = speed
        self._follow_speed = follow_speed
        self._follow_kp = follow_kp
        self._follow_ki = follow_ki
        self._follow_kd = follow_kd

    def _t(self, override):
        return override if override is not None else self._threshold

    def _s(self, override):
        return override if override is not None else self._speed

    # ── paired lineup ───────────────────────────────────────────

    def lineup_on_black(self, threshold=None):
        from .lineup import forward_lineup_on_black

        return forward_lineup_on_black(
            self.left, self.right, detection_threshold=self._t(threshold)
        )

    def lineup_on_white(self, threshold=None):
        from .lineup import forward_lineup_on_white

        return forward_lineup_on_white(self.left, self.right)

    def lineup(self, target=None, threshold=None):
        from .lineup import SurfaceColor
        from .lineup import lineup as _lineup

        if target is None:
            target = SurfaceColor.BLACK
        return _lineup(
            left_sensor=self.left,
            right_sensor=self.right,
            target=target,
            detection_threshold=self._t(threshold),
        )

    def backward_lineup_on_black(self):
        from .lineup import backward_lineup_on_black

        return backward_lineup_on_black(self.left, self.right)

    def backward_lineup_on_white(self):
        from .lineup import backward_lineup_on_white

        return backward_lineup_on_white(self.left, self.right)

    # ── drive until ─────────────────────────────────────────────

    def drive_until_black(self, speed=None, threshold=None):
        from .drive_dsl import drive_forward

        return drive_forward(speed=self._s(speed)).until(
            _any_condition(on_black, *self._both, threshold=self._t(threshold))
        )

    def drive_until_white(self, speed=None, threshold=None):
        from .drive_dsl import drive_forward

        return drive_forward(speed=self._s(speed)).until(
            _any_condition(on_white, *self._both, threshold=self._t(threshold))
        )

    def drive_over_line(self, speed=None, threshold=None):
        """Drive forward through black then white (crosses one line)."""
        from raccoon.step import seq

        return seq(
            [
                self.drive_until_black(speed, threshold),
                self.drive_until_white(speed, threshold),
            ]
        )

    def drive_backward_until_black(self, sensor=None, speed=None, threshold=None):
        from .drive_dsl import drive_backward

        s = sensor or self.right or self.left
        return drive_backward(speed=self._s(speed)).until(on_black(s, self._t(threshold)))

    # ── strafe until ────────────────────────────────────────────

    def strafe_left_until_black(self, sensor=None, speed=None, threshold=None):
        from .drive_dsl import strafe_left

        s = sensor or self.left or self.right
        return strafe_left(speed=self._s(speed)).until(on_black(s, self._t(threshold)))

    def strafe_right_until_black(self, sensor=None, speed=None, threshold=None):
        from .drive_dsl import strafe_right

        s = sensor or self.right or self.left
        return strafe_right(speed=self._s(speed)).until(on_black(s, self._t(threshold)))

    def strafe_right_until_white(self, sensor=None, speed=None, threshold=None):
        from .drive_dsl import strafe_right

        s = sensor or self.right or self.left
        return strafe_right(speed=self._s(speed)).until(on_white(s, self._t(threshold)))

    # ── line following ──────────────────────────────────────────

    def follow_right_edge(self, cm, speed=None):
        from .line_follow import LineSide
        from .line_follow_dsl import follow_line_single

        return follow_line_single(
            self.right,
            cm,
            speed or self._follow_speed,
            side=LineSide.RIGHT,
            kp=self._follow_kp,
            ki=self._follow_ki,
            kd=self._follow_kd,
        )

    def follow_right_until_black(self, speed=None):
        from .line_follow import LineSide
        from .line_follow_dsl import follow_line_single

        return follow_line_single(
            self.right,
            speed=speed or self._follow_speed,
            side=LineSide.RIGHT,
            kp=self._follow_kp,
            ki=self._follow_ki,
            kd=self._follow_kd,
        ).until(on_black(self.left, threshold=1.0))
