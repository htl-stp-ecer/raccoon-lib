from __future__ import annotations

from enum import StrEnum

from raccoon.sensor_ir import IRSensor
from raccoon.step.annotation import dsl
from raccoon.step.condition import StopCondition
from raccoon.step.step_builder import StepBuilder

from .line_follow import (
    DirectionalLineFollow,
    DirectionalLineFollowConfig,
    DirectionalSingleLineFollow,
    DirectionalSingleLineFollowConfig,
    LineSide,
)

_UNSET = object()


class FollowCorrection(StrEnum):
    ANGULAR = "angular"
    LATERAL = "lateral"
    FORWARD = "forward"


def _forward_correction_sign_for_lateral_follow(strafe_speed: float) -> float:
    return -1.0 if strafe_speed >= 0.0 else 1.0


class ConfigurableLineFollowBuilder(StepBuilder):
    def __init__(self) -> None:
        super().__init__()
        self._left_sensor = _UNSET
        self._right_sensor = _UNSET
        self._sensor = _UNSET
        self._side = LineSide.LEFT
        self._forward_speed = 0.0
        self._strafe_speed = 0.0
        self._distance_cm: float | None = None
        self._kp = 0.4
        self._ki = 0.0
        self._kd = 0.1
        self._until: StopCondition | None = None
        self._correction = FollowCorrection.ANGULAR
        self._heading_hold = True
        self._correction_sign = 1.0

    def dual(self, left_sensor: IRSensor, right_sensor: IRSensor) -> ConfigurableLineFollowBuilder:
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._sensor = _UNSET
        return self

    def single(
        self,
        sensor: IRSensor,
        side: LineSide = LineSide.LEFT,
    ) -> ConfigurableLineFollowBuilder:
        self._sensor = sensor
        self._side = side
        self._left_sensor = _UNSET
        self._right_sensor = _UNSET
        return self

    def move(
        self,
        forward: float = 0.0,
        strafe: float = 0.0,
    ) -> ConfigurableLineFollowBuilder:
        self._forward_speed = forward
        self._strafe_speed = strafe
        return self

    def forward_speed(self, value: float) -> ConfigurableLineFollowBuilder:
        self._forward_speed = value
        return self

    def strafe_speed(self, value: float) -> ConfigurableLineFollowBuilder:
        self._strafe_speed = value
        return self

    def correct_angular(self) -> ConfigurableLineFollowBuilder:
        self._correction = FollowCorrection.ANGULAR
        self._heading_hold = True
        return self

    def correct_lateral(self, hold_heading: bool = True) -> ConfigurableLineFollowBuilder:
        self._correction = FollowCorrection.LATERAL
        self._heading_hold = hold_heading
        return self

    def correct_forward(self, hold_heading: bool = True) -> ConfigurableLineFollowBuilder:
        self._correction = FollowCorrection.FORWARD
        self._heading_hold = hold_heading
        return self

    def correction_sign(self, value: float) -> ConfigurableLineFollowBuilder:
        self._correction_sign = value
        return self

    def distance_cm(self, value: float | None) -> ConfigurableLineFollowBuilder:
        self._distance_cm = value
        return self

    def pid(self, kp: float, ki: float = 0.0, kd: float = 0.1) -> ConfigurableLineFollowBuilder:
        self._kp = kp
        self._ki = ki
        self._kd = kd
        return self

    def until(self, value: StopCondition | None) -> ConfigurableLineFollowBuilder:
        self._until = value
        return self

    def _build(self):
        if self._distance_cm is None and self._until is None:
            msg = "line_follow() requires either distance_cm(...) or until(...)"
            raise ValueError(msg)

        has_pair = self._left_sensor is not _UNSET or self._right_sensor is not _UNSET
        has_single = self._sensor is not _UNSET
        if has_pair == has_single:
            msg = "line_follow() requires exactly one tracking mode: single(...) or dual(...)"
            raise ValueError(msg)
        if has_pair and (self._left_sensor is _UNSET or self._right_sensor is _UNSET):
            msg = "dual(...) requires both left and right sensors"
            raise ValueError(msg)

        forward_correction = self._correction == FollowCorrection.FORWARD
        lateral_correction = self._correction == FollowCorrection.LATERAL

        if lateral_correction and self._strafe_speed != 0.0:
            msg = "correct_lateral() uses strafe as the correction axis; base strafe must stay 0"
            raise ValueError(msg)
        if forward_correction and self._forward_speed != 0.0:
            msg = "correct_forward() uses forward as the correction axis; base forward must stay 0"
            raise ValueError(msg)
        if self._correction == FollowCorrection.ANGULAR and not (
            self._forward_speed or self._strafe_speed
        ):
            msg = "correct_angular() needs non-zero base motion"
            raise ValueError(msg)
        correction_sign = self._correction_sign
        if forward_correction:
            correction_sign *= _forward_correction_sign_for_lateral_follow(self._strafe_speed)

        common_kwargs = {
            "heading_speed": self._forward_speed,
            "strafe_speed": self._strafe_speed,
            "distance_cm": self._distance_cm,
            "kp": self._kp,
            "ki": self._ki,
            "kd": self._kd,
            "lateral_correction": lateral_correction,
            "forward_correction": forward_correction,
            "heading_hold": self._heading_hold,
            "correction_sign": correction_sign,
        }

        if has_single:
            return DirectionalSingleLineFollow(
                DirectionalSingleLineFollowConfig(
                    sensor=self._sensor,
                    side=self._side,
                    **common_kwargs,
                ),
                until=self._until,
            )

        return DirectionalLineFollow(
            DirectionalLineFollowConfig(
                left_sensor=self._left_sensor,
                right_sensor=self._right_sensor,
                **common_kwargs,
            ),
            until=self._until,
        )


@dsl(tags=["motion", "line-follow"])
def line_follow() -> ConfigurableLineFollowBuilder:
    return ConfigurableLineFollowBuilder()


__all__ = [
    "ConfigurableLineFollowBuilder",
    "FollowCorrection",
    "line_follow",
]
