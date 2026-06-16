"""Explicitly choose which odometry source should be preferred.

Unlike SpeedMode, odometry-source selection is purely a host-side preference:
the HAL keeps publishing internal STM32 odometry at all times, and the
calibration board remains secondary unless a caller explicitly prefers it.
If the requested source is unavailable, ``get_active_source()`` will continue
to report the source actually in use.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from raccoon.hal import OdometrySource

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["motion", "feature", "odometry"])
class SetOdometrySource(Step):
    """Select the preferred odometry source for subsequent pose reads.

    This does not force the source to exist. It updates the odometry object's
    preference, then the HAL resolves the active source from that preference and
    actual hardware availability. On wombat, preferring the calibration board
    while it is connected makes it the active pose source; otherwise internal
    odometry remains active.

    Args:
        source: Preferred odometry source enum.

    Example::

        from raccoon.hal import OdometrySource
        from raccoon.step.motion import set_odometry_source

        set_odometry_source(OdometrySource.CALIBRATION_BOARD)
    """

    def __init__(self, source: OdometrySource) -> None:
        super().__init__()
        if not isinstance(source, OdometrySource):
            msg = f"source must be an OdometrySource, got {type(source).__name__}"
            raise TypeError(msg)
        self._source = source

    def _generate_signature(self) -> str:
        return f"SetOdometrySource(source={self._source.name})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        robot.odometry.set_preferred_source(self._source)
        active = robot.odometry.get_active_source()
        if active == self._source:
            self.info(f"Odometry source active: {active.name}")
        else:
            self.warn(
                f"Preferred odometry source is {self._source.name}, "
                f"but active source is {active.name}"
            )


__all__ = ["SetOdometrySource"]
