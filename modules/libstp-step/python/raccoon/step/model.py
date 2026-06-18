from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@runtime_checkable
class StepProtocol(Protocol):
    """Structural protocol implemented by executable step objects."""

    async def run_step(self, robot: "GenericRobot") -> None: ...

    def required_resources(self) -> frozenset[str]: ...

    def collected_resources(self) -> frozenset[str]: ...
