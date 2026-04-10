"""Reusable base class for interactive calibration steps.

Subclasses implement four hooks to define what data is collected, how it is
confirmed, how it is applied to hardware, and how it is serialised for
persistent storage.  The base class takes care of:

* Running optional *setup_steps* before the first attempt and on every retry.
* The collect → confirm → apply/retry loop.
* Persisting confirmed values to ``racoon.calibration.yml``.
* Honouring ``--no-calibrate``: loading stored values instead of running the
  interactive flow.
"""
from __future__ import annotations

from abc import abstractmethod
from typing import Generic, TypeVar

from raccoon.no_calibrate import is_no_calibrate
from raccoon.ui.step import UIStep

from ..base import Step
from .store import CalibrationStore

T = TypeVar("T")


class CalibrateStep(UIStep, Generic[T]):
    """Template for an interactive calibration step with persistence.

    Type parameter *T* is the calibration payload — the domain-specific
    value object that flows through collect → confirm → apply → store.
    """

    def __init__(
        self,
        store_section: str,
        store_set: str = "default",
        setup_steps: list[Step] | None = None,
    ) -> None:
        super().__init__()
        self._store_section = store_section
        self._store_set = store_set
        self._setup_steps = setup_steps or []
        self._store = CalibrationStore()

    # ── hooks for subclasses ─────────────────────────────────────

    @abstractmethod
    async def _collect(self, robot: "GenericRobot") -> T | None:
        """Collect sensor data and compute a calibration proposal.

        Return *None* to silently retry (e.g. too few samples).
        """

    @abstractmethod
    async def _confirm(self, robot: "GenericRobot", calibration: T) -> tuple[bool, T]:
        """Present the calibration to the user for confirmation.

        Returns ``(confirmed, calibration)`` — the calibration may have been
        adjusted by the user via the UI.
        """

    @abstractmethod
    def _apply(self, robot: "GenericRobot", calibration: T) -> None:
        """Apply the confirmed calibration to the running hardware."""

    @abstractmethod
    def _serialize(self, calibration: T) -> dict:
        """Convert *calibration* to a plain dict for YAML storage."""

    @abstractmethod
    def _deserialize(self, data: dict) -> T:
        """Reconstruct a calibration object from a stored dict."""

    # ── orchestration ────────────────────────────────────────────

    async def _run_setup(self, robot: "GenericRobot") -> None:
        for step in self._setup_steps:
            await step.run_step(robot)

    async def _execute_step(self, robot: "GenericRobot") -> None:
        if is_no_calibrate():
            data = self._store.load(self._store_section, self._store_set)
            if data is not None:
                self._apply(robot, self._deserialize(data))
                self.info(
                    f"--no-calibrate: loaded stored "
                    f"{self._store_section}/{self._store_set}",
                )
                return
            self.warn(
                f"--no-calibrate but no stored data for "
                f"{self._store_section}/{self._store_set} — running calibration",
            )

        await self._run_setup(robot)

        while True:
            calibration = await self._collect(robot)
            if calibration is None:
                continue

            confirmed, calibration = await self._confirm(robot, calibration)
            if confirmed:
                self._apply(robot, calibration)
                self._store.store(
                    self._store_section,
                    self._serialize(calibration),
                    self._store_set,
                )
                return

            await self._run_setup(robot)
