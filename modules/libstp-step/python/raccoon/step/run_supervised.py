"""Run a step under operator supervision with a Cancel / Retry / Confirm UI."""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from contextlib import suppress
from typing import TYPE_CHECKING

from raccoon.ui import UIScreen, UIStep, on_click
from raccoon.ui.widgets import (
    Button,
    Center,
    Column,
    HintBox,
    ProgressSpinner,
    Row,
    Spacer,
    StatusIcon,
    Text,
    Widget,
)

from .annotation import dsl_step
from .base import Step
from .model import StepProtocol

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

# Result screen outcomes
_DONE = "done"
_CANCELLED = "cancelled"
_ERROR = "error"


class _SupervisedRunningScreen(UIScreen[str]):
    """Shown while the inner step runs — spinner plus a Cancel button."""

    def __init__(self, title: str, text: str, cancel_label: str) -> None:
        super().__init__()
        self.title = title
        self._text = text
        self._cancel_label = cancel_label

    def build(self) -> Widget:
        return Center(
            children=[
                Column(
                    children=[
                        Row(
                            children=[
                                ProgressSpinner(size=28),
                                Spacer(12),
                                Text(self._text, size="large"),
                            ],
                            align="center",
                        ),
                        Spacer(24),
                        HintBox("Do not touch the robot", icon="pan_tool"),
                        Spacer(24),
                        Button("cancel", self._cancel_label, style="warning", icon="close"),
                    ],
                    align="center",
                ),
            ]
        )

    @on_click("cancel")
    async def on_cancel(self) -> None:
        self.close("cancel")


class _SupervisedResultScreen(UIScreen[str]):
    """Shown after the inner step finished — Retry or Confirm."""

    _primary_button_id = "confirm"

    def __init__(
        self,
        title: str,
        outcome: str,
        text: str,
        confirm_label: str,
        retry_label: str,
    ) -> None:
        super().__init__()
        self.title = title
        self._outcome = outcome
        self._text = text
        self._confirm_label = confirm_label
        self._retry_label = retry_label

    def build(self) -> Widget:
        icon, color, status = {
            _DONE: ("check", "green", "Finished"),
            _CANCELLED: ("block", "orange", "Cancelled"),
            _ERROR: ("warning", "red", "Failed"),
        }[self._outcome]

        return Center(
            children=[
                Column(
                    children=[
                        Row(
                            children=[
                                StatusIcon(icon=icon, color=color),
                                Spacer(12),
                                Text(status, size="title", bold=True),
                            ],
                            align="center",
                        ),
                        Spacer(16),
                        Text(self._text, size="large", align="center"),
                        Spacer(24),
                        Row(
                            children=[
                                Button("retry", self._retry_label, style="secondary", icon="refresh"),
                                Button(
                                    "confirm",
                                    self._confirm_label,
                                    style="success" if self._outcome == _DONE else "warning",
                                    icon="check",
                                ),
                            ],
                            align="center",
                            spacing=16,
                        ),
                    ],
                    align="center",
                ),
            ]
        )

    @on_click("confirm")
    async def on_confirm(self) -> None:
        self.close("confirm")

    @on_click("retry")
    async def on_retry(self) -> None:
        self.close("retry")


@dsl_step(tags=["ui", "control"])
class RunSupervised(UIStep):
    """Run a step under operator supervision with Cancel, Retry and Confirm.

    While the supervised step runs, the screen shows a spinner and a cancel
    button. Pressing it cancels the step; its ``finally`` blocks run, so a
    motion step hard-stops the motors. Once the step has completed, been
    cancelled or raised an exception, the screen shows the outcome with a
    retry and a confirm button. Retry runs the step again, confirm continues
    the mission. The physical button acts as confirm. An exception from the
    supervised step is logged and shown on the result screen; it does not
    abort the mission.

    Pass either a step or a zero-argument factory returning one. A factory is
    called for every attempt (and once at construction to determine the
    required resources), so each retry starts with freshly built steps. Use a
    factory for composite sequences.

    Prerequisites:
        The supervised step must not show a UI screen of its own, because
        only one screen can be active at a time.

    Args:
        step: The step to supervise, or a zero-argument factory returning it.
        title: Title of the running and the result screen.
        text: Message shown while the step runs.
        done_text: Message on the result screen after a successful run.
        confirm_label: Label of the confirm button.
        retry_label: Label of the retry button.
        cancel_label: Label of the cancel button.

    Raises:
        TypeError: If ``step`` is neither a step nor a factory returning one.

    Example::

        from raccoon.step import run_supervised, seq
        from raccoon.step.motion import drive_forward, turn_right

        def move_into_position():
            return seq([drive_forward(30), turn_right(90)])

        run_supervised(
            move_into_position,
            title="Start position",
            text="Driving into the start position...",
        )
    """

    _composite = True

    def __init__(
        self,
        step: "Step | Callable[[], Step]",
        title: str = "Automatic Step",
        text: str = "Robot is moving…",
        done_text: str = "Step finished.",
        confirm_label: str = "Confirm",
        retry_label: str = "Retry",
        cancel_label: str = "Cancel",
    ) -> None:
        super().__init__()
        if isinstance(step, StepProtocol):
            self._factory: Callable[[], Step] = lambda: step
            probe = step
        elif callable(step):
            self._factory = step
            probe = step()
            if not isinstance(probe, StepProtocol):
                msg = f"factory must return a Step, got {type(probe)}"
                raise TypeError(msg)
        else:
            msg = f"Expected a Step or a factory returning one, got {type(step)}"
            raise TypeError(msg)
        # Resolved once for resource accounting / signature only.
        self._probe = probe.resolve()
        self._title = title
        self._text = text
        self._done_text = done_text
        self._confirm_label = confirm_label
        self._retry_label = retry_label
        self._cancel_label = cancel_label

    def collected_resources(self) -> frozenset[str]:
        return self._probe.collected_resources()

    def _generate_signature(self) -> str:
        return f"RunSupervised(step={self._probe.__class__.__name__}, title={self._title!r})"

    async def _run_inner_once(self, robot: "GenericRobot") -> tuple[str, str]:
        """Run the inner step once under the running screen.

        Returns ``(outcome, detail_text)`` for the result screen.
        """
        inner = self._factory().resolve()
        screen = _SupervisedRunningScreen(self._title, self._text, self._cancel_label)
        await self.display(screen)

        task = asyncio.create_task(inner.run_step(robot))
        cancelled_by_user = False
        try:
            while not task.done():
                await self.pump_events()
                if screen._closed:  # Cancel pressed
                    cancelled_by_user = True
                    self.warn(f"{inner.__class__.__name__} cancelled by operator")
                    task.cancel()
                    break
                await asyncio.sleep(0.05)

            try:
                await task
            except asyncio.CancelledError:
                current = asyncio.current_task()
                if not cancelled_by_user or (current is not None and current.cancelling()):
                    raise  # outer cancellation (mission abort) — propagate
                return _CANCELLED, "Cancelled — reposition the robot, then retry."
            except Exception as e:  # noqa: BLE001 — surfaced on the result screen
                self.error(f"{inner.__class__.__name__} failed: {e!r}")
                return _ERROR, f"{type(e).__name__}: {e}"
            return _DONE, self._done_text
        finally:
            # Mission abort while the inner task is still running: make sure
            # the inner step is cancelled so its finally blocks stop the motors.
            if not task.done():
                task.cancel()
                with suppress(asyncio.CancelledError):
                    await task
            await self.close_ui()

    async def _execute_step(self, robot: "GenericRobot") -> None:
        while True:
            outcome, detail = await self._run_inner_once(robot)

            choice = await self.show(
                _SupervisedResultScreen(
                    title=self._title,
                    outcome=outcome,
                    text=detail,
                    confirm_label=self._confirm_label,
                    retry_label=self._retry_label,
                )
            )
            if choice != "retry":
                return
            # retry → loop: factory rebuilds the inner step from scratch
