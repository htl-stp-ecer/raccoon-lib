"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: run_supervised.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .run_supervised import RunSupervised


class RunSupervisedBuilder(StepBuilder):
    """Builder for RunSupervised. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._step = _UNSET
        self._title = "Automatic Step"
        self._text = "Robot is moving…"
        self._done_text = "Step finished."
        self._confirm_label = "Confirm"
        self._retry_label = "Retry"
        self._cancel_label = "Cancel"

    def step(self, value: "Step | Callable[[], Step]"):
        self._step = value
        return self

    def title(self, value: str):
        self._title = value
        return self

    def text(self, value: str):
        self._text = value
        return self

    def done_text(self, value: str):
        self._done_text = value
        return self

    def confirm_label(self, value: str):
        self._confirm_label = value
        return self

    def retry_label(self, value: str):
        self._retry_label = value
        return self

    def cancel_label(self, value: str):
        self._cancel_label = value
        return self

    def _build(self):
        kwargs = {}
        if self._step is not _UNSET:
            kwargs["step"] = self._step
        kwargs["title"] = self._title
        kwargs["text"] = self._text
        kwargs["done_text"] = self._done_text
        kwargs["confirm_label"] = self._confirm_label
        kwargs["retry_label"] = self._retry_label
        kwargs["cancel_label"] = self._cancel_label
        return RunSupervised(**kwargs)


@dsl(tags=["ui", "control"])
def run_supervised(
    step: "Step | Callable[[], Step]" = _UNSET,
    title: str = "Automatic Step",
    text: str = "Robot is moving…",
    done_text: str = "Step finished.",
    confirm_label: str = "Confirm",
    retry_label: str = "Retry",
    cancel_label: str = "Cancel",
):
    """
    Run a step under operator supervision with Cancel, Retry and Confirm.

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

    Returns:
        A RunSupervisedBuilder (chainable via ``.step()``, ``.title()``, ``.text()``, ``.done_text()``, ``.confirm_label()``, ``.retry_label()``, ``.cancel_label()``, ``.on_anomaly()``, ``.skip_timing()``).

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
    b = RunSupervisedBuilder()
    if step is not _UNSET:
        b._step = step
    b._title = title
    b._text = text
    b._done_text = done_text
    b._confirm_label = confirm_label
    b._retry_label = retry_label
    b._cancel_label = cancel_label
    return b


__all__ = ["RunSupervisedBuilder", "run_supervised"]
