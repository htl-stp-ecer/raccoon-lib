"""Environment-gated step execution.

A ``run_if_env(step, var, ...)`` call runs the wrapped step only when an
environment variable matches (or does not match) an expected value. This
is the generic building block behind the convenience guards
``run_unless_no_calibrate``, ``run_unless_no_checkpoints``,
``run_if_debug`` and ``run_if_dev``, which map onto the ``LIBSTP_*``
flags set by ``raccoon run`` (``--no-calibrate``, ``--no-checkpoints``,
``--debug``, ``--dev``).

The environment is read at execution time, not when the step tree is
built, so it reflects the flags the run was actually launched with.
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from .. import Step, StepProtocol
from ..annotation import dsl

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl(hidden=True)
class RunIfEnv(Step):
    """Run a wrapped step only when an environment variable gate passes.

    The gate is ``os.environ.get(var) == equals`` (so ``equals=None``
    means "the variable is unset"), optionally inverted by ``negate``.
    When the gate passes the wrapped step runs normally; otherwise it is
    skipped and a log line is emitted.
    """

    _composite = True

    def __init__(
        self,
        step: StepProtocol,
        var: str,
        equals: str | None = "1",
        *,
        negate: bool = False,
    ) -> None:
        super().__init__()

        if not isinstance(step, StepProtocol):
            msg = f"step must be a Step, got {type(step).__name__}"
            raise TypeError(msg)
        if not var:
            msg = "var must be a non-empty environment variable name"
            raise ValueError(msg)

        self._step = step.resolve()
        self._var = var
        self._equals = equals
        self._negate = negate

    def collected_resources(self) -> frozenset[str]:
        # Conservatively report the wrapped step's resources: it may run,
        # so surrounding Parallel/DoWhileActive validation must account
        # for the resources it would claim.
        return self._step.collected_resources()

    def _gate_open(self) -> bool:
        matches = os.environ.get(self._var) == self._equals
        return (not matches) if self._negate else matches

    def _gate_description(self) -> str:
        op = "!=" if self._negate else "=="
        return f"{self._var} {op} {self._equals!r}"

    def _generate_signature(self) -> str:
        return f"RunIfEnv({self._gate_description()}, step={self._step._generate_signature()})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        if self._gate_open():
            await self._step.run_step(robot)
        else:
            self.info(
                f"Skipping {self._step._generate_signature()} "
                f"(gate closed: {self._gate_description()})"
            )


@dsl(tags=["control", "env"])
def run_if_env(
    step: StepProtocol,
    var: str,
    equals: str | None = "1",
    negate: bool = False,
) -> RunIfEnv:
    """Run a step only when an environment variable matches a value.

    Evaluates ``os.environ.get(var) == equals`` at execution time and
    runs *step* when the result is truthy (or falsy, if ``negate`` is
    set). Use ``equals=None`` to gate on the variable being **unset**.
    The environment is read when the step runs, so it reflects the
    ``LIBSTP_*`` flags ``raccoon run`` was launched with.

    This is the generic primitive; for the common flags prefer the
    named guards ``run_unless_no_calibrate``, ``run_unless_no_checkpoints``,
    ``run_if_debug`` and ``run_if_dev``.

    Args:
        step: The step to execute when the gate is open.
        var: Name of the environment variable to inspect.
        equals: Expected value. The gate matches when the variable equals
            this string. Pass ``None`` to match when the variable is
            unset. Defaults to ``"1"``.
        negate: Invert the gate — run *step* when the variable does **not**
            match ``equals``. Defaults to ``False``.

    Returns:
        A RunIfEnv step wrapping *step*.

    Example::

        from raccoon.step.logic import run_if_env
        from raccoon.step.motion import drive_forward

        # Only drive when MY_FLAG is set to "1"
        run_if_env(drive_forward(20), "MY_FLAG")

        # Only run when DEMO_MODE is unset
        run_if_env(drive_forward(20), "DEMO_MODE", equals=None)
    """
    return RunIfEnv(step, var, equals=equals, negate=negate)


@dsl(tags=["control", "env"])
def run_unless_no_calibrate(step: StepProtocol) -> RunIfEnv:
    """Run a step unless calibration was skipped (``--no-calibrate``).

    Runs *step* only when ``LIBSTP_NO_CALIBRATE`` is **not** set to
    ``"1"`` — i.e. when ``raccoon run`` was launched without
    ``--no-calibrate``. Use this to wrap interactive calibration that
    should be bypassed on fast iteration runs.

    Args:
        step: The calibration step to run when calibration is enabled.

    Returns:
        A RunIfEnv step wrapping *step*.

    Example::

        from raccoon.step.logic import run_unless_no_calibrate

        run_unless_no_calibrate(calibrate_line_sensor())
    """
    return RunIfEnv(step, "LIBSTP_NO_CALIBRATE", equals="1", negate=True)


@dsl(tags=["control", "env"])
def run_unless_no_checkpoints(step: StepProtocol) -> RunIfEnv:
    """Run a step unless checkpoint waits were disabled (``--no-checkpoints``).

    Runs *step* only when ``LIBSTP_NO_CHECKPOINTS`` is **not** set to
    ``"1"`` — i.e. when ``raccoon run`` was launched without
    ``--no-checkpoints``.

    Args:
        step: The step to run when checkpoint timing is active.

    Returns:
        A RunIfEnv step wrapping *step*.

    Example::

        from raccoon.step.logic import run_unless_no_checkpoints

        run_unless_no_checkpoints(wait_for_checkpoint("phase-2"))
    """
    return RunIfEnv(step, "LIBSTP_NO_CHECKPOINTS", equals="1", negate=True)


@dsl(tags=["control", "env"])
def run_if_debug(step: StepProtocol) -> RunIfEnv:
    """Run a step only in debug mode (``--debug``).

    Runs *step* only when ``LIBSTP_DEBUG`` is set to ``"1"`` — i.e. when
    ``raccoon run`` was launched with ``--debug``. Use this to gate
    diagnostics, extra logging, or interactive pauses that should not
    affect normal competition runs.

    Args:
        step: The step to run when debug mode is active.

    Returns:
        A RunIfEnv step wrapping *step*.

    Example::

        from raccoon.step.logic import run_if_debug

        run_if_debug(wait_for_button("Inspect arm position"))
    """
    return RunIfEnv(step, "LIBSTP_DEBUG", equals="1")


@dsl(tags=["control", "env"])
def run_if_dev(step: StepProtocol) -> RunIfEnv:
    """Run a step only in dev mode (``--dev``).

    Runs *step* only when ``LIBSTP_DEV_MODE`` is set to ``"1"`` — i.e.
    when ``raccoon run`` was launched with ``--dev``.

    Args:
        step: The step to run when dev mode is active.

    Returns:
        A RunIfEnv step wrapping *step*.

    Example::

        from raccoon.step.logic import run_if_dev

        run_if_dev(drive_forward(5))  # short dev-only sanity hop
    """
    return RunIfEnv(step, "LIBSTP_DEV_MODE", equals="1")
