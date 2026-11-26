"""Check ahead of time that the heading reference is marked before it is used.

The step trees of all missions are walked in execution order before any of them
runs. A step that needs the heading reference while nothing before it can have
set it is reported immediately, so the robot does not stop in the middle of a
run.

The check only reports certain failures. Steps whose behaviour is unknown ahead
of time (``defer``, ``run``, steps defined outside of ``raccoon``) may set the
reference, so nothing after them is reported.
"""

from __future__ import annotations

from collections.abc import Iterator

from .base import Step
from .condition import StopCondition

NO_REFERENCE_MESSAGE = "No valid heading reference. Call mark_heading_reference() first."


class HeadingReferenceChecker:
    """Walk step trees in execution order and track the heading reference.

    Use one instance for a whole run and pass it the root steps in the order
    they execute.
    """

    def __init__(self) -> None:
        self._may_be_marked = False

    def assume_marked(self) -> None:
        """Record that unknown code may have set the reference from here on."""
        self._may_be_marked = True

    def check(self, root: Step, context: str) -> None:
        """Check one step tree.

        Args:
            root: Root step of the tree.
            context: Where the tree comes from, used in the error message.

        Raises:
            RuntimeError: If a step needs the heading reference before anything
                can have set it.
        """
        step = root.resolve()
        if self._may_be_marked:
            return
        if _is_opaque(step) or step.sets_heading_reference():
            self.assume_marked()
            return
        if step.requires_heading_reference() or any(
            condition.requires_heading_reference() for condition in _conditions(step)
        ):
            msg = f"{NO_REFERENCE_MESSAGE} (needed by {step._generate_signature()} in {context})"
            raise RuntimeError(msg)

        children = [child.resolve() for child in step.child_steps()]
        if step._children_run_concurrently and any(_may_set(child) for child in children):
            self.assume_marked()
        for child in children:
            self.check(child, context)


def _is_opaque(step: Step) -> bool:
    return step._opaque_to_checks or not type(step).__module__.startswith("raccoon.")


def _may_set(step: Step) -> bool:
    if _is_opaque(step) or step.sets_heading_reference():
        return True
    return any(_may_set(child.resolve()) for child in step.child_steps())


def _conditions(step: Step) -> Iterator[StopCondition]:
    return (value for value in vars(step).values() if isinstance(value, StopCondition))
