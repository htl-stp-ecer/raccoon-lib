from __future__ import annotations

import inspect
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import wraps
from typing import Any, TypeVar, get_type_hints, overload

from .base import Step
from .model import StepProtocol


@dataclass(frozen=True)
class DslMeta:
    """Discovery metadata attached by ``@dsl`` to steps and factories."""

    hidden: bool = False
    name: str | None = None
    tags: tuple[str, ...] = field(default_factory=tuple)


T = TypeVar("T", bound=type[Step])
F = TypeVar("F", bound=Callable[..., StepProtocol])


@overload
def dsl_step(cls: T) -> T: ...
@overload
def dsl_step(
    cls: None = None,
    *,
    tags: list[str] | tuple[str, ...] | None = None,
) -> Callable[[T], T]: ...


def dsl_step(
    cls: T | None = None,
    *,
    tags: list[str] | tuple[str, ...] | None = None,
) -> T | Callable[[T], T]:
    """Mark a Step class for builder + DSL function generation.

    Codegen scans for classes decorated with ``@dsl_step``,
    introspects their ``__init__`` parameters, and generates:

    - A ``<ClassName>Builder(StepBuilder)`` with fluent setter methods
    - A ``snake_case()`` factory function decorated with ``@dsl(tags=...)``

    The marker also applies ``@dsl(hidden=True)`` so the raw class is
    hidden from the UI (users interact via the generated DSL function).

    Usage::

        @dsl_step(tags=["motion", "drive"])
        class DriveForward(MotionStep):
            def __init__(self, cm: float = None, speed: float = 1.0, until: StopCondition = None): ...

    Codegen produces ``DriveForwardBuilder`` and ``drive_forward()``.
    Tags are propagated to the generated ``@dsl(tags=...)`` factory.
    """
    tags_tuple = tuple(tags) if tags else ()

    def _apply(cls: T) -> T:
        if not isinstance(cls, type) or not issubclass(cls, Step):
            msg = f"@dsl_step requires a Step subclass; got {cls}"
            raise TypeError(msg)

        # Mark for codegen discovery
        cls.__dsl_step__ = True
        cls.__dsl_step_tags__ = tags_tuple

        # Also apply @dsl(hidden=True) — the generated function is the public API
        meta = DslMeta(hidden=True, name=cls.__name__, tags=tags_tuple)
        cls.__dsl__ = meta
        cls.__dsl_hidden__ = True
        cls.__dsl_name__ = cls.__name__
        cls.__dsl_tags__ = tags_tuple

        return cls

    if cls is not None:
        # Bare @dsl_step without arguments
        return _apply(cls)
    return _apply


@overload
def dsl(_target: T) -> T: ...
@overload
def dsl(_target: F) -> F: ...
@overload
def dsl(
    _target: None = None,
    *,
    hidden: bool = False,
    name: str | None = None,
    tags: list[str] | tuple[str, ...] | None = None,
) -> Callable[[T | F], T | F]: ...


def dsl(
    _target: T | F | None = None,
    *,
    hidden: bool = False,
    name: str | None = None,
    tags: list[str] | tuple[str, ...] | None = None,
) -> Callable[[T | F], T | F] | T | F:
    """
    Mark a class or function as part of the DSL.

    Usage:
      @dsl
      class X(Step): ...

      @dsl(hidden=True, name="TurnLeft", tags=["movement", "rotation"])
      class Y(Step): ...

      @dsl(name="Wait", tags=["timing"])
      def wait_seconds(seconds: float) -> WaitStep:
          return WaitStep(seconds)

    Args:
        hidden: If True, hide from auto-discovery/UI.
        name: Custom display name (defaults to class/function name).
        tags: List of tags for grouping in web UI.
    """
    tags_tuple = tuple(tags) if tags else ()
    meta = DslMeta(hidden=hidden, name=name, tags=tags_tuple)

    def wrap_class(cls: T) -> T:
        if not issubclass(cls, Step):
            msg = f"@dsl requires the class to extend Step; got {cls.__name__}"
            raise TypeError(msg)

        cls.__dsl__ = meta
        cls.__dsl_hidden__ = hidden
        cls.__dsl_name__ = name or cls.__name__
        cls.__dsl_tags__ = tags_tuple
        return cls

    def wrap_function(fn: F) -> F:
        # Validate return type annotation if available
        try:
            hints = get_type_hints(fn) if hasattr(fn, "__annotations__") else {}
        except NameError:
            hints = {}
        hints.get("return")

        # We can't always verify at decoration time, but we attach metadata
        @wraps(fn)
        def wrapper(*args: Any, **kwargs: Any) -> StepProtocol:
            result = fn(*args, **kwargs)
            if not isinstance(result, StepProtocol):
                msg = (
                    f"@dsl function '{fn.__name__}' must return a StepProtocol; "
                    f"got {type(result).__name__}"
                )
                raise TypeError(msg)
            return result

        # Preserve the original signature so stubgen emits full typed params
        # instead of collapsing to (*args, **kwargs).
        wrapper.__signature__ = inspect.signature(fn)  # type: ignore[attr-defined]

        # Attach DSL metadata to the wrapper
        wrapper.__dsl__ = meta
        wrapper.__dsl_hidden__ = hidden
        wrapper.__dsl_name__ = name or fn.__name__
        wrapper.__dsl_tags__ = tags_tuple
        wrapper.__dsl_is_factory__ = True

        return wrapper  # type: ignore[return-value]

    def wrap(target: T | F) -> T | F:
        if isinstance(target, type):
            if issubclass(target, Step):
                return wrap_class(target)  # type: ignore[return-value]
            msg = f"@dsl requires the class to extend Step; got {target.__name__}"
            raise TypeError(msg)
        if callable(target):
            return wrap_function(target)  # type: ignore[return-value]
        msg = f"@dsl can only decorate classes or functions; got {type(target)}"
        raise TypeError(msg)

    # Support both @dsl and @dsl(...)
    return wrap(_target) if _target is not None else wrap
