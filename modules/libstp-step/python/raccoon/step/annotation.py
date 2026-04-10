from __future__ import annotations
import inspect
from dataclasses import dataclass, field
from typing import Callable, Optional, Type, TypeVar, Any, Union, overload, get_type_hints
from functools import wraps
from .base import Step
from .model import StepProtocol


@dataclass(frozen=True)
class DslMeta:
    """Discovery metadata attached by ``@dsl`` to steps and factories."""

    hidden: bool = False
    name: Optional[str] = None
    tags: tuple[str, ...] = field(default_factory=tuple)


T = TypeVar("T", bound=Type[Step])
F = TypeVar("F", bound=Callable[..., StepProtocol])


@overload
def dsl_step(cls: T) -> T: ...
@overload
def dsl_step(
    cls: None = None,
    *,
    tags: Optional[list[str] | tuple[str, ...]] = None,
) -> Callable[[T], T]: ...


def dsl_step(
    cls: Optional[T] = None,
    *,
    tags: Optional[list[str] | tuple[str, ...]] = None,
) -> Union[T, Callable[[T], T]]:
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
            def __init__(self, cm: float = None, speed: float = 1.0,
                         until: StopCondition = None):
                ...

    Codegen produces ``DriveForwardBuilder`` and ``drive_forward()``.
    Tags are propagated to the generated ``@dsl(tags=...)`` factory.
    """
    tags_tuple = tuple(tags) if tags else ()

    def _apply(cls: T) -> T:
        if not isinstance(cls, type) or not issubclass(cls, Step):
            raise TypeError(f"@dsl_step requires a Step subclass; got {cls}")

        # Mark for codegen discovery
        setattr(cls, "__dsl_step__", True)
        setattr(cls, "__dsl_step_tags__", tags_tuple)

        # Also apply @dsl(hidden=True) — the generated function is the public API
        meta = DslMeta(hidden=True, name=cls.__name__, tags=tags_tuple)
        setattr(cls, "__dsl__", meta)
        setattr(cls, "__dsl_hidden__", True)
        setattr(cls, "__dsl_name__", cls.__name__)
        setattr(cls, "__dsl_tags__", tags_tuple)

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
    name: Optional[str] = None,
    tags: Optional[list[str] | tuple[str, ...]] = None,
) -> Callable[[Union[T, F]], Union[T, F]]: ...


def dsl(
    _target: Optional[Union[T, F]] = None,
    *,
    hidden: bool = False,
    name: Optional[str] = None,
    tags: Optional[list[str] | tuple[str, ...]] = None,
) -> Union[Callable[[Union[T, F]], Union[T, F]], T, F]:
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
            raise TypeError(f"@dsl requires the class to extend Step; got {cls.__name__}")

        setattr(cls, "__dsl__", meta)
        setattr(cls, "__dsl_hidden__", hidden)
        setattr(cls, "__dsl_name__", name or cls.__name__)
        setattr(cls, "__dsl_tags__", tags_tuple)
        return cls

    def wrap_function(fn: F) -> F:
        # Validate return type annotation if available
        try:
            hints = get_type_hints(fn) if hasattr(fn, "__annotations__") else {}
        except NameError:
            hints = {}
        return_hint = hints.get("return")

        # We can't always verify at decoration time, but we attach metadata
        @wraps(fn)
        def wrapper(*args: Any, **kwargs: Any) -> StepProtocol:
            result = fn(*args, **kwargs)
            if not isinstance(result, StepProtocol):
                raise TypeError(
                    f"@dsl function '{fn.__name__}' must return a StepProtocol; "
                    f"got {type(result).__name__}"
                )
            return result

        # Preserve the original signature so stubgen emits full typed params
        # instead of collapsing to (*args, **kwargs).
        wrapper.__signature__ = inspect.signature(fn)  # type: ignore[attr-defined]

        # Attach DSL metadata to the wrapper
        setattr(wrapper, "__dsl__", meta)
        setattr(wrapper, "__dsl_hidden__", hidden)
        setattr(wrapper, "__dsl_name__", name or fn.__name__)
        setattr(wrapper, "__dsl_tags__", tags_tuple)
        setattr(wrapper, "__dsl_is_factory__", True)

        return wrapper  # type: ignore[return-value]

    def wrap(target: Union[T, F]) -> Union[T, F]:
        if isinstance(target, type):
            if issubclass(target, Step):
                return wrap_class(target)  # type: ignore[return-value]
            raise TypeError(f"@dsl requires the class to extend Step; got {target.__name__}")
        elif callable(target):
            return wrap_function(target)  # type: ignore[return-value]
        else:
            raise TypeError(f"@dsl can only decorate classes or functions; got {type(target)}")

    # Support both @dsl and @dsl(...)
    return wrap(_target) if _target is not None else wrap
