from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Optional, Type, TypeVar, Any
from .base import Step


@dataclass(frozen=True)
class DslMeta:
    hidden: bool = False
    name: Optional[str] = None

T = TypeVar("T", bound=Type[Step])

def dsl(_cls: Optional[T] = None, *, hidden: bool = False, name: Optional[str] = None) -> Callable[[T], T] | T:
    """
    Usage:
      @dsl
      class X(Step): ...

      @dsl(hidden=True, name="TurnLeft")
      class Y(Step): ...
    """
    def wrap(cls: T) -> T:
        if not isinstance(cls, type):
            raise TypeError("@dsl can only decorate classes")
        if not issubclass(cls, Step):
            raise TypeError(f"@dsl requires the class to extend Step; got {cls.__name__}")

        # attach metadata to the class (simple + discoverable)
        setattr(cls, "__dsl__", DslMeta(hidden=hidden, name=name))
        setattr(cls, "__dsl_hidden__", hidden)
        setattr(cls, "__dsl_name__", name or cls.__name__)
        return cls

    # Support both @dsl and @dsl(...)
    return wrap(_cls) if _cls is not None else wrap