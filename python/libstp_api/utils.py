from contextlib import contextmanager
from typing import Iterator

@contextmanager
def swallow_exceptions() -> Iterator[None]:
    try:
        yield
    except Exception:
        # In dev mode we do not propagate; callers decide when to print.
        pass