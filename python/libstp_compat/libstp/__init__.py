"""Compatibility shim: ``libstp`` has been renamed to ``raccoon``.

Importing ``libstp`` (or any submodule) emits a :class:`DeprecationWarning`
and transparently resolves to the corresponding ``raccoon`` module. Existing
code that still does ``from libstp.step import seq`` keeps working while you
migrate; ``libstp.X`` is the same module object as ``raccoon.X``, so
``isinstance`` checks against classes from either name continue to work.

Update your imports to use ``raccoon`` directly. This shim will be removed
in a future release.
"""
from __future__ import annotations

import importlib
import pkgutil
import sys
import warnings

_OLD = "libstp"
_NEW = "raccoon"

warnings.warn(
    "'libstp' has been renamed to 'raccoon'. Please update your imports; "
    "the libstp compatibility shim will be removed in a future release.",
    DeprecationWarning,
    stacklevel=2,
)

_raccoon = importlib.import_module(_NEW)


def _alias_recursive(pkg, old_name: str, new_name: str) -> None:
    """Mirror ``pkg`` (a raccoon module) into sys.modules under ``old_name``.

    Walks ``pkg``'s subpackages with :func:`pkgutil.iter_modules` and recurses
    so every successfully-loaded raccoon submodule is also reachable via the
    ``libstp.*`` name. Modules that fail to import are silently skipped — they
    will fall through to the normal import machinery on demand.
    """
    sys.modules[old_name] = pkg
    pkg_path = getattr(pkg, "__path__", None)
    if pkg_path is None:
        return
    for info in pkgutil.iter_modules(pkg_path):
        sub_new = f"{new_name}.{info.name}"
        sub_old = f"{old_name}.{info.name}"
        try:
            sub_pkg = importlib.import_module(sub_new)
        except Exception:  # noqa: BLE001 — best-effort alias, skip failures
            continue
        _alias_recursive(sub_pkg, sub_old, sub_new)


_alias_recursive(_raccoon, _OLD, _NEW)
