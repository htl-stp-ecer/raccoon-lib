"""Transport contract check and library version check at import time.

Runs once when ``import raccoon`` happens. On a Raccoon device, a transport that
does not match the bundle, or a library version that is not the bundle's, makes the
import fail with a clear message (it ends up in the user's run log) instead of the
library silently talking past the other components. Everywhere else (laptop,
simulation, ``raccoon run --local``) the check is skipped.

The transport check is ``contract/CHECK_SPEC.md`` in raccoon-transport. The version
check is this component's own (``contract/COMPONENT_VERSION_CHECK.md`` says what every
component does); it is handed to the startup check as a component check.

This module deliberately imports nothing from the compiled parts of raccoon.
"""

from __future__ import annotations

import json
import os
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

BUNDLE_KEY = "raccoon-lib"
_DEFAULT_BUNDLE_FILE = "/etc/raccoon-bundle.json"


def _own_version() -> str:
    for dist in ("raccoon-library", "raccoon-sim"):
        try:
            return version(dist)
        except PackageNotFoundError:
            continue
    return "dev"


def _normalize(value: str) -> str:
    """``v173`` and ``173`` are the same version; nothing else is normalised."""
    value = value.strip()
    return value[1:] if value[:1] in ("v", "V") else value


def version_check(component_version: str, bundle_file: str | None = None, component_check_cls=None):
    """This library's version against its ``raccoon-lib`` entry in the bundle file.

    Returns None when the bundle file cannot be read (the transport check reports that).
    """
    if component_check_cls is None:
        from raccoon_transport.contract_check import ComponentCheck as component_check_cls
    path = bundle_file or os.environ.get("RACCOON_BUNDLE_FILE") or _DEFAULT_BUNDLE_FILE
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if not isinstance(document, dict):
        return None
    components = document.get("components")
    entry = components.get(BUNDLE_KEY) if isinstance(components, dict) else None
    bundle = document.get("bundle") if isinstance(document.get("bundle"), str) else None

    def result(outcome, message, reason=None, expected=None):
        return component_check_cls(name="version", result=outcome, message=message, reason=reason, expected=expected)

    if not isinstance(entry, str):
        return result("bundle-fehler", f"bundle check failed: bundle file has no version for {BUNDLE_KEY}: {path}",
                      "version-field-missing")
    expected = entry.strip()
    if not expected:
        return result("bundle-fehler", f"bundle check failed: bundle file has an empty version for {BUNDLE_KEY}: {path}",
                      "version-field-empty")
    if expected == "dev":
        return result(
            "dev-warnung",
            f"component version check omitted ('{BUNDLE_KEY}' is 'dev' in the bundle): "
            f"raccoon-lib {component_version or 'unknown'} is not compared; mismatches are not enforced",
            "version-omitted", expected)
    if _normalize(expected) != _normalize(component_version):
        return result(
            "version-mismatch",
            f"component version mismatch: raccoon-lib is {component_version or 'unknown'} but bundle "
            f"{bundle or 'unknown'} expects {expected}; update or reflash so all components match",
            None, expected)
    return result("ok", f"version ok: {component_version}", None, expected)


def check_transport_contract(component_version: str | None = None):
    """Run and record the check. Returns the result, or None for a transport without the check.

    Raises ``raccoon_transport.contract_check.ContractError`` (a ``RuntimeError``) when
    this installation must not run.
    """
    try:
        from raccoon_transport import contract_check
    except ImportError:  # transport older than the contract check
        return None
    own_version = component_version or _own_version()
    return contract_check.startup_check(
        "raccoon-lib",
        own_version,
        persistent=False,  # a run-scoped process, not a service
        component_check=version_check(own_version, component_check_cls=contract_check.ComponentCheck),
    )
