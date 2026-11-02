"""Transport contract check at import time.

Runs once when ``import raccoon`` happens. On a Raccoon device, a transport that
does not match the bundle makes the import fail with a clear message (it ends up
in the user's run log) instead of the library silently talking past the other
components. Everywhere else (laptop, simulation, ``raccoon run --local``) the
check is skipped. Semantics: ``contract/CHECK_SPEC.md`` in raccoon-transport.

This module deliberately imports nothing from the compiled parts of raccoon.
"""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version


def _own_version() -> str:
    for dist in ("raccoon-library", "raccoon-sim"):
        try:
            return version(dist)
        except PackageNotFoundError:
            continue
    return "dev"


def check_transport_contract(component_version: str | None = None):
    """Run and record the check. Returns the result, or None for a transport without the check.

    Raises ``raccoon_transport.contract_check.ContractError`` (a ``RuntimeError``) when
    this installation must not run.
    """
    try:
        from raccoon_transport import contract_check
    except ImportError:  # transport older than the contract check
        return None
    return contract_check.startup_check(
        "raccoon-lib",
        component_version or _own_version(),
        persistent=False,  # a run-scoped process, not a service
    )
