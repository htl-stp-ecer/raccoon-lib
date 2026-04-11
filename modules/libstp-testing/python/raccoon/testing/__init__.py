"""Testing helpers for raccoon-based projects.

This subpackage contains utilities intended for use from a project's
``tests/`` directory — nothing in here should be imported from production
mission code.

The top-level surface is intentionally small:

- :mod:`raccoon.testing.sim` — high-level sim wrapper (``use_scene``,
  ``SimRobotConfig``, ``pose``, etc.).
- :mod:`raccoon.testing.pytest_plugin` — the pytest plugin that exposes
  the ``robot``, ``scene``, and ``run_step`` fixtures. Auto-loaded via the
  ``pytest11`` entry point in the raccoon wheel; you usually don't need
  to import it directly.
"""
from __future__ import annotations

__all__: list[str] = []
