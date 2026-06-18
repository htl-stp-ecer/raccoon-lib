"""Pass contract primitives — segment representation + extended-contract defaults.

This module is intentionally neutral: it imports nothing from ``optimize.py``
or from any sibling pass, so passes can declare ``requires`` / ``produces`` /
``terminal`` against :class:`Representation` without a circular import.

A pass may optionally declare ``requires`` / ``produces`` (instances of
:class:`Representation`) and ``terminal`` (a bool) as class attributes.  Passes
that don't declare these behave as ``EITHER`` / ``SAME`` / non-terminal via the
``DEFAULT_*`` constants below.
"""

from __future__ import annotations

from enum import Enum


class Representation(Enum):
    """Segment representation a pass consumes or produces.

    - ``RELATIVE`` — body-frame deltas (the default stream representation).
    - ``ABSOLUTE`` — world-frame waypoints.
    - ``EITHER``   — a pass that accepts whatever representation is current.
    - ``SAME``     — a pass that leaves the representation unchanged.
    """

    RELATIVE = "relative"
    ABSOLUTE = "absolute"
    EITHER = "either"
    SAME = "same"


# Module-level defaults for the (optional) extended pass contract.  A pass may
# declare ``requires`` / ``produces`` / ``terminal`` as class attributes; if it
# doesn't, these defaults apply.  Passes therefore behave as
# EITHER / SAME / non-terminal unless they opt in.
DEFAULT_REQUIRES = Representation.EITHER
DEFAULT_PRODUCES = Representation.SAME
DEFAULT_TERMINAL = False
