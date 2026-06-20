"""Virtual simulated-time event loop for the headless mock sim.

Env-gated (``RACCOON_SIM_FASTTIME=1``, OFF by default). The mock sim normally
auto-ticks on WALL-CLOCK time, so a mission that takes ~45 s of robot time takes
~45 s of real time (and parallel runs slow each other into spurious 45 s
timeouts). When fast time is enabled, asyncio's clock becomes VIRTUAL: instead
of really sleeping, the event loop jumps its clock forward to the next scheduled
callback and advances the sim (``raccoon.sim.mock.tick``) by exactly that delta.
A 45 s-robot-time mission then finishes in ~1-2 s of wall time, deterministically
(fixed per-cycle dt), and the real-time ``asyncio.wait_for`` timeout never fires.

Only the headless mock sim is affected — this module ticks ``raccoon.sim.mock``
and is only imported/enabled by the sim test harnesses. Real-robot code never
sets ``RACCOON_SIM_FASTTIME`` and never imports this. Call :func:`enable` once
BEFORE ``asyncio.run`` (it installs an event-loop policy), and make sure the
sim is configured with ``auto_tick=False`` under fast time (``configure`` does
this automatically) so the C++ wall-clock auto-tick doesn't double-count.

Usage::

    RACCOON_SIM_FASTTIME=1 .venv-test/bin/python tools/optimize_validate.py --config merge
"""

from __future__ import annotations

import asyncio
import os

# Cap per individual sim tick so a long virtual jump integrates in small steps,
# matching the wall-clock auto-tick's max-step behaviour (avoids one giant Euler
# step when a step sleeps for, say, a full second).
_SUBSTEP_S = 0.05

# Sim tick callable, populated by enable(). A list holds it so the loop can read
# it without a module-global rebind race.
_TICK: list = [None]
_HAS_SIM: list = [None]
# Current virtual time in seconds, mirrored from the running loop so the patched
# time.monotonic() (see enable) can expose it to wall-clock-based stop
# conditions (after_seconds, on_analog_flank's sim fallback, stall detectors).
_VCLOCK: list = [0.0]
_enabled = False


def is_fast() -> bool:
    """True when fast simulated-time mode is requested via the environment."""
    return os.environ.get("RACCOON_SIM_FASTTIME") == "1"


class _VirtualTimeSelector:
    """Wraps the loop's real selector.

    Converts a positive idle wait (the loop blocking until the next timer)
    into a virtual-time jump that advances the clock + ticks the sim, instead
    of really blocking. Ready I/O (timeout 0 / None) is delegated unchanged.
    """

    def __init__(self, real, loop: "_FastLoop") -> None:
        self._real = real
        self._loop = loop

    def select(self, timeout):
        if timeout is not None and timeout > 0:
            self._loop._advance_virtual(timeout)
            timeout = 0  # the next timer is now "due" in virtual time
        return self._real.select(timeout)

    # --- delegate the rest of the selector protocol unchanged ---
    def register(self, *a, **k):
        return self._real.register(*a, **k)

    def unregister(self, *a, **k):
        return self._real.unregister(*a, **k)

    def modify(self, *a, **k):
        return self._real.modify(*a, **k)

    def get_map(self):
        return self._real.get_map()

    def get_key(self, *a, **k):
        return self._real.get_key(*a, **k)

    def close(self):
        return self._real.close()


class _FastLoop(asyncio.SelectorEventLoop):
    """SelectorEventLoop whose clock is virtual; idle waits tick the sim."""

    def __init__(self) -> None:
        super().__init__()
        self._vtime = 0.0
        self._selector = _VirtualTimeSelector(self._selector, self)

    def time(self) -> float:
        return self._vtime

    def _advance_virtual(self, dt: float) -> None:
        self._vtime += dt
        _VCLOCK[0] = self._vtime
        tick = _TICK[0]
        has_sim = _HAS_SIM[0]
        if tick is None or (has_sim is not None and not has_sim()):
            return
        remaining = dt
        while remaining > 1e-9:
            step = remaining if remaining < _SUBSTEP_S else _SUBSTEP_S
            tick(step)
            remaining -= step


class _FastPolicy(asyncio.DefaultEventLoopPolicy):
    def new_event_loop(self):
        return _FastLoop()


def enable() -> bool:
    """Install the virtual-time loop policy if fast mode is requested.

    Idempotent and a no-op unless ``RACCOON_SIM_FASTTIME=1``. Returns whether
    fast mode is now active. Must be called before ``asyncio.run``.
    """
    global _enabled
    if _enabled:
        return True
    if not is_fast():
        return False
    from raccoon import sim

    _TICK[0] = sim.mock.tick
    _HAS_SIM[0] = sim.mock.has_sim

    # The per-step timing tracker records into SQLite via aiosqlite, which runs
    # the DB on a real background thread. Under virtual time the loop jumps over
    # the real time that thread needs, so the DB future never resolves and the
    # step's finally hangs until the wait_for timeout. Timing is purely
    # observational (it never affects control), so disable it under fast time —
    # this also removes per-step DB overhead. (Note: LIBSTP_TIMING_ENABLED is
    # not actually read by the tracker, so this is the real off switch.)
    try:
        from raccoon.timing.tracker import StepTimingTracker

        StepTimingTracker.get_instance().config.enabled = False
    except Exception:  # noqa: BLE001 — best effort; timing module optional
        pass

    # Stop conditions that wait on wall time (after_seconds, on_analog_flank's
    # sim fallback, stall detectors) call time.monotonic(). Under virtual time
    # real wall time is frozen, so they'd never fire and the mission would hang
    # to the timeout. Point time.monotonic() at the virtual clock so they fire
    # in simulated seconds instead. (condition.py / the cube-bot flank use
    # `import time; time.monotonic()`, so patching the module attribute works.)
    import time

    time.monotonic = lambda: _VCLOCK[0]

    asyncio.set_event_loop_policy(_FastPolicy())
    _enabled = True
    return True
