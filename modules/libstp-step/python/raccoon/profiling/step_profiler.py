"""Deep per-step runtime profiler for raccoon missions.

The mission step machinery wraps every user step in :meth:`Step.run_step`, which
does more than just run the step: it reads/writes the SQLite timing database,
acquires hardware-resource locks, generates signatures and logs.  When a robot
"stands still" between two steps, the standstill is this *overhead* — not the
step body — and until now there was no way to see where it actually goes.

``StepProfiler`` answers that by instrumenting the machinery at runtime (no edits
to the core).  For every step it records a phase breakdown::

    total = execute + db_read + db_write + acquire + (unattributed overhead)

and it runs an event-loop-lag monitor in parallel, which reveals whether
*something blocks the asyncio loop* (a synchronous C++/HAL call, an fsync that
doesn't really yield, GC pauses, ...).  That distinguishes "Python overhead
between steps" from "a blocking call stalls everything".

Usage (wrap the mission run)::

    from raccoon.profiling import StepProfiler


    async def main():
        async with StepProfiler(trace_path="profile.json") as prof:
            await mission.run(robot)
        # report prints automatically on exit; prof holds the raw samples

Then open ``profile.json`` at ``chrome://tracing`` (or ``edge://tracing`` /
https://ui.perfetto.dev) to *see* the gaps between steps on a timeline.

It also works as a plain ``with`` block if you are not inside async code yet;
the loop-lag monitor then starts lazily on the first step.
"""

from __future__ import annotations

import asyncio
import contextlib
import contextvars
import functools
import inspect
import json
import os
import statistics
import sys
import time
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, ClassVar

# Sample currently being filled in, so the phase wrappers (execute/db/acquire)
# can attribute their time to the right step. A ContextVar (not a plain global)
# so parallel steps — which run in separate asyncio tasks — don't clobber each
# other; ContextVars copy-on-task-spawn.
_current_sample: contextvars.ContextVar["StepSample | None"] = contextvars.ContextVar(
    "step_profiler_current_sample", default=None
)


@dataclass
class Phase:
    """A timed sub-region of a step (execute / db_read / db_write / acquire)."""

    label: str
    ts: float  # start, seconds relative to profiler t0
    dur: float  # seconds


@dataclass
class StepSample:
    """One execution of one step, with its phase breakdown."""

    signature: str
    path: str
    depth: int
    composite: bool
    tid: int
    ts: float = 0.0  # run_step start, relative to t0
    total: float = 0.0  # full run_step wall time
    execute: float = 0.0  # _execute_step body (children included for composites)
    db_read: float = 0.0  # get_upper_bound
    db_write: float = 0.0  # record_execution
    acquire: float = 0.0  # resource-manager acquire
    motion_compute: float = 0.0  # cumulative on_update() compute (motion steps)
    motion_updates: int = 0  # number of on_update() cycles
    hal: float = 0.0  # cumulative synchronous HAL/transport calls (set_position, ...)
    hal_calls: int = 0  # number of HAL calls made inside the step body
    sleep: float = 0.0  # cumulative deliberate ``asyncio.sleep`` waits inside the body
    phases: list[Phase] = field(default_factory=list)

    @property
    def overhead(self) -> float:
        """Wall time inside run_step not spent in the step body."""
        return max(self.total - self.execute, 0.0)

    @property
    def unattributed(self) -> float:
        """Overhead not explained by db/acquire (logging, signatures, asyncio)."""
        return max(self.overhead - self.db_read - self.db_write - self.acquire, 0.0)

    @property
    def body_compute(self) -> float:
        """Step-body time that is neither a deliberate sleep nor a HAL call.

        This is the "stands still but profiling shows no wait" budget: pure
        Python compute *plus* any blocking that escaped attribution. ``hal`` is
        a subset of ``execute`` (for motion steps it happens inside
        ``on_update``), so it is subtracted here only to isolate the remainder.
        """
        return max(self.execute - self.sleep - self.hal, 0.0)


@dataclass
class LagSpike:
    ts: float  # relative to t0
    lag_ms: float


class StepProfiler:
    """Runtime profiler that decomposes per-step wall time and event-loop lag.

    Args:
        trace_path: If set, write a Chrome-tracing (``chrome://tracing``) JSON
            file here on exit. Open it there or at https://ui.perfetto.dev.
        loop_lag: Run the event-loop-lag monitor (default True).
        loop_lag_interval: Sampling period of the lag monitor, seconds. Smaller
            = finer resolution but more samples. Default 0.005 (5 ms).
        lag_spike_ms: Lag above this (ms) is recorded as a spike and shown in
            the report / trace. Default 5 ms.
        print_report: Print the text report on exit (default True).
        top: How many step signatures to show in the per-signature table.
        defer_trace: Buffer the Chrome-trace write in RAM instead of writing it
            to disk when the profiled block exits, and flush all buffered traces
            in one go via :meth:`flush_traces` after the run. Default True — a
            per-mission profiler otherwise json.dumps a multi-MB trace to the
            Pi's SD card *between missions*, standing the robot still (~1.8 s for
            the setup trace) and perturbing the very run it measures.
    """

    # Traces buffered by every deferred profiler in this process. Class-level so
    # the per-mission profilers (each a short-lived instance) share one queue
    # that the run-completion path drains with :meth:`flush_traces`.
    _deferred_traces: ClassVar["list[tuple[str, StepProfiler]]"] = []

    def __init__(
        self,
        trace_path: str | None = None,
        *,
        loop_lag: bool = True,
        loop_lag_interval: float = 0.005,
        lag_spike_ms: float = 5.0,
        print_report: bool = True,
        top: int = 25,
        hal: bool = True,
        sleep_track: bool = True,
        hal_span_ms: float = 1.0,
        defer_trace: bool = True,
    ) -> None:
        self.trace_path = trace_path
        self.loop_lag = loop_lag
        self.loop_lag_interval = loop_lag_interval
        self.lag_spike_ms = lag_spike_ms
        self.print_report = print_report
        self.top = top
        self.hal = hal
        self.sleep_track = sleep_track
        self.hal_span_ms = hal_span_ms
        self.defer_trace = defer_trace

        self.samples: list[StepSample] = []
        self.lag_spikes: list[LagSpike] = []
        self.lag_samples: list[tuple[float, float]] = []  # (ts, lag_ms) for the trace
        self.max_lag_ms: float = 0.0
        # Per-HAL-method running aggregation: name -> [count, total_s, max_s].
        self.hal_stats: dict[str, list[float]] = {}

        self._t0: float = 0.0
        self._patches: list[tuple[Any, str, Any]] = []
        self._patched_exec_classes: set[type] = set()
        self._patched_motion_classes: set[type] = set()
        self._scanned_motion_classes: set[type] = set()
        self._hal_patched: set[type] = set()
        self._tids: dict[int, int] = {}
        self._next_tid: int = 0
        self._lag_task: asyncio.Task[None] | None = None
        self._lag_running: bool = False
        self._active: bool = False

    # -- env configuration ----------------------------------------------
    @classmethod
    def from_env(cls, env: "dict[str, str] | None" = None) -> "StepProfiler | None":
        """Build a profiler from environment variables, or ``None`` if disabled.

        Profiling is **off** unless ``RACCOON_PROFILE`` is set to a truthy value.
        Every knob of :class:`StepProfiler` is exposed so a run can be tuned
        without touching any code:

        ====================================  ====================================
        Environment variable                  Effect
        ====================================  ====================================
        ``RACCOON_PROFILE``                   Master switch. A path (contains ``/``
                                              or ends ``.json``) is used as the
                                              Chrome-trace path. ``1``/``true``/
                                              ``on``/``yes`` enables with the
                                              default trace path
                                              (``raccoon-profile.json``). ``0``/
                                              ``false``/``off``/``""`` disables.
        ``RACCOON_PROFILE_TRACE``             Explicit trace path (overrides the
                                              path inferred from ``RACCOON_PROFILE``;
                                              set to empty to skip the trace file).
        ``RACCOON_PROFILE_REPORT``            ``0``/``off`` to suppress the text
                                              report (default on).
        ``RACCOON_PROFILE_LOOP_LAG``          ``0``/``off`` to disable the
                                              event-loop-lag monitor (default on).
        ``RACCOON_PROFILE_LAG_INTERVAL_MS``   Lag sampling period in ms (default 5).
        ``RACCOON_PROFILE_LAG_SPIKE_MS``      Spike threshold in ms (default 5).
        ``RACCOON_PROFILE_TOP``               Rows in the per-signature table
                                              (default 25).
        ``RACCOON_PROFILE_HAL``               ``0``/``off`` to disable timing of
                                              synchronous HAL/transport calls
                                              (``set_position``, ``set_velocity``,
                                              ...) made *inside* a step body
                                              (default on).
        ``RACCOON_PROFILE_SLEEP``             ``0``/``off`` to disable splitting
                                              deliberate ``asyncio.sleep`` waits
                                              out of the step body (default on).
        ``RACCOON_PROFILE_HAL_SPAN_MS``       Only HAL calls at least this long
                                              (ms) get an individual trace span
                                              (the per-method totals always
                                              accumulate). Default 1 ms.
        ``RACCOON_PROFILE_DEFER``             ``0``/``off`` to write each trace
                                              inline when its block exits instead
                                              of buffering it for a single
                                              post-run flush (default on — keeps
                                              the multi-MB json.dump off the
                                              between-missions hot path).
        ====================================  ====================================

        Example::

            RACCOON_PROFILE=1 RACCOON_PROFILE_LAG_SPIKE_MS=2 uv run raccoon run
        """
        env = os.environ if env is None else env

        raw = (env.get("RACCOON_PROFILE") or "").strip()
        if raw == "" or raw.lower() in {"0", "false", "off", "no"}:
            return None

        # Decide the trace path. An explicit override wins; otherwise infer from
        # the master switch (a path-like value is the path, a bare flag uses the
        # default). An empty override means "no trace file, report only".
        if "RACCOON_PROFILE_TRACE" in env:
            trace_path = env["RACCOON_PROFILE_TRACE"].strip() or None
        elif raw.lower() in {"1", "true", "on", "yes"}:
            trace_path = "raccoon-profile.json"
        else:
            trace_path = raw  # treat the value itself as the trace path

        def _flag(name: str, default: bool) -> bool:
            val = env.get(name)
            if val is None:
                return default
            return val.strip().lower() not in {"0", "false", "off", "no", ""}

        def _num(name: str, default: float) -> float:
            try:
                return float(env[name])
            except (KeyError, ValueError):
                return default

        interval_ms = _num("RACCOON_PROFILE_LAG_INTERVAL_MS", 5.0)
        return cls(
            trace_path=trace_path,
            loop_lag=_flag("RACCOON_PROFILE_LOOP_LAG", True),
            loop_lag_interval=max(interval_ms, 0.1) / 1000.0,
            lag_spike_ms=_num("RACCOON_PROFILE_LAG_SPIKE_MS", 5.0),
            print_report=_flag("RACCOON_PROFILE_REPORT", True),
            top=int(_num("RACCOON_PROFILE_TOP", 25)),
            hal=_flag("RACCOON_PROFILE_HAL", True),
            sleep_track=_flag("RACCOON_PROFILE_SLEEP", True),
            hal_span_ms=_num("RACCOON_PROFILE_HAL_SPAN_MS", 1.0),
            defer_trace=_flag("RACCOON_PROFILE_DEFER", True),
        )

    # -- relative clock -------------------------------------------------
    def _now(self) -> float:
        return time.perf_counter() - self._t0

    def _tid_for_current_task(self) -> int:
        try:
            task = asyncio.current_task()
        except RuntimeError:
            task = None
        key = id(task) if task is not None else 0
        tid = self._tids.get(key)
        if tid is None:
            tid = self._next_tid
            self._next_tid += 1
            self._tids[key] = tid
        return tid

    # -- lifecycle ------------------------------------------------------
    def __enter__(self) -> "StepProfiler":
        self._t0 = time.perf_counter()
        self._install_patches()
        self._active = True
        # If a loop is already running, start the lag monitor now; otherwise it
        # starts lazily on the first wrapped run_step.
        if self.loop_lag:
            try:
                asyncio.get_running_loop()
                self._start_lag_monitor()
            except RuntimeError:
                pass
        return self

    def __exit__(self, *exc: Any) -> None:
        self._finish()

    async def __aenter__(self) -> "StepProfiler":
        self.__enter__()
        if self.loop_lag and self._lag_task is None:
            self._start_lag_monitor()
        return self

    async def __aexit__(self, *exc: Any) -> None:
        await self._stop_lag_monitor()
        self._finish()

    def _finish(self) -> None:
        if not self._active:
            return
        self._active = False
        self._lag_running = False
        self._remove_patches()
        if self.trace_path:
            if self.defer_trace:
                # Buffer for a single post-run flush so the multi-MB json.dump
                # never lands on the SD card between two missions. The samples
                # are already in RAM on ``self``; keeping the instance alive
                # defers *all* trace work (build + serialize + write) to idle.
                StepProfiler._deferred_traces.append((self.trace_path, self))
            else:
                self._write_trace(self.trace_path)
        if self.print_report:
            # Explicit stdout write (not print) so the diagnostics report is
            # config-agnostic w.r.t. flake8-print/T20 lint rules.
            sys.stdout.write(self.format_report() + "\n")

    @classmethod
    def flush_traces(cls) -> list[str]:
        """Write every buffered Chrome trace to disk; return the paths written.

        Call once after the run (robot idle) to drain the traces buffered by
        deferred per-mission profilers. Failures on one trace never block the
        others. Idempotent: a second call finds an empty queue.
        """
        pending = cls._deferred_traces
        cls._deferred_traces = []
        written: list[str] = []
        for path, prof in pending:
            try:
                prof._write_trace(path)
                written.append(path)
            except Exception:  # a diagnostic write must never crash shutdown
                pass
        return written

    # -- patching -------------------------------------------------------
    def _patch(self, owner: Any, attr: str, factory: Callable[[Any], Any]) -> None:
        original = getattr(owner, attr)
        setattr(owner, attr, factory(original))
        self._patches.append((owner, attr, original))

    def _remove_patches(self) -> None:
        for owner, attr, original in reversed(self._patches):
            setattr(owner, attr, original)
        self._patches.clear()
        self._patched_exec_classes.clear()
        self._hal_patched.clear()

    def _install_patches(self) -> None:
        from raccoon.step.base import Step, _step_path  # local import: optional dep

        prof = self

        # run_step: total wall time + sample creation + lazy lag start + lazy
        # per-class _execute_step patching.
        original_run_step = Step.run_step

        @functools.wraps(original_run_step)
        async def run_step(self: Any, robot: Any) -> Any:
            if prof.loop_lag and prof._lag_task is None:
                prof._start_lag_monitor()
            prof._ensure_execute_patched(type(self))

            try:
                signature = self._generate_signature()
            except Exception:
                signature = type(self).__name__
            path = _step_path.get()
            sample = StepSample(
                signature=signature,
                path=" > ".join(path),
                depth=len(path),
                composite=bool(getattr(self, "_composite", False)),
                tid=prof._tid_for_current_task(),
                ts=prof._now(),
            )
            prof.samples.append(sample)
            token = _current_sample.set(sample)
            start = time.perf_counter()
            try:
                return await original_run_step(self, robot)
            finally:
                sample.total = time.perf_counter() - start
                _current_sample.reset(token)

        self._patch(Step, "run_step", lambda _orig: run_step)

        # Timing DB read/write.
        try:
            from raccoon.timing import StepTimingTracker

            self._patch(
                StepTimingTracker,
                "record_execution",
                lambda orig: self._wrap_async_phase(orig, "db_write"),
            )
            self._patch(
                StepTimingTracker,
                "get_upper_bound",
                lambda orig: self._wrap_async_phase(orig, "db_read"),
            )
        except Exception:
            pass

        # Resource acquire (synchronous).
        try:
            from raccoon.step.resource import ResourceManager

            self._patch(
                ResourceManager,
                "acquire",
                lambda orig: self._wrap_sync_phase(orig, "acquire"),
            )
        except Exception:
            pass

        # Deep, *inside the step* instrumentation: deliberate sleeps and the
        # synchronous HAL/transport calls that block the loop without showing up
        # as a wait. These explain a step that "stands still" while the overhead
        # and lag reports show nothing.
        self._install_sleep_patch()
        self._install_hal_patches()

    # -- inside-step: deliberate sleeps ---------------------------------
    def _install_sleep_patch(self) -> None:
        """Wrap ``asyncio.sleep`` so deliberate waits split out of the body.

        Step bodies call ``await asyncio.sleep(...)`` to wait for a servo/motor
        move to physically finish. That time is real but *intended* — separating
        it from the body leaves ``body_compute`` as the suspicious "stood still
        for no reason" remainder. Attribution is via the live sample, so the
        lag-monitor's own sleeps (sample is ``None``) are ignored.
        """
        if not self.sleep_track:
            return
        prof = self
        original = asyncio.sleep

        @functools.wraps(original)
        async def sleep(delay: float, *args: Any, **kwargs: Any) -> Any:
            sample = _current_sample.get()
            start = time.perf_counter()
            ts = prof._now()
            try:
                return await original(delay, *args, **kwargs)
            finally:
                dur = time.perf_counter() - start
                if sample is not None:
                    sample.sleep += dur
                    if dur * 1000.0 >= prof.hal_span_ms:
                        sample.phases.append(Phase("sleep", ts, dur))

        self._patch(asyncio, "sleep", lambda _orig: sleep)

    # -- inside-step: HAL / transport calls ----------------------------
    def _install_hal_patches(self) -> None:
        """Time every synchronous HAL/transport call made inside a step body.

        These are the pybind-bound hardware methods — ``set_position``,
        ``set_velocity``, ``move_to_position``, ``read``, ... — that cross into
        C++ and the transport. If one blocks (e.g. a reliable publish waiting on
        the bus) the loop stalls *without* an ``await``, so the existing
        overhead/lag view can't see which call did it. Per-method totals always
        accumulate; only calls >= ``hal_span_ms`` get an individual trace span,
        so a 100 Hz ``get_position`` poll doesn't bloat the trace.
        """
        if not self.hal:
            return
        import importlib

        targets = {
            "raccoon.hal": [
                "Servo",
                "Motor",
                "IMotor",
                "IMU",
                "AnalogSensor",
                "DigitalSensor",
                "ButtonGroup",
                "IOdometry",
            ],
            "raccoon.drive": ["Drive", "VelocityController", "MotorAdapter"],
        }
        for mod_name, cls_names in targets.items():
            try:
                mod = importlib.import_module(mod_name)
            except Exception:
                continue
            for cls_name in cls_names:
                cls = getattr(mod, cls_name, None)
                if not isinstance(cls, type) or cls in self._hal_patched:
                    continue
                self._hal_patched.add(cls)
                for meth_name in dir(cls):
                    if meth_name.startswith("_"):
                        continue
                    raw = getattr(cls, meth_name, None)
                    # Skip properties / data descriptors and non-callables; only
                    # the bound C++ instance methods are worth timing.
                    if raw is None or isinstance(raw, property) or not callable(raw):
                        continue
                    # Skip static/class methods (e.g. Servo.fully_disable_all,
                    # Motor.disable_all): they are called with no instance, so a
                    # plain-function wrapper would break their call convention.
                    static_kind = inspect.getattr_static(cls, meth_name, None)
                    if isinstance(static_kind, staticmethod | classmethod):
                        continue
                    try:
                        wrapper = self._wrap_hal_method(cls_name, meth_name, raw)
                        setattr(cls, meth_name, wrapper)
                    except (TypeError, AttributeError):
                        continue
                    self._patches.append((cls, meth_name, raw))

    def _wrap_hal_method(
        self, cls_name: str, meth_name: str, original: Callable[..., Any]
    ) -> Callable[..., Any]:
        prof = self
        label = f"{cls_name}.{meth_name}"

        # ``*args`` (not an explicit ``self``) so the wrapper binds correctly no
        # matter how the method is invoked — as a bound instance method
        # (``servo.set_position(x)`` -> args == (servo, x)) or explicitly via the
        # class. Static/class methods are filtered out before we get here.
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            sample = _current_sample.get()
            start = time.perf_counter()
            ts = prof._now()
            try:
                return original(*args, **kwargs)
            finally:
                dur = time.perf_counter() - start
                st = prof.hal_stats.get(label)
                if st is None:
                    prof.hal_stats[label] = [1.0, dur, dur]
                else:
                    st[0] += 1.0
                    st[1] += dur
                    st[2] = max(dur, st[2])
                if sample is not None:
                    sample.hal += dur
                    sample.hal_calls += 1
                    if dur * 1000.0 >= prof.hal_span_ms:
                        sample.phases.append(Phase(f"hal:{label}", ts, dur))

        wrapper.__name__ = meth_name
        return wrapper

    def _ensure_execute_patched(self, cls: type) -> None:
        """Wrap the step body (``_execute_step``) once, to time it directly.

        ``_execute_step`` may be defined on the concrete class *or* inherited
        from a base that drives a loop — most importantly ``MotionStep``, whose
        ``_execute_step`` runs the fixed-rate ``on_update`` loop. We resolve the
        class in the MRO that actually *defines* ``_execute_step`` and patch
        that one. Without this, motion steps (which don't override
        ``_execute_step``) had their entire drive time fall through to
        "overhead" instead of the step body.
        """
        defining = next((c for c in cls.__mro__ if "_execute_step" in c.__dict__), None)
        if defining is not None and defining not in self._patched_exec_classes:
            self._patched_exec_classes.add(defining)
            original = defining.__dict__["_execute_step"]
            prof = self

            @functools.wraps(original)
            async def _execute_step(self: Any, robot: Any) -> Any:
                sample = _current_sample.get()
                start = time.perf_counter()
                ts = prof._now()
                try:
                    return await original(self, robot)
                finally:
                    dur = time.perf_counter() - start
                    if sample is not None:
                        sample.execute += dur
                        sample.phases.append(Phase("execute", ts, dur))

            defining._execute_step = _execute_step
            self._patches.append((defining, "_execute_step", original))

        self._ensure_motion_update_patched(cls)

    def _ensure_motion_update_patched(self, cls: type) -> None:
        """Patch the concrete ``on_update`` of a motion step once.

        ``on_update`` is overridden on the concrete class (or an intermediate
        base like a shared line-follow base) — *not* on ``MotionStep``, whose
        ``on_update`` is the abstract stub. So we resolve the class that
        actually defines it (excluding the stub) and patch that one.
        """
        if cls in self._scanned_motion_classes:
            return
        self._scanned_motion_classes.add(cls)
        try:
            from raccoon.step.motion.motion_step import MotionStep
        except Exception:
            return
        if not (isinstance(cls, type) and issubclass(cls, MotionStep)):
            return
        defining = next(
            (c for c in cls.__mro__ if "on_update" in c.__dict__ and c is not MotionStep),
            None,
        )
        if defining is None or defining in self._patched_motion_classes:
            return
        self._patched_motion_classes.add(defining)
        self._patch(defining, "on_update", lambda orig: self._wrap_motion_update(orig))

    def _wrap_motion_update(self, original: Callable[..., Any]) -> Callable[..., Any]:
        """Accumulate per-cycle ``on_update`` compute time onto the live sample.

        Recorded as a scalar (not a per-cycle trace span) so a 100 Hz loop over
        several seconds doesn't bloat the trace with thousands of events. The
        loop's wall time is already attributed to ``execute``; this isolates how
        much of it was Python compute vs waiting for the robot to move.
        """

        @functools.wraps(original)
        def wrapper(self: Any, robot: Any, dt: float) -> Any:
            sample = _current_sample.get()
            start = time.perf_counter()
            try:
                return original(self, robot, dt)
            finally:
                if sample is not None:
                    sample.motion_compute += time.perf_counter() - start
                    sample.motion_updates += 1

        return wrapper

    def _wrap_async_phase(
        self, original: Callable[..., Any], field_name: str
    ) -> Callable[..., Any]:
        prof = self

        @functools.wraps(original)
        async def wrapper(*args: Any, **kwargs: Any) -> Any:
            sample = _current_sample.get()
            start = time.perf_counter()
            ts = prof._now()
            try:
                return await original(*args, **kwargs)
            finally:
                dur = time.perf_counter() - start
                if sample is not None:
                    setattr(sample, field_name, getattr(sample, field_name) + dur)
                    sample.phases.append(Phase(field_name, ts, dur))

        return wrapper

    def _wrap_sync_phase(self, original: Callable[..., Any], field_name: str) -> Callable[..., Any]:
        prof = self

        @functools.wraps(original)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            sample = _current_sample.get()
            start = time.perf_counter()
            ts = prof._now()
            try:
                return original(*args, **kwargs)
            finally:
                dur = time.perf_counter() - start
                if sample is not None:
                    setattr(sample, field_name, getattr(sample, field_name) + dur)
                    sample.phases.append(Phase(field_name, ts, dur))

        return wrapper

    # -- event-loop lag monitor ----------------------------------------
    def _start_lag_monitor(self) -> None:
        if self._lag_task is not None:
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return
        self._lag_running = True
        self._lag_task = loop.create_task(self._lag_monitor())

    async def _stop_lag_monitor(self) -> None:
        self._lag_running = False
        task = self._lag_task
        self._lag_task = None
        if task is not None:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await task

    async def _lag_monitor(self) -> None:
        loop = asyncio.get_running_loop()
        interval = self.loop_lag_interval
        expected = loop.time() + interval
        while self._lag_running:
            try:
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            now = loop.time()
            lag = now - expected  # how much later than scheduled we woke up
            expected = now + interval
            lag_ms = lag * 1000.0
            if lag_ms < 0:
                lag_ms = 0.0
            ts = self._now()
            self.lag_samples.append((ts, lag_ms))
            self.max_lag_ms = max(self.max_lag_ms, lag_ms)
            if lag_ms >= self.lag_spike_ms:
                self.lag_spikes.append(LagSpike(ts, lag_ms))

    # -- reporting ------------------------------------------------------
    def format_report(self) -> str:
        leaf = [s for s in self.samples if not s.composite]
        lines: list[str] = []
        w = lines.append

        wall = max((s.ts + s.total for s in self.samples), default=0.0)
        n_leaf = len(leaf)
        sum_total = sum(s.total for s in leaf)
        sum_exec = sum(s.execute for s in leaf)
        sum_over = sum(s.overhead for s in leaf)
        sum_dbw = sum(s.db_write for s in leaf)
        sum_dbr = sum(s.db_read for s in leaf)
        sum_acq = sum(s.acquire for s in leaf)
        sum_unattr = sum(s.unattributed for s in leaf)

        def pct(x: float) -> str:
            return f"{(100.0 * x / wall):5.1f}%" if wall > 0 else "  n/a"

        w("")
        w("=" * 78)
        w("  STEP PROFILER REPORT")
        w("=" * 78)
        w(f"  wall clock (all steps)      : {wall:8.3f} s")
        w(f"  leaf steps executed         : {n_leaf}")
        w("")
        w("  WHERE THE TIME GOES (leaf steps; overhead = run_step minus step body)")
        w("  " + "-" * 74)
        w(f"  step body  (execute)        : {sum_exec:8.3f} s   {pct(sum_exec)}")
        w(f"  overhead   (between steps)  : {sum_over:8.3f} s   {pct(sum_over)}")
        w(
            f"      db write (record_exec)  : {sum_dbw:8.3f} s   {pct(sum_dbw)}"
            f"   <- SQLite insert+commit per step"
        )
        w(f"      db read  (upper_bound)  : {sum_dbr:8.3f} s   {pct(sum_dbr)}")
        w(f"      resource acquire        : {sum_acq:8.3f} s   {pct(sum_acq)}")
        w(f"      other (log/sig/asyncio) : {sum_unattr:8.3f} s   {pct(sum_unattr)}")
        if sum_total > 0:
            w("")
            w(
                f"  => {100.0 * sum_over / sum_total:.1f}% of step wall time is overhead, "
                f"not the step body."
            )

        # Motion steps: split the drive time (counted in `execute`) into Python
        # compute vs waiting for the robot to physically move.
        sum_mc = sum(s.motion_compute for s in leaf)
        n_upd = sum(s.motion_updates for s in leaf)
        if n_upd:
            sum_motion_exec = sum(s.execute for s in leaf if s.motion_updates)
            wait = max(sum_motion_exec - sum_mc, 0.0)
            w("")
            w("  MOTION on_update() — of drive time, how much is compute vs waiting?")
            w("  " + "-" * 74)
            w(f"  on_update compute           : {sum_mc:8.3f} s   {pct(sum_mc)}")
            w(f"  waiting for motion          : {wait:8.3f} s   {pct(wait)}")
            w(
                f"  on_update cycles            : {n_upd}"
                f"   (avg {1000.0 * sum_mc / n_upd:.3f} ms/cycle)"
            )

        # Inside the step body: split execute into deliberate sleeps, blocking
        # HAL/transport calls, and the leftover compute (the "stands still but
        # the wait reports show nothing" budget the user is chasing).
        sum_sleep = sum(s.sleep for s in leaf)
        sum_hal = sum(s.hal for s in leaf)
        sum_body = sum(s.body_compute for s in leaf)
        n_hal_calls = sum(s.hal_calls for s in leaf)
        if sum_exec > 0:
            w("")
            w("  INSIDE THE STEP BODY (execute split: where does in-step time go?)")
            w("  " + "-" * 74)
            w(f"  deliberate sleep (awaits)   : {sum_sleep:8.3f} s   {pct(sum_sleep)}")
            w(
                f"  HAL/transport calls (block) : {sum_hal:8.3f} s   {pct(sum_hal)}"
                f"   <- {n_hal_calls} synchronous hardware calls"
            )
            w(
                f"  python compute / unattrib   : {sum_body:8.3f} s   {pct(sum_body)}"
                f"   <- standstill w/o a visible wait"
            )

        # Per-HAL-method table: which hardware call eats the time / blocks.
        if self.hal_stats:
            hal_rows = sorted(
                ((name, int(st[0]), st[1], st[2]) for name, st in self.hal_stats.items()),
                key=lambda r: r[2],
                reverse=True,
            )
            w("")
            w("  TOP HAL / TRANSPORT CALLS (synchronous; a slow one blocks the loop)")
            w("  " + "-" * 74)
            w(f"  {'method':<34}{'n':>6}{'total':>10}{'mean_ms':>10}{'max_ms':>9}")
            for name, n, tot, mx in hal_rows[: self.top]:
                label = name if len(name) <= 33 else name[:30] + "..."
                mean_ms = (tot / n * 1000.0) if n else 0.0
                w(f"  {label:<34}{n:>6}{tot:>9.3f}s{mean_ms:>10.3f}{mx * 1000.0:>9.3f}")

        # Per-signature aggregation, sorted by total overhead (biggest standstills).
        agg: dict[str, list[StepSample]] = defaultdict(list)
        for s in leaf:
            agg[s.signature].append(s)
        rows = []
        for sig, ss in agg.items():
            rows.append(
                (
                    sig,
                    len(ss),
                    sum(x.overhead for x in ss),
                    statistics.mean([x.execute for x in ss]) * 1000,
                    statistics.mean([x.overhead for x in ss]) * 1000,
                    statistics.mean([x.db_write for x in ss]) * 1000,
                )
            )
        rows.sort(key=lambda r: r[2], reverse=True)

        w("")
        w("  TOP STEPS BY TOTAL OVERHEAD (the standstill budget)")
        w("  " + "-" * 74)
        w(f"  {'signature':<34}{'n':>4}{'ovh_tot':>9}{'exec_ms':>9}" f"{'ovh_ms':>8}{'dbw_ms':>8}")
        for sig, n, ovh_tot, exec_ms, ovh_ms, dbw_ms in rows[: self.top]:
            label = sig if len(sig) <= 33 else sig[:30] + "..."
            w(f"  {label:<34}{n:>4}{ovh_tot:>8.3f}s{exec_ms:>9.1f}" f"{ovh_ms:>8.1f}{dbw_ms:>8.1f}")

        # Event-loop lag.
        w("")
        w("  EVENT-LOOP LAG (does something block the asyncio loop?)")
        w("  " + "-" * 74)
        if self.lag_samples:
            lags = [x[1] for x in self.lag_samples]
            lags_sorted = sorted(lags)

            def p(q: float) -> float:
                return lags_sorted[min(int(q * len(lags_sorted)), len(lags_sorted) - 1)]

            w(
                f"  samples                     : {len(lags)} @ {self.loop_lag_interval * 1000:.0f} ms"
            )
            w(f"  max lag                     : {self.max_lag_ms:8.1f} ms")
            w(f"  p50 / p99 lag               : {p(0.50):8.1f} / {p(0.99):.1f} ms")
            w(f"  spikes > {self.lag_spike_ms:.0f} ms             : {len(self.lag_spikes)}")
            if self.lag_spikes:
                w("  worst spikes (loop blocked here):")
                for sp in sorted(self.lag_spikes, key=lambda s: s.lag_ms, reverse=True)[:8]:
                    near = self._step_near(sp.ts)
                    w(f"      t={sp.ts:7.3f}s  lag={sp.lag_ms:7.1f} ms   near: {near}")
        else:
            w("  (loop-lag monitor did not run — were any steps executed under async?)")

        w("")
        if self.trace_path:
            w(f"  Chrome trace written: {self.trace_path}")
            w("  Open it at chrome://tracing  (or https://ui.perfetto.dev) to see the gaps.")
        w("=" * 78)
        return "\n".join(lines)

    def _step_near(self, ts: float) -> str:
        """Signature of the step whose run_step window contains ``ts``."""
        best = None
        for s in self.samples:
            if s.composite:
                continue
            # deepest matching window = most specific step
            if s.ts <= ts <= s.ts + s.total and (best is None or s.depth > best.depth):
                best = s
        if best is not None:
            return best.signature
        # otherwise: between steps
        return "(between steps)"

    # -- chrome trace ---------------------------------------------------
    def _write_trace(self, path: str) -> None:
        events: list[dict[str, Any]] = []

        # Lane (tid) names.
        used_tids = sorted({s.tid for s in self.samples})
        for tid in used_tids:
            events.append(
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": 1,
                    "tid": tid,
                    "args": {"name": f"task-{tid}" if tid else "main"},
                }
            )
        lag_tid = (max(used_tids) + 1) if used_tids else 1
        events.append(
            {
                "name": "thread_name",
                "ph": "M",
                "pid": 1,
                "tid": lag_tid,
                "args": {"name": "event-loop-lag"},
            }
        )

        # Step spans (complete events) + phase spans.
        for s in self.samples:
            events.append(
                {
                    "name": s.signature,
                    "cat": "composite" if s.composite else "step",
                    "ph": "X",
                    "pid": 1,
                    "tid": s.tid,
                    "ts": round(s.ts * 1e6, 1),
                    "dur": round(s.total * 1e6, 1),
                    "args": {
                        "path": s.path,
                        "execute_ms": round(s.execute * 1e3, 3),
                        "overhead_ms": round(s.overhead * 1e3, 3),
                        "db_write_ms": round(s.db_write * 1e3, 3),
                        "db_read_ms": round(s.db_read * 1e3, 3),
                        "acquire_ms": round(s.acquire * 1e3, 3),
                        "hal_ms": round(s.hal * 1e3, 3),
                        "hal_calls": s.hal_calls,
                        "sleep_ms": round(s.sleep * 1e3, 3),
                        "body_compute_ms": round(s.body_compute * 1e3, 3),
                    },
                }
            )
            for ph in s.phases:
                events.append(
                    {
                        "name": ph.label,
                        "cat": "phase",
                        "ph": "X",
                        "pid": 1,
                        "tid": s.tid,
                        "ts": round(ph.ts * 1e6, 1),
                        "dur": round(ph.dur * 1e6, 1),
                    }
                )

        # Loop lag as a counter track + instant markers for spikes.
        for ts, lag_ms in self.lag_samples:
            events.append(
                {
                    "name": "loop_lag",
                    "ph": "C",
                    "pid": 1,
                    "tid": lag_tid,
                    "ts": round(ts * 1e6, 1),
                    "args": {"ms": round(lag_ms, 2)},
                }
            )
        for sp in self.lag_spikes:
            events.append(
                {
                    "name": f"LAG {sp.lag_ms:.0f}ms",
                    "ph": "i",
                    "s": "g",
                    "pid": 1,
                    "tid": lag_tid,
                    "ts": round(sp.ts * 1e6, 1),
                }
            )

        with Path(path).open("w") as f:
            json.dump({"traceEvents": events, "displayTimeUnit": "ms"}, f)
