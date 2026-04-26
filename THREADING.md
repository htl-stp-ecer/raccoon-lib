# Threading model

This file documents the concurrency contract across the C++ and Python
layers of raccoon-lib so contributors can tell at a glance what they
need to lock and what is single-threaded by construction.

## Mission code (Python, asyncio)

User-written missions live in a single asyncio event loop owned by
`GenericRobot.run()`. There is **one** event loop, **one** thread.
Everything in `raccoon.step.*` is async-cooperative and not thread-safe
unless documented otherwise.

- `Step._execute_step()` and the `MotionStep.on_start/update/stop` hooks
  run on this loop.
- `seq()`, `parallel()`, `loop()` etc. compose steps within the same
  task tree; `parallel` uses `asyncio.gather`, not threads.
- `Defs.arm_servo`, `robot.drive`, `robot.odometry` etc. are not
  protected by Python-level locks. Multiple parallel steps must not
  command the same servo / motor / sensor — the `ResourceManager`
  (libstp-step) detects this at construction time when `required_resources()`
  is set on the step.

## C++ background work

A handful of C++ subsystems run their own threads and call back into
shared state:

- **`hal::motor::MotorFailSafe` watchdog** (Motor.cpp) — single std::thread
  spawned at process start. Polls a `volatile sig_atomic_t` flag set by
  the SIGINT/SIGTERM signal handler and calls `Motor::disableAll()`
  when the flag is set. Joined in the destructor before
  `disable_all_once()` runs as a final atexit safety net. The signal
  handler itself only writes the flag (async-signal-safe); all
  non-reentrant cleanup happens on the watchdog thread.

- **LCM reader thread** (libstp-platforms/wombat) — `LcmReader` runs the
  LCM `lcm::LCM::handle()` loop on a dedicated thread. Subscriber
  callbacks fire on this thread and copy received data into shared
  state guarded by `std::mutex`. Python users never see this thread
  directly: the readers expose snapshot accessors that briefly lock,
  copy, return.

- **Mock platform auto-tick** (libstp-platforms/mock) — when
  `setAutoTickEnabled(true)` is set in tests, the next read of an
  analog/IMU sensor advances the simulation. There is no separate
  ticker thread; auto-tick runs synchronously on the calling thread
  under `m_simMutex` (separate from the HAL state mutex `m_mutex` to
  prevent physics integration from blocking sensor reads).

## Locks and lock order

| Lock | Guards | Owner |
| ---- | ------ | ----- |
| `MockPlatform::m_mutex` | HAL state: motors[], servos[], analog values, digital bitmask, IMU values | mock platform |
| `MockPlatform::m_simMutex` | `m_sim`, sim origin, autoTick fields, sim clock | mock platform |
| `hal::detail::PortRegistry::mu_` | shared `std::set<int>` of in-use ports per HAL class (Motor/Servo/Analog/Digital) | hal |
| `LcmReader::*Mutex` | per-channel last-received-message snapshots | wombat platform |

**Acquire order:** when a method touches both `m_mutex` and `m_simMutex`,
take `m_mutex` first, then `m_simMutex`. There is no path that reverses
this; if you add one, you create a deadlock.

## Pybind11 / GIL

C++ code that calls back into Python (notably the LCM reader thread
when an LCM-driven update triggers a Python callback) must hold the
GIL. The current code does not expose user callbacks from background
threads — all cross-thread communication is "background C++ writes
into shared state, Python reads on the asyncio thread under a brief
lock" — so the GIL story stays simple.

If you add a Python callback that fires from `LcmReader`, wrap the
invocation in `py::gil_scoped_acquire` and document the new contract
here.

## What you should NOT assume

- Steps are *not* thread-safe across threads. They are async-safe within
  one event loop.
- HAL classes (`Motor`, `Servo`, `AnalogSensor`, `DigitalSensor`) cache
  state and are not designed for simultaneous-thread access. Their
  port-registry guard catches **construction** races, not concurrent
  command races.
- `SimWorld` itself has no internal locking. All synchronisation
  happens at the `MockPlatform` boundary via `m_simMutex`.

## When to add another lock

You probably do not. New shared state should either:
1. Live behind one of the existing locks (extend its scope, document
   here), or
2. Use `std::atomic` for a single primitive and avoid the mutex
   entirely, or
3. Stay strictly on the asyncio thread (no shared state at all).

Adding a third lock to a HAL singleton is almost always a sign that
the abstraction is wrong. Discuss before merging.
