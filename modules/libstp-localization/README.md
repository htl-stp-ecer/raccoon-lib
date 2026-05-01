# libstp-localization

Phase-2 pass-through localization service. Owns a background thread that
polls an `IOdometry` source at 100 Hz (configurable via `LocalizationConfig`)
and propagates per-tick deltas into a world-frame pose snapshot.

This module deliberately does **not** filter. It exists to give the rest of
the stack a stable, persistent world pose across motion-step boundaries —
the property that `Stm32Odometry` alone cannot provide because of its reset
behavior.

## API surface

- `Localization(std::shared_ptr<IOdometry>, LocalizationConfig)` — RAII; the
  constructor starts the worker thread, the destructor joins.
- `getPose()` — thread-safe snapshot.
- `observe(Observation)` — hard-snap selected axes (any axis whose sigma is
  finite). After a snap, the next tick rebases its delta from the current
  odometry sample so the snap is not undone.
- `start()` / `stop()` — idempotent, exposed for tests.

## Phase 6 follow-up

The pass-through is replaced by a particle filter that consumes
`Observation` as a soft, sigma-weighted likelihood. Until then, finite sigma
means "snap exactly", infinite sigma means "leave that axis alone".
