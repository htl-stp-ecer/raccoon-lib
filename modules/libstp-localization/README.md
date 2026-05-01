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

## Wiring example

`GenericRobot.localization` is opt-in. A robot subclass enables the service
by stashing an instance under `self._localization` — the property exposes
it, and motions/resync sites pick it up from there. For sim/test builders
that return a `SimpleNamespace` (see `tests/python/sim/_robot_builder.py`)
the same attribute name works because the property's lookup is duck-typed.

```python
from raccoon.localization import Localization, LocalizationConfig
from raccoon.odometry_stm32 import Stm32Odometry, Stm32OdometryConfig


class MyRobot(GenericRobot):
    def __init__(self) -> None:
        # ... build drive, kinematics, IMU, bridge as usual ...
        self._odometry = Stm32Odometry(
            imu=imu, kinematics=kin, bridge=bridge, config=Stm32OdometryConfig()
        )
        # 100 Hz pass-through; finer ticks (e.g. 5 ms) are appropriate for
        # short-running tests.
        self._localization = Localization(self._odometry, LocalizationConfig())
        super().__init__()

    @property
    def odometry(self):
        return self._odometry
```

The constructor starts the worker thread; the destructor joins it. Keep the
`Localization` instance alive for the full mission lifetime — never
re-create it per motion.

## Reset detection (heuristic, until Phase 4)

Motion `start()` still calls `odometry.reset()` (Phase ≤3 behaviour; the
cleanup is scheduled for Phase 4). To keep the world pose stable across
those resets, `tickLoop()` swallows two classes of "non-deltas":

- **Origin snap** — current odometry sample lands at the origin while the
  previous sample was meaningfully off it. That is the unambiguous
  signature of `Stm32Odometry::reset()` and the only way the underlying
  pose can return to exactly `(0, 0, 0)`.
- **Impossible jump** — any single-tick delta larger than 0.5 m
  (≥ 50 m/s at 100 Hz) is rejected defensively. This catches future
  odometry sources whose reset does not land precisely on the origin.

Both branches rebase `m_lastOdom` to the current sample without touching
the world pose, so the next genuine motion delta is measured from the
post-reset reference. Once Phase 4 lifts `odometry.reset()` out of motion
`start()`, the heuristic becomes redundant — it stays in place as a
defensive net.

The end-to-end behaviour is covered by
`Localization.WorldPoseSurvivesOdometryReset`,
`Localization.WorldPoseSurvivesResetAfterObserve`, and
`Localization.ImpossibleJumpIsTreatedAsReset` in
`tests/cpp/localization/test_localization.cpp`, plus the cross-motion
smoke test in `tests/python/sim/test_localization_cross_motion.py`.

## Phase 6 follow-up

The pass-through is replaced by a particle filter that consumes
`Observation` as a soft, sigma-weighted likelihood. Until then, finite sigma
means "snap exactly", infinite sigma means "leave that axis alone".
