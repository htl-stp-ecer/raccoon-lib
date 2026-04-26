# Testing Debt

This file tracks tests that are currently disabled or excluded from CI, along
with the reason and a triage owner. It exists because a green CI badge is
worthless if the build skips the tests that would have caught a regression.

Before disabling another test, append an entry here.

---

## C++

### `MockSimIntegration` (excluded from `ctest`)

- **Where:** `.github/workflows/test.yml` — `ctest -E MockSimIntegration`
- **Why:** Tests pass individually but segfault during singleton teardown
  between test cases. Pre-existing issue introduced before the public
  open-sourcing.
- **Real bug, not just a test bug.** `MockPlatform::instance()` is a Meyers
  singleton; when gtest tears down the test fixtures the static destructor
  order across translation units can leave dangling references between the
  mock and `SimWorld` / `OdometryBridge`.
- **Plan:**
  1. Reproduce locally with `ctest --rerun-failed` after a single passing
     `MockSimIntegration` invocation.
  2. Either change `MockPlatform` to per-test reset semantics (preferred),
     or stop holding non-trivial state across test cases.
  3. Re-enable `ctest` without `-E`.

---

## Python

The CI workflow ignores six Python test files via `pytest --ignore=…`. Each
needs to be triaged: either re-enabled (if it now passes), fixed, or marked
with `@pytest.mark.xfail(reason=…, strict=False)` so it stays visible.

`xfail_strict = true` is set in `pyproject.toml`, so xfailed tests will
fail CI if they unexpectedly start passing — that is the correct safety
net once the entries below are migrated from `--ignore` to `xfail`.

### `tests/python/test_calibrate_distance.py`

- **Why excluded:** Last green run pre-dates the smooth_path / per-wheel
  calibration refactor (`5dbad3f` "smooth path optimization"). The
  `_update_yaml_calibration` signature and the `MotorCalibration.ema_baseline`
  field both changed; tests reference the old shape.
- **Plan:** Update the assertions to match the current
  `result.new_ticks_to_rad`-only flow (see commit
  `38b7c56` "misc: clamp PID lpf alpha…").

### `tests/python/test_input_validation.py`

- **Why excluded:** 32 KB file covering construction-time validation across
  every step + condition class. Likely broken by a step-class signature
  change in the smooth_path refactor. **This is one of the most
  load-bearing tests in the suite for production safety** — many of the
  bugs flagged in the open-source code review (e.g. trapezoidal-profile
  zero-acceleration, motor-calibration zero ticks) would be caught here.
- **Plan:** Highest-priority of the Python excludes. Re-enable next.

### `tests/python/test_resource_conflicts.py`

- **Why excluded:** Tests that the `ResourceManager` rejects parallel steps
  that would touch the same hardware port simultaneously. Likely broken
  by the `_force_quit` / cancellation regression-fix in `7fc7a89`
  ("robot: robust shutdown + firmware-side smooth servo") changing how
  parallel branches release resources on cancellation.
- **Plan:** Re-run after `test_input_validation` is fixed (shared mock
  fixtures).

### `tests/python/test_watchdog.py`

- **Why excluded:** Tests the watchdog-manager / mission `time_budget`
  flow. Broken by the same shutdown / cancellation work. The `_force_quit`
  path bypasses the watchdog `expired_name` reporting that this test
  asserts on.
- **Plan:** Verify whether the test is still semantically valid after
  the force-quit fallback — if yes, fix the assertion; if no, rewrite to
  cover the new contract (graceful cancel → force-quit fallback).

### `tests/python/testing/test_plugin_end_to_end.py`

- **Why excluded:** Spawns `pytest` in a synthetic raccoon project to
  validate the `pytest11` entry point. Likely flaky in QEMU-emulated
  ARM64 (the inner pytest fork is slow and the timeout is tight).
- **Plan:** Either bump the inner timeout or split into a smaller smoke
  test that doesn't fork pytest.

### `tests/python/sim/test_auto_tune.py` (segregated, not excluded)

Not strictly excluded — runs in a dedicated `test-auto-tune` GitHub
Actions job because each test takes ~13 minutes. This is fine; just
documented here for completeness.

---

## How to clear an entry

1. Run the test locally:
   ```bash
   pip install -e . --config-settings=cmake.define.DRIVER_BUNDLE=mock
   pytest tests/python/<file>.py -v
   ```
2. Fix the failing case or mark it `@pytest.mark.xfail(reason="…")`.
3. Remove the corresponding `--ignore=…` line from `.github/workflows/test.yml`.
4. Delete the entry above.
5. Open a follow-up issue if the underlying bug needs a separate fix.
