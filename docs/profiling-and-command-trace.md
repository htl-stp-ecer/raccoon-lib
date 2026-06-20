# Profiling & command-ordering tracing

Two complementary diagnostics ship with raccoon-lib:

1. **StepProfiler** — *where does the time inside a mission go?* Now drills
   *inside* each step body, not just between steps.
2. **Command trace** — *do motor/servo commands actually arrive and execute in
   the order they were sent?* Correlates raccoon-lib (publisher) with
   stm32-data-reader (consumer) and flags reordering.

---

## 1. StepProfiler — deep per-step profiling

Wrap a mission run (or just set the env var — `raccoon run` builds the profiler
from the environment automatically):

```bash
RACCOON_PROFILE=1 uv run raccoon run        # report + raccoon-profile.json
```

Open `raccoon-profile.json` at <chrome://tracing> or <https://ui.perfetto.dev>.

### What's new: inside-the-step breakdown

Previously a step's `execute` time was a black box, so a step that "stood still"
with no visible wait was unexplained. The profiler now splits `execute` into:

| Bucket | Meaning |
|---|---|
| **deliberate sleep** | `await asyncio.sleep(...)` — intended waits (e.g. waiting for a servo move to finish). |
| **HAL/transport calls** | synchronous pybind hardware calls (`set_position`, `set_velocity`, `move_to_position`, `read`, …). A slow one here **blocks the event loop without an `await`** — the usual cause of a standstill the lag/overhead views can't see. |
| **python compute / unattrib** | the leftover — the real "stood still for no reason" budget. |

A **TOP HAL / TRANSPORT CALLS** table shows per-method `n / total / mean / max`,
so e.g. a `Servo.set_position` that blocks 50 ms per call is immediately
visible. Calls ≥ `RACCOON_PROFILE_HAL_SPAN_MS` (default 1 ms) also get their own
span in the chrome trace (labelled `hal:Servo.set_position`); `sleep` spans are
added too.

### Relevant env vars

| Variable | Effect |
|---|---|
| `RACCOON_PROFILE` | master switch (path, or `1`/`true`). |
| `RACCOON_PROFILE_HAL` | `0`/`off` disables HAL-call timing (default on). |
| `RACCOON_PROFILE_SLEEP` | `0`/`off` disables sleep splitting (default on). |
| `RACCOON_PROFILE_HAL_SPAN_MS` | min call length (ms) to get its own trace span (default 1). |

(The existing `RACCOON_PROFILE_LOOP_LAG*`, `_TRACE`, `_REPORT`, `_TOP` knobs are
unchanged.)

> Note: on the host/mock bundle the HAL calls are near-instant. The HAL table is
> most useful on the **real robot**, where `set_*` cross the transport.

---

## 2. Command trace — send→receive ordering

Symptom this targets: *a servo that should move before another moves after it.*
Each servo port is its own transport channel with **no cross-channel ordering
guarantee**, and the reader drains channels in port order within a spin window —
so a command sent later can be applied earlier.

### How it works

* **Send side** (raccoon-lib, `CommandTrace`): set `RACCOON_CMD_TRACE` to a file
  path on the **robot** before the mission runs. Every published motor/servo/
  chassis command is appended as JSONL with a process-global monotonic `seq`
  (the *intended* order across all channels) and the wire `ts_us` timestamp.
* **Receive side** (stm32-data-reader, `CmdTrace`): set `WOMBAT_CMD_TRACE` to a
  file path before the reader starts. Each command is logged when its handler
  fires (`recv`) and when a servo position is staged to SPI (`spi`), with its own
  monotonic `rseq` and a system-clock `w_us`.

Both run on the same Pi, so `ts_us` is a shared clock — `(channel, ts_us)` is the
correlation key between the two streams.

### Capture (on the Pi)

The reader runs under systemd, so set its env there (or run it manually):

```bash
# Reader: add to the unit's Environment= or export before launching manually
sudo systemctl edit stm32_data_reader        # add: Environment=WOMBAT_CMD_TRACE=/tmp/recv.jsonl
sudo systemctl restart stm32_data_reader

# Mission run (shell env IS forwarded for a manual run; for run-configs put it there)
RACCOON_CMD_TRACE=/tmp/send.jsonl uv run raccoon run
```

> Reminder (from past debugging): the systemd service does **not** inherit your
> shell env — set `WOMBAT_CMD_TRACE` in the unit, not just your shell.

### Analyze (anywhere)

Pull both files and run the analyzer (stdlib-only, use any Python):

```bash
python3 scripts/analyze_cmd_trace.py \
    --send send.jsonl --recv recv.jsonl \
    --trace cmd-trace.json
```

It reports: commands sent/received, **dropped** commands (sent but never seen —
a transport overrun), send→recv latency percentiles, and **ordering
inversions** — with cross-servo inversions (the bug) called out explicitly and
the servo SPI apply order. The `--trace` file places send and recv on one shared
timeline (open at <chrome://tracing>) so crossing bars reveal a reordering at a
glance. `--fail-on-inversion` exits non-zero when a cross-servo inversion is
found (useful in a regression gate).

Both tracers are **inert unless their env var is set**, so there is no overhead
on a normal run.
