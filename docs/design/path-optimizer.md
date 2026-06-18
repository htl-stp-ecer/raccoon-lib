# Path Optimizer — `optimize()` Design

Status: **design / brainstorm locked** — supersedes `smooth_path()`, the `spline`
special-case, and the `abs_*` parallel pipeline.

## Motivation

Today the same idea is smeared across three half-overlapping things:

- `smooth_path()` — a function that is secretly a pipeline *configuration*
  (`optimize`, `corner_cut_cm`, `spline` flags), mixing DSL, compilation, and
  execution in one `SmoothPath` class.
- `spline` — exists twice: a standalone `SplinePath`/`spline()` step **and** a
  "terminal" compiler pass that bypasses the IR and short-circuits the executor.
- `flow` — not code at all; a docstring word for "velocity carried across
  segment boundaries".
- A whole **parallel** `abs_*` compiler (`abs_compiler`, `abs_desugar`,
  `abs_ir`, `abs_passes`, `compile_via_absolute_bridge`, `fallback_reason`)
  doing a half-finished relative→absolute migration.

The replacement is a single, explicit, composable pipeline exposed as a
**stream builder**: `optimize([...])`.

## Mental model: a consumed stream (Java-streams)

`optimize(steps)` builds a **lazy stream** of path nodes. Each chained method is
an *intermediate op* (returns the builder). The *terminal op* is execution (the
returned Step is run) or `.explain()`. Once consumed, the stream is spent.

```python
optimize(steps)                        # DEFAULT: nothing. Identical to seq([...]).
optimize(steps).warm()                 # warm-start is a PASS, not a default
optimize(steps).warm().merge().cut_corners(5)
optimize(steps).to_absolute()          # motion runs -> goto(x,y,θ) on the goto system
optimize(steps).warm().splinify()      # splinify is terminal
optimize(steps).apply(MyPass())        # custom pass
optimize(steps).explain()              # prints the pipeline trace, does not run
```

Adoption is a drop-in: every mission `return seq([...])` becomes
`return optimize([...])`. With no passes it is behaviourally identical, because
**the default does nothing**. Passes are added incrementally.

### Decisions (locked)

- **Ordering:** explicit. Call order = pass order. No auto-sort, no presets.
- **Validation:** compile-time, in `_add()` (see below).
- **Replace, don't alias:** `smooth_path`, the spline special-case, and the
  `abs_*` pipeline are deleted, not kept as shims.
- **`warm` is a pass.** "smooth" / "flow" disappear as names.
- **Defer = hard barrier.** A `defer()` resolves at runtime and runs
  **unoptimized**; passes do not re-run on the resolved sub-stream.
- **Custom opt-in = `lower_to_segments()`** (see "Self-describing steps").
- **Nested `optimize()` is supported** (see "Nesting").

## Self-describing steps: `lower_to_segments()`

The core principle — **nothing is hardcoded**. The optimizer never asks "what
kind of step is this?". It asks the step to lower itself:

```python
class Step:
    def lower_to_segments(self) -> list[Segment] | None:
        """Lower this step into optimizer IR segments.

        Return None (default) to remain an OPAQUE node — the optimizer treats
        it as a barrier and never transforms across it.
        """
        return None
```

This mirrors the **already-existing** `to_lnion_step() -> SimulationStep`
protocol (`modules/libstp-step/python/raccoon/step/model.py`,
`base.py`), which lowers a step into simulation/timing metadata:

```python
@dataclass
class SimulationStepDelta:   # "Estimated pose change caused by a step"
    forward: float           # m
    strafe:  float           # m
    angular: float           # rad

@dataclass
class SimulationStep:
    id: str; label: str | None
    average_duration_ms: float; duration_stddev_ms: float
    delta: SimulationStepDelta
```

The optimizer is simply a **second consumer of the same "steps describe
themselves" idea**. `lower_to_segments()` lives next to `to_lnion_step()`.

Note the synergy: `SimulationStepDelta` *is* the pose integration that
`to_absolute()`/`goto()` need. A step that can report a deterministic delta is
exactly a step with a `known_endpoint`.

### Two node kinds fall out automatically

- **Transparent** — `lower_to_segments()` returned segments. Geometry passes may
  touch them.
- **Opaque** — returned `None`. The optimizer preserves them verbatim; they are
  barriers per their declared capabilities.

Built-in `drive`/`turn`/`arc`/`spline` implement `lower_to_segments()`. A custom
step opts into optimization by implementing it; otherwise it is safely opaque by
default (e.g. drumbot's `CollectDrumsStep`, an internal state machine — stays a
barrier with zero special-casing).

## Node capabilities

Each node carries caps that **come from the node**, not the pass. A pass walks
the stream and respects them; it never needs to understand what a node *is*.

```python
class NodeCaps:
    is_motion: bool          # transparent to geometry passes
    known_endpoint: bool     # pose statically integrable? (.until() -> False)
    truncatable: bool        # may end early? (timeout() -> True)
    is_barrier: bool         # no pass may merge/reorder across it
    blocks_flow: bool        # interrupts warm-start? (wait_for_* True; background False)
    mutates_state: bool      # service mutation -> no reorder
```

Every geometry pass works the same way: **operate on maximal runs of compatible
transparent segments, flush at every barrier.** A pass never special-cases an
edge case — the caps do the work.

## Passes

```python
class PathPass(Protocol):
    name: str
    requires: Representation = EITHER   # RELATIVE | ABSOLUTE | EITHER
    produces: Representation = SAME
    terminal: bool = False

    def run(self, plan: Plan, ctx: CompileContext) -> Plan: ...
```

| Builder method        | Pass                | Notes |
|-----------------------|---------------------|-------|
| `.warm()`             | `WarmPass`          | mark warm-start transitions between adjacent motion |
| `.merge()`            | `MergePass`         | merge adjacent same-axis segments (exists today) |
| `.cut_corners(radius_cm=)` | `CornerCutPass` | replace linear+turn+linear → linear+arc+linear (exists today) |
| `.absolute_heading()` | `AbsoluteHeadingPass` | per-segment: regulate against absolute heading reference (cheap, new) |
| `.to_absolute()`      | `ToAbsolutePass`    | integrate pose over known runs → `goto(x,y,θ)` (replaces `abs_*`) |
| `.splinify(...)`      | `SplinePass`        | terminal: replace run → `Segment(kind="spline")` |
| `.apply(p)`           | any                 | custom |

`absolute_heading()` (cheap per-segment flag) and `to_absolute()` (heavy pose
rewrite) are **separate passes** — you often want only the former (drift
robustness) without the full goto rewrite.

### The builder `_add()` — compile-time validation

```python
def _add(self, p):
    if self._terminated_by:
        raise PathBuildError(f"{p.name}() after terminal {self._terminated_by}()")
    if p.requires not in (self._repr, EITHER):
        raise PathBuildError(
            f"{p.name}() needs {p.requires} segments, stream is {self._repr} "
            f"— {_hint(p, self._repr)}")
    self._passes.append(p)
    if p.produces != SAME: self._repr = p.produces
    if p.terminal: self._terminated_by = p.name
    return self
```

`_add()` validates **pass composition** (order, representation, terminality).
Each pass validates **node composition** (caps) at run time. Clean split.

Examples of errors caught at build time:

```
optimize(s).splinify().merge()
  -> PathBuildError: merge() after terminal splinify()

optimize(s).to_absolute().cut_corners(5)
  -> PathBuildError: cut_corners() needs RELATIVE segments, stream is ABSOLUTE
                     — put cut_corners() before to_absolute()
```

This replaces today's runtime `ValueError` minefield (`corner_cut_cm and
spline=True are mutually exclusive`, `fallback_reason`, etc.).

## `to_absolute()` + `goto()`

`goto(x, y, theta)` is a **new motion primitive** that drives to an absolute pose
using the goto navigation system (plan to a pose, not chain relatives —
otherwise the conversion is pointless).

`ToAbsolutePass` integrates pose over a **run of known-endpoint relative
segments** and emits a `goto` node:

```python
optimize([drive_forward(100), turn_right(90), drive_forward(50)]).to_absolute()
#   -> goto(x=1.0, y=-0.5, theta=-90°)
```

Constraint falls straight out of the caps model: at a `.until()` move the
endpoint is unknown → pose integration breaks → the pass flushes the run there.
It converts contiguous known runs and leaves the rest. No special case.

## Nesting: `optimize()` inside `optimize()`

A nested `optimize(...)` is a **sealed sub-stream splice**:

- Its own passes have already run; they do **not** re-run.
- Its compiled nodes are spliced into the parent stream as a run.
- Parent passes may further touch those nodes per their caps.
- Its terminality is scoped to the inner builder — the parent may keep building.

```python
optimize([
    optimize([drive, turn, drive]).splinify(),   # inner: this part becomes a spline
    turn(90),
    drive(50),
]).warm()                                        # outer warms across the whole thing
```

This lets you splinify (or otherwise terminally transform) a *sub-path* and keep
composing around it.

## `parallel()` handling

`parallel(motion_spine, side_branch)` is common (clawbot m020/m030).

- If a **single continuous motion spine** is detectable (one branch lowers to a
  contiguous motion run, others are non-motion), expose the spine as an
  optimizable sub-stream; pin the side branches.
- Otherwise (multiple motion branches, or no continuous path), treat the whole
  `parallel()` as an **opaque motion barrier**.

## `explain()`

Because passes are real stream transforms, `explain()` prints the stream after
each stage **and why a pass stopped at a node**:

```
optimize([...]).warm().merge().cut_corners(5).explain()

stream (6 nodes, repr=RELATIVE):
  raw:
    [0] linear  fwd 100cm
    [1] turn    R 90°
    [2] ║barrier║ switch_calibration_set("upper")
    [3] linear  fwd 80cm  .until(on_black)      known_endpoint=False
    [4] background "drop_cone"                   blocks_flow=False
    [5] linear  fwd 50cm
  warm:    warmed 0→1, 4→5   (stop at [2] barrier, [3] unknown-end)
  merge:   no merge across [2]; [3] not mergeable (unknown end)
  cut_corners(5): 0→1 → arc(r=5); [3..5] skipped
result: linear(95) arc(r5,90°) ║cal║ linear(80).until ▸bg linear(50)
```

## Real-world edge cases (from Ecer2026 missions) → caps

Catalogued from 27 mission files across clawbot/conebot/drumbot/cube-bot/
packingbot. Each is *just* a caps combination — no optimizer special-casing.

| Pattern (example) | Caps | Effect |
|---|---|---|
| `switch_calibration_set("upper")` (clawbot m010/m050) | `is_barrier` | no merge/splinify across it |
| `wait_for_checkpoint(68)` (clawbot m030) | `is_barrier, blocks_flow` | preserved; warm-flow breaks here |
| `defer(...)` / `after_collect()` (drumbot m020) | `is_barrier, known_endpoint=False` | resolves at runtime, runs unoptimized |
| `drive_backward().until(on_black())` (everywhere) | `is_motion, known_endpoint=False` | `warm()` ok; merge/cut/splinify/to_absolute stop |
| `timeout(strafe().until(), 4s)` (packingbot m070) | `is_motion, truncatable` | geometry passes treat as unknown end |
| `background(seq([...]), name="drop_cone")` (clawbot m020) | `is_motion=False, blocks_flow=False` | pinned at transition; warm-flow not broken |
| `wait_for_background("return_tray")` (clawbot m040) | `blocks_flow, is_barrier` | preserved, no reorder; cross-mission sync intact |
| `parallel(line_follow, seq([servo...]))` (clawbot m020) | spine node | spine optimized if continuous, else opaque barrier |
| custom `CollectDrumsStep` (drumbot) | opaque (no `lower_to_segments`) | untouched by default |
| `arm.move_angles()` between drives (clawbot m010) | `SideAction, is_motion=False` | pinned at transition point |

## What gets deleted

- `modules/libstp-motion/.../motion/smooth_path.py` (`SmoothPath` + `smooth_path()`).
- `abs_compiler.py`, `abs_desugar.py`, `abs_ir.py`, `abs_passes.py`,
  `compile_via_absolute_bridge`, `fallback_reason` → one `ToAbsolutePass`.
- The spline special-case in the executor (`if self._spline_step is not None`):
  `splinify` becomes a normal terminal pass writing `Segment(kind="spline")`;
  the executor gets **one** path.
- `smooth_path()` call sites in missions (clawbot m040/m050) →
  `optimize(...).warm()...`.

Kept: `SplinePath`/`spline()` as the explicit-waypoint step (`splinify()` builds
it internally). One spline implementation, two entry points.

## Open items

- Exact `goto()` semantics and which planner backs it.
- `parallel()` spine detection heuristic (v1 may ship "opaque unless trivially
  one motion branch").
- Whether `to_lnion_step()` and `lower_to_segments()` should share a single
  lowering visit (they compute overlapping pose info).
- Naming: the existing `to_lnion_step` vs. `SimulationStep` inconsistency in the
  current code should be reconciled while we are here.
