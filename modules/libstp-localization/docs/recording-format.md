# Localization Recording Format (RLREC v1)

Plain **JSONL** stream produced by `libstp::localization::LocalizationRecorder`
and consumed by the Web-IDE replay panel. One JSON object per line, UTF-8,
no trailing comma. Designed for **post-run replay** only — no streaming, no
truncation-recovery wire format. Simplicity over cleverness.

This file is the contract between recorder and player. Bump
`format_version` in the header line before changing field semantics.

## Why JSONL (and not binary)

- Debuggable with `cat | jq` and a text editor.
- Trivial to write from C++ (a `std::ofstream` and a fixed format string).
- HTTP gzip on the IDE-backend response handles transport compression; the
  on-disk file stays human-inspectable.
- A 1-minute run at 20 Hz with 128 particles → ~20–30 MB uncompressed,
  ~3–4 MB after HTTP gzip. Acceptable for a debug feature.

## File Structure

```
{ header line  }\n
{ frame line 0 }\n
{ frame line 1 }\n
...
{ frame line N }\n
```

A consumer reads line-by-line; a partial last line (process killed) is
silently dropped.

## Header Line

The first line. Required fields:

```jsonc
{
  "kind": "header",
  "format_version": 1,
  "started_at_unix_ns": 1716489600000000000,
  "tick_hz": 100.0,                  // nominal filter rate
  "record_hz": 20.0,                 // actual recording rate (after downsampling)
  "particle_count": 128,             // upper bound; per-frame may be lower
  "units": {                         // explicit so consumers don't guess
    "position": "m",
    "heading": "rad",
    "sensor_offset": "cm"
  },
  "robot": {                         // snapshot for the renderer
    "width_cm": 22.0,
    "length_cm": 18.0,
    "sensors": [
      {"name": "front_line", "kind": "line", "forward_cm": 8.0, "strafe_cm": 0.0},
      {"name": "left_wall",  "kind": "wall", "forward_cm": 4.0, "strafe_cm": 9.0}
    ]
  },
  "table_map": null,                 // optional embedded map snapshot; null if unavailable
  "notes": ""                        // optional, free text
}
```

Unknown keys MUST be ignored.

## Frame Line

One per recorded tick. All numeric units match `header.units`.

```jsonc
{
  "kind": "frame",
  "t_ns": 12345678,                  // since header.started_at_unix_ns
  "pose": [0.512, 1.203, 0.785],     // [x_m, y_m, heading_rad] — filter estimate
  "sigma": [0.01, 0.01, 0.005],      // [σx_m, σy_m, σθ_rad] — estimate stddev
  "odom_delta": [0.002, 0.0, 0.001], // per-tick odometry delta; [] if not captured
  "particles": [                     // length ≤ header.particle_count
    [0.510, 1.200, 0.780, 0.0091],   // [x_m, y_m, heading_rad, weight]
    [0.515, 1.205, 0.790, 0.0083]
    // ...
  ],
  "observations": [                  // 0 entries on most ticks; populated at resync
    {
      "surface_kind": "line",        // "line" | "wall"
      "detected": true,
      "sensor_offset_cm": [8.0, 0.0],     // [forward_cm, strafe_cm]
      "sigma_cm": 1.5,
      "measured_distance_cm": null,       // null if N/A (line sensor)
      "pose": [0.500, 1.200, 0.785],      // observation.pose (resync pose, may be null)
      "pose_sigma": [0.02, 0.02, 0.01]    // observation.sigma per axis
    }
  ],
  "resampled": false                 // true if this tick triggered low-variance resample
}
```

For per-tick recording the typical `observations` length is 0 — observations
only arrive at resync points. Snapshots written at a resync include that
tick's observations so the visualizer can highlight sensor hits exactly where
they were fed into the filter.

> **Scope note**: This format captures what the filter *integrated*, not
> continuous raw sensor traces. If a separate "show me every line-sensor
> reading even when ignored" feature is wanted, it belongs in its own log
> stream, not in this format.

## Sizing & Rate

At default config (`record_hz=20`, 128 particles, 0 obs/tick):

```
particles:    128 × ~40 chars = ~5 KB
pose/sigma:   ~100 chars
frame total:  ~5.5 KB
per second:   ~110 KB
per minute:   ~6.5 MB
gzipped:      ~1.5 MB / minute
```

Acceptable for a debug-only feature. If files get larger than this in
practice, we can switch to MessagePack later without rethinking the schema.

## Recorder Threading Contract

The C++ recorder runs *alongside* `Localization::tickLoop` and MUST NOT block
the filter. Required behavior:

1. **Snapshot under lock.** Copy the particles + estimate + pending
   observations into a plain frame struct while holding the filter's mutex.
   Do NOT touch the filesystem here.
2. **Hand off via SPSC queue.** Push the snapshot into a bounded
   producer/consumer queue (`std::deque<Frame>` + condvar, or
   `boost::lockfree::spsc_queue`). If the queue is full, drop the oldest
   frame and log a warning — file I/O backpressure must never stall the
   tick loop.
3. **Writer thread.** A dedicated thread pops frames, serializes to JSON,
   writes a line, optionally flushes. Joined on `stop_recording()` or in
   the destructor.

Hold the mutex for ≤ a memcpy of `particle_count × 32 B` ≈ 4 KB. That is
the entire steady-state work the recorder adds to the tick loop.

## Recording Activation

Driven from environment variables read **once** when `Localization` is
constructed (so end-user code doesn't change):

```
LIBSTP_RECORD_LOCALIZATION=1       # enable
LIBSTP_RECORDING_PATH=<file>       # absolute path on the device
LIBSTP_RECORDING_HZ=<float>        # downsample target (default 20.0)
```

If `LIBSTP_RECORDING_PATH` points to a non-writable location, the recorder
logs a warning and disables itself — no exception. Localization stays
functional; only the debug recording is lost.

## File Naming Convention

Toolchain CLI writes to:

```
<project_root>/.runs/<UTC-ISO-timestamp>/localization.jsonl
```

`UTC-ISO-timestamp` format: `20260523T143012Z` (no separators). The IDE
backend lists `.runs/*/localization.jsonl` and exposes each as a run.

For remote (Pi) runs the CLI pulls this file back over SFTP after the run
finishes.
