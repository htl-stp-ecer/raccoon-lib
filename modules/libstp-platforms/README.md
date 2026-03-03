# libstp-platforms

`libstp-platforms` supplies the concrete method definitions for the HAL classes declared in [`libstp-hal`](../libstp-hal/README.md). It does not define a new public device API. Instead, it decides which implementation bundle is exposed as the CMake target `platform`.

## How driver bundle selection works

The top-level `CMakeLists.txt` in this module defines two helper functions:

- `register_driver(<name>)`: build one driver library from the current subdirectory and remember it in a global property.
- `register_platform(<bundle>)`: create `platform_<bundle>` as an interface target that links every registered driver, then expose `platform` as an alias to it.

Bundle selection happens through the cache variable:

```cmake
set(DRIVER_BUNDLE "wombat" CACHE STRING "Which driver bundle to use (e.g. wombat, esp32, mock)")
add_subdirectory(${DRIVER_BUNDLE})
```

In this repository snapshot, the in-tree bundles are:

- `wombat`
- `mock`

`esp32` is still listed in the cache string, but there is no `modules/libstp-platforms/esp32` directory in this tree.

Example configure commands:

```bash
cmake -S . -B build -DDRIVER_BUNDLE=mock
cmake -S . -B build -DDRIVER_BUNDLE=wombat
```

## Bundle layout

Each bundle follows the same rough structure:

- `core/`: bundle-specific helper library exported as `platform_core`
- `analog/`
- `digital/`
- `imu/`
- `motor/`
- `servo/`
- optional extras such as `screen/` for wombat

Each leaf driver directory usually contains:

- `src/*.cpp` with the method bodies for the corresponding HAL wrapper
- `CMakeLists.txt` calling `register_driver(...)`

The driver library always links:

- `hal`
- `platform_core`

That keeps HAL headers stable while allowing each bundle to depend on very different runtime machinery.

## Public surfaces contributors extend

These are the main extension points inside this module:

- `mock/core/include/core/MockPlatform.hpp`: the fake backend used by the mock bundle.
- `wombat/core/include/core/LcmReader.hpp`: cached sensor reader backed by raccoon transport subscriptions.
- `wombat/core/include/core/LcmWriter.hpp`: command publisher for motors, servos, and shutdown control.

The actual device classes (`libstp::hal::motor::Motor`, `libstp::hal::imu::IMU`, and so on) remain declared in `libstp-hal`; this module only provides their implementations.

## Dependencies

Bundle-specific dependencies differ:

- `mock` only depends on the HAL surface and its own in-process `MockPlatform`.
- `wombat` links `platform_core` against `raccoon::transport` and `foundation`.
- `wombat/screen` also links `raccoon::transport` and `foundation`.

If a new bundle needs third-party libraries, add them to that bundle's `core` or individual driver target instead of widening the dependency surface for every platform.

## Testing and verification

This module does not define dedicated tests under `modules/libstp-platforms`.

Current contributor workflow is usually:

1. Build with `-DDRIVER_BUNDLE=mock` first to catch link errors and exercise the HAL against a deterministic in-process backend.
2. Run repo-level binding or integration tests that import `libstp.hal`.
3. Build with `-DDRIVER_BUNDLE=wombat` when changing raccoon transport wiring or wombat-specific behavior.

The `mock` bundle is the most practical target for local verification because it has no external transport dependency.

## Adding a new platform bundle

To add a new bundle such as `foo`:

1. Create `modules/libstp-platforms/foo/CMakeLists.txt`.
2. Add a `core` directory that builds `platform_core`.
3. Add driver subdirectories for every HAL class the bundle supports.
4. In each driver directory, implement the methods declared in `libstp-hal/include/hal/*.hpp`.
5. Call `register_driver(...)` from each driver directory.
6. Call `register_platform(foo)` after all driver subdirectories are added.
7. Update the `DRIVER_BUNDLE` cache string list so the new bundle shows up in CMake GUIs.

The bundle must provide implementations for every HAL symbol that downstream modules link against, otherwise the final link will fail.

## Adding or changing a driver inside an existing bundle

When extending one device area in a bundle:

1. Confirm the HAL declaration already exists in `libstp-hal`.
2. Add or update the matching `.cpp` implementation under the bundle's driver directory.
3. Keep transport helpers and cache state in `core/` if multiple drivers share them.
4. Register the driver target through that directory's `CMakeLists.txt`.
5. Rebuild at least one consumer that links `platform`.

If the change introduces a new cross-driver concept, prefer adding it to `platform_core` rather than duplicating state in multiple drivers.
