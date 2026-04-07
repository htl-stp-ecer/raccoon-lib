# Contributing to RaccoonLib

Thanks for wanting to contribute. This guide covers everything you need to add a new module, step, or fix to the library.

---

## Building & deploying

All compilation targets ARM64 (Raspberry Pi). Local installs are not supported -- use the Docker cross-compiler.

```bash
# Build wheel + run tests, then deploy to Pi
RPI_HOST=<your-pi-ip> bash deploy.sh

# Build only (output: build-docker/*.whl)
bash build.sh

# Skip tests during iteration
SKIP_TESTS=1 bash build.sh
```

Useful build env vars:

| Variable | Default | Description |
|:---------|:--------|:------------|
| `RPI_HOST` | *(required for deploy)* | Raspberry Pi IP |
| `RPI_USER` | `pi` | SSH username |
| `BUILD_TYPE` | `Release` | `Release` or `Debug` |
| `BUILD_JOBS` | auto | Parallel compile jobs |
| `SKIP_TESTS` | -- | Set to `1` to skip C++ tests |

---

## Project layout

```
library/
├── modules/                      # C++ modules (one folder each)
│   └── libstp-<name>/
│       ├── include/              # Public C++ headers
│       ├── src/                  # C++ sources
│       ├── bindings/             # pybind11 bindings (optional)
│       └── python/libstp/        # Python files shipped with this module
├── python/libstp/                # Global Python-only package files (minimal)
├── tests/
│   ├── cpp/                      # Google Test suites
│   └── python/                   # pytest suites
├── tools/                        # Code generators
│   └── generate_step_builders.py
├── build.sh                      # Docker cross-compile + codegen
└── deploy.sh                     # build.sh + install on Pi
```

Python code almost always lives inside the module it belongs to (`modules/libstp-<name>/python/libstp/`). The root `python/libstp/` folder is for the handful of things that don't belong to any specific module (package init, logging helpers, project YAML utilities).

---

## Adding a C++ module

### 1. Create the module directory

```
modules/libstp-mymodule/
├── CMakeLists.txt
├── include/mymodule/
│   └── my_class.hpp
├── src/
│   └── my_class.cpp
├── bindings/                 # only if you need Python bindings
│   ├── bindings.cpp
│   └── my_class.cpp
└── python/libstp/mymodule/   # Python files for this module (steps, helpers, etc.)
    └── __init__.py
```

### 2. CMakeLists.txt

One line:

```cmake
register_libstp_module(mymodule "FALSE" "foundation;drive")
#                       ^name    ^interface-only  ^dependencies
```

Set the second arg to `"TRUE"` for header-only modules (no `src/`).

### 3. Register in modules/CMakeLists.txt

Add after its dependencies:

```cmake
add_subdirectory(libstp-mymodule)
```

### 4. Python bindings (optional)

`bindings/bindings.cpp` -- entry point:

```cpp
#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_my_class(py::module_& m);

PYBIND11_MODULE(mymodule, m) {
    init_my_class(m);
}
```

`bindings/my_class.cpp` -- expose a class:

```cpp
#include <pybind11/pybind11.h>
#include "mymodule/my_class.hpp"
namespace py = pybind11;

void init_my_class(py::module_& m) {
    py::class_<MyClass>(m, "MyClass")
        .def(py::init<float>(), py::arg("param"))
        .def("compute", &MyClass::compute);
}
```

Export the symbol in `python/libstp/__init__.py` so users can import it directly from `libstp`.

---

## Adding a Python step

Steps are the building blocks users write missions with (`drive_forward`, `turn_right`, etc.). They live inside the module they belong to:

```
modules/libstp-<domain>/python/libstp/step/<domain>/my_step.py
```

Export it from the domain's `__init__.py`.

### Simple step

```python
from typing import TYPE_CHECKING

from libstp.step.annotation import dsl_step
from libstp.step.base import Step

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl_step(tags=["domain", "subcategory"])
class MyStep(Step):
    """One-line summary.

    Explain what it does and how -- which controllers or sensors it uses.

    Args:
        distance_cm: How far to travel, in centimetres.
    """

    def __init__(self, distance_cm: float) -> None:
        super().__init__()
        self._distance_cm = distance_cm

    def _generate_signature(self) -> str:
        return f"MyStep(distance_cm={self._distance_cm:.1f})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        # implementation
        pass
```

### Motion step (start / update / stop loop)

```python
from libstp.step.motion.motion_step import MotionStep

@dsl_step(tags=["motion", "drive"])
class MyMotion(MotionStep):
    def on_start(self, robot: GenericRobot) -> None:
        # set up controllers
        pass

    def on_update(self, robot: GenericRobot, dt: float) -> bool:
        # return True when done
        return False

    def on_stop(self, robot: GenericRobot) -> None:
        pass
```

### Builder codegen

`@dsl_step` decorated classes are picked up automatically by `tools/generate_step_builders.py`, which runs as part of `build.sh`. It generates the `*Builder` class and the lowercase factory function (`my_step(...)`) that users call in missions. You don't write these by hand.

---

## Code style

**C++** -- follow the existing `.clang-tidy` rules (bugprone, modernize, performance, readability checks are all active). Key conventions:
- Namespace: `libstp`
- Headers under `include/<module>/`, sources under `src/`
- Use `Result<T>` for error handling, not exceptions
- Prefer `std::shared_ptr` for shared ownership

**Python** -- the project uses [Ruff](https://docs.astral.sh/ruff/) and targets Python 3.11+:

```bash
pip install ruff
ruff check python/ modules/
ruff format python/ modules/
```

Keep `GenericRobot` imports behind `TYPE_CHECKING` to avoid circular imports.

---

## Testing

### C++ (Google Test)

```bash
cmake -B build -DLIBSTP_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Tests live in `tests/cpp/`. Follow the existing fixture pattern -- see `tests/cpp/kinematics/` for examples.

### Python (pytest)

```bash
pytest tests/python/
```

Requires the wheel installed. Deploy to the Pi first.

---

## Pull request checklist

- [ ] New C++ module registered in `modules/CMakeLists.txt`
- [ ] Bindings exported from `python/libstp/__init__.py`
- [ ] Module Python files placed under `modules/libstp-<name>/python/libstp/`
- [ ] New step has `@dsl_step(tags=[...])` and a docstring
- [ ] `_generate_signature()` implemented
- [ ] Tests added (C++ or Python as appropriate)
- [ ] `ruff check` passes with no errors
- [ ] `SKIP_TESTS=1 bash build.sh` completes without errors
