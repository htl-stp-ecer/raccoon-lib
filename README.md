<div align="center">

<img src="https://raw.githubusercontent.com/htl-stp-ecer/.github/main/profile/raccoon-logo.svg" alt="RaccoonLib" width="100"/>

# RaccoonLib

**The core robotics library powering RaccoonOS for Botball.**

PID motion control · Kinematics · Odometry · Step-based missions · Python bindings

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](COPYING)
![C++20](https://img.shields.io/badge/C%2B%2B20-00599C?logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.11+-3776AB?logo=python&logoColor=ffdd54)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-ARM64-A22846?logo=raspberrypi&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-KIPR%20Wombat-orange)

> 📖 **Full documentation at [raccoon-docs.pages.dev](https://raccoon-docs.pages.dev/)**

</div>

---

RaccoonLib is the heart of [RaccoonOS](https://github.com/htl-stp-ecer) — a full robotics platform built for [Botball](https://www.kipr.org/botball/) competition. It gives your robot a solid foundation: reliable motion, clean sensor abstractions, and a step-based mission system that makes autonomous code readable and maintainable.

Built from years of competition experience at HTL St. Pölten, it's designed so that new team doesn't have to start from zero.

---

## What's Inside

| Module | What it does |
|:-------|:-------------|
| **Motion Control** | PID-controlled `drive`, `turn`, `strafe` with feedforward compensation |
| **Calibration** | Auto-tune feedforward (kS, kV, kA) and PID via relay-feedback — no guessing |
| **Kinematics** | Differential and mecanum drivetrains — forward & inverse |
| **Odometry** | Real-time position and heading from wheel encoders |
| **Step Framework** | Sequential and parallel task execution — write missions like a recipe |
| **Sensors** | IR sensors, buttons, servos — clean, consistent API |
| **Python Bindings** | Full Python API via pybind11 — same power, less boilerplate |

---

## Quick Start

### 1. Clone & install

```bash
git clone https://github.com/htl-stp-ecer/raccoon-lib.git
cd raccoon-lib

python3 -m venv .venv
source .venv/bin/activate
pip install .
```

### 2. Drive your robot

```python
from libstp.robot import Robot

robot = Robot()

robot.drive.straight(500)   # 500 mm forward
robot.drive.turn(90)        # 90° right
robot.drive.strafe(200)     # 200 mm right (mecanum only)
```

### 3. Write a mission

```python
from libstp.step import Sequential, Parallel, Wait
from libstp.step.motion import Drive, Turn

mission = Sequential([
    Drive(500),
    Turn(90),
    Parallel([
        Drive(300),
        Wait(1.0),
    ]),
])

mission.run()
```

> **New to RaccoonOS?** Start with the [Getting Started guide](https://raccoon-docs.pages.dev/00-quick-start/) or clone [`raccoon-example`](https://github.com/htl-stp-ecer/raccoon-example) — a fully commented reference robot that demonstrates every concept.

---

## Deploy to Raspberry Pi (ARM64)

RaccoonLib runs on the [KIPR Wombat](https://www.kipr.org/kipr/hardware-software) controller (Raspberry Pi, ARM64). Cross-compilation is handled by Docker — no toolchain setup needed on your side.

```bash
# Build ARM64 wheel via Docker
bash build.sh
# Output: build-docker/*.whl

# Deploy to your Pi
RPI_HOST=<your-pi-ip> bash deploy.sh
```

| Variable | Default | Description |
|:---------|:--------|:------------|
| `RPI_HOST` | *(required)* | Raspberry Pi IP address |
| `RPI_USER` | `pi` | SSH username |
| `RPI_DIR` | `/home/pi/python-libs` | Install directory |
| `BUILD_TYPE` | `Release` | CMake build type |

---

## Architecture

```
High-Level APIs  (Robot, Step Framework, Python Bindings)
         ↓
Specialized Systems  (Calibration, Sensors, Timing)
         ↓
Core Control  (Drive, Motion, Kinematics, Odometry)
         ↓
Platform Implementation  (Wombat / Raspberry Pi)
         ↓
Hardware Abstraction Layer
```

---

## Requirements

- CMake >= 3.15
- C++20 compatible compiler
- Python >= 3.11
- Docker (for ARM64 cross-compilation)

**C++ dependencies:** Eigen3 3.4.0 · LCM 1.5.0 · spdlog 1.14.1 · pybind11 2.13.6  
**Python dependencies:** pyyaml · aiosqlite

---

## Testing

```bash
# C++ tests
cmake -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build

# Python tests
pytest tests/
```

---

## Part of RaccoonOS

RaccoonLib is one piece of the full platform:

| Repository | What it is |
|:-----------|:-----------|
| [raccoon-cli](https://github.com/htl-stp-ecer/raccoon-cli) | Dev toolchain — scaffolding, hardware wizard, `raccoon run` |
| [raccoon-example](https://github.com/htl-stp-ecer/raccoon-example) | Reference robot — start here if you're new |
| [raccoon-transport](https://github.com/htl-stp-ecer/raccoon-transport) | LCM messaging layer (C++, Python, Dart) |
| [documentation](https://raccoon-docs.pages.dev/) | Full platform docs |

---

## License

Copyright (C) 2026 Tobias Madlberger  
Licensed under the GNU General Public License v3.0 — see [COPYING](COPYING) for details.
