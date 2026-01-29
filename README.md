# LibStp

A modern robotics library for Botball robots, built with C++20 and Python. Provides high-performance motion control, calibration systems, and sensor integration with accessible Python bindings.

## Features

- **Motion Control** - PID-controlled movement primitives (drive, turn, strafe) with feedforward compensation
- **Motor Calibration** - Automatic feedforward (kS, kV, kA) and PID tuning via relay-feedback methods
- **Kinematics** - Differential and mecanum drivetrain support with forward/inverse kinematics
- **Odometry** - Real-time position and heading tracking via wheel encoders
- **Sensor Integration** - IR sensors, buttons, and servos
- **Step Framework** - Sequential and parallel task execution for autonomous missions
- **Cross-Platform** - Native ARM64 support with Docker-based cross-compilation for Raspberry Pi

## Requirements

- C++20 compatible compiler
- CMake >= 3.15
- Python >= 3.11
- Docker (for ARM64 cross-compilation)

## Installation

### Local Development

```bash
git clone https://gitlab.com/da253/library.git
cd library

python3 -m venv .venv
source .venv/bin/activate
pip install .
```

### Building for Raspberry Pi (ARM64)

Build the wheel using Docker:

```bash
bash build.sh
```

The wheel is output to `build-docker/*.whl`.

### Deploying to Raspberry Pi

```bash
RPI_HOST=10.101.156.206 bash deploy.sh
```

This builds the wheel, copies it to the Pi, and installs it via pip.

**Environment Variables:**
| Variable | Default | Description |
|----------|---------|-------------|
| `RPI_HOST` | (required) | Raspberry Pi IP address |
| `RPI_USER` | `pi` | SSH username |
| `RPI_DIR` | `/home/pi/python-libs` | Installation directory |
| `BUILD_TYPE` | `Release` | CMake build type |

## Usage

### Basic Robot Control

```python
from libstp.robot import Robot

robot = Robot()

# Drive forward 500mm
robot.drive.straight(500)

# Turn 90 degrees
robot.drive.turn(90)

# Strafe right 200mm (mecanum only)
robot.drive.strafe(200)
```

### Motor Calibration

```python
from libstp.step.calibration import calibrate_motors

# Run calibration with default settings
results = calibrate_motors()

# Results contain PID and feedforward gains
print(f"Left motor - kP: {results.left.kp}, kV: {results.left.kv}")
print(f"Right motor - kP: {results.right.kp}, kV: {results.right.kv}")
```

### Step-Based Missions

```python
from libstp.step import Sequential, Parallel, Wait
from libstp.step.motion import Drive, Turn

# Define a mission as a sequence of steps
mission = Sequential([
    Drive(500),           # Drive forward 500mm
    Turn(90),             # Turn right 90 degrees
    Parallel([            # Execute in parallel
        Drive(300),
        Wait(1.0),
    ]),
])

mission.run()
```

## Architecture

```
Hardware Abstraction Layer (HAL)
            ↓
Platform Implementations (Wombat)
            ↓
Core Control (Drive, Motion, Kinematics, Odometry)
            ↓
Specialized Systems (Calibration, Sensors, Timing)
            ↓
High-Level APIs (Robot, Step Framework, Python Bindings)
```

## Project Structure

```
library/
├── modules/                  # C++ modules with Python bindings
│   ├── libstp-foundation/    # Core types, PID, feedforward
│   ├── libstp-hal/           # Hardware abstraction layer
│   ├── libstp-platforms/     # Platform implementations (Wombat)
│   ├── libstp-kinematics/    # Drivetrain kinematics
│   ├── libstp-drive/         # Motor velocity control
│   ├── libstp-motion/        # High-level motion primitives
│   ├── libstp-odometry/      # Position tracking
│   ├── libstp-calibration/   # Motor & motion calibration
│   ├── libstp-sensor-ir/     # IR sensor interface
│   ├── libstp-button/        # Button input handling
│   ├── libstp-servo/         # Servo control
│   ├── libstp-timing/        # Timing utilities
│   ├── libstp-step/          # Step execution framework
│   ├── libstp-mission/       # Mission planning
│   └── libstp-robot/         # Central robot API
├── python/                   # Python package root
├── examples/                 # Example scripts
├── tests/                    # Test suites
├── docs/                     # Documentation
├── build.sh                  # Docker build script
├── deploy.sh                 # Raspberry Pi deployment
└── Dockerfile                # ARM64 build container
```

## Dependencies

**C++ Libraries:**
- Eigen3 3.4.0 - Linear algebra
- LCM 1.5.0 - Inter-process communication
- SPDLOG 1.14.1 - Logging
- pybind11 2.13.6 - Python bindings

**Python Packages:**
- pyyaml - Configuration
- aiosqlite - Async database support

## Documentation

Build the documentation:

```bash
cd docs
pip install -r requirements.txt
make html
```

## Testing

Run C++ tests:

```bash
cmake -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build
```

Run Python tests:

```bash
pytest tests/
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

[pybind11]: https://pybind11.readthedocs.io
[scikit-build-core]: https://scikit-build-core.readthedocs.io
