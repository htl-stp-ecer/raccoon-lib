# Calibration Module

Comprehensive motor and motion calibration system for the libstp robotics library.

## Quick Overview

This module provides **two independent calibration systems**:

### 🔧 Motor Calibration (`motor/`)
Calibrates **individual motors** for precise velocity control by measuring feedforward parameters (kS, kV, kA) and tuning PID gains.

### 🎯 Motion Calibration (`motion/`)
Calibrates **high-level motions** (turn, drive, strafe) by tuning motion controller PID gains for smooth, accurate behavior.

## Features

### Motor Calibration
- ✅ Automatic feedforward parameter identification
- ✅ Two PID tuning methods (step response, relay feedback)
- ✅ Comprehensive validation and safety monitoring
- ✅ 20+ small, focused components (~75 lines each)
- ✅ Easy to test and extend

### Motion Calibration
- ✅ Turn, drive straight, and strafe calibration
- ✅ Nelder-Mead optimization for no-overshoot tuning
- ✅ Advanced trajectory metrics (ITAE, jerk, settling time)
- ✅ Cross-validation across speeds and distances
- ✅ Modular design ready for expansion

## Usage

### Motor Calibration Example

```cpp
#include <calibration/motor/calibration.hpp>

// Create a motor
hal::motor::Motor motor(0);

// Create calibrator with default config
libstp::calibration::MotorCalibrator calibrator(motor);

// Run calibration
auto result = calibrator.calibrate();

if (result.success) {
    std::cout << "Motor calibrated successfully!\n";
    std::cout << "kS: " << result.ff.kS << "\n";
    std::cout << "kV: " << result.ff.kV << "\n";
    std::cout << "kA: " << result.ff.kA << "\n";
    std::cout << "PID: " << result.pid.kp << ", "
              << result.pid.ki << ", " << result.pid.kd << "\n";
}
```

### Motion Calibration Example

```cpp
#include <calibration/motion_calibration.hpp>

// Assuming you have drive and odometry objects
libstp::calibration::MotionCalibrator calibrator(drive, odometry);

// Calibrate all motions
auto result = calibrator.calibrate();

// Or calibrate individually
auto turn_result = calibrator.calibrateTurnMotion();
auto drive_result = calibrator.calibrateDriveStraightMotion();
```

## Project Structure

```
calibration/
├── motor/              # Individual motor calibration (20+ files)
│   ├── utils/          # Math and timing utilities
│   ├── control/        # Motor control interface
│   ├── data/           # Data recording
│   ├── analysis/       # Signal analysis
│   ├── feedforward/    # kS, kV, kA calibration
│   ├── pid/            # PID tuning methods
│   └── validation/     # Result validation
│
└── motion/             # Motion primitive calibration (4+ files)
    ├── analysis/       # Trajectory analysis
    ├── test/           # Test execution (future)
    ├── optimization/   # Optimization algorithms (future)
    └── tuners/         # Motion-specific tuners (future)
```

## Documentation

- **[STRUCTURE.md](STRUCTURE.md)** - Detailed structure and component overview
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - System architecture and design
- **[REFACTORING_SUMMARY.md](REFACTORING_SUMMARY.md)** - Refactoring details

## Key Design Principles

1. **Separation**: Motor and motion calibration are completely independent
2. **Modularity**: 24+ small files instead of 2 large monoliths
3. **Single Responsibility**: Each file does ONE thing well
4. **Testability**: Easy to unit test individual components
5. **Extensibility**: Simple to add new calibration methods

## Configuration

### Motor Calibration Config

```cpp
libstp::calibration::CalibrationConfig config;
config.use_relay_feedback = false;  // Use step response (conservative)
config.velocity_test_commands = {10.0, 15.0, 20.0};  // Test speeds (%)
config.max_calibration_duration = 120.0;  // Timeout (seconds)

libstp::calibration::MotorCalibrator calibrator(motor, config);
```

### Motion Calibration Config

```cpp
libstp::calibration::MotionCalibrationConfig config;
config.target_settling_time = 1.5;  // Target settling time (s)
config.max_overshoot = 0.15;        // Max overshoot (15%)
config.max_iterations = 10;         // Optimization iterations

libstp::calibration::MotionCalibrator calibrator(drive, odometry, config);
```

## Dependencies

- `foundation` - Core types (PidGains, Feedforward, logging)
- `hal` - Motor hardware interface
- `drive` - Motor adapter and velocity control
- `motion` - Motion primitives (turn, drive, strafe)
- `odometry` - Position and heading tracking

## Building

The calibration module is automatically built as part of libstp:

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

## Python Bindings

Python bindings are available when `BUILD_PYTHON_BINDINGS=ON`:

```python
import libstp.calibration as cal

# Motor calibration
motor = libstp.hal.Motor(0)
calibrator = cal.MotorCalibrator(motor)
result = calibrator.calibrate()

print(f"Success: {result.success}")
print(f"kS: {result.ff.kS}, kV: {result.ff.kV}, kA: {result.ff.kA}")
```

## Performance

### Before Refactoring
- 2 monolithic files (~1,940 lines)
- Mixed concerns
- Hard to test
- Difficult to extend

### After Refactoring
- 24+ focused files (50-150 lines each)
- Clear separation
- Easy to test
- Simple to extend

## Contributing

When adding new features:

1. **Motor calibration**: Add to appropriate `motor/` subdirectory
2. **Motion calibration**: Add to appropriate `motion/` subdirectory
3. Keep files small (< 200 lines)
4. One responsibility per file
5. Update CMakeLists.txt
6. Add to documentation

## License

Part of the libstp robotics library.

## Authors

- Tobias Schoch (@tobias)
- Refactored with Claude Code (2025)
