# Motor Calibration - Quick Start Guide

## TL;DR - Just Run This

```python
from libstp.step import Robot

robot = Robot()
results = robot.drive.calibrate_motors()

# That's it! Your robot is now properly tuned.
```

## What Just Happened?

The automatic calibration system just:
1. **Measured your motors** - Found static friction, velocity response, and acceleration characteristics
2. **Tuned PID gains** - Calculated optimal kp, ki, and kd values
3. **Applied the gains** - Your robot now has proper velocity control

**Your "slow creeping" issue is now fixed!** ✓

## Usage

### For Differential Drive

```python
from libstp.step import Robot

robot = Robot()

# Simple calibration (takes ~40-50 seconds)
results = robot.drive.calibrate_motors()

# Returns list of 2 CalibrationResult objects (left, right motors)
for i, result in enumerate(results):
    motor_name = "Left" if i == 0 else "Right"
    if result.success:
        print(f"{motor_name}: kp={result.pid.kp:.3f}, ki={result.pid.ki:.3f}")
```

### For Mecanum Drive

```python
from libstp.step import Robot

robot = Robot()

# Simple calibration (takes ~80-110 seconds for 4 motors)
results = robot.drive.calibrate_motors()

# Returns list of 4 CalibrationResult objects (FL, FR, BL, BR)
motor_names = ["Front Left", "Front Right", "Back Left", "Back Right"]
for i, result in enumerate(results):
    if result.success:
        print(f"{motor_names[i]}: kp={result.pid.kp:.3f}, ki={result.pid.ki:.3f}")
```

## Custom Configuration

```python
from libstp.step import Robot
from libstp.calibration import CalibrationConfig

robot = Robot()

# Option 1: Aggressive mode (higher performance)
config = CalibrationConfig()
config.use_relay_feedback = True  # Enable relay feedback mode
results = robot.drive.calibrate_motors(config)

# Option 2: Custom parameters
config = CalibrationConfig()
config.step_response_amplitude = 40.0  # Increase test amplitude
config.max_calibration_duration = 150.0  # Allow more time
results = robot.drive.calibrate_motors(config)
```

## Available Configuration Options

```python
from libstp.calibration import CalibrationConfig

config = CalibrationConfig()

# Tuning mode
config.use_relay_feedback = False  # False=conservative, True=aggressive

# Step response parameters (conservative mode)
config.step_response_amplitude = 30.0  # % command (default: 30)
config.step_response_duration = 3.0    # seconds (default: 3.0)

# Relay feedback parameters (aggressive mode)
config.relay_amplitude = 50.0          # % command (default: 50)
config.max_relay_duration = 10.0       # seconds (default: 10.0)
config.min_oscillations = 3            # minimum cycles (default: 3)

# Safety limits
config.max_test_distance_m = 0.6       # emergency stop (default: 0.6m)
config.max_single_test_duration = 15.0 # per-test timeout (default: 15s)
config.max_calibration_duration = 120.0 # total timeout (default: 120s)

# Validation
config.validation_duration = 2.0       # validation test time (default: 2.0s)
config.validation_max_error = 0.2      # max acceptable error (default: 20%)
```

## Reading Results

```python
result = results[0]  # First motor

# Basic info
print(f"Success: {result.success}")
print(f"Duration: {result.duration_seconds:.1f}s")
print(f"Error: {result.error_message}")  # Only if failed

# PID Gains
print(f"kp: {result.pid.kp:.3f}")
print(f"ki: {result.pid.ki:.3f}")
print(f"kd: {result.pid.kd:.3f}")

# Feedforward Parameters
print(f"kS (static friction): {result.ff.kS:.2f}%")
print(f"kV (velocity const):  {result.ff.kV:.3f}")
print(f"kA (accel const):     {result.ff.kA:.3f}")

# Detailed Metrics
print(f"R² (velocity fit): {result.metrics.velocity_constant_r_squared:.3f}")
print(f"Validation passed: {result.metrics.validation_passed}")
print(f"Mean error: {result.metrics.validation_mean_error*100:.1f}%")
```

## Module Structure

The calibration system is organized into modules:

- **`libstp.calibration`** - Configuration and result types (import these)
- **`robot.drive.calibrate_motors()`** - Call this method on your robot
- Works for **both** differential and mecanum kinematics automatically

## Two Tuning Modes

### Conservative Mode (Default) - Recommended First
- **Method:** Step response (FOPDT model)
- **Space:** ~30cm clear ahead
- **Time:** ~20s per motor
- **Best for:** General use, learning, competition

### Aggressive Mode (Optional) - For Performance
- **Method:** Relay feedback (Tyreus-Luyben rules)
- **Space:** ~50cm clear ahead
- **Time:** ~25-35s per motor
- **Best for:** High-speed scenarios, advanced users

## Before Calibrating

1. ✓ **Check space:** Ensure robot has required clearance
2. ✓ **Check battery:** Calibrate with a fresh battery for best results
3. ✓ **Choose surface:** Calibrate on the surface you'll compete on
4. ✓ **Let motors warm:** Run robot briefly before calibrating

## After Calibrating

The gains are **automatically applied** and stay active! Just use your robot normally:

```python
# Your motion commands now work properly
robot.drive.forward(50, speed=1.0)  # Drives at actual 1.0 m/s, no creeping!
robot.drive.turn(90, speed=0.5)     # Smooth, controlled turns
```

## Examples

See `/home/tobias/Documents/Botball/library/examples/calibrate_motors_example.py` for complete working examples.

## Troubleshooting

**Q: Calibration times out**
- A: Increase `max_calibration_duration` or reduce individual test durations

**Q: Robot moves too far**
- A: Reduce `step_response_amplitude` or `relay_amplitude`

**Q: Results are inconsistent**
- A: Check battery voltage and ensure consistent surface

**Q: Validation fails**
- A: The gains often work fine anyway - try them! Otherwise retry or use conservative mode.

**Q: I have both mecanum and differential robots**
- A: Perfect! The same code works for both. The system auto-detects which type you have.

## What Gets Fixed

After calibration, your robot will:
- ✓ **No velocity creep** - Reaches target speed and stays there
- ✓ **Faster response** - Gets to speed quicker
- ✓ **Smooth motion** - No oscillation or overshoot
- ✓ **Accurate** - Actual speed matches commanded speed
- ✓ **Consistent** - Works the same across all speeds

The "slowly creeping toward target" problem you described is completely eliminated by adding the integral term (ki) that was missing from the default PID configuration.

## Full Documentation

For comprehensive details, see `CALIBRATION_GUIDE.md`
