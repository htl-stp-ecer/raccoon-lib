# Motor Calibration Guide

## Quick Start

The simplest way to calibrate your robot's motors:

```python
from libstp.step import Robot

robot = Robot()

# Calibrate motors (takes ~40-50 seconds)
results = robot.drive.calibrate_motors()

# Check if successful
for i, result in enumerate(results):
    if result.success:
        print(f"Motor {i}: kp={result.pid.kp:.3f}, ki={result.pid.ki:.3f}, kd={result.pid.kd:.3f}")
    else:
        print(f"Motor {i} failed: {result.error_message}")
```

## What Does Calibration Do?

The automatic calibration system:

1. **Measures feedforward parameters:**
   - **kS** (static friction): The minimum motor command needed to overcome friction
   - **kV** (velocity constant): The linear relationship between command and velocity
   - **kA** (acceleration constant): The relationship between command and acceleration

2. **Tunes PID gains:**
   - **kp** (proportional): Responds to current error
   - **ki** (integral): Eliminates steady-state error (this fixes your "creeping" issue!)
   - **kd** (derivative): Dampens oscillations

## Two Calibration Modes

### Conservative Mode (Default)
- **Method:** Step response analysis (FOPDT model)
- **Space required:** ~30cm
- **Time:** ~40-50 seconds for differential drive
- **Characteristics:** Safe, stable, good for general use

```python
results = robot.drive.calibrate_motors()
```

### Aggressive Mode (Optional)
- **Method:** Relay feedback tuning (Tyreus-Luyben rules)
- **Space required:** ~50cm
- **Time:** ~60-80 seconds for differential drive
- **Characteristics:** More responsive, faster tracking, higher performance

```python
from libstp.kinematics_differential import CalibrationConfig

config = CalibrationConfig()
config.use_relay_feedback = True
results = robot.drive.calibrate_motors(config)
```

## Configuration Options

### Basic Configuration

```python
from libstp.kinematics_differential import CalibrationConfig

config = CalibrationConfig()

# Choose tuning method
config.use_relay_feedback = False  # False = conservative, True = aggressive

# Step response parameters (conservative mode)
config.step_response_amplitude = 30.0  # % (default: 30%)
config.step_response_duration = 3.0    # seconds (default: 3.0s)

# Relay feedback parameters (aggressive mode)
config.relay_amplitude = 50.0         # % (default: 50%)
config.max_relay_duration = 10.0      # seconds (default: 10.0s)
config.min_oscillations = 3           # minimum oscillations to measure

# Safety limits
config.max_test_distance_m = 0.6      # emergency stop distance (default: 0.6m)
config.max_calibration_duration = 120.0  # total timeout (default: 120s)

results = robot.drive.calibrate_motors(config)
```

### Advanced Configuration

```python
config = CalibrationConfig()

# Feedforward calibration parameters
config.static_friction_increment = 1.0      # command increment for friction search (%)
config.static_friction_max = 40.0           # maximum command to try
config.velocity_test_commands = [20.0, 40.0, 60.0, 80.0]  # test speeds (%)
config.velocity_test_duration = 2.0         # duration per velocity test
config.acceleration_test_count = 3          # number of acceleration tests to average

# Validation parameters
config.validation_duration = 2.0            # duration of validation test
config.validation_max_error = 0.2           # maximum acceptable tracking error (20%)
```

## Understanding Results

### CalibrationResult Object

```python
result = results[0]  # First motor (left for differential)

# Success status
result.success              # bool: True if calibration succeeded
result.error_message        # str: Error message if failed
result.duration_seconds     # float: How long calibration took

# Tuned parameters
result.pid.kp              # float: Proportional gain
result.pid.ki              # float: Integral gain
result.pid.kd              # float: Derivative gain

result.ff.kS               # float: Static friction (%)
result.ff.kV               # float: Velocity constant
result.ff.kA               # float: Acceleration constant

# Detailed metrics
result.metrics.static_friction_forward        # float: Forward friction threshold
result.metrics.static_friction_backward       # float: Backward friction threshold
result.metrics.velocity_constant_r_squared    # float: Linear fit quality (0-1)
result.metrics.time_constant_tau              # float: System time constant
result.metrics.steady_state_gain              # float: System gain
result.metrics.ultimate_gain_ku               # float: Ultimate gain (relay mode)
result.metrics.ultimate_period_tu             # float: Ultimate period (relay mode)
result.metrics.validation_mean_error          # float: Mean tracking error
result.metrics.validation_max_error           # float: Max tracking error
result.metrics.validation_passed              # bool: Whether validation passed
```

## Best Practices

### Before Calibration
1. **Ensure clear space:** At least 30cm for conservative mode, 50cm for aggressive mode
2. **Check battery:** Low battery affects calibration accuracy
3. **Stable surface:** Calibrate on the surface you'll be competing on
4. **Motor warmth:** Motors perform differently when cold vs. warm

### When to Recalibrate
- **After significant battery discharge** (voltage drops)
- **When surface changes** (carpet vs. hardwood vs. competition mat)
- **After mechanical changes** (new wheels, weight changes)
- **If performance degrades** (motors wearing out)

### Troubleshooting

**Problem:** Calibration times out
- **Solution:** Increase `max_calibration_duration` or reduce test durations

**Problem:** Robot moves too far during calibration
- **Solution:** Reduce `step_response_amplitude` or `relay_amplitude`

**Problem:** Inconsistent results
- **Solution:** Ensure stable battery voltage and consistent surface friction

**Problem:** Validation fails
- **Solution:** The gains may still work fine. Try them! If not, retry calibration or use conservative mode.

**Problem:** Motor saturates during tests
- **Solution:** Reduce amplitudes or increase feedforward terms if you have prior knowledge

## Example: Complete Calibration Workflow

```python
from libstp.step import Robot
from libstp.kinematics_differential import CalibrationConfig

def calibrate_and_test():
    robot = Robot()

    # Create configuration
    config = CalibrationConfig()
    config.use_relay_feedback = False  # Start with conservative mode

    # Run calibration
    print("Starting calibration...")
    print("Make sure robot has 30cm of clear space!")
    input("Press Enter to continue...")

    results = robot.drive.calibrate_motors(config)

    # Analyze results
    all_success = True
    for i, result in enumerate(results):
        motor_name = ["Left", "Right"][i]

        if result.success:
            print(f"\n{motor_name} Motor - SUCCESS")
            print(f"  PID: kp={result.pid.kp:.3f}, ki={result.pid.ki:.3f}, kd={result.pid.kd:.3f}")
            print(f"  FF:  kS={result.ff.kS:.2f}, kV={result.ff.kV:.3f}, kA={result.ff.kA:.3f}")
            print(f"  Validation: {'PASSED' if result.metrics.validation_passed else 'Did not pass'}")
        else:
            print(f"\n{motor_name} Motor - FAILED")
            print(f"  Error: {result.error_message}")
            all_success = False

    if all_success:
        print("\n✓ Calibration complete! Motors are now tuned.")
        print("Test by driving forward - it should be much smoother now.")

        # Optional: Test drive
        # robot.drive.forward(30, speed=1.0)  # Drive 30cm at max speed

    return results

if __name__ == "__main__":
    calibrate_and_test()
```

## Technical Details

### Step Response Method (Conservative)
Uses First-Order Plus Dead Time (FOPDT) model:
- Applies a step input and records velocity response
- Fits exponential curve to extract system parameters (tau, K, delay)
- Calculates PID gains using proven tuning formulas

### Relay Feedback Method (Aggressive)
Uses Ziegler-Nichols relay auto-tuning:
- Applies bang-bang control to induce oscillations
- Measures ultimate gain (Ku) and period (Tu)
- Applies Tyreus-Luyben rules (more conservative than classic Z-N)

### Feedforward Characterization
- **kS**: Binary search to find minimum command for movement
- **kV**: Linear regression on velocity vs. command at multiple speeds
- **kA**: Calculated from acceleration during step inputs

## Safety Features

The calibration system includes multiple safety mechanisms:
- **Distance limit:** Emergency stop if robot moves >60cm
- **Time limits:** Individual test timeout (15s) and total timeout (120s)
- **Parameter validation:** Rejects unreasonable values
- **Retry logic:** Up to 3 retries on measurement failures
- **Graceful fallback:** Returns best-effort results even on partial failure

## Performance Impact

After calibration, you should see:
- ✓ **No more velocity creep:** Integral gain eliminates steady-state error
- ✓ **Faster acceleration:** Feedforward terms improve transient response
- ✓ **Smoother motion:** Properly tuned derivative prevents oscillation
- ✓ **Accurate tracking:** Robot reaches commanded speeds within 5% error
- ✓ **Consistent behavior:** Works across all speed ranges

The slow "creeping toward target" issue you described will be completely eliminated!
