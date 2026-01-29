#!/usr/bin/env python3
"""
Example: Automatic Motor Calibration

This script demonstrates how to use the automatic PID tuning system
to calibrate your robot's motors for optimal velocity tracking.

Works for both differential and mecanum drive robots!
"""

from libstp.step import Robot
from libstp.calibration import CalibrationConfig

def main():
    print("=== Automatic Motor Calibration Example ===\n")

    # Initialize robot
    robot = Robot()

    # Option 1: Simple calibration with default settings (recommended for first try)
    print("Option 1: Running calibration with default settings (conservative mode)...")
    print("- Uses step response tuning (safe, requires ~30cm space)")
    print("- Takes approximately 40-50 seconds for differential drive")
    print("- Make sure robot has at least 30cm of clear space ahead!\n")

    input("Press Enter to start calibration...")

    results = robot.drive.calibrate_motors()

    print("\n=== Calibration Results ===")
    for i, result in enumerate(results):
        motor_name = "Left" if i == 0 else "Right"
        print(f"\n{motor_name} Motor:")
        print(f"  Success: {result.success}")

        if result.success:
            print(f"  PID Gains:")
            print(f"    kp = {result.pid.kp:.3f}")
            print(f"    ki = {result.pid.ki:.3f}")
            print(f"    kd = {result.pid.kd:.3f}")
            print(f"  Feedforward:")
            print(f"    kS (static friction) = {result.ff.kS:.2f}%")
            print(f"    kV (velocity const)  = {result.ff.kV:.3f}")
            print(f"    kA (accel const)     = {result.ff.kA:.3f}")
            print(f"  Duration: {result.duration_seconds:.1f}s")

            # Show validation results
            if result.metrics.validation_passed:
                print(f"  Validation: PASSED ✓")
                print(f"    Mean error: {result.metrics.validation_mean_error*100:.1f}%")
                print(f"    Max error:  {result.metrics.validation_max_error*100:.1f}%")
            else:
                print(f"  Validation: Did not pass (but gains may still work)")
        else:
            print(f"  Error: {result.error_message}")

    # Now test the calibrated robot
    if all(r.success for r in results):
        print("\n=== Testing Calibrated Motors ===")
        print("The robot should now move smoothly without creeping!")
        print("Try driving forward to see the difference.")

        # Example: Drive forward 50cm at speed 1.0
        # robot.drive.forward(50, speed=1.0)  # Uncomment to test

    print("\n✓ Calibration complete!")
    print("The new PID gains are now active and will persist until next calibration.")


def example_aggressive_tuning():
    """
    Option 2: Aggressive tuning for higher performance

    Use this if you want faster, more responsive control.
    Requires more space (~50cm) and takes longer.
    - Differential: ~60-80 seconds (2 motors)
    - Mecanum: ~120-160 seconds (4 motors)
    """
    print("\n=== Option 2: Aggressive Tuning (Relay Feedback) ===")

    robot = Robot()

    # Create custom configuration
    config = CalibrationConfig()
    config.use_relay_feedback = True  # Enable aggressive mode
    config.relay_amplitude = 60.0      # Increase relay amplitude for stronger oscillations
    config.max_relay_duration = 12.0   # Allow more time for oscillations

    print("Configuration:")
    print(f"  Mode: Relay Feedback (aggressive)")
    print(f"  Space required: ~50cm")
    print("\nThis will produce more responsive control but may be less stable.")
    print("Make sure robot has at least 50cm of clear space!\n")

    input("Press Enter to start aggressive calibration...")

    results = robot.drive.calibrate_motors(config)

    print("\n=== Results ===")
    motor_names = ["FL", "FR", "BL", "BR"] if len(results) == 4 else ["Left", "Right"]
    for i, result in enumerate(results):
        if result.success:
            print(f"{motor_names[i]}: kp={result.pid.kp:.3f}, ki={result.pid.ki:.3f}, kd={result.pid.kd:.3f}")
            print(f"    Ku={result.metrics.ultimate_gain_ku:.2f}, Tu={result.metrics.ultimate_period_tu:.3f}s")


def example_custom_parameters():
    """
    Option 3: Fine-tune calibration parameters

    For advanced users who want to customize the calibration process.
    """
    robot = Robot()

    config = CalibrationConfig()

    # Customize step response test
    config.step_response_amplitude = 40.0  # Higher amplitude (default: 30%)
    config.step_response_duration = 4.0    # Longer test (default: 3.0s)

    # Customize safety limits
    config.max_calibration_duration = 150.0  # Allow more time (default: 120s)
    config.max_test_distance_m = 0.8        # Allow more distance (default: 0.6m)

    # Customize velocity tests
    config.velocity_test_commands = [15.0, 30.0, 45.0, 60.0, 75.0]  # More test points
    config.velocity_test_duration = 2.5  # Longer per test

    # Optional: enforce parameter range validation (off by default)
    config.validate_parameter_ranges = True

    print("Running calibration with custom parameters...")
    results = robot.drive.calibrate_motors(config)

    # Process results...


if __name__ == "__main__":
    # Run the basic calibration
    main()

    # Uncomment to try aggressive tuning:
    # example_aggressive_tuning()

    # Uncomment to try custom parameters:
    # example_custom_parameters()
