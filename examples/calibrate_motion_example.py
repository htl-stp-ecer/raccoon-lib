#!/usr/bin/env python3
"""
Example script demonstrating automatic motion calibration for robot motion systems.

This script calibrates the PID gains for turn, drive straight, and strafe motions
to eliminate overshoot and achieve optimal settling time.

Usage:
    python3 calibrate_motion_example.py
"""

import sys
import time
from pathlib import Path

# Import libstp modules
import libstp.hal as hal
import libstp.foundation as foundation
import libstp.drive as drive
import libstp.odometry as odometry
import libstp.motion as motion
import libstp.calibration as calibration


def main():
    print("=" * 60)
    print("Motion Calibration System - Automatic PID Tuning")
    print("=" * 60)
    print()

    # Initialize HAL
    print("[1/6] Initializing hardware...")
    hal_instance = hal.Hal()

    # Initialize motors (adjust motor indices for your robot)
    left_motor = foundation.Motor(hal_instance, 0)   # Motor 0
    right_motor = foundation.Motor(hal_instance, 1)  # Motor 1

    # Create drive system (differential or mecanum)
    # For differential drive:
    drive_system = drive.DifferentialDrive(left_motor, right_motor)

    # For mecanum drive (4 motors):
    # front_left = foundation.Motor(hal_instance, 0)
    # front_right = foundation.Motor(hal_instance, 1)
    # back_left = foundation.Motor(hal_instance, 2)
    # back_right = foundation.Motor(hal_instance, 3)
    # drive_system = drive.MecanumDrive(front_left, front_right, back_left, back_right)

    print("   ✓ Drive system initialized")

    # Initialize odometry
    print("[2/6] Initializing odometry...")
    odom = odometry.Odometry(drive_system)
    print("   ✓ Odometry initialized")

    # Configure calibration parameters
    print("[3/6] Configuring calibration parameters...")
    config = calibration.MotionCalibrationConfig()

    # Tuning parameters (adjust based on your robot's characteristics)
    config.initial_kp = 2.0              # Starting proportional gain estimate
    config.max_iterations = 10            # Maximum tuning iterations per controller
    config.target_settling_time = 1.5     # Target settling time (seconds)
    config.max_overshoot = 0.15          # Maximum overshoot (15%)
    config.target_steady_state_error = 0.02  # Target steady-state error (2%)
    config.max_test_duration = 900.0     # Total timeout: 15 minutes
    config.max_single_test_time = 10.0   # Timeout for individual tests

    print(f"   - Initial Kp: {config.initial_kp}")
    print(f"   - Max iterations: {config.max_iterations}")
    print(f"   - Target settling time: {config.target_settling_time}s")
    print(f"   - Max overshoot: {config.max_overshoot * 100}%")
    print(f"   ✓ Configuration complete")

    # Create motion calibrator
    print("[4/6] Creating motion calibrator...")
    calibrator = calibration.MotionCalibrator(drive_system, odom, config)
    print("   ✓ Calibrator ready")

    # Run calibration
    print()
    print("[5/6] Running automatic calibration...")
    print("   This may take up to 15 minutes depending on tuning convergence.")
    print("   The robot will execute test motions to tune PID gains.")
    print()

    start_time = time.time()
    result = calibrator.calibrate()
    duration = time.time() - start_time

    print()
    print("[6/6] Calibration complete!")
    print()

    # Display results
    print("=" * 60)
    print("CALIBRATION RESULTS")
    print("=" * 60)

    if result.success:
        print(f"✓ SUCCESS - Calibration completed in {duration:.1f} seconds")
        print()

        # Display calibrated gains for each motion type
        for gain_set in result.gains:
            motion_name = {
                calibration.MotionType.TURN: "Turn Motion",
                calibration.MotionType.DRIVE_STRAIGHT: "Drive Straight Motion",
                calibration.MotionType.STRAFE: "Strafe Motion"
            }.get(gain_set.motion_type, "Unknown")

            print(f"{motion_name} ({gain_set.controller_name}):")
            print(f"  Kp: {gain_set.kp:.4f}")
            print(f"  Ki: {gain_set.ki:.4f}")
            print(f"  Kd: {gain_set.kd:.4f}")
            print()

        # Apply gains to motion systems
        print("Applying calibrated gains to motion systems...")

        # Example: Apply turn gains
        turn_gains = [g for g in result.gains if g.motion_type == calibration.MotionType.TURN]
        if turn_gains:
            turn_config = motion.TurnConfig()
            turn_config.angle_kp = turn_gains[0].kp
            turn_config.angle_ki = turn_gains[0].ki
            turn_config.angle_kd = turn_gains[0].kd
            print(f"  ✓ Turn motion configured with Kp={turn_config.angle_kp:.4f}")

        # Example: Apply drive straight gains
        drive_gains = [g for g in result.gains if g.motion_type == calibration.MotionType.DRIVE_STRAIGHT]
        if drive_gains:
            drive_config = motion.DriveStraightConfig()
            # Find distance, heading, and lateral controllers
            for g in drive_gains:
                if "distance" in g.controller_name.lower():
                    drive_config.distance_kp = g.kp
                    drive_config.distance_ki = g.ki
                    drive_config.distance_kd = g.kd
                elif "heading" in g.controller_name.lower():
                    drive_config.heading_kp = g.kp
                    drive_config.heading_ki = g.ki
                    drive_config.heading_kd = g.kd
                elif "lateral" in g.controller_name.lower():
                    drive_config.lateral_kp = g.kp
                    drive_config.lateral_ki = g.ki
                    drive_config.lateral_kd = g.kd
            print(f"  ✓ Drive straight motion configured")

        print()
        print("Calibration gains are now ready to use!")
        print("You can save these values to a configuration file for later use.")

    else:
        print(f"✗ FAILED - {result.error_message}")
        print(f"   Duration: {duration:.1f} seconds")
        print()
        print("Troubleshooting tips:")
        print("  - Ensure the robot has enough space to move (at least 1m)")
        print("  - Check that odometry is working correctly")
        print("  - Verify motor directions are correct")
        print("  - Try adjusting initial_kp or max_iterations")

    print("=" * 60)

    return 0 if result.success else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
