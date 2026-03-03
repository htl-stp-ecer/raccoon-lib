# libstp-robot

`libstp-robot` defines the Python-side robot abstraction used by missions and step workflows.

This module does not provide hardware drivers itself. Instead, it ties together definitions from the HAL and control modules into a single object that missions can depend on.

## Public API

Python package:

- `python/libstp/robot/`

Main exports:

- `GenericRobot`
- `RobotDefinitionsProtocol`
- `RobotGeometry`
- `SensorPosition`
- `WheelPosition`
- `RobotService`

## Responsibilities

- Define the minimum contract a concrete robot implementation must satisfy.
- Provide mission startup and shutdown orchestration.
- Handle the pre-start gate: wait-for-light or button press depending on configuration.
- Cache long-lived service objects.
- Expose geometry helpers for sensors and wheels.

## Implementing A Robot

A concrete robot subclass is expected to provide:

- `defs`
- `drive`
- `odometry`
- `shutdown_in`

Optional overrides:

- `motion_pid_config`
- `missions`
- `setup_mission`
- `shutdown_mission`

The `defs` object should hold the concrete hardware definitions used across the robot, including at least:

- `analog_sensors`
- `button`
- `wait_for_light_sensor` when wait-for-light is supported

## Mission Lifecycle

`GenericRobot.start()` and `start_async()` run the same internal sequence:

1. Preload missions where possible to catch construction errors early.
2. Run the optional setup mission.
3. Run the pre-start gate:
   - in dev mode (`LIBSTP_DEV_MODE=1`), wait for the button
   - otherwise, prefer wait-for-light and fall back to button if no sensor is configured
4. Start synchronizer recording and execute the main mission list under `shutdown_in`.
5. Run the optional shutdown mission even after a timeout.

This module is where contributors should document any changes to startup semantics, not inside individual missions.

## Services

`RobotService` is the recommended place for reusable robot-specific business logic that is too stateful or too domain-specific for generic steps. Services are created lazily through `robot.get_service(ServiceType)` and cached for the life of the robot instance.

## Geometry

`RobotGeometry` is a mixin rather than a complete robot implementation. It expects subclasses to provide:

- `width_cm`
- `length_cm`
- `rotation_center_forward_cm`
- `rotation_center_strafe_cm`
- `_sensor_positions`
- `_wheel_positions`

Those values are then used to compute distances and angles that higher-level motion or sensing code can reuse.
