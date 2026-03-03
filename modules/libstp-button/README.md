# libstp-button

`libstp-button` wraps the robot's physical start / confirm button behind a small singleton interface.

This module is intentionally narrow. It exists so higher-level code can depend on a single shared button source without passing a `DigitalSensor` instance through every API.

## Public API

Primary header:

- `include/button/button.hpp`

Python bindings:

- native module `libstp.button`

Main operations:

- `Button::instance()`
- `Button::setDigital(int)`
- `Button::setDigital(std::unique_ptr<DigitalSensor>)`
- `Button::isPressed()`
- `Button::waitForButtonPress()`

The Python module exposes the same behavior as module-level helpers:

- `set_digital(...)`
- `is_pressed()`
- `wait_for_button_press()`

## Usage Model

The button is configured once during robot startup, typically from `GenericRobot.__init__()`:

1. Create or resolve the real `DigitalSensor`.
2. Register it with the singleton.
3. Read the button anywhere else through `libstp.button`.

This pattern keeps missions, UI flows, and pre-start gates independent from the concrete button implementation.

## Extension Points

- Keep the singleton focused on one shared digital input.
- If you need richer input behavior such as debouncing, long-press detection, or edge callbacks, document the semantics clearly and preserve the existing blocking API.
- Use the injectable `setDigital(std::unique_ptr<...>)` overload for tests instead of introducing global hooks elsewhere.
